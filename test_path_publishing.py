#!/usr/bin/env python3
"""
경로 발행과 보간 테스트 코드
utm_pure_pursuit.py의 경로 생성 및 발행 기능을 테스트합니다.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math
import time
import json
import scipy.interpolate as si
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu
import utm
import re

class PathPublishingTest(Node):
    def __init__(self):
        super().__init__('path_publishing_test')
        
        # 테스트용 waypoint 파일 경로
        self.waypoints_file_path = "/home/jh/ros2_workspace/src/remapped_utmcoordinates.txt"
        
        # waypoint 로드
        self.waypoints = self.load_waypoints_from_file(self.waypoints_file_path)
        
        if not self.waypoints:
            self.get_logger().error("waypoint를 로드할 수 없습니다. 테스트를 종료합니다.")
            return
        
        # 첫 번째 waypoint를 현재 GPS 위치로 설정
        first_waypoint = self.waypoints[0]
        self.current_x = first_waypoint[0]
        self.current_y = first_waypoint[1]
        self.yaw = 0.0
        
        self.get_logger().info(f"테스트용 현재 위치 설정: ({self.current_x:.3f}, {self.current_y:.3f})")
        
        # 구간 설정 (테스트용)
        self.segment_config = [[8, 's', 1], [16, 's', 2], [20, 's', 3], [25, 's', 4], [30, 's', 5], [35, 'c', 1], [43, 'c', 2], [48, 'c', 3], [53, 'r', 1], [58, 'r', 2], [63, 's', 3], [442, 'c', 3]]
        
        # 경로 생성 및 발행
        self.generate_and_publish_path()
        
        # 퍼블리셔 설정
        self.path_publisher = self.create_publisher(PoseArray, 'interpolated_path', 10)
        self.segments_publisher = self.create_publisher(String, 'path_segments', 10)
        
        # 가상 GPS/IMU 데이터 발행
        self.gps_publisher = self.create_publisher(NavSatFix, '/ublox_gps_node/fix', 10)
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)
        
        # 타이머 설정 (1초마다 경로 발행)
        self.timer = self.create_timer(1.0, self.publish_test_data)
        
        self.get_logger().info("🚀 경로 발행 테스트 노드가 시작되었습니다!")
        self.get_logger().info("📡 경로 데이터를 발행 중입니다...")
    
    def load_waypoints_from_file(self, file_path):
        """waypoint 파일에서 좌표 로드"""
        waypoints = []
        try:
            with open(file_path, 'r') as f:
                for line in f:
                    stripped = line.strip()
                    if not stripped or stripped.startswith('#'):
                        continue
                    # 주석 제거 후 숫자만 추출
                    content = stripped.split('#', 1)[0]
                    numbers = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", content)
                    if len(numbers) >= 2:
                        x = float(numbers[0])
                        y = float(numbers[1])
                        waypoints.append((x, y))
            
            if waypoints:
                self.get_logger().info(f"Waypoint 파일 로드 성공: {file_path}")
                self.get_logger().info(f"총 {len(waypoints)}개 좌표를 불러왔습니다.")
            else:
                self.get_logger().warn(f"유효한 좌표를 찾지 못했습니다: {file_path}")
        except Exception as e:
            self.get_logger().error(f"Waypoint 파일 읽기 실패({file_path}): {e}")
        
        return waypoints
    
    def generate_and_publish_path(self):
        """경로 생성 및 발행"""
        if len(self.waypoints) < 2:
            self.get_logger().error("waypoint가 2개 미만입니다. 경로 생성이 불가능합니다.")
            return
        
        self.get_logger().info("🗺️ 테스트용 경로 생성 중...")
        
        # 현재 위치를 시작점으로 하는 전체 경로 생성
        full_path_waypoints = [(self.current_x, self.current_y)]
        full_path_waypoints.extend(self.waypoints)
        
        # 구간 설정을 현재 위치 포함 기준으로 조정 (인덱스 +1)
        adjusted_segment_config = []
        for config_item in self.segment_config:
            if len(config_item) == 3:
                idx, seg_type, version = config_item
                adjusted_segment_config.append([idx + 1, seg_type, version])
            else:
                idx, seg_type = config_item
                adjusted_segment_config.append([idx + 1, seg_type])
        
        # 구간 생성
        self.get_logger().info(f"원본 segment_config: {self.segment_config}")
        self.get_logger().info(f"조정된 segment_config: {adjusted_segment_config}")
        self.get_logger().info(f"전체 waypoint 개수 (현재 위치 포함): {len(full_path_waypoints)}")
        
        waypoint_segments = self.create_manual_segments(full_path_waypoints, adjusted_segment_config)
        
        # 생성된 구간 정보 출력
        self.get_logger().info("생성된 구간 정보:")
        for i, seg in enumerate(waypoint_segments):
            self.get_logger().info(f"  구간 {i+1}: {seg['type']}{seg.get('version', 1)} - waypoint {seg['waypoint_start']}~{seg['waypoint_end']}")
        
        # 구간별 보간으로 경로 생성
        self.path, self.path_segments = self.segment_based_bspline_planning(full_path_waypoints, waypoint_segments)
        
        # 보간 후 구간 정보 출력
        self.get_logger().info("보간 후 구간 정보:")
        for i, seg in enumerate(self.path_segments):
            self.get_logger().info(f"  보간 구간 {i+1}: {seg['type']}{seg.get('version', 1)} - waypoint {seg['waypoint_start']}~{seg['waypoint_end']}, 경로 {seg['start_index']}~{seg['end_index']}")
        
        # 총 거리 계산
        total_distance = 0.0
        for i in range(len(full_path_waypoints) - 1):
            dist = math.sqrt((full_path_waypoints[i+1][0] - full_path_waypoints[i][0])**2 + 
                           (full_path_waypoints[i+1][1] - full_path_waypoints[i][1])**2)
            total_distance += dist
        
        # 구간 통계 계산
        straight_segments = sum(1 for seg in self.path_segments if seg['type'] == 'straight')
        curve_segments = sum(1 for seg in self.path_segments if seg['type'] == 'curve')
        reverse_segments = sum(1 for seg in self.path_segments if seg['type'] == 'reverse')
        
        self.get_logger().info("✅ 테스트용 경로 생성 완료!")
        self.get_logger().info(f"   스무딩된 경로점: {len(self.path)}개")
        self.get_logger().info(f"   총 거리: {total_distance:.2f}m")
        self.get_logger().info(f"   직선 구간: {straight_segments}개")
        self.get_logger().info(f"   곡선 구간: {curve_segments}개")
        self.get_logger().info(f"   후진 구간: {reverse_segments}개")
        
        # 구간별 상세 정보 출력
        for i, seg in enumerate(self.path_segments):
            version = seg.get('version', 1)
            self.get_logger().info(f"   구간 {i+1}: {seg['type']}{version} (waypoint {seg['waypoint_start']}-{seg['waypoint_end']}, "
                                 f"경로 {seg['start_index']}-{seg['end_index']}, 거리: {seg['segment_distance']:.2f}m)")
    
    def create_manual_segments(self, waypoints, segment_config):
        """인덱스별 구간 타입 설정을 기반으로 구간 정보 생성"""
        segments = []
        total_waypoints = len(waypoints)
        
        if not segment_config or total_waypoints < 2:
            segments.append({
                'type': 'straight',
                'version': 1,
                'start_index': 0,
                'end_index': total_waypoints - 1,
                'waypoint_start': 0,
                'waypoint_end': total_waypoints - 1
            })
            return segments
        
        # 구간 타입 매핑
        type_mapping = {
            's': 'straight',
            'c': 'curve', 
            'r': 'reverse'
        }
        
        # 설정을 인덱스 순으로 정렬
        sorted_config = sorted(segment_config, key=lambda x: x[0])
        
        # 유효성 검사
        valid_config = []
        for config_item in sorted_config:
            if len(config_item) == 3:
                idx, seg_type, version = config_item
            else:
                idx, seg_type = config_item
                version = 1
            
            if 0 <= idx <= total_waypoints and seg_type in type_mapping:
                valid_config.append((idx, seg_type, version))
            else:
                self.get_logger().warn(f"경고: 잘못된 구간 설정 무시 - 인덱스: {idx}, 타입: {seg_type}, 버전: {version}")
        
        if not valid_config:
            segments.append({
                'type': 'straight',
                'version': 1,
                'start_index': 0,
                'end_index': total_waypoints - 1,
                'waypoint_start': 0,
                'waypoint_end': total_waypoints - 1
            })
            return segments
        
        # 구간 생성
        start_idx = 0
        for i, (end_idx, seg_type, version) in enumerate(valid_config):
            segments.append({
                'type': type_mapping[seg_type],
                'version': version,
                'start_index': start_idx,
                'end_index': end_idx,
                'waypoint_start': start_idx,
                'waypoint_end': end_idx
            })
            start_idx = end_idx + 1
        
        # 마지막 구간이 전체 waypoint를 포함하지 않는 경우, 마지막 구간을 확장
        if segments and segments[-1]['end_index'] < total_waypoints - 1:
            segments[-1]['end_index'] = total_waypoints - 1
            segments[-1]['waypoint_end'] = total_waypoints - 1
        
        return segments
    
    def segment_based_bspline_planning(self, waypoints, segments):
        """구간별 B-Spline 경로 스무딩 (구간별 개별 보간)"""
        try:
            if len(waypoints) < 2:
                return [], []
            
            total_path = []
            path_segments = []
            
            # 보간 밀도 설정 (원본 코드와 동일하게)
            interpolation_density_s1 = 2
            interpolation_density_s2 = 2
            interpolation_density_s3 = 2
            interpolation_density_s4 = 2
            interpolation_density_s5 = 2
            interpolation_density_c1 = 5
            interpolation_density_c2 = 5
            interpolation_density_c3 = 5
            interpolation_density_r1 = 5
            interpolation_density_r2 = 5
            
            # 구간별로 개별적으로 B-Spline 보간 (구간 간 연결점 포함)
            for i, segment in enumerate(segments):
                start_wp = segment['waypoint_start']
                end_wp = segment['waypoint_end']
                segment_type = segment['type']
                version = segment.get('version', 1)
                
                # 구간별 waypoint 추출 (구간 간 연결점 포함하되 겹치지 않도록)
                if i < len(segments) - 1:
                    # 마지막 구간이 아니면 다음 구간의 시작점까지 포함
                    next_segment = segments[i + 1]
                    segment_waypoints = waypoints[start_wp:next_segment['waypoint_start'] + 1]
                else:
                    # 마지막 구간이면 끝점까지 포함
                    segment_waypoints = waypoints[start_wp:end_wp + 1]
                
                if len(segment_waypoints) < 2:
                    continue
                
                # 구간별 거리 계산
                segment_distance = 0.0
                for j in range(len(segment_waypoints) - 1):
                    dist = math.sqrt((segment_waypoints[j+1][0] - segment_waypoints[j][0])**2 + 
                                   (segment_waypoints[j+1][1] - segment_waypoints[j][1])**2)
                    segment_distance += dist
                
                # 구간별 보간 밀도 결정
                if segment_type == 'straight':
                    densities = [interpolation_density_s1, interpolation_density_s2, interpolation_density_s3, 
                               interpolation_density_s4, interpolation_density_s5]
                    interpolation_density = densities[min(version - 1, 4)]
                elif segment_type == 'curve':
                    densities = [interpolation_density_c1, interpolation_density_c2, interpolation_density_c3]
                    interpolation_density = densities[min(version - 1, 2)]
                else:  # reverse
                    densities = [interpolation_density_r1, interpolation_density_r2]
                    interpolation_density = densities[min(version - 1, 1)]
                
                # 보간 점 수 계산
                interpolation_points = max(2, int(segment_distance * interpolation_density))
                
                # B-Spline 스무딩
                if len(segment_waypoints) >= 3:
                    x = np.array([wp[0] for wp in segment_waypoints])
                    y = np.array([wp[1] for wp in segment_waypoints])
                    
                    N = min(2, len(x) - 1)  # 차수는 점의 개수에 따라 조정
                    t = range(len(x))
                    x_tup = si.splrep(t, x, k=N)
                    y_tup = si.splrep(t, y, k=N)

                    x_list = list(x_tup)
                    xl = x.tolist()
                    x_list[1] = xl + [0.0] * (N + 1)

                    y_list = list(y_tup)
                    yl = y.tolist()
                    y_list[1] = yl + [0.0] * (N + 1)

                    ipl_t = np.linspace(0.0, len(x) - 1, interpolation_points)
                    rx = si.splev(ipl_t, x_list)
                    ry = si.splev(ipl_t, y_list)
                    segment_path = [(rx[i], ry[i]) for i in range(len(rx))]
                else:
                    # 점이 2개뿐이면 선형 보간
                    segment_path = segment_waypoints
                
                # 전체 경로에 추가
                path_start_idx = len(total_path)
                total_path.extend(segment_path)
                path_end_idx = len(total_path) - 1
                
                # 경로 구간 정보 저장 (원래 구간 정보 유지)
                path_segments.append({
                    'type': segment_type,
                    'version': version,
                    'start_index': path_start_idx,
                    'end_index': path_end_idx,
                    'waypoint_start': start_wp,
                    'waypoint_end': end_wp,  # 원래 구간 끝점 유지
                    'interpolation_density': interpolation_density,
                    'segment_distance': segment_distance
                })
            
            return total_path, path_segments
        except Exception as e:
            self.get_logger().error(f"구간별 보간 오류: {e}")
            return waypoints, []
    
    def publish_test_data(self):
        """테스트 데이터 발행"""
        # 보간된 경로 발행
        if hasattr(self, 'path') and self.path:
            self.publish_interpolated_path()
        
        # 구간 정보 발행
        if hasattr(self, 'path_segments') and self.path_segments:
            self.publish_path_segments()
        
        # 가상 GPS 데이터 발행
        self.publish_virtual_gps()
        
        # 가상 IMU 데이터 발행
        self.publish_virtual_imu()
    
    def publish_interpolated_path(self):
        """보간된 경로를 PoseArray 메시지로 발행"""
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "utm"
        
        for point in self.path:
            pose = Pose()
            pose.position.x = float(point[0])
            pose.position.y = float(point[1])
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        
        self.path_publisher.publish(pose_array)
        self.get_logger().info(f"📡 보간된 경로 발행: {len(self.path)}개 점")
    
    def publish_path_segments(self):
        """구간 정보를 JSON 형태로 발행"""
        import json
        
        # 총 경로 거리 계산
        full_path_waypoints = [(self.current_x, self.current_y)]
        full_path_waypoints.extend(self.waypoints)
        
        total_distance = 0.0
        for i in range(len(full_path_waypoints) - 1):
            dist = math.sqrt((full_path_waypoints[i+1][0] - full_path_waypoints[i][0])**2 + 
                           (full_path_waypoints[i+1][1] - full_path_waypoints[i][1])**2)
            total_distance += dist
        
        segments_data = {
            'segments': self.path_segments,
            'path_length': total_distance,
            'total_waypoints': len(full_path_waypoints),
            'total_path_points': len(self.path)
        }
        
        msg = String()
        msg.data = json.dumps(segments_data)
        self.segments_publisher.publish(msg)
        self.get_logger().info(f"📡 구간 정보 발행: {len(self.path_segments)}개 구간")
    
    def publish_virtual_gps(self):
        """가상 GPS 데이터 발행"""
        # UTM 좌표를 위도/경도로 변환 (대략적인 변환)
        # 실제로는 정확한 UTM zone 정보가 필요하지만, 테스트용으로 간단히 처리
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps"
        
        # UTM을 위도/경도로 변환 (대략적)
        # 실제 환경에서는 정확한 zone 정보를 사용해야 함
        msg.latitude = 37.5665 + (self.current_y - 500000) / 111000  # 대략적 변환
        msg.longitude = 126.9780 + (self.current_x - 200000) / 111000  # 대략적 변환
        msg.altitude = 0.0
        
        # GPS 상태 설정
        msg.status.status = 0  # STATUS_FIX
        msg.status.service = 1  # GPS
        
        # 위치 정확도 설정
        msg.position_covariance_type = 1  # COVARIANCE_TYPE_APPROXIMATED
        msg.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        self.gps_publisher.publish(msg)
    
    def publish_virtual_imu(self):
        """가상 IMU 데이터 발행"""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu"
        
        # 쿼터니언 설정 (yaw = 0도)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        
        # 각속도 설정 (정지 상태)
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        
        # 선형 가속도 설정 (정지 상태)
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81  # 중력 가속도
        
        # 공분산 설정
        msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        
        self.imu_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    # 테스트 노드 생성
    test_node = PathPublishingTest()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info("테스트 노드 종료 중...")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
