import rclpy
from rclpy.node import Node
import numpy as np
import math
# import scipy.interpolate as si  # 보간 사용하지 않으므로 주석 처리
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist, PoseArray, Pose
from std_msgs.msg import String
import time
import matplotlib.pyplot as plt
from matplotlib import patches, transforms
import utm
import os
import re

# Pure Pursuit 파라미터 (구간별 적응적 제어)
# 전방주시거리 설정 실시간 그래프로 보여주므로, 아두이노 IDE의 툴바에서 통신 속도를 동일하게 설정한 후 각 
lookahead_distance_straight = 1.5  # 직선 구간 전방주시거리 (미터)
lookahead_distance_curve = 0.8    # 곡선 구간 전방주시거리 (미터)
current_lookahead_distance = 0.2   # 현재 전방주시거리 (초기값)

# 속도 설정
speed_straight = 70.0 #직선 구간 속도 (m/s)
speed_curve = 60.0   # 곡선 구간 속도 (m/s)
current_speed = 0.0   # 현재 속도 (초기값)


# 수동 구간 경계 인덱스 설정 (waypoint 인덱스 기준)
segment_boundary_indices = [3, 9, 11, 18, 19, 27, 30, 36]  # 이 배열을 수동으로 설정

# 구간 전환 거리 임계값
segment_transition_threshold = 0.5  # 전방주시거리에 추가할 거리 (m)

def euler_from_quaternion(x, y, z, w):
    """쿼터니언에서 yaw 각도 추출 (control.py와 동일)"""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def create_manual_segments(waypoints, boundary_indices):
    """수동으로 지정된 경계 인덱스를 기반으로 구간 정보 생성"""
    segments = []
    total_waypoints = len(waypoints)
    
    if not boundary_indices or total_waypoints < 2:
        # 경계가 없으면 전체를 직선으로 처리
        segments.append({
            'type': 'straight',
            'start_index': 0,
            'end_index': total_waypoints - 1,
            'waypoint_start': 0,
            'waypoint_end': total_waypoints - 1
        })
        return segments
    
    # 경계 인덱스 정렬 및 유효성 검사
    sorted_boundaries = sorted([idx for idx in boundary_indices if 0 <= idx < total_waypoints])
    
    current_type = 'straight'  # 항상 직선으로 시작
    start_idx = 0
    
    for boundary in sorted_boundaries:
        # 현재 구간 저장
        segments.append({
            'type': current_type,
            'start_index': start_idx,
            'end_index': boundary,
            'waypoint_start': start_idx,
            'waypoint_end': boundary
        })
        
        # 다음 구간으로 전환
        start_idx = boundary + 1
        current_type = 'curve' if current_type == 'straight' else 'straight'
    
    # 마지막 구간 처리
    if start_idx < total_waypoints:
        segments.append({
            'type': current_type,
            'start_index': start_idx,
            'end_index': total_waypoints - 1,
            'waypoint_start': start_idx,
            'waypoint_end': total_waypoints - 1
        })
    
    return segments

def create_path_from_waypoints(waypoints, segments):
    """waypoint를 그대로 path로 사용"""
    try:
        if len(waypoints) < 2:
            return [], []
        
        # waypoint를 그대로 path로 사용
        total_path = waypoints.copy()
        path_segments = []
        
        for segment in segments:
            start_wp = segment['waypoint_start']
            end_wp = segment['waypoint_end']
            segment_type = segment['type']
            
            # 구간별 거리 계산
            segment_distance = 0.0
            for i in range(start_wp, end_wp):
                if i + 1 < len(waypoints):
                    dist = math.sqrt((waypoints[i+1][0] - waypoints[i][0])**2 + 
                                   (waypoints[i+1][1] - waypoints[i][1])**2)
                    segment_distance += dist
            
            # 경로 구간 정보 저장 (waypoint 인덱스와 동일)
            path_segments.append({
                'type': segment_type,
                'start_index': start_wp,
                'end_index': end_wp,
                'waypoint_start': start_wp,
                'waypoint_end': end_wp,
                'segment_distance': segment_distance
            })
        
        return total_path, path_segments
    except Exception as e:
        print(f"경로 생성 오류: {e}")
        return waypoints, []

def pure_pursuit(current_x, current_y, current_heading, path, index, current_segment_info=None):
    """구간별 적응적 Pure Pursuit 알고리즘"""
    global current_lookahead_distance, current_speed
    closest_point = None
    v = current_speed
    
    if current_segment_info is None:
        # 기본 전방주시거리 사용
        lookahead = current_lookahead_distance
    else:
        segment_type = current_segment_info['type']
        segment_end_index = current_segment_info['end_index']
        segment_end_point = path[segment_end_index]
        
        if segment_type == 'straight':
            # 직선 구간
            if index >= segment_end_index:
                # 구간 끝에 도달 - 끝점을 목표로
                closest_point = segment_end_point
                index = segment_end_index
            else:
                # 구간 내 - 직선 전방주시거리 사용
                lookahead = lookahead_distance_straight
                
                # 전방주시거리보다 먼 경로점 찾기
                for i in range(index, min(segment_end_index + 1, len(path))):
                    x = path[i][0]
                    y = path[i][1]
                    distance = math.hypot(current_x - x, current_y - y)
                    if lookahead < distance:
                        closest_point = (x, y)
                        index = i
                        break
                else:
                    # 전방주시거리 내에 구간 끝점이 있으면 끝점을 목표로
                    closest_point = segment_end_point
                    index = segment_end_index
        
        else:  # curve
            # 곡선 구간
            if index >= segment_end_index:
                # 구간 끝에 도달 - 끝점을 목표로
                closest_point = segment_end_point
                index = segment_end_index
            else:
                # 구간 내 - 곡선 전방주시거리 사용
                lookahead = lookahead_distance_curve
                
                # 전방주시거리보다 먼 경로점 찾기 (구간 끝까지만)
                for i in range(index, min(segment_end_index + 1, len(path))):
                    x = path[i][0]
                    y = path[i][1]
                    distance = math.hypot(current_x - x, current_y - y)
                    if lookahead < distance:
                        closest_point = (x, y)
                        index = i
                        break
                else:
                    # 전방주시거리 내에 구간 끝점이 있으면 끝점을 목표로
                    closest_point = segment_end_point
                    index = segment_end_index
    
    # 기본 전방주시거리 로직 (current_segment_info가 None인 경우)
    if current_segment_info is None:
        for i in range(index, len(path)):
            x = path[i][0]
            y = path[i][1]
            distance = math.hypot(current_x - x, current_y - y)
            if current_lookahead_distance < distance:
                closest_point = (x, y)
                index = i
                break
    
    # 목표점 설정
    if closest_point is not None:
        target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)

        if target_heading < 0:
            target_heading += 2 * math.pi
        else :
            pass

        desired_steering_angle = target_heading - current_heading
    else:
        target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)

        if target_heading < 0:
            target_heading += 2 * math.pi
        else :
            pass

        desired_steering_angle = target_heading - current_heading
        index = len(path) - 1
    
    # 각도 정규화
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    
    return float(v), float(desired_steering_angle*180/math.pi), index # degree 단위로 변환

class UTMPurePursuit(Node):
    def __init__(self):
        super().__init__('utm_pure_pursuit')
        
        # 로봇 상태
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Waypoint 파일 경로 (수동 지정)
        self.waypoints_file_path = "/home/jh/ros2_workspace/src/remapped_utmcoordinates.txt"  # 필요 시 이 경로를 수정하세요

        # Waypoint 설정 (control.py의 goal과 유사)
        # Waypoint 설정: 단일 경로에서 로드
        self.waypoints = self.load_waypoints_from_file(self.waypoints_file_path)

        
        # control.py와 동일한 flag 시스템
        self.flag = 0  # 0: 대기, 1: 경로 생성, 2: 추적 중
        self.path = []
        self.path_segments = []    # 경로의 구간 정보
        self.i = 0  # 경로 인덱스
        self.current_segment_index = 0  # 현재 구간 인덱스
        self.last_segment_index = -1    # 마지막으로 로깅한 구간 인덱스
        self.first_odometry_received = False  # 첫 번째 GPS 데이터 수신 여부
        self.global_path_generated = False  # 전체 경로 생성 완료 여부
        self.current_waypoint_index = 0  # 현재 waypoint 인덱스
        self.last_target_index = -1  # 마지막 타겟 index (로그 출력용)
        
        # 수동 구간 경계 설정
        self.segment_boundary_indices = segment_boundary_indices
        
        # IMU 보정각도 로드
        self.imu_calibration_angle = self.load_imu_calibration_angle()
        
        # ROS2 인터페이스 설정 (control.py와 동일한 구조)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.path_publisher = self.create_publisher(PoseArray, 'interpolated_path', 10)
        
        # 구간 정보 발행을 위한 String 퍼블리셔 추가
        self.segments_publisher = self.create_publisher(String, 'path_segments', 10)
        
        # 토픽 구독자 - GPS fix 데이터 구독
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/ublox_gps_node/fix',
            self.gps_callback,
            10
        )
        
        # IMU 데이터 구독 (yaw 정보용)
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        # control.py와 동일한 타이머 주기
        timer_period = 0.01  # 100Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info("🚀 구간별 적응적 UTM Pure Pursuit 노드가 시작되었습니다!")
        self.get_logger().info(f"IMU 보정각도: {self.imu_calibration_angle:.6f} rad ({math.degrees(self.imu_calibration_angle):.2f}°)")
        self.get_logger().info(f"총 {len(self.waypoints)}개의 waypoint가 설정되었습니다.")
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f"  Waypoint {i+1}: ({wp[0]:.3f}, {wp[1]:.3f})")
        self.get_logger().info("📡 GPS 및 IMU 데이터 수신 후 자동으로 추적을 시작합니다...")
        self.get_logger().info("🎯 구간별 적응적 전방주시거리, 속도 기능이 활성화되었습니다.")
    
    def load_imu_calibration_angle(self):
        """IMU 보정각도 파일에서 읽기"""
        import os
        
        # 여러 경로에서 파일 찾기
        possible_paths = [
            "imu_calibration_angle.txt",  # 현재 디렉토리
            "/home/jh/ros2_workspace/src/imu_calibration_angle.txt",  # 절대 경로
            os.path.join(os.path.dirname(__file__), "imu_calibration_angle.txt"),  # 스크립트 디렉토리
            os.path.join(os.getcwd(), "imu_calibration_angle.txt")  # 작업 디렉토리
        ]
        
        for file_path in possible_paths:
            try:
                if os.path.exists(file_path):
                    with open(file_path, 'r') as f:
                        # 첫 번째 줄: 라디안 단위 보정각도
                        calibration_angle_rad = float(f.readline().strip())
                        self.get_logger().info(f"IMU 보정각도 파일 로드 성공: {file_path}")
                        self.get_logger().info(f"보정각도: {calibration_angle_rad:.6f} rad")
                        return calibration_angle_rad
            except Exception as e:
                self.get_logger().debug(f"파일 {file_path} 읽기 실패: {e}")
                continue
        
        # 모든 경로에서 파일을 찾지 못한 경우
        self.get_logger().warn("IMU 보정각도 파일(imu_calibration_angle.txt)을 찾을 수 없습니다.")
        self.get_logger().warn("검색한 경로들:")
        for path in possible_paths:
            self.get_logger().warn(f"  - {path}")
        self.get_logger().warn("기본값 0을 사용합니다.")
        return 0.0

    def load_waypoints_from_file(self, file_path):
        """지정된 파일에서 (x, y) 좌표 리스트 로드

        - utm_coordinates.txt: "UTM_X, UTM_Y" (콤마/공백 허용)
        - remapped_wp.txt: "UTM_X UTM_Y YAW" (YAW는 무시)
        """
        waypoints = []
        try:
            if not os.path.exists(file_path):
                self.get_logger().warn(f"Waypoints 파일이 존재하지 않습니다: {file_path}")
                return waypoints
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
                self.get_logger().info(f"Waypoints 파일 로드 성공: {file_path}")
                self.get_logger().info(f"총 {len(waypoints)}개 좌표를 불러왔습니다.")
            else:
                self.get_logger().warn(f"유효한 좌표를 찾지 못했습니다: {file_path}")
        except Exception as e:
            self.get_logger().warn(f"Waypoints 파일 읽기 실패({file_path}): {e}")
        return waypoints

    def generate_global_path(self):
        """수동 구간 설정을 기반으로 글로벌 경로 생성"""
        if len(self.waypoints) < 2:
            self.get_logger().error("waypoint가 2개 미만입니다. 경로 생성이 불가능합니다.")
            return False
        
        
        self.get_logger().info("🗺️ 구간별 경로 생성 중...")
        self.get_logger().info(f"   총 waypoint: {len(self.waypoints)}개")
        self.get_logger().info(f"   시작점: ({self.waypoints[0][0]:.3f}, {self.waypoints[0][1]:.3f})")
        self.get_logger().info(f"   종료점: ({self.waypoints[-1][0]:.3f}, {self.waypoints[-1][1]:.3f})")
        self.get_logger().info(f"   구간 경계 인덱스: {self.segment_boundary_indices}")
        
        # 수동 구간 생성 (waypoint만 사용)
        # waypoint를 그대로 경로로 사용
        full_path_waypoints = self.waypoints.copy()
        
        # 구간 경계 인덱스는 그대로 사용 (현재 위치 추가하지 않음)
        self.get_logger().info(f"   구간 경계 인덱스: {self.segment_boundary_indices}")
        
        waypoint_segments = create_manual_segments(full_path_waypoints, self.segment_boundary_indices)
        
        # waypoint로부터 경로 생성
        self.path, self.path_segments = create_path_from_waypoints(full_path_waypoints, waypoint_segments)
        
        # 총 거리 계산
        total_distance = 0.0
        for i in range(len(self.waypoints) - 1):
            dist = math.sqrt((self.waypoints[i+1][0] - self.waypoints[i][0])**2 + 
                           (self.waypoints[i+1][1] - self.waypoints[i][1])**2)
            total_distance += dist
        
        # 구간 통계 계산
        straight_segments = sum(1 for seg in self.path_segments if seg['type'] == 'straight')
        curve_segments = sum(1 for seg in self.path_segments if seg['type'] == 'curve')

        # 경로를 ROS2 토픽으로 발행
        self.publish_interpolated_path()
        
        # 구간 정보를 ROS2 토픽으로 발행
        self.publish_path_segments()
        
        self.get_logger().info("✅ 구간별 경로 생성 완료!")
        self.get_logger().info(f"   경로점: {len(self.path)}개 (waypoint 그대로 사용)")
        self.get_logger().info(f"   총 거리: {total_distance:.2f}m")
        self.get_logger().info(f"   직선 구간: {straight_segments}개")
        self.get_logger().info(f"   곡선 구간: {curve_segments}개")
        self.get_logger().info(f"   직선 전방주시거리: {lookahead_distance_straight}m")
        self.get_logger().info(f"   곡선 전방주시거리: {lookahead_distance_curve}m")
        self.get_logger().info(f"   직선 속도: {speed_straight}m/s")
        self.get_logger().info(f"   곡선 속도: {speed_curve}m/s")
        self.get_logger().info(f"   구간 그룹: {len(self.path_segments)}개")
        
        # 구간별 상세 정보 출력
        for i, seg in enumerate(self.path_segments):
            self.get_logger().info(f"   구간 {i+1}: {seg['type']} (waypoint {seg['waypoint_start']}-{seg['waypoint_end']}, "
                                 f"경로 {seg['start_index']}-{seg['end_index']}, 거리: {seg['segment_distance']:.2f}m)")
        
        return True
    

    def publish_interpolated_path(self):
        """경로를 PoseArray 메시지로 발행"""
        if not self.path:
            return
            
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "utm"
        
        for point in self.path:
            pose = Pose()
            pose.position.x = float(point[0])
            pose.position.y = float(point[1])
            pose.position.z = 0.0
            # orientation은 기본값 (0, 0, 0, 1)
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        
        self.path_publisher.publish(pose_array)
        self.get_logger().info(f"📡 경로 발행: {len(self.path)}개 점")
    
    def publish_path_segments(self):
        """구간 정보를 JSON 형태로 발행"""
        if not self.path_segments:
            return
        
        import json
        
        # 총 경로 거리 계산
        total_distance = 0.0
        for i in range(len(self.waypoints) - 1):
            dist = math.sqrt((self.waypoints[i+1][0] - self.waypoints[i][0])**2 + 
                           (self.waypoints[i+1][1] - self.waypoints[i][1])**2)
            total_distance += dist
        
        segments_data = {
            'segments': self.path_segments,
            'path_length': total_distance,
            'total_waypoints': len(self.waypoints),
            'total_path_points': len(self.path)
        }
        
        msg = String()
        msg.data = json.dumps(segments_data)
        self.segments_publisher.publish(msg)
        self.get_logger().info(f"📡 구간 정보 발행: {len(self.path_segments)}개 구간")
        
    
    def update_current_segment(self):
        """현재 위치에 따라 구간 정보 업데이트 (거리 기반)"""
        if not self.path_segments or self.current_segment_index >= len(self.path_segments):
            return None
        
        current_segment = self.path_segments[self.current_segment_index]
        
        # 마지막 구간이면 현재 구간 유지
        if self.current_segment_index >= len(self.path_segments) - 1:
            return current_segment
        
        # 다음 구간 정보 가져오기
        next_segment = self.path_segments[self.current_segment_index + 1]
        
        # 다음 구간의 전방주시거리 결정
        if next_segment['type'] == 'straight':
            next_lookahead = lookahead_distance_straight
        else:  # curve
            next_lookahead = lookahead_distance_curve
        
        # 구간 전환 기준 거리 = 다음 구간의 전방주시거리 + 쓰레쉬홀드
        transition_distance = next_lookahead + segment_transition_threshold
        
        # 현재 위치에서 현재 구간의 마지막 점(끝점)까지의 거리 계산
        current_segment_end_point = self.path[current_segment['end_index']]
        distance_to_current_segment_end = math.hypot(
            self.x - current_segment_end_point[0], 
            self.y - current_segment_end_point[1]
        )
        
        # 거리 기반 구간 전환 판정
        if distance_to_current_segment_end <= transition_distance:
            # 다음 구간으로 이동
            self.current_segment_index += 1
            new_segment = self.path_segments[self.current_segment_index]
            self.get_logger().info(f"🔄 구간 전환: {current_segment['type']} → {new_segment['type']}")
            self.get_logger().info(f"   현재 위치: ({self.x:.3f}, {self.y:.3f})")
            self.get_logger().info(f"   현재 구간 끝점: ({current_segment_end_point[0]:.3f}, {current_segment_end_point[1]:.3f})")
            self.get_logger().info(f"   거리: {distance_to_current_segment_end:.3f}m, 전환 기준: {transition_distance:.3f}m")
            self.get_logger().info(f"   구간: {new_segment['start_index']}-{new_segment['end_index']}")
            return new_segment
        
        return current_segment
    
    def update_speed_and_lookahead(self, current_segment):
        """구간에 따른 속도와 전방주시거리 업데이트"""
        global current_speed, current_lookahead_distance
        
        if current_segment is None:
            return
        
        segment_type = current_segment['type']
        
        if segment_type == 'straight':
            # 직선 구간
            target_speed = speed_straight
            target_lookahead = lookahead_distance_straight
        
        else:  # curve
            # 곡선 구간
            target_speed = speed_curve
            target_lookahead = lookahead_distance_curve
        
        # 속도는 부드러운 전환, 전방주시거리는 즉시 변경
        
        current_speed = target_speed
        current_lookahead_distance = target_lookahead  # 즉시 변경
        
    def gps_callback(self, msg):
        """GPS fix 데이터 처리 - 위도/경도를 UTM으로 변환"""
        # GPS 상태 확인
        if msg.status.status < 0:  # NavSatStatus.STATUS_NO_FIX
            self.get_logger().warn("GPS fix가 없습니다. 대기 중...")
            return
        
        # 위도/경도 추출
        latitude = msg.latitude
        longitude = msg.longitude
        
        # UTM 좌표로 변환
        try:
            easting, northing, zone_number, zone_letter = utm.from_latlon(latitude, longitude)
            self.x = easting
            self.y = northing
            
            # 첫 번째 GPS 데이터 수신 시 로그 출력 및 경로 생성 시작
            if not self.first_odometry_received:
                self.get_logger().info("=" * 60)
                self.get_logger().info("📍 GPS 초기 위치 정보")
                self.get_logger().info(f"   위도: {latitude:.8f}°")
                self.get_logger().info(f"   경도: {longitude:.8f}°")
                self.get_logger().info(f"   UTM X (Easting): {self.x:.6f} m")
                self.get_logger().info(f"   UTM Y (Northing): {self.y:.6f} m")
                self.get_logger().info(f"   UTM Zone: {zone_number}{zone_letter}")
                self.get_logger().info("=" * 60)
                self.first_odometry_received = True
                
                # 전체 경로 생성 시작
                if len(self.waypoints) > 0 and not self.global_path_generated:
                    self.get_logger().info("🚀 전체 경로 생성을 시작합니다!")
                    if self.generate_global_path():
                        self.global_path_generated = True
                        self.flag = 2  # 바로 추적 시작
                        self.get_logger().info("🎯 경로 추적을 시작합니다!")
                    else:
                        self.get_logger().error("경로 생성에 실패했습니다.")
                        
        except Exception as e:
            self.get_logger().error(f"UTM 변환 실패: {e}")
    
    def imu_callback(self, msg):
        """IMU 데이터 처리 - yaw 각도만 추출"""
        # 쿼터니언에서 yaw 각도 추출
        raw_yaw = euler_from_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        
        # IMU 보정각도 적용
        self.yaw = raw_yaw + self.imu_calibration_angle
    
    def timer_callback(self):
        """메인 제어 루프 (전체 경로 추적)"""
        if self.flag == 2:
            # 구간 정보 업데이트
            current_segment = self.update_current_segment()
            
            # 속도와 전방주시거리 업데이트
            self.update_speed_and_lookahead(current_segment)

            # 구간이 바뀐 경우: 현재 구간 타입과 속도/전방주시거리 로그
            if current_segment is not None and self.current_segment_index != self.last_segment_index:
                seg_type = current_segment['type']
                seg_start = current_segment['start_index']
                seg_end = current_segment['end_index']
                try:
                    speed_val = current_speed
                except NameError:
                    speed_val = 0.0
                try:
                    lookahead_val = current_lookahead_distance
                except NameError:
                    lookahead_val = 0.0
                self.get_logger().info(f"🧭 현재 주행 구간: {seg_type} (index {seg_start}-{seg_end})")
                self.get_logger().info(f"   현재 속도: {speed_val:.2f} m/s, 전방주시거리: {lookahead_val:.2f} m")
                self.last_segment_index = self.current_segment_index
            
            # 구간별 적응적 Pure Pursuit 제어 실행
            twist = Twist()
            twist.linear.x, twist.angular.z, self.i = pure_pursuit(
                self.x, self.y, self.yaw, self.path, self.i, current_segment
            )
            
            # 디버깅: 타겟 index가 변경될 때만 로그 출력
            if self.i != self.last_target_index:
                if current_segment is not None:
                    print(f"🎯 타겟 변경: index={self.i}, 구간={current_segment['type']} ({current_segment['start_index']}-{current_segment['end_index']}), 좌표=({self.path[self.i][0]:.3f}, {self.path[self.i][1]:.3f})")
                else:
                    print(f"🎯 타겟 변경: index={self.i}, 구간 정보 없음, 좌표=({self.path[self.i][0]:.3f}, {self.path[self.i][1]:.3f})")
                self.last_target_index = self.i
            
            
            # 경로 완료 판정 (마지막 path 도달)
            if self.i >= len(self.path) - 1:
                # 경로 완료
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.flag = 0
                self.get_logger().info("🎉 전체 경로 완료!")
                self.get_logger().info("✅ 미션 성공!")
                self.get_logger().info("로봇이 정지합니다.")
            
            
            self.publisher.publish(twist)
            
    
    


def main(args=None):
    rclpy.init(args=args)
    
    # UTM Pure Pursuit 노드 생성
    utm_pure_pursuit = UTMPurePursuit()
    
    try:
        rclpy.spin(utm_pure_pursuit)
    except KeyboardInterrupt:
        utm_pure_pursuit.get_logger().info("노드 종료 중...")
    finally:
        utm_pure_pursuit.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()