import rclpy
from rclpy.node import Node
import numpy as np
import math
import scipy.interpolate as si
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
# 전방주시거리 설정 (세분화된 구간별)
# 직선 구간 (1-5)
lookahead_distance_s1 = 1.5  # 직선1 구간 전방주시거리 (미터)
lookahead_distance_s2 = 1.4  # 직선2 구간 전방주시거리 (미터)
lookahead_distance_s3 = 1.3  # 직선3 구간 전방주시거리 (미터)
lookahead_distance_s4 = 1.2  # 직선4 구간 전방주시거리 (미터)
lookahead_distance_s5 = 1.1  # 직선5 구간 전방주시거리 (미터)

# 곡선 구간 (1-3)
lookahead_distance_c1 = 0.5  # 곡선1 구간 전방주시거리 (미터)
lookahead_distance_c2 = 0.4  # 곡선2 구간 전방주시거리 (미터)
lookahead_distance_c3 = 0.3  # 곡선3 구간 전방주시거리 (미터)

# 후진 구간 (1-2)
lookahead_distance_r1 = 0.8  # 후진1 구간 전방주시거리 (미터)
lookahead_distance_r2 = 0.7  # 후진2 구간 전방주시거리 (미터)

current_lookahead_distance = 0.3   # 현재 전방주시거리 (초기값)

# 속도 설정 (세분화된 구간별)
# 직선 구간 (1-5)
speed_s1 = 30.0   # 직선1 구간 속도 (m/s)
speed_s2 = 28.0   # 직선2 구간 속도 (m/s)
speed_s3 = 26.0   # 직선3 구간 속도 (m/s)
speed_s4 = 24.0   # 직선4 구간 속도 (m/s)
speed_s5 = 22.0   # 직선5 구간 속도 (m/s)

# 곡선 구간 (1-3)
speed_c1 = 25.0   # 곡선1 구간 속도 (m/s)
speed_c2 = 23.0   # 곡선2 구간 속도 (m/s)
speed_c3 = 21.0   # 곡선3 구간 속도 (m/s)

# 후진 구간 (1-2)
speed_r1 = -20.0  # 후진1 구간 속도 (m/s, 음수)
speed_r2 = -18.0  # 후진2 구간 속도 (m/s, 음수)

current_speed = 30.0    # 현재 속도 (초기값)

# 보간 밀도 설정 (세분화된 구간별)
# 직선 구간 (1-5)
interpolation_density_s1 = 2   # 직선1 구간: 1m당 점의 개수
interpolation_density_s2 = 2   # 직선2 구간: 1m당 점의 개수
interpolation_density_s3 = 2   # 직선3 구간: 1m당 점의 개수
interpolation_density_s4 = 2   # 직선4 구간: 1m당 점의 개수
interpolation_density_s5 = 2   # 직선5 구간: 1m당 점의 개수

# 곡선 구간 (1-3)
interpolation_density_c1 = 5   # 곡선1 구간: 1m당 점의 개수
interpolation_density_c2 = 5   # 곡선2 구간: 1m당 점의 개수
interpolation_density_c3 = 5   # 곡선3 구간: 1m당 점의 개수

# 후진 구간 (1-2)
interpolation_density_r1 = 5   # 후진1 구간: 1m당 점의 개수
interpolation_density_r2 = 5   # 후진2 구간: 1m당 점의 개수

# 구간 설정: [인덱스, 구간타입, 버전] 형태로 설정
# 구간타입: 's' (straight), 'c' (curve), 'r' (reverse)
# 버전: 1-5 (직선), 1-3 (곡선), 1-2 (후진)
# 예: [[8, 's', 1], [16, 's', 2], [20, 'c', 1], [25, 'r', 1], [30, 'c', 2], [35, 's', 3], [43, 'c', 3]]
segment_config = [[13, 's3', 1], [17, 's2', 2], [19, 's1', 1], [25, 'r', 1], [30, 'c', 2], [35, 's', 3], [43, 'c', 3]]  # 이 배열을 수동으로 설정

# 구간 전환 거리 임계값
segment_transition_threshold = 0.0  # 전방주시거리에 추가할 거리 (m)

# 경사로 대기 기능 설정
slope_speed = 25.0  # 경사로에서 멈춰있기 위한 최소 출력값 (m/s)
slope_wait_time = 4.0  # 경사로에서 대기할 시간 (초)
slope_waypoints = [10, 25]  # 경사로 대기가 필요한 waypoint 인덱스들 (0부터 시작)

def euler_from_quaternion(x, y, z, w):
    """쿼터니언에서 yaw 각도 추출 (control.py와 동일)"""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def create_manual_segments(waypoints, segment_config):
    """인덱스별 구간 타입 설정을 기반으로 구간 정보 생성"""
    segments = []
    total_waypoints = len(waypoints)
    
    if not segment_config or total_waypoints < 2:
        # 설정이 없으면 전체를 직선으로 처리
        segments.append({
            'type': 'straight',
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
    
    # 유효성 검사: 인덱스가 waypoint 범위 내에 있는지 확인
    valid_config = []
    for config_item in sorted_config:
        if len(config_item) == 3:
            idx, seg_type, version = config_item
        else:
            idx, seg_type = config_item
            version = 1  # 기본 버전
        
        if 0 <= idx <= total_waypoints and seg_type in type_mapping:
            valid_config.append((idx, seg_type, version))
        else:
            print(f"경고: 잘못된 구간 설정 무시 - 인덱스: {idx}, 타입: {seg_type}, 버전: {version}")
    
    if not valid_config:
        # 유효한 설정이 없으면 전체를 직선으로 처리
        segments.append({
            'type': 'straight',
            'start_index': 0,
            'end_index': total_waypoints - 1,
            'waypoint_start': 0,
            'waypoint_end': total_waypoints - 1
        })
        return segments
    
    # 구간 생성
    start_idx = 0
    
    for i, (end_idx, seg_type, version) in enumerate(valid_config):
        # 현재 구간 저장 (version 정보 포함)
        segments.append({
            'type': type_mapping[seg_type],
            'version': version,
            'start_index': start_idx,
            'end_index': end_idx,
            'waypoint_start': start_idx,
            'waypoint_end': end_idx
        })
        
        # 다음 구간 시작점 설정
        start_idx = end_idx + 1
    
    # 마지막 구간이 전체 waypoint를 포함하지 않는 경우, 마지막 구간을 확장
    if segments and segments[-1]['end_index'] < total_waypoints - 1:
        # 마지막 구간을 전체 끝까지 확장
        segments[-1]['end_index'] = total_waypoints - 1
        segments[-1]['waypoint_end'] = total_waypoints - 1
    
    return segments

def segment_based_bspline_planning(waypoints, segments):
    """구간별 B-Spline 경로 스무딩 (구간별 개별 보간)"""
    try:
        if len(waypoints) < 2:
            return [], []
        
        total_path = []
        path_segments = []
        
        # 구간별로 개별적으로 B-Spline 보간 (구간 간 연결점 포함)
        for i, segment in enumerate(segments):
            start_wp = segment['waypoint_start']
            end_wp = segment['waypoint_end']
            segment_type = segment['type']
            version = segment.get('version', 1)
            
            # 구간별 waypoint 추출 (다음 구간의 시작점도 포함하여 연속성 보장)
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
            for i in range(len(segment_waypoints) - 1):
                dist = math.sqrt((segment_waypoints[i+1][0] - segment_waypoints[i][0])**2 + 
                               (segment_waypoints[i+1][1] - segment_waypoints[i][1])**2)
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
        print(f"구간별 보간 오류: {e}")
        return waypoints, []

def pure_pursuit(current_x, current_y, current_heading, path, index, current_segment_info=None):
    """구간별 적응적 Pure Pursuit 알고리즘 (직선-곡선-후진 지원)"""
    global current_lookahead_distance, current_speed
    closest_point = None
    v = current_speed
    
    # 후진 구간인지 확인
    is_reverse_segment = current_segment_info is not None and current_segment_info['type'] == 'reverse'
    
    # 후진 구간에서는 yaw에 π를 더함
    effective_heading = (current_heading + math.pi) % (2 * math.pi) if is_reverse_segment else current_heading
    
    if current_segment_info is None:
        # 기본 전방주시거리 사용
        lookahead = current_lookahead_distance
    else:
        segment_type = current_segment_info['type']
        segment_end_index = current_segment_info['end_index']
        segment_end_point = path[segment_end_index]
        
        if segment_type == 'straight':
            # 직선 구간 - version별 전방주시거리 사용
            version = current_segment_info.get('version', 1)
            lookahead_distances = [lookahead_distance_s1, lookahead_distance_s2, lookahead_distance_s3, 
                                 lookahead_distance_s4, lookahead_distance_s5]
            lookahead = lookahead_distances[min(version - 1, 4)]  # 1-5를 0-4로 변환, 범위 초과시 마지막 값
            
            if index >= segment_end_index:
                # 구간 끝에 도달 - 끝점을 목표로
                closest_point = segment_end_point
                index = segment_end_index
            else:
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
        
        elif segment_type == 'curve':
            # 곡선 구간 - version별 전방주시거리 사용
            version = current_segment_info.get('version', 1)
            lookahead_distances = [lookahead_distance_c1, lookahead_distance_c2, lookahead_distance_c3]
            lookahead = lookahead_distances[min(version - 1, 2)]  # 1-3을 0-2로 변환, 범위 초과시 마지막 값
            
            if index >= segment_end_index:
                # 구간 끝에 도달 - 끝점을 목표로
                closest_point = segment_end_point
                index = segment_end_index
            else:
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
        
        else:  # reverse
            # 후진 구간 - version별 전방주시거리 사용
            version = current_segment_info.get('version', 1)
            lookahead_distances = [lookahead_distance_r1, lookahead_distance_r2]
            lookahead = lookahead_distances[min(version - 1, 1)]  # 1-2를 0-1로 변환, 범위 초과시 마지막 값
            
            if index >= segment_end_index:
                # 구간 끝에 도달 - 끝점을 목표로
                closest_point = segment_end_point
                index = segment_end_index
            else:
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

        desired_steering_angle = target_heading - effective_heading
    else:
        target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)

        if target_heading < 0:
            target_heading += 2 * math.pi
        else :
            pass

        desired_steering_angle = target_heading - effective_heading
        index = len(path) - 1
    
    # 각도 정규화
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    
    # 후진 구간에서는 조향각도에 음수 적용
    if is_reverse_segment:
        desired_steering_angle = -desired_steering_angle
    
    return v, float(desired_steering_angle*180/math.pi), index # degree 단위로 변환

class UTMPurePursuit(Node):
    def __init__(self):
        super().__init__('utm_pure_pursuit')
        
        # 로봇 상태
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Waypoint 파일 경로 (수동 지정)
        self.waypoints_file_path = "/home/jh/ros2_workspace/src/utm_coordinates.txt"  # 필요 시 이 경로를 수정하세요

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
        
        # 경사로 대기 기능 관련 변수들
        self.is_slope_waiting = False  # 경사로 대기 중인지 여부
        self.slope_wait_start_time = 0.0  # 경사로 대기 시작 시간
        self.slope_wait_waypoint_index = -1  # 경사로 대기 중인 waypoint 인덱스
        
        # 구간 설정 (인덱스별 타입 지정)
        self.segment_config = segment_config
        
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
        self.get_logger().info("🎯 구간별 적응적 전방주시거리, 속도, 보간 기능이 활성화되었습니다.")
        self.get_logger().info("🛑 경사로 대기 기능이 활성화되었습니다:")
        self.get_logger().info(f"   대기 waypoint: {slope_waypoints}")
        self.get_logger().info(f"   대기 시간: {slope_wait_time}초")
        self.get_logger().info(f"   최소 출력 속도: {slope_speed} m/s")
    
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
        
        
        self.get_logger().info("🗺️ 수동 구간 기반 경로 생성 중...")
        self.get_logger().info(f"   총 waypoint: {len(self.waypoints)}개")
        self.get_logger().info(f"   현재 위치 시작점: ({self.x:.3f}, {self.y:.3f})")
        self.get_logger().info(f"   종료점: ({self.waypoints[-1][0]:.3f}, {self.waypoints[-1][1]:.3f})")
        self.get_logger().info(f"   원본 구간 설정: {self.segment_config}")
        
        # 수동 구간 생성 (현재 위치를 포함한 전체 경로 기준)
        # 현재 위치를 시작점으로 하는 전체 경로 생성
        full_path_waypoints = [(self.x, self.y)]  # 현재 위치를 시작점으로
        full_path_waypoints.extend(self.waypoints)  # 모든 waypoint 추가
        
        # 구간 설정을 현재 위치 포함 기준으로 조정 (인덱스 +1)
        adjusted_segment_config = []
        for config_item in self.segment_config:
            if len(config_item) == 3:
                # [인덱스, 구간타입, 버전] 형식
                idx, seg_type, version = config_item
                adjusted_segment_config.append([idx + 1, seg_type, version])  # +1은 현재 위치 때문
            else:
                # [인덱스, 구간타입] 형식 (하위 호환성)
                idx, seg_type = config_item
                adjusted_segment_config.append([idx + 1, seg_type])  # +1은 현재 위치 때문
        self.get_logger().info(f"   조정된 구간 설정: {adjusted_segment_config}")
        
        waypoint_segments = create_manual_segments(full_path_waypoints, adjusted_segment_config)
        
        # 구간별 보간으로 경로 생성 (현재 위치 포함)
        self.path, self.path_segments = segment_based_bspline_planning(full_path_waypoints, waypoint_segments)
        
        # 총 거리 계산 (현재 위치 포함한 전체 경로 기준)
        total_distance = 0.0
        for i in range(len(full_path_waypoints) - 1):
            dist = math.sqrt((full_path_waypoints[i+1][0] - full_path_waypoints[i][0])**2 + 
                           (full_path_waypoints[i+1][1] - full_path_waypoints[i][1])**2)
            total_distance += dist
        
        # 구간 통계 계산
        straight_segments = sum(1 for seg in self.path_segments if seg['type'] == 'straight')
        curve_segments = sum(1 for seg in self.path_segments if seg['type'] == 'curve')
        reverse_segments = sum(1 for seg in self.path_segments if seg['type'] == 'reverse')

        # 보간된 경로를 ROS2 토픽으로 발행
        self.publish_interpolated_path()
        
        # 구간 정보를 ROS2 토픽으로 발행
        self.publish_path_segments()
        
        self.get_logger().info("✅ 수동 구간 기반 경로 생성 완료!")
        self.get_logger().info(f"   스무딩된 경로점: {len(self.path)}개")
        self.get_logger().info(f"   총 거리: {total_distance:.2f}m")
        self.get_logger().info(f"   직선 구간: {straight_segments}개 (s1-s5 버전별 보간 밀도)")
        self.get_logger().info(f"   곡선 구간: {curve_segments}개 (c1-c3 버전별 보간 밀도)")
        self.get_logger().info(f"   후진 구간: {reverse_segments}개 (r1-r2 버전별 보간 밀도)")
        self.get_logger().info(f"   직선 전방주시거리: s1({lookahead_distance_s1}m) ~ s5({lookahead_distance_s5}m)")
        self.get_logger().info(f"   곡선 전방주시거리: c1({lookahead_distance_c1}m) ~ c3({lookahead_distance_c3}m)")
        self.get_logger().info(f"   후진 전방주시거리: r1({lookahead_distance_r1}m) ~ r2({lookahead_distance_r2}m)")
        self.get_logger().info(f"   직선 속도: s1({speed_s1}m/s) ~ s5({speed_s5}m/s)")
        self.get_logger().info(f"   곡선 속도: c1({speed_c1}m/s) ~ c3({speed_c3}m/s)")
        self.get_logger().info(f"   후진 속도: r1({speed_r1}m/s) ~ r2({speed_r2}m/s)")
        self.get_logger().info(f"   구간 그룹: {len(self.path_segments)}개")
        
        # 구간별 상세 정보 출력 (version 정보 포함)
        for i, seg in enumerate(self.path_segments):
            version = seg.get('version', 1)
            self.get_logger().info(f"   구간 {i+1}: {seg['type']}{version} (waypoint {seg['waypoint_start']}-{seg['waypoint_end']}, "
                                 f"경로 {seg['start_index']}-{seg['end_index']}, 거리: {seg['segment_distance']:.2f}m)")
        
        return True
    

    def publish_interpolated_path(self):
        """보간된 경로를 PoseArray 메시지로 발행"""
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
        self.get_logger().info(f"📡 보간된 경로 발행: {len(self.path)}개 점")
    
    def publish_path_segments(self):
        """구간 정보를 JSON 형태로 발행"""
        if not self.path_segments:
            return
        
        import json
        
        # 총 경로 거리 계산 (현재 위치 포함한 전체 경로 기준)
        full_path_waypoints = [(self.x, self.y)]  # 현재 위치를 시작점으로
        full_path_waypoints.extend(self.waypoints)  # 모든 waypoint 추가
        
        total_distance = 0.0
        for i in range(len(full_path_waypoints) - 1):
            dist = math.sqrt((full_path_waypoints[i+1][0] - full_path_waypoints[i][0])**2 + 
                           (full_path_waypoints[i+1][1] - full_path_waypoints[i][1])**2)
            total_distance += dist
        
        segments_data = {
            'segments': self.path_segments,
            'path_length': total_distance,
            'total_waypoints': len(full_path_waypoints),  # 현재 위치 포함한 전체 waypoint 수
            'total_path_points': len(self.path)
        }
        
        msg = String()
        msg.data = json.dumps(segments_data)
        self.segments_publisher.publish(msg)
        self.get_logger().info(f"📡 구간 정보 발행: {len(self.path_segments)}개 구간")
        
    
    def update_current_segment(self):
        """현재 위치에 따라 구간 정보 업데이트 (거리 기반 전환)"""
        current_segment = self.get_current_segment()
        if current_segment is None:
            return None
        
        # 현재 구간의 끝점까지의 거리 계산
        if current_segment['end_index'] < len(self.path):
            segment_end_point = self.path[current_segment['end_index']]
            distance_to_segment_end = math.hypot(
                self.x - segment_end_point[0], 
                self.y - segment_end_point[1]
            )
        else:
            distance_to_segment_end = 0
        
        # 현재 구간 타입과 다음 구간 타입에 따른 전환 조건
        current_type = current_segment['type']
        next_segment_type = self.get_next_segment_type()
        
        # 구간 전환 조건 확인
        should_transition = False
        
        if current_type == 'straight':
            if next_segment_type == 'curve':
                # 직선 → 곡선: 다음 곡선 구간의 전방주시거리 기준
                next_segment = self.get_next_segment()
                next_version = next_segment.get('version', 1) if next_segment else 1
                next_lookaheads = [lookahead_distance_c1, lookahead_distance_c2, lookahead_distance_c3]
                next_lookahead = next_lookaheads[min(next_version - 1, 2)]
                should_transition = distance_to_segment_end <= next_lookahead
            elif next_segment_type == 'reverse':
                # 직선 → 후진
                should_transition = distance_to_segment_end < 0.1
        
        elif current_type == 'curve':
            if next_segment_type == 'straight':
                # 곡선 → 직선: 다음 직선 구간의 전방주시거리 기준
                next_segment = self.get_next_segment()
                next_version = next_segment.get('version', 1) if next_segment else 1
                next_lookaheads = [lookahead_distance_s1, lookahead_distance_s2, lookahead_distance_s3, 
                                 lookahead_distance_s4, lookahead_distance_s5]
                next_lookahead = next_lookaheads[min(next_version - 1, 4)]
                should_transition = distance_to_segment_end < next_lookahead
            elif next_segment_type == 'reverse':
                # 곡선 → 후진
                should_transition = distance_to_segment_end < 0.1
        
        elif current_type == 'reverse':
            if next_segment_type == 'straight':
                # 후진 → 직선: 다음 직선 구간의 전방주시거리 기준
                next_segment = self.get_next_segment()
                next_version = next_segment.get('version', 1) if next_segment else 1
                next_lookaheads = [lookahead_distance_s1, lookahead_distance_s2, lookahead_distance_s3, 
                                 lookahead_distance_s4, lookahead_distance_s5]
                next_lookahead = next_lookaheads[min(next_version - 1, 4)]
                should_transition = distance_to_segment_end < next_lookahead
            elif next_segment_type == 'curve':
                # 후진 → 곡선: 다음 곡선 구간의 전방주시거리 기준
                next_segment = self.get_next_segment()
                next_version = next_segment.get('version', 1) if next_segment else 1
                next_lookaheads = [lookahead_distance_c1, lookahead_distance_c2, lookahead_distance_c3]
                next_lookahead = next_lookaheads[min(next_version - 1, 2)]
                should_transition = distance_to_segment_end < next_lookahead
            else:
                should_transition = distance_to_segment_end < 0.08
        
        # 구간 전환 실행
        if should_transition and self.get_next_segment() is not None:
            self.current_segment_index += 1
            new_segment = self.get_current_segment()
            
            # 구간 타입을 한글로 표시 (version 정보 포함)
            type_names = {'straight': '직선', 'curve': '곡선', 'reverse': '후진'}
            current_type_name = type_names.get(current_segment['type'], current_segment['type'])
            new_type_name = type_names.get(new_segment['type'], new_segment['type'])
            
            # version 정보 추가
            current_version = current_segment.get('version', 1)
            new_version = new_segment.get('version', 1)
            
            self.get_logger().info(f"🔄 구간 전환: {current_type_name}{current_version} → {new_type_name}{new_version}")
            self.get_logger().info(f"   구간: {new_segment['start_index']}-{new_segment['end_index']}")
            return new_segment
        
        return current_segment
    
    def get_next_segment_type(self):
        """다음 구간의 타입을 반환"""
        if self.current_segment_index < len(self.path_segments) - 1:
            return self.path_segments[self.current_segment_index + 1]['type']
        return None
    
    def get_next_segment(self):
        """다음 구간 전체 정보를 반환 (type, version 포함)"""
        if self.current_segment_index < len(self.path_segments) - 1:
            return self.path_segments[self.current_segment_index + 1]
        return None
    
    def get_current_segment(self):
        """현재 구간 전체 정보를 반환 (type, version 포함)"""
        if 0 <= self.current_segment_index < len(self.path_segments):
            return self.path_segments[self.current_segment_index]
        return None
    
    def update_speed_and_lookahead(self, current_segment):
        """구간에 따른 속도와 전방주시거리 업데이트 (후진 구간 지원)"""
        global current_speed, current_lookahead_distance
        
        if current_segment is None:
            return
        
        segment_type = current_segment['type']
        segment_end_index = current_segment['end_index']
        
        # 현재 위치에서 구간 끝점까지의 거리
        if segment_end_index < len(self.path):
            segment_end_point = self.path[segment_end_index]
            distance_to_segment_end = math.hypot(
                self.x - segment_end_point[0], 
                self.y - segment_end_point[1]
            )
        else:
            distance_to_segment_end = 0
        
        if segment_type == 'straight':
            # 직선 구간 - version별 파라미터 사용
            version = current_segment.get('version', 1)
            speeds = [speed_s1, speed_s2, speed_s3, speed_s4, speed_s5]
            lookaheads = [lookahead_distance_s1, lookahead_distance_s2, lookahead_distance_s3, 
                         lookahead_distance_s4, lookahead_distance_s5]
            target_speed = speeds[min(version - 1, 4)]
            target_lookahead = lookaheads[min(version - 1, 4)]
            
            # 구간 끝에 가까워지면 속도를 점진적으로 감소
            if distance_to_segment_end < (target_lookahead + segment_transition_threshold):
                next_segment_type = self.get_next_segment_type()
                if next_segment_type == 'curve':
                    # 다음 곡선 구간의 version을 가져와서 전방주시거리 결정
                    next_segment = self.get_next_segment()
                    next_version = next_segment.get('version', 1) if next_segment else 1
                    next_lookaheads = [lookahead_distance_c1, lookahead_distance_c2, lookahead_distance_c3]
                    next_lookahead = next_lookaheads[min(next_version - 1, 2)]
                    
                    transition_ratio = (distance_to_segment_end - next_lookahead) / (target_lookahead + segment_transition_threshold)
                    target_speed = target_speed * transition_ratio
                elif next_segment_type == 'reverse':
                    # 후진 전에는 속도를 0으로 감소
                    transition_ratio = distance_to_segment_end / (target_lookahead + segment_transition_threshold)
                    target_speed = target_speed * transition_ratio
        
        elif segment_type == 'curve':
            # 곡선 구간 - version별 파라미터 사용
            version = current_segment.get('version', 1)
            speeds = [speed_c1, speed_c2, speed_c3]
            lookaheads = [lookahead_distance_c1, lookahead_distance_c2, lookahead_distance_c3]
            target_speed = speeds[min(version - 1, 2)]
            target_lookahead = lookaheads[min(version - 1, 2)]
            
            if distance_to_segment_end < (target_lookahead + segment_transition_threshold):
                # 구간 끝에 가까워짐
                next_segment_type = self.get_next_segment_type()
                if next_segment_type == 'reverse':
                    # 후진 전에는 속도를 0으로 감소
                    transition_ratio = distance_to_segment_end / (target_lookahead + segment_transition_threshold)
                    target_speed = target_speed * transition_ratio
        
        else:  # reverse
            # 후진 구간 - version별 파라미터 사용
            version = current_segment.get('version', 1)
            speeds = [speed_r1, speed_r2]
            lookaheads = [lookahead_distance_r1, lookahead_distance_r2]
            target_speed = speeds[min(version - 1, 1)]
            target_lookahead = lookaheads[min(version - 1, 1)]
            
            # 후진 구간에서 다음 구간으로 전환 시
            if distance_to_segment_end < (target_lookahead + segment_transition_threshold):
                # 구간 끝에 가까워짐
                transition_ratio = distance_to_segment_end / (target_lookahead + segment_transition_threshold)
                target_speed = target_speed * transition_ratio
        
        # 속도는 부드러운 전환, 전방주시거리는 즉시 변경
        speed_transition_rate = 0.2  #20%씩 변화
        
        current_speed = (1 - speed_transition_rate) * current_speed + speed_transition_rate * target_speed
        current_lookahead_distance = target_lookahead  # 즉시 변경
    
    def calculate_current_waypoint_index(self):
        """현재 경로 인덱스를 기반으로 waypoint 인덱스 계산"""
        if not self.path_segments:
            return 0
        
        # 현재 경로 인덱스가 어느 구간에 속하는지 찾기
        for i, segment in enumerate(self.path_segments):
            if segment['start_index'] <= self.i <= segment['end_index']:
                # 구간 내에서의 상대적 위치 계산
                relative_pos = (self.i - segment['start_index']) / max(1, segment['end_index'] - segment['start_index'])
                # waypoint 인덱스 계산 (구간의 시작 waypoint + 상대적 위치)
                waypoint_idx = segment['waypoint_start'] + int(relative_pos * (segment['waypoint_end'] - segment['waypoint_start']))
                return waypoint_idx
        
        return 0
    
    def should_start_slope_waiting(self, current_waypoint_idx):
        """경사로 대기를 시작해야 하는지 확인"""
        if self.is_slope_waiting:
            return False  # 이미 대기 중이면 시작하지 않음
        
        # 현재 waypoint가 경사로 대기 목록에 있는지 확인
        return current_waypoint_idx in slope_waypoints
    
    def start_slope_waiting(self, waypoint_idx):
        """경사로 대기 시작"""
        self.is_slope_waiting = True
        self.slope_wait_start_time = time.time()
        self.slope_wait_waypoint_index = waypoint_idx
        self.get_logger().info(f"🛑 경사로 대기 시작: Waypoint {waypoint_idx}에서 {slope_wait_time}초 대기")
        self.get_logger().info(f"   최소 출력 속도: {slope_speed} m/s")
    
    def handle_slope_waiting(self):
        """경사로 대기 중 처리"""
        current_time = time.time()
        elapsed_time = current_time - self.slope_wait_start_time
        
        # 대기 시간이 지났는지 확인
        if elapsed_time >= slope_wait_time:
            # 대기 완료
            self.is_slope_waiting = False
            self.slope_wait_start_time = 0.0
            self.slope_wait_waypoint_index = -1
            self.get_logger().info(f"✅ 경사로 대기 완료: {elapsed_time:.1f}초 대기 후 정상 주행 재개")
            # 정상 주행을 위해 빈 twist 반환 (다음 루프에서 Pure Pursuit 실행)
            return Twist()
        
        # 대기 중: 최소 출력으로 유지
        twist = Twist()
        twist.linear.x = slope_speed
        twist.angular.z = 0.0  # 조향각은 0으로 유지
        
        # 대기 상태 로그 (1초마다)
        if int(elapsed_time) != int(elapsed_time - 0.01):  # 1초마다 로그
            remaining_time = slope_wait_time - elapsed_time
            self.get_logger().info(f"⏳ 경사로 대기 중: {remaining_time:.1f}초 남음 (속도: {slope_speed} m/s)")
        
        return twist
        
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
        self.yaw = (raw_yaw + self.imu_calibration_angle) % (2 * math.pi)
    
    def timer_callback(self):
        """메인 제어 루프 (전체 경로 추적)"""
        if self.flag == 2:
            # 구간 정보 업데이트
            current_segment = self.update_current_segment()
            
            # 속도와 전방주시거리 업데이트
            self.update_speed_and_lookahead(current_segment)

            # 구간이 바뀐 경우: 현재 구간 타입과 속도/전방주시거리 로그
            if current_segment is not None and self.current_segment_index != self.last_segment_index:
                current_segment = self.get_current_segment()  # 최신 구간 정보 가져오기
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
                # 구간 타입을 한글로 표시 (version 정보 포함)
                type_names = {'straight': '직선', 'curve': '곡선', 'reverse': '후진'}
                seg_type_name = type_names.get(seg_type, seg_type)
                seg_version = current_segment.get('version', 1)
                
                self.get_logger().info(f"🧭 현재 주행 구간: {seg_type_name}{seg_version} (index {seg_start}-{seg_end})")
                self.get_logger().info(f"   현재 속도: {speed_val:.2f} m/s, 전방주시거리: {lookahead_val:.2f} m")
                self.last_segment_index = self.current_segment_index
            
            # 경사로 대기 로직 처리
            twist = Twist()
            
            # 현재 waypoint 인덱스 계산 (전체 waypoint 기준)
            current_waypoint_idx = self.calculate_current_waypoint_index()
            
            # 경사로 대기 체크
            if self.should_start_slope_waiting(current_waypoint_idx):
                self.start_slope_waiting(current_waypoint_idx)
            
            # 경사로 대기 중인 경우
            if self.is_slope_waiting:
                twist = self.handle_slope_waiting()
            else:
                # 구간별 적응적 Pure Pursuit 제어 실행
                twist.linear.x, twist.angular.z, self.i = pure_pursuit(
                    self.x, self.y, self.yaw, self.path, self.i, current_segment
                )

            distance_to_path_end = math.hypot(
                self.x - self.path[-1][0], 
                self.y - self.path[-1][1]
            )

            # 경로 완료 판정 (마지막 path 도달)
            if (self.i >= len(self.path) - 10) and distance_to_path_end < 0.1:
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