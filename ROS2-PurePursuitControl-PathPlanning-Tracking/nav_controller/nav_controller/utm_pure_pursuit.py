import rclpy
from rclpy.node import Node
import numpy as np
import math
# import scipy.interpolate as si  # 보간 사용하지 않으므로 주석 처리
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist, PoseArray, Pose
from std_msgs.msg import String
from std_msgs.msg import Bool
import time
import matplotlib.pyplot as plt
from matplotlib import patches, transforms
import utm
import os
import re

# Pure Pursuit 파라미터 (구간별 적응적 제어)
# 전방주시거리 설정 (세분화된 구간별)
# 직선 구간 (1-5)
lookahead_distance_s1 = 0.5  # 직선1 구간 전방주시거리 (미터) parking
lookahead_distance_s2 = 2.0  # 직선2 구간 전방주시거리 (미터) downhill
lookahead_distance_s3 = 2.0  # 직선3 구간 전방주시거리 (미터) uphill
lookahead_distance_s4 = 2.0  # 직선4 구간 전방주시거리 (미터) default
lookahead_distance_s5 = 2.0  # 직선5 구간 전방주시거리 (미터) fast

# 곡선 구간 (1-3)
lookahead_distance_c1 = 0.5  # 곡선1 구간 전방주시거리 (미터) parking
lookahead_distance_c2 = 1.0  # 곡선2 구간 전방주시거리 (미터) default
lookahead_distance_c3 = 1.5  # 곡선3 구간 전방주시거리 (미터)

# 후진 구간 (1-2)
lookahead_distance_r1 = 1.5  # 후진1 구간 전방주시거리 (미터) default
lookahead_distance_r2 = 2.5  # 후진2 구간 전방주시거리 (미터) slow

current_lookahead_distance = 0.3   # 현재 전방주시거리 (초기값)

# 속도 설정 (세분화된 구간별)
# 직선 구간 (1-5)
speed_s1 = 35.0   # 직선1 구간 속도 (m/s)
speed_s2 = 70.0   # 직선2 구간 속도 (m/s) 
speed_s3 = 90.0   # 직선3 구간 속도 (m/s) 
speed_s4 = 100.0   # 직선4 구간 속도 (m/s) 
speed_s5 = 130.0   # 직선5 구간 속도 (m/s)

# 곡선 구간 (1-3)
speed_c1 = 35.0   # 곡선1 구간 속도 (m/s)
speed_c2 = 50.0   # 곡선2 구간 속도 (m/s)
speed_c3 = 100.0   # 곡선3 구간 속도 (m/s)

# 후진 구간 (1-2)
speed_r1 = -50.0  # 후진1 구간 속도 (m/s, 음수)
speed_r2 = -30.0  # 후진2 구간 속도 (m/s, 음수)

current_speed = 30.0    # 현재 속도 (초기값)

# 후진 구간 조향각 계수
reverse_steering_gain = 10.0



# 구간 설정: [인덱스, 구간타입, 버전] 형태로 설정
# 구간타입: 's' (straight), 'c' (curve), 'r' (reverse)
# 버전: 1-5 (직선), 1-3 (곡선), 1-2 (후진)
# 버전별 세그먼트 설정(필요시 각 버전에 맞게 수정하세요)
segment_configs = {
    'ver1': [[15,'s',5], [16, 's', 4], [29, 's', 2], [117, 'c', 3],[138, 'c', 3], [159, 's', 4], [273, 'c', 3], [294, 's', 3], [319, 'c', 3],
             [337, 's', 4], [339, 's', 1], [380, 'r', 1], [401, 'c', 2], #T
             [413, 's', 3], [447, 'c', 3], [451, 's' ,4], [471, 'c',3], [474, 's',4], [491,'c', 3], [501, 's', 4], [519, 'c', 3], [533, 's', 4], [548, 'c', 3], [556, 's', 4], [575, 'c', 3], [601, 's', 5], [619, 'c', 3], [628, 's', 4], [646, 'c', 2],
             [683, 'r', 1],[740,'c', 2], #ll
             [774, 'c', 3], [783, 's', 4]],

    'ver2': [[15,'s',5], [16, 's', 4], [29, 's', 2], [117, 'c', 3], [138, 'c', 3], [159, 's', 4], [273, 'c', 3], [294, 's', 3], [319, 'c', 3],
             [337, 's', 4], [339, 's', 1], [373, 'r', 1], [401, 'c', 2], #T
             [413, 's', 3], [447, 'c', 3], [451, 's' ,4], [471, 'c',3], [474, 's',4], [491,'c', 3], [501, 's', 4], [519, 'c', 3], [533, 's', 4], [548, 'c', 3], [556, 's', 4], [575, 'c', 3], [601, 's', 5], [619, 'c', 3], [628, 's', 4], [646, 'c', 2],
             [683, 'r',1],[740,'c', 2], #ll
             [774, 'c', 3], [783, 's', 4]],  # 초기값: ver1과 동일
    
    'ver3': [[15,'s',5], [16, 's', 4], [29, 's', 2], [117, 'c', 3], [138, 'c', 3], [159, 's', 4], [273, 'c', 3], [294, 's', 3], [319, 'c', 3],
             [337, 's', 4], [339, 's', 1], [373, 'r', 1], [401, 'c', 2], #T
             [413, 's', 3], [447, 'c', 3], [451, 's' ,4], [471, 'c',3], [474, 's',4], [491,'c', 3], [501, 's', 4], [519, 'c', 3], [533, 's', 4], [548, 'c', 3], [556, 's', 4], [575, 'c', 3], [601, 's', 5], [619, 'c', 3], [628, 's', 4], [646, 'c', 2],
             [659, 'c', 1], [706, 'r',1], [740,'c', 2], #ll 바꿈
             [774, 'c', 3], [783, 's', 4]],  # 초기값: ver1과 동일
}


# 구간 전환 거리 임계값 (다음 구간 타입별)
segment_transition_distances = {
    'straight_to_curve': 4.0,    # 직선 → 곡선
    'straight_to_reverse': 0.2,  # 직선 → 후진

    'curve_to_straight': 4.0,    # 곡선 → 직선
    'curve_to_reverse': 0.2,     # 곡선 → 후진

    'reverse_to_straight': 0.2,  # 후진 → 직선
    'reverse_to_curve': 0.2,     # 후진 → 곡선

    'straight_to_straight': 4.0, # 직선 → 직선
    'curve_to_curve': 2.0,       # 곡선 → 곡선
    'reverse_to_reverse': 0.2    # 후진 → 후진
}

# Waypoint별 출력 유지 기능 설정 (버전별)
# [waypoint_index, output_speed, wait_time, distance_threshold]
waypoint_wait_configs = {
    'ver1': [
        # 예시: [10, 0.0, 4.0, 3.0]
        # [25, 22.0, 2.0, 2.0]
        [16, 20.0, 5, 0.5],
        [380, 0.0, 2, 0.2], #T 멈추기
        [683, 0.0, 2, 0.2] #ll 멈추기
    ],
    'ver2': [
        [16, 20.0, 5, 0.5],
        [373, 0.0, 2, 0.2], #T 멈추기
        [683, 0.0, 2, 0.2] #ll 멈추기
    ],
    'ver3': [
        [16, 20.0, 5, 0.5],
        [706, 0.0, 2, 0.2] #ll 멈추기 바꿈
    ],
}

# # 구간 설정: [인덱스, 구간타입, 버전] 형태로 설정
# # 구간타입: 's' (straight), 'c' (curve), 'r' (reverse)
# # 버전: 1-5 (직선), 1-3 (곡선), 1-2 (후진)
# # 버전별 세그먼트 설정(필요시 각 버전에 맞게 수정하세요)
# segment_configs = {
#     'ver1': [[15,'s',4], [16, 's', 3], [29, 's', 2], [117, 'c', 3], [121, 's', 4], [136, 'c', 3], [159, 's', 4], [271, 'c', 3], [294, 's', 3], [317, 'c', 3],
#              [337, 's', 4], [339, 's', 1], [380, 'r', 1], [401, 'c', 2], #T
#              [413, 's', 3], [445, 'c', 3], [451, 's' ,4], [469, 'c',3], [474, 's',4], [489,'c', 3], [501, 's', 4], [517, 'c', 3], [533, 's', 4], [546, 'c', 3], [556, 's', 4], [573, 'c', 3], [601, 's', 4], [617, 'c', 3], [628, 's', 4], 
#              [647, 'c', 2], [713, 'r',1], [764,'c', 2], #ll
#              [798, 'c', 3], [807, 's', 4]],

#     'ver2': [[15,'s',4], [16, 's', 3], [29, 's', 2], [117, 'c', 3], [121, 's', 4], [136, 'c', 3], [159, 's', 4], [271, 'c', 3], [294, 's', 3], [317, 'c', 3],
#              [337, 's', 4], [339, 's', 1], [373, 'r', 1], [401, 'c', 2], #T
#              [413, 's', 3], [445, 'c', 3], [451, 's' ,4], [469, 'c',3], [474, 's',4], [489,'c', 3], [501, 's', 4], [517, 'c', 3], [533, 's', 4], [546, 'c', 3], [556, 's', 4], [573, 'c', 3], [601, 's', 4], [617, 'c', 3], [628, 's', 4], 
#              [646, 'c', 2], [713, 'r',1], [764,'c', 2], #ll
#              [798, 'c', 3], [807, 's', 4]],  # 초기값: ver1과 동일
    
#     'ver3': [[15,'s',4], [16, 's', 3], [29, 's', 2], [117, 'c', 3], [121, 's', 4], [136, 'c', 3], [159, 's', 4], [271, 'c', 3], [294, 's', 3], [317, 'c', 3],
#              [337, 's', 4], [339, 's', 1], [373, 'r', 1], [401, 'c', 2], #T
#              [413, 's', 3], [445, 'c', 3], [451, 's' ,4], [469, 'c',3], [474, 's',4], [489,'c', 3], [501, 's', 4], [517, 'c', 3], [533, 's', 4], [546, 'c', 3], [556, 's', 4], [573, 'c', 3], [601, 's', 4], [617, 'c', 3], [628, 's', 4], 
#              [646, 'c', 2], [665, 'c', 1], [698, 'r',1], [764,'c', 2], #ll
#              [798, 'c', 3], [807, 's', 4]],  # 초기값: ver1과 동일
# }


# # 구간 전환 거리 임계값 (다음 구간 타입별)
# segment_transition_distances = {
#     'straight_to_curve': 1.0,    # 직선 → 곡선
#     'straight_to_reverse': 0.2,  # 직선 → 후진

#     'curve_to_straight': 0.5,    # 곡선 → 직선
#     'curve_to_reverse': 0.2,     # 곡선 → 후진

#     'reverse_to_straight': 0.2,  # 후진 → 직선
#     'reverse_to_curve': 0.2,     # 후진 → 곡선

#     'straight_to_straight': 1.0, # 직선 → 직선
#     'curve_to_curve': 0.5,       # 곡선 → 곡선
#     'reverse_to_reverse': 0.2    # 후진 → 후진
# }

# # Waypoint별 출력 유지 기능 설정 (버전별)
# # [waypoint_index, output_speed, wait_time, distance_threshold]
# waypoint_wait_configs = {
#     'ver1': [
#         # 예시: [10, 0.0, 4.0, 3.0]
#         # [25, 22.0, 2.0, 2.0]
#         [16, 22.0, 5, 0.5],
#         [323, 0.0, 2, 0.3], #T 확인
#         [646, 0.0, 3, 0.3] #ll 확인
#         [713] #ll 멈추기
#     ],
#     'ver2': [
#         [16, 22.0, 5, 0.5],
#         [323, 0.0, 2, 0.3],
#         [646, 0.0, 3, 0.3],
#         [713] #ll 멈추기
#     ],
#     'ver3': [
#         [16, 22.0, 5, 0.5],
#         [323, 0.0, 2, 0.3],
#         [646, 0.0, 3, 0.3],
#         [698] #ll 멈추기
#     ],
# }

# 라이다 장애물에 따른 경로 스위칭 설정
route_switch_config = {
    'wp_index_A': 323,  
    'wp_index_B': 646,  
    'distance_threshold': 0.6,
    'sample_required': 5,  # 판단에 필요한 최소 True 샘플 수
    'stop_time': 2.0,      # 정지하여 샘플 수집 및 전환/재생성에 할당할 시간(초)
    'left_topic': '/lidar_obstacle_info_left',
    'right_topic': '/lidar_obstacle_info_right',
     'waypoint_files': {
         'ver1': '/home/jh/ros2_workspace/src/waypoints/full_wp_ver1.txt',
        'ver2': '/home/jh/ros2_workspace/src/waypoints/full_wp_ver2.txt',
         'ver3': '/home/jh/ros2_workspace/src/waypoints/full_wp_ver3.txt'
    }
}

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
    
    # 유효성 검사: 인덱스가 waypoint 범위 내(0 <= idx < total_waypoints)에 있는지 확인
    valid_config = []
    for config_item in sorted_config:
        if len(config_item) == 3:
            idx, seg_type, version = config_item
        else:
            idx, seg_type = config_item
            version = 1  # 기본 버전
        
        if 0 <= idx < total_waypoints and seg_type in type_mapping:
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
        # 경계 클램핑: end_idx가 총 길이를 넘지 않도록 조정
        if total_waypoints > 0:
            end_idx = max(0, min(end_idx, total_waypoints - 1))
        # start_idx가 end_idx를 넘으면 건너뜀
        if start_idx > end_idx:
            start_idx = min(start_idx, total_waypoints - 1)
            if start_idx > end_idx:
                continue
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
    if segments and total_waypoints > 0 and segments[-1]['end_index'] < total_waypoints - 1:
        # 마지막 구간을 전체 끝까지 확장
        segments[-1]['end_index'] = total_waypoints - 1
        segments[-1]['waypoint_end'] = total_waypoints - 1
    
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
            version = segment.get('version', 1)
            
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
                'version': version,
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
    """구간별 적응적 Pure Pursuit 알고리즘 (직선-곡선-후진 지원)"""
    global current_lookahead_distance, current_speed
    closest_point = None
    v = current_speed
    
    is_reverse_segment = False

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
    
    # 후진 구간에서는 조향각도에 음수 적용 및 계수 곱하기
    if is_reverse_segment:
        desired_steering_angle = -desired_steering_angle * reverse_steering_gain
    
    return v, float(desired_steering_angle*180/math.pi), index # degree 단위로 변환

class UTMPurePursuit(Node):
    def __init__(self):
        super().__init__('utm_pure_pursuit')
        
        # 로봇 상태
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # IMU 데이터 수신 상태 플래그 추가
        self.imu_data_received = False

        # 현재 경로 버전 추적 (초기 파일명에서 유추 또는 기본 ver1)
        self.current_route_version = 'ver1'

        # Waypoint 파일 경로 파라미터 설정
        self.declare_parameter('waypoint_ver1_path', '/home/jh/ros2_workspace/src/remapped_utmcoordinates_ver1.txt')
        self.declare_parameter('waypoint_ver2_path', '/home/jh/ros2_workspace/src/remapped_utmcoordinates_ver2.txt')
        self.declare_parameter('waypoint_ver3_path', '/home/jh/ros2_workspace/src/remapped_utmcoordinates_ver3.txt')

        # Waypoint 파일 경로: 파라미터에서 로드
        waypoint_files = {
            'ver1': self.get_parameter('waypoint_ver1_path').value,
            'ver2': self.get_parameter('waypoint_ver2_path').value,
            'ver3': self.get_parameter('waypoint_ver3_path').value
        }

        self.waypoints_file_path = waypoint_files.get(self.current_route_version, waypoint_files.get('ver1'))

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
        self.last_path_index = -1       # 마지막으로 로깅한 path 인덱스
        self.first_odometry_received = False  # 첫 번째 GPS 데이터 수신 여부
        self.global_path_generated = False  # 전체 경로 생성 완료 여부
        self.current_waypoint_index = 0  # 현재 waypoint 인덱스
        
        # Waypoint별 출력 유지 기능 관련 변수들
        self.is_waypoint_waiting = False  # waypoint 대기 중인지 여부
        self.waypoint_wait_start_time = 0.0  # waypoint 대기 시작 시간
        self.waypoint_wait_index = -1  # 대기 중인 waypoint 인덱스
        self.waypoint_wait_speed = 0.0  # 대기 중인 waypoint의 출력 속도
        self.waypoint_wait_duration = 0.0  # 대기 시간
        self.waypoint_wait_completed = set()  # 완료된 waypoint 대기 인덱스들
        
        # 라이다 장애물 감지 상태 및 경로 스위칭 관련 변수들
        self.left_obstacle_detected = False
        self.right_obstacle_detected = False
        self.route_switch_done_A = False
        self.route_switch_done_B = False
        # 전환 대기 상태 및 샘플 집계 변수
        self.route_switch_state = 'idle'  # 'idle' | 'A_pending' | 'B_pending'
        self.route_switch_start_time = 0.0
        self.left_true_count = 0
        self.left_total_count = 0
        self.right_true_count = 0
        self.right_total_count = 0
        
        # 구간 설정 (인덱스별 타입 지정) - 현재 버전에 따른 세그먼트 적용
        try:
            self.segment_config = segment_configs.get(self.current_route_version, [])
        except NameError:
            # 구버전 호환: segment_configs 미정의 시 빈 설정
            self.segment_config = []
        
        # IMU 보정각도 로드
        self.imu_calibration_angle = self.load_imu_calibration_angle()
        
        # ROS2 인터페이스 설정 (control.py와 동일한 구조)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.path_publisher = self.create_publisher(PoseArray, 'interpolated_path', 10)
        
        # 구간 정보 발행을 위한 String 퍼블리셔 추가
        self.segments_publisher = self.create_publisher(String, 'path_segments', 10)
        
        # 현재 waypoint 발행을 위한 String 퍼블리셔 추가
        self.current_waypoint_publisher = self.create_publisher(String, 'waypoint_zone_info', 10)
        
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
        
        # 라이다 Left/Right 장애물 Bool 구독
        try:
            self.left_sub = self.create_subscription(
                Bool,
                route_switch_config['left_topic'],
                self.left_obstacle_callback,
                10
            )
            self.right_sub = self.create_subscription(
                Bool,
                route_switch_config['right_topic'],
                self.right_obstacle_callback,
                10
            )
        except Exception as e:
            self.get_logger().warn(f"라이다 토픽 구독 설정 실패: {e}")
        
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
        self.get_logger().info(f"🔄 후진 구간 조향각 계수: {reverse_steering_gain}")
        self.get_logger().info("🔄 구간 전환 거리 설정:")
        for key, distance in segment_transition_distances.items():
            self.get_logger().info(f"   {key}: {distance}m")
        # 버전별 Waypoint 대기 설정 적용
        try:
            self.waypoint_wait_config = waypoint_wait_configs.get(self.current_route_version, [])
        except NameError:
            self.waypoint_wait_config = []

        self.get_logger().info("🛑 Waypoint별 출력 유지 기능이 활성화되었습니다:")
        if self.waypoint_wait_config:
            for i, config in enumerate(self.waypoint_wait_config):
                if len(config) >= 4:
                    waypoint_idx, output_speed, wait_time, distance_threshold = config
                    self.get_logger().info(f"   설정 {i+1}: Waypoint {waypoint_idx} - 출력: {output_speed}m/s, 대기: {wait_time}초, 거리: {distance_threshold}m")
        else:
            self.get_logger().info("   현재 설정된 waypoint 대기 없음")
        
        # 라이다 기반 경로 스위칭 설정 로그
        self.get_logger().info("🔁 라이다 장애물 기반 경로 스위칭 설정:")
        self.get_logger().info(f"   A index: {route_switch_config['wp_index_A']}, B index: {route_switch_config['wp_index_B']}")
        self.get_logger().info(f"   거리 임계값: {route_switch_config['distance_threshold']} m, 샘플임계: {route_switch_config['sample_required']}개, 정지시간: {route_switch_config['stop_time']}초")
        self.get_logger().info(f"   Left 토픽: {route_switch_config['left_topic']}")
        self.get_logger().info(f"   Right 토픽: {route_switch_config['right_topic']}")
        self.get_logger().info(f"   경로 파일들: {route_switch_config['waypoint_files']}")
    
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
        self.get_logger().info(f"   구간 설정: {self.segment_config}")
        
        # 수동 구간 생성 (waypoint만 사용)
        # waypoint를 그대로 경로로 사용
        full_path_waypoints = self.waypoints.copy()
        
        # 구간 설정은 그대로 사용 (현재 위치 추가하지 않음)
        self.get_logger().info(f"   구간 설정: {self.segment_config}")
        
        waypoint_segments = create_manual_segments(full_path_waypoints, self.segment_config)
        
        # waypoint로부터 경로 생성
        self.path, self.path_segments = create_path_from_waypoints(full_path_waypoints, waypoint_segments)
        
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

        # 경로를 ROS2 토픽으로 발행
        self.publish_interpolated_path()
        
        # 구간 정보를 ROS2 토픽으로 발행
        self.publish_path_segments()
        
        self.get_logger().info("✅ 구간별 경로 생성 완료!")
        self.get_logger().info(f"   경로점: {len(self.path)}개 (waypoint 그대로 사용)")
        self.get_logger().info(f"   총 거리: {total_distance:.2f}m")
        self.get_logger().info(f"   직선 구간: {straight_segments}개")
        self.get_logger().info(f"   곡선 구간: {curve_segments}개")
        self.get_logger().info(f"   후진 구간: {reverse_segments}개")
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
        
        # 초기 구간 정보 디버깅
        self.get_logger().info("🔍 초기 구간 디버깅:")
        self.get_logger().info(f"   현재 구간 인덱스: {self.current_segment_index}")
        if self.path_segments:
            initial_segment = self.path_segments[0]
            self.get_logger().info(f"   첫 번째 구간: {initial_segment['type']}{initial_segment.get('version', 1)}")
            self.get_logger().info(f"   첫 번째 구간 범위: {initial_segment['start_index']}-{initial_segment['end_index']}")
            self.get_logger().info(f"   현재 path 인덱스: {self.i}")
        
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
    
    def find_current_waypoint(self):
        """현재 위치에서 가장 가까운 waypoint 인덱스를 찾는다."""
        if not self.waypoints:
            return -1
        
        min_distance = float('inf')
        closest_index = 0
        
        for i, (wx, wy) in enumerate(self.waypoints):
            distance = math.hypot(self.x - wx, self.y - wy)
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        
        return closest_index
    
    def publish_current_waypoint(self):
        """현재 waypoint index를 String 형태로 발행"""
        if not self.waypoints:
            return
        
        current_wp_index = self.find_current_waypoint()
        if current_wp_index < 0:
            return
        
        msg = String()
        msg.data = str(current_wp_index)
        self.current_waypoint_publisher.publish(msg)
        
    def left_obstacle_callback(self, msg: Bool):
        self.left_obstacle_detected = bool(msg.data)
    
    def right_obstacle_callback(self, msg: Bool):
        self.right_obstacle_detected = bool(msg.data)
        
    
    def find_initial_path_index(self):
        """현재 위치와 진행 방향을 기준으로 초기 path 인덱스를 결정한다.

        - 로봇 진행 방향(heading) 앞쪽(dot>=0)에 있는 waypoint들 중에서 가장 가까운 점 선택
        - 만약 모두 뒤쪽이라면, 가장 가까운 점의 다음 인덱스를 선택(마지막이면 그대로 사용)
        """
        if not self.path:
            return 0
        heading_x = math.cos(self.yaw)
        heading_y = math.sin(self.yaw)
        best_index = -1
        best_distance = float('inf')
        # 진행 방향 전방에 있는 점 중 최단거리 선택
        for idx, (px, py) in enumerate(self.path):
            vec_x = px - self.x
            vec_y = py - self.y
            projection = heading_x * vec_x + heading_y * vec_y
            if projection >= 0.0:
                dist = math.hypot(vec_x, vec_y)
                if dist < best_distance:
                    best_distance = dist
                    best_index = idx
        # 전방 점이 하나도 없으면: 가장 가까운 점의 다음 인덱스를 선택
        if best_index == -1:
            nearest_idx = min(range(len(self.path)), key=lambda i: math.hypot(self.path[i][0] - self.x, self.path[i][1] - self.y))
            if nearest_idx < len(self.path) - 1:
                best_index = nearest_idx + 1
            else:
                best_index = nearest_idx
        return best_index

    def update_segment_index_from_path_index(self):
        """self.i가 속한 구간을 찾아 self.current_segment_index를 동기화한다."""
        if not self.path_segments:
            self.current_segment_index = 0
            return
        for seg_idx, seg in enumerate(self.path_segments):
            if seg['start_index'] <= self.i <= seg['end_index']:
                self.current_segment_index = seg_idx
                return
        # 해당 없으면 처음으로 고정
        self.current_segment_index = 0
        
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
        transition_distance = 0.0
        
        # 디버깅용 로그 (5초마다)
        if hasattr(self, '_last_debug_time'):
            if time.time() - self._last_debug_time > 5.0:
                self.get_logger().info(f"🔍 구간 전환 디버그: 현재 구간 {self.current_segment_index}/{len(self.path_segments)-1}")
                self.get_logger().info(f"   현재 타입: {current_type}, 다음 타입: {next_segment_type}")
                self.get_logger().info(f"   구간 끝까지 거리: {distance_to_segment_end:.2f}m")
                if next_segment_type:
                    self.get_logger().info(f"   전환 조건: {current_type} → {next_segment_type}")
                self._last_debug_time = time.time()
        else:
            self._last_debug_time = time.time()
        
        # 다음 구간이 없으면 전환하지 않음
        if next_segment_type is None:
            return current_segment
        
        # 구간 전환 거리 임계값 결정
        transition_key = f"{current_type}_to_{next_segment_type}"
        if transition_key in segment_transition_distances:
            transition_distance = segment_transition_distances[transition_key]
        else:
            # 기본값 (혹시 모를 경우)
            transition_distance = 0.5
        
        # 거리 기반 구간 전환 판정
        should_transition = distance_to_segment_end <= transition_distance
        
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
            self.get_logger().info(f"   현재 위치: ({self.x:.3f}, {self.y:.3f})")
            self.get_logger().info(f"   현재 구간 끝점: ({segment_end_point[0]:.3f}, {segment_end_point[1]:.3f})")
            self.get_logger().info(f"   거리: {distance_to_segment_end:.3f}m, 전환 기준: {transition_distance:.3f}m")
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

        if segment_type == 'straight':
            # 직선 구간 - version별 파라미터 사용
            version = current_segment.get('version', 1)
            speeds = [speed_s1, speed_s2, speed_s3, speed_s4, speed_s5]
            lookaheads = [lookahead_distance_s1, lookahead_distance_s2, lookahead_distance_s3, 
                         lookahead_distance_s4, lookahead_distance_s5]
            target_speed = speeds[min(version - 1, 4)]
            target_lookahead = lookaheads[min(version - 1, 4)]
            
        elif segment_type == 'curve':
            # 곡선 구간 - version별 파라미터 사용
            version = current_segment.get('version', 1)
            speeds = [speed_c1, speed_c2, speed_c3]
            lookaheads = [lookahead_distance_c1, lookahead_distance_c2, lookahead_distance_c3]
            target_speed = speeds[min(version - 1, 2)]
            target_lookahead = lookaheads[min(version - 1, 2)]
        
        else:  # reverse
            # 후진 구간 - version별 파라미터 사용
            version = current_segment.get('version', 1)
            speeds = [speed_r1, speed_r2]
            lookaheads = [lookahead_distance_r1, lookahead_distance_r2]
            target_speed = speeds[min(version - 1, 1)]
            target_lookahead = lookaheads[min(version - 1, 1)]
        
        # 속도와 전방주시거리 모두 즉시 변경
        current_speed = target_speed
        current_lookahead_distance = target_lookahead
    
    def should_start_waypoint_waiting(self):
        """waypoint별 출력 유지를 시작해야 하는지 확인 (거리 기반)"""
        if self.is_waypoint_waiting:
            return None  # 이미 대기 중이면 시작하지 않음
        
        # waypoint 대기 설정과의 거리 확인
        for config in self.waypoint_wait_config:
            if len(config) < 4:
                continue  # 설정이 올바르지 않으면 무시
                
            waypoint_idx, output_speed, wait_time, distance_threshold = config
            
            if waypoint_idx in self.waypoint_wait_completed:
                continue  # 이미 완료된 waypoint는 무시
            
            if waypoint_idx < len(self.waypoints):
                waypoint = self.waypoints[waypoint_idx]
                distance = math.hypot(self.x - waypoint[0], self.y - waypoint[1])
                
                # 디버깅 로그 (5초마다)
                if hasattr(self, '_last_waypoint_debug_time'):
                    if time.time() - self._last_waypoint_debug_time > 5.0:
                        self.get_logger().info(f"🔍 Waypoint 대기 디버그: Waypoint {waypoint_idx}")
                        self.get_logger().info(f"   현재 위치: ({self.x:.2f}, {self.y:.2f})")
                        self.get_logger().info(f"   Waypoint 위치: ({waypoint[0]:.2f}, {waypoint[1]:.2f})")
                        self.get_logger().info(f"   거리: {distance:.2f}m, 임계값: {distance_threshold}m")
                        self.get_logger().info(f"   출력 속도: {output_speed}m/s, 대기 시간: {wait_time}초")
                        self._last_waypoint_debug_time = time.time()
                else:
                    self._last_waypoint_debug_time = time.time()
                
                if distance <= distance_threshold:
                    return config  # 해당 waypoint 설정 반환
        
        return None  # 대기할 waypoint 없음
    
    def start_waypoint_waiting(self, config):
        """waypoint별 출력 유지 시작"""
        waypoint_idx, output_speed, wait_time, distance_threshold = config
        
        self.is_waypoint_waiting = True
        self.waypoint_wait_start_time = time.time()
        self.waypoint_wait_index = waypoint_idx
        self.waypoint_wait_speed = output_speed
        self.waypoint_wait_duration = wait_time
        
        waypoint = self.waypoints[waypoint_idx]
        distance = math.hypot(self.x - waypoint[0], self.y - waypoint[1])
        self.get_logger().info(f"🛑 Waypoint 출력 유지 시작: Waypoint {waypoint_idx} (거리: {distance:.2f}m)에서 {wait_time}초 유지")
        self.get_logger().info(f"   출력 속도: {output_speed} m/s")
    
    def handle_waypoint_waiting(self):
        """waypoint별 출력 유지 중 처리"""
        current_time = time.time()
        elapsed_time = current_time - self.waypoint_wait_start_time
        
        # 대기 시간이 지났는지 확인
        if elapsed_time >= self.waypoint_wait_duration:
            # 대기 완료 - 해당 waypoint를 완료 목록에 추가
            completed_waypoint = self.waypoint_wait_index
            self.waypoint_wait_completed.add(completed_waypoint)
            
            self.is_waypoint_waiting = False
            self.waypoint_wait_start_time = 0.0
            self.waypoint_wait_index = -1
            self.waypoint_wait_speed = 0.0
            self.waypoint_wait_duration = 0.0
            self.get_logger().info(f"✅ Waypoint 출력 유지 완료: Waypoint {completed_waypoint}에서 {elapsed_time:.1f}초 유지 후 정상 주행 재개")
            # 정상 주행을 위해 빈 twist 반환 (다음 루프에서 Pure Pursuit 실행)
            return Twist()
        
        # 대기 중: 설정된 출력으로 유지
        twist = Twist()
        twist.linear.x = self.waypoint_wait_speed
        twist.angular.z = 0.0  # 조향각은 0으로 유지
        
        # 대기 상태 로그 (1초마다)
        if int(elapsed_time) != int(elapsed_time - 0.01):  # 1초마다 로그
            remaining_time = self.waypoint_wait_duration - elapsed_time
            self.get_logger().info(f"⏳ Waypoint 출력 유지 중: {remaining_time:.1f}초 남음 (속도: {self.waypoint_wait_speed} m/s)")
        
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
                        # 현재 위치를 기준으로 초기 path 인덱스 결정
                        try:
                            initial_index = self.find_initial_path_index()
                            self.i = int(initial_index)
                            # 해당 인덱스 기준으로 현재 구간 인덱스 동기화
                            self.update_segment_index_from_path_index()
                            self.get_logger().info(f"🎯 초기 시작 인덱스 설정: i={self.i} (좌표: ({self.path[self.i][0]:.3f}, {self.path[self.i][1]:.3f}))")
                            if 0 <= self.current_segment_index < len(self.path_segments):
                                seg = self.path_segments[self.current_segment_index]
                                seg_name = {'straight': '직선', 'curve': '곡선', 'reverse': '후진'}.get(seg['type'], seg['type'])
                                self.get_logger().info(f"   시작 구간: {seg_name}{seg.get('version', 1)} (index {seg['start_index']}~{seg['end_index']})")
                        except Exception as e:
                            self.get_logger().warn(f"초기 시작 인덱스 계산 실패: {e}")
                        self.flag = 1  # GPS 수신 완료, IMU 대기 상태
                        self.get_logger().info("📍 GPS 수신 완료! IMU 데이터 대기 중...")
                    else:
                        self.get_logger().error("경로 생성에 실패했습니다.")
        except Exception as e:
            self.get_logger().error(f"UTM 변환 실패: {e}")

    def switch_route_to(self, version_key):
        """경로 파일을 지정 버전으로 전환하고, 경로를 재생성 후 재시작한다."""
        try:
            files = route_switch_config['waypoint_files']
            if version_key not in files:
                self.get_logger().warn(f"알 수 없는 경로 버전: {version_key}")
                return False
            new_path = files[version_key]
            if not os.path.exists(new_path):
                self.get_logger().warn(f"경로 파일이 존재하지 않습니다: {new_path}")
                return False
            # 정지는 check_and_switch_route()에서 지속적으로 처리됨
            # 파일 경로 변경 및 재로드
            self.waypoints_file_path = new_path
            self.waypoints = self.load_waypoints_from_file(self.waypoints_file_path)
            if not self.waypoints:
                self.get_logger().error("경로 파일 로드 실패로 스위칭 중단")
                return False
            # 버전별 세그먼트 구성 갱신
            try:
                self.segment_config = segment_configs.get(version_key, self.segment_config)
            except NameError:
                pass

            # 경로 재생성
            if not self.generate_global_path():
                self.get_logger().error("경로 재생성 실패")
                return False
            # 현재 위치 기준 시작 인덱스 재설정
            initial_index = self.find_initial_path_index()
            self.i = int(initial_index)
            self.update_segment_index_from_path_index()
            self.current_route_version = version_key
            self.get_logger().info(f"✅ 경로 스위칭 완료 → {version_key} ({self.waypoints_file_path}) | 시작 i={self.i}")
            # 주행 재개 (IMU 데이터 수신 확인 후)
            if self.imu_data_received:
                self.flag = 2
            else:
                self.flag = 1  # IMU 대기 상태
            return True
        except Exception as e:
            self.get_logger().error(f"경로 스위칭 중 오류: {e}")
            return False

    def check_and_switch_route(self):
        """A/B 웨이포인트 근접 시 일정 시간 정지하며 n회 샘플 수집 후 경로 전환."""
        if not self.path:
            return None
        threshold = route_switch_config['distance_threshold']
        sample_required = route_switch_config['sample_required']
        stop_time = route_switch_config['stop_time']

        # 전환 대기 상태 처리
        if self.route_switch_state in ('A_pending', 'B_pending'):
            # 정지 유지
            stop_twist = Twist(); stop_twist.linear.x = 0.0; stop_twist.angular.z = 0.0
            # 샘플 집계: 타이머 주기마다 한 샘플로 간주
            if self.route_switch_state == 'A_pending':
                self.left_total_count += 1
                if self.left_obstacle_detected:
                    self.left_true_count += 1
            else:
                self.right_total_count += 1
                if self.right_obstacle_detected:
                    self.right_true_count += 1

            elapsed = time.time() - self.route_switch_start_time
            if elapsed >= stop_time:
                if self.route_switch_state == 'A_pending':
                    # 판단: Left True 샘플이 임계 이상이면 ver2, 아니면 ver1
                    target_version = 'ver2' if self.left_true_count >= sample_required else 'ver1'
                    self.get_logger().info(
                        f"🅰️ A 판정 완료: True {self.left_true_count}/{self.left_total_count} → {target_version} 전환")
                    self.route_switch_done_A = True
                    # 상태 리셋
                    self.route_switch_state = 'idle'
                    self.left_true_count = self.left_total_count = 0
                    # 경로 전환 수행
                    if target_version != self.current_route_version:
                        self.switch_route_to(target_version)
                    # 전환 후 주행 재개는 switch_route_to 내부에서 처리됨
                else:
                    # 판단: Right True 샘플이 임계 이상이면 ver3, 아니면 유지
                    do_switch = self.right_true_count >= sample_required
                    self.get_logger().info(
                        f"🅱️ B 판정 완료: True {self.right_true_count}/{self.right_total_count} → {'ver3 전환' if do_switch else '유지'}")
                    self.route_switch_done_B = True
                    # 상태 리셋
                    self.route_switch_state = 'idle'
                    self.right_true_count = self.right_total_count = 0
                    if do_switch is False and self.current_route_version != 'ver3':
                        self.switch_route_to('ver3')
                # 정지 단계 종료 후 다음 루프에서 주행으로 넘어감
                return stop_twist
            else:
                # 아직 수집 중: 계속 정지
                return stop_twist

        # 전환 대기 상태가 아닐 때: A/B 진입 조건 확인 (각 한 번만)
        wpA = route_switch_config['wp_index_A']
        if isinstance(wpA, int) and 0 <= wpA < len(self.waypoints) and not self.route_switch_done_A:
            wx, wy = self.waypoints[wpA]
            distA = math.hypot(self.x - wx, self.y - wy)
            if distA <= threshold:
                self.get_logger().info(f"🅰️ A 지점 근접({distA:.2f}m) → 정지 후 샘플 수집 시작")
                self.route_switch_state = 'A_pending'
                self.route_switch_start_time = time.time()
                self.left_true_count = 0
                self.left_total_count = 0
                t = Twist(); t.linear.x = 0.0; t.angular.z = 0.0
                return t

        wpB = route_switch_config['wp_index_B']
        if isinstance(wpB, int) and 0 <= wpB < len(self.waypoints) and not self.route_switch_done_B:
            wx, wy = self.waypoints[wpB]
            distB = math.hypot(self.x - wx, self.y - wy)
            if distB <= threshold:
                self.get_logger().info(f"🅱️ B 지점 근접({distB:.2f}m) → 정지 후 샘플 수집 시작")
                self.route_switch_state = 'B_pending'
                self.route_switch_start_time = time.time()
                self.right_true_count = 0
                self.right_total_count = 0
                t = Twist(); t.linear.x = 0.0; t.angular.z = 0.0
                return t

        return None
    
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
        
        # IMU 데이터 수신 완료 플래그 설정
        if not self.imu_data_received:
            self.imu_data_received = True
            self.get_logger().info(f"🧭 IMU 데이터 수신 완료: yaw = {math.degrees(self.yaw):.1f}°")
            
            # GPS 데이터도 수신되었고 경로가 생성되었다면 flag = 2 설정
            if self.first_odometry_received and self.global_path_generated and self.flag == 1:
                self.flag = 2
                self.get_logger().info("🎯 IMU + GPS 데이터 모두 수신 완료! 경로 추적을 시작합니다!")
    
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
            
            # 현재 waypoint 발행 (거리 기반)
            self.publish_current_waypoint()
            
            # 라이다 기반 경로 스위칭 우선 처리
            switch_twist = self.check_and_switch_route()
            if switch_twist is not None:
                self.publisher.publish(switch_twist)
                return

            # Waypoint별 출력 유지 로직 처리
            twist = Twist()
            
            # Waypoint 대기 중인 경우
            if self.is_waypoint_waiting:
                twist = self.handle_waypoint_waiting()
            else:
                # Waypoint 대기 체크 (거리 기반) - 대기 중이 아닐 때만
                waypoint_config = self.should_start_waypoint_waiting()
                if waypoint_config is not None:
                    self.start_waypoint_waiting(waypoint_config)
                    twist = self.handle_waypoint_waiting()  # 대기 시작 후 즉시 대기 처리
                else:
                    # 구간별 적응적 Pure Pursuit 제어 실행
                    twist.linear.x, twist.angular.z, self.i = pure_pursuit(
                        self.x, self.y, self.yaw, self.path, self.i, current_segment
                    )
                    
                    # Path index 변경 시 디버깅 로그
                    if self.i != self.last_path_index:
                        if current_segment is not None:
                            seg_type = current_segment['type']
                            seg_version = current_segment.get('version', 1)
                            type_names = {'straight': '직선', 'curve': '곡선', 'reverse': '후진'}
                            seg_type_name = type_names.get(seg_type, seg_type)
                            self.get_logger().info(f"🎯 Path Index 변경: {self.i} (구간: {seg_type_name}{seg_version}, 좌표: ({self.path[self.i][0]:.3f}, {self.path[self.i][1]:.3f}))")
                        else:
                            self.get_logger().info(f"🎯 Path Index 변경: {self.i} (구간 정보 없음, 좌표: ({self.path[self.i][0]:.3f}, {self.path[self.i][1]:.3f}))")
                        self.last_path_index = self.i

            distance_to_path_end = math.hypot(
                self.x - self.path[-1][0], 
                self.y - self.path[-1][1]
            )

            # 경로 완료 판정 (마지막 path 도달)
            if (self.i >= len(self.path) - 10) and distance_to_path_end < 0.3:
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
