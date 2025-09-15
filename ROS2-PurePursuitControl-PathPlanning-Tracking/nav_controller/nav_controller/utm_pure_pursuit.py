import rclpy
from rclpy.node import Node
import numpy as np
import math
import scipy.interpolate as si
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
import time
import utm

# Pure Pursuit 파라미터 (구간별 적응적 제어)
# 전방주시거리 설정
lookahead_distance_straight = 2.0  # 직선 구간 전방주시거리 (미터)
lookahead_distance_curve = 0.5     # 곡선 구간 전방주시거리 (미터)
current_lookahead_distance = 0.3   # 현재 전방주시거리 (초기값)

# 속도 설정
speed_straight = 30.0 #직선 구간 속도 (m/s)
speed_curve = 20.0   # 곡선 구간 속도 (m/s)
current_speed = 30.0   # 현재 속도 (초기값)

# 보간 밀도 설정
interpolation_density_straight = 5   # 직선 구간: 1m당 점의 개수
interpolation_density_curve = 20     # 곡선 구간: 1m당 점의 개수

# 곡률 임계값 (이 값보다 크면 곡선으로 판단)
curvature_threshold = 0.1  # 1/m

# 구간 전환 거리 임계값
segment_transition_threshold = 0.4  # 전방주시거리에 추가할 거리 (m)

def euler_from_quaternion(x, y, z, w):
    """쿼터니언에서 yaw 각도 추출 (control.py와 동일)"""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def calculate_curvature(p1, p2, p3):
    """세 점을 이용하여 곡률 계산"""
    try:
        # 벡터 계산
        v1 = np.array([p2[0] - p1[0], p2[1] - p1[1]])
        v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]])
        
        # 외적 계산 (2D에서는 z 성분만)
        cross_product = v1[0] * v2[1] - v1[1] * v2[0]
        
        # 벡터의 크기
        v1_norm = np.linalg.norm(v1)
        v2_norm = np.linalg.norm(v2)
        
        if v1_norm == 0 or v2_norm == 0:
            return 0.0
        
        # 곡률 계산
        curvature = abs(cross_product) / (v1_norm * v2_norm)
        return curvature
    except:
        return 0.0

def adaptive_bspline_planning(array):
    """구간별 적응적 B-Spline 경로 스무딩"""
    try:
        array = np.array(array)
        if len(array) < 3:
            return array.tolist(), []
        
        x = array[:, 0]
        y = array[:, 1]
        
        # 각 구간의 곡률 계산
        curvatures = []
        for i in range(1, len(array) - 1):
            curvature = calculate_curvature(array[i-1], array[i], array[i+1])
            curvatures.append(curvature)
        
        # 구간별 보간 밀도 결정
        total_distance = 0.0
        segment_distances = []
        
        for i in range(len(array) - 1):
            dist = math.sqrt((array[i+1][0] - array[i][0])**2 + (array[i+1][1] - array[i][1])**2)
            segment_distances.append(dist)
            total_distance += dist
        
        # 구간별 보간 점 수 계산
        total_interpolation_points = 0
        for i, dist in enumerate(segment_distances):
            if i < len(curvatures):
                if curvatures[i] > curvature_threshold:
                    # 곡선 구간
                    points = max(10, int(dist * interpolation_density_curve))
                else:
                    # 직선 구간
                    points = max(5, int(dist * interpolation_density_straight))
            else:
                # 마지막 구간
                points = max(5, int(dist * interpolation_density_straight))
            
            total_interpolation_points += points
        
        # B-Spline 스무딩
        N = 2
        t = range(len(x))
        x_tup = si.splrep(t, x, k=N)
        y_tup = si.splrep(t, y, k=N)

        x_list = list(x_tup)
        xl = x.tolist()
        x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

        y_list = list(y_tup)
        yl = y.tolist()
        y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

        ipl_t = np.linspace(0.0, len(x) - 1, total_interpolation_points)
        rx = si.splev(ipl_t, x_list)
        ry = si.splev(ipl_t, y_list)
        path = [(rx[i], ry[i]) for i in range(len(rx))]
        
        return path, curvatures
    except Exception as e:
        print(f"보간 오류: {e}")
        return array.tolist(), []

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
        
        # 현재 위치에서 구간 끝점까지의 거리
        distance_to_segment_end = math.hypot(
            current_x - segment_end_point[0], 
            current_y - segment_end_point[1]
        )
        
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
            lookahead = lookahead_distance_curve
            
            # 전방주시거리보다 먼 경로점 찾기
            for i in range(index, len(path)):
                x = path[i][0]
                y = path[i][1]
                distance = math.hypot(current_x - x, current_y - y)
                if lookahead < distance:
                    closest_point = (x, y)
                    index = i
                    break
    
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
    
    return v, float(desired_steering_angle*180/math.pi), index # degree 단위로 변환

class UTMPurePursuit(Node):
    def __init__(self):
        super().__init__('utm_pure_pursuit')
        
        # 로봇 상태
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Waypoint 설정 (control.py의 goal과 유사)

        self.waypoints = [
            (329983.719725, 4123210.415129),  # 첫 번째 waypoint
            (329977.396338, 4123210.620894),   # 두 번째 waypoint
            (329964.442297, 4123208.130461),   # 세 번째 waypoint
            (329955.384840, 4123213.997123)   # 네 번째 waypoint
        ]
        
        
        # control.py와 동일한 flag 시스템
        self.flag = 0  # 0: 대기, 1: 경로 생성, 2: 추적 중
        self.path = []
        self.path_curvatures = []  # 경로의 곡률 정보
        self.path_segments = []    # 경로의 구간 정보
        self.i = 0  # 경로 인덱스
        self.current_segment_index = 0  # 현재 구간 인덱스
        self.first_odometry_received = False  # 첫 번째 GPS 데이터 수신 여부
        self.global_path_generated = False  # 전체 경로 생성 완료 여부
        self.current_waypoint_index = 0  # 현재 waypoint 인덱스
        
        # IMU 보정각도 로드
        self.imu_calibration_angle = self.load_imu_calibration_angle()
        
        # ROS2 인터페이스 설정 (control.py와 동일한 구조)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
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
    
    def generate_global_path(self):
        """전체 waypoint를 사용하여 글로벌 경로 생성"""
        if len(self.waypoints) < 2:
            self.get_logger().error("waypoint가 2개 미만입니다. 경로 생성이 불가능합니다.")
            return False
        
        # 현재 위치를 시작점으로 하고 모든 waypoint를 포함한 경로 생성
        path_points = [(self.x, self.y)]  # 현재 위치를 시작점으로
        path_points.extend(self.waypoints)  # 모든 waypoint 추가
        
        self.get_logger().info("🗺️ 전체 경로 생성 중...")
        self.get_logger().info(f"   총 경로점: {len(path_points)}개")
        self.get_logger().info(f"   시작점: ({self.x:.3f}, {self.y:.3f})")
        self.get_logger().info(f"   종료점: ({self.waypoints[-1][0]:.3f}, {self.waypoints[-1][1]:.3f})")
        
        # 적응적 B-Spline으로 경로 스무딩
        total_distance = 0.0
        for i in range(len(path_points) - 1):
            dist = math.sqrt((path_points[i+1][0] - path_points[i][0])**2 + 
                           (path_points[i+1][1] - path_points[i][1])**2)
            total_distance += dist
        
        # 적응적 보간으로 경로 생성
        self.path, self.path_curvatures = adaptive_bspline_planning(path_points)
        
        # 구간 정보 생성
        self.path_segments = self.create_path_segments(self.path_curvatures)
        
        # 곡률 통계 계산
        straight_segments = sum(1 for c in self.path_curvatures if c <= curvature_threshold)
        curve_segments = sum(1 for c in self.path_curvatures if c > curvature_threshold)
        
        self.get_logger().info("✅ 적응적 경로 생성 완료!")
        self.get_logger().info(f"   스무딩된 경로점: {len(self.path)}개")
        self.get_logger().info(f"   총 거리: {total_distance:.2f}m")
        self.get_logger().info(f"   직선 구간: {straight_segments}개 (보간 밀도: {interpolation_density_straight}/m)")
        self.get_logger().info(f"   곡선 구간: {curve_segments}개 (보간 밀도: {interpolation_density_curve}/m)")
        self.get_logger().info(f"   곡률 임계값: {curvature_threshold} 1/m")
        self.get_logger().info(f"   직선 전방주시거리: {lookahead_distance_straight}m")
        self.get_logger().info(f"   곡선 전방주시거리: {lookahead_distance_curve}m")
        self.get_logger().info(f"   직선 속도: {speed_straight}m/s")
        self.get_logger().info(f"   곡선 속도: {speed_curve}m/s")
        self.get_logger().info(f"   구간 그룹: {len(self.path_segments)}개")
        
        return True
    
    def create_path_segments(self, curvatures):
        """곡률 정보를 바탕으로 연속된 구간들을 생성"""
        if not curvatures:
            return []
        
        segments = []
        current_type = 'straight' if curvatures[0] <= curvature_threshold else 'curve'
        start_index = 0
        
        for i, curvature in enumerate(curvatures):
            segment_type = 'curve' if curvature > curvature_threshold else 'straight'
            
            # 구간 타입이 바뀌면 새로운 구간 시작
            if segment_type != current_type:
                # 이전 구간 저장
                segments.append({
                    'type': current_type,
                    'start_index': start_index,
                    'end_index': i - 1,
                    'length': i - start_index
                })
                
                # 새 구간 시작
                current_type = segment_type
                start_index = i
        
        # 마지막 구간 저장
        segments.append({
            'type': current_type,
            'start_index': start_index,
            'end_index': len(curvatures) - 1,
            'length': len(curvatures) - start_index
        })
        
        return segments
    
    def update_current_segment(self):
        """현재 위치에 따라 구간 정보 업데이트"""
        if not self.path_segments or self.current_segment_index >= len(self.path_segments):
            return None
        
        current_segment = self.path_segments[self.current_segment_index]
        
        # 현재 인덱스가 구간을 벗어났는지 확인
        if self.i > current_segment['end_index']:
            # 다음 구간으로 이동
            if self.current_segment_index < len(self.path_segments) - 1:
                self.current_segment_index += 1
                new_segment = self.path_segments[self.current_segment_index]
                self.get_logger().info(f"🔄 구간 전환: {current_segment['type']} → {new_segment['type']}")
                self.get_logger().info(f"   인덱스: {self.i}, 구간: {new_segment['start_index']}-{new_segment['end_index']}")
                return new_segment
        
        return current_segment
    
    def update_speed_and_lookahead(self, current_segment):
        """구간에 따른 속도와 전방주시거리 업데이트"""
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
            # 직선 구간
            target_speed = speed_straight
            target_lookahead = lookahead_distance_straight
            
            # 구간 끝에 가까워지면 속도를 점진적으로 감소
            if self.i >= segment_end_index:
                # 구간 끝에 도달 - 곡선 속도로 전환
                target_speed = speed_curve
                target_lookahead = lookahead_distance_curve
            elif distance_to_segment_end < (lookahead_distance_curve + segment_transition_threshold):
                # 구간 끝에 가까워짐 - 속도 점진적 감소
                transition_ratio = distance_to_segment_end / (lookahead_distance_curve + segment_transition_threshold)
                target_speed = speed_curve + (speed_straight - speed_curve) * transition_ratio
        
        else:  # curve
            # 곡선 구간
            target_speed = speed_curve
            target_lookahead = lookahead_distance_curve
            
            # 곡선에서 직선으로 전환 시
            if distance_to_segment_end > (lookahead_distance_straight + segment_transition_threshold):
                # 직선 전방주시거리로 전환 (오버슈트 방지)
                target_lookahead = lookahead_distance_straight
                # 속도는 구간을 완전히 벗어날 때까지 유지
                if self.i > segment_end_index:
                    target_speed = speed_straight
        
        # 속도는 부드러운 전환, 전방주시거리는 즉시 변경
        speed_transition_rate = 0.05  # 5%씩 변화
        
        current_speed = (1 - speed_transition_rate) * current_speed + speed_transition_rate * target_speed
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
            
            # 구간별 적응적 Pure Pursuit 제어 실행
            twist = Twist()
            twist.linear.x, twist.angular.z, self.i = pure_pursuit(
                self.x, self.y, self.yaw, self.path, self.i, current_segment
            )
            
            
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