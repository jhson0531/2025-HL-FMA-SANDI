import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from interfaces_pkg.msg import PathPlanningResult, DetectionArray, MotionCommand
from .lib import decision_making_func_lib as DMFL
import subprocess
import time
import os

#---------------Variable Setting---------------
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_PATH_TOPIC_NAME = "path_planning_result"
SUB_TRAFFIC_LIGHT_TOPIC_NAME = "yolov8_traffic_light_info"
SUB_LIDAR_OBSTACLE_TOPIC_NAME = "lidar_obstacle_info_front"
SUB_WAYPOINT_ZONE_TOPIC_NAME = "waypoint_zone_info"  # waypoint 구간 정보 토픽
SUB_CMD_VEL_TOPIC_NAME = "cmd_vel"
SUB_IMU_TOPIC_NAME = "/imu/data"  # IMU 데이터 토픽
PUB_TOPIC_NAME = "topic_control_signal"

# 시스템 재시작 관련 토픽
PUB_SYSTEM_RESET_TOPIC_NAME = "system_reset_request"  # 시스템 재시작 요청 토픽
SUB_SYSTEM_RESET_STATUS_TOPIC_NAME = "system_reset_status"  # 시스템 재시작 상태 토픽

#----------------------------------------------

# 모션 플랜 발행 주기 (초) - 소수점 필요 (int형은 반영되지 않음)
TIMER = 0.01

class MotionPlanningSystemResetNode(Node):
    def __init__(self):
        super().__init__('motion_planner_system_reset_node')

        # 토픽 이름 설정
        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.sub_path_topic = self.declare_parameter('sub_lane_topic', SUB_PATH_TOPIC_NAME).value
        self.sub_traffic_light_topic = self.declare_parameter('sub_traffic_light_topic', SUB_TRAFFIC_LIGHT_TOPIC_NAME).value
        self.sub_lidar_obstacle_topic = self.declare_parameter('sub_lidar_obstacle_topic', SUB_LIDAR_OBSTACLE_TOPIC_NAME).value
        self.sub_waypoint_zone_topic = self.declare_parameter('sub_waypoint_zone_topic', SUB_WAYPOINT_ZONE_TOPIC_NAME).value
        self.sub_cmd_vel_topic = self.declare_parameter('sub_cmd_vel_topic', SUB_CMD_VEL_TOPIC_NAME).value
        self.sub_imu_topic = self.declare_parameter('sub_imu_topic', SUB_IMU_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        
        self.timer_period = self.declare_parameter('timer', TIMER).value

        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # 변수 초기화
        self.detection_data = None
        self.path_data = None
        self.traffic_light_data = None
        self.lidar_data = None
        self.waypoint_zone_data = None  # waypoint 구간 정보 (string -> int 변환)
        self.cmd_vel_data = None
        self.imu_data = None

        # 신호등 상태 필터링 변수
        self.filtered_traffic_light_state = 'None'  # 필터링된 신호등 상태
        self.none_counter = 0  # 연속 None 카운터
        self.none_threshold = self.declare_parameter('traffic_light_none_threshold', 33).value  # None으로 간주할 임계값

        # waypoint 정지 구간 설정
        self.stop_zone_start = self.declare_parameter('stop_zone_start', 573).value  # 정지 시작 waypoint
        self.stop_zone_end = self.declare_parameter('stop_zone_end', 600).value     # 정지 끝 waypoint

        # 신호등 감지 웨이포인트 설정 (3개 특정 웨이포인트)
        self.traffic_light_waypoints = [
            self.declare_parameter('traffic_light_waypoint1', 141).value,  # 첫 번째 신호등 웨이포인트
            self.declare_parameter('traffic_light_waypoint2', 283).value,  # 두 번째 신호등 웨이포인트  
            self.declare_parameter('traffic_light_waypoint3', 497).value   # 세 번째 신호등 웨이포인트
        ]

        # 시스템 재시작 관련 설정
        self.system_reset_waypoints = [
            self.declare_parameter('system_reset_waypoint1', 380).value,  # 첫 번째 시스템 재시작 웨이포인트
            self.declare_parameter('system_reset_waypoint2', 683).value,  # 두 번째 시스템 재시작 웨이포인트
            self.declare_parameter('system_reset_waypoint3', 706).value   # 세 번째 시스템 재시작 웨이포인트
        ]

        self.steering_command = 0.0
        self.speed_command = 0.0
        
        # 시스템 재시작 상태 변수
        self.system_reset_in_progress = False
        self.system_reset_start_time = 0.0
        self.system_reset_step = 0  # 0: 대기, 1: IMU 종료, 2: IMU 재시작, 3: IMU 데이터 확인, 4: GPS 보정, 5: 보정 파일 확인, 6: 완료
        self.imu_data_received_after_reset = False
        self.gps_calibration_completed = False
        self.calibration_file_created = False

        # 서브스크라이버 설정
        self.detection_sub = self.create_subscription(DetectionArray, self.sub_detection_topic, self.detection_callback, self.qos_profile)
        self.path_sub = self.create_subscription(PathPlanningResult, self.sub_path_topic, self.path_callback, self.qos_profile)
        self.traffic_light_sub = self.create_subscription(String, self.sub_traffic_light_topic, self.traffic_light_callback, self.qos_profile)
        self.lidar_sub = self.create_subscription(Bool, self.sub_lidar_obstacle_topic, self.lidar_callback, self.qos_profile)
        self.waypoint_zone_sub = self.create_subscription(String, self.sub_waypoint_zone_topic, self.waypoint_zone_callback, self.qos_profile)
        self.cmd_vel_sub = self.create_subscription(Twist, self.sub_cmd_vel_topic, self.cmd_vel_callback, self.qos_profile)
        self.imu_sub = self.create_subscription(Imu, self.sub_imu_topic, self.imu_callback, self.qos_profile)

        # 퍼블리셔 설정
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, self.qos_profile)
        self.filtered_traffic_light_publisher = self.create_publisher(String, "filtered_traffic_light_info", self.qos_profile)
        self.system_reset_publisher = self.create_publisher(String, PUB_SYSTEM_RESET_TOPIC_NAME, self.qos_profile)

        # 타이머 설정
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def detection_callback(self, msg: DetectionArray):
        self.detection_data = msg

    def path_callback(self, msg: PathPlanningResult):
        self.path_data = list(zip(msg.x_points, msg.y_points))
                
    def traffic_light_callback(self, msg: String):
        self.traffic_light_data = msg
        self.get_logger().debug(f"Traffic light received: {msg.data}")

        # 신호등 상태 필터링 로직
        self._update_filtered_traffic_light_state(msg.data)

        # 신호등이 red로 인식되었을 때 추가 디버깅 로그
        if self.filtered_traffic_light_state == "red":
            self.get_logger().info("🚨 신호등 RED 감지!")

    def lidar_callback(self, msg: Bool):
        self.lidar_data = msg
        self.get_logger().debug(f"Lidar obstacle received: {msg.data}")

    def waypoint_zone_callback(self, msg: String):
        try:
            # String을 int로 변환
            self.waypoint_zone_data = int(msg.data)
            self.get_logger().debug(f"Waypoint zone received: {self.waypoint_zone_data}")
        except ValueError:
            self.get_logger().warning(f"Invalid waypoint zone data: {msg.data}")
            self.waypoint_zone_data = None

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel_data = msg

    def imu_callback(self, msg: Imu):
        """IMU 데이터 수신 - 시스템 재시작 후 IMU 데이터 수신 확인용"""
        self.imu_data = msg
        if self.system_reset_in_progress and self.system_reset_step >= 2:
            self.imu_data_received_after_reset = True
            self.get_logger().info("✅ IMU 데이터 수신 확인: 시스템 재시작 후 IMU 데이터 수신됨")

    def _update_filtered_traffic_light_state(self, current_state: str):
        """신호등 상태 필터링 로직 (None 안정화)"""
        if current_state != 'None':
            # 신호등 감지됨: 즉시 해당 상태로 변경하고 none 카운터 리셋
            if self.filtered_traffic_light_state != current_state:
                self.get_logger().debug(f"Traffic light state changed: {self.filtered_traffic_light_state} -> {current_state}")
            self.filtered_traffic_light_state = current_state
            self.none_counter = 0
        else:
            # 'None' 감지됨: 연속 카운터 증가
            self.none_counter += 1

            # 연속 N번 이상 None이면 None 상태로 변경
            if self.none_counter >= self.none_threshold:
                if self.filtered_traffic_light_state != 'None':
                    self.filtered_traffic_light_state = 'None'
                    self.get_logger().debug(f"Traffic light filtered to None (after {self.none_counter} consecutive None detections)")
            # 임계값 도달 전까지는 이전 상태 유지

    def _is_traffic_light_waypoint(self, waypoint: int) -> bool:
        """waypoint가 신호등 감지 웨이포인트 중 하나인지 확인"""
        return waypoint in self.traffic_light_waypoints

    def _is_system_reset_waypoint(self, waypoint: int) -> bool:
        """waypoint가 시스템 재시작 웨이포인트 중 하나인지 확인"""
        return waypoint in self.system_reset_waypoints

    def _start_system_reset(self):
        """시스템 재시작 시작"""
        self.system_reset_in_progress = True
        self.system_reset_start_time = time.time()
        self.system_reset_step = 1
        self.imu_data_received_after_reset = False
        self.gps_calibration_completed = False
        self.calibration_file_created = False
        self.get_logger().info("🔄 시스템 재시작 시작: IMU 오차 누적 해결을 위한 시스템 재시작")

    def _execute_system_reset_step(self):
        """시스템 재시작 단계별 실행"""
        current_time = time.time()
        
        if self.system_reset_step == 1:  # IMU 노드 종료
            self.get_logger().info("🔄 1단계: IMU 노드 종료 중...")
            try:
                subprocess.run(['pkill', '-f', 'imu_reader_node'], check=False)
                subprocess.run(['pkill', '-f', 'imu_raw_node'], check=False)
                time.sleep(1)  # 종료 대기
                self.system_reset_step = 2
                self.get_logger().info("   ✅ IMU 노드 종료 완료")
            except Exception as e:
                self.get_logger().warn(f"   ⚠️ IMU 노드 종료 중 오류: {e}")
                self.system_reset_step = 2  # 오류가 있어도 다음 단계로 진행

        elif self.system_reset_step == 2:  # IMU 노드 재시작
            self.get_logger().info("🔄 2단계: IMU 노드 재시작 중...")
            try:
                # 백그라운드에서 IMU 노드 재시작
                subprocess.Popen(['ros2', 'launch', 'imu_pkg', 'imu_launch.py'], 
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                time.sleep(2)  # 시작 대기
                self.system_reset_step = 3
                self.get_logger().info("   ✅ IMU 노드 재시작 완료")
            except Exception as e:
                self.get_logger().warn(f"   ⚠️ IMU 노드 재시작 중 오류: {e}")
                self.system_reset_step = 3  # 오류가 있어도 다음 단계로 진행

        elif self.system_reset_step == 3:  # IMU 데이터 수신 확인
            if self.imu_data_received_after_reset:
                self.system_reset_step = 4
                self.get_logger().info("🔄 3단계: IMU 데이터 수신 확인 완료")
            else:
                self.get_logger().info("🔄 3단계: IMU 데이터 수신 대기 중...")

        elif self.system_reset_step == 4:  # GPS-IMU 보정 실행
            self.get_logger().info("🔄 4단계: GPS-IMU 보정 실행 중...")
            try:
                workspace_path = os.environ.get('WORKSPACE_PATH', '/home/jh/ros2_workspace')
                calibration_script = f'{workspace_path}/src/gps_imu_calibration.py'
                
                if os.path.exists(calibration_script):
                    result = subprocess.run(['python3', calibration_script], 
                                          capture_output=True, text=True, timeout=30)
                    if result.returncode == 0:
                        self.gps_calibration_completed = True
                        self.system_reset_step = 5
                        self.get_logger().info("   ✅ GPS-IMU 보정 완료")
                    else:
                        self.get_logger().warn(f"   ⚠️ GPS-IMU 보정 실패: {result.stderr}")
                        self.gps_calibration_completed = True
                        self.system_reset_step = 5
                else:
                    self.get_logger().warn(f"   ⚠️ 보정 스크립트를 찾을 수 없습니다: {calibration_script}")
                    self.gps_calibration_completed = True
                    self.system_reset_step = 5
            except subprocess.TimeoutExpired:
                self.get_logger().warn("   ⚠️ GPS-IMU 보정 타임아웃")
                self.gps_calibration_completed = True
                self.system_reset_step = 5
            except Exception as e:
                self.get_logger().warn(f"   ⚠️ GPS-IMU 보정 중 오류: {e}")
                self.gps_calibration_completed = True
                self.system_reset_step = 5

        elif self.system_reset_step == 5:  # 보정 파일 생성 확인
            self.get_logger().info("🔄 5단계: 보정 파일 생성 확인 중...")
            try:
                workspace_path = os.environ.get('WORKSPACE_PATH', '/home/jh/ros2_workspace')
                calibration_file = f'{workspace_path}/src/imu_calibration_angle.txt'
                
                if os.path.exists(calibration_file):
                    self.calibration_file_created = True
                    self.system_reset_step = 6
                    self.get_logger().info("   ✅ 보정 파일 생성 확인 완료")
                else:
                    self.get_logger().warn("   ⚠️ 보정 파일이 생성되지 않았습니다")
                    self.calibration_file_created = True
                    self.system_reset_step = 6
            except Exception as e:
                self.get_logger().warn(f"   ⚠️ 보정 파일 확인 중 오류: {e}")
                self.calibration_file_created = True
                self.system_reset_step = 6

        elif self.system_reset_step == 6:  # 시스템 재시작 완료
            self.system_reset_in_progress = False
            self.system_reset_step = 0
            self.get_logger().info("✅ 시스템 재시작 완료: IMU 오차 누적 해결 완료")

    def timer_callback(self):
        # 시스템 재시작 진행 중인 경우
        if self.system_reset_in_progress:
            self._execute_system_reset_step()
            
            # 시스템 재시작 중에는 정지
            self.steering_command = 0.0
            self.speed_command = 0.0
            self.get_logger().info("🔄 시스템 재시작 진행 중: 정지 상태 유지")
            
            # 모션 명령 메시지 생성 및 퍼블리시
            motion_command_msg = MotionCommand()
            motion_command_msg.steering = self.steering_command
            motion_command_msg.speed = self.speed_command
            self.publisher.publish(motion_command_msg)
            return

        # 디버깅용 로그 추가
        self.get_logger().debug(f"waypoint_zone_data: {self.waypoint_zone_data}, lidar_data: {self.lidar_data}, traffic_light_raw: {self.traffic_light_data}, traffic_light_filtered: {self.filtered_traffic_light_state}, stop_zone: {self.stop_zone_start}-{self.stop_zone_end}")

        # 시스템 재시작 웨이포인트 확인
        if (self.waypoint_zone_data is not None and 
            self._is_system_reset_waypoint(self.waypoint_zone_data) and
            not self.system_reset_in_progress):
            self.get_logger().info(f"🔄 시스템 재시작 웨이포인트 {self.waypoint_zone_data} 도달: IMU 오차 누적 해결을 위한 시스템 재시작 시작")
            self._start_system_reset()
            
            # 시스템 재시작 시작 시 정지
            self.steering_command = 0.0
            self.speed_command = 0.0
            self.get_logger().info("🛑 시스템 재시작 시작: 정지 상태")
            
            # 모션 명령 메시지 생성 및 퍼블리시
            motion_command_msg = MotionCommand()
            motion_command_msg.steering = self.steering_command
            motion_command_msg.speed = self.speed_command
            self.publisher.publish(motion_command_msg)
            return

        # 정지 조건들 체크
        should_stop = False
        stop_reason = ""

        # 1. waypoint 정지 구간에서 라이다 장애물 감지
        if (self.waypoint_zone_data is not None and
            self.stop_zone_start <= self.waypoint_zone_data <= self.stop_zone_end and
            self.lidar_data is not None and self.lidar_data.data is True):
            should_stop = True
            stop_reason = f"Waypoint {self.waypoint_zone_data} 구간에서 장애물 감지"

        # 2. 신호등 웨이포인트에서 빨간색 감지 (필터링된 상태 사용)
        elif (self.waypoint_zone_data is not None and
              self.filtered_traffic_light_state == "red" and
              self._is_traffic_light_waypoint(self.waypoint_zone_data)):
            should_stop = True
            stop_reason = f"신호등 웨이포인트 {self.waypoint_zone_data}에서 빨간색 감지"

        if should_stop:
            # 정지
            self.steering_command = 0.0
            self.speed_command = 0.0
            self.get_logger().info(f"🛑 정지 발동: {stop_reason}")
            self.get_logger().debug(f"정지 조건 상세 - waypoint: {self.waypoint_zone_data}, lidar: {self.lidar_data}, traffic_light_filtered: {self.filtered_traffic_light_state}, none_counter: {self.none_counter}")
        else:
            # cmd_vel을 바탕으로 주행
            if self.cmd_vel_data is not None:
                # Twist 메시지에서 선속도와 각속도 추출
                self.speed_command = self.cmd_vel_data.linear.x  # 전진/후진 속도 (m/s)
                self.steering_command = self.cmd_vel_data.angular.z # 조향 각속도 (rad/s)

            else:
                # cmd_vel 데이터가 없으면 정지
                self.speed_command = 0.0  # 전진/후진 속도 (m/s)
                self.steering_command = 0.0 # 조향 각속도 (rad/s)
                

        # 필터링된 신호등 상태 발행
        filtered_msg = String()
        filtered_msg.data = self.filtered_traffic_light_state
        self.filtered_traffic_light_publisher.publish(filtered_msg)

        # 모션 명령 메시지 생성 및 퍼블리시
        motion_command_msg = MotionCommand()
        motion_command_msg.steering = self.steering_command
        motion_command_msg.speed = self.speed_command
        self.publisher.publish(motion_command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningSystemResetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
