import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Twist
from interfaces_pkg.msg import PathPlanningResult, DetectionArray, MotionCommand
from .lib import decision_making_func_lib as DMFL

#---------------Variable Setting---------------
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_PATH_TOPIC_NAME = "path_planning_result"
SUB_TRAFFIC_LIGHT_TOPIC_NAME = "yolov8_traffic_light_info"
SUB_LIDAR_OBSTACLE_TOPIC_NAME = "lidar_obstacle_info_front"
SUB_WAYPOINT_ZONE_TOPIC_NAME = "waypoint_zone_info"  # waypoint 구간 정보 토픽
SUB_CMD_VEL_TOPIC_NAME = "cmd_vel"
PUB_TOPIC_NAME = "topic_control_signal"

#----------------------------------------------

# 모션 플랜 발행 주기 (초) - 소수점 필요 (int형은 반영되지 않음)
TIMER = 0.01

class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('motion_planner_node')

        # 토픽 이름 설정
        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.sub_path_topic = self.declare_parameter('sub_lane_topic', SUB_PATH_TOPIC_NAME).value
        self.sub_traffic_light_topic = self.declare_parameter('sub_traffic_light_topic', SUB_TRAFFIC_LIGHT_TOPIC_NAME).value
        self.sub_lidar_obstacle_topic = self.declare_parameter('sub_lidar_obstacle_topic', SUB_LIDAR_OBSTACLE_TOPIC_NAME).value
        self.sub_waypoint_zone_topic = self.declare_parameter('sub_waypoint_zone_topic', SUB_WAYPOINT_ZONE_TOPIC_NAME).value
        self.sub_cmd_vel_topic = self.declare_parameter('sub_cmd_vel_topic', SUB_CMD_VEL_TOPIC_NAME).value
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

        # 신호등 상태 필터링 변수
        self.filtered_traffic_light_state = 'None'  # 필터링된 신호등 상태
        self.none_counter = 0  # 연속 None 카운터
        self.none_threshold = self.declare_parameter('traffic_light_none_threshold', 33).value  # None으로 간주할 임계값

        # waypoint 정지 구간 설정     --- 이구간 변경 ---
        self.stop_zone_start = self.declare_parameter('stop_zone_start', 573).value  # 정지 시작 waypoint
        self.stop_zone_end = self.declare_parameter('stop_zone_end', 600).value     # 정지 끝 waypoint

        # 신호등 정지 waypoint 설정 (특정 점에서만 멈춤)   --- 이구간 변경 ---
        self.traffic_light_stop_waypoints = [
            self.declare_parameter('traffic_light_stop_waypoint1', 141).value,  # 첫 번째 신호등 정지점
            self.declare_parameter('traffic_light_stop_waypoint2', 281).value,  # 두 번째 신호등 정지점
            self.declare_parameter('traffic_light_stop_waypoint3', 497).value   # 세 번째 신호등 정지점
        ]

        self.steering_command = 0.0
        self.speed_command = 0.0
        

        # 서브스크라이버 설정
        self.detection_sub = self.create_subscription(DetectionArray, self.sub_detection_topic, self.detection_callback, self.qos_profile)
        self.path_sub = self.create_subscription(PathPlanningResult, self.sub_path_topic, self.path_callback, self.qos_profile)
        self.traffic_light_sub = self.create_subscription(String, self.sub_traffic_light_topic, self.traffic_light_callback, self.qos_profile)
        self.lidar_sub = self.create_subscription(Bool, self.sub_lidar_obstacle_topic, self.lidar_callback, self.qos_profile)
        self.waypoint_zone_sub = self.create_subscription(String, self.sub_waypoint_zone_topic, self.waypoint_zone_callback, self.qos_profile)
        self.cmd_vel_sub = self.create_subscription(Twist, self.sub_cmd_vel_topic, self.cmd_vel_callback, self.qos_profile)

        # 퍼블리셔 설정
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, self.qos_profile)

        # 필터링된 신호등 상태 발행용 퍼블리셔
        self.filtered_traffic_light_publisher = self.create_publisher(String, "filtered_traffic_light_info", self.qos_profile)

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

    def _is_at_traffic_light_stop_point(self, waypoint: int) -> bool:
        """waypoint가 신호등 정지점 중 하나인지 확인"""
        return waypoint in self.traffic_light_stop_waypoints
        
    def timer_callback(self):
        # 디버깅용 로그 추가
        self.get_logger().debug(f"waypoint_zone_data: {self.waypoint_zone_data}, lidar_data: {self.lidar_data}, traffic_light_raw: {self.traffic_light_data}, traffic_light_filtered: {self.filtered_traffic_light_state}, stop_zone: {self.stop_zone_start}-{self.stop_zone_end}")

        # 정지 조건들 체크
        should_stop = False
        stop_reason = ""

        # 1. waypoint 정지 구간에서 라이다 장애물 감지
        if (self.waypoint_zone_data is not None and
            self.stop_zone_start <= self.waypoint_zone_data <= self.stop_zone_end and
            self.lidar_data is not None and self.lidar_data.data is True):
            should_stop = True
            stop_reason = f"Waypoint {self.waypoint_zone_data} 구간에서 장애물 감지"

        # 2. 신호등 정지점에서 빨간색 감지 (필터링된 상태 사용)
        elif (self.waypoint_zone_data is not None and
              self.filtered_traffic_light_state == "red" and
              self._is_at_traffic_light_stop_point(self.waypoint_zone_data)):
            should_stop = True
            stop_reason = f"신호등 정지점에서 빨간색 감지 (waypoint: {self.waypoint_zone_data})"

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
    node = MotionPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
