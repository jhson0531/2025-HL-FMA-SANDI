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

        # waypoint 정지 구간 설정
        self.stop_zone_start = self.declare_parameter('stop_zone_start', 346).value  # 정지 시작 waypoint
        self.stop_zone_end = self.declare_parameter('stop_zone_end', 374).value     # 정지 끝 waypoint

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

        # 타이머 설정
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def detection_callback(self, msg: DetectionArray):
        self.detection_data = msg

    def path_callback(self, msg: PathPlanningResult):
        self.path_data = list(zip(msg.x_points, msg.y_points))
                
    def traffic_light_callback(self, msg: String):
        self.traffic_light_data = msg

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
        
    def timer_callback(self):
        # 디버깅용 로그 추가
        self.get_logger().debug(f"waypoint_zone_data: {self.waypoint_zone_data}, lidar_data: {self.lidar_data}, stop_zone: {self.stop_zone_start}-{self.stop_zone_end}")

        # waypoint 정지 구간에 있고 전방 장애물 감지 시에만 정지
        if (self.waypoint_zone_data is not None and
            self.stop_zone_start <= self.waypoint_zone_data <= self.stop_zone_end and
            self.lidar_data is not None and self.lidar_data.data is True):
            # 정지 구간에서 전방 장애물을 감지한 경우 - 정지
            self.steering_command = 0.0
            self.speed_command = 0.0
            self.get_logger().info(f"Waypoint {self.waypoint_zone_data} 구간에서 장애물 감지로 인한 정지")

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
