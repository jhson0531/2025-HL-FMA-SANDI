import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from .lib import lidar_perception_func_lib as LPFL

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = 'lidar_processed'  # 구독할 토픽 이름

# Publish할 토픽 이름들
PUB_TOPIC_FRONT = 'lidar_obstacle_info_front'   # 전방 장애물 감지 (345~15도)
PUB_TOPIC_LEFT = 'lidar_obstacle_info_left'     # 좌측 장애물 감지 (60~120도)
PUB_TOPIC_RIGHT = 'lidar_obstacle_info_right'   # 우측 장애물 감지 (240~300도)
#----------------------------------------------


class ObjectDetection(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector_node')

        # 각 구간별 파라미터 선언
        self.declare_parameter('front_angle_start', 345) # 전방 시작 각도
        self.declare_parameter('front_angle_end', 15)    # 전방 끝 각도
        self.declare_parameter('front_range_min', 0.3)   # 전방 최소 감지 거리
        self.declare_parameter('front_range_max', 3.0)   # 전방 최대 감지 거리
        self.declare_parameter('left_angle_start', 240)  # 좌측 시작 각도
        self.declare_parameter('left_angle_end', 300)    # 좌측 끝 각도
        self.declare_parameter('left_range_min', 0.3)    # 좌측 최소 감지 거리
        self.declare_parameter('left_range_max', 2.0)    # 좌측 최대 감지 거리
        self.declare_parameter('right_angle_start', 60)  # 우측 시작 각도
        self.declare_parameter('right_angle_end', 120)   # 우측 끝 각도
        self.declare_parameter('right_range_min', 0.3)   # 우측 최소 감지 거리
        self.declare_parameter('right_range_max', 2.0)   # 우측 최대 감지 거리
        self.declare_parameter('consec_count', 5)        # 연속 감지 횟수

        # 각 구간별 각도 및 이름 설정
        self.ZONES = {
            'front': {'name': '전방'},
            'left': {'name': '좌측'},
            'right': {'name': '우측'}
        }

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.subscriber = self.create_subscription(LaserScan, SUB_TOPIC_NAME, self.lidar_callback, self.qos_profile)

        # 각 구간별 퍼블리셔 생성
        self.publisher_front = self.create_publisher(Bool, PUB_TOPIC_FRONT, self.qos_profile)
        self.publisher_left = self.create_publisher(Bool, PUB_TOPIC_LEFT, self.qos_profile)
        self.publisher_right = self.create_publisher(Bool, PUB_TOPIC_RIGHT, self.qos_profile)

        # 각 구간별 StabilityDetector 생성
        self.detection_checkers = {}
        consec_count = self.get_parameter('consec_count').get_parameter_value().integer_value
        for zone in self.ZONES.keys():
            self.detection_checkers[zone] = LPFL.StabilityDetector(consec_count=consec_count)




    def lidar_callback(self, msg):
        ranges = msg.ranges

        # 각 구간별로 장애물 감지 수행
        for zone_name, zone_config in self.ZONES.items():
            zone_name_ko = zone_config['name']

            # 각 구간별 각도 및 거리 파라미터 가져오기
            if zone_name == 'front':
                start_angle = self.get_parameter('front_angle_start').get_parameter_value().integer_value
                end_angle = self.get_parameter('front_angle_end').get_parameter_value().integer_value
                range_min = self.get_parameter('front_range_min').get_parameter_value().double_value
                range_max = self.get_parameter('front_range_max').get_parameter_value().double_value
            elif zone_name == 'left':
                start_angle = self.get_parameter('left_angle_start').get_parameter_value().integer_value
                end_angle = self.get_parameter('left_angle_end').get_parameter_value().integer_value
                range_min = self.get_parameter('left_range_min').get_parameter_value().double_value
                range_max = self.get_parameter('left_range_max').get_parameter_value().double_value
            elif zone_name == 'right':
                start_angle = self.get_parameter('right_angle_start').get_parameter_value().integer_value
                end_angle = self.get_parameter('right_angle_end').get_parameter_value().integer_value
                range_min = self.get_parameter('right_range_min').get_parameter_value().double_value
                range_max = self.get_parameter('right_range_max').get_parameter_value().double_value

            # 각 구간별 장애물 감지
            detected, detection_details = LPFL.detect_object_with_details(
                ranges=ranges,
                start_angle=start_angle,
                end_angle=end_angle,
                range_min=range_min,
                range_max=range_max
            )

            # 연속 감지 확인
            detection_result = self.detection_checkers[zone_name].check_consecutive_detections(detected)

            # 각 구간별 토픽 publish
            detection_msg = Bool()
            detection_msg.data = detection_result

            if zone_name == 'front':
                self.publisher_front.publish(detection_msg)
            elif zone_name == 'left':
                self.publisher_left.publish(detection_msg)
            elif zone_name == 'right':
                self.publisher_right.publish(detection_msg)

            # 디버깅 출력 - 각 구간별 감지 정보
            if detected and detection_details:
                min_angle = min(detail['angle'] for detail in detection_details)
                max_angle = max(detail['angle'] for detail in detection_details)
                min_distance = min(detail['distance'] for detail in detection_details)
                max_distance = max(detail['distance'] for detail in detection_details)

                self.get_logger().info(f'🚨 [{zone_name_ko}] 장애물 감지: {min_angle:.1f}도~{max_angle:.1f}도, {min_distance:.2f}m~{max_distance:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetection()
    rclpy.spin(object_detection_node)
    object_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

