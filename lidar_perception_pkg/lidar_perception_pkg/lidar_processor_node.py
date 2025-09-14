import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from .lib import lidar_perception_func_lib as LPFL

#---------------Variable Setting---------------
# Subscribe할 토픽 이름 (역할에 맞게 '/scan'으로 수정)
SUB_TOPIC_NAME = '/scan'

# Publish할 토픽 이름
PUB_TOPIC_NAME = 'lidar_processed'
#----------------------------------------------

class LidarSensorDataProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor_node')

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # 이제 변수를 사용하도록 수정
        self.subscription = self.create_subscription(
            LaserScan,
            SUB_TOPIC_NAME,
            self.scan_callback, # <-- 콜백 함수 이름도 역할에 맞게 변경 (선택 사항)
            self.qos_profile)  
        
        self.publisher = self.create_publisher(
            LaserScan,
            PUB_TOPIC_NAME,
            self.qos_profile) 

    def scan_callback(self, msg): # <-- 함수 이름 변경
        # 이 함수는 Lidar 데이터를 수신할 때마다 호출 됨.
        self.get_logger().info(f'Received scan from {SUB_TOPIC_NAME}')
        
        # 데이터 가공 로직은 그대로 유지
        processed_msg = LPFL.rotate_lidar_data(msg, offset = 0)
        processed_msg = LPFL.flip_lidar_data(processed_msg, pivot_angle = 0)
        
        # 가공된 메시지를 발행
        self.publisher.publish(processed_msg)
        self.get_logger().info(f'Publishing processed scan to {PUB_TOPIC_NAME}')

def main(args=None):
    rclpy.init(args=args)
    lidar_processor = LidarSensorDataProcessor()
    rclpy.spin(lidar_processor)
    lidar_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()