# lidar_publisher_node.py 파일을 아래 내용으로 수정 또는 교체하세요.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

# TF(좌표 변환)를 위한 import문은 그대로 둡니다.
import tf2_ros
import geometry_msgs.msg

class LidarPerceptionNode(Node):  # 역할에 맞게 클래스 이름을 변경했습니다.
    def __init__(self):
        # 노드 이름도 역할에 맞게 변경했습니다.
        super().__init__('lidar_perception_node')

        # sllidar_ros2가 발행하는 '/scan' 토픽을 구독합니다.
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',                # <-- 공식 드라이버가 발행하는 토픽
            self.scan_callback,     # <-- 데이터가 들어오면 이 함수가 실행됩니다.
            10)
        
        # TF Broadcaster는 필요하다면 그대로 사용할 수 있습니다.
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info('LIDAR 인식 노드가 시작되었습니다. /scan 토픽을 구독합니다.')

    def scan_callback(self, msg):
        """
        /scan 토픽으로 LaserScan 메시지가 들어올 때마다 이 함수가 자동으로 실행됩니다.
        이제 우리는 하드웨어 걱정 없이, 이미 잘 가공된 데이터만 받아서 사용하면 됩니다.
        """
        # TF 발행 로직은 그대로 유지할 수 있습니다.
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'laser_frame'
        # 실제 로봇의 센서 위치에 맞게 값을 조정해야 합니다.
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        self.tf_broadcaster.sendTransform(transform)
        
        # msg.ranges 배열에는 360도 전체의 거리 데이터(미터 단위)가 들어있습니다.
        # 예를 들어, 정면(0도)과 후면(180도)의 거리 값을 확인해 봅시다.
        front_distance = msg.ranges[0]
        rear_distance = msg.ranges[len(msg.ranges) // 2]

        self.get_logger().info(
            f'정면 거리: {front_distance:.2f}m, '
            f'후면 거리: {rear_distance:.2f}m'
        )

        # --- 여기에 원하는 장애물 감지 및 회피 로직을 구현하면 됩니다 ---
        if front_distance < 0.5 and front_distance > msg.range_min: # 0.5미터 이내 장애물 감지
            self.get_logger().warn('!!! 전방에 장애물 발견 !!!')
        # ----------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    lidar_perception_node = LidarPerceptionNode()
    try:
        rclpy.spin(lidar_perception_node)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_perception_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()