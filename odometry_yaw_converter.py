#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class OdometryYawConverter(Node):
    def __init__(self):
        super().__init__('odometry_yaw_converter')
        
        # Odometry 토픽 구독
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/global_utm',
            self.odometry_callback,
            10
        )
        
        # 첫 번째 Odometry 데이터 수신 여부
        self.first_odometry_received = False
        
        self.get_logger().info("Odometry Yaw 변환기 노드가 시작되었습니다.")
        self.get_logger().info("/odometry/global_utm 토픽을 구독 중...")
    
    def euler_from_quaternion(self, x, y, z, w):
        """쿼터니언에서 yaw 각도 추출"""
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return yaw_z
    
    def odometry_callback(self, msg):
        """Odometry 데이터 처리"""
        # 쿼터니언에서 yaw 각도 추출
        yaw_rad = self.euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        
        # 라디안을 도로 변환
        yaw_deg = math.degrees(yaw_rad)
        
        # 첫 번째 Odometry 수신 시 상세 정보 출력
        if not self.first_odometry_received:
            self.get_logger().info("=" * 80)
            self.get_logger().info("🧭 Odometry 초기 방위각 정보")
            self.get_logger().info(f"   위치 (x, y, z): ({msg.pose.pose.position.x:.6f}, {msg.pose.pose.position.y:.6f}, {msg.pose.pose.position.z:.6f})")
            self.get_logger().info(f"   쿼터니언 (w, x, y, z): ({msg.pose.pose.orientation.w:.6f}, {msg.pose.pose.orientation.x:.6f}, {msg.pose.pose.orientation.y:.6f}, {msg.pose.pose.orientation.z:.6f})")
            self.get_logger().info(f"   Yaw 각도: {yaw_rad:.6f} rad ({yaw_deg:.2f}°)")
            self.get_logger().info(f"   프레임 ID: {msg.header.frame_id}")
            self.get_logger().info(f"   자식 프레임 ID: {msg.child_frame_id}")
            self.get_logger().info(f"   타임스탬프: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}")
            self.get_logger().info("=" * 80)
            self.get_logger().info("실시간 Odometry Yaw 각도 모니터링을 시작합니다...")
            self.get_logger().info("(Ctrl+C로 종료)")
            self.get_logger().info("-" * 80)
            self.first_odometry_received = True
        
        # 실시간 Yaw 각도 출력 (5Hz로 제한 - 0.2초마다)
        current_time = self.get_clock().now()
        if not hasattr(self, 'last_print_time') or (current_time - self.last_print_time).nanoseconds >= 2e8:
            self.get_logger().info(f"📍 위치: ({msg.pose.pose.position.x:8.3f}, {msg.pose.pose.position.y:8.3f}) | Yaw: {yaw_deg:8.2f}° ({yaw_rad:8.6f} rad)")
            self.last_print_time = current_time

def main(args=None):
    rclpy.init(args=args)
    
    # Odometry Yaw 변환기 노드 생성
    odometry_yaw_converter = OdometryYawConverter()
    
    try:
        rclpy.spin(odometry_yaw_converter)
    except KeyboardInterrupt:
        odometry_yaw_converter.get_logger().info("노드 종료 중...")
    finally:
        odometry_yaw_converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
