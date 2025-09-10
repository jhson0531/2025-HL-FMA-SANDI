#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class IMUYawConverter(Node):
    def __init__(self):
        super().__init__('imu_yaw_converter')
        
        # IMU 토픽 구독
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        # 첫 번째 IMU 데이터 수신 여부
        self.first_imu_received = False
        
        self.get_logger().info("IMU Yaw 변환기 노드가 시작되었습니다.")
        self.get_logger().info("/imu/data 토픽을 구독 중...")
    
    def euler_from_quaternion(self, x, y, z, w):
        """쿼터니언에서 yaw 각도 추출"""
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return yaw_z
    
    def imu_callback(self, msg):
        """IMU 데이터 처리"""
        # 쿼터니언에서 yaw 각도 추출
        yaw_rad = self.euler_from_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        
        # 라디안을 도로 변환
        yaw_deg = math.degrees(yaw_rad)
        
        # 첫 번째 IMU 수신 시 상세 정보 출력
        if not self.first_imu_received:
            self.get_logger().info("=" * 70)
            self.get_logger().info("🧭 IMU 초기 방위각 정보")
            self.get_logger().info(f"   쿼터니언 (w, x, y, z): ({msg.orientation.w:.6f}, {msg.orientation.x:.6f}, {msg.orientation.y:.6f}, {msg.orientation.z:.6f})")
            self.get_logger().info(f"   Yaw 각도: {yaw_rad:.6f} rad ({yaw_deg:.2f}°)")
            self.get_logger().info(f"   프레임 ID: {msg.header.frame_id}")
            self.get_logger().info(f"   타임스탬프: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}")
            self.get_logger().info("=" * 70)
            self.get_logger().info("실시간 Yaw 각도 모니터링을 시작합니다...")
            self.get_logger().info("(Ctrl+C로 종료)")
            self.get_logger().info("-" * 70)
            self.first_imu_received = True
        
        # 실시간 Yaw 각도 출력 (10Hz로 제한 - 0.1초마다)
        current_time = self.get_clock().now()
        if not hasattr(self, 'last_print_time') or (current_time - self.last_print_time).nanoseconds >= 1e8:
            self.get_logger().info(f"Yaw: {yaw_deg:8.2f}° ({yaw_rad:8.6f} rad)")
            self.last_print_time = current_time

def main(args=None):
    rclpy.init(args=args)
    
    # IMU Yaw 변환기 노드 생성
    imu_yaw_converter = IMUYawConverter()
    
    try:
        rclpy.spin(imu_yaw_converter)
    except KeyboardInterrupt:
        imu_yaw_converter.get_logger().info("노드 종료 중...")
    finally:
        imu_yaw_converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
