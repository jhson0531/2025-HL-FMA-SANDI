#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_pose
import math
import utm
import numpy as np

class UTMOdometryConverter(Node):
    def __init__(self):
        super().__init__('utm_odometry_converter')
        
        # TF 변환을 위한 설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 토픽 구독/발행
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.odometry_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Odometry,
            '/odometry/utm',
            10
        )
        
        # UTM 변환을 위한 기준점 설정
        self.reference_lat = 37.5665  # 서울 위도 (기본값)
        self.reference_lon = 126.9780  # 서울 경도 (기본값)
        self.utm_zone = None
        self.utm_letter = None
        
        # 첫 번째 odometry 수신 여부
        self.first_odometry_received = False
        
        self.get_logger().info("UTM Odometry 변환기가 시작되었습니다.")
        self.get_logger().info(f"기준점: 위도 {self.reference_lat}, 경도 {self.reference_lon}")
        
    def setup_utm_reference(self, lat, lon):
        """UTM 기준점 설정"""
        self.reference_lat = lat
        self.reference_lon = lon
        
        # UTM 좌표로 변환
        utm_x, utm_y, zone, letter = utm.from_latlon(lat, lon)
        self.utm_zone = zone
        self.utm_letter = letter
        
        self.get_logger().info(f"UTM 기준점 설정: Zone {zone}{letter}")
        self.get_logger().info(f"UTM 좌표: X={utm_x:.3f}, Y={utm_y:.3f}")
        
        # 정적 TF 변환 설정
        self.setup_static_transform(utm_x, utm_y)
        
    def setup_static_transform(self, utm_x, utm_y):
        """정적 TF 변환 설정 (map → utm)"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "utm"
        
        # UTM 좌표계로 변환 (map 프레임의 원점을 UTM 좌표로 이동)
        transform.transform.translation.x = utm_x
        transform.transform.translation.y = utm_y
        transform.transform.translation.z = 0.0
        
        # 회전은 없음 (동일한 방향)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        # TF 변환 발행
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info("정적 TF 변환 설정 완료: map → utm")
        
    def odometry_callback(self, msg):
        """Odometry 메시지 변환"""
        try:
            # 첫 번째 odometry 수신 시 UTM 기준점 설정
            if not self.first_odometry_received:
                self.get_logger().info("=" * 60)
                self.get_logger().info("📍 첫 번째 Odometry 수신")
                self.get_logger().info(f"   Map 좌표: X={msg.pose.pose.position.x:.3f}, Y={msg.pose.pose.position.y:.3f}")
                self.get_logger().info("=" * 60)
                self.first_odometry_received = True
                
                # 현재 위치를 기준으로 UTM 기준점 설정 (실제로는 GPS에서 받아야 함)
                # 여기서는 기본값 사용
                self.setup_utm_reference(self.reference_lat, self.reference_lon)
            
            # TF 변환을 통한 좌표 변환
            utm_odometry = self.transform_odometry_to_utm(msg)
            
            if utm_odometry:
                self.publisher.publish(utm_odometry)
                
        except Exception as e:
            self.get_logger().error(f"Odometry 변환 실패: {e}")
    
    def transform_odometry_to_utm(self, odom_msg):
        """Odometry를 UTM 좌표계로 변환"""
        try:
            # map → utm 변환 가져오기
            transform = self.tf_buffer.lookup_transform(
                'utm',                    # target frame
                'map',                    # source frame
                odom_msg.header.stamp,    # timestamp
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 위치 변환
            pose_stamped = PoseStamped()
            pose_stamped.header = odom_msg.header
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose = odom_msg.pose.pose
            
            # TF 변환 적용
            transformed_pose = do_transform_pose(pose_stamped, transform)
            
            # 새로운 odometry 메시지 생성
            utm_odom = Odometry()
            utm_odom.header = odom_msg.header
            utm_odom.header.frame_id = "utm"  # 프레임 ID 변경
            utm_odom.child_frame_id = odom_msg.child_frame_id
            
            # 변환된 위치와 자세 설정
            utm_odom.pose.pose = transformed_pose.pose
            
            # 속도는 그대로 유지 (상대적 속도)
            utm_odom.twist = odom_msg.twist
            
            # 공분산은 그대로 유지
            utm_odom.pose.covariance = odom_msg.pose.covariance
            utm_odom.twist.covariance = odom_msg.twist.covariance
            
            return utm_odom
            
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"TF 변환 실패: {e}")
            return None
    
    def set_reference_coordinates(self, lat, lon):
        """기준 좌표 설정 (파라미터로 조절 가능)"""
        self.setup_utm_reference(lat, lon)
        self.get_logger().info(f"기준 좌표 변경: 위도 {lat}, 경도 {lon}")

def main(args=None):
    rclpy.init(args=args)
    
    # UTM 변환기 노드 생성
    utm_converter = UTMOdometryConverter()
    
    # 기준 좌표 설정 (예시: 서울)
    # utm_converter.set_reference_coordinates(37.5665, 126.9780)
    
    try:
        rclpy.spin(utm_converter)
    except KeyboardInterrupt:
        utm_converter.get_logger().info("UTM 변환기 종료 중...")
    finally:
        utm_converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
