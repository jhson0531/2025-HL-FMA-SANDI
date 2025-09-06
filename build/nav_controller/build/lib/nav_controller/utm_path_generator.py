#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
import scipy.interpolate as si
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
import utm

class UTMPathGenerator(Node):
    def __init__(self):
        super().__init__('utm_path_generator')
        
        # UTM 좌표 데이터 (사용자의 데이터)
        self.utm_waypoints = [
            [332256.21125681617, 4128605.3762128204, 68.90436278616231],
            [332254.3726604126, 4128606.1234118533, 67.74646969769768],
            [332252.4906271134, 4128606.9158805455, 72.29038663964751],
            [332250.6334126871, 4128607.619053368, 73.09305003944702],
            [332248.7037228432, 4128608.2459778464, 73.0930355053634],
            [332246.78423104354, 4128608.93929827, 72.94484069321054]
        ]
        
        # UTM 좌표를 로컬 좌표계로 변환 (첫 번째 점을 원점으로 설정)
        self.origin_utm = self.utm_waypoints[0]
        self.local_waypoints = []
        
        for waypoint in self.utm_waypoints:
            local_x = waypoint[0] - self.origin_utm[0]
            local_y = waypoint[1] - self.origin_utm[1]
            heading = waypoint[2]
            self.local_waypoints.append([local_x, local_y, heading])
        
        # B-Spline으로 경로 보간
        self.smooth_path = self.create_smooth_path(self.local_waypoints)
        
        # ROS2 설정
        self.path_publisher = self.create_publisher(PoseArray, 'utm_path', 10)
        
        # 경로 발행
        self.publish_path()
        
        self.get_logger().info('UTM Path Generator 시작됨')
        self.get_logger().info(f'원본 UTM 좌표: {len(self.utm_waypoints)}개')
        self.get_logger().info(f'보간된 경로: {len(self.smooth_path)}개')
        
        # UTM 좌표 정보 출력
        self.print_utm_info()
    
    def create_smooth_path(self, waypoints, num_points=100):
        """
        B-Spline을 사용하여 경로를 부드럽게 보간
        """
        try:
            waypoints = np.array(waypoints)
            x = waypoints[:, 0]
            y = waypoints[:, 1]
            
            # B-Spline 보간
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
            
            ipl_t = np.linspace(0.0, len(x) - 1, num_points)
            rx = si.splev(ipl_t, x_list)
            ry = si.splev(ipl_t, y_list)
            
            smooth_path = [(rx[i], ry[i]) for i in range(len(rx))]
            return smooth_path
            
        except Exception as e:
            self.get_logger().error(f'경로 보간 중 오류: {e}')
            # 보간 실패시 원본 경로 사용
            return [(wp[0], wp[1]) for wp in waypoints]
    
    def publish_path(self):
        """
        생성된 경로를 PoseArray로 발행
        """
        path_msg = PoseArray()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for point in self.smooth_path:
            pose = Pose()
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.position.z = 0.0
            path_msg.poses.append(pose)
        
        self.path_publisher.publish(path_msg)
        self.get_logger().info(f'경로 발행 완료: {len(path_msg.poses)}개 점')
    
    def print_utm_info(self):
        """
        UTM 좌표 정보 출력
        """
        self.get_logger().info("=== UTM 좌표 정보 ===")
        self.get_logger().info(f"원점 UTM: X={self.origin_utm[0]:.6f}, Y={self.origin_utm[1]:.6f}")
        
        for i, waypoint in enumerate(self.utm_waypoints):
            self.get_logger().info(f"점 {i+1}: UTM X={waypoint[0]:.6f}, UTM Y={waypoint[1]:.6f}, Heading={waypoint[2]:.6f}")
        
        self.get_logger().info("=== 로컬 좌표 (원점 기준) ===")
        for i, waypoint in enumerate(self.local_waypoints):
            self.get_logger().info(f"점 {i+1}: Local X={waypoint[0]:.6f}, Local Y={waypoint[1]:.6f}, Heading={waypoint[2]:.6f}")
    
    def get_path_info(self):
        """
        경로 정보 반환
        """
        return {
            'utm_waypoints': self.utm_waypoints,
            'local_waypoints': self.local_waypoints,
            'smooth_path': self.smooth_path,
            'origin_utm': self.origin_utm
        }

def main(args=None):
    rclpy.init(args=args)
    path_generator = UTMPathGenerator()
    
    try:
        rclpy.spin(path_generator)
    except KeyboardInterrupt:
        pass
    finally:
        path_generator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
