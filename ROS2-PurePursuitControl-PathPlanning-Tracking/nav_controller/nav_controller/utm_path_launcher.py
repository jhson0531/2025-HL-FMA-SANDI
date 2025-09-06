#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
import scipy.interpolate as si
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import utm

class UTMPathLauncher(Node):
    def __init__(self):
        super().__init__('utm_path_launcher')
        
        # UTM 좌표 데이터 (X, Y 좌표만)
        self.utm_waypoints = [
            [330034.491650, 4123117.956354],
            [330030.501943, 4123118.148129],
            [330024.601050, 4123120.376601]
        ]
        
        # 로봇 초기 위치 (odom에서 받아올 예정, 임시로 0,0 설정)
        self.robot_initial_x = 0.0
        self.robot_initial_y = 0.0
        self.robot_initial_yaw = 0.0
        
        # UTM 좌표를 로봇 초기 위치 기준 로컬 좌표계로 변환
        self.local_waypoints = []
        self.utm_to_local_offset_x = 0.0
        self.utm_to_local_offset_y = 0.0
        self.utm_to_local_scale = 1.0  # UTM 미터 단위를 로컬 미터 단위로 변환
        
        # 초기화 플래그
        self.robot_initialized = False
        
        # ROS2 설정
        self.subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_publisher = self.create_publisher(PoseArray, 'utm_path', 10)
        
        # 제어 변수
        self.lookahead_distance = 0.15
        self.speed = 0.1
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.path_index = 0
        self.is_navigating = False
        
        # B-Spline으로 경로 보간 (로봇 초기화 후에 실행)
        self.smooth_path = []
        
        # 타이머
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('UTM Path Launcher 시작됨')
        self.get_logger().info('로봇 초기 위치 대기 중...')
    
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
    
    def initialize_robot_position(self):
        """
        로봇 초기 위치를 기준으로 UTM waypoint를 로컬 좌표로 변환
        """
        if self.robot_initialized:
            return
        
        # 로봇 초기 위치를 UTM 좌표계의 첫 번째 waypoint 근처로 가정
        # 실제로는 GPS/UTM 센서에서 받아와야 함
        first_utm_x = self.utm_waypoints[0][0]
        first_utm_y = self.utm_waypoints[0][1]
        
        # 로봇 초기 위치를 첫 번째 waypoint에서 약간 떨어진 곳으로 설정
        # (실제 환경에서는 GPS에서 받아온 UTM 좌표 사용)
        self.robot_initial_utm_x = first_utm_x - 5.0  # 5미터 앞
        self.robot_initial_utm_y = first_utm_y - 2.0  # 2미터 왼쪽
        
        # UTM 좌표를 로봇 초기 위치 기준 로컬 좌표로 변환
        self.local_waypoints = []
        for waypoint in self.utm_waypoints:
            local_x = (waypoint[0] - self.robot_initial_utm_x) * self.utm_to_local_scale
            local_y = (waypoint[1] - self.robot_initial_utm_y) * self.utm_to_local_scale
            self.local_waypoints.append([local_x, local_y])
        
        # B-Spline으로 경로 보간
        self.smooth_path = self.create_smooth_path(self.local_waypoints)
        
        # 경로 발행
        self.publish_path()
        
        self.robot_initialized = True
        
        self.get_logger().info("=== 로봇 초기화 완료 ===")
        self.get_logger().info(f"로봇 초기 UTM 위치: X={self.robot_initial_utm_x:.6f}, Y={self.robot_initial_utm_y:.6f}")
        self.get_logger().info(f"로봇 초기 로컬 위치: X={self.robot_initial_x:.6f}, Y={self.robot_initial_y:.6f}")
        self.get_logger().info(f"총 {len(self.smooth_path)}개의 경로점 생성됨")
        
        # UTM 좌표 정보 출력
        self.print_utm_info()
        
        # 3초 후 자동으로 경로 추적 시작
        import threading
        def delayed_start():
            import time
            time.sleep(3.0)
            self.start_navigation()
        
        start_thread = threading.Thread(target=delayed_start)
        start_thread.daemon = True
        start_thread.start()
    
    def print_utm_info(self):
        """
        UTM 좌표 정보 출력
        """
        self.get_logger().info("=== UTM 좌표 정보 ===")
        for i, waypoint in enumerate(self.utm_waypoints):
            self.get_logger().info(f"점 {i+1}: UTM X={waypoint[0]:.6f}, UTM Y={waypoint[1]:.6f}")
        
        self.get_logger().info("=== 로컬 좌표 (로봇 초기 위치 기준) ===")
        for i, waypoint in enumerate(self.local_waypoints):
            self.get_logger().info(f"점 {i+1}: Local X={waypoint[0]:.6f}, Local Y={waypoint[1]:.6f}")
    
    def pure_pursuit_control(self, current_x, current_y, current_heading, path, index):
        """
        Pure Pursuit 알고리즘으로 경로 추적
        """
        closest_point = None
        v = self.speed
        
        for i in range(index, len(path)):
            x = path[i][0]
            y = path[i][1]
            distance = math.hypot(current_x - x, current_y - y)
            
            if self.lookahead_distance < distance:
                closest_point = (x, y)
                index = i
                break
        
        if closest_point is not None:
            target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
            desired_steering_angle = target_heading - current_heading
        else:
            target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
            desired_steering_angle = target_heading - current_heading
            index = len(path) - 1
        
        # 각도 정규화
        if desired_steering_angle > math.pi:
            desired_steering_angle -= 2 * math.pi
        elif desired_steering_angle < -math.pi:
            desired_steering_angle += 2 * math.pi
        
        # 급격한 회전시 속도 조절
        if desired_steering_angle > math.pi/6 or desired_steering_angle < -math.pi/6:
            sign = 1 if desired_steering_angle > 0 else -1
            desired_steering_angle = sign * math.pi/4
            v = 0.0
        
        return v, desired_steering_angle, index
    
    def odom_callback(self, msg):
        """
        로봇의 현재 위치 및 자세 정보 업데이트
        """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # 쿼터니언을 오일러 각으로 변환
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        
        self.current_yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        
        # 로봇 초기화 (첫 번째 odom 메시지 수신시)
        if not self.robot_initialized:
            self.robot_initial_x = self.current_x
            self.robot_initial_y = self.current_y
            self.robot_initial_yaw = self.current_yaw
            self.initialize_robot_position()
    
    def timer_callback(self):
        """
        주기적으로 경로 추적 제어 실행
        """
        if not self.is_navigating:
            return
        
        # 목표점에 도달했는지 확인
        if self.path_index >= len(self.smooth_path) - 1:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            self.is_navigating = False
            self.get_logger().info('목표점에 도달했습니다!')
            return
        
        # Pure Pursuit 제어
        linear_vel, angular_vel, new_index = self.pure_pursuit_control(
            self.current_x, self.current_y, self.current_yaw, 
            self.smooth_path, self.path_index
        )
        
        self.path_index = new_index
        
        # 속도 명령 발행
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.publisher.publish(twist)
    
    def start_navigation(self):
        """
        경로 추적 시작
        """
        self.is_navigating = True
        self.path_index = 0
        self.get_logger().info('경로 추적을 시작합니다!')
    
    def stop_navigation(self):
        """
        경로 추적 중지
        """
        self.is_navigating = False
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info('경로 추적을 중지했습니다!')

def main(args=None):
    rclpy.init(args=args)
    utm_launcher = UTMPathLauncher()
    
    try:
        rclpy.spin(utm_launcher)
    except KeyboardInterrupt:
        utm_launcher.stop_navigation()
    finally:
        utm_launcher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
