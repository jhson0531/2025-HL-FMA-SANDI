#!/usr/bin/env python3

"""
UTM Pure Pursuit 실시간 시각화 프로그램
- 생성된 path를 실시간으로 표시
- 로봇이 path를 따라가는 것을 실시간으로 표시
- PNG 저장 없이 실시간 시각화만 제공
"""

import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import time
import threading

class RealtimeVisualizer(Node):
    def __init__(self):
        super().__init__('realtime_visualizer')
        
        # 시각화 데이터
        self.robot_x_history = []
        self.robot_y_history = []
        self.robot_yaw_history = []
        self.cmd_vel_linear = []
        self.cmd_vel_angular = []
        self.time_history = []
        
        # Waypoint 정보 (utm_pure_pursuit.py와 동일)
        self.waypoints = [
            (330030.501943, 4123118.148129),  # 첫 번째 waypoint
            (330024.601050, 4123120.376601)   # 두 번째 waypoint
        ]
        self.initial_pos = (330034.491650, 4123117.956354)
        
        # 현재 로봇 상태
        self.current_x = self.initial_pos[0]
        self.current_y = self.initial_pos[1]
        # 첫 번째 waypoint 방향으로 초기 yaw 설정
        if len(self.waypoints) > 0:
            first_waypoint = self.waypoints[0]
            self.current_yaw = math.atan2(
                first_waypoint[1] - self.current_y, 
                first_waypoint[0] - self.current_x
            )
        else:
            self.current_yaw = 0.0
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        
        # 경로 정보
        self.generated_path = []
        self.path_received = False
        
        # 마지막 업데이트 시간
        self.last_update_time = time.time()
        self.start_time = time.time()
        
        # cmd_vel 메시지 수신 상태
        self.cmd_vel_received = False
        self.last_cmd_vel_time = 0.0
        self.pure_pursuit_active = False
        
        # odometry 데이터 수신 상태
        self.odometry_received = False
        
        # ROS2 구독자 설정
        self.setup_subscribers()
        
        # 시각화 설정
        self.setup_visualization()
        
        # 데이터 수집 시작
        self.data_collection_timer = self.create_timer(0.1, self.collect_data)
        
        self.get_logger().info("실시간 시각화가 시작되었습니다.")
        self.get_logger().info("Pure Pursuit 노드가 실행되면 자동으로 시각화를 시작합니다.")
    
    def setup_subscribers(self):
        """ROS2 구독자 설정"""
        # cmd_vel 구독 (실제 제어 명령)
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            QoSProfile(depth=10)
        )
        
        # odometry 구독 (실제 위치 - 테스트 모드에서는 사용되지 않음)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.odometry_callback,
            QoSProfile(depth=10)
        )
    
    def setup_visualization(self):
        """시각화 설정"""
        # matplotlib 설정
        plt.ion()  # 인터랙티브 모드
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(14, 12))
        self.fig.suptitle('UTM Pure Pursuit 실시간 시각화', fontsize=16, fontweight='bold')
        
        # 첫 번째 서브플롯: 경로 추적
        self.ax1.set_title('경로 추적 및 로봇 움직임', fontsize=14, fontweight='bold')
        self.ax1.set_xlabel('UTM X (m)', fontsize=12)
        self.ax1.set_ylabel('UTM Y (m)', fontsize=12)
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_aspect('equal')
        
        # Waypoint 표시
        waypoint_x = [wp[0] for wp in self.waypoints]
        waypoint_y = [wp[1] for wp in self.waypoints]
        self.ax1.plot(waypoint_x, waypoint_y, 'ro', markersize=12, label='Waypoints', markeredgecolor='darkred', markeredgewidth=2)
        self.ax1.plot(self.initial_pos[0], self.initial_pos[1], 'go', markersize=12, label='시작점', markeredgecolor='darkgreen', markeredgewidth=2)
        
        # Waypoint 번호 표시
        for i, (x, y) in enumerate(self.waypoints):
            self.ax1.annotate(f'WP{i+1}', (x, y), xytext=(8, 8), textcoords='offset points', 
                            fontsize=10, fontweight='bold', color='darkred')
        
        # 두 번째 서브플롯: 선속도
        self.ax2.set_title('제어 선속도', fontsize=14, fontweight='bold')
        self.ax2.set_xlabel('시간 (초)', fontsize=12)
        self.ax2.set_ylabel('선속도 (m/s)', fontsize=12)
        self.ax2.grid(True, alpha=0.3)
        
        # 세 번째 서브플롯: 각속도
        self.ax3.set_title('제어 각속도', fontsize=14, fontweight='bold')
        self.ax3.set_xlabel('시간 (초)', fontsize=12)
        self.ax3.set_ylabel('각속도 (rad/s)', fontsize=12)
        self.ax3.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    def odometry_callback(self, msg):
        """Odometry 데이터 처리 (실제 위치 데이터 사용)"""
        # 실제 위치 데이터 사용 (cmd_vel 추정 대신)
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # 쿼터니언에서 yaw 추출
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        self.current_yaw = math.atan2(t3, t4)
        
        # odometry 데이터를 받았음을 표시
        self.odometry_received = True
    
    def cmd_vel_callback(self, msg):
        """cmd_vel 데이터 처리 및 위치 추정"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        self.last_cmd_vel_time = current_time
        
        # cmd_vel 수신 상태 업데이트
        self.cmd_vel_received = True
        
        # Pure Pursuit 활성 상태 확인 (속도가 0이 아니거나 각속도가 0이 아닌 경우)
        if abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001:
            self.pure_pursuit_active = True
        else:
            # 5초간 속도가 0이면 비활성 상태로 간주
            if current_time - self.last_cmd_vel_time > 5.0:
                self.pure_pursuit_active = False
        
        # 속도 업데이트
        self.current_linear_vel = msg.linear.x
        self.current_angular_vel = msg.angular.z
        
        # 위치 추정 비활성화 (odometry 데이터 사용)
        # odometry 데이터가 있으면 cmd_vel로부터 위치 추정하지 않음
        if not self.odometry_received and self.pure_pursuit_active and dt > 0 and dt < 1.0:
            # 각도 업데이트
            self.current_yaw += self.current_angular_vel * dt
            
            # 위치 업데이트
            dx = self.current_linear_vel * math.cos(self.current_yaw) * dt
            dy = self.current_linear_vel * math.sin(self.current_yaw) * dt
            
            self.current_x += dx
            self.current_y += dy
    
    def collect_data(self):
        """데이터 수집"""
        current_time = time.time() - self.start_time
        
        # cmd_vel 메시지가 2초 이상 없으면 정지 상태로 간주
        if current_time - self.last_cmd_vel_time > 2.0:
            self.current_linear_vel = 0.0
            self.current_angular_vel = 0.0
            self.pure_pursuit_active = False
        
        # 데이터 저장
        self.robot_x_history.append(self.current_x)
        self.robot_y_history.append(self.current_y)
        self.robot_yaw_history.append(self.current_yaw)
        self.cmd_vel_linear.append(self.current_linear_vel)
        self.cmd_vel_angular.append(self.current_angular_vel)
        self.time_history.append(current_time)
        
        # 최대 2000개 데이터 포인트만 유지
        max_points = 2000
        if len(self.robot_x_history) > max_points:
            self.robot_x_history = self.robot_x_history[-max_points:]
            self.robot_y_history = self.robot_y_history[-max_points:]
            self.robot_yaw_history = self.robot_yaw_history[-max_points:]
            self.cmd_vel_linear = self.cmd_vel_linear[-max_points:]
            self.cmd_vel_angular = self.cmd_vel_angular[-max_points:]
            self.time_history = self.time_history[-max_points:]
        
        # 실시간 업데이트
        self.update_visualization()
    
    def update_visualization(self):
        """시각화 업데이트"""
        if len(self.robot_x_history) < 2:
            return
        
        # 첫 번째 서브플롯: 경로 추적
        self.ax1.clear()
        
        # 상태에 따른 제목 설정
        if not self.odometry_received:
            title = '경로 추적 및 로봇 움직임 (위치 데이터 대기 중...)'
        elif not self.cmd_vel_received:
            title = '경로 추적 및 로봇 움직임 (Pure Pursuit 대기 중...)'
        elif not self.pure_pursuit_active:
            title = '경로 추적 및 로봇 움직임 (Pure Pursuit 정지됨)'
        else:
            title = '경로 추적 및 로봇 움직임 (Pure Pursuit 활성)'
        
        self.ax1.set_title(title, fontsize=14, fontweight='bold')
        self.ax1.set_xlabel('UTM X (m)', fontsize=12)
        self.ax1.set_ylabel('UTM Y (m)', fontsize=12)
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_aspect('equal')
        
        # Waypoint 표시
        waypoint_x = [wp[0] for wp in self.waypoints]
        waypoint_y = [wp[1] for wp in self.waypoints]
        self.ax1.plot(waypoint_x, waypoint_y, 'ro', markersize=12, label='Waypoints', markeredgecolor='darkred', markeredgewidth=2)
        self.ax1.plot(self.initial_pos[0], self.initial_pos[1], 'go', markersize=12, label='시작점', markeredgecolor='darkgreen', markeredgewidth=2)
        
        # Waypoint 번호 표시
        for i, (x, y) in enumerate(self.waypoints):
            self.ax1.annotate(f'WP{i+1}', (x, y), xytext=(8, 8), textcoords='offset points', 
                            fontsize=10, fontweight='bold', color='darkred')
        
        # 생성된 경로 표시 (각 waypoint별로)
        if self.pure_pursuit_active:
            try:
                # 현재 위치에서 각 waypoint까지의 경로를 표시
                current_pos = (self.current_x, self.current_y)
                
                # 첫 번째 waypoint로의 경로
                if len(self.waypoints) > 0:
                    path_points = [current_pos, self.waypoints[0]]
                    path_array = np.array(path_points)
                    
                    if len(path_points) > 1:
                        x_coords = path_array[:, 0]
                        y_coords = path_array[:, 1]
                        
                        # 간단한 선형 보간으로 경로 생성
                        t = np.linspace(0, 1, 50)
                        path_x = np.interp(t, np.linspace(0, 1, len(x_coords)), x_coords)
                        path_y = np.interp(t, np.linspace(0, 1, len(y_coords)), y_coords)
                        
                        self.ax1.plot(path_x, path_y, 'b--', linewidth=2, alpha=0.7, label='생성된 경로')
                
                # 두 번째 waypoint로의 경로 (첫 번째 waypoint 도달 후)
                if len(self.waypoints) > 1 and len(self.robot_x_history) > 50:
                    # 첫 번째 waypoint에서 두 번째 waypoint로의 경로
                    path_points = [self.waypoints[0], self.waypoints[1]]
                    path_array = np.array(path_points)
                    
                    if len(path_points) > 1:
                        x_coords = path_array[:, 0]
                        y_coords = path_array[:, 1]
                        
                        # 간단한 선형 보간으로 경로 생성
                        t = np.linspace(0, 1, 50)
                        path_x = np.interp(t, np.linspace(0, 1, len(x_coords)), x_coords)
                        path_y = np.interp(t, np.linspace(0, 1, len(y_coords)), y_coords)
                        
                        self.ax1.plot(path_x, path_y, 'b--', linewidth=2, alpha=0.7)
            except:
                pass
        
        # 로봇 경로 표시 (Pure Pursuit가 활성일 때만)
        if len(self.robot_x_history) > 1 and self.pure_pursuit_active:
            self.ax1.plot(self.robot_x_history, self.robot_y_history, 'b-', linewidth=3, label='로봇 실제 경로')
            
            # 현재 위치 표시
            self.ax1.plot(self.current_x, self.current_y, 'bo', markersize=10, label='현재 위치', markeredgecolor='darkblue', markeredgewidth=2)
            
            # 로봇 방향 표시 (화살표)
            arrow_length = 3.0
            dx = arrow_length * math.cos(self.current_yaw)
            dy = arrow_length * math.sin(self.current_yaw)
            self.ax1.arrow(self.current_x, self.current_y, dx, dy, 
                          head_width=1.5, head_length=1.5, fc='blue', ec='blue', alpha=0.8)
        else:
            # 시작점만 표시
            self.ax1.plot(self.current_x, self.current_y, 'go', markersize=10, label='현재 위치', markeredgecolor='darkgreen', markeredgewidth=2)
        
        self.ax1.legend(loc='upper right', fontsize=10)
        
        # 두 번째 서브플롯: 선속도
        self.ax2.clear()
        self.ax2.set_title('제어 선속도', fontsize=14, fontweight='bold')
        self.ax2.set_xlabel('시간 (초)', fontsize=12)
        self.ax2.set_ylabel('선속도 (m/s)', fontsize=12)
        self.ax2.grid(True, alpha=0.3)
        if len(self.time_history) > 1:
            self.ax2.plot(self.time_history, self.cmd_vel_linear, 'g-', linewidth=2, label='선속도')
            self.ax2.legend(loc='upper right', fontsize=10)
        
        # 세 번째 서브플롯: 각속도
        self.ax3.clear()
        self.ax3.set_title('제어 각속도', fontsize=14, fontweight='bold')
        self.ax3.set_xlabel('시간 (초)', fontsize=12)
        self.ax3.set_ylabel('각속도 (rad/s)', fontsize=12)
        self.ax3.grid(True, alpha=0.3)
        if len(self.time_history) > 1:
            self.ax3.plot(self.time_history, self.cmd_vel_angular, 'r-', linewidth=2, label='각속도')
            self.ax3.legend(loc='upper right', fontsize=10)
        
        plt.tight_layout()
        plt.pause(0.01)
    
    def print_statistics(self):
        """통계 정보 출력"""
        if len(self.robot_x_history) > 1:
            total_distance = 0.0
            for i in range(1, len(self.robot_x_history)):
                dx = self.robot_x_history[i] - self.robot_x_history[i-1]
                dy = self.robot_y_history[i] - self.robot_y_history[i-1]
                total_distance += math.sqrt(dx*dx + dy*dy)
            
            max_linear_vel = max(self.cmd_vel_linear) if self.cmd_vel_linear else 0
            max_angular_vel = max([abs(v) for v in self.cmd_vel_angular]) if self.cmd_vel_angular else 0
            
            print("\n" + "="*60)
            print("UTM Pure Pursuit 실시간 시각화 통계")
            print("="*60)
            print(f"총 이동 거리: {total_distance:.2f} m")
            print(f"최대 선속도: {max_linear_vel:.3f} m/s")
            print(f"최대 각속도: {max_angular_vel:.3f} rad/s")
            print(f"총 실행 시간: {self.time_history[-1]:.1f} 초")
            print(f"데이터 포인트 수: {len(self.robot_x_history)}")
            print(f"Pure Pursuit 상태: {'활성' if self.pure_pursuit_active else '비활성'}")
            print("="*60)


def main(args=None):
    rclpy.init(args=args)
    
    # 시각화 노드 생성
    visualizer = RealtimeVisualizer()
    
    try:
        print("\n" + "="*60)
        print("UTM Pure Pursuit 실시간 시각화가 시작되었습니다!")
        print("="*60)
        print("1. 다른 터미널에서 'ros2 run nav_controller utm_pure_pursuit' 실행")
        print("2. 시각화 창에서 실시간 경로 추적 확인")
        print("3. 생성된 경로와 로봇의 실제 경로를 비교")
        print("4. Ctrl+C로 종료")
        print("="*60)
        
        rclpy.spin(visualizer)
        
    except KeyboardInterrupt:
        print("\n시각화를 종료합니다...")
        visualizer.print_statistics()
        
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()
        plt.close('all')


if __name__ == '__main__':
    main()
