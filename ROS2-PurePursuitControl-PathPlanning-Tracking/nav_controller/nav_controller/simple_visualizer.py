#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
import math
import time

class SimpleVisualizer(Node):
    def __init__(self):
        super().__init__('simple_visualizer')
        
        # 데이터 저장
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Waypoint 데이터 (utm_pure_pursuit_test.py와 동일)
        self.waypoints = [
            (329983.719725, 4123210.415129),
            (329977.396338, 4123210.620894),
            (329964.442297, 4123208.130461),
            (329955.384840, 4123213.997123)
        ]
        
        # 보간된 경로
        self.interpolated_path = []
        
        # 속도 데이터 저장
        self.vel_time = []
        self.vel_data = []
        self.ang_vel_data = []
        self.start_time = time.time()
        
        # 구독자 설정
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/global_utm', self.odom_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/pure_pursuit/path', self.path_callback, 10)
        
        # matplotlib 설정
        plt.ion()  # 인터랙티브 모드
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(12, 10))
        self.fig.suptitle('Pure Pursuit Visualization', fontsize=16)
        
        # 1. 경로 플롯
        self.ax1.set_title('Path and Waypoints')
        self.ax1.set_xlabel('UTM X (m)')
        self.ax1.set_ylabel('UTM Y (m)')
        self.ax1.grid(True)
        self.ax1.set_aspect('equal')
        
        # Waypoint 그리기
        waypoint_x = [wp[0] for wp in self.waypoints]
        waypoint_y = [wp[1] for wp in self.waypoints]
        self.ax1.plot(waypoint_x, waypoint_y, 'ro-', markersize=8, linewidth=2, label='Waypoints')
        
        # Waypoint 번호 표시
        for i, (x, y) in enumerate(self.waypoints):
            self.ax1.annotate(f'WP{i+1}', (x, y), xytext=(5, 5), 
                            textcoords='offset points', fontsize=10, fontweight='bold')
        
        # 경로와 로봇 위치 플롯 초기화
        self.path_line, = self.ax1.plot([], [], 'b-', linewidth=2, label='Interpolated Path')
        self.robot_plot, = self.ax1.plot([], [], 'go', markersize=10, label='Robot')
        
        # 로봇 방향 화살표 초기화 (FancyArrowPatch 사용)
        from matplotlib.patches import FancyArrowPatch
        self.robot_arrow = FancyArrowPatch((0, 0), (1, 0), 
                                         arrowstyle='->', color='green', lw=3, alpha=0.8)
        self.robot_arrow.set_visible(False)  # 처음에는 숨김
        self.ax1.add_patch(self.robot_arrow)
        
        # 목표 방향 화살표 초기화 (FancyArrowPatch 사용)
        self.target_arrow = FancyArrowPatch((0, 0), (1, 0), 
                                          arrowstyle='->', color='red', lw=3, alpha=0.8)
        self.target_arrow.set_visible(False)  # 처음에는 숨김
        self.ax1.add_patch(self.target_arrow)
        
        # 초기 축 범위 설정
        margin = 20.0
        self.ax1.set_xlim(min(waypoint_x) - margin, max(waypoint_x) + margin)
        self.ax1.set_ylim(min(waypoint_y) - margin, max(waypoint_y) + margin)
        
        # 범례에 화살표 설명 추가
        green_arrow_legend = FancyArrowPatch((0, 0), (1, 0), arrowstyle='->', color='green', lw=3)
        red_arrow_legend = FancyArrowPatch((0, 0), (1, 0), arrowstyle='->', color='red', lw=3)
        self.ax1.legend([green_arrow_legend, red_arrow_legend], ['Robot Direction', 'Target Direction'], loc='upper right')
        
        # 2. 선속도 그래프
        self.ax2.set_title('Linear Velocity')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Velocity (m/s)')
        self.ax2.grid(True)
        self.vel_line, = self.ax2.plot([], [], 'b-', linewidth=2)
        
        # 3. 각속도 그래프
        self.ax3.set_title('Angular Velocity')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Angular Velocity (rad/s)')
        self.ax3.grid(True)
        self.ang_vel_line, = self.ax3.plot([], [], 'r-', linewidth=2)
        
        # 4. 상태 정보
        self.ax4.set_title('Status')
        self.ax4.axis('off')
        self.status_text = self.ax4.text(0.05, 0.95, '', transform=self.ax4.transAxes, 
                                       fontsize=12, verticalalignment='top',
                                       bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        
        # 애니메이션 시작
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100, blit=False)
        
        self.get_logger().info("Simple Visualizer가 시작되었습니다.")
        self.get_logger().info("Waypoint와 경로를 시각화합니다.")
    
    def odom_callback(self, msg):
        """Odometry 콜백"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # 쿼터니언에서 yaw 추출
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        self.robot_yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    
    def cmd_vel_callback(self, msg):
        """cmd_vel 콜백"""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
        
        # 시간과 속도 데이터 저장
        current_time = time.time() - self.start_time
        self.vel_time.append(current_time)
        self.vel_data.append(self.linear_vel)
        self.ang_vel_data.append(self.angular_vel)
        
        # 최대 100개 포인트만 유지
        if len(self.vel_time) > 100:
            self.vel_time.pop(0)
            self.vel_data.pop(0)
            self.ang_vel_data.pop(0)
    
    def path_callback(self, msg):
        """Path 콜백"""
        self.get_logger().info(f"Path 메시지 수신: {len(msg.poses)}개 점")
        
        path_points = []
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            path_points.append((x, y))
        
        self.interpolated_path = path_points
        self.get_logger().info(f"경로 업데이트 완료: {len(path_points)}개 점")
        
        # 축 범위 업데이트
        if path_points:
            all_x = [point[0] for point in path_points]
            all_y = [point[1] for point in path_points]
            
            # waypoint 추가
            waypoint_x = [wp[0] for wp in self.waypoints]
            waypoint_y = [wp[1] for wp in self.waypoints]
            all_x.extend(waypoint_x)
            all_y.extend(waypoint_y)
            
            # 로봇 위치 추가
            if self.robot_x != 0 or self.robot_y != 0:
                all_x.append(self.robot_x)
                all_y.append(self.robot_y)
            
            # 축 범위 설정
            margin = 20.0
            self.ax1.set_xlim(min(all_x) - margin, max(all_x) + margin)
            self.ax1.set_ylim(min(all_y) - margin, max(all_y) + margin)
    
    def update_plot(self, frame):
        """플롯 업데이트"""
        # 1. 경로 업데이트
        if self.interpolated_path:
            path_x = [point[0] for point in self.interpolated_path]
            path_y = [point[1] for point in self.interpolated_path]
            self.path_line.set_data(path_x, path_y)
        
        # 로봇 위치 업데이트
        self.robot_plot.set_data([self.robot_x], [self.robot_y])
        
        # 로봇 방향 화살표 업데이트 (현재 yaw 방향)
        arrow_length = 5.0
        
        # 로봇이 (0,0)이 아닌 유효한 위치에 있을 때만 화살표 표시
        if self.robot_x != 0.0 or self.robot_y != 0.0:
            # 초록 화살표 (현재 방향)
            robot_end_x = self.robot_x + arrow_length * math.cos(self.robot_yaw)
            robot_end_y = self.robot_y + arrow_length * math.sin(self.robot_yaw)
            
            # FancyArrowPatch 위치 업데이트
            self.robot_arrow.set_positions((self.robot_x, self.robot_y), (robot_end_x, robot_end_y))
            self.robot_arrow.set_visible(True)
            
            # 빨간 화살표 (목표 방향)
            # angular.z는 디그리 단위이므로 라디안으로 변환
            target_angle_rad = math.radians(self.angular_vel)
            target_end_x = self.robot_x + arrow_length * math.cos(target_angle_rad)
            target_end_y = self.robot_y + arrow_length * math.sin(target_angle_rad)
            
            # FancyArrowPatch 위치 업데이트
            self.target_arrow.set_positions((self.robot_x, self.robot_y), (target_end_x, target_end_y))
            self.target_arrow.set_visible(True)
            
            # 디버깅: 화살표 위치 로그 (처음 몇 번만)
            if frame < 3:
                self.get_logger().info(f"화살표 위치 업데이트 (프레임 {frame}):")
                self.get_logger().info(f"  로봇 위치: ({self.robot_x:.3f}, {self.robot_y:.3f})")
                self.get_logger().info(f"  초록 화살표: ({self.robot_x:.3f}, {self.robot_y:.3f}) -> ({robot_end_x:.3f}, {robot_end_y:.3f})")
                self.get_logger().info(f"  빨간 화살표: ({self.robot_x:.3f}, {self.robot_y:.3f}) -> ({target_end_x:.3f}, {target_end_y:.3f})")
                self.get_logger().info(f"  현재 yaw: {math.degrees(self.robot_yaw):.1f}°, 목표 각도: {self.angular_vel:.1f}°")
        else:
            # 로봇 위치가 없으면 화살표 숨기기
            self.robot_arrow.set_visible(False)
            self.target_arrow.set_visible(False)
        
        # 2. 속도 그래프 업데이트
        if self.vel_time:
            self.vel_line.set_data(self.vel_time, self.vel_data)
            self.ax2.set_xlim(max(0, self.vel_time[-1] - 10), self.vel_time[-1] + 1)
            if self.vel_data:
                self.ax2.set_ylim(min(self.vel_data) - 1, max(self.vel_data) + 1)
        
        # 3. 각속도 그래프 업데이트
        if self.vel_time:
            self.ang_vel_line.set_data(self.vel_time, self.ang_vel_data)
            self.ax3.set_xlim(max(0, self.vel_time[-1] - 10), self.vel_time[-1] + 1)
            if self.ang_vel_data:
                self.ax3.set_ylim(min(self.ang_vel_data) - 0.5, max(self.ang_vel_data) + 0.5)
        
        # 4. 상태 정보 업데이트
        current_time = time.time() - self.start_time
        status_info = f"""Status:
        
Robot Position:
  X: {self.robot_x:.3f} m
  Y: {self.robot_y:.3f} m
  Current Yaw: {math.degrees(self.robot_yaw):.1f}°

Direction Arrows:
  Green Arrow: Current Robot Direction
  Red Arrow: Target Direction (from angular.z)
  Target Angle: {self.angular_vel:.1f}°

Current Commands:
  Linear Vel: {self.linear_vel:.3f} m/s
  Angular.z: {self.angular_vel:.1f}° (angle, not angular velocity)

Runtime: {current_time:.1f} s
Path Points: {len(self.interpolated_path)}
"""
        self.status_text.set_text(status_info)
        
        return [self.path_line, self.robot_plot, self.vel_line, self.ang_vel_line, self.status_text]

def main(args=None):
    rclpy.init(args=args)
    
    visualizer = SimpleVisualizer()
    
    try:
        # matplotlib과 ROS2를 함께 실행
        while rclpy.ok():
            rclpy.spin_once(visualizer, timeout_sec=0.1)
            plt.pause(0.01)
    except KeyboardInterrupt:
        visualizer.get_logger().info("시각화 노드 종료 중...")
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
