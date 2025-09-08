#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
import scipy.interpolate as si
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.qos import QoSProfile
import time

# Pure Pursuit 파라미터 (control.py와 동일한 구조)
lookahead_distance = 1.0  # UTM 좌표 기준 (미터)
speed = 0.1            # 기본 속도 (m/s)

def euler_from_quaternion(x, y, z, w):
    """쿼터니언에서 yaw 각도 추출 (control.py와 동일)"""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def bspline_planning(array, sn):
    """B-Spline 경로 스무딩 (control.py와 동일)"""
    try:
        array = np.array(array)
        x = array[:, 0]
        y = array[:, 1]
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

        ipl_t = np.linspace(0.0, len(x) - 1, sn)
        rx = si.splev(ipl_t, x_list)
        ry = si.splev(ipl_t, y_list)
        path = [(rx[i], ry[i]) for i in range(len(rx))]
    except:
        path = array
    return path

def pure_pursuit(current_x, current_y, current_heading, path, index):
    """Pure Pursuit 알고리즘 (control.py와 동일한 로직)"""
    global lookahead_distance
    closest_point = None
    v = speed
    
    # lookahead_distance보다 먼 경로점 찾기
    for i in range(index, len(path)):
        x = path[i][0]
        y = path[i][1]
        distance = math.hypot(current_x - x, current_y - y)
        if lookahead_distance < distance:
            closest_point = (x, y)
            index = i
            break
    
    # 목표점 설정
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
    
    # 각도 제한 (30도 이상이면 45도로 제한하고 정지)
    if desired_steering_angle > math.pi/6 or desired_steering_angle < -math.pi/6:
        sign = 1 if desired_steering_angle > 0 else -1
        desired_steering_angle = sign * math.pi/4
        v = 0.0
    
    return v, desired_steering_angle, index   # degree 단위로 변환

class UTMPurePursuit(Node):
    def __init__(self):
        super().__init__('utm_pure_pursuit')
        
        # Pure Pursuit 파라미터
        self.waypoint_tolerance = 1.0  # waypoint 도달 판정 거리 (미터)
        
        # 로봇 상태
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Waypoint 설정 (control.py의 goal과 유사)

        self.waypoints = [
            (329983.719725, 4123210.415129),  # 첫 번째 waypoint
            (329977.396338, 4123210.620894),   # 두 번째 waypoint
            (329964.442297, 4123208.130461),   # 세 번째 waypoint
            (329955.384840, 4123213.997123)   # 네 번째 waypoint
        ]
        
        # 초기 위치 설정 (테스트용)
        self.initial_x = 329983.719725
        self.initial_y = 4123210.415129
        self.initial_yaw = 0.0
        
        # 테스트 모드 설정
        self.test_mode = False  # True: 시뮬레이션 모드, False: 실제 토픽 모드
        
        # control.py와 동일한 flag 시스템
        self.flag = 0  # 0: 대기, 1: 경로 생성, 2: 추적 중
        self.path = []
        self.i = 0  # 경로 인덱스
        self.current_waypoint_index = 0  # 현재 추적 중인 waypoint 인덱스
        
        # ROS2 인터페이스 설정 (control.py와 동일한 구조)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 테스트 모드일 때만 현재 위치 발행 (시각화용)
        if self.test_mode:
            self.position_publisher = self.create_publisher(Odometry, '/odometry/global', 10)
        
        # 실제 토픽 구독자 (테스트 모드가 아닐 때만 활성화)
        if not self.test_mode:
            self.subscription = self.create_subscription(
                Odometry,
                '/odometry/global',
                self.info_callback,
                10
            )
            
            self.subscription = self.create_subscription(
                PoseStamped,
                'goal_pose',
                self.goal_pose_callback,
                QoSProfile(depth=10)
            )
        
        # control.py와 동일한 타이머 주기
        timer_period = 0.01  # 100Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 테스트 모드 초기화
        if self.test_mode:
            self.setup_test_mode()
        
        self.get_logger().info("UTM Pure Pursuit 노드가 시작되었습니다.")
        self.get_logger().info(f"테스트 모드: {'ON' if self.test_mode else 'OFF'}")
        self.get_logger().info("Waypoint 추적을 시작합니다...")
        
    def goal_pose_callback(self, msg):
        """목표점 설정 콜백 (control.py와 동일한 구조)"""
        if self.test_mode:
            return
            
        goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"목표점 설정: {goal[0]:.3f}, {goal[1]:.3f}")
        
        # 새로운 waypoint 추가
        self.waypoints.append(goal)
        self.flag = 1  # 경로 생성 시작
    
    def setup_test_mode(self):
        """테스트 모드 설정"""
        self.x = self.initial_x
        self.y = self.initial_y
        self.yaw = self.initial_yaw
        
        self.get_logger().info(f"테스트 모드 초기 위치: X={self.x:.3f}, Y={self.y:.3f}")
        
        # 테스트 모드에서는 바로 경로 생성 시작
        self.flag = 1
    
    def info_callback(self, msg):
        """Odometry 데이터 처리 (control.py와 동일한 구조)"""
        if self.test_mode:
            return
            
        # UTM 좌표 추출
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # 쿼터니언에서 yaw 각도 추출
        self.yaw = euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
    
    def timer_callback(self):
        """메인 제어 루프 (waypoint별 순차 추적)"""
        if self.flag == 1:
            # 현재 waypoint로의 경로 생성
            if self.current_waypoint_index < len(self.waypoints):
                current_waypoint = self.waypoints[self.current_waypoint_index]
                
                # 현재 위치에서 현재 waypoint까지의 경로 생성
                path_points = [(self.x, self.y), current_waypoint]
                
                # B-Spline으로 경로 스무딩
                self.path = bspline_planning(path_points, 50)  # 50개 점으로 스무딩
                
                waypoint_num = self.current_waypoint_index + 1
                self.get_logger().info("🚀 경로 생성 완료!")
                self.get_logger().info(f"   목표: Waypoint {waypoint_num}")
                self.get_logger().info(f"   경로 점 수: {len(self.path)}개")
                self.get_logger().info(f"   시작 위치: ({self.x:.3f}, {self.y:.3f})")
                self.get_logger().info(f"   목표 위치: ({current_waypoint[0]:.3f}, {current_waypoint[1]:.3f})")
                self.get_logger().info("   추적을 시작합니다...")
                
                self.i = 0
                self.flag = 2
        
        elif self.flag == 2:
            # Pure Pursuit 제어 실행
            twist = Twist()
            twist.linear.x, twist.angular.z, self.i = pure_pursuit(
                self.x, self.y, self.yaw, self.path, self.i
            )
            
            # 테스트 모드에서는 시뮬레이션 실행
            if self.test_mode:
                self.simulate_robot_movement()
            
            # 현재 waypoint 도달 판정
            if self.current_waypoint_index < len(self.waypoints):
                current_waypoint = self.waypoints[self.current_waypoint_index]
                distance_to_waypoint = math.sqrt(
                    (self.x - current_waypoint[0])**2 + (self.y - current_waypoint[1])**2
                )
                
                if distance_to_waypoint < 0.5:  # 50cm 이내면 도달로 판정
                    waypoint_num = self.current_waypoint_index + 1
                    self.get_logger().info("=" * 50)
                    self.get_logger().info(f"🎯 Waypoint {waypoint_num} 도달!")
                    self.get_logger().info(f"   위치: ({current_waypoint[0]:.3f}, {current_waypoint[1]:.3f})")
                    self.get_logger().info(f"   거리: {distance_to_waypoint:.2f}m")
                    self.get_logger().info(f"   진행률: {waypoint_num}/{len(self.waypoints)}")
                    self.get_logger().info("=" * 50)
                    
                    self.current_waypoint_index += 1
                    
                    if self.current_waypoint_index >= len(self.waypoints):
                        # 모든 waypoint 완료
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.flag = 0
                        self.get_logger().info("🎉 모든 waypoint 완료!")
                        self.get_logger().info("✅ 미션 성공!")
                        self.get_logger().info("새로운 목표점을 기다립니다...")
                    else:
                        # 다음 waypoint로 경로 생성
                        next_waypoint = self.waypoints[self.current_waypoint_index]
                        self.get_logger().info(f"➡️ 다음 목표: Waypoint {self.current_waypoint_index + 1}")
                        self.get_logger().info(f"   위치: ({next_waypoint[0]:.3f}, {next_waypoint[1]:.3f})")
                        self.flag = 1
            
            self.publisher.publish(twist)
            
            # 테스트 모드에서는 현재 위치 발행 (시각화용)
            if self.test_mode:
                self.publish_current_position()
    
    def publish_current_position(self):
        """테스트 모드에서 현재 위치를 /odometry/global 토픽으로 발행"""
        if not self.test_mode:
            return
            
        # Odometry 메시지 생성
        odom_msg = Odometry()
        
        # 헤더 설정
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"
        
        # 위치 설정
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # yaw 각도를 쿼터니언으로 변환
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        
        # 속도 설정 (현재 cmd_vel 값 사용)
        odom_msg.twist.twist.linear.x = 0.0  # 실제 속도는 0으로 설정
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        
        # 공분산 설정 (단위 행렬)
        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.twist.covariance = [0.0] * 36
        
        # 발행
        self.position_publisher.publish(odom_msg)
    
    def simulate_robot_movement(self):
        """테스트 모드에서 로봇 움직임 시뮬레이션"""
        if not self.test_mode or self.flag != 2:
            return
        
        # 간단한 시뮬레이션: 현재 cmd_vel에 따라 위치 업데이트
        dt = 0.01  # 타이머 주기 (100Hz)
        
        # 마지막 cmd_vel 값 사용 (실제로는 이전 프레임의 값)
        # 여기서는 간단히 경로를 따라 이동하는 것으로 시뮬레이션
        if self.i < len(self.path):
            target_x, target_y = self.path[self.i]
            
            # 목표점 방향으로 이동
            dx = target_x - self.x
            dy = target_y - self.y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance > 0.01:  # 1cm 이상 떨어져 있으면
                # 목표점 방향으로 이동
                move_distance = min(0.01, distance)  # 최대 1cm씩 이동
                self.x += (dx / distance) * move_distance
                self.y += (dy / distance) * move_distance
                
                # yaw 각도 업데이트
                self.yaw = math.atan2(dy, dx)


def main(args=None):
    rclpy.init(args=args)
    
    # UTM Pure Pursuit 노드 생성
    utm_pure_pursuit = UTMPurePursuit()
    
    try:
        rclpy.spin(utm_pure_pursuit)
    except KeyboardInterrupt:
        utm_pure_pursuit.get_logger().info("노드 종료 중...")
    finally:
        utm_pure_pursuit.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()