#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import numpy as np
import utm
import threading
import sys
import select
import tty
import termios
import math
import os
import serial
import time

class IMUNorth(Node):
   
    def __init__(self):
        super().__init__('imu_north_node')
        
        # QoS 설정
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, 
                             history=QoSHistoryPolicy.KEEP_LAST, 
                             durability=QoSDurabilityPolicy.VOLATILE, 
                             depth=1)
        
        # GPS fix 토픽 구독자 생성
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/ublox_gps_node/fix',
            self.gps_callback,
            qos_profile
        )
        
        # 시리얼 포트 초기화
        self.serial_port = None
        self.init_serial_port()
        
        # 현재 GPS 좌표 저장 변수
        self.current_lat = None
        self.current_lon = None
        self.current_alt = None
        
        # UTM 좌표 저장 배열 (2차원, 크기 2x2)
        self.utm_coordinates = np.zeros((2, 2))
        
        # Enter키 입력 카운터
        self.enter_count = 0
        
        # 로봇 움직임 제어 상태
        self.robot_moving = False
        self.movement_timer = None
        
        # 키보드 입력 감지를 위한 스레드 시작
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        self.get_logger().info('IMU North 노드가 시작되었습니다.')
        self.get_logger().info('/ublox_gps_node/fix 토픽을 구독 중...')
        self.get_logger().info('Enter키를 두 번 눌러서 두 개의 GPS 좌표를 UTM으로 변환하여 저장합니다.')
        self.get_logger().info('첫 번째 Enter키 전까지 s0p0, 첫 번째와 두 번째 사이에 1초간 s0p50, 두 번째 후 s0p0')
        
        # 초기 로봇 정지
        self.send_serial_command(0, 0)
    
    def gps_callback(self, msg):
        """
        GPS fix 메시지 콜백 함수
        """
        # GPS 상태 확인
        if msg.status.status < 0:
            self.get_logger().warn('GPS 상태가 유효하지 않습니다.')
            return
        
        # 현재 GPS 좌표 업데이트
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude
        
        # 로그 출력 (첫 번째 데이터일 때만)
        if self.current_lat is not None:
            self.get_logger().info(
                f'GPS 위치 업데이트: 위도={self.current_lat:.6f}, 경도={self.current_lon:.6f}, 고도={self.current_alt:.2f}m',
                throttle_duration_sec=5.0  # 5초마다 한 번씩만 출력
            )
    
    def latlon_to_utm(self, lat, lon):
        """
        위도/경도를 UTM 좌표로 변환
        
        Args:
            lat: 위도 (도 단위)
            lon: 경도 (도 단위)
        
        Returns:
            tuple: (easting, northing) UTM 좌표
        """
        try:
            utm_x, utm_y, zone_number, zone_letter = utm.from_latlon(lat, lon)
            return utm_x, utm_y
        except Exception as e:
            self.get_logger().error(f'UTM 변환 오류: {e}')
            return None, None
    
    def init_serial_port(self):
        """
        시리얼 포트 초기화
        """
        try:
            self.serial_port = serial.Serial(
                port='/dev/ttyACM1',
                baudrate=115200,
                timeout=1
            )
            self.get_logger().info('시리얼 포트 /dev/ttyACM1 초기화 완료 (115200 baud)')
        except Exception as e:
            self.get_logger().error(f'시리얼 포트 초기화 실패: {e}')
            self.serial_port = None
    
    def send_serial_command(self, steering, speed):
        """
        시리얼 포트로 명령 전송
        
        Args:
            steering: 조향각 (0)
            speed: 속도 (0 또는 50)
        """
        if self.serial_port is not None and self.serial_port.is_open:
            try:
                command = f"s{steering}p{speed}\n"
                self.serial_port.write(command.encode())
                self.get_logger().info(f'시리얼 명령 전송: {command.strip()}')
            except Exception as e:
                self.get_logger().error(f'시리얼 명령 전송 실패: {e}')
        else:
            self.get_logger().warn('시리얼 포트가 열려있지 않습니다.')
    
    def close_serial_port(self):
        """
        시리얼 포트 종료
        """
        if self.serial_port is not None and self.serial_port.is_open:
            try:
                self.serial_port.close()
                self.get_logger().info('시리얼 포트가 종료되었습니다.')
            except Exception as e:
                self.get_logger().error(f'시리얼 포트 종료 실패: {e}')
    
    def stop_robot(self):
        """
        로봇을 정지시키는 함수 (s0p0)
        """
        self.send_serial_command(0, 0)
        self.robot_moving = False
        self.get_logger().info('로봇 정지 (s0p0)')
    
    def move_robot_forward(self):
        """
        로봇을 앞으로 이동시키는 함수 (s0p20)
        """
        self.send_serial_command(0, 20)
        self.robot_moving = True
        self.get_logger().info('로봇 전진 시작 (s0p20)')
    
    def start_movement_timer(self):
        """
        1초 후 로봇을 정지시키는 타이머 시작
        """
        if self.movement_timer is not None:
            self.movement_timer.cancel()
        
        self.movement_timer = self.create_timer(1.0, self.stop_robot_after_movement)
    
    def stop_robot_after_movement(self):
        """
        타이머 콜백: 1초 후 로봇 정지
        """
        self.stop_robot()
        if self.movement_timer is not None:
            self.movement_timer.cancel()
            self.movement_timer = None
    
    def calculate_angle_between_points(self):
        """
        두 UTM 좌표점의 차이를 이용하여 각도(theta)를 계산
        
        Returns:
            float: 각도 (라디안 단위)
        """
        # 두 점의 좌표
        x1, y1 = self.utm_coordinates[0, 0], self.utm_coordinates[0, 1]  # 첫 번째 점
        x2, y2 = self.utm_coordinates[1, 0], self.utm_coordinates[1, 1]  # 두 번째 점
        
        # 두 점의 차이 계산
        dx = x2 - x1  # Easting 차이
        dy = y2 - y1  # Northing 차이
        
        # atan2를 사용하여 각도 계산 (라디안)
        theta = math.atan2(dy, dx)

        if theta < 0:
            theta += 2*math.pi
        
        self.get_logger().info(f'첫 번째 점: ({x1:.2f}, {y1:.2f})')
        self.get_logger().info(f'두 번째 점: ({x2:.2f}, {y2:.2f})')
        self.get_logger().info(f'차이: dx={dx:.2f}, dy={dy:.2f}')
        self.get_logger().info(f'계산된 각도: {theta:.6f} 라디안 ({math.degrees(theta):.2f} 도)')
        
        return theta
    
    def save_angle_to_file(self, theta):
        """
        계산된 각도를 offset_angle.txt 파일에 저장
        
        Args:
            theta: 각도 (라디안 단위)
        """
        try:
            # 절대경로로 파일 저장 (daesun 수정하셈)
            file_path = '/home/jh/ros2_workspace/src/offset_angle.txt'
            
            with open(file_path, 'w') as f:
                f.write(f"# UTM 좌표를 이용한 각도 계산 결과\n")
                f.write(f"# 첫 번째 점: ({self.utm_coordinates[0, 0]:.6f}, {self.utm_coordinates[0, 1]:.6f})\n")
                f.write(f"# 두 번째 점: ({self.utm_coordinates[1, 0]:.6f}, {self.utm_coordinates[1, 1]:.6f})\n")
                f.write(f"# 차이: dx={self.utm_coordinates[1, 0] - self.utm_coordinates[0, 0]:.6f}, dy={self.utm_coordinates[1, 1] - self.utm_coordinates[0, 1]:.6f}\n")
                f.write(f"# 계산된 각도 (라디안): {theta:.10f}\n")
                f.write(f"# 계산된 각도 (도): {math.degrees(theta):.10f}\n")
                f.write(f"theta={theta:.10f}\n")
            
            self.get_logger().info(f'각도가 {file_path}에 저장되었습니다.')
            self.get_logger().info(f'각도: {theta:.6f} 라디안 ({math.degrees(theta):.2f} 도)')
            
        except Exception as e:
            self.get_logger().error(f'파일 저장 오류: {e}')
    
    def keyboard_listener(self):
        """
        키보드 입력을 감지하는 스레드 함수
        """
        while True:
            try:
                # Enter키 입력 대기
                input()
                
                # 현재 GPS 좌표가 있는지 확인
                if self.current_lat is not None and self.current_lon is not None:
                    # UTM 좌표로 변환
                    utm_x, utm_y = self.latlon_to_utm(self.current_lat, self.current_lon)
                    
                    if utm_x is not None and utm_y is not None:
                        # Enter키 카운터 증가
                        self.enter_count += 1
                        
                        if self.enter_count == 1:
                            # 첫 번째 Enter키: 첫 번째 좌표 저장 및 로봇 이동 시작
                            self.utm_coordinates[0, 0] = utm_x
                            self.utm_coordinates[0, 1] = utm_y
                            
                            self.get_logger().info(
                                f'첫 번째 UTM 좌표 저장됨: Easting={utm_x:.2f}m, Northing={utm_y:.2f}m'
                            )
                            self.get_logger().info('두 번째 Enter키를 눌러서 두 번째 좌표를 저장하세요.')
                            
                            # 로봇 이동 시작 (1초간)
                            self.move_robot_forward()
                            self.start_movement_timer()
                            
                        elif self.enter_count == 2:
                            # 두 번째 Enter키: 두 번째 좌표 저장 및 로봇 정지
                            self.utm_coordinates[1, 0] = utm_x
                            self.utm_coordinates[1, 1] = utm_y
                            
                            self.get_logger().info(
                                f'두 번째 UTM 좌표 저장됨: Easting={utm_x:.2f}m, Northing={utm_y:.2f}m'
                            )
                            self.get_logger().info('두 개의 UTM 좌표가 모두 저장되었습니다!')
                            self.get_logger().info(f'저장된 UTM 배열:\n{self.utm_coordinates}')
                            
                            # 로봇 정지
                            self.stop_robot()
                            if self.movement_timer is not None:
                                self.movement_timer.cancel()
                                self.movement_timer = None
                            
                            # 두 점의 각도 계산 및 파일 저장
                            theta = self.calculate_angle_between_points()
                            self.save_angle_to_file(theta)
                            
                            # # 카운터 리셋 (다시 두 개의 좌표를 저장할 수 있도록)
                            # self.enter_count = 0
                            # self.get_logger().info('새로운 좌표 저장을 위해 첫 번째 Enter키를 누르세요.')
                        
                    else:
                        self.get_logger().error('UTM 좌표 변환에 실패했습니다.')
                else:
                    self.get_logger().warn('유효한 GPS 좌표가 없습니다.')
                    
            except EOFError:
                break
            except Exception as e:
                self.get_logger().error(f'키보드 입력 처리 오류: {e}')
                break

def main(args=None):
    """
    메인 함수
    """
    rclpy.init(args=args)
    
    imu_north_node = IMUNorth()
    
    try:
        rclpy.spin(imu_north_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 시리얼 포트 종료
        imu_north_node.close_serial_port()
        imu_north_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
