#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import time
import math
import utm
from datetime import datetime

class GPSIMUCalibration(Node):
    def __init__(self):
        super().__init__('gps_imu_calibration')
        
        # GPS 토픽 구독
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/ublox_gps_node/fix',
            self.gps_callback,
            10
        )
        
        # 시리얼 포트 설정
        self.serial_port = "/dev/ttyUSB1"
        self.baud_rate = 115200
        self.serial_connection = None
        
        # 보정 파라미터
        self.forward_duration = 1.0  # 전진 시간 (초) - 파라미터로 조절 가능
        self.forward_speed = 20      # 전진 속도
        self.steering_angle = 0      # 조향각 (직진)
        
        # GPS 데이터 저장
        self.first_position = None   # 첫 번째 위치 (위도, 경도)
        self.second_position = None  # 두 번째 위치 (위도, 경도)
        self.first_utm = None        # 첫 번째 UTM 좌표
        self.second_utm = None       # 두 번째 UTM 좌표
        
        # 상태 관리
        self.calibration_state = "WAITING_FOR_FIRST_GPS"  # WAITING_FOR_FIRST_GPS, WAITING_FOR_SECOND_GPS, CALCULATING, COMPLETED
        self.gps_data_received = False
        
        self.get_logger().info("GPS-IMU 보정 프로그램이 시작되었습니다.")
        self.get_logger().info(f"전진 시간: {self.forward_duration}초")
        self.get_logger().info("첫 번째 GPS 위치를 기다리는 중...")
        
    def connect_serial(self):
        """시리얼 포트 연결"""
        try:
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1
            )
            self.get_logger().info(f"시리얼 포트 연결 성공: {self.serial_port}")
            return True
        except Exception as e:
            self.get_logger().error(f"시리얼 포트 연결 실패: {e}")
            return False
    
    def send_serial_command(self, command):
        """시리얼 명령 전송"""
        if self.serial_connection and self.serial_connection.is_open:
            try:
                self.serial_connection.write(f"{command}\n".encode())
                self.get_logger().info(f"시리얼 명령 전송: {command}")
                return True
            except Exception as e:
                self.get_logger().error(f"시리얼 명령 전송 실패: {e}")
                return False
        else:
            self.get_logger().error("시리얼 포트가 연결되지 않았습니다.")
            return False
    
    def gps_callback(self, msg):
        """GPS 데이터 처리"""
        if msg.status.status < 0:  # GPS 신호가 없으면 무시
            return
            
        latitude = msg.latitude
        longitude = msg.longitude
        
        # UTM 좌표로 변환
        utm_x, utm_y, zone_number, zone_letter = utm.from_latlon(latitude, longitude)
        
        self.get_logger().info(f"GPS 수신: 위도={latitude:.8f}, 경도={longitude:.8f}")
        self.get_logger().info(f"UTM 변환: X={utm_x:.3f}, Y={utm_y:.3f}, Zone={zone_number}{zone_letter}")
        
        if self.calibration_state == "WAITING_FOR_FIRST_GPS":
            # 첫 번째 위치 저장
            self.first_position = (latitude, longitude)
            self.first_utm = (utm_x, utm_y)
            self.calibration_state = "WAITING_FOR_SECOND_GPS"
            
            self.get_logger().info("=" * 60)
            self.get_logger().info("📍 첫 번째 위치 저장 완료!")
            self.get_logger().info(f"   위도: {latitude:.8f}")
            self.get_logger().info(f"   경도: {longitude:.8f}")
            self.get_logger().info(f"   UTM X: {utm_x:.3f}")
            self.get_logger().info(f"   UTM Y: {utm_y:.3f}")
            self.get_logger().info("=" * 60)
            self.get_logger().info("로봇을 전진시킵니다...")
            
            # 시리얼 포트 연결 및 로봇 전진 명령
            if self.connect_serial():
                command = f"s{self.steering_angle}p{self.forward_speed}"
                if self.send_serial_command(command):
                    # 전진 시간 후 정지
                    time.sleep(self.forward_duration)
                    self.send_serial_command("s0p0")  # 정지
                    self.get_logger().info("로봇 전진 완료. 두 번째 GPS 위치를 기다리는 중...")
                else:
                    self.get_logger().error("로봇 전진 명령 실패")
            else:
                self.get_logger().error("시리얼 포트 연결 실패")
                
        elif self.calibration_state == "WAITING_FOR_SECOND_GPS":
            # 두 번째 위치 저장
            self.second_position = (latitude, longitude)
            self.second_utm = (utm_x, utm_y)
            self.calibration_state = "CALCULATING"
            
            self.get_logger().info("=" * 60)
            self.get_logger().info("📍 두 번째 위치 저장 완료!")
            self.get_logger().info(f"   위도: {latitude:.8f}")
            self.get_logger().info(f"   경도: {longitude:.8f}")
            self.get_logger().info(f"   UTM X: {utm_x:.3f}")
            self.get_logger().info(f"   UTM Y: {utm_y:.3f}")
            self.get_logger().info("=" * 60)
            
            # 방위각 계산
            self.calculate_bearing_angle()
    
    def calculate_bearing_angle(self):
        """두 UTM 좌표로부터 방위각 계산"""
        if self.first_utm is None or self.second_utm is None:
            self.get_logger().error("UTM 좌표가 부족합니다.")
            return
        
        # UTM 좌표에서 동쪽으로부터의 각도 계산
        dx = self.second_utm[0] - self.first_utm[0]  # 동쪽 방향
        dy = self.second_utm[1] - self.first_utm[1]  # 북쪽 방향
        
        # atan2를 사용하여 각도 계산 (라디안)
        bearing_rad = math.atan2(dx, dy)
        
        # 라디안을 도로 변환
        bearing_deg = math.degrees(bearing_rad)
        
        # 0~360도 범위로 정규화
        if bearing_deg < 0:
            bearing_deg += 360
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🧭 방위각 계산 결과")
        self.get_logger().info(f"   첫 번째 UTM: ({self.first_utm[0]:.3f}, {self.first_utm[1]:.3f})")
        self.get_logger().info(f"   두 번째 UTM: ({self.second_utm[0]:.3f}, {self.second_utm[1]:.3f})")
        self.get_logger().info(f"   이동 벡터: dx={dx:.3f}, dy={dy:.3f}")
        self.get_logger().info(f"   계산된 방위각: {bearing_deg:.2f}°")
        self.get_logger().info("=" * 60)
        
        # 파일에 저장
        self.save_angle_to_file(bearing_deg)
        
        self.calibration_state = "COMPLETED"
    
    def save_angle_to_file(self, angle_deg):
        """계산된 각도를 txt 파일에 저장"""
        try:
            # 읽기 쉬운 형식으로 저장
            filename = "imu_calibration_angle.txt"
            
            with open(filename, 'w') as f:
                # 첫 번째 줄: 보정각도 (라디안 단위)
                angle_rad = math.radians(angle_deg)
                f.write(f"{angle_rad:.6f}\n")

                # 두 번째 줄: 보정각도 (도 단위)
                f.write(f"{angle_deg:.6f}\n")
                            
                # 주석으로 상세 정보 추가
                f.write(f"# IMU 방위각 보정값\n")
                f.write(f"# 생성 시간: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"# 첫 번째 GPS 위치: {self.first_position[0]:.8f}, {self.first_position[1]:.8f}\n")
                f.write(f"# 두 번째 GPS 위치: {self.second_position[0]:.8f}, {self.second_position[1]:.8f}\n")
                f.write(f"# 첫 번째 UTM 좌표: {self.first_utm[0]:.3f}, {self.first_utm[1]:.3f}\n")
                f.write(f"# 두 번째 UTM 좌표: {self.second_utm[0]:.3f}, {self.second_utm[1]:.3f}\n")
                f.write(f"# 계산된 방위각: {angle_deg:.6f} degrees ({angle_rad:.6f} radians)\n")
                f.write(f"# 첫 번째 줄: 보정각도 (라디안)\n")
                f.write(f"# 두 번째 줄: 보정각도 (도)\n")
            
            self.get_logger().info(f"✅ 보정각이 파일에 저장되었습니다: {filename}")
            self.get_logger().info(f"   저장된 각도: {angle_deg:.6f}° ({angle_rad:.6f} rad)")
            self.get_logger().info(f"   IMU 드라이버에서 첫 번째 줄을 읽어서 사용하세요.")
            
        except Exception as e:
            self.get_logger().error(f"파일 저장 실패: {e}")
    
    def set_forward_duration(self, duration):
        """전진 시간 설정 (파라미터 조절용)"""
        self.forward_duration = duration
        self.get_logger().info(f"전진 시간이 {duration}초로 설정되었습니다.")
    
    def __del__(self):
        """소멸자 - 시리얼 포트 정리"""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()

def main(args=None):
    rclpy.init(args=args)
    
    # GPS-IMU 보정 노드 생성
    calibration_node = GPSIMUCalibration()
    
    # 전진 시간을 파라미터로 설정 (예시: 5초)
    # calibration_node.set_forward_duration(5.0)
    
    try:
        rclpy.spin(calibration_node)
    except KeyboardInterrupt:
        calibration_node.get_logger().info("보정 프로그램 종료 중...")
    finally:
        calibration_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
