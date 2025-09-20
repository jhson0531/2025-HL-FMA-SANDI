#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import os
from datetime import datetime
import utm

class GPSUTMRecorder(Node):
    def __init__(self):
        super().__init__('gps_utm_recorder')
        
        # GPS 토픽 구독
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/ublox_gps_node/fix',
            self.gps_callback,
            10
        )
        
        # GPS 및 UTM 좌표 저장용 변수
        self.current_latitude = 0.0
        self.current_longitude = 0.0
        self.current_utm_x = 0.0
        self.current_utm_y = 0.0
        self.gps_available = False
        self.zone_number = 0
        self.zone_letter = ''
        
        # 파일 저장 경로 (src 디렉토리)
        self.save_directory = "/home/jh/ros2_workspace/src"
        self.coordinates_file = os.path.join(self.save_directory, "utm_coordinates.txt")
        
        # 파일 초기화 (기존 내용 삭제)
        self.initialize_coordinates_file()
        
        self.get_logger().info("GPS UTM 좌표 기록기가 시작되었습니다.")
        self.get_logger().info("저장 방식:")
        self.get_logger().info("  0.5초 주기로 현재 UTM 좌표를 자동 저장")
        self.get_logger().info("  종료: Ctrl+C")
        self.get_logger().info(f"좌표 저장 파일: {self.coordinates_file}")
        self.get_logger().info("GPS 신호를 기다리는 중...")

        # 0.5초 주기로 자동 저장 타이머
        self.save_timer = self.create_timer(0.5, self.timer_callback)
        
        # 저장된 좌표 개수 카운터
        self.save_count = 0
    
    def initialize_coordinates_file(self):
        """좌표 파일 초기화"""
        try:
            with open(self.coordinates_file, 'w', encoding='utf-8') as f:
                f.write(f"# UTM 좌표 기록 파일\n")
                f.write(f"# 생성 시간: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"# 형식: UTM_X, UTM_Y\n")
                f.write(f"# 저장 방식: 0.5초 주기로 자동 저장\n\n")
            self.get_logger().info(f"좌표 파일이 초기화되었습니다: {self.coordinates_file}")
        except Exception as e:
            self.get_logger().error(f"파일 초기화 실패: {e}")
    
    def gps_callback(self, msg: NavSatFix):
        """GPS 좌표 콜백 함수"""
        if msg.status.status < 0:  # GPS 신호가 없으면 무시
            return
            
        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude
        
        # GPS 좌표를 UTM으로 변환
        try:
            utm_x, utm_y, zone_number, zone_letter = utm.from_latlon(msg.latitude, msg.longitude)
            self.current_utm_x = utm_x
            self.current_utm_y = utm_y
            self.zone_number = zone_number
            self.zone_letter = zone_letter
            self.gps_available = True
            
            # GPS 데이터 수신 로그 (처음 몇 번만 출력)
            if not hasattr(self, 'gps_log_count'):
                self.gps_log_count = 0
            if self.gps_log_count < 3:
                self.get_logger().info(f"GPS 수신: 위도={msg.latitude:.8f}, 경도={msg.longitude:.8f}")
                self.get_logger().info(f"UTM 변환: X={utm_x:.3f}, Y={utm_y:.3f}, Zone={zone_number}{zone_letter}")
                self.gps_log_count += 1
            elif self.gps_log_count == 3:
                self.get_logger().info("GPS 신호 수신 중... (로그 출력 중단)")
                self.gps_log_count += 1
                
        except Exception as e:
            self.get_logger().warn(f"UTM 변환 실패: {e}")
            self.gps_available = False
    
    def save_utm_coordinates(self):
        """현재 UTM 좌표를 파일에 추가 저장"""
        if not self.gps_available:
            self.get_logger().warn("GPS 데이터가 아직 수신되지 않았습니다.")
            return
        
        try:
            # 파일에 좌표 추가 (append 모드)
            with open(self.coordinates_file, 'a', encoding='utf-8') as f:
                timestamp = datetime.now().strftime("%H:%M:%S")
                f.write(f"{self.current_utm_x:.6f}, {self.current_utm_y:.6f}  # {timestamp}\n")
            
            # 저장 횟수 증가 및 로그 출력
            self.save_count += 1
            self.get_logger().info(f"📍 [{self.save_count}] UTM 좌표 저장됨: X={self.current_utm_x:.6f}, Y={self.current_utm_y:.6f}")
            self.get_logger().info(f"   GPS: 위도={self.current_latitude:.8f}, 경도={self.current_longitude:.8f}")
            self.get_logger().info(f"   Zone: {self.zone_number}{self.zone_letter}")
            
        except Exception as e:
            self.get_logger().error(f"파일 저장 중 오류 발생: {e}")

    def timer_callback(self):
        """0.5초 주기로 현재 좌표 자동 저장"""
        self.save_utm_coordinates()

def main(args=None):
    rclpy.init(args=args)
    
    # GPS UTM 기록기 노드 생성
    recorder_node = GPSUTMRecorder()
    
    try:
        rclpy.spin(recorder_node)
    except KeyboardInterrupt:
        recorder_node.get_logger().info("프로그램 종료 중...")
    finally:
        recorder_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()