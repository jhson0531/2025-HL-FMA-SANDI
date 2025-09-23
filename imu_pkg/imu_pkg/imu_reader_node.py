# ~/gps_ws/src/imu_pkg/imu_pkg/imu_reader_node.py  (경로 다르면 find로 확인)
#   find ~/gps_ws/src -name imu_reader_node.py

import re
import json, math, serial, threading, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from transforms3d import euler

G = 9.80665

def rpy2q(r, p, y):
    cr, sr = math.cos(r/2), math.sin(r/2)
    cp, sp = math.cos(p/2), math.sin(p/2)
    cy, sy = math.cos(y/2), math.sin(y/2)
    return Quaternion(
        w=cy*cp*cr + sy*sp*sr,
        x=cy*cp*sr - sy*sp*cr,
        y=cy*sp*cr + sy*cp*sr,
        z=sy*cp*cr - cy*sp*sr
    )

class IAhrsReader(Node):
    def __init__(self):
        super().__init__('iahrs_reader')

        # QoS: /imu를 RELIABLE로 퍼블리시
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        self.pub_raw = self.create_publisher(String, '/iahrs/raw', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/data', qos)   # 표준 IMU 토픽으로 변경
        
        # 서비스 생성 (센서 리셋용)
        from std_srvs.srv import Empty
        self.reset_sensor_service = self.create_service(Empty, 'reset_sensor', self._reset_sensor_callback)
        self.reset_angle_service = self.create_service(Empty, 'reset_angle', self._reset_angle_callback)

        # 파라미터 기본값 교체(0-기준 1..9)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter(
            'map_json',
            '{"ax":0,"ay":1,"az":2,"gx":3,"gy":4,"gz":5,'
            '"roll":6,"pitch":7,"yaw":8,"accel_in_g":true,"gyro_in_deg":true}'
        )


        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.map  = json.loads(self.get_parameter('map_json').get_parameter_value().string_value)

        self.accel_in_g = bool(self.map.get('accel_in_g', True))
        self.gyro_in_deg = bool(self.map.get('gyro_in_deg', True))

        # 필요한 최소 토큰 개수
        idxs = [self.map[k] for k in ('ax','ay','az','gx','gy','gz','roll','pitch','yaw') if isinstance(self.map.get(k), int)]
        self.min_needed = (max(idxs) + 1) if idxs else 0

        self.num_re = re.compile(r'[-+]?\d+(?:\.\d+)?')
        self._stop = threading.Event()
        self.ser = None
        self._open_serial()
        
        # 센서 초기화
        self._reset_sensor()

        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    def _open_serial(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        try:
            # exclusive=True로 중복 접속 방지(구버전 호환 처리)
            try:
                self.ser = serial.Serial(self.port, baudrate=self.baud, timeout=0.5,
                                         rtscts=False, dsrdtr=False, exclusive=True)
            except TypeError:
                self.ser = serial.Serial(self.port, baudrate=self.baud, timeout=0.5,
                                         rtscts=False, dsrdtr=False)
            self.ser.reset_input_buffer()
            self.get_logger().info(f'opened {self.port} @ {self.baud}')
        except Exception as e:
            self.get_logger().warn(f'open failed: {e}')
            self.ser = None

    def _write_port(self, buffer):
        """시리얼 포트에 명령 전송"""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((buffer + "\n").encode())
                self.get_logger().info(f"Sent command: {buffer}")
            except Exception as e:
                self.get_logger().warn(f"Failed to send command {buffer}: {e}")

    def _write_port_timeout(self, buffer, timeout=0.5):
        """시리얼 포트에 명령 전송 후 응답 대기"""
        self._write_port(buffer)
        started_time = time.time()
        while time.time() - started_time < timeout:
            if self.ser and self.ser.is_open:
                try:
                    data = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if data and data == buffer:
                        self.get_logger().info(f"Command {buffer} acknowledged")
                        return True
                except Exception:
                    pass
            time.sleep(0.01)
        self.get_logger().warn(f"Command {buffer} timeout")
        return False

    def _reset_sensor(self):
        """센서 초기화 (za 명령)"""
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("Serial port not available for sensor reset")
            return
            
        self.get_logger().info("Initializing IMU sensor...")
        
        # za 명령으로 센서 초기화
        self._write_port("za")
        time.sleep(0.1)
        
        # 동기화 설정
        self._write_port_timeout("so=1")  # USB/Serial 동기화
        time.sleep(0.1)
        
        self._write_port_timeout("sp=50")  # 50ms 주기 (20Hz)
        time.sleep(0.1)
        
        # 데이터 동기화 설정 (가속도, 각속도, 오일러각)
        self._write_port_timeout("sd=0x4C")  # 0x0004 | 0x0008 | 0x0040
        time.sleep(0.1)
        
        self.get_logger().info("IMU sensor initialization completed")

    def _reset_angle(self):
        """각도 리셋 (c=7 명령)"""
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("Serial port not available for angle reset")
            return
            
        self.get_logger().info("Resetting IMU angle...")
        self._write_port("c=7")
        time.sleep(0.1)
        self.get_logger().info("IMU angle reset completed")

    def _reset_sensor_callback(self, request, response):
        """센서 리셋 서비스 콜백"""
        self.get_logger().info("Reset sensor service called")
        self._reset_sensor()
        return response

    def _reset_angle_callback(self, request, response):
        """각도 리셋 서비스 콜백"""
        self.get_logger().info("Reset angle service called")
        self._reset_angle()
        return response

    def _val(self, tokens, key, default=0.0):
        idx = self.map.get(key, None)
        if not isinstance(idx, int) or idx < 0 or idx >= len(tokens):
            return default
        try:
            return float(tokens[idx])
        except Exception:
            return default

    def read_loop(self):
        empty_cnt = 0
        while rclpy.ok() and not self._stop.is_set():
            try:
                if not self.ser or not self.ser.is_open:
                    self._open_serial()
                    time.sleep(0.5)
                    continue

                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    empty_cnt += 1
                    if empty_cnt % 50 == 0:
                        self.get_logger().warn('no data from device (check cable/VM attach or multiple access)')
                    continue
                empty_cnt = 0

                self.pub_raw.publish(String(data=line))

                # iAHRS 형식 데이터 파싱 (9개 값: ax,ay,az,gx,gy,gz,roll,pitch,yaw)
                sync_data_splitted = line.split(",")
                if len(sync_data_splitted) == 9:
                    try:
                        # 데이터 유효성 검사
                        for item in sync_data_splitted:
                            if item.count('.') > 1 or (item.count('-') > 0 and not item.startswith('-')):
                                continue
                        
                        # 데이터 변환
                        sync_data_splitted = [float(x.replace(',', '.')) for x in sync_data_splitted]
                        
                        # IMU 메시지 생성
                        msg = Imu()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = 'imu_link'
                        
                        # 가속도 (g → m/s²)
                        msg.linear_acceleration.x = sync_data_splitted[0] * G
                        msg.linear_acceleration.y = sync_data_splitted[1] * G
                        msg.linear_acceleration.z = sync_data_splitted[2] * G
                        
                        # 각속도 (deg/s → rad/s)
                        msg.angular_velocity.x = sync_data_splitted[3] * (math.pi / 180)
                        msg.angular_velocity.y = sync_data_splitted[4] * (math.pi / 180)
                        msg.angular_velocity.z = sync_data_splitted[5] * (math.pi / 180)
                        
                        # 오일러각을 쿼터니언으로 변환 (iAHRS 방식)
                        q = euler.euler2quat(
                            sync_data_splitted[6] * (math.pi / 180),  # roll
                            sync_data_splitted[7] * (math.pi / 180),  # pitch
                            sync_data_splitted[8] * (math.pi / 180),  # yaw
                            "sxyz",
                        )
                        
                        msg.orientation.w = q[0]
                        msg.orientation.x = q[1]
                        msg.orientation.y = q[2]
                        msg.orientation.z = q[3]
                        
                        # 공분산 설정
                        msg.linear_acceleration_covariance[0] = 0.0064
                        msg.linear_acceleration_covariance[4] = 0.0063
                        msg.linear_acceleration_covariance[8] = 0.0064
                        
                        msg.angular_velocity_covariance[0] = 0.032 * (math.pi / 180.0)
                        msg.angular_velocity_covariance[4] = 0.028 * (math.pi / 180.0)
                        msg.angular_velocity_covariance[8] = 0.006 * (math.pi / 180.0)
                        
                        msg.orientation_covariance[0] = 0.013 * (math.pi / 180.0)
                        msg.orientation_covariance[4] = 0.011 * (math.pi / 180.0)
                        msg.orientation_covariance[8] = 0.006 * (math.pi / 180.0)
                        
                        # 발행
                        self.pub_imu.publish(msg)
                        self.get_logger().info(f"IMU data published: ax={msg.linear_acceleration.x:.3f}, ay={msg.linear_acceleration.y:.3f}, az={msg.linear_acceleration.z:.3f}, yaw={sync_data_splitted[8]:.1f}°")
                        
                    except (ValueError, IndexError) as e:
                        self.get_logger().warn(f"Failed to parse IMU data: {line}, error: {e}")
                        continue

            except (serial.SerialException, OSError) as e:
                self.get_logger().warn(f'serial error: {e}; reopening')
                time.sleep(0.5)
                self._open_serial()
            except Exception as e:
                self.get_logger().warn(f'parse/read error: {e}')
                continue

    def destroy_node(self):
        self._stop.set()
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        finally:
            super().destroy_node()

def main():
    rclpy.init()
    node = IAhrsReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
