import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from iahrs_driver_interface.srv import Set

import tf2_ros
from transforms3d import euler

import io
import serial
import time
import math


class IahrsDriver(Node):
    CONF_SYNC_LIN_ACC = 0x0004  # 센서 좌표계 가속도 (g)
    CONF_SYNC_ANG_VEL = 0x0008  # 센서 좌표계 각속도 (deg/s)
    CONF_SYNC_EULER = 0x0040  # 오일러각
    CONF_SYNC_QUATERNION = 0x0080  # 쿼터니언

    def __init__(self):
        super().__init__("iahrs_driver_node")
        self._tf_prefix = self.get_parameter_or("tf_prefix", "")
        self._is_send_tf = self.get_parameter_or("send_tf", True)

        self.port_name = "/dev/ttyUSB0"
        self.baud_rate = "115200"
        self._ser = serial.Serial(
            self.port_name, 
            self.baud_rate,
            timeout=0.01,  # 10ms 타임아웃으로 블로킹 방지
            write_timeout=0.1
        )
        self._ser_io = io.TextIOWrapper(
            io.BufferedRWPair(self._ser, self._ser, 1),
            newline="\n",
            line_buffering=False,  # 라인 버퍼링 비활성화로 지연 감소
        )
        self._ser.flushInput()
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

        self._msg = Imu()

        # https://github.com/wookbin/iahrs_driver/blob/master/src/iahrs_driver.cpp#L294
        self._msg.linear_acceleration_covariance[0] = 0.0064
        self._msg.linear_acceleration_covariance[4] = 0.0063
        self._msg.linear_acceleration_covariance[8] = 0.0064

        self._msg.angular_velocity_covariance[0] = 0.032 * (math.pi / 180.0)
        self._msg.angular_velocity_covariance[4] = 0.028 * (math.pi / 180.0)
        self._msg.angular_velocity_covariance[8] = 0.006 * (math.pi / 180.0)

        self._msg.orientation_covariance[0] = 0.013 * (math.pi / 180.0)
        self._msg.orientation_covariance[4] = 0.011 * (math.pi / 180.0)
        self._msg.orientation_covariance[8] = 0.006 * (math.pi / 180.0)

        self._reset_sensor()

        self._imu_pub_handler = self.create_publisher(Imu, "/imu/data", 10)
        self.create_service(Set, "reset_sensor", self._reset_sensor_callback)
        self.create_service(Set, "reset_angle", self._reset_angle_callback)
        # 타이머 주기를 더 빠르게 설정 (5ms = 200Hz)
        self.timer = self.create_timer(0.005, self._serial_timer)
        
        # 성능 모니터링 변수
        self._last_publish_time = time.time()
        self._publish_count = 0
        self._error_count = 0

    def _set_sync_port(self):
        self._write_port_timeout("so=1")

    def _set_sync_period(self, period):
        self._write_port_timeout("sp={0}".format(period))

    def _set_sync_data(self, sync_data_type):
        sync_conf = "{:X}".format(sync_data_type)
        self._write_port_timeout("sd=0x" + sync_conf)

    def _is_port_available(self):
        return self._ser.isOpen()

    def _serial_timer(self):
        if not self._is_port_available():
            return
            
        try:
            # 여러 라인을 한 번에 읽어서 처리
            available_data = self._ser.in_waiting
            if available_data > 0:
                # 최대 1024바이트까지 읽기
                raw_data = self._ser.read(min(available_data, 1024))
                lines = raw_data.decode('utf-8', errors='ignore').split('\n')
                
                for line in lines:
                    line = line.strip()
                    if not line or '=' in line:
                        continue
                        
                    sync_data_splitted = line.split(",")
                    if len(sync_data_splitted) == 9:
                        self._new_data_flag = True
                        
                        try:
                            # 빠른 유효성 검사
                            valid_data = True
                            for item in sync_data_splitted:
                                if item.count('.') > 1 or (item.count('-') > 0 and not item.startswith('-')):
                                    valid_data = False
                                    break
                            
                            if not valid_data:
                                self._error_count += 1
                                continue
                                
                            # 데이터 변환
                            sync_data_splitted = [float(x.replace(',', '.')) for x in sync_data_splitted]
                            
                        except (ValueError, IndexError) as e:
                            self._error_count += 1
                            continue
                        
                        # IMU 데이터 설정
                        self._msg.linear_acceleration.x = sync_data_splitted[0] * 9.80665
                        self._msg.linear_acceleration.y = sync_data_splitted[1] * 9.80665
                        self._msg.linear_acceleration.z = sync_data_splitted[2] * 9.80665

                        self._msg.angular_velocity.x = sync_data_splitted[3] * (math.pi / 180)
                        self._msg.angular_velocity.y = sync_data_splitted[4] * (math.pi / 180)
                        self._msg.angular_velocity.z = sync_data_splitted[5] * (math.pi / 180)

                        # 오일러각을 쿼터니언으로 변환
                        q = euler.euler2quat(
                            sync_data_splitted[6] * (math.pi / 180),
                            sync_data_splitted[7] * (math.pi / 180),
                            sync_data_splitted[8] * (math.pi / 180),
                            "sxyz",
                        )

                        self._msg.orientation.w = q[0]
                        self._msg.orientation.x = q[1]
                        self._msg.orientation.y = q[2]
                        self._msg.orientation.z = q[3]

                        self._msg.header.stamp = self.get_clock().now().to_msg()
                        self._msg.header.frame_id = self._tf_prefix + "imu_link"
                        self._imu_pub_handler.publish(self._msg)
                        
                        self._publish_count += 1
                        
                        if self._is_send_tf:
                            self._send_tf()
                        
                        # 성능 모니터링 (10초마다)
                        current_time = time.time()
                        if current_time - self._last_publish_time >= 10.0:
                            rate = self._publish_count / 10.0
                            self.get_logger().info(f"IMU Rate: {rate:.1f} Hz, Errors: {self._error_count}")
                            self._publish_count = 0
                            self._error_count = 0
                            self._last_publish_time = current_time
                        
                        break  # 첫 번째 유효한 데이터만 처리
                        
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")
            self._ser.flushInput()
            self._error_count += 1

    def _write_port(self, buffer):
        if self._is_port_available() == True:
            self._ser.write((buffer + "\n").encode())

    def _write_port_timeout(self, buffer, timeout=0.5):
        self._write_port(buffer)
        started_time = time.time()
        while True:
            if self._is_port_available() == True:
                data = (self._ser.readline().split(b"\r\n")[0]).decode("utf-8").strip()
                if data and data == buffer:
                    break
            else:
                raise Exception("USB_NOT_CONNECTED")
            if time.time() - started_time >= 1:
                raise Exception("SYNC_SET_TIMEOUT")
        return True

    def _send_tf(self):
        br = tf2_ros.TransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._tf_prefix + "base_link"
        t.child_frame_id = self._tf_prefix + "imu_link"
        t.transform.rotation.x = self._msg.orientation.x
        t.transform.rotation.y = self._msg.orientation.y
        t.transform.rotation.z = self._msg.orientation.z
        t.transform.rotation.w = self._msg.orientation.w
        br.sendTransform(t)

    def _reset_sensor(self):
        self._write_port("za")
        self._set_sync_port()  # USB/Serial
        self._set_sync_period(50)  # 주기 10ms (100hz)
        self._set_sync_data(
            self.CONF_SYNC_LIN_ACC
            | self.CONF_SYNC_ANG_VEL
            | self.CONF_SYNC_EULER
            # | self.CONF_SYNC_QUATERNION  # 펌웨어 버그로 데이터가 유효하지 않음 (F/W v1.08)
        )
    
    def _reset_angle(self):
        self._write_port("c=7")

    def _reset_sensor_callback(self, req, res):
        self._reset_sensor()
        self._new_data_flag = False
        while True:
            if self._new_data_flag == True:
                break
            self._serial_timer()
        res.result = True
        return res

    def _reset_angle_callback(self, req, res):
        self._reset_angle()
        self._new_data_flag = False
        while True:
            if self._new_data_flag == True:
                break
            self._serial_timer()
        res.result = True
        return res


def main(args=None):
    rclpy.init(args=args)

    iahrs_driver_node = IahrsDriver()

    rclpy.spin(iahrs_driver_node)

    iahrs_driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
