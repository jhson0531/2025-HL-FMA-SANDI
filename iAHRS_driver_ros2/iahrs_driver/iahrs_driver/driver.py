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
        self._ser = serial.Serial(self.port_name, self.baud_rate)
        self._ser_io = io.TextIOWrapper(
            io.BufferedRWPair(self._ser, self._ser, 1),
            newline="\n",
            line_buffering=True,
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

        self._imu_pub_handler = self.create_publisher(Imu, "/imu/data", 1)
        self.create_service(Set, "reset_sensor", self._reset_sensor_callback)
        self.create_service(Set, "reset_angle", self._reset_angle_callback)
        self.timer = self.create_timer(0.01, self._serial_timer)

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
        if self._is_port_available() == True:
            sync_data = (self._ser.readline().split(b"\r\n")[0]).decode("utf-8").strip()
            if sync_data and len(sync_data.split("=")) == 1:
                sync_data_splitted = sync_data.split(",")
                if len(sync_data_splitted) == 9:
                    self._new_data_flag = True

                    try:
                        # 변환 전에 각 항목이 유효한지 간단히 확인
                        for item in sync_data_splitted:
                            # 소수점이 2개 이상이거나, '-'가 맨 앞이 아닌 곳에 있으면 건너뛰기
                            if item.count('.') > 1 or (item.count('-') > 0 and not item.startswith('-')):
                                # self.get_logger().warn(f"Invalid data format received, skipping: {item} in {sync_data}")
                                return # 이 메시지는 처리하지 않고 함수 종료

                        # 유효성 검사를 통과한 데이터만 변환 시도
                        sync_data_splitted = list(
                            map(lambda x: float(x.replace(',', '.')), sync_data_splitted)
                        )
                    except ValueError as e:
                        self.get_logger().error(f"Failed to parse IMU data: {sync_data}, error: {e}")
                        return
                    self._msg.linear_acceleration.x = sync_data_splitted[0] * 9.80665
                    self._msg.linear_acceleration.y = sync_data_splitted[1] * 9.80665
                    self._msg.linear_acceleration.z = sync_data_splitted[2] * 9.80665

                    self._msg.angular_velocity.x = sync_data_splitted[3] * (
                        math.pi / 180
                    )
                    self._msg.angular_velocity.y = sync_data_splitted[4] * (
                        math.pi / 180
                    )
                    self._msg.angular_velocity.z = sync_data_splitted[5] * (
                        math.pi / 180
                    )

                    # 펌웨어 버전 v1.08버전에서 펌웨어 버그로 쿼터니언 데이터는 유효하지 않음
                    # self._msg.orientation.w = sync_data_splitted[9]
                    # self._msg.orientation.x = sync_data_splitted[10]
                    # self._msg.orientation.y = sync_data_splitted[11]
                    # self._msg.orientation.z = sync_data_splitted[12]
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
                    if self._is_send_tf is True:
                        self._send_tf()

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
        self._set_sync_period(10)  # 주기 10ms (100hz)
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
