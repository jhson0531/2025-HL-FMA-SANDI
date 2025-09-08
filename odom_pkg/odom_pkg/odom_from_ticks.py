# odom_from_ticks.py
import rclpy, math, serial, threading, time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

SQRT12 = math.sqrt(12.0)
U_LONG_MAX = 4294967295  # Arduino의 unsigned long 최대값 (2^32 - 1)

class OdomFromTicks(Node):
    def __init__(self):
        super().__init__('odom_from_ticks')
        # ... 파라미터 선언 부분은 이전과 동일 ...
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('wheelbase', 0.72)
        self.declare_parameter('ticks_per_meter', 337.0)
        self.declare_parameter('pot_left_raw',   1023)
        self.declare_parameter('pot_center_raw',  512)
        self.declare_parameter('pot_right_raw',     0)
        self.declare_parameter('steer_left_max_deg',  30.0)
        self.declare_parameter('steer_right_max_deg', -30.0)
        self.declare_parameter('pos_x_var_floor',   0.05)
        self.declare_parameter('pos_yaw_var_floor', 0.20)
        self.declare_parameter('pos_x_var_max',    10.0)
        self.declare_parameter('pos_yaw_var_max',  10.0)
        self.declare_parameter('pos_y_var_const',   0.5)
        self.declare_parameter('big_var',       1e6)
        self.declare_parameter('twist_vx_var_floor', 0.02)
        self.declare_parameter('twist_wz_var_floor', 0.10)
        self.declare_parameter('steer_extra_var', 1e-5)
        self.declare_parameter('debug', True)

        self.port  = self.get_parameter('port').value
        self.baud  = int(self.get_parameter('baud').value)
        self.frame = self.get_parameter('frame_id').value
        self.child = self.get_parameter('child_frame_id').value
        self.pub_tf= bool(self.get_parameter('publish_tf').value)
        self.L     = float(self.get_parameter('wheelbase').value)
        self.TPM   = float(self.get_parameter('ticks_per_meter').value)
        self.pL    = int(self.get_parameter('pot_left_raw').value)
        self.pC    = int(self.get_parameter('pot_center_raw').value)
        self.pR    = int(self.get_parameter('pot_right_raw').value)
        self.dL    = float(self.get_parameter('steer_left_max_deg').value)
        self.dR    = float(self.get_parameter('steer_right_max_deg').value)
        self.debug = bool(self.get_parameter('debug').value)
        self.pos_x_var_floor   = float(self.get_parameter('pos_x_var_floor').value)
        self.pos_yaw_var_floor = float(self.get_parameter('pos_yaw_var_floor').value)
        self.pos_x_var_max     = float(self.get_parameter('pos_x_var_max').value)
        self.pos_yaw_var_max   = float(self.get_parameter('pos_yaw_var_max').value)
        self.pos_y_var_const   = float(self.get_parameter('pos_y_var_const').value)
        self.big_var           = float(self.get_parameter('big_var').value)
        self.twist_vx_var_floor= float(self.get_parameter('twist_vx_var_floor').value)
        self.twist_wz_var_floor= float(self.get_parameter('twist_wz_var_floor').value)
        self.steer_extra_var   = float(self.get_parameter('steer_extra_var').value)

        self.ser = serial.Serial(self.port, self.baud, timeout=0.3)
        
        self.qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1
        )

        self.pub = self.create_publisher(Odometry, '/odometry/wheel', self.qos_profile)
        self.tfb = TransformBroadcaster(self) if self.pub_tf else None

        # --- 상태 변수 ---
        self.x = self.y = self.yaw = 0.0
        self.prev_ticks = None
        self.prev_ms = None  # [수정] ROS 시간 대신 아두이노 시간(ms)을 추적
        self.line_cnt = 0
        self.pos_x_var_acc = max(1e-9, self.pos_x_var_floor)
        self.pos_yaw_var_acc = max(1e-9, self.pos_yaw_var_floor)

        # [추가] 스레드 동기화를 위한 Lock 객체 생성
        self.lock = threading.Lock()

        self.thread = threading.Thread(target=self.reader, daemon=True)
        self.thread.start()
        self.get_logger().info(f'Opened {self.port}@{self.baud}, L={self.L}, TPM={self.TPM}, pot(L,C,R)=({self.pL},{self.pC},{self.pR})')

    # ... pot_to_deg, steering_slope_rad_per_raw, update_covariances 함수는 이전과 동일 ...
    def pot_to_deg(self, raw:int)->float:
        if raw >= self.pC:
            span = max(1, self.pL - self.pC)
            t = min(1.0, max(0.0, (raw - self.pC)/span))
            return t * self.dL
        else:
            span = max(1, self.pC - self.pR)
            t = min(1.0, max(0.0, (self.pC - raw)/span))
            return -t * abs(self.dR)
    
    def steering_slope_rad_per_raw(self, raw:int)->float:
        if raw >= self.pC:
            span = max(1, self.pL - self.pC)
            slope_deg_per_raw = self.dL / span
        else:
            span = max(1, self.pC - self.pR)
            slope_deg_per_raw = (-abs(self.dR)) / span
        return slope_deg_per_raw * math.pi / 180.0

    def update_covariances(self, v:float, delta:float, dt:float, pot_raw:int):
        delta_d = 1.0 / self.TPM
        var_dx = (delta_d ** 2) / 12.0
        var_vx = var_dx / max(1e-6, dt*dt)
        slope = abs(self.steering_slope_rad_per_raw(pot_raw))
        var_delta = (slope * slope) * (1.0/12.0) + self.steer_extra_var
        c = math.cos(delta)
        sec2 = 1.0 / max(1e-9, c*c)
        tan = math.tan(delta)
        dwd_delta = (v * sec2) / max(1e-9, self.L)
        dwd_v     =  tan       / max(1e-9, self.L)
        var_wz = (dwd_delta*dwd_delta) * var_delta + (dwd_v*dwd_v) * var_vx
        self.pos_x_var_acc  = min(self.pos_x_var_max, self.pos_x_var_acc + var_dx)
        self.pos_yaw_var_acc= min(self.pos_yaw_var_max, self.pos_yaw_var_acc + var_wz * dt*dt)
        pos_x_var   = max(self.pos_x_var_floor,   self.pos_x_var_acc)
        pos_yaw_var = max(self.pos_yaw_var_floor, self.pos_yaw_var_acc)
        twist_vx_var = max(self.twist_vx_var_floor, var_vx)
        twist_wz_var = max(self.twist_wz_var_floor, var_wz)
        return pos_x_var, pos_yaw_var, twist_vx_var, twist_wz_var

    def reader(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
                if not line:
                    continue
                
                if self.debug and self.line_cnt < 10:
                    self.get_logger().info(f'RAW: {line}')

                if line[0] != 'T':
                    continue
                parts = line.split(',')
                if len(parts) < 4:
                    continue

                # --- [추가] 스레드 락 시작 ---
                # 공유 변수(self.x, self.yaw 등)를 안전하게 수정하기 위해 lock을 겁니다.
                with self.lock:
                    self.line_cnt += 1 # line_cnt도 공유변수이므로 lock 안으로 이동
                    
                    ms = int(parts[1]); ticks = int(parts[2]); pot_raw = int(parts[3])

                    # --- [수정] 아두이노 시간(ms) 기준으로 dt 계산 ---
                    if self.prev_ms is None:
                        self.prev_ms = ms
                        self.prev_ticks = ticks
                        continue

                    # millis() 오버플로우(약 49.7일)를 안전하게 처리
                    d_ms = (ms - self.prev_ms + U_LONG_MAX + 1) % (U_LONG_MAX + 1)
                    dt = d_ms / 1000.0
                    self.prev_ms = ms

                    if dt <= 0.0 or dt > 0.5:
                        self.prev_ticks = ticks
                        continue
                    
                    # --- dticks 계산 ---
                    dticks = ticks - self.prev_ticks
                    self.prev_ticks = ticks

                    # --- 계산 로직 (이전과 동일) ---
                    # 엔코더가 '앞바퀴'에 있으므로 이 모델이 맞습니다.
                    v_front = (dticks / self.TPM) / dt
                    delta = math.radians(self.pot_to_deg(pot_raw))
                    c = math.cos(delta)
                    v = v_front * c
                    omega = 0.0 if abs(c) < 1e-4 else v * math.tan(delta) / self.L

                    self.yaw += omega * dt
                    self.x   += v * math.cos(self.yaw) * dt
                    self.y   += v * math.sin(self.yaw) * dt

                    pos_x_var, pos_yaw_var, tw_vx_var, tw_wz_var = self.update_covariances(v, delta, dt, pot_raw)

                    now = self.get_clock().now() # 메시지 발행 시점의 ROS 시간
                    
                    # --- 메시지 생성 및 발행 (이전과 동일) ---
                    msg = Odometry()
                    msg.header.stamp = now.to_msg()
                    msg.header.frame_id = self.frame
                    msg.child_frame_id  = self.child
                    msg.pose.pose.position.x = self.x
                    msg.pose.pose.position.y = self.y
                    cy = math.cos(self.yaw*0.5); sy = math.sin(self.yaw*0.5)
                    msg.pose.pose.orientation.z = sy; msg.pose.pose.orientation.w = cy
                    msg.twist.twist.linear.x = v
                    msg.twist.twist.angular.z = omega
                    
                    msg.pose.covariance = [0.0]*36
                    diag_pose = [pos_x_var, self.pos_y_var_const, self.big_var, self.big_var, self.big_var, pos_yaw_var]
                    for i, vv in enumerate(diag_pose): msg.pose.covariance[i*6 + i] = vv
                    
                    msg.twist.covariance = [0.0]*36
                    diag_twist = [tw_vx_var, self.big_var, self.big_var, self.big_var, self.big_var, tw_wz_var]
                    for i, vv in enumerate(diag_twist): msg.twist.covariance[i*6 + i] = vv

                    if self.debug and (self.line_cnt % 100 == 0):
                        self.get_logger().info(f'Publishing odom: x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.1f}°, v={v:.3f}, ω={math.degrees(omega):.1f}°/s')
                    
                    self.pub.publish(msg)

                    if self.tfb:
                        tf = TransformStamped()
                        tf.header.stamp = now.to_msg()
                        tf.header.frame_id = self.frame
                        tf.child_frame_id  = self.child
                        tf.transform.translation.x = self.x
                        tf.transform.translation.y = self.y
                        tf.transform.rotation.z = sy
                        tf.transform.rotation.w = cy
                        self.tfb.sendTransform(tf)
                # --- [추가] 스레드 락 종료 ---

            except Exception as e:
                self.get_logger().warn(f'err: {e}')
                time.sleep(0.05)

def main():
    rclpy.init()
    node = OdomFromTicks()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()