# odom_from_ticks.py
import rclpy, math, serial, threading, time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

SQRT12 = math.sqrt(12.0)

class OdomFromTicks(Node):
    def __init__(self):
        super().__init__('odom_from_ticks')
        # params
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('wheelbase', 0.72)
        self.declare_parameter('ticks_per_meter', 337.0)

        # ⚠ Mega 10-bit ADC 기준 기본값으로 수정
        self.declare_parameter('pot_left_raw',   1023)
        self.declare_parameter('pot_center_raw',  512)
        self.declare_parameter('pot_right_raw',     0)
        self.declare_parameter('steer_left_max_deg',  30.0)
        self.declare_parameter('steer_right_max_deg', -30.0)

        # ---- 공분산 튜닝 파라미터(실전 기본값) ----
        # pose 바닥/상한
        self.declare_parameter('pos_x_var_floor',   0.05)   # Var(x) 최소 [m^2]
        self.declare_parameter('pos_yaw_var_floor', 0.20)   # Var(yaw) 최소 [rad^2]
        self.declare_parameter('pos_x_var_max',    10.0)    # Var(x) 상한(폭주방지)
        self.declare_parameter('pos_yaw_var_max',  10.0)    # Var(yaw) 상한
        self.declare_parameter('pos_y_var_const',   0.5)    # Var(y) 고정치(관측약함)
        self.declare_parameter('big_var',       1e6)        # 관측하지 않는 축

        # twist 바닥
        self.declare_parameter('twist_vx_var_floor', 0.02)  # Var(vx) 최소 [(m/s)^2]
        self.declare_parameter('twist_wz_var_floor', 0.10)  # Var(wz) 최소 [(rad/s)^2]

        # 조향각 추가 불확실도(캘리브 오차 등) [rad^2]
        self.declare_parameter('steer_extra_var', 1e-5)
        # ---------------------------

        # 디버그 스위치
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

        self.ser = serial.Serial(self.port, self.baud, timeout=0.3)

        
        self.pos_x_var_floor   = float(self.get_parameter('pos_x_var_floor').value)
        self.pos_yaw_var_floor = float(self.get_parameter('pos_yaw_var_floor').value)
        self.pos_x_var_max     = float(self.get_parameter('pos_x_var_max').value)
        self.pos_yaw_var_max   = float(self.get_parameter('pos_yaw_var_max').value)
        self.pos_y_var_const   = float(self.get_parameter('pos_y_var_const').value)
        self.big_var           = float(self.get_parameter('big_var').value)

        self.twist_vx_var_floor= float(self.get_parameter('twist_vx_var_floor').value)
        self.twist_wz_var_floor= float(self.get_parameter('twist_wz_var_floor').value)
        self.steer_extra_var   = float(self.get_parameter('steer_extra_var').value)

        
        self.qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1
        )

        self.pub = self.create_publisher(Odometry, '/odometry/wheel', self.qos_profile)
        self.tfb = TransformBroadcaster(self) if self.pub_tf else None

        self.x = self.y = self.yaw = 0.0
        self.prev_ticks = None
        self.last_ros = self.get_clock().now()
        self.line_cnt = 0

        # 공분산 누적(포즈용): 측정마다 늘어나는 불확실도 적분
        self.pos_x_var_acc = max(1e-9, self.pos_x_var_floor)    # 시작 바닥
        self.pos_yaw_var_acc = max(1e-9, self.pos_yaw_var_floor)

        self.thread = threading.Thread(target=self.reader, daemon=True)
        self.thread.start()
        self.get_logger().info(f'Opened {self.port}@{self.baud}, L={self.L}, TPM={self.TPM}, pot(L,C,R)=({self.pL},{self.pC},{self.pR})')

    def pot_to_deg(self, raw:int)->float:
        # center 기준 좌/우 선형 보간
        if raw >= self.pC:
            span = max(1, self.pL - self.pC)
            t = min(1.0, max(0.0, (raw - self.pC)/span))
            return t * self.dL
        else:
            span = max(1, self.pC - self.pR)
            t = min(1.0, max(0.0, (self.pC - raw)/span))
            return -t * abs(self.dR)
    
    # --- 조향각 보간 기울기(rad/raw) ---
    def steering_slope_rad_per_raw(self, raw:int)->float:
        if raw >= self.pC:
            span = max(1, self.pL - self.pC)
            slope_deg_per_raw = self.dL / span
        else:
            span = max(1, self.pC - self.pR)
            slope_deg_per_raw = (-abs(self.dR)) / span
        return slope_deg_per_raw * math.pi / 180.0

    # --- 공분산 계산/업데이트 (실전 방식) ---
    def update_covariances(self, v:float, delta:float, dt:float, pot_raw:int):
        """
        포즈 누적분산(pos_x_var_acc, pos_yaw_var_acc)을 업데이트하고,
        트위스트 분산(vx, wz)을 현재 샘플 기준으로 계산해서 반환
        """
        # 엔코더 양자화: Δd = 1/TPM, Var(dx) = Δd²/12
        delta_d = 1.0 / self.TPM
        var_dx = (delta_d ** 2) / 12.0

        # 속도 분산(샘플 미분): Var(vx) = Var(dx)/dt² (dt>0 가정)
        var_vx = var_dx / max(1e-6, dt*dt)

        # 포텐셔미터 각도 분산: raw 1 LSB 분산 = 1/12, δ = slope*raw
        slope = abs(self.steering_slope_rad_per_raw(pot_raw))
        var_delta = (slope * slope) * (1.0/12.0) + self.steer_extra_var

        # ω = v * tan(δ) / L 의 1차 오차전파
        c = math.cos(delta)
        sec2 = 1.0 / max(1e-9, c*c)               # sec^2(δ)
        tan = math.tan(delta)

        dwd_delta = (v * sec2) / max(1e-9, self.L)
        dwd_v     =  tan       / max(1e-9, self.L)

        var_wz = (dwd_delta*dwd_delta) * var_delta + (dwd_v*dwd_v) * var_vx

        # ---- 포즈 누적분산 업데이트(적분) ----
        # x는 이동거리 누적으로 증가, yaw는 ω 적분으로 증가
        self.pos_x_var_acc  = min(self.pos_x_var_max,
                                  self.pos_x_var_acc + var_dx)
        self.pos_yaw_var_acc= min(self.pos_yaw_var_max,
                                  self.pos_yaw_var_acc + var_wz * dt*dt)

        # 바닥값 적용(너무 작아지지 않게)
        pos_x_var   = max(self.pos_x_var_floor,   self.pos_x_var_acc)
        pos_yaw_var = max(self.pos_yaw_var_floor, self.pos_yaw_var_acc)

        # twist(즉시 측정)에도 바닥값 적용
        twist_vx_var = max(self.twist_vx_var_floor, var_vx)
        twist_wz_var = max(self.twist_wz_var_floor, var_wz)

        return pos_x_var, pos_yaw_var, twist_vx_var, twist_wz_var

    def reader(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
                if not line:
                    continue

                # 처음에는 몇 줄이건 그대로 로그에 찍어서 포맷 확인
                if self.debug and self.line_cnt < 10:
                    self.get_logger().info(f'RAW: {line}')
                    self.line_cnt += 1

                if line[0] != 'T':   # 기대 포맷: T,ms,ticks,pot_raw
                    continue
                parts = line.split(',')
                if len(parts) < 4:
                    if self.debug:
                        self.get_logger().warn(f'bad parts: {parts}')
                    continue

                ticks = int(parts[2]); pot_raw = int(parts[3])

                # ROS 시간으로 dt 계산
                now = self.get_clock().now()
                dt = (now - self.last_ros).nanoseconds * 1e-9
                self.last_ros = now
                if dt <= 0.0 or dt > 0.5:   # 간격 넉넉히 0.5s까지 허용
                    self.prev_ticks = ticks
                    continue

                if self.prev_ticks is None:
                    self.prev_ticks = ticks
                    continue

                dticks = ticks - self.prev_ticks
                self.prev_ticks = ticks

                if self.debug and (self.line_cnt % 50 == 0):
                    self.get_logger().info(f'dt={dt:.3f}s, dticks={dticks}, ticks={ticks}, pot={pot_raw}')

                # 속도 계산
                v_front = (dticks / self.TPM) / dt
                delta = math.radians(self.pot_to_deg(pot_raw))
                c = math.cos(delta)
                v = v_front * c
                omega = 0.0 if abs(c) < 1e-4 else v * math.tan(delta) / self.L

                # 적분
                self.yaw += omega * dt
                self.x   += v * math.cos(self.yaw) * dt
                self.y   += v * math.sin(self.yaw) * dt

                # 공분산 업데이트 및 현재 샘플 기준 분산 계산
                pos_x_var, pos_yaw_var, tw_vx_var, tw_wz_var = self.update_covariances(v, delta, dt, pot_raw)

                # 퍼블리시
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
                
                # pose covariance (x,y,z,roll,pitch,yaw)
                msg.pose.covariance = [0.0]*36
                diag_pose = [
                    pos_x_var,                 # Var(x)
                    self.pos_y_var_const,      # Var(y) - 관측취약: 고정 큰 값
                    self.big_var,              # Var(z)
                    self.big_var,              # Var(roll)
                    self.big_var,              # Var(pitch)
                    pos_yaw_var                # Var(yaw)
                ]
                for i, vv in enumerate(diag_pose):
                    msg.pose.covariance[i*6 + i] = vv
                
                # twist covariance (vx,vy,vz,wx,wy,wz)
                msg.twist.covariance = [0.0]*36
                diag_twist = [
                    tw_vx_var,    # Var(vx)
                    self.big_var, # Var(vy)
                    self.big_var, # Var(vz)
                    self.big_var, # Var(wx)
                    self.big_var, # Var(wy)
                    tw_wz_var     # Var(wz)
                ]
                for i, vv in enumerate(diag_twist):
                    msg.twist.covariance[i*6 + i] = vv

                # 메시지 발행 전 로깅
                if self.debug and (self.line_cnt % 100 == 0):
                    self.get_logger().info(f'Publishing odom: x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.1f}°, v={v:.3f}, ω={math.degrees(omega):.1f}°/s')
                    self.get_logger().info(f'Frame: {self.frame} -> {self.child}, Topic: /odometry/wheel')
                
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

