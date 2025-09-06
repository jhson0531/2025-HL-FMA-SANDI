import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration
import math, numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
import scipy.interpolate as si
from tf2_ros import Buffer, TransformListener

# ---------- 하드코딩된 웨이포인트 파일 경로 ----------
WAYPOINTS_FILE = "yongin_wp.txt"   # <- 여기만 네 파일 경로로 바꾸면 됨

# ---------- Pure-Pursuit 기본 파라미터 ----------
lookahead_distance = 0.15   # m
speed = 0.10                # m/s

def euler_from_quaternion(x,y,z,w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    t2 = +2.0 * (w * y - z * x); t2 = max(min(t2, +1.0), -1.0)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)  # yaw

def bspline_planning(points, sn):
    """points=[(x,y),...], sn=샘플 개수. 실패 시 입력 그대로."""
    try:
        arr = np.array(points); x, y = arr[:,0], arr[:,1]
        t = range(len(x)); k = 2
        xt = si.splrep(t, x, k=k); yt = si.splrep(t, y, k=k)
        xl, yl = list(xt), list(yt)
        xl[1] = x.tolist() + [0.0]*4
        yl[1] = y.tolist() + [0.0]*4
        tt = np.linspace(0.0, len(x)-1, sn)
        rx = si.splev(tt, xl); ry = si.splev(tt, yl)
        return [(rx[i], ry[i]) for i in range(len(rx))]
    except Exception:
        return list(points)

def pure_pursuit(cx, cy, cyaw, path, index):
    v = speed; target = None
    for i in range(index, len(path)):
        px, py = path[i]
        if math.hypot(cx - px, cy - py) > lookahead_distance:
            target = (px, py); index = i; break
    if target is None:
        px, py = path[-1]; th = math.atan2(py - cy, px - cx); index = len(path)-1
    else:
        th = math.atan2(target[1] - cy, target[0] - cx)

    steer = th - cyaw
    while steer >  math.pi: steer -= 2*math.pi
    while steer < -math.pi: steer += 2*math.pi

    if abs(steer) > math.pi/6:
        steer = (math.pi/4) * (1 if steer > 0 else -1)
        v = 0.0
    return v, steer, index

def normalize_angle(a):
    while a >  math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

class NavigationPPFromUTM(Node):
    def __init__(self):
        super().__init__('navigation_pp_from_utm_hardcoded')

        # -------- 파라미터(필요한 것만 유지) --------
        self.declare_parameter('utm_frame_id', 'utm')       # navsat_transform의 UTM 프레임명
        self.declare_parameter('upsample_factor', 5)        # 보간 배수(427점이면 5배≈2,100점)
        self.declare_parameter('decimate_m', 0.0)           # 0이면 사용 안함
        self.declare_parameter('use_final_heading', True)   # 마지막 각도 사용해 헤딩 정렬
        self.declare_parameter('final_heading_tolerance_deg', 5.0)
        self.declare_parameter('heading_is_bearing_from_north', False)  # 방위각이면 True

        # ---- 하드코딩된 파일 경로 사용 ----
        self.wp_file   = WAYPOINTS_FILE
        self.utm_frame = self.get_parameter('utm_frame_id').value
        self.upsample  = int(self.get_parameter('upsample_factor').value)
        self.decimate_m = float(self.get_parameter('decimate_m').value)
        self.use_final_heading = bool(self.get_parameter('use_final_heading').value)
        self.final_heading_tol = math.radians(float(self.get_parameter('final_heading_tolerance_deg').value))
        self.heading_is_bearing = bool(self.get_parameter('heading_is_bearing_from_north').value)

        # -------- 현재 자세: /odometry/global(map) --------
        self.create_subscription(Odometry, '/odometry/global', self.odom_cb, QoSProfile(depth=10))

        # -------- cmd_vel 퍼블리셔 --------
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # -------- TF (utm -> map) --------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -------- 상태 --------
        self.path = []
        self.i = 0
        self.flag = 0
        self.final_yaw_target = None  # 라디안
        self.x = self.y = self.yaw = 0.0

        # -------- 웨이포인트 로드/변환/보간 --------
        self.path, self.final_yaw_target = self.load_and_transform_waypoints()
        if len(self.path) >= 2:
            # 보간(업샘플)
            self.path = bspline_planning(self.path, max(len(self.path)*self.upsample, len(self.path)))
            self.flag = 2
            self.get_logger().info(f'Path ready: {len(self.path)} points (map).')
        else:
            self.get_logger().error(f'Waypoint count < 2 or file missing: {self.wp_file}')

        # 제어 루프(100Hz)
        self.create_timer(0.01, self.timer_cb)

    # ----- 파일 읽기 + UTM→map 변환 -----
    def load_and_transform_waypoints(self):
        raw = []
        headings = []
        try:
            with open(self.wp_file, 'r') as f:
                seen = set()
                for line in f:
                    s = line.strip()
                    if not s or s.startswith('#'): continue
                    parts = s.replace(',', ' ').split()
                    if len(parts) < 2: continue
                    E, N = float(parts[0]), float(parts[1])
                    key = (round(E,6), round(N,6))
                    if key in seen:  # 완전 중복 라인 스킵
                        continue
                    seen.add(key)
                    raw.append((E, N))
                    headings.append(float(parts[2]) if len(parts) >= 3 else None)
        except Exception as e:
            self.get_logger().error(f'Failed to read waypoints from {self.wp_file}: {e}')
            return [], None

        # 거리 간소화(선택)
        if self.decimate_m > 0 and len(raw) > 2:
            dec = [raw[0]]; acc = 0.0
            for a, b in zip(raw, raw[1:]):
                acc += math.hypot(b[0]-a[0], b[1]-a[1])
                if acc >= self.decimate_m:
                    dec.append(b); acc = 0.0
            if dec[-1] != raw[-1]: dec.append(raw[-1])
            raw = dec

        # UTM → map 변환 (tf2 사용)
        pts_map = []
        for (E, N) in raw:
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = self.utm_frame
            ps.pose.position.x = E
            ps.pose.position.y = N
            ps.pose.orientation.w = 1.0
            try:
                out = self.tf_buffer.transform(ps, 'map', timeout=Duration(seconds=0.5))
                pts_map.append((out.pose.position.x, out.pose.position.y))
            except Exception as e:
                self.get_logger().warn(f'UTM->map transform failed for ({E},{N}): {e}')

        # 최종 헤딩(있다면)
        final_yaw = None
        if self.use_final_heading and headings and headings[-1] is not None:
            hdg_deg = headings[-1]
            if self.heading_is_bearing:
                # 방위각(북=0°, +CW) -> ROS yaw(동=0°, +CCW)
                hdg_deg = 90.0 - hdg_deg
            final_yaw = normalize_angle(math.radians(hdg_deg))

        return pts_map, final_yaw

    # ----- 상태 업데이트 -----
    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

    # ----- 제어 루프 -----
    def timer_cb(self):
        if self.flag != 2 or not self.path: return

        v, w, self.i = pure_pursuit(self.x, self.y, self.yaw, self.path, self.i)
        twist = Twist(); twist.linear.x = v; twist.angular.z = w

        # 최종점 도달 & (옵션) 헤딩 정렬
        if (abs(self.x - self.path[-1][0]) < 0.05 and
            abs(self.y - self.path[-1][1]) < 0.05):

            if self.final_yaw_target is not None:
                err = normalize_angle(self.final_yaw_target - self.yaw)
                if abs(err) > self.final_heading_tol:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5 * err   # 간단 P제어
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.flag = 0
                    self.get_logger().info('Reached final waypoint & heading.')
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.flag = 0
                self.get_logger().info('Reached final waypoint.')

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationPPFromUTM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
