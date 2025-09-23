import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy

class ImuProcessedEcho(Node):
    def __init__(self):
        super().__init__('imu_processed_node')  # 이름 명확화
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, '/imu', self.cb, qos)

    def cb(self, msg: Imu):
        la, av, q = msg.linear_acceleration, msg.angular_velocity, msg.orientation
        self.get_logger().info(
            f"lin=({la.x:.3f},{la.y:.3f},{la.z:.3f}) "
            f"ang=({av.x:.3f},{av.y:.3f},{av.z:.3f}) "
            f"q=({q.w:.3f},{q.x:.3f},{q.y:.3f},{q.z:.3f})"
        )

def main():
    rclpy.init(); n = ImuProcessedEcho()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.destroy_node(); rclpy.shutdown()
