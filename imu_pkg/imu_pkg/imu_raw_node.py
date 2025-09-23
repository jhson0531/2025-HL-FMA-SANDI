import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos import QoSProfile, ReliabilityPolicy
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
# create_subscription(..., qos)


class EchoRaw(Node):
    def __init__(self):
        super().__init__('echo_iahrs_raw')
        self.create_subscription(String, '/iahrs/raw', self.cb, 10)

    def cb(self, msg: String):
        self.get_logger().info(msg.data)

def main():
    rclpy.init(); n = EchoRaw()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.destroy_node(); rclpy.shutdown()
