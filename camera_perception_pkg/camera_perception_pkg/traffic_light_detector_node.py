#traffic_light_detector_node.py

import cv2
import random
import numpy as np
from typing import Tuple
import sys, os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from interfaces_pkg.msg import DetectionArray, BoundingBox2D, Detection
from std_msgs.msg import String

from .lib import camera_perception_func_lib as CPFL

# ---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_IMAGE_TOPIC_NAME = "image_raw"

# Publish할 토픽 이름
PUB_TOPIC_NAME = "yolov8_traffic_light_info"

# ----------------------------------------------

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector_node')

        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.sub_image_topic = self.declare_parameter('sub_image_topic', SUB_IMAGE_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value

        self.cv_bridge = CvBridge()

        # 안정화를 위한 파라미터 (연속 프레임 확인 임계값)
        self.confirmation_threshold = self.declare_parameter('confirmation_threshold', 3).value

        # 신뢰도 임계값 (이 값 이상일 때만 판단)
        self.score_threshold = self.declare_parameter('score_threshold', 0.7).value

        # 상태 관리 변수
        self.last_published_state = 'None'
        self.current_candidate = 'None'
        self.candidate_counter = 0

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.detection_sub = Subscriber(self, DetectionArray, self.sub_detection_topic, qos_profile=self.qos_profile)
        self.image_sub = Subscriber(self, Image, self.sub_image_topic, qos_profile=self.qos_profile)
        self.ts = ApproximateTimeSynchronizer([self.detection_sub, self.image_sub], queue_size=1, slop=0.5)
        self.ts.registerCallback(self.sync_callback)

        self.publisher = self.create_publisher(String, self.pub_topic, self.qos_profile)

    def sync_callback(self, detection_msg: DetectionArray, image_msg: Image):
        # 이미지는 현재 사용하지 않음 (필요 시 ROI 등 추가)
        # cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg)

        # 이번 프레임에서의 임시 판단값
        current_frame_detection = 'None'

        # 신호등 전용 클래스만 통과 (YOLOv8/커스텀 pt: traffic_sign_*)
        best_det = None
        for det in detection_msg.detections:
            cls = (det.class_name or '').strip()
            if cls in (
                'traffic_sign_red',
                'traffic_sign_yellow',
                'traffic_sign_green',
                'traffic_sign_left_arrow'
            ) and (det.score is not None and det.score >= self.score_threshold):
                if best_det is None or det.score > best_det.score:
                    best_det = det

        if best_det is not None:
            mapping = {
                'traffic_sign_red': 'red',
                'traffic_sign_yellow': 'yellow',
                'traffic_sign_green': 'green',
                'traffic_sign_left_arrow': 'left_arrow'
            }
            current_frame_detection = mapping.get(best_det.class_name, 'None')
        else:
            current_frame_detection = 'None'

        # 매번 현재 상태 발행 (상태 변화와 무관)
        # 상태 변화 체크를 위한 임시 변수
        previous_state = self.last_published_state

        self.last_published_state = current_frame_detection
        color_msg = String()
        color_msg.data = self.last_published_state
        self.publisher.publish(color_msg)

        # 상태 변화 시에만 로그 출력 (디버깅용)
        if current_frame_detection != previous_state:
            self.get_logger().info(f'Traffic light state changed to: {color_msg.data}')
        
        


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
