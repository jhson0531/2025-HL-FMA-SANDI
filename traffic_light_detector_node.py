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

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.detection_sub = Subscriber(self, DetectionArray, self.sub_detection_topic, qos_profile=self.qos_profile)
        self.image_sub = Subscriber(self, Image, self.sub_image_topic, qos_profile=self.qos_profile)
        
        # 동기화 객체를 생성. 이 객체는 [self.detection_sub, self.image_sub] 두 구독자에게 메시지가 도착하는 것을 감시
        self.ts = ApproximateTimeSynchronizer([self.detection_sub, self.image_sub], queue_size=1, slop=0.5)
        self.ts.registerCallback(self.sync_callback)

        # 최종적으로 판별된 신호등 색상을 발행할 발행자를 생성
        self.publisher = self.create_publisher(String, self.pub_topic, self.qos_profile)

    def sync_callback(self, detection_msg: DetectionArray, image_msg: Image):
        cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg)
        
        # 신호등을 찾았는지 여부를 기록할 때 쓰는 플래그 변수
        traffic_light_detected = False
        # detection_msg에 포함된 모든 탐지 객체들을 하나씩 순회
        for detection in detection_msg.detections:
            # 현재 순회중인 객체의 클래스 이름이 traffic_light인지 확인
            if detection.class_name == 'left_arrow':
                traffic_light_color = "Left_arrow"
            elif detection.class_name == 'stop_sign':
                traffic_light_color = "Stop"
            else:
                traffic_light_color = "Unknow"

            # 발행할 String 객체를 생성. 
            # Publish traffic light color as string
            color_msg = String()
            color_msg.data = traffic_light_color
            print(f'traffic light: {color_msg.data}') 
            self.publisher.publish(color_msg)
            traffic_light_detected = True
            break  # Only process the first detected traffic light

        if not traffic_light_detected:
            # Publish 'None' if no traffic light is detected
            color_msg = String()
            color_msg.data = 'None'
            print(f'traffic light: {color_msg.data}')
            self.publisher.publish(color_msg)


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
