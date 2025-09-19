#yolov8_node.py 

# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


from typing import List, Dict

import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState

from cv_bridge import CvBridge

from ultralytics import YOLO
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
from ultralytics.engine.results import Masks
from ultralytics.engine.results import Keypoints
from torch import cuda

from sensor_msgs.msg import Image
from interfaces_pkg.msg import Point2D
from interfaces_pkg.msg import BoundingBox2D
from interfaces_pkg.msg import Mask
from interfaces_pkg.msg import KeyPoint2D
from interfaces_pkg.msg import KeyPoint2DArray
from interfaces_pkg.msg import Detection
from interfaces_pkg.msg import DetectionArray

from std_srvs.srv import SetBool


class Yolov8Node(LifecycleNode):

    def __init__(self, **kwargs) -> None:
        super().__init__("yolov8_node", **kwargs)
        
        #---------------Variable Setting---------------
        # 딥러닝 모델 pt 파일명 작성
        self.declare_parameter("detection_model", "traffic_sign_detection.pt")  # traffic_sign detection용
        
        # 추론 하드웨어 선택 (cpu / gpu) 
        self.declare_parameter("device", "cuda:0")
        #----------------------------------------------
        
        self.declare_parameter("threshold", 0.5)
        self.declare_parameter("enable", True)
        self.declare_parameter("image_reliability",
                               QoSReliabilityPolicy.RELIABLE)

        self.get_logger().info('Yolov8Node created')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring {self.get_name()}')

        self.detection_model = self.get_parameter(
            "detection_model").get_parameter_value().string_value

        self.device = self.get_parameter(
            "device").get_parameter_value().string_value

        self.threshold = self.get_parameter(
            "threshold").get_parameter_value().double_value

        self.enable = self.get_parameter(
            "enable").get_parameter_value().bool_value

        self.reliability = self.get_parameter(
            "image_reliability").get_parameter_value().integer_value

        self.image_qos_profile = QoSProfile(
            reliability=self.reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # 퍼블리셔 생성해서 전송 함수
        self._pub = self.create_lifecycle_publisher(
            DetectionArray, "detections", 10)
        self._srv = self.create_service(
            SetBool, "enable", self.enable_cb # enable_cb의 스위치
        )
        self.cv_bridge = CvBridge()

        return TransitionCallbackReturn.SUCCESS

    def enable_cb(self, request, response):
        self.enable = request.data
        response.success = True
        return response

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating {self.get_name()}')

        try:
            # Detection 모델 로딩 (traffic_sign용)
            self.yolo_detection = YOLO(self.detection_model)
            self.yolo_detection.fuse()
            self.get_logger().info(f'Detection model loaded: {self.detection_model}')
            
        except FileNotFoundError as e:
            self.get_logger().error(f"Error: Model file not found! {str(e)}")
            return TransitionCallbackReturn.FAILURE
        except Exception as e:
            self.get_logger().error(f"Error while loading model: {str(e)}")
            return TransitionCallbackReturn.FAILURE

        # subs
        self._sub = self.create_subscription(
            Image,
            "image_raw",
            self.image_cb, # 메세지를 수신할 때마다 호출될 함수
            self.image_qos_profile
        ) 
        # create_subscription 함수를 통해 서브스크라이버를 생성하면 
        # 해당 토픽으로 전달되는 메세지가 있을 때마다 데이터콜백 함수가 자동으로 실행된다.

        super().on_activate(state)

        return TransitionCallbackReturn.SUCCESS

    # YOLO 모델 사용 중지 + 구독 해제 + GPU 메모리 정리
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating {self.get_name()}')

        # 모델 삭제
        if hasattr(self, 'yolo_detection'):
            del self.yolo_detection
            
        if 'cuda' in self.device:
            self.get_logger().info("Clearing CUDA cache")
            cuda.empty_cache()

        self.destroy_subscription(self._sub)
        self._sub = None

        super().on_deactivate(state)

        return TransitionCallbackReturn.SUCCESS

    # 퍼블리셔와 서비스 제거 + QoS 프로파일 삭제
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up {self.get_name()}')

        self.destroy_publisher(self._pub)

        del self.image_qos_profile

        return TransitionCallbackReturn.SUCCESS
    
    # 무엇이 검출됐는지(label+확률) 를 리스트로 정리.
    def parse_hypothesis(self, results: Results, model) -> List[Dict]:

        hypothesis_list = []

        box_data: Boxes
        for box_data in results.boxes:
            hypothesis = {
                "class_id": int(box_data.cls),
                "class_name": model.names[int(box_data.cls)],
                "score": float(box_data.conf)
            }
            hypothesis_list.append(hypothesis)

        return hypothesis_list

    # 검출된 물체의 위치와 크기를 ROS 메시지로 변환 
    # YOLO box [100, 200, 50, 80] → ROS BoundingBox2D(center=(100,200), size=(50,80))
    def parse_boxes(self, results: Results) -> List[BoundingBox2D]:

        boxes_list = []

        box_data: Boxes
        for box_data in results.boxes:

            msg = BoundingBox2D()

            # get boxes values
            box = box_data.xywh[0]
            msg.center.position.x = float(box[0])
            msg.center.position.y = float(box[1])
            msg.size.x = float(box[2])
            msg.size.y = float(box[3])

            # append msg
            boxes_list.append(msg)

        return boxes_list

    # YOLO로 검출된 마스크(윤곽선)를 ROS 메시지로 변환
    def parse_masks(self, results: Results) -> List[Mask]:

        masks_list = []

        def create_point2d(x: float, y: float) -> Point2D:
            p = Point2D()
            p.x = x
            p.y = y
            return p

        mask: Masks
        for mask in results.masks:

            msg = Mask()

            msg.data = [create_point2d(float(ele[0]), float(ele[1]))
                        for ele in mask.xy[0].tolist()]
            msg.height = results.orig_img.shape[0]
            msg.width = results.orig_img.shape[1]

            masks_list.append(msg)

        return masks_list

    # YOLO로 검출된 키포인트를 ROS 메시지로 변환 
    def parse_keypoints(self, results: Results) -> List[KeyPoint2DArray]:

        keypoints_list = []

        points: Keypoints
        for points in results.keypoints:

            msg_array = KeyPoint2DArray()

            if points.conf is None:
                continue

            for kp_id, (p, conf) in enumerate(zip(points.xy[0], points.conf[0])):

                if conf >= self.threshold:
                    msg = KeyPoint2D()

                    msg.id = kp_id + 1
                    msg.point.x = float(p[0])
                    msg.point.y = float(p[1])
                    msg.score = float(conf)

                    msg_array.data.append(msg)

            keypoints_list.append(msg_array)

        return keypoints_list

    # 이미지 들어오면 매번 호출.
    def image_cb(self, msg: Image) -> None: # 수신된 이미지는 msg로 전달됨 
        print(msg.header) # 시간, frame_id 출력해서 디버깅용

        if self.enable: # enable 켜져 있을 때만

            # convert image + predict
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg) # OpenCV 이미지로 변환 
            
            # Detection 모델로 추론 (traffic_sign용)
            detection_results = self.yolo_detection.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                conf=self.threshold,
                device=self.device
            )
            detection_results: Results = detection_results[0].cpu()

            # create detection msgs
            detections_msg = DetectionArray() # 최종 퍼블리시할 컨테이너 메세지 생성

            # Detection 결과 처리 (traffic_sign)
            if detection_results.boxes:
                detection_hypothesis = self.parse_hypothesis(detection_results, self.yolo_detection)
                detection_boxes = self.parse_boxes(detection_results)
                
                for i in range(len(detection_results.boxes)):
                    aux_msg = Detection()
                    aux_msg.class_id = detection_hypothesis[i]["class_id"]
                    aux_msg.class_name = detection_hypothesis[i]["class_name"]
                    aux_msg.score = detection_hypothesis[i]["score"]
                    aux_msg.bbox = detection_boxes[i]
                    detections_msg.detections.append(aux_msg)

            # publish detections
            detections_msg.header = msg.header # 원본 이미지의 헤더 정보 사용
            self._pub.publish(detections_msg)  # 추론 결과를 detections 토픽으로 발행

            # 메모리 정리
            del detection_results
            del cv_image


def main():
    rclpy.init()
    node = Yolov8Node()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
