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
        #self.declare_parameter("model", "yolov8m.pt")
        self.declare_parameter("model", "best.pt")
        
        # 추론 하드웨어 선택 (cpu / gpu) 
        self.declare_parameter("device", "cpu")
        #self.declare_parameter("device", "cuda:0")
        #----------------------------------------------
        
        self.declare_parameter("threshold", 0.5)
        self.declare_parameter("enable", True)
        self.declare_parameter("image_reliability",
                               QoSReliabilityPolicy.RELIABLE)

        self.get_logger().info('Yolov8Node created')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring {self.get_name()}')

        self.model = self.get_parameter(
            "model").get_parameter_value().string_value

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
            self.yolo = YOLO(self.model)  # 모델 로딩
            self.yolo.fuse()
        except FileNotFoundError:
            self.get_logger().error(f"Error: Model file '{self.model}' not found!")
            return TransitionCallbackReturn.FAILURE
        except Exception as e:
            self.get_logger().error(f"Error while loading model '{self.model}': {str(e)}")
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

        del self.yolo
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
    def parse_hypothesis(self, results: Results) -> List[Dict]:

        hypothesis_list = []

        box_data: Boxes
        for box_data in results.boxes:
            hypothesis = {
                "class_id": int(box_data.cls),
                "class_name": self.yolo.names[int(box_data.cls)],
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
            results = self.yolo.predict( # YOLOv8 모델로 추론 
                source=cv_image,
                verbose=False,
                stream=False,
                conf=self.threshold, # 지정한 신뢰도 이상만 추론 
                device=self.device # cpu / gpu 선택
            )
            results: Results = results[0].cpu() 

            if results.boxes: # 박스 있으면 
                hypothesis = self.parse_hypothesis(results) # 클래스 id/이름/점수 리스트 생성
                boxes = self.parse_boxes(results) # 위치/크기를 담은 BoundingBox2D 리스트 생성

            if results.masks: # 마스크(윤곽선) 있으면 
                masks = self.parse_masks(results) # 세그멘테이션 결과(마스크)가 있으면 파싱해서 Mask 메시지 리스트로 변환 

            if results.keypoints: # 키포인트 있으면
                keypoints = self.parse_keypoints(results) #키포인트 결과가 있으면 파싱해서 KeyPoint2DArray 리스트로 변환.

            # create detection msgs
            detections_msg = DetectionArray() # 최종 퍼블리시할 컨테이너 메세지 생성, 이 안에 여러개의 detection 메시지를 담김.

            for i in range(len(results)): # yolo가 검출한 객체만큼 반복해서 

                aux_msg = Detection()

                if results.boxes:
                    aux_msg.class_id = hypothesis[i]["class_id"]
                    aux_msg.class_name = hypothesis[i]["class_name"]
                    aux_msg.score = hypothesis[i]["score"]

                    aux_msg.bbox = boxes[i]

                if results.masks:
                    aux_msg.mask = masks[i]

                if results.keypoints:
                    aux_msg.keypoints = keypoints[i]

                detections_msg.detections.append(aux_msg) # detection 메세지 완성해서 detectionArray에 추가.

            # publish detections
            detections_msg.header = msg.header # 원본 이미지의 헤더 정보 사용
            self._pub.publish(detections_msg)  # 추론 결과를 detections 토픽으로 발행 (위에서 만든 함수)

            del results
            del cv_image # 메모리 정리.


def main():
    rclpy.init()
    node = Yolov8Node()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
