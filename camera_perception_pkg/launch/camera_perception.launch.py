from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 카메라 이미지 퍼블리셔
        Node(
            package='camera_perception_pkg',
            executable='image_publisher_node',
            name='image_publisher_node',
            output='screen'
        ),

        # 2. YOLO 객체 감지 노드
        Node(
            package='camera_perception_pkg',
            executable='yolov8_node',
            name='yolov8_node',
            output='screen',
            parameters=[
                {'model': 'src/best.pt'},  # YOLO 모델 파일 경로
                {'device': 'cuda:0'},       # GPU 사용
                {'threshold': 0.25}        # 감지 임계값
            ]
        ),

        # 3. 신호등 감지 노드
        Node(
            package='camera_perception_pkg',
            executable='traffic_light_detector_node',
            name='traffic_light_detector_node',
            output='screen',
            parameters=[
                {'confirmation_threshold': 3},  # 안정화 임계값
                {'score_threshold': 0.7}        # 신뢰도 임계값
            ]
        )
    ])
