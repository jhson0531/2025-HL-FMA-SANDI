from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # 1단계: GPS-IMU 보정 노드 먼저 실행
        ExecuteProcess(
            cmd=['python3', '/home/woong/our_ros2_ws/src/gps_imu_calibration.py'],
            name='gps_imu_calibration',
            output='screen',
            on_exit=ExecuteProcess(
                cmd=['bash', '-c', '''
                    echo "GPS-IMU 보정 완료! 나머지 노드들을 시작합니다..."
                    if [ -f "/home/woong/our_ros2_ws/src/imu_calibration_angle.txt" ]; then
                        echo "=== 보정 각도 정보 ==="
                        head -2 /home/woong/our_ros2_ws/src/imu_calibration_angle.txt
                        echo "====================="
                    else
                        echo "보정 각도 파일을 찾을 수 없습니다."
                    fi
                '''],
                output='screen'
            )
        ),
        
        # 2단계: 보정 완료 후 5초 뒤에 나머지 노드들 실행
        TimerAction(
            period=2.0,
            actions=[             
                # Motion Planner 노드
                Node(
                    package='decision_making_pkg',
                    executable='motion_planner_node',
                    name='motion_planner_node',
                    output='screen'
                ),
                
                # Serial Sender 노드
                Node(
                    package='serial_communication_pkg',
                    executable='serial_sender_node',
                    name='serial_sender_node',
                    output='screen'
                ),
                
                # Lidar S2 장애물 감지 시스템
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('lidar_perception_pkg'),
                            'launch',
                            'lidar_s2_obstacle_detection_launch.py'
                        )
                    )
                ),

                # 카메라 인식 시스템
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('camera_perception_pkg'),
                            'launch',
                            'camera_perception.launch.py'
                        )
                    )
                ),
                
                # UTM Pure Pursuit 노드
                # Node(
                #     package='nav_controller',
                #     executable='utm_pure_pursuit',
                #     name='utm_pure_pursuit',
                #     output='screen'
                # ),
                
            ]
        ),
    ])