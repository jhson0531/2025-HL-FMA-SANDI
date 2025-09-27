from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 워크스페이스 경로 환경변수 설정
    workspace_path = os.environ.get('WORKSPACE_PATH', '/home/woong/our_ros2_ws')

    return LaunchDescription([
        # 환경변수 설정
        SetEnvironmentVariable('WORKSPACE_PATH', workspace_path),

        # 1단계: IMU 패키지 먼저 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('imu_pkg'),
                    'launch',
                    'imu_launch.py'
                )
            )
        ),

        # 2단계: IMU 실행 후 5초 뒤에 GPS-IMU 보정 노드 실행
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', f'{workspace_path}/src/gps_imu_calibration.py'],
                    name='gps_imu_calibration',
                    output='screen',
                    on_exit=ExecuteProcess(
                        cmd=['bash', '-c', f'''
                            echo "GPS-IMU 보정 완료! 나머지 노드들을 시작합니다..."
                            if [ -f "{workspace_path}/src/imu_calibration_angle.txt" ]; then
                                echo "=== 보정 각도 정보 ==="
                                head -2 {workspace_path}/src/imu_calibration_angle.txt
                                echo "====================="
                            else
                                echo "보정 각도 파일을 찾을 수 없습니다."
                            fi
                        '''],
                        output='screen'
                    )
                )
            ]
        ),
        
        # 3단계: 보정 완료 후 10초 뒤에 나머지 노드들 실행 (IMU + 보정 완료 후)
        TimerAction(
            period=7.0,
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
                
                
                
            ]
        )
        # TimerAction(
        #     period=10.0,
        #     actions=[
        #         # UTM Pure Pursuit 노드
        #         Node(
        #             package='nav_controller',
        #             executable='utm_pure_pursuit',
        #             name='utm_pure_pursuit',
        #             output='screen',
        #             parameters=[
        #                 {'waypoint_ver1_path': f'{workspace_path}/src/waypoints/full_wp_ver1.txt'},
        #                 {'waypoint_ver2_path': f'{workspace_path}/src/waypoints/full_wp_ver2.txt'},
        #                 {'waypoint_ver3_path': f'{workspace_path}/src/waypoints/full_wp_ver3.txt'}
        #             ]
        #         )
        #     ]
        # )
    ])
