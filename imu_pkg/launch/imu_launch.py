#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 디렉토리 가져오기
    pkg_dir = get_package_share_directory('imu_pkg')
    
    # 파라미터 파일 경로
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    # Launch 인수 선언
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='IMU 시리얼 포트 경로'
    )
    
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='IMU 시리얼 통신 보드레이트'
    )
    
    # IMU Reader 노드
    imu_reader_node = Node(
        package='imu_pkg',
        executable='imu_reader_node',
        name='imu_reader',
        output='screen',
        parameters=[params_file, {
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
        }],
        remappings=[
            ('/imu/data', '/imu/data'),  # 표준 IMU 토픽으로 발행
        ]
    )
    
    return LaunchDescription([
        port_arg,
        baud_arg,
        imu_reader_node,
    ])
