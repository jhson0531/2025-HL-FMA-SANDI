#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # SLLidar S2 parameters
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB1')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='1000000') # for s2 is 1000000
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='DenseBoost')

    # Obstacle detector parameters
    start_angle = LaunchConfiguration('start_angle', default='0')
    end_angle = LaunchConfiguration('end_angle', default='90')
    range_min = LaunchConfiguration('range_min', default='0.3')
    range_max = LaunchConfiguration('range_max', default='3.0')
    consec_count = LaunchConfiguration('consec_count', default='5')

    return LaunchDescription([
        # SLLidar S2 arguments
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),

        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        # Obstacle detector arguments (각 구간별 각도 및 거리 범위 설정)
        DeclareLaunchArgument(
            'front_angle_start',
            default_value='345',
            description='전방 감지 각도 범위의 시작값 (0-359도)'
        ),

        DeclareLaunchArgument(
            'front_angle_end',
            default_value='15',
            description='전방 감지 각도 범위의 끝값 (0-359도)'
        ),

        DeclareLaunchArgument(
            'front_range_min',
            default_value='0.3',
            description='전방 감지 거리 범위의 최소값 (미터)'
        ),

        DeclareLaunchArgument(
            'front_range_max',
            default_value='3.0',
            description='전방 감지 거리 범위의 최대값 (미터)'
        ),

        DeclareLaunchArgument(
            'left_angle_start',
            default_value='240',
            description='좌측 감지 각도 범위의 시작값 (0-359도)'
        ),

        DeclareLaunchArgument(
            'left_angle_end',
            default_value='300',
            description='좌측 감지 각도 범위의 끝값 (0-359도)'
        ),

        DeclareLaunchArgument(
            'left_range_min',
            default_value='0.3',
            description='좌측 감지 거리 범위의 최소값 (미터)'
        ),

        DeclareLaunchArgument(
            'left_range_max',
            default_value='2.0',
            description='좌측 감지 거리 범위의 최대값 (미터)'
        ),

        DeclareLaunchArgument(
            'right_angle_start',
            default_value='60',
            description='우측 감지 각도 범위의 시작값 (0-359도)'
        ),

        DeclareLaunchArgument(
            'right_angle_end',
            default_value='120',
            description='우측 감지 각도 범위의 끝값 (0-359도)'
        ),

        DeclareLaunchArgument(
            'right_range_min',
            default_value='0.3',
            description='우측 감지 거리 범위의 최소값 (미터)'
        ),

        DeclareLaunchArgument(
            'right_range_max',
            default_value='2.0',
            description='우측 감지 거리 범위의 최대값 (미터)'
        ),

        DeclareLaunchArgument(
            'consec_count',
            default_value='5',
            description='연속 감지 횟수 (안정성 확인용)'
        ),

        # SLLidar S2 node
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type': channel_type,
                         'serial_port': serial_port,
                         'serial_baudrate': serial_baudrate,
                         'frame_id': frame_id,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate,
                         'scan_mode': scan_mode}],
            output='screen'),

        # Lidar processor node
        Node(
            package='lidar_perception_pkg',
            executable='lidar_processor_node',
            name='lidar_processor_node',
            output='screen'
        ),

        # Lidar obstacle detector node
        Node(
            package='lidar_perception_pkg',
            executable='lidar_obstacle_detector_node',
            name='lidar_obstacle_detector_node',
            output='screen',
            parameters=[
                {'front_angle_start': 342},
                {'front_angle_end': 18},
                {'front_range_min': 0.0},
                {'front_range_max': 2.0},
                {'left_angle_start': 262},
                {'left_angle_end': 278},
                {'left_range_min': 0.0},
                {'left_range_max': 4.5},
                {'right_angle_start': 63},
                {'right_angle_end': 117},
                {'right_range_min': 0.0},
                {'right_range_max': 2.5},
                {'consec_count': 5}
            ]
        ),
    ])
