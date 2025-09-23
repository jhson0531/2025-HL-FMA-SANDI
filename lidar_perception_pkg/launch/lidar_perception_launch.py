#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 파라미터 선언
    start_angle = LaunchConfiguration('start_angle', default='0')
    end_angle = LaunchConfiguration('end_angle', default='30')
    range_min = LaunchConfiguration('range_min', default='0.5')
    range_max = LaunchConfiguration('range_max', default='2.0')
    consec_count = LaunchConfiguration('consec_count', default='5')

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'start_angle',
            default_value=start_angle,
            description='감지할 각도 범위의 시작값 (0-359도)'
        ),
        
        DeclareLaunchArgument(
            'end_angle',
            default_value=end_angle,
            description='감지할 각도 범위의 끝값 (0-359도)'
        ),
        
        DeclareLaunchArgument(
            'range_min',
            default_value=range_min,
            description='감지할 거리 범위의 최소값 (미터)'
        ),
        
        DeclareLaunchArgument(
            'range_max',
            default_value=range_max,
            description='감지할 거리 범위의 최대값 (미터)'
        ),
        
        DeclareLaunchArgument(
            'consec_count',
            default_value=consec_count,
            description='연속 감지 횟수 (안정성 확인용)'
        ),

        # Lidar processor node
        Node(
            package='lidar_perception_pkg',
            executable='lidar_processor_node',
            name='lidar_processor_node',
            output='screen',
            parameters=[{
                'start_angle': start_angle,
                'end_angle': end_angle,
                'range_min': range_min,
                'range_max': range_max,
                'consec_count': consec_count
            }]
        ),

        # Lidar obstacle detector node
        Node(
            package='lidar_perception_pkg',
            executable='lidar_obstacle_detector_node',
            name='lidar_obstacle_detector_node',
            output='screen',
            parameters=[{
                'start_angle': start_angle,
                'end_angle': end_angle,
                'range_min': range_min,
                'range_max': range_max,
                'consec_count': consec_count
            }]
        ),
    ])