#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 런치 인수 선언
        DeclareLaunchArgument(
            'reference_lat',
            default_value='37.5665',
            description='UTM 변환을 위한 기준 위도 (서울 기본값)'
        ),
        DeclareLaunchArgument(
            'reference_lon',
            default_value='126.9780',
            description='UTM 변환을 위한 기준 경도 (서울 기본값)'
        ),
        
        # UTM 변환기 노드
        Node(
            package='utm_odometry_converter',
            executable='utm_converter',
            name='utm_odometry_converter',
            output='screen',
            parameters=[{
                'reference_lat': LaunchConfiguration('reference_lat'),
                'reference_lon': LaunchConfiguration('reference_lon'),
            }],
            remappings=[
                ('/odometry/global', '/odometry/global'),
                ('/odometry/utm', '/odometry/utm'),
            ]
        ),
    ])
