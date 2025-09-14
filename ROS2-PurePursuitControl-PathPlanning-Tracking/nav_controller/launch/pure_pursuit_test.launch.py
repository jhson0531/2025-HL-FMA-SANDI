#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch.conditions

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_visualizer',
            default_value='true',
            description='시각화 노드 실행 여부'
        ),
        
        # UTM 테스트 퍼블리셔 노드
        Node(
            package='nav_controller',
            executable='utm_test_publisher',
            name='utm_test_publisher',
            output='screen',
            parameters=[]
        ),
        
        # UTM Pure Pursuit 테스트 제어 노드
        Node(
            package='nav_controller',
            executable='utm_pure_pursuit_test',
            name='utm_pure_pursuit_test',
            output='screen',
            parameters=[]
        ),
        
        # 시각화 노드 (조건부 실행)
        Node(
            package='nav_controller',
            executable='pure_pursuit_visualizer',
            name='pure_pursuit_visualizer',
            output='screen',
            condition=launch.conditions.IfCondition(LaunchConfiguration('use_visualizer'))
        ),
    ])
