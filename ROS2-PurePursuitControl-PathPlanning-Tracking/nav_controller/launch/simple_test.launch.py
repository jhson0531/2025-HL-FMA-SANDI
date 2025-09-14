from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch.conditions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_visualizer',
            default_value='true',
            description='시각화 노드 실행 여부'
        ),
        Node(
            package='nav_controller',
            executable='utm_test_publisher',
            name='utm_test_publisher',
            output='screen',
            parameters=[]
        ),
        Node(
            package='nav_controller',
            executable='utm_pure_pursuit_test',
            name='utm_pure_pursuit_test',
            output='screen',
            parameters=[]
        ),
        Node(
            package='nav_controller',
            executable='simple_visualizer',
            name='simple_visualizer',
            output='screen',
            condition=launch.conditions.IfCondition(LaunchConfiguration('use_visualizer'))
        ),
    ])
