from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # GPS 및 센서 관련 노드들
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ublox_gps'),
                    'launch',
                    'ublox_gps_node-launch.py'
                )
            ])
        ),

        Node(
            package='fix2nmea',
            executable='fix2nmea',
            name='fix2nmea',
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ntrip_client'),
                    'ntrip_client_launch.py'
                )
            ])
        )
    ])