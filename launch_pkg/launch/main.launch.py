from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='camera_perception_pkg',
        #     executable='image_publisher_node',
        #     name='image_publisher_node',
        #     output='screen'
        # ),
        # Node(
        #     package='camera_perception_pkg',
        #     executable='yolov8_node',
        #     name='yolov8_node',
        #     output='screen'
        # ),
        # Node(
        #     package='camera_perception_pkg',
        #     executable='lane_info_extractor_node',
        #     name='lane_info_extractor_node',
        #     output='screen'
        # ),
        # Node(
        #    package='camera_perception_pkg',
        #    executable='traffic_light_detector_node',
        #     name='traffic_light_detector_node',
        #    output='screen'
        # ),
        # #Node(
        # #    package='lidar_perception_pkg',
        # #    executable='lidar_publisher_node',
        # #    name='lidar_publisher_node',
        # #    output='screen'
        # #),"
        # Node(
        #     package='lidar_perception_pkg',
        #     executable='lidar_processor_node',
        #     name='lidar_processor_node',
        #     output='screen'
        # ),
        # Node(
        #     package='lidar_perception_pkg',
        #     executable='lidar_obstacle_detector_node',
        #     name='lidar_obstacle_detector_node',
        #     output='screen'
        # ),
        Node(
            package='decision_making_pkg',
            executable='motion_planner_node',
            name='motion_planner_node',
            output='screen'
        ),
        # Node(
        #     package='decision_making_pkg',
        #     executable='path_planner_node',
        #     name='path_planner_node',
        #     output='screen'
        # ),
        Node(
         package='serial_communication_pkg',
            executable='serial_sender_node',
            name='serial_sender_node',
            output='screen'
        ),

        # GPS 및 센서 관련 노드들
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         os.path.join(
        #             get_package_share_directory('ublox_gps'),
        #             'launch',
        #             'ublox_gps_node-launch.py'
        #         )
        #     ])
        # ),

        # Node(
        #     package='fix2nmea',
        #     executable='fix2nmea',
        #     name='fix2nmea',
        #     output='screen'
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         os.path.join(
        #             get_package_share_directory('ntrip_client'),
        #             'ntrip_client_launch.py'
        #         )
        #     ])
        # ),

        # Node(
        #     package='iahrs_driver',
        #     executable='driver',
        #     name='iahrs_driver',
        #     output='screen'
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('robot_localization'),
                    'launch',
                    'dual_ekf_navsat_example.launch.py'
                )
            ])
        ),

        # Node(
        #     package='global_to_utm_odometry',
        #     executable='global_to_utm_odometry',
        #     name='global_to_utm_odometry',
        #     output='screen'
        # ),

        Node(
            package='odom_pkg',
            executable='odom_from_ticks',
            name='odom_from_ticks',
            output='screen'
        ),
    ])