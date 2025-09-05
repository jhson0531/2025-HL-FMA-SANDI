# Copyright 2018 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_localization_dir = get_package_share_directory('robot_localization')
    parameters_file_dir = os.path.join(robot_localization_dir, 'params')
    parameters_file_path = os.path.join(parameters_file_dir, 'dual_ekf_navsat_example.yaml')
    os.environ['FILE_PATH'] = str(parameters_file_dir)
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'output_final_position',
            default_value='false'),
        launch.actions.DeclareLaunchArgument(
            'output_location',
	    default_value='~/dual_ekf_navsat_example_debug.txt'),
	
    launch_ros.actions.Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_odom',
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('odometry/filtered', 'odometry/local')]           
           ),
    launch_ros.actions.Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_map',
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('odometry/filtered', 'odometry/global')]
           ),           
    # Static transform: base_link -> gps
    # GPS 센서의 실제 위치에 맞게 offset 조정 필요
    # arguments: [x, y, z, roll, pitch, yaw, parent_frame, child_frame]
    launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_gps',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'gps']
    ),

    launch_ros.actions.Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('imu/data', 'imu/data'),
                        ('gps/fix', 'ublox_gps_node/fix'),
                        ('gps/filtered', 'gps/filtered'),
                        ('odometry/gps', 'odometry/gps'),
                        ('odometry/filtered', 'odometry/global')]           

           )           
])
