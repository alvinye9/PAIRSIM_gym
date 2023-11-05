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

# FIXME: This file is for unit-testing purpose ONLY.
# DO NOT INCLUDE this file in the launch procedure of the whole stack.
# Modification of this file will NOT be reflected in the launch of the whole stack.

from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import LaunchConfiguration
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    """
    robot_localization_dir = get_package_share_directory('robot_localization')
    this_package_dir = get_package_share_directory('localization_fusion')
    parameters_file_dir = os.path.join(this_package_dir, 'config')
    parameters_fusion_cfg_path = os.path.join(parameters_file_dir, 'odom_fusion.yaml')
    parameters_odom_path = os.path.join(parameters_file_dir, 'odom.yaml')
    os.environ['FILE_PATH'] = str(parameters_file_dir)
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'output_final_position',
            default_value='false'),
        launch.actions.DeclareLaunchArgument(
            'output_location',
	    default_value='screen'),
	
    launch_ros.actions.Node(
            package='localization_fusion', 
            executable='wheelOdomRepub', 
            name='wheel_odometer_repub',
            output='screen',
            parameters=[parameters_odom_path]
           ),

    launch_ros.actions.Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[parameters_fusion_cfg_path],
            remappings=[('odometry/filtered', '/filtered_odometry/global')]           
           ),
    #launch_ros.actions.Node(
    #        package='robot_localization', 
    #        executable='ekf_node', 
    #        name='ekf_filter_node_map',
    #        output='screen',
    #        parameters=[parameters_file_path],
    #        remappings=[('odometry/filtered', 'odometry/global')]
    #       ),           
    
    launch_ros.actions.Node(
            package='robot_localization', 
            executable='navsat_transform_node', 
            name='navsat_transform',
	        output='screen',
            parameters=[parameters_fusion_cfg_path],
            remappings=[('imu/data', '/novatel_bottom/imu'),
                        ('gps/fix', '/novatel_top/fix'), 
                        ('gps/filtered', '/novatel_top/bestpos'),
                        ('odometry/filtered', '/filtered_odometry/global'),
                        ('odometry/gps', '/filtered_odometry/all_fused')]           
           )
    ])
    """
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="False", description="Use simulation clock if True"
    )
    wheel_odom_node =  Node(
        package='wheel_odometer_integrator',
        executable='wheel_odom_integrator_node',
        name='wheel_odometer_integrator',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('wheel_odometer_integrator'), 'config', 'odom.yaml'),
            {'use_sim_time': LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ('wheel_speed_report',  '/raptor_dbw_interface/wheel_speed_report'),
            ('steering_report',     '/raptor_dbw_interface/steering_report'),
            ('odometry',            '/odometry/wheel_odom'),
            ('odometry_feedback',   '/odometry/filtered'),
        ]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        wheel_odom_node,
    ])
