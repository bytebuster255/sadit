#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = FindPackageShare('sadit_mapping')
    
    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'yaml_filename': LaunchConfiguration('yaml_filename')}

    configured_params = RewrittenYaml(
        source_file=PathJoinSubstitution([bringup_dir, 'config', 'slam_params.yaml']),
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([bringup_dir, 'config', 'slam_params.yaml']),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    declare_yaml_filename_cmd = DeclareLaunchArgument(
        'yaml_filename',
        default_value='slam_params.yaml',
        description='YAML file name')

    # Specify the actions
    start_slam_toolbox_cmd = Node(
        parameters=[configured_params],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_yaml_filename_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_slam_toolbox_cmd)

    return ld 