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
    sadit_description_dir = FindPackageShare('sadit_description')
    
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
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([bringup_dir, 'config', 'slam_params.yaml']),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    declare_yaml_filename_cmd = DeclareLaunchArgument(
        'yaml_filename',
        default_value='slam_params.yaml',
        description='YAML file name')

    # Include Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            sadit_description_dir,
            'launch',
            'gazebo.launch.py'
        ])
    )

    # TF Static Publisher for map to odom (if not already in gazebo launch)
    tf_static_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # SLAM Toolbox
    start_slam_toolbox_cmd = Node(
        parameters=[configured_params],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        remappings=[('scan', 'sadit/scan'),
                   ('map', 'map'),
                   ('map_metadata', 'map_metadata')])

    # Map Saver
    map_saver_cmd = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver',
        output='screen',
        arguments=['-f', PathJoinSubstitution([bringup_dir, 'maps', 'sadit_map'])])

    # RViz2
    rviz2_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([bringup_dir, 'config', 'mapping.rviz'])],
        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_yaml_filename_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(gazebo_launch)
    ld.add_action(tf_static_publisher)
    ld.add_action(start_slam_toolbox_cmd)
    ld.add_action(rviz2_cmd)

    return ld 