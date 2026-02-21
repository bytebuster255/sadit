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
        source_file=PathJoinSubstitution([bringup_dir, 'config', 'nav2_params.yaml']),
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([bringup_dir, 'config', 'nav2_params.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_yaml_filename_cmd = DeclareLaunchArgument(
        'yaml_filename',
        default_value='nav2_params.yaml',
        description='YAML file name')

    # Specify the actions
    start_nav2_controller_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=[('cmd_vel', 'cmd_vel'),
                   ('odom', 'odom'),
                   ('scan', 'sadit/scan')])

    start_nav2_planner_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        remappings=[('scan', 'sadit/scan')])

    start_nav2_behaviors_cmd = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params])

    start_nav2_bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params])

    start_nav2_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[configured_params, {
            'node_names': ['map_server', 'amcl', 'controller_server', 'planner_server', 'behavior_server', 'bt_navigator'],
            'autostart': True,
            'attempt_respawn_reconnection': True,
            'bond_timeout': 4.0
        }])

    start_nav2_map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params, {'yaml_filename': PathJoinSubstitution([bringup_dir, 'maps', 'sadit.yaml'])}])

    start_nav2_amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params],
        remappings=[('scan', 'sadit/scan')])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_yaml_filename_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_nav2_controller_cmd)
    ld.add_action(start_nav2_planner_cmd)
    ld.add_action(start_nav2_behaviors_cmd)
    ld.add_action(start_nav2_bt_navigator_cmd)
    ld.add_action(start_nav2_lifecycle_manager_cmd)
    ld.add_action(start_nav2_map_server_cmd)
    ld.add_action(start_nav2_amcl_cmd)

    return ld 
