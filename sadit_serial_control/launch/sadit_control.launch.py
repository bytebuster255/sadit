#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Arduino connection'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='9600',
        description='Baud rate for serial communication'
    )
    
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='255',
        description='Maximum linear speed (PWM value 0-255)'
    )
    
    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='30.0',
        description='Maximum angular speed (degrees)'
    )
    
    # Scaling parametreleri
    linear_scale_factor_arg = DeclareLaunchArgument(
        'linear_scale_factor',
        default_value='127.5',
        description='Linear velocity scaling factor (PWM per m/s) - 2.0 m/s = 255 PWM'
    )
    
    angular_scale_factor_arg = DeclareLaunchArgument(
        'angular_scale_factor',
        default_value='30.0',
        description='Angular velocity scaling factor (degrees per rad/s)'
    )
    
    min_pwm_threshold_arg = DeclareLaunchArgument(
        'min_pwm_threshold',
        default_value='0',
        description='Minimum PWM threshold'
    )
    
    max_cmd_linear_arg = DeclareLaunchArgument(
        'max_cmd_linear',
        default_value='2.0',
        description='Maximum accepted linear command (m/s)'
    )
    
    max_cmd_angular_arg = DeclareLaunchArgument(
        'max_cmd_angular',
        default_value='1.5',
        description='Maximum accepted angular command (rad/s)'
    )
    
    # SADIT robot parametreleri
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.5',
        description='Distance between wheels (m)'
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.048',
        description='Wheel radius (m)'
    )
    
    enable_4wd_arg = DeclareLaunchArgument(
        'enable_4wd',
        default_value='true',
        description='Enable 4-wheel drive'
    )
    
    # SADIT serial controller node
    sadit_serial_controller_node = Node(
        package='sadit_serial_control',
        executable='sadit_serial_controller',
        name='sadit_serial_controller',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            'linear_scale_factor': LaunchConfiguration('linear_scale_factor'),
            'angular_scale_factor': LaunchConfiguration('angular_scale_factor'),
            'min_pwm_threshold': LaunchConfiguration('min_pwm_threshold'),
            'max_cmd_linear': LaunchConfiguration('max_cmd_linear'),
            'max_cmd_angular': LaunchConfiguration('max_cmd_angular'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'enable_4wd': LaunchConfiguration('enable_4wd'),
        }],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('arduino_status', '/arduino_status'),
            ('joint_states', '/joint_states'),
            ('current_speed', '/current_speed'),
            ('battery_voltage', '/battery_voltage'),
        ]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        max_linear_speed_arg,
        max_angular_speed_arg,
        linear_scale_factor_arg,
        angular_scale_factor_arg,
        min_pwm_threshold_arg,
        max_cmd_linear_arg,
        max_cmd_angular_arg,
        wheel_separation_arg,
        wheel_radius_arg,
        enable_4wd_arg,
        sadit_serial_controller_node,
    ])
