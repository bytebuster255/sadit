from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # SADIT Serial Controller node
    sadit_serial_controller = Node(
        package='sadit_serial_control',
        executable='sadit_serial_controller',
        name='sadit_serial_controller',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'baud_rate': 115200,
            'timeout': 1.0,
            'max_pwm': 140,
            'min_pwm': 5,
            'max_cmd_linear': 2.0,
            'max_cmd_angular': 1.5,
            'wheel_separation': 0.5,
            'wheel_radius': 0.048
        }],
        arguments=['--ros-args', '--log-level', 'sadit_serial_controller:=DEBUG']
    )

    return LaunchDescription([
        sadit_serial_controller
    ])

