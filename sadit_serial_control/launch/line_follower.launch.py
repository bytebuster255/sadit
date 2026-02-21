from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. sadit_serial_controller düğümünü başlatır.
        # Bu düğüm, cmd_vel komutlarını okur ve Arduino'ya gönderir.
        Node(
            package='sadit_controller',
            executable='sadit_serial_controller',
            name='sadit_serial_controller_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0', # Bağlantı noktasını kontrol edip gerekirse değiştirin
                'baud_rate': 115200
            }]
        ),

        # 2. opencv_line_follower_node düğümünü başlatır.
        # Bu düğüm, görüntüyü işler ve cmd_vel komutlarını yayımlar.
        Node(
            package='sadit_controller',
            executable='opencv_line_follower',
            name='line_follower_node',
            output='screen'
        ),

        # 3. Opsiyonel: İşlenmiş görüntüyü görselleştirmek için RViz'i başlatır.
        # Bu, çizgi algılamasını görsel olarak doğrulamanız için faydalıdır.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_node',
            output='screen',
            arguments=['-d', '/opt/ros/humble/share/rviz2/config/rviz2_default.rviz']
        )
    ])
