# sadit_controller/launch/sadit.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os # Bu import'un olduğundan emin ol

def generate_launch_description():

    # --- 1. Gazebo, Dünya ve Robotu sadit_description'dan Başlat ---
    
    # === HATA BURADAYDI, ŞİMDİ DÜZELTİLDİ ===
    gazebo_and_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sadit_description'), 'launch', 'gazebo.launch.py')
        )
    )
    # =======================================

    # --- 2. Kontrolcüleri Başlat ---
    
    spawn_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'joint_broad'],
        shell=True,
        output='screen',
    )
    spawn_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'diff_cont'],
        shell=True,
        output='screen',
    )
    
    # --- 3. Teleop Düğümünü Başlat ---
    
    teleop_keyboard = ExecuteProcess(
        cmd=[
            'xterm', '-e', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
            '--ros-args', '-r', '/cmd_vel:=/diff_cont/cmd_vel_unstamped'
        ],
        shell=True,
        output='screen'
    )

    # --- 4. Her Şeyi LaunchDescription'a Ekle ---
    return LaunchDescription([
        gazebo_and_robot_launch,
        spawn_joint_state_broadcaster,
        spawn_diff_drive_controller,
        teleop_keyboard,
    ])
