from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # USB Camera Node
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="camera",
            remappings=[
                ("image_raw", "camera/camera/color/image_raw")
            ],
            parameters=[{
                "video_device": "/dev/video2",   # <- kendi kameranı buraya yaz
                "image_width": 1280,
                "image_height": 720,
                "framerate": 30.0,
                "pixel_format": "yuyv"
            }]
        ),
        
        # Lane Follower Node
        Node(
            package='sdt_perception',
            executable='lane_follower',
            name='lane_follower',
            output='screen',
            parameters=[{
                'max_linear_speed': 1.0,
                'max_angular_speed': 1.0,
                'min_angular_speed': -1.0,
                'kp': 0.5,
                'ki': 0.001,
                'kd': 0.0005,
                'roi_height': 0.6,
                'roi_width': 0.8
            }]
        )
    ])
