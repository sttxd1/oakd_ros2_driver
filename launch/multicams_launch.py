from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch first camera (oakd_w)
        Node(
            package='oakd_ros2_driver',
            executable='multicams_publisher',
            name='oakd_w_publisher',
            parameters=[{
                'camera_name': 'oakd_w',
                'enable_rgb': True,
                'enable_stereo': True,
                'enable_depth': True,
                'enable_imu': True
            }]
        ),
        # Launch second camera (oakd_pro_w)
        Node(
            package='oakd_ros2_driver',
            executable='multicams_publisher',
            name='oakd_pro_w_publisher',
            parameters=[{
                'camera_name': 'oakd_pro_w',
                'enable_rgb': True,
                'enable_stereo': True,
                'enable_depth': True,
                'enable_imu': True
            }]
        )
    ])
