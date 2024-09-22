from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oakd_ros2_driver',
            executable='oakd_ros2_publisher',
            name='oakd_ros2_publisher',
            parameters=[
                {"enable_rgb": True},        
                {"enable_stereo": True},
                {"enable_depth": True},
                {"enable_imu": True},
            ]
        )
    ])
