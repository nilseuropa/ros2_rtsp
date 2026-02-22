from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ros2_rtsp",
            executable="rtsp_multi",
            name="rtsp_multi",
            parameters=[{
                "ip_addresses": [],
                "user_name": "",
                "user_password": "",
                "rtsp_path": "stream2",
            }],
            output="screen",
        )
    ])
