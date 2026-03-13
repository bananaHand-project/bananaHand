from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    realsense_serial = LaunchConfiguration("realsense_serial")

    return LaunchDescription(
        [
            DeclareLaunchArgument("realsense_serial", default_value=""),
            Node(
                package="banana_hand_tracking",
                executable="realsense_node",
                name="realsense_node",
                output="screen",
                parameters=[
                    {"device_serial": realsense_serial},
                    {"output_topic": "/camera/realsense/image_raw"},
                    {"frame_id": "realsense_camera"},
                    {"width": 640},
                    {"height": 480},
                    {"fps": 30},
                    {"show_preview": False},
                ],
            )
        ]
    )
