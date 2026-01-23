from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="hand_tracking",
                executable="webcam_node",
                name="webcam_node",
                output="screen",
                parameters=[
                    {"camera_index": 0},
                    {"frame_id": "camera"},
                    {"publish_hz": 30.0},
                ],
            )
        ]
    )
