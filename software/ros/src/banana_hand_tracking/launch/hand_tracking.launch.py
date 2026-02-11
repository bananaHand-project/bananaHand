from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="banana_hand_tracking",
                executable="hand_tracking_node",
                name="hand_tracking_node",
                output="screen",
                parameters=[
                    {"input_topic": "/camera/image_raw"},
                    {"frame_id": "camera"},
                    {"show_preview": True},
                ],
            )
        ]
    )
