from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    show_preview = LaunchConfiguration("show_preview")

    return LaunchDescription(
        [
            DeclareLaunchArgument("show_preview", default_value="true"),
            Node(
                package="banana_hand_tracking",
                executable="webcam_node",
                name="webcam_node",
                output="screen",
                parameters=[
                    {"camera_index": 0},
                    {"frame_id": "camera"},
                    {"publish_hz": 30.0},
                    {"show_preview": False},
                    {"use_v4l2": True},
                ],
            ),
            Node(
                package="banana_hand_tracking",
                executable="hand_tracking_node",
                name="hand_tracking_node",
                output="screen",
                parameters=[
                    {"input_topic": "/camera/image_raw"},
                    {"frame_id": "camera"},
                    {"show_preview": show_preview},
                ],
            ),
        ]
    )
