from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    show_preview = LaunchConfiguration("show_preview")
    dual_cam = LaunchConfiguration("dual_cam")
    realsense_serial = LaunchConfiguration("realsense_serial")

    return LaunchDescription(
        [
            DeclareLaunchArgument("show_preview", default_value="true"),
            DeclareLaunchArgument("dual_cam", default_value="false"),
            DeclareLaunchArgument("realsense_serial", default_value=""),
            Node(
                package="banana_hand_tracking",
                executable="webcam_node",
                name="webcam_node",
                output="screen",
                condition=UnlessCondition(dual_cam),
                parameters=[
                    {"camera_index": 0},
                    {"frame_id": "camera"},
                    {"output_topic": "/camera/image_raw"},
                    {"publish_hz": 30.0},
                    {"show_preview": False},
                    {"use_v4l2": True},
                ],
            ),
            Node(
                package="banana_hand_tracking",
                executable="webcam_node",
                name="webcam_node",
                output="screen",
                condition=IfCondition(dual_cam),
                parameters=[
                    {"camera_index": 0},
                    {"frame_id": "camera"},
                    {"output_topic": "/camera/webcam/image_raw"},
                    {"publish_hz": 30.0},
                    {"show_preview": False},
                    {"use_v4l2": True},
                ],
            ),
            Node(
                package="banana_hand_tracking",
                executable="realsense_node",
                name="realsense_node",
                output="screen",
                condition=IfCondition(dual_cam),
                parameters=[
                    {"device_serial": realsense_serial},
                    {"output_topic": "/camera/realsense/image_raw"},
                    {"frame_id": "realsense_camera"},
                    {"width": 640},
                    {"height": 480},
                    {"fps": 30},
                    {"show_preview": False},
                ],
            ),
            Node(
                package="banana_hand_tracking",
                executable="hand_tracking_node",
                name="hand_tracking_node",
                output="screen",
                condition=UnlessCondition(dual_cam),
                parameters=[
                    {"input_topic": "/camera/image_raw"},
                    {"frame_id": "camera"},
                    {"show_preview": show_preview},
                ],
            ),
            Node(
                package="banana_hand_tracking",
                executable="hand_tracking_node",
                name="hand_tracking_node",
                output="screen",
                condition=IfCondition(dual_cam),
                parameters=[
                    {
                        "input_topics": [
                            "/camera/webcam/image_raw",
                            "/camera/realsense/image_raw",
                        ]
                    },
                    {"camera_names": ["webcam", "realsense"]},
                    {"frame_ids": ["camera", "realsense_camera"]},
                    {
                        "landmark_topics": [
                            "/hand/landmarks/webcam",
                            "/hand/landmarks/realsense",
                        ]
                    },
                    {"show_preview": show_preview},
                ],
            ),
        ]
    )
