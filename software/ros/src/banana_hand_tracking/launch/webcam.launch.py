from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    camera_index = LaunchConfiguration("camera_index")
    frame_id = LaunchConfiguration("frame_id")
    publish_hz = LaunchConfiguration("publish_hz")
    capture_fps = LaunchConfiguration("capture_fps")
    capture_width = LaunchConfiguration("capture_width")
    capture_height = LaunchConfiguration("capture_height")
    pixel_format = LaunchConfiguration("pixel_format")
    repeat_last_frame = LaunchConfiguration("repeat_last_frame")
    publish_tick_topic = LaunchConfiguration("publish_tick_topic")
    show_preview = LaunchConfiguration("show_preview")
    use_v4l2 = LaunchConfiguration("use_v4l2")
    log_rate_interval = LaunchConfiguration("log_rate_interval")

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_index", default_value="0"),
            DeclareLaunchArgument("frame_id", default_value="camera"),
            DeclareLaunchArgument("publish_hz", default_value="30.0"),
            DeclareLaunchArgument("capture_fps", default_value="30.0"),
            DeclareLaunchArgument("capture_width", default_value="320"),
            DeclareLaunchArgument("capture_height", default_value="240"),
            DeclareLaunchArgument("pixel_format", default_value="MJPG"),
            DeclareLaunchArgument("repeat_last_frame", default_value="true"),
            DeclareLaunchArgument("publish_tick_topic", default_value="true"),
            DeclareLaunchArgument("show_preview", default_value="false"),
            DeclareLaunchArgument("use_v4l2", default_value="true"),
            DeclareLaunchArgument("log_rate_interval", default_value="5.0"),
            Node(
                package="banana_hand_tracking",
                executable="webcam_node",
                name="webcam_node",
                output="screen",
                parameters=[
                    {"camera_index": camera_index},
                    {"frame_id": frame_id},
                    {"publish_hz": publish_hz},
                    {"capture_fps": capture_fps},
                    {"capture_width": capture_width},
                    {"capture_height": capture_height},
                    {"pixel_format": pixel_format},
                    {"repeat_last_frame": repeat_last_frame},
                    {"publish_tick_topic": publish_tick_topic},
                    {"show_preview": show_preview},
                    {"use_v4l2": use_v4l2},
                    {"log_rate_interval": log_rate_interval},
                ],
            )
        ]
    )
