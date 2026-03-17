from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic_name = LaunchConfiguration("topic_name")
    position_topic_name = LaunchConfiguration("position_topic_name")
    position_message_type = LaunchConfiguration("position_message_type")
    alpha = LaunchConfiguration("alpha")
    position_alpha = LaunchConfiguration("position_alpha")
    global_min = LaunchConfiguration("global_min")
    global_max = LaunchConfiguration("global_max")
    position_min = LaunchConfiguration("position_min")
    position_max = LaunchConfiguration("position_max")
    plot_window_seconds = LaunchConfiguration("plot_window_seconds")
    refresh_hz = LaunchConfiguration("refresh_hz")
    contact_threshold = LaunchConfiguration("contact_threshold")
    simulate_if_no_data = LaunchConfiguration("simulate_if_no_data")

    return LaunchDescription([
        DeclareLaunchArgument("topic_name", default_value="rx_force"),
        DeclareLaunchArgument("position_topic_name", default_value="rx_positions"),
        DeclareLaunchArgument("position_message_type", default_value="joint_state"),
        DeclareLaunchArgument("alpha", default_value="0.25"),
        DeclareLaunchArgument("position_alpha", default_value="0.22"),
        DeclareLaunchArgument("global_min", default_value="0.0"),
        DeclareLaunchArgument("global_max", default_value="4095.0"),
        DeclareLaunchArgument("position_min", default_value="0.0"),
        DeclareLaunchArgument("position_max", default_value="4095.0"),
        DeclareLaunchArgument("plot_window_seconds", default_value="8.0"),
        DeclareLaunchArgument("refresh_hz", default_value="30.0"),
        DeclareLaunchArgument("contact_threshold", default_value="0.7"),
        DeclareLaunchArgument("simulate_if_no_data", default_value="false"),
        Node(
            package="banana_hand_fsr_visualizer",
            executable="fsr_visualizer",
            name="banana_fsr_visualizer",
            output="screen",
            parameters=[{
                "topic_name": topic_name,
                "position_topic_name": position_topic_name,
                "position_message_type": position_message_type,
                "alpha": alpha,
                "position_alpha": position_alpha,
                "global_min": global_min,
                "global_max": global_max,
                "position_min": position_min,
                "position_max": position_max,
                "plot_window_seconds": plot_window_seconds,
                "refresh_hz": refresh_hz,
                "contact_threshold": contact_threshold,
                "simulate_if_no_data": simulate_if_no_data,
            }],
        ),
    ])
