from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic_name = LaunchConfiguration("topic_name")
    alpha = LaunchConfiguration("alpha")
    global_min = LaunchConfiguration("global_min")
    global_max = LaunchConfiguration("global_max")
    refresh_hz = LaunchConfiguration("refresh_hz")
    simulate_if_no_data = LaunchConfiguration("simulate_if_no_data")

    return LaunchDescription([
        DeclareLaunchArgument("topic_name", default_value="rx_force"),
        DeclareLaunchArgument("alpha", default_value="0.25"),
        DeclareLaunchArgument("global_min", default_value="0.0"),
        DeclareLaunchArgument("global_max", default_value="4095.0"),
        DeclareLaunchArgument("refresh_hz", default_value="30.0"),
        DeclareLaunchArgument("simulate_if_no_data", default_value="false"),
        Node(
            package="banana_hand_fsr_visualizer",
            executable="fsr_visualizer",
            name="banana_fsr_visualizer",
            output="screen",
            parameters=[{
                "topic_name": topic_name,
                "alpha": alpha,
                "global_min": global_min,
                "global_max": global_max,
                "refresh_hz": refresh_hz,
                "simulate_if_no_data": simulate_if_no_data,
            }],
        ),
    ])
