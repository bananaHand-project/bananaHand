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
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    period_s = LaunchConfiguration("period_s")
    max_value = LaunchConfiguration("max_value")

    return LaunchDescription([
        DeclareLaunchArgument("topic_name", default_value="/rx_force"),
        DeclareLaunchArgument("alpha", default_value="0.25"),
        DeclareLaunchArgument("global_min", default_value="0.0"),
        DeclareLaunchArgument("global_max", default_value="4095.0"),
        DeclareLaunchArgument("refresh_hz", default_value="30.0"),
        DeclareLaunchArgument("publish_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("period_s", default_value="3.0"),
        DeclareLaunchArgument("max_value", default_value="1200.0"),
        Node(
            package="banana_hand_visualization",
            executable="fsr_visualizer",
            name="banana_fsr_visualizer",
            output="screen",
            parameters=[{
                "topic_name": topic_name,
                "alpha": alpha,
                "global_min": global_min,
                "global_max": global_max,
                "refresh_hz": refresh_hz,
            }],
        ),
        Node(
            package="banana_hand_visualization",
            executable="fsr_visualizer_test_publisher",
            name="banana_fsr_visualizer_test_publisher",
            output="screen",
            parameters=[{
                "output_topic": topic_name,
                "publish_rate_hz": publish_rate_hz,
                "period_s": period_s,
                "max_value": max_value,
            }],
        ),
    ])
