import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    include_vision = LaunchConfiguration("include_vision")
    include_mapping = LaunchConfiguration("include_mapping")
    show_preview = LaunchConfiguration("show_preview")

    hand_tracking_launch = os.path.join(
        get_package_share_directory("banana_hand_tracking"),
        "launch",
        "vision_teleop.launch.py",
    )
    hand_mapping_launch = os.path.join(
        get_package_share_directory("banana_hand_mapping"),
        "launch",
        "hand_mapping.launch.py",
    )

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baud", default_value="115200"),
        DeclareLaunchArgument("include_vision", default_value="true"),
        DeclareLaunchArgument("include_mapping", default_value="true"),
        DeclareLaunchArgument("show_preview", default_value="true"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hand_tracking_launch),
            condition=IfCondition(include_vision),
            launch_arguments={"show_preview": show_preview}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hand_mapping_launch),
            condition=IfCondition(include_mapping),
            launch_arguments={}.items(),
        ),

        Node(
            package="banana_serial_bridge",
            executable="serial_bridge",
            name="serial_bridge",
            output="screen",
            parameters=[{
                "port": port,
                "baud": baud,
                "publish_rate_hz": 100.0,
                "joint_names": [f"joint_{i}" for i in range(8)],
            }],
        ),
    ])
