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
    include_visualization = LaunchConfiguration("include_visualization")
    show_preview = LaunchConfiguration("show_preview")
    show_mujoco_viewer = LaunchConfiguration("show_mujoco_viewer")
    state_source = LaunchConfiguration("state_source")
    show_fsr_debugger = LaunchConfiguration("show_fsr_debugger")
    launch_foxglove_bridge = LaunchConfiguration("launch_foxglove_bridge")

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
    mujoco_visualization_launch = os.path.join(
        get_package_share_directory("banana_hand_visualization"),
        "launch",
        "mujoco_visualization.launch.py",
    )

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baud", default_value="115200"),
        DeclareLaunchArgument("include_vision", default_value="true"),
        DeclareLaunchArgument("include_mapping", default_value="true"),
        DeclareLaunchArgument("include_visualization", default_value="true"),
        DeclareLaunchArgument("show_preview", default_value="true"),
        DeclareLaunchArgument("show_mujoco_viewer", default_value="false"),
        DeclareLaunchArgument("state_source", default_value="rx_positions"),
        DeclareLaunchArgument("show_fsr_debugger", default_value="false"),
        DeclareLaunchArgument("launch_foxglove_bridge", default_value="true"),

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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mujoco_visualization_launch),
            condition=IfCondition(include_visualization),
            launch_arguments={
                "show_mujoco_viewer": show_mujoco_viewer,
                "state_source": state_source,
                "launch_foxglove_bridge": launch_foxglove_bridge,
            }.items(),
        ),
        Node(
            package="banana_hand_visualization",
            executable="fsr_visualizer",
            name="banana_fsr_visualizer",
            output="screen",
            condition=IfCondition(show_fsr_debugger),
            parameters=[{
                "topic_name": "rx_force",
            }],
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
