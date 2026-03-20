from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    show_mujoco_viewer = LaunchConfiguration("show_mujoco_viewer")
    state_source = LaunchConfiguration("state_source")
    launch_foxglove_bridge = LaunchConfiguration("launch_foxglove_bridge")

    actions = [
        DeclareLaunchArgument("show_mujoco_viewer", default_value="false"),
        DeclareLaunchArgument("state_source", default_value="rx_positions"),
        DeclareLaunchArgument("launch_foxglove_bridge", default_value="true"),
        Node(
            package="banana_hand_visualization",
            executable="mujoco_visualizer",
            name="banana_hand_visualization",
            output="screen",
            parameters=[{"show_mujoco_viewer": show_mujoco_viewer, "state_source": state_source}],
        ),
        Node(
            package="banana_hand_visualization",
            executable="test_force_publisher",
            name="banana_hand_test_force_publisher",
            output="screen",
        ),
    ]

    try:
        foxglove_share = get_package_share_directory("foxglove_bridge")
        foxglove_launch = f"{foxglove_share}/launch/foxglove_bridge_launch.xml"
        actions.append(
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(foxglove_launch),
                condition=IfCondition(launch_foxglove_bridge),
            )
        )
    except PackageNotFoundError:
        actions.append(
            LogInfo(
                condition=IfCondition(launch_foxglove_bridge),
                msg=(
                    "foxglove_bridge is not installed. Install "
                    "ros-$ROS_DISTRO-foxglove-bridge to stream to Foxglove."
                ),
            )
        )

    return LaunchDescription(actions)
