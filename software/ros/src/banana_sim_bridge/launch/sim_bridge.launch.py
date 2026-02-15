from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="banana_sim_bridge",
                executable="sim_bridge_node",
                name="sim_bridge_node",
                output="screen",
                parameters=[
                    {"requested_grip_topic": "/sim/requested_grip_type"},
                    {"success_metric_topic": "/sim/success_rate"},
                    {"artifact_path_topic": "/sim/artifact_path"},
                ],
            )
        ]
    )
