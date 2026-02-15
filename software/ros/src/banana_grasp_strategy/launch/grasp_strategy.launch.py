from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="banana_grasp_strategy",
                executable="grasp_strategy_node",
                name="grasp_strategy_node",
                output="screen",
                parameters=[
                    {"input_object_topic": "/object/point_cloud"},
                    {"output_grip_type_topic": "/grasp/grip_type"},
                    {"output_confidence_topic": "/grasp/grip_confidence"},
                ],
            )
        ]
    )
