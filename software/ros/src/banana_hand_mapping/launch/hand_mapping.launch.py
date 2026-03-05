from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="banana_hand_mapping",
                executable="teleop_to_tx_positions",
                name="teleop_to_tx_positions",
                output="screen",
                parameters=[
                    {"input_topic": "/hand/teleop_joint_trajectory"},
                    {"output_topic": "/tx_positions"},
                    {"input_min_ratio": 0.0},
                    {"input_max_ratio": 1.0},
                    {"adc_min": 0},
                    {"adc_max": 4095},
                    {"source_indices": [0, 1, 2, 3, 4, 5, -1, -1]},
                    {"fill_ratio": 0.0},
                ],
            ),
        ]
    )
