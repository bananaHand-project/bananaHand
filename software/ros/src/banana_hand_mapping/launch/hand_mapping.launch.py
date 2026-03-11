from launch import LaunchDescription
from launch_ros.actions import Node

# Matches firmware/experiments/uart_g4_combo/src/control_config.rs
# Position/command index order:
# 0 index-1, 1 middle, 2 ring, 3 pinky, 4 thumb-1, 5 thumb-2, 6 index-2, 7 thumb-3.
POS_INDEX_1 = 0
POS_MIDDLE = 1
POS_RING = 2
POS_PINKY = 3
POS_THUMB_1 = 4
POS_THUMB_2 = 5
POS_INDEX_2 = 6 # (USELESS)
POS_THUMB_3 = 7 # (USELESS)

DEFAULT_MIN_MOTOR_POSITIONS = [0] * 8
DEFAULT_MAX_MOTOR_POSITIONS = [4095] * 8

SPECIFIED_MIN_MOTOR_POSITIONS = [
    400,
    400,
    400,
    400,
    300,
    150,
    0, # USELESS
    0, # USELESS
]

SPECIFIED_MAX_MOTOR_POSITIONS = [
    3900,
    3900,
    3900,
    3900,
    3900,
    3900,
    4095, # USELESS
    4095, # USELESS
]


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
                    {
                        "min_motor_positions": [
                            SPECIFIED_MIN_MOTOR_POSITIONS[POS_INDEX_1],
                            SPECIFIED_MIN_MOTOR_POSITIONS[POS_MIDDLE],
                            SPECIFIED_MIN_MOTOR_POSITIONS[POS_RING],
                            SPECIFIED_MIN_MOTOR_POSITIONS[POS_PINKY],
                            SPECIFIED_MIN_MOTOR_POSITIONS[POS_THUMB_1],
                            SPECIFIED_MIN_MOTOR_POSITIONS[POS_THUMB_2],
                            SPECIFIED_MIN_MOTOR_POSITIONS[POS_INDEX_2],
                            SPECIFIED_MIN_MOTOR_POSITIONS[POS_THUMB_3],
                        ]
                    },
                    {
                        "max_motor_positions": [
                            SPECIFIED_MAX_MOTOR_POSITIONS[POS_INDEX_1],
                            SPECIFIED_MAX_MOTOR_POSITIONS[POS_MIDDLE],
                            SPECIFIED_MAX_MOTOR_POSITIONS[POS_RING],
                            SPECIFIED_MAX_MOTOR_POSITIONS[POS_PINKY],
                            SPECIFIED_MAX_MOTOR_POSITIONS[POS_THUMB_1],
                            SPECIFIED_MAX_MOTOR_POSITIONS[POS_THUMB_2],
                            SPECIFIED_MAX_MOTOR_POSITIONS[POS_INDEX_2],
                            SPECIFIED_MAX_MOTOR_POSITIONS[POS_THUMB_3],
                        ]
                    },
                    {"source_indices": [0, 1, 2, 3, 4, 5, -1, -1]},
                    {"fill_ratio": 0.0},
                ],
            ),
        ]
    )
