"""Launch the ground-removal node first, then the rule-based grasp classifier."""

import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    input_dir = LaunchConfiguration("input_dir")
    recursive_search = LaunchConfiguration("recursive_search")
    ground_output_suffix = LaunchConfiguration("output_suffix")
    classifier_output_suffix = LaunchConfiguration("classifier_output_suffix")
    opening_margin_m = LaunchConfiguration("opening_margin_m")
    max_hand_opening_m = LaunchConfiguration("max_hand_opening_m")
    small_object_max_span_m = LaunchConfiguration("small_object_max_span_m")
    tripod_object_max_span_m = LaunchConfiguration("tripod_object_max_span_m")
    power_grasp_min_span_m = LaunchConfiguration("power_grasp_min_span_m")
    package_prefix = get_package_prefix("banana_grasp_classification")
    executable_dir = os.path.join(
        package_prefix,
        "lib",
        "banana_grasp_classification",
    )

    pipeline_runner = ExecuteProcess(
        cmd=[
            "/bin/bash",
            "-lc",
            [
                os.path.join(executable_dir, "ground_plane_removal_node"),
                " --ros-args",
                " -p input_dir:=",
                input_dir,
                " -p recursive_search:=",
                recursive_search,
                " -p output_suffix:=",
                ground_output_suffix,
                " && ",
                os.path.join(executable_dir, "grasp_rule_classifier_node"),
                " --ros-args",
                " -p input_dir:=",
                input_dir,
                " -p recursive_search:=",
                recursive_search,
                " -p ground_removed_suffix:=",
                ground_output_suffix,
                " -p output_suffix:=",
                classifier_output_suffix,
                " -p opening_margin_m:=",
                opening_margin_m,
                " -p max_hand_opening_m:=",
                max_hand_opening_m,
                " -p small_object_max_span_m:=",
                small_object_max_span_m,
                " -p tripod_object_max_span_m:=",
                tripod_object_max_span_m,
                " -p power_grasp_min_span_m:=",
                power_grasp_min_span_m,
            ],
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("input_dir", default_value=""),
            DeclareLaunchArgument("recursive_search", default_value="true"),
            DeclareLaunchArgument("output_suffix", default_value="_ground_removed"),
            DeclareLaunchArgument("classifier_output_suffix", default_value="_grasp"),
            DeclareLaunchArgument("opening_margin_m", default_value="0.03"),
            DeclareLaunchArgument("max_hand_opening_m", default_value="0.0"),
            DeclareLaunchArgument("small_object_max_span_m", default_value="0.045"),
            DeclareLaunchArgument("tripod_object_max_span_m", default_value="0.065"),
            DeclareLaunchArgument("power_grasp_min_span_m", default_value="0.045"),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=pipeline_runner,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(reason="grasp classification pipeline completed")
                        )
                    ],
                )
            ),
            pipeline_runner,
        ]
    )
