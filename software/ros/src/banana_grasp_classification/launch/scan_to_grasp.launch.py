"""Launch the scan node and automatically run grasp processing after each scan."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    reconstruction_share = get_package_share_directory("banana_3d_reconstruction")
    default_params = os.path.join(
        reconstruction_share,
        "config",
        "object_scan_params.yaml",
    )
    params_file = LaunchConfiguration("params_file")
    show_preview = LaunchConfiguration("show_preview")
    enable_roi_selection = LaunchConfiguration("enable_roi_selection")
    device_serial = LaunchConfiguration("device_serial")
    output_dir = LaunchConfiguration("output_dir")
    result_topic = LaunchConfiguration("result_topic")
    ground_removed_suffix = LaunchConfiguration("ground_removed_suffix")
    classifier_output_suffix = LaunchConfiguration("classifier_output_suffix")
    opening_margin_m = LaunchConfiguration("opening_margin_m")
    max_hand_opening_m = LaunchConfiguration("max_hand_opening_m")
    small_object_max_span_m = LaunchConfiguration("small_object_max_span_m")
    tripod_object_max_span_m = LaunchConfiguration("tripod_object_max_span_m")
    power_grasp_min_span_m = LaunchConfiguration("power_grasp_min_span_m")

    object_scan_launch = os.path.join(
        reconstruction_share,
        "launch",
        "object_scan.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("show_preview", default_value="true"),
            DeclareLaunchArgument("enable_roi_selection", default_value="true"),
            DeclareLaunchArgument("device_serial", default_value=""),
            DeclareLaunchArgument("output_dir", default_value="~/banana_scans"),
            DeclareLaunchArgument(
                "result_topic",
                default_value="/grasp_classification/recommendation",
            ),
            DeclareLaunchArgument(
                "ground_removed_suffix",
                default_value="_ground_removed",
            ),
            DeclareLaunchArgument(
                "classifier_output_suffix",
                default_value="_grasp",
            ),
            DeclareLaunchArgument("opening_margin_m", default_value="0.03"),
            DeclareLaunchArgument("max_hand_opening_m", default_value="0.0"),
            DeclareLaunchArgument(
                "small_object_max_span_m",
                default_value="0.045",
            ),
            DeclareLaunchArgument(
                "tripod_object_max_span_m",
                default_value="0.065",
            ),
            DeclareLaunchArgument(
                "power_grasp_min_span_m",
                default_value="0.045",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(object_scan_launch),
                launch_arguments={
                    "params_file": params_file,
                    "show_preview": show_preview,
                    "enable_roi_selection": enable_roi_selection,
                    "device_serial": device_serial,
                    "output_dir": output_dir,
                }.items(),
            ),
            Node(
                package="banana_grasp_classification",
                executable="scan_grasp_pipeline_node",
                name="scan_grasp_pipeline_node",
                output="screen",
                parameters=[
                    {
                        "result_topic": result_topic,
                        "ground_removed_suffix": ground_removed_suffix,
                        "classifier_output_suffix": classifier_output_suffix,
                        "opening_margin_m": ParameterValue(
                            opening_margin_m,
                            value_type=float,
                        ),
                        "max_hand_opening_m": ParameterValue(
                            max_hand_opening_m,
                            value_type=float,
                        ),
                        "small_object_max_span_m": ParameterValue(
                            small_object_max_span_m,
                            value_type=float,
                        ),
                        "tripod_object_max_span_m": ParameterValue(
                            tripod_object_max_span_m,
                            value_type=float,
                        ),
                        "power_grasp_min_span_m": ParameterValue(
                            power_grasp_min_span_m,
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
