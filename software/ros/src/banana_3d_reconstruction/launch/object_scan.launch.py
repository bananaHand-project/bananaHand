import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("banana_3d_reconstruction")
    default_params = os.path.join(package_share, "config", "object_scan_params.yaml")

    params_file = LaunchConfiguration("params_file")
    show_preview = LaunchConfiguration("show_preview")
    enable_roi_selection = LaunchConfiguration("enable_roi_selection")
    device_serial = LaunchConfiguration("device_serial")
    output_dir = LaunchConfiguration("output_dir")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("show_preview", default_value="true"),
            DeclareLaunchArgument("enable_roi_selection", default_value="true"),
            DeclareLaunchArgument("device_serial", default_value=""),
            DeclareLaunchArgument("output_dir", default_value="~/banana_scans"),
            Node(
                package="banana_3d_reconstruction",
                executable="object_scan_node",
                name="object_scan_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "show_preview": show_preview,
                        "enable_roi_selection": enable_roi_selection,
                        "device_serial": device_serial,
                        "output_dir": output_dir,
                    },
                ],
            ),
        ]
    )
