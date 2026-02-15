from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="banana_sfm_reconstruction",
                executable="reconstruction_node",
                name="reconstruction_node",
                output="screen",
                parameters=[
                    {"input_image_topic": "/camera/image_raw"},
                    {"output_pointcloud_topic": "/reconstruction/point_cloud"},
                    {"output_metadata_topic": "/reconstruction/metadata"},
                ],
            )
        ]
    )
