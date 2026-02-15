"""Placeholder ROS 2 node for SfM and 3D reconstruction."""

from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


class ReconstructionNode(Node):
    """Scaffold node for future COLMAP-based reconstruction."""

    def __init__(self) -> None:
        super().__init__("reconstruction_node")

        self.declare_parameter("input_image_topic", "/camera/image_raw")
        self.declare_parameter("output_pointcloud_topic", "/reconstruction/point_cloud")
        self.declare_parameter("output_metadata_topic", "/reconstruction/metadata")

        pointcloud_topic = str(self.get_parameter("output_pointcloud_topic").value)
        metadata_topic = str(self.get_parameter("output_metadata_topic").value)

        self._pointcloud_pub = self.create_publisher(PointCloud2, pointcloud_topic, 10)
        self._metadata_pub = self.create_publisher(String, metadata_topic, 10)

        self.get_logger().info(
            "TODO: Implement COLMAP-based SfM reconstruction and publish outputs."
        )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ReconstructionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
