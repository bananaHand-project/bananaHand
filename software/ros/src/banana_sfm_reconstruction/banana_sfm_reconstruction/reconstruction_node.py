"""Placeholder ROS 2 node for SfM and 3D reconstruction."""

from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String

import numpy as np
from cv_bridge import CvBridge
import pyrealsense2 as rs
from pathlib import Path
import cv2


class ReconstructionNode(Node):
    """Scaffold node for future COLMAP-based reconstruction."""

    def __init__(self) -> None:
        super().__init__("reconstruction_node")

        self.declare_parameter("input_image_topic", "/camera/image_raw")
        self.declare_parameter("output_pointcloud_topic", "/reconstruction/point_cloud")
        self.declare_parameter("output_metadata_topic", "/reconstruction/metadata")

        pointcloud_topic = str(self.get_parameter("output_pointcloud_topic").value)
        metadata_topic = str(self.get_parameter("output_metadata_topic").value)
        self._image_topic = str(self.get_parameter("input_image_topic").value)

        self._pointcloud_pub = self.create_publisher(PointCloud2, pointcloud_topic, 10)
        self._metadata_pub = self.create_publisher(String, metadata_topic, 10)
        self._image_pub = self.create_publisher(Image, self._image_topic, 10)

        self.declare_parameter("realsense_width", 640)
        self.declare_parameter("realsense_height", 480)
        self.declare_parameter("realsense_fps", 30)

        width = int(self.get_parameter("realsense_width").value)
        height = int(self.get_parameter("realsense_height").value)
        fps = int(self.get_parameter("realsense_fps").value)

        self._save_path = Path(__file__).parent / "frames"
        self._save_path.mkdir(exist_ok=True)

        self._bridge = CvBridge()
        self._pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self._pipeline.start(cfg)

        self._frame_idx = 0
        period = 1.0 / fps
        self._timer = self.create_timer(period, self._capture_cb)

        self.get_logger().info("Realsense pipeline started, publishing images on "
                                f"'{self._image_topic}', saving to '{self._save_path.resolve()}'")

    def _capture_cb(self) -> None:
        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=5000)
        except RuntimeError:
            # timeout; no frame, just skip
            return
        colour = frames.get_color_frame()
        if not colour:
            return

        img = np.asanyarray(colour.get_data())
        msg = self._bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        self._image_pub.publish(msg)

        if self._save_path:
            # write a copy to disk for later colmap processing
            path = Path(self._save_path) / f"frame_{self._frame_idx:05d}.png"
            cv2.imwrite(str(path), img)
            self._frame_idx += 1

    def destroy_node(self) -> None:
        try:
            self._pipeline.stop()
        except Exception as e:
            self.get_logger().error(f"Error stopping Realsense pipeline: {e}")
        return super().destroy_node()


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
