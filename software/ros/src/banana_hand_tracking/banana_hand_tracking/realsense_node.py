"""ROS 2 node that publishes RealSense color frames as sensor_msgs/Image."""

from typing import Optional

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    import pyrealsense2 as rs
except ImportError as exc:
    raise SystemExit(
        "pyrealsense2 is not installed. Try: pip install pyrealsense2"
    ) from exc


class RealSenseNode(Node):
    """Capture color frames from an Intel RealSense camera and publish them."""

    def __init__(self) -> None:
        super().__init__("realsense_node")
        self.declare_parameter("device_serial", "")
        self.declare_parameter("output_topic", "/camera/realsense/image_raw")
        self.declare_parameter("frame_id", "realsense_camera")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("show_preview", False)
        self.declare_parameter("preview_window_name", "realsense_preview")

        self._device_serial = str(self.get_parameter("device_serial").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._width = int(self.get_parameter("width").value)
        self._height = int(self.get_parameter("height").value)
        self._fps = int(self.get_parameter("fps").value)
        self._show_preview = bool(self.get_parameter("show_preview").value)
        self._preview_window_name = str(
            self.get_parameter("preview_window_name").value
        )

        self._bridge = CvBridge()
        self._publisher = self.create_publisher(Image, output_topic, 10)
        self._pipeline = None
        self._profile = None

        self._open_camera()
        self._timer = self.create_timer(1.0 / max(float(self._fps), 1.0), self._tick)

        if self._show_preview:
            cv2.namedWindow(self._preview_window_name, cv2.WINDOW_NORMAL)

    def _open_camera(self) -> None:
        if self._pipeline is not None:
            return

        pipeline = rs.pipeline()
        config = rs.config()
        if self._device_serial:
            config.enable_device(self._device_serial)
        config.enable_stream(
            rs.stream.color,
            self._width,
            self._height,
            rs.format.bgr8,
            self._fps,
        )

        try:
            profile = pipeline.start(config)
        except Exception as exc:
            self.get_logger().error(f"Unable to start RealSense camera: {exc}")
            return

        self._pipeline = pipeline
        self._profile = profile
        device = profile.get_device()
        device_name = "RealSense"
        serial_number = self._device_serial
        try:
            device_name = device.get_info(rs.camera_info.name)
            serial_number = device.get_info(rs.camera_info.serial_number)
        except Exception:
            pass
        self.get_logger().info(
            f"Streaming {device_name} ({serial_number}) at "
            f"{self._width}x{self._height}@{self._fps}Hz"
        )

    def _close_camera(self) -> None:
        if self._pipeline is None:
            return
        try:
            self._pipeline.stop()
        except RuntimeError:
            pass
        self._pipeline = None
        self._profile = None

    def _tick(self) -> None:
        if self._pipeline is None:
            self._open_camera()
            return

        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=100)
        except RuntimeError as exc:
            self.get_logger().warn(f"RealSense frame wait failed: {exc}")
            self._close_camera()
            return

        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        frame = np.asanyarray(color_frame.get_data())
        msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._publisher.publish(msg)

        if self._show_preview:
            cv2.imshow(self._preview_window_name, frame)
            cv2.waitKey(1)

    def destroy_node(self) -> bool:
        self._close_camera()
        if self._show_preview:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RealSenseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
