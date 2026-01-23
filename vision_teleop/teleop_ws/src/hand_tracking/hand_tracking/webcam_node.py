"""ROS 2 node that publishes webcam frames as sensor_msgs/Image."""

from typing import Optional

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class WebcamNode(Node):
    """Capture frames from a webcam and publish them as ROS 2 images."""

    def __init__(self) -> None:
        super().__init__("webcam_node")
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("show_preview", True)
        self.declare_parameter("use_v4l2", True)

        camera_index = int(self.get_parameter("camera_index").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        publish_hz = float(self.get_parameter("publish_hz").value)
        self._show_preview = bool(self.get_parameter("show_preview").value)
        self._use_v4l2 = bool(self.get_parameter("use_v4l2").value)

        self._bridge = CvBridge()
        self._publisher = self.create_publisher(Image, "/camera/image_raw", 10)
        self._timer = self.create_timer(1.0 / max(publish_hz, 1.0), self._tick)
        self._cap = None
        self._camera_index = camera_index
        self._open_camera()
        if self._show_preview:
            cv2.namedWindow("webcam_preview", cv2.WINDOW_NORMAL)

    def _open_camera(self) -> None:
        if self._cap and self._cap.isOpened():
            return
        if self._use_v4l2:
            self._cap = cv2.VideoCapture(self._camera_index, cv2.CAP_V4L2)
        else:
            self._cap = cv2.VideoCapture(self._camera_index)
        if not self._cap.isOpened():
            self.get_logger().error(
                f"Unable to open webcam index {self._camera_index}"
            )

    def _tick(self) -> None:
        if not self._cap or not self._cap.isOpened():
            self._open_camera()
            return
        ok, frame = self._cap.read()
        if not ok:
            return
        msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._publisher.publish(msg)
        if self._show_preview:
            cv2.imshow("webcam_preview", frame)
            cv2.waitKey(1)

    def destroy_node(self) -> bool:
        if self._cap and self._cap.isOpened():
            self._cap.release()
        if self._show_preview:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = WebcamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
