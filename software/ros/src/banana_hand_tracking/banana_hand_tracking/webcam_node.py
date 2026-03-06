"""ROS 2 node that publishes webcam frames as sensor_msgs/Image."""

from typing import Optional
import threading
import time

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

        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._latest_seq = 0
        self._last_published_seq = -1

        self._capture_stop = threading.Event()
        self._capture_thread = None

        self._open_camera()
        self._start_capture_thread()

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

    def _start_capture_thread(self) -> None:
        if self._capture_thread is not None and self._capture_thread.is_alive():
            return
        self._capture_stop.clear()
        self._capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True
        )
        self._capture_thread.start()

    def _capture_loop(self) -> None:
        while not self._capture_stop.is_set() and rclpy.ok():
            if not self._cap or not self._cap.isOpened():
                self._open_camera()
                time.sleep(0.1)
                continue

            ok, frame = self._cap.read()
            if not ok:
                time.sleep(0.01)
                continue

            with self._frame_lock:
                self._latest_frame = frame
                self._latest_seq += 1

    def _tick(self) -> None:
        frame = None
        seq = -1

        with self._frame_lock:
            if self._latest_frame is not None:
                frame = self._latest_frame.copy()
                seq = self._latest_seq

        if frame is None:
            return

        # Skip republishing the same frame if no new frame has arrived.
        if seq == self._last_published_seq:
            return
        self._last_published_seq = seq

        msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._publisher.publish(msg)

        if self._show_preview:
            cv2.imshow("webcam_preview", frame)
            cv2.waitKey(1)

    def destroy_node(self) -> bool:
        self._capture_stop.set()

        if self._capture_thread is not None:
            self._capture_thread.join(timeout=1.0)

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
