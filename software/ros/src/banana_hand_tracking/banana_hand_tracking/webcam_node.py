"""ROS 2 node that publishes webcam frames as sensor_msgs/Image."""

from typing import Optional
import threading
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import UInt32


def _as_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {
            "1",
            "true",
            "t",
            "yes",
            "y",
            "on",
        }
    return bool(value)


class WebcamNode(Node):
    """Capture frames from a webcam and publish them as ROS 2 images."""

    def __init__(self) -> None:
        super().__init__("webcam_node")
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("capture_fps", 30.0)
        self.declare_parameter("capture_width", 640)
        self.declare_parameter("capture_height", 480)
        self.declare_parameter("pixel_format", "MJPG")
        self.declare_parameter("repeat_last_frame", True)
        self.declare_parameter("publish_tick_topic", True)
        self.declare_parameter("show_preview", True)
        self.declare_parameter("use_v4l2", True)
        self.declare_parameter("log_rate_interval", 5.0)

        camera_index = int(self.get_parameter("camera_index").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._publish_hz = float(self.get_parameter("publish_hz").value)
        self._capture_fps = float(self.get_parameter("capture_fps").value)
        self._capture_width = int(self.get_parameter("capture_width").value)
        self._capture_height = int(self.get_parameter("capture_height").value)
        self._pixel_format = str(self.get_parameter("pixel_format").value).upper()
        self._repeat_last_frame = _as_bool(
            self.get_parameter("repeat_last_frame").value
        )
        self._publish_tick_topic = _as_bool(
            self.get_parameter("publish_tick_topic").value
        )
        self._show_preview = _as_bool(self.get_parameter("show_preview").value)
        self._use_v4l2 = _as_bool(self.get_parameter("use_v4l2").value)
        self._log_rate_interval = max(
            float(self.get_parameter("log_rate_interval").value), 1.0
        )

        self._bridge = CvBridge()
        self._publisher = self.create_publisher(
            Image, "/camera/image_raw", qos_profile_sensor_data
        )
        self._tick_publisher = None
        if self._publish_tick_topic:
            self._tick_publisher = self.create_publisher(
                UInt32, "/camera/frame_tick", qos_profile_sensor_data
            )
        self._timer = self.create_timer(
            1.0 / max(self._publish_hz, 1.0), self._tick
        )

        self._cap = None
        self._camera_index = camera_index

        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._latest_seq = 0
        self._last_published_seq = -1

        self._capture_stop = threading.Event()
        self._capture_thread = None
        self._stats_lock = threading.Lock()
        self._stats_start = time.monotonic()
        self._capture_count = 0
        self._publish_count = 0
        self._stale_publish_count = 0

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
            return

        self._configure_camera()

    def _configure_camera(self) -> None:
        if not self._cap or not self._cap.isOpened():
            return

        if len(self._pixel_format) == 4:
            fourcc = cv2.VideoWriter_fourcc(*self._pixel_format)
            self._cap.set(cv2.CAP_PROP_FOURCC, float(fourcc))
        elif self._pixel_format and self._pixel_format != "AUTO":
            self.get_logger().warn(
                "pixel_format must be 4 chars (for example MJPG) or AUTO"
            )

        if self._capture_width > 0:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self._capture_width))
        if self._capture_height > 0:
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self._capture_height))
        if self._capture_fps > 0:
            self._cap.set(cv2.CAP_PROP_FPS, float(self._capture_fps))

        if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1.0)

        actual_width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = float(self._cap.get(cv2.CAP_PROP_FPS))
        actual_fourcc_int = int(self._cap.get(cv2.CAP_PROP_FOURCC))
        actual_fourcc_chars = [
            chr((actual_fourcc_int >> (8 * i)) & 0xFF) for i in range(4)
        ]
        if all(32 <= ord(char) <= 126 for char in actual_fourcc_chars):
            actual_fourcc = "".join(actual_fourcc_chars)
        else:
            actual_fourcc = f"0x{actual_fourcc_int:08X}"

        self.get_logger().info(
            "Camera configured: "
            f"requested={self._capture_width}x{self._capture_height}@"
            f"{self._capture_fps:.1f}Hz {self._pixel_format}, "
            f"actual={actual_width}x{actual_height}@{actual_fps:.2f}Hz "
            f"{actual_fourcc}"
        )
        if self._capture_fps > 0 and 0 < actual_fps < (0.8 * self._capture_fps):
            self.get_logger().warn(
                "Camera driver is reporting a lower FPS than requested; "
                "published rate will be limited by capture rate."
            )

    def _maybe_log_rates(self) -> None:
        now = time.monotonic()
        with self._stats_lock:
            elapsed = now - self._stats_start
            if elapsed < self._log_rate_interval:
                return
            capture_rate = self._capture_count / elapsed
            publish_rate = self._publish_count / elapsed
            stale_ratio = (
                self._stale_publish_count / self._publish_count
                if self._publish_count > 0
                else 0.0
            )
            self._stats_start = now
            self._capture_count = 0
            self._publish_count = 0
            self._stale_publish_count = 0

        self.get_logger().info(
            "Effective rates: "
            f"capture={capture_rate:.2f}Hz publish={publish_rate:.2f}Hz "
            f"(target publish_hz={self._publish_hz:.2f}, "
            f"stale_publish_ratio={stale_ratio:.2f})"
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

            with self._stats_lock:
                self._capture_count += 1
            self._maybe_log_rates()

    def _tick(self) -> None:
        frame = None
        seq = -1
        stale_publish = False

        with self._frame_lock:
            if self._latest_frame is not None:
                frame = self._latest_frame.copy()
                seq = self._latest_seq

        if frame is None:
            return

        if seq == self._last_published_seq:
            if not self._repeat_last_frame:
                return
            stale_publish = True
        else:
            self._last_published_seq = seq

        msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._publisher.publish(msg)
        if self._tick_publisher is not None:
            self._tick_publisher.publish(UInt32(data=int(seq)))

        with self._stats_lock:
            self._publish_count += 1
            if stale_publish:
                self._stale_publish_count += 1
        self._maybe_log_rates()

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
