"""ROS 2 node that runs MediaPipe Hands on incoming images."""

from typing import Optional
import math
import statistics
import threading
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_srvs.srv import Trigger

# Change this alias if the hand landmark message type moves packages.
from banana_interfaces.msg import HandState as HandLandmarks

try:
    import mediapipe as mp
except ImportError as exc:
    raise SystemExit(
        "mediapipe is not installed. Try: pip install mediapipe"
    ) from exc


class HandTrackingNode(Node):
    """Subscribe to /camera/image_raw and run MediaPipe Hands."""

    def __init__(self) -> None:
        super().__init__("hand_tracking_node")
        self.declare_parameter("input_topic", "/camera/image_raw")
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("show_preview", True)
        self.declare_parameter("mirror_handedness", True)
        self.declare_parameter("alpha", 0.2)
        self.declare_parameter("process_width", 640)
        self.declare_parameter("process_height", 480)
        self.declare_parameter("preview_fps", 15.0)
        self.declare_parameter("process_fps", 0.0)

        input_topic = str(self.get_parameter("input_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._show_preview = bool(self.get_parameter("show_preview").value)
        self._mirror_handedness = bool(
            self.get_parameter("mirror_handedness").value
        )
        self._alpha = float(self.get_parameter("alpha").value)
        self._process_width = int(self.get_parameter("process_width").value)
        self._process_height = int(self.get_parameter("process_height").value)
        self._preview_fps = float(self.get_parameter("preview_fps").value)
        self._process_fps = float(self.get_parameter("process_fps").value)

        self._callback_group = ReentrantCallbackGroup()

        self._bridge = CvBridge()
        self._subscription = self.create_subscription(
            Image, input_topic, self._on_image, 10, callback_group=self._callback_group
        )
        self._publisher = self.create_publisher(HandLandmarks, "/hand/landmarks", 10)
        self._teleop_publisher = self.create_publisher(
            JointTrajectory, "/hand/teleop_joint_trajectory", 10
        )
        self._calibrate_srv = self.create_service(
            Trigger,
            "/hand/calibrate",
            self._on_calibrate,
            callback_group=self._callback_group,
        )

        self._joint_names = [
            "index_flex",
            "middle_flex",
            "ring_flex",
            "pinky_flex",
            "thumb_flex",
            "thumb_opp",
        ]
        self._teleop_prev = [0.0] * 6
        self._decay_step = 0.02
        self._last_warn_ns = 0

        self._mp_hands = mp.solutions.hands
        self._mp_draw = mp.solutions.drawing_utils
        self._hands = self._mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        if self._show_preview:
            cv2.namedWindow("hand_tracking", cv2.WINDOW_NORMAL)
        self._preview_lock = threading.Lock()
        self._preview_frame = None
        self._last_preview_time = 0.0
        self._last_process_time = 0.0

        self._latest_frame_lock = threading.Lock()
        self._latest_frame = None
        self._latest_frame_seq = 0

        self._overlay_lock = threading.Lock()
        self._overlay_points = None

        self._hand_connections = list(self._mp_hands.HAND_CONNECTIONS)
        self._process_stop = threading.Event()
        self._process_thread = threading.Thread(
            target=self._process_loop, daemon=True
        )
        self._process_thread.start()

        self._calibrated = False
        self._calibration_active = False
        self._calibration_stage = 0
        self._calibration_start_ns = 0
        self._calibration_done = threading.Event()
        self._calibration_result = (False, "Calibration not started")

        self._open_ref = {
            "index": None,
            "middle": None,
            "ring": None,
            "pinky": None,
            "thumb": None,
        }
        self._closed_ref = {
            "index": None,
            "middle": None,
            "ring": None,
            "pinky": None,
            "thumb": None,
        }
        self._thumb_unopp = None
        self._thumb_opp_min = None

        self._calib_samples = {}
        self._last_hand_detected = False

    def _dist2d(self, a: Point, b: Point) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    def _clip01(self, value: float) -> float:
        return max(0.0, min(1.0, value))

    def _compute_hand_metrics(self, landmarks: list[Point]) -> tuple[dict, float]:
        hand_scale = self._dist2d(landmarks[0], landmarks[9]) + 1e-6
        metrics = {
            "index": self._dist2d(landmarks[8], landmarks[5]) / hand_scale,
            "middle": self._dist2d(landmarks[12], landmarks[9]) / hand_scale,
            "ring": self._dist2d(landmarks[16], landmarks[13]) / hand_scale,
            "pinky": self._dist2d(landmarks[20], landmarks[17]) / hand_scale,
            "thumb": self._dist2d(landmarks[4], landmarks[2]) / hand_scale,
        }
        thumb_opp = self._dist2d(landmarks[4], landmarks[17]) / hand_scale
        return metrics, thumb_opp

    def _compute_curl(self, d_norm: float, d_open: float, d_closed: float) -> float:
        denom = d_open - d_closed
        if abs(denom) < 1e-6:
            return 0.0
        return self._clip01((d_open - d_norm) / denom)

    def _compute_thumb_opp(
        self, d_norm: float, d_unopp: float, d_opp_min: float
    ) -> float:
        denom = d_unopp - d_opp_min
        if abs(denom) < 1e-6:
            return 0.0
        return self._clip01((d_unopp - d_norm) / denom)

    def _start_calibration(self) -> None:
        self._calibration_active = True
        self._calibration_stage = 1
        self._calibration_start_ns = self.get_clock().now().nanoseconds
        self._calibration_done.clear()
        self._calib_samples = {
            "index": [],
            "middle": [],
            "ring": [],
            "pinky": [],
            "thumb": [],
            "opp": [],
        }
        self.get_logger().info("Calibration stage 1/2: Hold hand OPEN")

    def _finish_calibration(self, success: bool, message: str) -> None:
        self._calibration_active = False
        self._calibration_stage = 0
        self._calibration_result = (success, message)
        self._calibration_done.set()

    def _advance_calibration_stage(self, now_ns: int) -> None:
        if self._calibration_stage == 1:
            if not all(
                self._calib_samples[f]
                for f in ["index", "middle", "ring", "pinky", "thumb"]
            ) or not self._calib_samples["opp"]:
                self._finish_calibration(False, "Calibration failed: no hand detected")
                return
            self._open_ref["index"] = statistics.median(self._calib_samples["index"])
            self._open_ref["middle"] = statistics.median(self._calib_samples["middle"])
            self._open_ref["ring"] = statistics.median(self._calib_samples["ring"])
            self._open_ref["pinky"] = statistics.median(self._calib_samples["pinky"])
            self._open_ref["thumb"] = statistics.median(self._calib_samples["thumb"])
            self._thumb_unopp = statistics.median(self._calib_samples["opp"])

            self._calibration_stage = 2
            self._calibration_start_ns = now_ns
            self._calib_samples = {
                "index": [],
                "middle": [],
                "ring": [],
                "pinky": [],
                "thumb": [],
                "opp": [],
            }
            self.get_logger().info(
                "Calibration stage 2/2: Make a FIST and oppose thumb"
            )
        elif self._calibration_stage == 2:
            if not all(
                self._calib_samples[f]
                for f in ["index", "middle", "ring", "pinky", "thumb"]
            ) or not self._calib_samples["opp"]:
                self._finish_calibration(False, "Calibration failed: no hand detected")
                return
            self._closed_ref["index"] = statistics.median(
                self._calib_samples["index"]
            )
            self._closed_ref["middle"] = statistics.median(
                self._calib_samples["middle"]
            )
            self._closed_ref["ring"] = statistics.median(self._calib_samples["ring"])
            self._closed_ref["pinky"] = statistics.median(
                self._calib_samples["pinky"]
            )
            self._closed_ref["thumb"] = statistics.median(
                self._calib_samples["thumb"]
            )
            self._thumb_opp_min = statistics.median(self._calib_samples["opp"])

            self._calibrated = True
            self._finish_calibration(True, "Calibration complete")

    def _on_calibrate(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self._calibration_active:
            response.success = False
            response.message = "Calibration already in progress"
            return response
        if not self._last_hand_detected:
            response.success = False
            response.message = "No hand detected"
            return response

        self._start_calibration()
        self._calibration_done.wait()
        response.success, response.message = self._calibration_result
        return response

    def _publish_teleop(self, outputs: list[float]) -> None:
        msg = JointTrajectory()
        msg.joint_names = list(self._joint_names)
        point = JointTrajectoryPoint()
        point.positions = list(outputs)
        point.time_from_start = Duration(sec=0, nanosec=50_000_000)
        msg.points = [point]
        self._teleop_publisher.publish(msg)

    def _on_image(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        with self._latest_frame_lock:
            self._latest_frame = frame
            self._latest_frame_seq += 1
        if self._show_preview:
            with self._preview_lock:
                self._preview_frame = frame

    def _process_loop(self) -> None:
        last_seq = -1
        while not self._process_stop.is_set() and rclpy.ok():
            with self._latest_frame_lock:
                seq = self._latest_frame_seq
                frame = self._latest_frame
            if frame is None or seq == last_seq:
                time.sleep(0.001)
                continue
            if self._process_fps > 0:
                now = time.monotonic()
                if now - self._last_process_time < 1.0 / self._process_fps:
                    time.sleep(0.001)
                    continue
                self._last_process_time = now
            last_seq = seq

            if self._process_width > 0 and self._process_height > 0:
                frame_proc = cv2.resize(
                    frame, (self._process_width, self._process_height)
                )
            else:
                frame_proc = frame
            rgb = cv2.cvtColor(frame_proc, cv2.COLOR_BGR2RGB)
            results = self._hands.process(rgb)

            has_hand = bool(results.multi_hand_landmarks)
            self._last_hand_detected = has_hand

            landmarks_points: list[Point] = []
            if has_hand:
                landmarks = results.multi_hand_landmarks[0]
                landmarks_msg = HandLandmarks()
                landmarks_msg.header.stamp = self.get_clock().now().to_msg()
                landmarks_msg.header.frame_id = self._frame_id
                landmarks_msg.landmarks = [Point() for _ in range(21)]
                landmarks_msg.handedness = "none"

                for idx, lm in enumerate(landmarks.landmark):
                    if idx >= 21:
                        break
                    landmarks_msg.landmarks[idx].x = float(lm.x)
                    landmarks_msg.landmarks[idx].y = float(lm.y)
                    landmarks_msg.landmarks[idx].z = float(lm.z)
                    landmarks_points.append(
                        Point(x=float(lm.x), y=float(lm.y), z=float(lm.z))
                    )

                if results.multi_handedness:
                    handedness = str(
                        results.multi_handedness[0].classification[0].label
                    ).lower()
                    if self._mirror_handedness:
                        handedness = "left" if handedness == "right" else "right"
                    landmarks_msg.handedness = handedness

                self._publisher.publish(landmarks_msg)

            with self._overlay_lock:
                self._overlay_points = landmarks_points if has_hand else None

            now_ns = self.get_clock().now().nanoseconds
            if self._calibration_active:
                if has_hand and len(landmarks_points) == 21:
                    metrics, thumb_opp = self._compute_hand_metrics(landmarks_points)
                    for key in ["index", "middle", "ring", "pinky", "thumb"]:
                        self._calib_samples[key].append(metrics[key])
                    self._calib_samples["opp"].append(thumb_opp)

                if now_ns - self._calibration_start_ns >= 2_000_000_000:
                    self._advance_calibration_stage(now_ns)

            if not self._calibrated:
                outputs = [0.0] * 6
                if now_ns - self._last_warn_ns > 2_000_000_000:
                    self.get_logger().warn("Not calibrated; teleop outputs are zero")
                    self._last_warn_ns = now_ns
                self._teleop_prev = outputs
                self._publish_teleop(outputs)
            else:
                if has_hand and len(landmarks_points) == 21:
                    metrics, thumb_opp = self._compute_hand_metrics(landmarks_points)
                    measured = [
                        self._compute_curl(
                            metrics["index"],
                            self._open_ref["index"],
                            self._closed_ref["index"],
                        ),
                        self._compute_curl(
                            metrics["middle"],
                            self._open_ref["middle"],
                            self._closed_ref["middle"],
                        ),
                        self._compute_curl(
                            metrics["ring"],
                            self._open_ref["ring"],
                            self._closed_ref["ring"],
                        ),
                        self._compute_curl(
                            metrics["pinky"],
                            self._open_ref["pinky"],
                            self._closed_ref["pinky"],
                        ),
                        self._compute_curl(
                            metrics["thumb"],
                            self._open_ref["thumb"],
                            self._closed_ref["thumb"],
                        ),
                        self._compute_thumb_opp(
                            thumb_opp, self._thumb_unopp, self._thumb_opp_min
                        ),
                    ]
                    outputs = []
                    for idx, value in enumerate(measured):
                        prev = self._teleop_prev[idx]
                        outputs.append(
                            self._alpha * value + (1.0 - self._alpha) * prev
                        )
                    self._teleop_prev = outputs
                    self._publish_teleop(outputs)
                else:
                    outputs = [
                        max(0.0, value - self._decay_step)
                        for value in self._teleop_prev
                    ]
                    self._teleop_prev = outputs
                    self._publish_teleop(outputs)

            time.sleep(0.001)

    def render_preview(self) -> None:
        if not self._show_preview:
            return
        if self._preview_fps > 0:
            now = time.monotonic()
            if now - self._last_preview_time < 1.0 / self._preview_fps:
                return
            self._last_preview_time = now
        frame = None
        with self._preview_lock:
            if self._preview_frame is not None:
                frame = self._preview_frame.copy()
        if frame is not None:
            with self._overlay_lock:
                overlay_points = self._overlay_points
            if overlay_points:
                self._draw_overlay(frame, overlay_points)
            cv2.imshow("hand_tracking", frame)
            cv2.waitKey(1)
        else:
            cv2.waitKey(1)
            time.sleep(0.01)

    def _draw_overlay(self, frame, points: list[Point]) -> None:
        if len(points) < 21:
            return
        height, width = frame.shape[:2]
        coords = []
        for p in points[:21]:
            x = int(p.x * width)
            y = int(p.y * height)
            x = max(0, min(width - 1, x))
            y = max(0, min(height - 1, y))
            coords.append((x, y))
        for a, b in self._hand_connections:
            cv2.line(frame, coords[a], coords[b], (0, 255, 0), 2)
        for x, y in coords:
            cv2.circle(frame, (x, y), 3, (255, 0, 0), -1)

    def destroy_node(self) -> bool:
        self._hands.close()
        self._process_stop.set()
        if self._process_thread is not None:
            self._process_thread.join(timeout=1.0)
        if self._show_preview:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = HandTrackingNode()
    try:
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        if node._show_preview:
            spin_thread = threading.Thread(target=executor.spin, daemon=True)
            spin_thread.start()
            while rclpy.ok():
                node.render_preview()
                time.sleep(0.01)
        else:
            executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
