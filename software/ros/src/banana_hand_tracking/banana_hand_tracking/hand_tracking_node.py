"""ROS 2 node that runs MediaPipe Hands on one or more image topics."""

from dataclasses import dataclass, field
from typing import Any, Optional
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


def _empty_finger_refs() -> dict[str, Optional[float]]:
    return {
        "index": None,
        "middle": None,
        "ring": None,
        "pinky": None,
        "thumb": None,
    }


def _calibration_samples_for_stage(stage: int) -> dict[str, list[float]]:
    if stage in (1, 2):
        return {
            "index": [],
            "middle": [],
            "ring": [],
            "pinky": [],
        }
    if stage in (3, 4):
        return {"thumb": []}
    if stage in (5, 6):
        return {"opp": []}
    return {}


def _as_string_list(value: Any) -> list[str]:
    if value is None:
        return []
    if isinstance(value, str):
        return [value] if value else []
    return [str(item) for item in value if str(item)]


@dataclass
class CameraState:
    """Per-camera processing state."""

    name: str
    input_topic: str
    frame_id: str
    landmark_topic: str
    preview_window_name: str
    hands: Any
    landmark_publisher: Any
    legacy_landmark_publisher: Optional[Any] = None
    subscription: Optional[Any] = None
    latest_frame_lock: threading.Lock = field(default_factory=threading.Lock)
    latest_frame: Optional[Any] = None
    latest_frame_seq: int = 0
    last_processed_seq: int = -1
    preview_lock: threading.Lock = field(default_factory=threading.Lock)
    preview_frame: Optional[Any] = None
    overlay_lock: threading.Lock = field(default_factory=threading.Lock)
    overlay_points: Optional[list[Point]] = None
    last_process_time: float = 0.0
    last_hand_detected: bool = False
    last_measurement: Optional[list[float]] = None
    last_measurement_time: float = 0.0
    open_ref: dict[str, Optional[float]] = field(default_factory=_empty_finger_refs)
    closed_ref: dict[str, Optional[float]] = field(default_factory=_empty_finger_refs)
    thumb_unopp: Optional[float] = None
    thumb_opp_min: Optional[float] = None
    calib_samples: dict[str, list[float]] = field(default_factory=dict)


class HandTrackingNode(Node):
    """Subscribe to camera image topics, track a hand, and publish teleop output."""

    def __init__(self) -> None:
        super().__init__("hand_tracking_node")
        self.declare_parameter("input_topic", "/camera/image_raw")
        self.declare_parameter("input_topics", [""])
        self.declare_parameter("camera_names", [""])
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("frame_ids", [""])
        self.declare_parameter("landmark_topics", [""])
        self.declare_parameter("show_preview", True)
        self.declare_parameter("mirror_handedness", True)
        self.declare_parameter("alpha", 0.2)
        self.declare_parameter("process_width", 640)
        self.declare_parameter("process_height", 480)
        self.declare_parameter("preview_fps", 15.0)
        self.declare_parameter("process_fps", 0.0)
        self.declare_parameter("camera_stale_timeout", 0.25)
        self.declare_parameter("calibration_seconds", 5.0)

        configured_input_topics = _as_string_list(
            self.get_parameter("input_topics").value
        )
        if configured_input_topics:
            input_topics = configured_input_topics
        else:
            input_topics = [str(self.get_parameter("input_topic").value)]

        configured_camera_names = _as_string_list(
            self.get_parameter("camera_names").value
        )
        if configured_camera_names and len(configured_camera_names) == len(input_topics):
            camera_names = configured_camera_names
        else:
            if configured_camera_names:
                self.get_logger().warn(
                    "camera_names length does not match input_topics; using defaults"
                )
            camera_names = (
                ["camera"]
                if len(input_topics) == 1
                else [f"camera_{idx}" for idx in range(len(input_topics))]
            )

        configured_frame_ids = _as_string_list(self.get_parameter("frame_ids").value)
        if configured_frame_ids and len(configured_frame_ids) == len(input_topics):
            frame_ids = configured_frame_ids
        else:
            if configured_frame_ids:
                self.get_logger().warn(
                    "frame_ids length does not match input_topics; using defaults"
                )
            single_frame_id = str(self.get_parameter("frame_id").value)
            frame_ids = (
                [single_frame_id]
                if len(input_topics) == 1
                else [f"{name}_frame" for name in camera_names]
            )

        configured_landmark_topics = _as_string_list(
            self.get_parameter("landmark_topics").value
        )
        if configured_landmark_topics and len(configured_landmark_topics) == len(
            input_topics
        ):
            landmark_topics = configured_landmark_topics
        else:
            if configured_landmark_topics:
                self.get_logger().warn(
                    "landmark_topics length does not match input_topics; using defaults"
                )
            landmark_topics = (
                ["/hand/landmarks"]
                if len(input_topics) == 1
                else [f"/hand/landmarks/{name}" for name in camera_names]
            )

        self._show_preview = bool(self.get_parameter("show_preview").value)
        self._mirror_handedness = bool(
            self.get_parameter("mirror_handedness").value
        )
        self._alpha = float(self.get_parameter("alpha").value)
        self._process_width = int(self.get_parameter("process_width").value)
        self._process_height = int(self.get_parameter("process_height").value)
        self._preview_fps = float(self.get_parameter("preview_fps").value)
        self._process_fps = float(self.get_parameter("process_fps").value)
        self._camera_stale_timeout = float(
            self.get_parameter("camera_stale_timeout").value
        )
        calibration_seconds = float(self.get_parameter("calibration_seconds").value)
        self._calibration_stage_duration_ns = int(
            max(calibration_seconds, 0.1) * 1_000_000_000
        )

        self._callback_group = ReentrantCallbackGroup()
        self._bridge = CvBridge()
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
        self._teleop_prev = [0.0] * len(self._joint_names)
        self._decay_step = 0.02
        self._last_warn_ns = 0
        self._last_preview_time = 0.0

        self._mp_hands = mp.solutions.hands
        self._hand_connections = list(self._mp_hands.HAND_CONNECTIONS)
        self._cameras: list[CameraState] = []
        for idx, input_topic in enumerate(input_topics):
            name = camera_names[idx]
            frame_id = frame_ids[idx]
            landmark_topic = landmark_topics[idx]
            preview_window_name = f"hand_tracking_{name}"
            hands = self._mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=1,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5,
            )
            landmark_publisher = self.create_publisher(
                HandLandmarks, landmark_topic, 10
            )
            legacy_landmark_publisher = None
            if idx == 0 and len(input_topics) > 1 and landmark_topic != "/hand/landmarks":
                legacy_landmark_publisher = self.create_publisher(
                    HandLandmarks, "/hand/landmarks", 10
                )

            camera = CameraState(
                name=name,
                input_topic=input_topic,
                frame_id=frame_id,
                landmark_topic=landmark_topic,
                preview_window_name=preview_window_name,
                hands=hands,
                landmark_publisher=landmark_publisher,
                legacy_landmark_publisher=legacy_landmark_publisher,
            )
            camera.subscription = self.create_subscription(
                Image,
                input_topic,
                lambda msg, tracked_camera=camera: self._on_image(
                    tracked_camera, msg
                ),
                10,
                callback_group=self._callback_group,
            )
            self._cameras.append(camera)
            if self._show_preview:
                cv2.namedWindow(preview_window_name, cv2.WINDOW_NORMAL)

        self.get_logger().info(
            "Tracking cameras: "
            + ", ".join(
                f"{camera.name} ({camera.input_topic})" for camera in self._cameras
            )
        )
        if len(self._cameras) > 1:
            self.get_logger().info(
                "Publishing averaged teleop output across all active cameras"
            )

        self._calibrated = False
        self._calibration_active = False
        self._calibration_stage = 0
        self._calibration_start_ns = 0
        self._calibration_done = threading.Event()
        self._calibration_result = (False, "Calibration not started")
        self._process_stop = threading.Event()
        self._process_thread = threading.Thread(
            target=self._process_loop, daemon=True
        )
        self._process_thread.start()

    def _dist2d(self, a: Point, b: Point) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    def _clip01(self, value: float) -> float:
        return max(0.0, min(1.0, value))

    def _camera_has_calibration(self, camera: CameraState) -> bool:
        return (
            all(camera.open_ref[key] is not None for key in camera.open_ref)
            and all(camera.closed_ref[key] is not None for key in camera.closed_ref)
            and camera.thumb_unopp is not None
            and camera.thumb_opp_min is not None
        )

    def _compute_hand_metrics(self, landmarks: list[Point]) -> tuple[dict, float]:
        hand_scale = self._dist2d(landmarks[0], landmarks[9]) + 1e-6
        metrics = {
            "index": self._dist2d(landmarks[8], landmarks[5]) / hand_scale,
            "middle": self._dist2d(landmarks[12], landmarks[9]) / hand_scale,
            "ring": self._dist2d(landmarks[16], landmarks[13]) / hand_scale,
            "pinky": self._dist2d(landmarks[20], landmarks[17]) / hand_scale,
            "thumb": self._dist2d(landmarks[4], landmarks[2]) / hand_scale,
        }
        thumb_opp = self._dist2d(landmarks[3], landmarks[17]) / hand_scale
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

    def _set_calibration_samples_for_stage(self, stage: int) -> None:
        for camera in self._cameras:
            camera.calib_samples = _calibration_samples_for_stage(stage)

    def _start_calibration(self) -> None:
        self._calibration_active = True
        self._calibration_stage = 1
        self._calibration_start_ns = self.get_clock().now().nanoseconds
        self._calibration_done.clear()
        self._set_calibration_samples_for_stage(self._calibration_stage)
        self.get_logger().info("Finger flex calibration stage 1/6: Hold hand OPEN")

    def _finish_calibration(self, success: bool, message: str) -> None:
        self._calibration_active = False
        self._calibration_stage = 0
        self._calibration_result = (success, message)
        self._calibration_done.set()

    def _calibration_missing_cameras(self, sample_keys: list[str]) -> list[str]:
        missing = []
        for camera in self._cameras:
            if not all(camera.calib_samples.get(key) for key in sample_keys):
                missing.append(camera.name)
        return missing

    def _advance_calibration_stage(self, now_ns: int) -> None:
        if self._calibration_stage == 1:
            missing = self._calibration_missing_cameras(
                ["index", "middle", "ring", "pinky"]
            )
            if missing:
                self._finish_calibration(
                    False,
                    "Calibration failed: no hand detected on "
                    + ", ".join(missing)
                    + " during open-hand capture",
                )
                return

            for camera in self._cameras:
                for key in ["index", "middle", "ring", "pinky"]:
                    camera.open_ref[key] = statistics.median(
                        camera.calib_samples[key]
                    )

            self._calibration_stage = 2
            self._calibration_start_ns = now_ns
            self._set_calibration_samples_for_stage(self._calibration_stage)
            self.get_logger().info("Finger flex calibration stage 2/6: Make a FIST")
        elif self._calibration_stage == 2:
            missing = self._calibration_missing_cameras(
                ["index", "middle", "ring", "pinky"]
            )
            if missing:
                self._finish_calibration(
                    False,
                    "Calibration failed: no hand detected on "
                    + ", ".join(missing)
                    + " during fist capture",
                )
                return

            for camera in self._cameras:
                for key in ["index", "middle", "ring", "pinky"]:
                    camera.closed_ref[key] = statistics.median(
                        camera.calib_samples[key]
                    )

            self._calibration_stage = 3
            self._calibration_start_ns = now_ns
            self._set_calibration_samples_for_stage(self._calibration_stage)
            self.get_logger().info("Thumb flex calibration stage 3/6: Hold hand OPEN")
        elif self._calibration_stage == 3:
            missing = self._calibration_missing_cameras(["thumb"])
            if missing:
                self._finish_calibration(
                    False,
                    "Calibration failed: no hand detected on "
                    + ", ".join(missing)
                    + " during thumb-open capture",
                )
                return

            for camera in self._cameras:
                camera.open_ref["thumb"] = statistics.median(
                    camera.calib_samples["thumb"]
                )

            self._calibration_stage = 4
            self._calibration_start_ns = now_ns
            self._set_calibration_samples_for_stage(self._calibration_stage)
            self.get_logger().info("Thumb flex calibration stage 4/6: Flex THUMB")
        elif self._calibration_stage == 4:
            missing = self._calibration_missing_cameras(["thumb"])
            if missing:
                self._finish_calibration(
                    False,
                    "Calibration failed: no hand detected on "
                    + ", ".join(missing)
                    + " during thumb-flex capture",
                )
                return

            for camera in self._cameras:
                camera.closed_ref["thumb"] = statistics.median(
                    camera.calib_samples["thumb"]
                )

            self._calibration_stage = 5
            self._calibration_start_ns = now_ns
            self._set_calibration_samples_for_stage(self._calibration_stage)
            self.get_logger().info(
                "Thumb opposition calibration stage 5/6: Hold hand OPEN"
            )
        elif self._calibration_stage == 5:
            missing = self._calibration_missing_cameras(["opp"])
            if missing:
                self._finish_calibration(
                    False,
                    "Calibration failed: no hand detected on "
                    + ", ".join(missing)
                    + " during thumb-open opposition capture",
                )
                return

            for camera in self._cameras:
                camera.thumb_unopp = statistics.median(camera.calib_samples["opp"])

            self._calibration_stage = 6
            self._calibration_start_ns = now_ns
            self._set_calibration_samples_for_stage(self._calibration_stage)
            self.get_logger().info(
                "Thumb opposition calibration stage 6/6: Touch THUMB to base of PINKY"
            )
        elif self._calibration_stage == 6:
            missing = self._calibration_missing_cameras(["opp"])
            if missing:
                self._finish_calibration(
                    False,
                    "Calibration failed: no hand detected on "
                    + ", ".join(missing)
                    + " during thumb-opposition capture",
                )
                return

            for camera in self._cameras:
                camera.thumb_opp_min = statistics.median(
                    camera.calib_samples["opp"]
                )

            self._calibrated = True
            self._finish_calibration(True, "Calibration complete")

    def _on_calibrate(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self._calibration_active:
            response.success = False
            response.message = "Calibration already in progress"
            return response

        missing = [camera.name for camera in self._cameras if not camera.last_hand_detected]
        if missing:
            response.success = False
            response.message = "No hand detected on: " + ", ".join(missing)
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

    def _on_image(self, camera: CameraState, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        with camera.latest_frame_lock:
            camera.latest_frame = frame
            camera.latest_frame_seq += 1
        if self._show_preview:
            with camera.preview_lock:
                camera.preview_frame = frame

    def _publish_landmarks(
        self,
        camera: CameraState,
        results: Any,
    ) -> list[Point]:
        landmarks = results.multi_hand_landmarks[0]
        landmarks_msg = HandLandmarks()
        landmarks_msg.header.stamp = self.get_clock().now().to_msg()
        landmarks_msg.header.frame_id = camera.frame_id
        landmarks_msg.landmarks = [Point() for _ in range(21)]
        landmarks_msg.handedness = "none"

        landmarks_points: list[Point] = []
        for idx, lm in enumerate(landmarks.landmark):
            if idx >= 21:
                break
            point = Point(x=float(lm.x), y=float(lm.y), z=float(lm.z))
            landmarks_msg.landmarks[idx] = point
            landmarks_points.append(point)

        if results.multi_handedness:
            handedness = str(
                results.multi_handedness[0].classification[0].label
            ).lower()
            if self._mirror_handedness:
                handedness = "left" if handedness == "right" else "right"
            landmarks_msg.handedness = handedness

        camera.landmark_publisher.publish(landmarks_msg)
        if camera.legacy_landmark_publisher is not None:
            camera.legacy_landmark_publisher.publish(landmarks_msg)

        return landmarks_points

    def _record_calibration_sample(
        self,
        camera: CameraState,
        landmarks_points: list[Point],
    ) -> None:
        metrics, thumb_opp = self._compute_hand_metrics(landmarks_points)
        if self._calibration_stage in (1, 2):
            for key in ["index", "middle", "ring", "pinky"]:
                camera.calib_samples[key].append(metrics[key])
        elif self._calibration_stage in (3, 4):
            camera.calib_samples["thumb"].append(metrics["thumb"])
        elif self._calibration_stage in (5, 6):
            camera.calib_samples["opp"].append(thumb_opp)

    def _compute_camera_measurement(
        self,
        camera: CameraState,
        landmarks_points: list[Point],
    ) -> list[float]:
        metrics, thumb_opp = self._compute_hand_metrics(landmarks_points)
        return [
            self._compute_curl(
                metrics["index"],
                camera.open_ref["index"],
                camera.closed_ref["index"],
            ),
            self._compute_curl(
                metrics["middle"],
                camera.open_ref["middle"],
                camera.closed_ref["middle"],
            ),
            self._compute_curl(
                metrics["ring"],
                camera.open_ref["ring"],
                camera.closed_ref["ring"],
            ),
            self._compute_curl(
                metrics["pinky"],
                camera.open_ref["pinky"],
                camera.closed_ref["pinky"],
            ),
            self._compute_curl(
                metrics["thumb"],
                camera.open_ref["thumb"],
                camera.closed_ref["thumb"],
            ),
            self._compute_thumb_opp(
                thumb_opp,
                camera.thumb_unopp,
                camera.thumb_opp_min,
            ),
        ]

    def _process_camera(self, camera: CameraState) -> bool:
        with camera.latest_frame_lock:
            seq = camera.latest_frame_seq
            frame = camera.latest_frame

        if frame is None or seq == camera.last_processed_seq:
            return False

        now = time.monotonic()
        if self._process_fps > 0:
            if now - camera.last_process_time < 1.0 / self._process_fps:
                return False
        camera.last_process_time = now
        camera.last_processed_seq = seq

        if self._process_width > 0 and self._process_height > 0:
            frame_proc = cv2.resize(frame, (self._process_width, self._process_height))
        else:
            frame_proc = frame

        rgb = cv2.cvtColor(frame_proc, cv2.COLOR_BGR2RGB)
        results = camera.hands.process(rgb)

        has_hand = bool(results.multi_hand_landmarks)
        camera.last_hand_detected = has_hand

        landmarks_points: list[Point] = []
        if has_hand:
            landmarks_points = self._publish_landmarks(camera, results)

        with camera.overlay_lock:
            camera.overlay_points = landmarks_points if has_hand else None

        if self._calibration_active and has_hand and len(landmarks_points) == 21:
            self._record_calibration_sample(camera, landmarks_points)

        if (
            self._calibrated
            and has_hand
            and len(landmarks_points) == 21
            and self._camera_has_calibration(camera)
        ):
            camera.last_measurement = self._compute_camera_measurement(
                camera, landmarks_points
            )
            camera.last_measurement_time = now
        else:
            camera.last_measurement = None

        return True

    def _compute_teleop_outputs(self) -> list[float]:
        now = time.monotonic()
        active_measurements = [
            camera.last_measurement
            for camera in self._cameras
            if camera.last_measurement is not None
            and now - camera.last_measurement_time <= self._camera_stale_timeout
        ]

        if not active_measurements:
            return [max(0.0, value - self._decay_step) for value in self._teleop_prev]

        averaged = []
        for idx in range(len(self._joint_names)):
            averaged.append(
                sum(measurement[idx] for measurement in active_measurements)
                / len(active_measurements)
            )

        outputs = []
        for idx, value in enumerate(averaged):
            prev = self._teleop_prev[idx]
            outputs.append(self._alpha * value + (1.0 - self._alpha) * prev)
        return outputs

    def _process_loop(self) -> None:
        while not self._process_stop.is_set() and rclpy.ok():
            processed_any_frame = False
            for camera in self._cameras:
                if self._process_camera(camera):
                    processed_any_frame = True

            now_ns = self.get_clock().now().nanoseconds
            if self._calibration_active:
                if (
                    now_ns - self._calibration_start_ns
                    >= self._calibration_stage_duration_ns
                ):
                    self._advance_calibration_stage(now_ns)

            if not processed_any_frame:
                time.sleep(0.001)
                continue

            if not self._calibrated:
                outputs = [0.0] * len(self._joint_names)
                if now_ns - self._last_warn_ns > 2_000_000_000:
                    self._last_warn_ns = now_ns
            else:
                outputs = self._compute_teleop_outputs()

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

        showed_frame = False
        for camera in self._cameras:
            frame = None
            with camera.preview_lock:
                if camera.preview_frame is not None:
                    frame = camera.preview_frame.copy()
            if frame is None:
                continue

            showed_frame = True
            with camera.overlay_lock:
                overlay_points = camera.overlay_points
            if overlay_points:
                self._draw_overlay(frame, overlay_points)
            cv2.imshow(camera.preview_window_name, frame)

        cv2.waitKey(1)
        if not showed_frame:
            time.sleep(0.01)

    def _draw_overlay(self, frame, points: list[Point]) -> None:
        if len(points) < 21:
            return
        height, width = frame.shape[:2]
        coords = []
        for point in points[:21]:
            x = int(point.x * width)
            y = int(point.y * height)
            x = max(0, min(width - 1, x))
            y = max(0, min(height - 1, y))
            coords.append((x, y))
        for a, b in self._hand_connections:
            cv2.line(frame, coords[a], coords[b], (0, 255, 0), 2)
        for x, y in coords:
            cv2.circle(frame, (x, y), 3, (255, 0, 0), -1)

    def destroy_node(self) -> bool:
        self._process_stop.set()
        if self._process_thread is not None:
            self._process_thread.join(timeout=1.0)
        for camera in self._cameras:
            camera.hands.close()
        if self._show_preview:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = HandTrackingNode()
    executor = None
    try:
        executor = MultiThreadedExecutor(num_threads=max(2, len(node._cameras) + 1))
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
            if executor is not None:
                executor.shutdown()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
