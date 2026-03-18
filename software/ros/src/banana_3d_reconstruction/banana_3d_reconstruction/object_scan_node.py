"""ROS 2 node for burst-mode tabletop object scanning with an Intel RealSense camera."""

from __future__ import annotations

import copy
import json
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import select
import sys
import threading
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    import termios
    import tty
except ImportError:
    termios = None
    tty = None

try:
    import pyrealsense2 as rs
except ImportError as exc:
    raise SystemExit(
        "pyrealsense2 is not installed. Try: pip install pyrealsense2"
    ) from exc

from banana_3d_reconstruction.pointcloud_utils import (
    CameraIntrinsics,
    camera_intrinsics_from_realsense,
    cluster_candidate_to_dict,
    clone_cloud,
    compute_cloud_metadata,
    depth_image_to_point_cloud,
    open3d_to_pointcloud2,
    point_count,
    postprocess_object_cloud,
    register_cloud_icp,
    resolve_output_dir,
    scale_camera_intrinsics,
    segment_tabletop_object,
    transform_cloud_copy,
    voxel_downsample,
    write_metadata_json,
    write_point_cloud,
)


@dataclass
class FrameBundle:
    color_image_bgr: np.ndarray
    depth_image_m: np.ndarray
    intrinsics: CameraIntrinsics
    stamp_ns: int


@dataclass
class SavedScan:
    clean_cloud: object
    downsampled_cloud: object
    color_image_bgr: np.ndarray
    metadata: dict
    scan_dir: Path


class ObjectScanNode(Node):
    """Capture, segment, fuse, and publish a burst-mode tabletop object cloud."""

    def __init__(self) -> None:
        super().__init__("object_scan_node")
        self._declare_parameters()
        self._load_parameters()

        self._state_lock = threading.Lock()
        self._latest_frame: Optional[FrameBundle] = None
        self._processing_thread: Optional[threading.Thread] = None
        self._pending_start = False
        self._scanning = False
        self._processing = False
        self._stop_scan_early = False
        self._scan_started_monotonic = 0.0
        self._scan_end_monotonic = 0.0
        self._last_keyframe_time = 0.0
        self._captured_frames: list[FrameBundle] = []
        self._last_saved_scan: Optional[SavedScan] = None
        self._last_status_message = ""
        self._shutdown_requested = False
        self._keyboard_thread: Optional[threading.Thread] = None
        self._keyboard_stop = threading.Event()
        self._stdin_fd: Optional[int] = None
        self._stdin_term_settings: Optional[list[int]] = None

        self._final_cloud_publisher = self.create_publisher(
            PointCloud2, self._output_topic, 1
        )
        self._status_publisher = self.create_publisher(
            String, "/object_scan/status", 10
        )
        self._raw_debug_publisher = None
        self._plane_debug_publisher = None
        self._selected_debug_publisher = None
        if self._publish_debug_topics:
            self._raw_debug_publisher = self.create_publisher(
                PointCloud2, "/object_scan/debug/raw_frame_cloud", 1
            )
            self._plane_debug_publisher = self.create_publisher(
                PointCloud2, "/object_scan/debug/plane_removed_cloud", 1
            )
            self._selected_debug_publisher = self.create_publisher(
                PointCloud2, "/object_scan/debug/selected_object_cloud", 1
            )

        self._start_service = self.create_service(
            Trigger, "/object_scan/start_scan", self._on_start_scan
        )
        self._save_last_service = self.create_service(
            Trigger, "/object_scan/save_last_scan", self._on_save_last_scan
        )

        self._preview_window = "object_scan_preview"
        if self._show_preview:
            cv2.namedWindow(self._preview_window, cv2.WINDOW_NORMAL)

        self._start_camera()
        self._start_keyboard_listener()
        self._timer = self.create_timer(1.0 / max(float(self._fps), 1.0), self._tick)
        self._publish_status(
            "idle: press s in the terminal or call /object_scan/start_scan"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("frame_id", "object_scan_frame")
        self.declare_parameter("device_serial", "")
        self.declare_parameter("color_width", 640)
        self.declare_parameter("color_height", 480)
        self.declare_parameter("depth_width", 640)
        self.declare_parameter("depth_height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("show_preview", True)
        self.declare_parameter("burst_duration_sec", 1.5)
        self.declare_parameter("burst_frame_limit", 40)
        self.declare_parameter("output_dir", "~/banana_scans")
        self.declare_parameter("output_topic", "/object/point_cloud")
        self.declare_parameter("min_depth_m", 0.15)
        self.declare_parameter("max_depth_m", 1.00)
        self.declare_parameter("voxel_size_m", 0.005)
        self.declare_parameter("plane_distance_threshold_m", 0.008)
        self.declare_parameter("min_object_height_above_plane_m", 0.010)
        self.declare_parameter("dbscan_eps_m", 0.025)
        self.declare_parameter("dbscan_min_points", 60)
        self.declare_parameter("icp_max_correspondence_distance_m", 0.040)
        self.declare_parameter("icp_fitness_threshold", 0.35)
        self.declare_parameter("save_intermediate_debug_clouds", False)
        self.declare_parameter("publish_debug_topics", False)
        self.declare_parameter("decimation_magnitude", 2)
        self.declare_parameter("spatial_filter_magnitude", 2.0)
        self.declare_parameter("spatial_filter_alpha", 0.5)
        self.declare_parameter("spatial_filter_delta", 20.0)
        self.declare_parameter("temporal_filter_alpha", 0.4)
        self.declare_parameter("temporal_filter_delta", 20.0)
        self.declare_parameter("hole_filling_mode", 1)
        self.declare_parameter("plane_ransac_iterations", 300)

    def _load_parameters(self) -> None:
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._device_serial = str(self.get_parameter("device_serial").value)
        self._color_width = int(self.get_parameter("color_width").value)
        self._color_height = int(self.get_parameter("color_height").value)
        self._depth_width = int(self.get_parameter("depth_width").value)
        self._depth_height = int(self.get_parameter("depth_height").value)
        self._fps = int(self.get_parameter("fps").value)
        self._show_preview = bool(self.get_parameter("show_preview").value)
        self._burst_duration_sec = float(
            self.get_parameter("burst_duration_sec").value
        )
        self._burst_frame_limit = int(self.get_parameter("burst_frame_limit").value)
        self._output_dir = resolve_output_dir(str(self.get_parameter("output_dir").value))
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._min_depth_m = float(self.get_parameter("min_depth_m").value)
        self._max_depth_m = float(self.get_parameter("max_depth_m").value)
        self._voxel_size_m = float(self.get_parameter("voxel_size_m").value)
        self._plane_distance_threshold_m = float(
            self.get_parameter("plane_distance_threshold_m").value
        )
        self._min_object_height_above_plane_m = float(
            self.get_parameter("min_object_height_above_plane_m").value
        )
        self._dbscan_eps_m = float(self.get_parameter("dbscan_eps_m").value)
        self._dbscan_min_points = int(self.get_parameter("dbscan_min_points").value)
        self._icp_max_correspondence_distance_m = float(
            self.get_parameter("icp_max_correspondence_distance_m").value
        )
        self._icp_fitness_threshold = float(
            self.get_parameter("icp_fitness_threshold").value
        )
        self._save_intermediate_debug_clouds = bool(
            self.get_parameter("save_intermediate_debug_clouds").value
        )
        self._publish_debug_topics = bool(
            self.get_parameter("publish_debug_topics").value
        )
        self._decimation_magnitude = int(
            self.get_parameter("decimation_magnitude").value
        )
        self._spatial_filter_magnitude = float(
            self.get_parameter("spatial_filter_magnitude").value
        )
        self._spatial_filter_alpha = float(
            self.get_parameter("spatial_filter_alpha").value
        )
        self._spatial_filter_delta = float(
            self.get_parameter("spatial_filter_delta").value
        )
        self._temporal_filter_alpha = float(
            self.get_parameter("temporal_filter_alpha").value
        )
        self._temporal_filter_delta = float(
            self.get_parameter("temporal_filter_delta").value
        )
        self._hole_filling_mode = int(self.get_parameter("hole_filling_mode").value)
        self._plane_ransac_iterations = int(
            self.get_parameter("plane_ransac_iterations").value
        )

    def _start_camera(self) -> None:
        self._pipeline = rs.pipeline()
        config = rs.config()
        if self._device_serial:
            config.enable_device(self._device_serial)
        config.enable_stream(
            rs.stream.color,
            self._color_width,
            self._color_height,
            rs.format.bgr8,
            self._fps,
        )
        config.enable_stream(
            rs.stream.depth,
            self._depth_width,
            self._depth_height,
            rs.format.z16,
            self._fps,
        )

        try:
            self._pipeline_profile = self._pipeline.start(config)
        except RuntimeError as exc:
            raise SystemExit(f"Unable to start RealSense pipeline: {exc}") from exc

        self._align = rs.align(rs.stream.color)
        depth_sensor = self._pipeline_profile.get_device().first_depth_sensor()
        self._depth_scale = float(depth_sensor.get_depth_scale())

        self._decimation_filter = rs.decimation_filter()
        self._decimation_filter.set_option(
            rs.option.filter_magnitude, float(max(self._decimation_magnitude, 1))
        )
        self._spatial_filter = rs.spatial_filter()
        self._spatial_filter.set_option(
            rs.option.filter_magnitude, self._spatial_filter_magnitude
        )
        self._spatial_filter.set_option(
            rs.option.filter_smooth_alpha, self._spatial_filter_alpha
        )
        self._spatial_filter.set_option(
            rs.option.filter_smooth_delta, self._spatial_filter_delta
        )
        self._temporal_filter = rs.temporal_filter()
        self._temporal_filter.set_option(
            rs.option.filter_smooth_alpha, self._temporal_filter_alpha
        )
        self._temporal_filter.set_option(
            rs.option.filter_smooth_delta, self._temporal_filter_delta
        )
        self._hole_filling_filter = rs.hole_filling_filter(self._hole_filling_mode)

        try:
            device = self._pipeline_profile.get_device()
            serial = device.get_info(rs.camera_info.serial_number)
            name = device.get_info(rs.camera_info.name)
            self.get_logger().info(
                f"RealSense started: {name} ({serial}), depth_scale={self._depth_scale:.6f}"
            )
        except RuntimeError:
            self.get_logger().info(
                f"RealSense started, depth_scale={self._depth_scale:.6f}"
            )

    def _start_keyboard_listener(self) -> None:
        if not sys.stdin.isatty() or termios is None or tty is None:
            self.get_logger().info(
                "Terminal keyboard controls unavailable; use /object_scan/start_scan"
            )
            return

        try:
            self._stdin_fd = sys.stdin.fileno()
            self._stdin_term_settings = termios.tcgetattr(self._stdin_fd)
            tty.setcbreak(self._stdin_fd)
        except (termios.error, ValueError, OSError) as exc:
            self.get_logger().warning(
                f"Unable to enable terminal keyboard controls: {exc}"
            )
            self._stdin_fd = None
            self._stdin_term_settings = None
            return

        self._keyboard_thread = threading.Thread(
            target=self._keyboard_loop,
            name="object_scan_keyboard",
            daemon=True,
        )
        self._keyboard_thread.start()
        self.get_logger().info(
            "Keyboard controls enabled on the terminal: s=start, e=end early, q=quit"
        )

    def _keyboard_loop(self) -> None:
        while not self._keyboard_stop.is_set() and not self._shutdown_requested:
            if self._stdin_fd is None:
                return
            try:
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            except (ValueError, OSError):
                return
            if not readable:
                continue
            try:
                key = sys.stdin.read(1)
            except (OSError, ValueError):
                return
            if not key:
                continue
            self._handle_terminal_key(key)

    def _handle_terminal_key(self, key: str) -> None:
        if key == "s":
            self._request_scan_start(source="terminal")
        elif key == "e":
            with self._state_lock:
                if self._scanning:
                    self._stop_scan_early = True
            self._publish_status("early scan stop requested")
        elif key == "q":
            self._shutdown_requested = True
            self._publish_status("shutdown requested from terminal")
            rclpy.shutdown()

    def _apply_depth_filters(self, depth_frame: object) -> object:
        filtered = self._decimation_filter.process(depth_frame)
        filtered = self._spatial_filter.process(filtered)
        filtered = self._temporal_filter.process(filtered)
        filtered = self._hole_filling_filter.process(filtered)
        return filtered

    def _tick(self) -> None:
        if self._shutdown_requested:
            return

        frame = self._capture_frame()
        if frame is None:
            return

        with self._state_lock:
            self._latest_frame = frame

        self._handle_capture_state(frame)

        if self._show_preview:
            self._render_preview(frame)

    def _capture_frame(self) -> Optional[FrameBundle]:
        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=100)
        except RuntimeError as exc:
            self.get_logger().warning(f"RealSense frame wait failed: {exc}")
            return None

        aligned_frames = self._align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return None

        filtered_depth = self._apply_depth_filters(depth_frame)
        color_image_bgr = np.asarray(color_frame.get_data()).copy()
        depth_image_m = (
            np.asarray(filtered_depth.get_data()).astype(np.float32).copy()
            * self._depth_scale
        )
        intrinsics = camera_intrinsics_from_realsense(
            color_frame.profile.as_video_stream_profile().intrinsics
        )

        if color_image_bgr.shape[:2] != depth_image_m.shape[:2]:
            depth_height, depth_width = depth_image_m.shape[:2]
            color_image_bgr = cv2.resize(
                color_image_bgr,
                (depth_width, depth_height),
                interpolation=cv2.INTER_LINEAR,
            )
            intrinsics = scale_camera_intrinsics(intrinsics, depth_width, depth_height)

        depth_image_m[~np.isfinite(depth_image_m)] = 0.0
        depth_image_m[
            (depth_image_m < self._min_depth_m) | (depth_image_m > self._max_depth_m)
        ] = 0.0

        return FrameBundle(
            color_image_bgr=color_image_bgr,
            depth_image_m=depth_image_m,
            intrinsics=intrinsics,
            stamp_ns=self.get_clock().now().nanoseconds,
        )

    def _copy_frame_bundle(self, frame: FrameBundle) -> FrameBundle:
        return FrameBundle(
            color_image_bgr=frame.color_image_bgr.copy(),
            depth_image_m=frame.depth_image_m.copy(),
            intrinsics=frame.intrinsics,
            stamp_ns=frame.stamp_ns,
        )

    def _handle_capture_state(self, frame: FrameBundle) -> None:
        frames_to_process: Optional[list[FrameBundle]] = None

        with self._state_lock:
            if self._pending_start and not self._scanning and not self._processing:
                self._begin_scan_locked(frame)

            if self._scanning:
                now = time.monotonic()
                capture_interval = 1.0 / max(float(self._fps), 1.0)
                if now - self._last_keyframe_time >= capture_interval:
                    self._captured_frames.append(self._copy_frame_bundle(frame))
                    self._last_keyframe_time = now

                burst_limit_hit = len(self._captured_frames) >= max(
                    self._burst_frame_limit, 1
                )
                if self._stop_scan_early or now >= self._scan_end_monotonic or burst_limit_hit:
                    frames_to_process = list(self._captured_frames)
                    self._captured_frames = []
                    self._scanning = False
                    self._processing = True
                    self._pending_start = False
                    self._stop_scan_early = False

        if frames_to_process:
            self._publish_status(
                f"processing {len(frames_to_process)} burst frames"
            )
            self._processing_thread = threading.Thread(
                target=self._process_scan,
                args=(frames_to_process,),
                daemon=True,
            )
            self._processing_thread.start()

    def _begin_scan_locked(self, frame: FrameBundle) -> None:
        now = time.monotonic()
        self._pending_start = False
        self._scanning = True
        self._stop_scan_early = False
        self._captured_frames = [self._copy_frame_bundle(frame)]
        self._scan_started_monotonic = now
        self._scan_end_monotonic = now + self._burst_duration_sec
        self._last_keyframe_time = now
        self.get_logger().info("Starting burst scan")
        self._publish_status(
            "burst scan started: keep one object on a mostly clear desk and hold the camera mostly still"
        )

    def _process_scan(self, captured_frames: list[FrameBundle]) -> None:
        scan_dir = self._make_scan_dir()
        debug_dir = scan_dir / "debug"
        if self._save_intermediate_debug_clouds:
            debug_dir.mkdir(parents=True, exist_ok=True)
        representative_frame = captured_frames[0]

        fused_cloud = None
        last_accepted_centroid = None
        accepted_frames = 0
        frame_logs: list[dict] = []
        burst_fitness_threshold = max(
            0.18, min(self._icp_fitness_threshold, 0.28)
        )

        for frame_index, frame in enumerate(captured_frames):
            raw_cloud = depth_image_to_point_cloud(
                frame.color_image_bgr,
                frame.depth_image_m,
                frame.intrinsics,
                self._min_depth_m,
                self._max_depth_m,
            )
            raw_cloud = voxel_downsample(raw_cloud, max(self._voxel_size_m * 0.5, 0.002))

            self._publish_raw_debug_cloud(raw_cloud)
            self._save_raw_debug_cloud(debug_dir, frame_index, raw_cloud)

            if point_count(raw_cloud) == 0:
                frame_log = {
                    "frame_index": frame_index,
                    "segmentation_succeeded": False,
                    "accepted": False,
                    "reason": "empty_raw_cloud",
                }
                frame_logs.append(frame_log)
                self._write_frame_debug_json(debug_dir, frame_index, frame_log)
                continue

            segmentation = segment_tabletop_object(
                raw_cloud=raw_cloud,
                intrinsics=frame.intrinsics,
                plane_distance_threshold_m=self._plane_distance_threshold_m,
                min_object_height_above_plane_m=self._min_object_height_above_plane_m,
                dbscan_eps_m=self._dbscan_eps_m,
                dbscan_min_points=self._dbscan_min_points,
                voxel_size_m=self._voxel_size_m,
                plane_ransac_iterations=self._plane_ransac_iterations,
            )

            self._publish_plane_debug_cloud(segmentation.plane_removed_cloud)
            if segmentation.selected_cloud is not None:
                self._publish_selected_debug_cloud(segmentation.selected_cloud)
            self._save_debug_cloud(
                debug_dir,
                f"keyframe_{frame_index:03d}_plane_removed_cloud.ply",
                segmentation.plane_removed_cloud,
            )
            if segmentation.selected_cloud is not None:
                self._save_debug_cloud(
                    debug_dir,
                    f"keyframe_{frame_index:03d}_selected_object_cloud.ply",
                    segmentation.selected_cloud,
                )

            frame_log = {
                "frame_index": frame_index,
                "segmentation_succeeded": bool(
                    segmentation.selected_cloud is not None
                    and point_count(segmentation.selected_cloud) > 0
                ),
                "selection_reason": segmentation.selection_reason,
                "candidate_clusters": [
                    cluster_candidate_to_dict(candidate)
                    for candidate in segmentation.clusters
                ],
                "selected_cluster": None
                if segmentation.selected_cluster is None
                else cluster_candidate_to_dict(segmentation.selected_cluster),
                "selected_cluster_point_count": None
                if segmentation.selected_cluster is None
                else int(segmentation.selected_cluster.point_count),
                "selected_cluster_bbox_extents_m": None
                if segmentation.selected_cluster is None
                else segmentation.selected_cluster.extent_xyz.tolist(),
                "selected_cluster_degeneracy_reason": None
                if segmentation.selected_cluster is None
                else segmentation.selected_cluster.degeneracy_reason,
                "accepted": False,
                "reason": "segmentation_failed",
            }

            if (
                segmentation.selected_cloud is None
                or segmentation.selected_cluster is None
                or point_count(segmentation.selected_cloud) == 0
            ):
                frame_logs.append(frame_log)
                self._write_frame_debug_json(debug_dir, frame_index, frame_log)
                continue

            selected_cloud = voxel_downsample(
                segmentation.selected_cloud, max(self._voxel_size_m * 0.5, 0.002)
            )
            if point_count(selected_cloud) == 0:
                frame_log["reason"] = "selected_cloud_empty_after_downsample"
                frame_logs.append(frame_log)
                self._write_frame_debug_json(debug_dir, frame_index, frame_log)
                continue

            accepted_transform = np.eye(4, dtype=np.float64)
            registration_fitness = None
            registration_rmse = None
            registration_attempted = False

            if fused_cloud is None:
                fused_cloud = clone_cloud(selected_cloud)
                frame_log["accepted"] = True
                frame_log["reason"] = "initialized_from_segmented_object_cloud"
            else:
                registration_attempted = True
                registration = register_cloud_icp(
                    source_cloud=selected_cloud,
                    target_cloud=fused_cloud,
                    voxel_size_m=max(self._voxel_size_m, 0.003),
                    max_correspondence_distance_m=self._icp_max_correspondence_distance_m,
                    initial_transform=np.eye(4, dtype=np.float64),
                )
                if registration is not None:
                    registration_fitness = registration.fitness
                    registration_rmse = registration.inlier_rmse

                if registration is not None and registration.fitness >= burst_fitness_threshold:
                    accepted_transform = registration.transformation
                    registered_cloud = transform_cloud_copy(
                        selected_cloud, accepted_transform
                    )
                    fused_cloud += registered_cloud
                    fused_cloud = voxel_downsample(
                        fused_cloud, max(self._voxel_size_m * 0.5, 0.002)
                    )
                    frame_log["accepted"] = True
                    frame_log["reason"] = "registered_segmented_object_cloud"
                else:
                    centroid_distance = None
                    if last_accepted_centroid is not None:
                        centroid_distance = float(
                            np.linalg.norm(
                                segmentation.selected_cluster.centroid
                                - last_accepted_centroid
                            )
                        )
                    if centroid_distance is not None and centroid_distance <= max(
                        0.02, self._voxel_size_m * 6.0
                    ):
                        fused_cloud += clone_cloud(selected_cloud)
                        fused_cloud = voxel_downsample(
                            fused_cloud, max(self._voxel_size_m * 0.5, 0.002)
                        )
                        frame_log["accepted"] = True
                        frame_log["reason"] = "identity_fallback_small_motion"
                        frame_log["identity_fallback_centroid_distance_m"] = centroid_distance
                    else:
                        frame_log["reason"] = "registration_rejected"

            frame_log["icp_attempted"] = registration_attempted
            frame_log["icp_fitness"] = registration_fitness
            frame_log["icp_rmse"] = registration_rmse

            if not frame_log["accepted"]:
                frame_logs.append(frame_log)
                self._write_frame_debug_json(debug_dir, frame_index, frame_log)
                continue

            if fused_cloud is None:
                fused_cloud = clone_cloud(selected_cloud)
            last_accepted_centroid = segmentation.selected_cluster.centroid.copy()
            accepted_frames += 1

            registered_selected_cloud = transform_cloud_copy(
                selected_cloud, accepted_transform
            )
            self._save_debug_cloud(
                debug_dir,
                f"keyframe_{frame_index:03d}_registered_selected_object_cloud.ply",
                registered_selected_cloud,
            )

            frame_logs.append(frame_log)
            self._write_frame_debug_json(debug_dir, frame_index, frame_log)

        if fused_cloud is None or point_count(fused_cloud) == 0:
            self._finish_processing(
                success=False,
                message="scan failed: no segmented object cloud was accepted",
            )
            return

        self._save_debug_cloud(
            debug_dir,
            "fused_segmented_cloud_pre_cleanup.ply",
            fused_cloud,
        )

        clean_cloud = postprocess_object_cloud(
            fused_cloud,
            voxel_size_m=max(self._voxel_size_m * 0.5, 0.002),
            dbscan_eps_m=self._dbscan_eps_m,
            dbscan_min_points=self._dbscan_min_points,
            plane_distance_threshold_m=self._plane_distance_threshold_m,
            min_object_height_above_plane_m=self._min_object_height_above_plane_m,
            plane_ransac_iterations=self._plane_ransac_iterations,
        )
        if point_count(clean_cloud) == 0:
            clean_cloud = clone_cloud(fused_cloud)

        downsampled_cloud = voxel_downsample(clean_cloud, self._voxel_size_m)
        if point_count(downsampled_cloud) == 0:
            downsampled_cloud = clone_cloud(clean_cloud)

        metadata = compute_cloud_metadata(downsampled_cloud)
        metadata.update(
            {
                "scan_timestamp": datetime.now().isoformat(timespec="seconds"),
                "captured_frame_count": len(captured_frames),
                "accepted_frame_count": accepted_frames,
                "output_topic": self._output_topic,
                "frame_id": self._frame_id,
                "processing_mode": "burst_single_view_tabletop_capture",
                "burst_duration_sec": self._burst_duration_sec,
                "burst_frame_limit": self._burst_frame_limit,
                "files": {
                    "final_object_cloud": "final_object_cloud.ply",
                    "final_object_cloud_downsampled": "final_object_cloud_downsampled.ply",
                    "final_rgb_image": "final_rgb_image.png",
                    "metadata": "metadata.json",
                },
                "frame_logs": frame_logs,
                "parameters": self._parameter_snapshot(),
            }
        )

        write_point_cloud(scan_dir / "final_object_cloud.ply", clean_cloud)
        write_point_cloud(
            scan_dir / "final_object_cloud_downsampled.ply",
            downsampled_cloud,
        )
        self._save_scan_images(
            scan_dir,
            representative_frame.color_image_bgr,
        )
        write_metadata_json(scan_dir / "metadata.json", metadata)

        if self._save_intermediate_debug_clouds:
            write_point_cloud(
                scan_dir / "debug" / "final_cleaned_object_cloud.ply",
                clean_cloud,
            )

        self._publish_final_cloud(downsampled_cloud)
        saved_scan = SavedScan(
            clean_cloud=clone_cloud(clean_cloud),
            downsampled_cloud=clone_cloud(downsampled_cloud),
            color_image_bgr=representative_frame.color_image_bgr.copy(),
            metadata=copy.deepcopy(metadata),
            scan_dir=scan_dir,
        )
        self._finish_processing(
            success=True,
            message=(
                f"scan complete: saved {point_count(downsampled_cloud)} points to {scan_dir}"
            ),
            saved_scan=saved_scan,
        )

    def _publish_raw_debug_cloud(self, cloud: object) -> None:
        if not self._publish_debug_topics or self._raw_debug_publisher is None:
            return
        stamp = self.get_clock().now().to_msg()
        self._raw_debug_publisher.publish(
            open3d_to_pointcloud2(cloud, stamp, self._frame_id)
        )

    def _publish_plane_debug_cloud(self, cloud: object) -> None:
        if not self._publish_debug_topics or self._plane_debug_publisher is None:
            return
        stamp = self.get_clock().now().to_msg()
        self._plane_debug_publisher.publish(
            open3d_to_pointcloud2(cloud, stamp, self._frame_id)
        )

    def _publish_selected_debug_cloud(self, cloud: object) -> None:
        if not self._publish_debug_topics or self._selected_debug_publisher is None:
            return
        stamp = self.get_clock().now().to_msg()
        self._selected_debug_publisher.publish(
            open3d_to_pointcloud2(cloud, stamp, self._frame_id)
        )

    def _save_raw_debug_cloud(
        self, debug_dir: Path, frame_index: int, cloud: object
    ) -> None:
        if not self._save_intermediate_debug_clouds:
            return
        write_point_cloud(debug_dir / f"keyframe_{frame_index:03d}_raw_cloud.ply", cloud)

    def _save_debug_cloud(self, debug_dir: Path, filename: str, cloud: object) -> None:
        if not self._save_intermediate_debug_clouds or point_count(cloud) == 0:
            return
        write_point_cloud(debug_dir / filename, cloud)

    def _write_frame_debug_json(
        self, debug_dir: Path, frame_index: int, payload: dict
    ) -> None:
        if not self._save_intermediate_debug_clouds:
            return
        path = debug_dir / f"keyframe_{frame_index:03d}_debug.json"
        path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def _make_scan_dir(self) -> Path:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        scan_dir = self._output_dir / f"scan_{timestamp}"
        scan_dir.mkdir(parents=True, exist_ok=True)
        return scan_dir

    def _save_scan_images(self, scan_dir: Path, color_image_bgr: np.ndarray) -> None:
        rgb_path = scan_dir / "final_rgb_image.png"
        rgb_written = cv2.imwrite(str(rgb_path), color_image_bgr)
        if not rgb_written:
            raise RuntimeError(
                f"failed to save scan images to {scan_dir}"
            )

    def _publish_final_cloud(self, cloud: object) -> None:
        msg = open3d_to_pointcloud2(cloud, self.get_clock().now().to_msg(), self._frame_id)
        self._final_cloud_publisher.publish(msg)

    def _finish_processing(
        self,
        success: bool,
        message: str,
        saved_scan: Optional[SavedScan] = None,
    ) -> None:
        with self._state_lock:
            self._processing = False
            if saved_scan is not None:
                self._last_saved_scan = saved_scan

        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().warning(message)
        self._publish_status(message)

    def _parameter_snapshot(self) -> dict:
        return {
            "frame_id": self._frame_id,
            "device_serial": self._device_serial,
            "color_width": self._color_width,
            "color_height": self._color_height,
            "depth_width": self._depth_width,
            "depth_height": self._depth_height,
            "fps": self._fps,
            "burst_duration_sec": self._burst_duration_sec,
            "burst_frame_limit": self._burst_frame_limit,
            "output_dir": str(self._output_dir),
            "output_topic": self._output_topic,
            "min_depth_m": self._min_depth_m,
            "max_depth_m": self._max_depth_m,
            "voxel_size_m": self._voxel_size_m,
            "plane_distance_threshold_m": self._plane_distance_threshold_m,
            "min_object_height_above_plane_m": self._min_object_height_above_plane_m,
            "dbscan_eps_m": self._dbscan_eps_m,
            "dbscan_min_points": self._dbscan_min_points,
            "icp_max_correspondence_distance_m": self._icp_max_correspondence_distance_m,
            "icp_fitness_threshold": self._icp_fitness_threshold,
            "save_intermediate_debug_clouds": self._save_intermediate_debug_clouds,
            "publish_debug_topics": self._publish_debug_topics,
            "decimation_magnitude": self._decimation_magnitude,
            "spatial_filter_magnitude": self._spatial_filter_magnitude,
            "spatial_filter_alpha": self._spatial_filter_alpha,
            "spatial_filter_delta": self._spatial_filter_delta,
            "temporal_filter_alpha": self._temporal_filter_alpha,
            "temporal_filter_delta": self._temporal_filter_delta,
            "hole_filling_mode": self._hole_filling_mode,
            "plane_ransac_iterations": self._plane_ransac_iterations,
        }

    def _publish_status(self, message: str) -> None:
        if message == self._last_status_message:
            return
        self._last_status_message = message
        msg = String()
        msg.data = message
        self._status_publisher.publish(msg)

    def _on_start_scan(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        success, message = self._request_scan_start(source="service")
        response.success = success
        response.message = message
        return response

    def _on_save_last_scan(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        with self._state_lock:
            saved_scan = self._last_saved_scan

        if saved_scan is None:
            response.success = False
            response.message = "no completed scan is available to save"
            return response

        scan_dir = self._make_scan_dir()
        metadata = copy.deepcopy(saved_scan.metadata)
        metadata["resaved_from"] = str(saved_scan.scan_dir)
        write_point_cloud(scan_dir / "final_object_cloud.ply", saved_scan.clean_cloud)
        write_point_cloud(
            scan_dir / "final_object_cloud_downsampled.ply",
            saved_scan.downsampled_cloud,
        )
        self._save_scan_images(
            scan_dir,
            saved_scan.color_image_bgr,
        )
        write_metadata_json(scan_dir / "metadata.json", metadata)
        self._publish_final_cloud(saved_scan.downsampled_cloud)
        response.success = True
        response.message = f"saved last scan copy to {scan_dir}"
        self._publish_status(response.message)
        return response

    def _request_scan_start(self, source: str) -> tuple[bool, str]:
        with self._state_lock:
            if self._processing:
                return False, "scan reconstruction already running"
            if self._scanning or self._pending_start:
                return False, "scan already active"
            if self._latest_frame is None:
                return False, "camera frames are not ready yet"
            self._pending_start = True

        message = f"burst scan requested from {source}"
        self._publish_status(message)
        self.get_logger().info(message)
        return True, message

    def _render_preview(self, frame: FrameBundle) -> None:
        preview = frame.color_image_bgr.copy()

        cv2.imshow(self._preview_window, preview)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("s"):
            self._request_scan_start(source="preview")
        elif key == ord("e"):
            with self._state_lock:
                if self._scanning:
                    self._stop_scan_early = True
            self._publish_status("early scan stop requested")
        elif key == ord("q"):
            self._shutdown_requested = True
            self._publish_status("shutdown requested from preview")
            rclpy.shutdown()

    def destroy_node(self) -> bool:
        self._keyboard_stop.set()
        if self._stdin_fd is not None and self._stdin_term_settings is not None:
            try:
                termios.tcsetattr(
                    self._stdin_fd, termios.TCSADRAIN, self._stdin_term_settings
                )
            except (termios.error, ValueError, OSError):
                pass
        if hasattr(self, "_pipeline"):
            try:
                self._pipeline.stop()
            except RuntimeError:
                pass
        if self._show_preview:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ObjectScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
