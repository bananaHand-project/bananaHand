"""ROS 2 node that chains scan outputs into ground removal and grasp classification."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix
from banana_interfaces.msg import GraspRecommendation
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class ScanGraspPipelineNode(Node):
    """Run the existing grasp pipeline on each completed scan directory."""

    def __init__(self) -> None:
        super().__init__("scan_grasp_pipeline_node")

        self.declare_parameter("scan_complete_topic", "/object_scan/completed_scan_dir")
        self.declare_parameter("result_topic", "/grasp_classification/recommendation")
        self.declare_parameter("ground_removed_suffix", "_ground_removed")
        self.declare_parameter("classifier_output_suffix", "_grasp")
        self.declare_parameter("opening_margin_m", 0.03)
        self.declare_parameter("max_hand_opening_m", 0.0)
        self.declare_parameter("small_object_max_span_m", 0.045)
        self.declare_parameter("tripod_object_max_span_m", 0.065)
        self.declare_parameter("power_grasp_min_span_m", 0.045)

        self._scan_complete_topic = str(
            self.get_parameter("scan_complete_topic").value
        ).strip()
        self._result_topic = str(self.get_parameter("result_topic").value).strip()
        self._ground_removed_suffix = str(
            self.get_parameter("ground_removed_suffix").value
        ).strip()
        self._classifier_output_suffix = str(
            self.get_parameter("classifier_output_suffix").value
        ).strip()
        self._opening_margin_m = float(self.get_parameter("opening_margin_m").value)
        self._max_hand_opening_m = float(
            self.get_parameter("max_hand_opening_m").value
        )
        self._small_object_max_span_m = float(
            self.get_parameter("small_object_max_span_m").value
        )
        self._tripod_object_max_span_m = float(
            self.get_parameter("tripod_object_max_span_m").value
        )
        self._power_grasp_min_span_m = float(
            self.get_parameter("power_grasp_min_span_m").value
        )

        package_prefix = Path(get_package_prefix("banana_grasp_classification"))
        self._executable_dir = package_prefix / "lib" / "banana_grasp_classification"

        result_qos = QoSProfile(depth=1)
        result_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._recommendation_publisher = self.create_publisher(
            GraspRecommendation,
            self._result_topic,
            result_qos,
        )
        self._scan_complete_subscription = self.create_subscription(
            String,
            self._scan_complete_topic,
            self._on_completed_scan_dir,
            10,
        )

        self.get_logger().info(
            "Listening for completed scans on "
            f"{self._scan_complete_topic} and publishing grasp recommendations on "
            f"{self._result_topic}"
        )

    def _on_completed_scan_dir(self, msg: String) -> None:
        scan_dir = Path(msg.data).expanduser()
        if not scan_dir.is_dir():
            self.get_logger().error(
                f"Completed scan directory does not exist: {scan_dir}"
            )
            return

        self.get_logger().info(f"Running grasp pipeline for scan: {scan_dir}")
        try:
            self._run_ground_plane_removal(scan_dir)
            self._run_grasp_classifier(scan_dir)
            result_path = self._find_result_json(scan_dir)
            recommendation = self._load_recommendation(result_path)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return

        self._recommendation_publisher.publish(recommendation)
        self.get_logger().info(
            "Published grasp recommendation: "
            f"{recommendation.selected_grip} "
            f"(recommended_opening_m={recommendation.recommended_opening_m:.4f})"
        )

    def _run_ground_plane_removal(self, scan_dir: Path) -> None:
        self._run_executable(
            executable_name="ground_plane_removal_node",
            parameters={
                "input_dir": scan_dir,
                "recursive_search": False,
                "output_suffix": self._ground_removed_suffix,
            },
        )

    def _run_grasp_classifier(self, scan_dir: Path) -> None:
        self._run_executable(
            executable_name="grasp_rule_classifier_node",
            parameters={
                "input_dir": scan_dir,
                "recursive_search": False,
                "ground_removed_suffix": self._ground_removed_suffix,
                "output_suffix": self._classifier_output_suffix,
                "opening_margin_m": self._opening_margin_m,
                "max_hand_opening_m": self._max_hand_opening_m,
                "small_object_max_span_m": self._small_object_max_span_m,
                "tripod_object_max_span_m": self._tripod_object_max_span_m,
                "power_grasp_min_span_m": self._power_grasp_min_span_m,
            },
        )

    def _run_executable(
        self,
        executable_name: str,
        parameters: dict[str, object],
    ) -> None:
        executable_path = self._executable_dir / executable_name
        if not executable_path.is_file():
            raise RuntimeError(f"Executable not found: {executable_path}")

        cmd = [str(executable_path), "--ros-args"]
        for name, value in parameters.items():
            cmd.extend(["-p", f"{name}:={self._format_ros_value(value)}"])

        self.get_logger().info(f"Starting {executable_name}")
        completed = subprocess.run(cmd, check=False)
        if completed.returncode != 0:
            raise RuntimeError(
                f"{executable_name} failed for {parameters['input_dir']} "
                f"with exit code {completed.returncode}"
            )

    def _find_result_json(self, scan_dir: Path) -> Path:
        pattern = (
            f"*{self._ground_removed_suffix}{self._classifier_output_suffix}.json"
        )
        matches = sorted(scan_dir.glob(pattern))
        if not matches:
            raise RuntimeError(
                f"No grasp result JSON matching {pattern} was found in {scan_dir}"
            )
        if len(matches) > 1:
            listed = ", ".join(str(path) for path in matches)
            raise RuntimeError(
                "Found multiple grasp result JSON files. "
                f"Expected exactly one in {scan_dir}. Matches: {listed}"
            )
        return matches[0]

    def _load_recommendation(self, result_path: Path) -> GraspRecommendation:
        try:
            payload = json.loads(result_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            raise RuntimeError(
                f"Failed to parse grasp result JSON {result_path}: {exc}"
            ) from exc

        try:
            selected_grip = str(payload["selected_grip"])
            recommended_opening_m = float(payload["recommended_opening_m"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(
                "Grasp result JSON is missing required fields "
                f"'selected_grip' or 'recommended_opening_m': {result_path}"
            ) from exc

        message = GraspRecommendation()
        message.selected_grip = selected_grip
        message.recommended_opening_m = recommended_opening_m
        return message

    def _format_ros_value(self, value: object) -> str:
        if isinstance(value, bool):
            return "true" if value else "false"
        if isinstance(value, Path):
            return str(value)
        return str(value)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ScanGraspPipelineNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

