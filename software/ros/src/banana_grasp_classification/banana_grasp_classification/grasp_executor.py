"""ROS 2 node that converts grasp recommendations into gated /tx_positions commands."""

from __future__ import annotations

import math
from typing import Optional

from banana_interfaces.msg import GraspRecommendation
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Empty, UInt16MultiArray

ADC_MIN = 0
ADC_MAX = 4095
OPENING_FULLY_OPEN_M = 0.18
OPENING_FULLY_CLOSED_M = 0.06
THUMB_REVOLVE_INDEX = 5

# CAN CHANGE ON THE FLY
OPEN_COMMAND = [500, 500, 500, 500, 300, 150, 0, 0]
GRIP_COMMANDS = {
    "cylindrical": [1800, 1800, 1800, 1800, 1500, 4000, 0, 0],
    "spherical": [3000, 2300, 2300, 4000, 2000, 4000, 0, 0],
    "pinch": [1700, 0, 0, 0, 1500, 2000, 0, 0],
    "tripod": [1800, 1800, 0, 0, 1500, 3000, 0, 0],
    "hook": [4000, 4000, 4000, 4000, 0, 0, 0, 0],
}


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class GraspExecutorNode(Node):
    """Cache grasp recommendations and execute them only on explicit triggers."""

    def __init__(self) -> None:
        super().__init__("grasp_executor")

        self.declare_parameter(
            "input_topic", "/grasp_classification/recommendation"
        )
        self.declare_parameter(
            "execute_topic", "/grasp_classification/exeucte_grasp"
        )
        self.declare_parameter("release_topic", "/grasp_classification/release")
        self.declare_parameter("output_topic", "/tx_positions")
        self.declare_parameter("fully_open_opening_m", OPENING_FULLY_OPEN_M)
        self.declare_parameter("fully_closed_opening_m", OPENING_FULLY_CLOSED_M)

        self._input_topic = str(self.get_parameter("input_topic").value).strip()
        self._execute_topic = str(self.get_parameter("execute_topic").value).strip()
        self._release_topic = str(self.get_parameter("release_topic").value).strip()
        self._output_topic = str(self.get_parameter("output_topic").value).strip()
        self._fully_open_opening_m = float(
            self.get_parameter("fully_open_opening_m").value
        )
        self._fully_closed_opening_m = float(
            self.get_parameter("fully_closed_opening_m").value
        )
        self._pending_command: list[int] | None = None
        self._pending_grip: str = ""
        self._pending_opening_m: float = 0.0

        if self._fully_open_opening_m <= self._fully_closed_opening_m:
            self.get_logger().warn(
                "fully_open_opening_m must be greater than fully_closed_opening_m; "
                "falling back to 0.15 m and 0.03 m"
            )
            self._fully_open_opening_m = OPENING_FULLY_OPEN_M
            self._fully_closed_opening_m = OPENING_FULLY_CLOSED_M

        recommendation_qos = QoSProfile(depth=1)
        recommendation_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._recommendation_subscription = self.create_subscription(
            GraspRecommendation,
            self._input_topic,
            self._on_recommendation,
            recommendation_qos,
        )
        self._execute_subscription = self.create_subscription(
            Empty,
            self._execute_topic,
            self._on_execute,
            10,
        )
        self._release_subscription = self.create_subscription(
            Empty,
            self._release_topic,
            self._on_release,
            10,
        )
        self._publisher = self.create_publisher(
            UInt16MultiArray,
            self._output_topic,
            10,
        )

        self.get_logger().info(
            f"Listening for recommendations on {self._input_topic}, execute "
            f"triggers on {self._execute_topic}, release triggers on "
            f"{self._release_topic}, and publishing tx positions on "
            f"{self._output_topic}; open={self._fully_open_opening_m:.3f} m, "
            f"closed={self._fully_closed_opening_m:.3f} m"
        )

    def _compute_close_fraction(self, recommended_opening_m: float) -> float:
        if not math.isfinite(recommended_opening_m):
            return 0.0

        normalized = (
            self._fully_open_opening_m - recommended_opening_m
        ) / (
            self._fully_open_opening_m - self._fully_closed_opening_m
        )
        return _clamp(normalized, 0.0, 1.0)

    def _build_command(
        self,
        selected_grip: str,
        recommended_opening_m: float,
    ) -> list[int] | None:
        grip_key = selected_grip.strip().lower()
        grip_command = GRIP_COMMANDS.get(grip_key)
        if grip_command is None:
            return None

        close_fraction = self._compute_close_fraction(recommended_opening_m)
        command: list[int] = []

        for index, (open_value, grip_value) in enumerate(
            zip(OPEN_COMMAND, grip_command)
        ):
            if index == THUMB_REVOLVE_INDEX:
                command.append(grip_value)
                continue

            interpolated = open_value + close_fraction * (grip_value - open_value)
            command.append(int(round(_clamp(interpolated, ADC_MIN, ADC_MAX))))

        return command

    def _on_recommendation(self, msg: GraspRecommendation) -> None:
        command = self._build_command(
            selected_grip=msg.selected_grip,
            recommended_opening_m=msg.recommended_opening_m,
        )
        if command is None:
            self._clear_pending_command()
            supported = ", ".join(sorted(GRIP_COMMANDS))
            self.get_logger().warn(
                f"Unsupported grip '{msg.selected_grip}'. Supported grips: {supported}"
            )
            return

        self._pending_command = command
        self._pending_grip = msg.selected_grip.strip().lower()
        self._pending_opening_m = float(msg.recommended_opening_m)

        self.get_logger().info(
            "Cached tx positions for "
            f"{self._pending_grip} "
            f"(recommended_opening_m={self._pending_opening_m:.4f} m): "
            f"{command}"
        )

    def _publish_command(self, command: list[int], reason: str) -> None:
        out_msg = UInt16MultiArray()
        out_msg.data = command
        self._publisher.publish(out_msg)
        self.get_logger().info(f"{reason}: {command}")

    def _clear_pending_command(self) -> None:
        self._pending_command = None
        self._pending_grip = ""
        self._pending_opening_m = 0.0

    def _on_execute(self, _: Empty) -> None:
        if self._pending_command is None:
            self.get_logger().warn(
                "Received execute trigger, but no grasp recommendation is cached"
            )
            return

        self._publish_command(
            self._pending_command,
            "Executed cached grasp "
            f"{self._pending_grip} "
            f"(recommended_opening_m={self._pending_opening_m:.4f} m)",
        )

    def _on_release(self, _: Empty) -> None:
        self._publish_command(OPEN_COMMAND, "Released grasp and returned to open pose")
        self._clear_pending_command()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = GraspExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
