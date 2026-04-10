#!/usr/bin/env python3
"""Convert teleop joint ratios to uint16 tx positions for the serial bridge."""

from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt16MultiArray
from trajectory_msgs.msg import JointTrajectory

POS_INDEX_1 = 0
POS_MIDDLE = 1
POS_RING = 2
POS_PINKY = 3
POS_THUMB_1 = 4
POS_THUMB_2 = 5
POS_INDEX_2 = 6
POS_THUMB_3 = 7

POSITION_NAMES = [
    "index_1",
    "middle",
    "ring",
    "pinky",
    "thumb_1",
    "thumb_2",
    "index_2",
    "thumb_3",
]

SAFE_MODE_THRESHOLD = 1500
SAFE_RETRACT_THRESHOLD = 2000


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _is_moving_in(new_value: int, old_value: int) -> bool:
    return new_value > old_value


class TeleopToTxPositionsNode(Node):
    def __init__(self) -> None:
        super().__init__("teleop_to_tx_positions")

        self.declare_parameter("input_topic", "/hand/teleop_joint_trajectory")
        self.declare_parameter("output_topic", "/tx_positions")
        self.declare_parameter("input_min_ratio", 0.0)
        self.declare_parameter("input_max_ratio", 1.0)
        self.declare_parameter("min_motor_positions", [0, 0, 0, 0, 0, 0, 0, 0])
        self.declare_parameter(
            "max_motor_positions", [4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095]
        )
        self.declare_parameter("source_indices", [0, 1, 2, 3, 4, 5, -1, -1])
        self.declare_parameter("fill_ratio", 0.0)
        self.declare_parameter("safe_mode_enabled", False)
        self.declare_parameter("mode_topic", "/tx_positions_mode")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._input_min = float(self.get_parameter("input_min_ratio").value)
        self._input_max = float(self.get_parameter("input_max_ratio").value)
        self._min_motor_positions = [
            int(v) for v in self.get_parameter("min_motor_positions").value
        ]
        self._max_motor_positions = [
            int(v) for v in self.get_parameter("max_motor_positions").value
        ]
        self._source_indices = [
            int(v) for v in self.get_parameter("source_indices").value
        ]
        self._fill_ratio = float(self.get_parameter("fill_ratio").value)
        self._safe_mode_enabled = bool(self.get_parameter("safe_mode_enabled").value)
        mode_topic = str(self.get_parameter("mode_topic").value)
        self._last_published = [0] * len(POSITION_NAMES)

        if self._input_max <= self._input_min:
            self.get_logger().warn(
                "input_max_ratio must be > input_min_ratio; forcing 0..1"
            )
            self._input_min = 0.0
            self._input_max = 1.0
        if len(self._min_motor_positions) != len(POSITION_NAMES):
            self.get_logger().warn(
                f"min_motor_positions must be length {len(POSITION_NAMES)}; forcing all zeros"
            )
            self._min_motor_positions = [0] * len(POSITION_NAMES)
        if len(self._max_motor_positions) != len(POSITION_NAMES):
            self.get_logger().warn(
                f"max_motor_positions must be length {len(POSITION_NAMES)}; forcing all 4095"
            )
            self._max_motor_positions = [4095] * len(POSITION_NAMES)
        if len(self._source_indices) != len(POSITION_NAMES):
            self.get_logger().warn(
                f"source_indices must be length {len(POSITION_NAMES)}; forcing default mapping"
            )
            self._source_indices = [0, 1, 2, 3, 4, 5, -1, -1]
        for idx, (min_pos, max_pos) in enumerate(
            zip(self._min_motor_positions, self._max_motor_positions)
        ):
            if max_pos < min_pos:
                self.get_logger().warn(
                    f"motor {POSITION_NAMES[idx]} (index {idx}): max_motor_positions must be >= min_motor_positions; forcing 0..4095"
                )
                self._min_motor_positions[idx] = 0
                self._max_motor_positions[idx] = 4095

        self._sub = self.create_subscription(
            JointTrajectory, input_topic, self._on_teleop, 10
        )
        self._mode_sub = self.create_subscription(
            String, mode_topic, self._on_mode_cmd, 10
        )
        self._pub = self.create_publisher(UInt16MultiArray, output_topic, 10)

        self.get_logger().info(
            f"Mapping {input_topic} -> {output_topic} with positions "
            f"{list(enumerate(POSITION_NAMES))}, mins={self._min_motor_positions}, "
            f"maxes={self._max_motor_positions}, safe_mode={self._safe_mode_enabled}, "
            f"mode_topic={mode_topic}"
        )

    def _ratio_to_adc(self, ratio: float, output_idx: int) -> int:
        normalized = (ratio - self._input_min) / (
            self._input_max - self._input_min
        )
        normalized = _clamp(normalized, 0.0, 1.0)
        adc = self._min_motor_positions[output_idx] + normalized * (
            self._max_motor_positions[output_idx]
            - self._min_motor_positions[output_idx]
        )
        return int(round(adc))

    def _resolve_ratio(self, source: List[float], src_idx: int) -> float:
        if src_idx < 0 or src_idx >= len(source):
            return self._fill_ratio
        return float(source[src_idx])

    def _on_mode_cmd(self, msg: String) -> None:
        mode = msg.data.strip().lower()
        if mode == "safe":
            self._safe_mode_enabled = True
        elif mode == "regular":
            self._safe_mode_enabled = False
        else:
            self.get_logger().warn(
                f"Unknown mode '{msg.data}'. Expected 'safe' or 'regular'."
            )
            return

        self.get_logger().info(
            f"Switched teleop safety mode to {'safe' if self._safe_mode_enabled else 'regular'}"
        )

    def _apply_safe_mode(self, values: List[int]) -> List[int]:
        if not self._safe_mode_enabled:
            return values

        out_values = list(values)
        last_index = self._last_published[POS_INDEX_1]
        last_middle = self._last_published[POS_MIDDLE]
        last_thumb = self._last_published[POS_THUMB_1]

        index_moving_in = _is_moving_in(out_values[POS_INDEX_1], last_index)
        middle_moving_in = _is_moving_in(out_values[POS_MIDDLE], last_middle)
        thumb_moving_in = _is_moving_in(out_values[POS_THUMB_1], last_thumb)

        thumb_not_clear = last_thumb > SAFE_RETRACT_THRESHOLD
        index_not_clear = last_index > SAFE_RETRACT_THRESHOLD
        middle_not_clear = last_middle > SAFE_RETRACT_THRESHOLD

        if thumb_not_clear:
            if index_moving_in:
                out_values[POS_INDEX_1] = last_index
            if middle_moving_in:
                out_values[POS_MIDDLE] = last_middle

        if index_not_clear or middle_not_clear:
            if thumb_moving_in:
                out_values[POS_THUMB_1] = last_thumb

        last_index_or_middle_past = (
            last_index > SAFE_MODE_THRESHOLD
            or last_middle > SAFE_MODE_THRESHOLD
        )
        last_thumb_past = last_thumb > SAFE_MODE_THRESHOLD
        index_or_middle_past = (
            out_values[POS_INDEX_1] > SAFE_MODE_THRESHOLD
            or out_values[POS_MIDDLE] > SAFE_MODE_THRESHOLD
        )
        thumb_past = out_values[POS_THUMB_1] > SAFE_MODE_THRESHOLD

        if last_thumb_past:
            out_values[POS_INDEX_1] = min(out_values[POS_INDEX_1], SAFE_MODE_THRESHOLD)
            out_values[POS_MIDDLE] = min(out_values[POS_MIDDLE], SAFE_MODE_THRESHOLD)
        elif last_index_or_middle_past:
            out_values[POS_THUMB_1] = min(out_values[POS_THUMB_1], SAFE_MODE_THRESHOLD)
        elif index_or_middle_past:
            out_values[POS_THUMB_1] = min(out_values[POS_THUMB_1], SAFE_MODE_THRESHOLD)
        elif thumb_past:
            out_values[POS_INDEX_1] = min(out_values[POS_INDEX_1], SAFE_MODE_THRESHOLD)
            out_values[POS_MIDDLE] = min(out_values[POS_MIDDLE], SAFE_MODE_THRESHOLD)

        return out_values

    def _on_teleop(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return

        src_positions = list(msg.points[0].positions)
        out_values: List[int] = []

        for output_idx, src_idx in enumerate(self._source_indices):
            ratio = self._resolve_ratio(src_positions, src_idx)
            out_values.append(self._ratio_to_adc(ratio, output_idx))

        out_values = self._apply_safe_mode(out_values)

        out_msg = UInt16MultiArray()
        out_msg.data = out_values
        self._pub.publish(out_msg)
        self._last_published = list(out_values)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TeleopToTxPositionsNode()
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
