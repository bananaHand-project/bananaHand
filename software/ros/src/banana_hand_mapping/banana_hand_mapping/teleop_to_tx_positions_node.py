#!/usr/bin/env python3
"""Convert teleop joint ratios to uint16 tx positions for the serial bridge."""

from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
from trajectory_msgs.msg import JointTrajectory


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class TeleopToTxPositionsNode(Node):
    def __init__(self) -> None:
        super().__init__("teleop_to_tx_positions")

        self.declare_parameter("input_topic", "/hand/teleop_joint_trajectory")
        self.declare_parameter("output_topic", "/tx_positions")
        self.declare_parameter("input_min_ratio", 0.0)
        self.declare_parameter("input_max_ratio", 1.0)
        self.declare_parameter("adc_min", 0)
        self.declare_parameter("adc_max", 4095)
        self.declare_parameter("source_indices", [0, 1, 2, 3, 4, 5, -1, -1])
        self.declare_parameter("fill_ratio", 0.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._input_min = float(self.get_parameter("input_min_ratio").value)
        self._input_max = float(self.get_parameter("input_max_ratio").value)
        self._adc_min = int(self.get_parameter("adc_min").value)
        self._adc_max = int(self.get_parameter("adc_max").value)
        self._source_indices = [
            int(v) for v in self.get_parameter("source_indices").value
        ]
        self._fill_ratio = float(self.get_parameter("fill_ratio").value)

        if self._input_max <= self._input_min:
            self.get_logger().warn(
                "input_max_ratio must be > input_min_ratio; forcing 0..1"
            )
            self._input_min = 0.0
            self._input_max = 1.0
        if self._adc_max < self._adc_min:
            self.get_logger().warn(
                "adc_max must be >= adc_min; forcing 0..4095"
            )
            self._adc_min = 0
            self._adc_max = 4095
        if len(self._source_indices) != 8:
            self.get_logger().warn(
                "source_indices must be length 8; forcing default mapping"
            )
            self._source_indices = [0, 1, 2, 3, 4, 5, -1, -1]

        self._sub = self.create_subscription(
            JointTrajectory, input_topic, self._on_teleop, 10
        )
        self._pub = self.create_publisher(UInt16MultiArray, output_topic, 10)

        self.get_logger().info(
            f"Mapping {input_topic} -> {output_topic} with ADC range "
            f"[{self._adc_min}, {self._adc_max}]"
        )

    def _ratio_to_adc(self, ratio: float) -> int:
        normalized = (ratio - self._input_min) / (
            self._input_max - self._input_min
        )
        normalized = _clamp(normalized, 0.0, 1.0)
        adc = self._adc_min + normalized * (self._adc_max - self._adc_min)
        return int(round(adc))

    def _resolve_ratio(self, source: List[float], src_idx: int) -> float:
        if src_idx < 0 or src_idx >= len(source):
            return self._fill_ratio
        return float(source[src_idx])

    def _on_teleop(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return

        src_positions = list(msg.points[0].positions)
        out_values: List[int] = []

        for src_idx in self._source_indices:
            ratio = self._resolve_ratio(src_positions, src_idx)
            out_values.append(self._ratio_to_adc(ratio))

        out_msg = UInt16MultiArray()
        out_msg.data = out_values
        self._pub.publish(out_msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TeleopToTxPositionsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
