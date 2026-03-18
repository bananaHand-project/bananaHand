#!/usr/bin/env python3
"""Bridge raw force readings to Foxglove-friendly gauge topics."""

from __future__ import annotations

from typing import Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, UInt16MultiArray


FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]


class ForceGaugeBridge(Node):
    def __init__(self) -> None:
        super().__init__("banana_hand_force_gauge_bridge")

        self.declare_parameter("input_topic", "/rx_force")
        self.declare_parameter("state_topic", "/banana_hand/force_state")
        self.declare_parameter("finger_indices", [0, 1, 2, 3, 4])
        self.declare_parameter("topic_prefix", "/banana_hand/force")
        self.declare_parameter("scale", 1.0)

        input_topic = str(self.get_parameter("input_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        topic_prefix = str(self.get_parameter("topic_prefix").value)
        self._finger_indices = [int(v) for v in self.get_parameter("finger_indices").value]
        self._scale = float(self.get_parameter("scale").value)

        if len(self._finger_indices) != len(FINGER_NAMES):
            raise ValueError("finger_indices must have 5 entries")

        self._state_pub = self.create_publisher(JointState, state_topic, 10)
        self._finger_pubs = {
            finger_name: self.create_publisher(Float32, f"{topic_prefix}/{finger_name}", 10)
            for finger_name in FINGER_NAMES
        }
        self.create_subscription(UInt16MultiArray, input_topic, self._on_force, 10)

    def _value_at(self, data: Sequence[int], index: int) -> float:
        if index < 0 or index >= len(data):
            return 0.0
        return float(data[index]) * self._scale

    def _on_force(self, msg: UInt16MultiArray) -> None:
        values = [
            self._value_at(msg.data, self._finger_indices[finger_idx])
            for finger_idx in range(len(FINGER_NAMES))
        ]

        state_msg = JointState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.name = list(FINGER_NAMES)
        state_msg.position = values
        self._state_pub.publish(state_msg)

        for finger_name, value in zip(FINGER_NAMES, values):
            out_msg = Float32()
            out_msg.data = float(value)
            self._finger_pubs[finger_name].publish(out_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ForceGaugeBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
