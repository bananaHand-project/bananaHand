#!/usr/bin/env python3
"""Publish synthetic force sensor values for Foxglove gauge testing."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray


FORCE_COUNT = 10


class TestForcePublisher(Node):
    def __init__(self) -> None:
        super().__init__("banana_hand_test_force_publisher")

        self.declare_parameter("output_topic", "/rx_force")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("period_s", 3.0)
        self.declare_parameter("max_value", 1200.0)

        output_topic = str(self.get_parameter("output_topic").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._period_s = float(self.get_parameter("period_s").value)
        self._max_value = float(self.get_parameter("max_value").value)
        self._phase = 0.0

        self._pub = self.create_publisher(UInt16MultiArray, output_topic, 10)
        self.create_timer(1.0 / publish_rate_hz, self._on_timer)

    def _on_timer(self) -> None:
        msg = UInt16MultiArray()
        values = []
        for sensor_idx in range(FORCE_COUNT):
            phase = self._phase + sensor_idx * 0.45
            value = 0.5 * (1.0 + math.sin(phase))
            values.append(int(round(self._max_value * value)))

        msg.data = values
        self._pub.publish(msg)
        self._phase += (2.0 * math.pi) / max(self._period_s * 20.0, 1.0)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TestForcePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
