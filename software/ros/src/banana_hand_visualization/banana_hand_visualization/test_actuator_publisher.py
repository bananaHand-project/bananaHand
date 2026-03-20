#!/usr/bin/env python3
"""Publish a simple 6-channel actuator command test pattern for MuJoCo."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


ACTUATOR_COUNT = 6


class TestActuatorPublisher(Node):
    def __init__(self) -> None:
        super().__init__("banana_hand_test_actuator_publisher")

        self.declare_parameter("command_topic", "/banana_hand/actuator_commands")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("period_s", 4.0)

        command_topic = str(self.get_parameter("command_topic").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._period_s = float(self.get_parameter("period_s").value)
        self._phase = 0.0

        self._pub = self.create_publisher(Float32MultiArray, command_topic, 10)
        self.create_timer(1.0 / publish_rate_hz, self._on_timer)

    def _on_timer(self) -> None:
        msg = Float32MultiArray()
        values = []
        for actuator_id in range(ACTUATOR_COUNT):
            phase = self._phase + actuator_id * 0.55
            values.append(0.5 * (1.0 + math.sin(phase)))

        msg.data = values
        self._pub.publish(msg)
        self._phase += (2.0 * math.pi) / max(self._period_s * 30.0, 1.0)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TestActuatorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
