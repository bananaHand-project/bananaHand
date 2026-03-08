#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray


class FsrTestPublisher(Node):
    def __init__(self):
        super().__init__("fsr_test_publisher")

        self.declare_parameter("topic_name", "rx_force")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("value_min", 0)
        self.declare_parameter("value_max", 1200)
        self.declare_parameter("vector_length", 10)

        topic_name = str(self.get_parameter("topic_name").value)
        publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self.value_min = int(self.get_parameter("value_min").value)
        self.value_max = int(self.get_parameter("value_max").value)
        self.vector_length = max(5, int(self.get_parameter("vector_length").value))

        self.pub = self.create_publisher(UInt16MultiArray, topic_name, 10)
        self.phase = 0.0

        self.create_timer(1.0 / publish_hz, self._publish)
        self.get_logger().info(f"Publishing simulated force data on '{topic_name}'")

    def _wave(self, x: float) -> float:
        return 0.5 + 0.5 * math.sin(x)

    def _publish(self) -> None:
        self.phase += 0.14
        span = max(1, self.value_max - self.value_min)

        samples = []
        for i in range(self.vector_length):
            signal = self._wave(self.phase * (1.0 + 0.09 * i) + i * 0.65)
            value = int(self.value_min + signal * span)
            samples.append(max(0, min(65535, value)))

        msg = UInt16MultiArray()
        msg.data = samples
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FsrTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
