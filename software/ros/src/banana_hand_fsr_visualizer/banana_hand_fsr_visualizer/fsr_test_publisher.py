#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16MultiArray


class FsrTestPublisher(Node):
    def __init__(self):
        super().__init__("fsr_test_publisher")

        self.declare_parameter("topic_name", "rx_force")
        self.declare_parameter("position_topic_name", "rx_positions")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("value_min", 0)
        self.declare_parameter("value_max", 3600)
        self.declare_parameter("position_min", 400)
        self.declare_parameter("position_max", 3200)
        self.declare_parameter("vector_length", 10)
        self.declare_parameter("position_length", 8)

        self.force_topic_name = str(self.get_parameter("topic_name").value)
        self.position_topic_name = str(self.get_parameter("position_topic_name").value)
        publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self.value_min = int(self.get_parameter("value_min").value)
        self.value_max = int(self.get_parameter("value_max").value)
        self.position_min = int(self.get_parameter("position_min").value)
        self.position_max = int(self.get_parameter("position_max").value)
        self.vector_length = max(10, int(self.get_parameter("vector_length").value))
        self.position_length = max(6, int(self.get_parameter("position_length").value))

        self.force_pub = self.create_publisher(UInt16MultiArray, self.force_topic_name, 10)
        self.position_pub = self.create_publisher(JointState, self.position_topic_name, 10)
        self.phase = 0.0

        self.create_timer(1.0 / publish_hz, self._publish)
        self.get_logger().info(
            f"Publishing simulated force data on '{self.force_topic_name}' and position data on '{self.position_topic_name}'"
        )

    def _wave(self, x: float) -> float:
        return 0.5 + 0.5 * math.sin(x)

    def _publish(self) -> None:
        self.phase += 0.14

        force_span = max(1, self.value_max - self.value_min)
        force_samples = []
        for index in range(self.vector_length):
            signal = 0.18 + 0.82 * self._wave(self.phase * (1.0 + 0.09 * index) + index * 0.65)
            value = int(self.value_min + signal * force_span)
            force_samples.append(max(0, min(65535, value)))

        force_msg = UInt16MultiArray()
        force_msg.data = force_samples
        self.force_pub.publish(force_msg)

        position_span = max(1, self.position_max - self.position_min)
        joint_state = JointState()
        joint_state.name = [f"joint_{index}" for index in range(self.position_length)]
        joint_state.position = [
            float(self.position_min + self._wave(self.phase * (0.82 + index * 0.06) + index * 0.45) * position_span)
            for index in range(self.position_length)
        ]
        self.position_pub.publish(joint_state)


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
