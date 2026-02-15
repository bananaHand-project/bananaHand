"""Placeholder ROS 2 node for grasp strategy prediction."""

from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
from std_msgs.msg import String


class GraspStrategyNode(Node):
    """Scaffold node for future grasp type and confidence prediction."""

    def __init__(self) -> None:
        super().__init__("grasp_strategy_node")

        self.declare_parameter("input_object_topic", "/object/point_cloud")
        self.declare_parameter("output_grip_type_topic", "/grasp/grip_type")
        self.declare_parameter("output_confidence_topic", "/grasp/grip_confidence")

        input_topic = str(self.get_parameter("input_object_topic").value)
        label_topic = str(self.get_parameter("output_grip_type_topic").value)
        confidence_topic = str(self.get_parameter("output_confidence_topic").value)

        self._object_sub = self.create_subscription(
            PointCloud2, input_topic, self._on_object_representation, 10
        )
        self._label_pub = self.create_publisher(String, label_topic, 10)
        self._confidence_pub = self.create_publisher(Float32, confidence_topic, 10)

        self.get_logger().info(
            "TODO: Implement grasp strategy prediction model inference and publishing."
        )

    def _on_object_representation(self, _msg: PointCloud2) -> None:
        # Intentionally left as TODO-only scaffold.
        return


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = GraspStrategyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
