"""Placeholder ROS 2 node for MuJoCo rollout bridging."""

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import String


class SimBridgeNode(Node):
    """Scaffold node for future simulation rollout triggering and reporting."""

    def __init__(self) -> None:
        super().__init__("sim_bridge_node")

        self.declare_parameter("requested_grip_topic", "/sim/requested_grip_type")
        self.declare_parameter("success_metric_topic", "/sim/success_rate")
        self.declare_parameter("artifact_path_topic", "/sim/artifact_path")

        requested_grip_topic = str(self.get_parameter("requested_grip_topic").value)
        success_metric_topic = str(self.get_parameter("success_metric_topic").value)
        artifact_path_topic = str(self.get_parameter("artifact_path_topic").value)

        self._request_sub = self.create_subscription(
            String, requested_grip_topic, self._on_requested_grip, 10
        )
        self._success_pub = self.create_publisher(Float32, success_metric_topic, 10)
        self._artifact_pub = self.create_publisher(String, artifact_path_topic, 10)

        self.get_logger().info(
            "TODO: Implement MuJoCo rollout bridge and metrics reporting."
        )

    def _on_requested_grip(self, _msg: String) -> None:
        # Intentionally left as TODO-only scaffold.
        return


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
