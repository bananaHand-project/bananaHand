#!/usr/bin/env python3
"""Minimal manual /tx_positions publisher."""

import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray

# Matches firmware/experiments/uart_g4_combo/src/control_config.rs
IDX_INDEX_1 = 0
IDX_MIDDLE = 1
IDX_RING = 2
IDX_PINKY = 3
IDX_THUMB_1 = 4
IDX_THUMB_2 = 5
IDX_INDEX_2 = 6
IDX_THUMB_3 = 7


class ManualControlNode(Node):
    def __init__(self) -> None:
        super().__init__("manual_control")
        self._pub = self.create_publisher(UInt16MultiArray, "/tx_positions", 10)
        self.get_logger().info("manual_control ready: publishing to /tx_positions")

    def publish_positions(self, positions: List[int]) -> None:
        # if len(positions) != 8:
        #     self.get_logger().warn(f"Expected 8 positions, got {len(positions)}")
        #     return

        clamped = [max(0, min(4095, int(v))) for v in positions]
        msg = UInt16MultiArray()
        msg.data = clamped
        self._pub.publish(msg)

    def run(self) -> None:        
        self.get_logger().info("started")
        while rclpy.ok():
            # regular (min values)
            cmd = [500, 500, 500, 500, 300, 150, 0, 0]
            self.publish_positions(cmd)
            time.sleep(2)

            # cylindrical
            cmd = [1800, 1800, 1800, 1800, 1500, 4000,0,0]
            self.publish_positions(cmd)
            time.sleep(2)

            # # spherical
            cmd = [2500, 1800, 1800, 3000, 1000, 4000,0,0]
            self.publish_positions(cmd)
            time.sleep(2)

            # pinch
            cmd = [1700, 0, 0, 0, 1500, 2000,0,0]
            self.publish_positions(cmd)
            time.sleep(2)

            # tripod
            cmd = [1800, 1800, 0, 0, 1500, 3000,0,0]
            self.publish_positions(cmd)
            time.sleep(2)

            # # hook
            cmd = [4000, 4000, 4000, 4000, 0, 0,0,0]
            self.publish_positions(cmd)
            time.sleep(2)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ManualControlNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
