#!/usr/bin/env python3
import struct
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

import serial


START_BYTE = 0xFF
END_BYTE = 0xFE
MSG_TYPE_POSITION = 0x01


def checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def build_frame(msg_type: int, positions: List[float]) -> bytes:
    # positions: list of 8 floats
    payload = b"".join(struct.pack("<f", float(p)) for p in positions)
    length = len(payload)
    chk = checksum(payload)
    return bytes([START_BYTE, msg_type, length]) + payload + bytes([chk, END_BYTE])


def parse_frame(frame: bytes) -> Optional[List[float]]:
    if len(frame) < 6:
        return None
    if frame[0] != START_BYTE or frame[-1] != END_BYTE:
        return None
    msg_type = frame[1]
    if msg_type != MSG_TYPE_POSITION:
        return None

    length = frame[2]
    if 3 + length + 2 != len(frame):
        return None

    payload = frame[3:3 + length]
    chk = frame[3 + length]
    if checksum(payload) != chk:
        return None

    if length % 4 != 0:
        return None

    positions = [struct.unpack("<f", payload[i:i + 4])[0] for i in range(0, length, 4)]
    return positions


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__("banana_serial_bridge")

        # Parameters
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("timeout_s", 0.02)
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("joint_names", [f"joint_{i}" for i in range(8)])

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.timeout_s = float(self.get_parameter("timeout_s").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.joint_names = list(self.get_parameter("joint_names").value)

        if len(self.joint_names) != 8:
            self.get_logger().warn("joint_names is not length 8; forcing 8 default names")
            self.joint_names = [f"joint_{i}" for i in range(8)]

        # ROS interfaces
        self.pub_js = self.create_publisher(JointState, "rx_positions", 10)
        self.sub_cmd = self.create_subscription(Float32MultiArray, "tx_positions", self.on_cmd, 10)

        # Serial
        self.ser = serial.Serial(self.port, baudrate=self.baud, timeout=self.timeout_s)
        self.get_logger().info(f"Opened serial {self.port} @ {self.baud}")

        # Buffers/state
        self._rx_buf = bytearray()
        self._last_cmd: Optional[List[float]] = None

        # Timers
        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self.poll_serial)

    def destroy_node(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()

    def on_cmd(self, msg: Float32MultiArray):
        if len(msg.data) != 8:
            self.get_logger().warn(f"tx_positions must have 8 floats; got {len(msg.data)}")
            return
        positions = [float(x) for x in msg.data]
        self._last_cmd = positions

        frame = build_frame(MSG_TYPE_POSITION, positions)
        try:
            self.ser.write(frame)
            print("Written to serial")

            self.ser.flush()
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def poll_serial(self):
        # Read whatever is available, append to buffer
        try:
            n = self.ser.in_waiting
            if n > 0:
                self._rx_buf += self.ser.read(n)
        except Exception as e:
            self.get_logger().error(f"Serial read failed: {e}")
            return

        # Try extracting frames from stream
        while True:
            frame = self._extract_one_frame()
            if frame is None:
                break

            positions = parse_frame(frame)
            if positions is None or len(positions) < 1:
                continue

            # Publish as JointState (use first 8 if more)
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = self.joint_names
            js.position = positions[:8] + ([0.0] * max(0, 8 - len(positions)))
            self.pub_js.publish(js)

    def _extract_one_frame(self) -> Optional[bytes]:
        """
        Stream parser:
        - find START_BYTE
        - need at least 3 bytes for header
        - use LEN to know total size: 3 + LEN + 2
        - verify END_BYTE before returning
        """
        buf = self._rx_buf
        if len(buf) < 3:
            return None

        # Find start
        try:
            start_idx = buf.index(START_BYTE)
        except ValueError:
            self._rx_buf.clear()
            return None

        if start_idx > 0:
            del buf[:start_idx]

        if len(buf) < 3:
            return None

        length = buf[2]
        total = 3 + length + 2
        if len(buf) < total:
            return None

        candidate = bytes(buf[:total])
        del buf[:total]

        # Quick end check; parse_frame will do full validation
        if candidate[-1] != END_BYTE:
            return None

        return candidate


def main():
    rclpy.init()
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
