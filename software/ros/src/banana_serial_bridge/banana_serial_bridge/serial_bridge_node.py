#!/usr/bin/env python3
import struct
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16MultiArray

import serial
from cobs import cobs  # pip install cobs


COBS_DELIM = b"\x00"
MSG_TYPE_POSITION = 0x01
MSG_TYPE_TELEMETRY = 0x03
POSITION_COUNT = 8
FORCE_COUNT = 10
POSITION_LEN = POSITION_COUNT * 2
FORCE_LEN = FORCE_COUNT * 2

MAX_ENC_FRAME = 512  # cap to avoid runaway buffer on noise


def checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def build_frame(msg_type: int, positions: List[int]) -> bytes:
    if len(positions) != POSITION_COUNT:
        raise ValueError(f"expected {POSITION_COUNT} uint16 values, got {len(positions)}")

    payload = b"".join(struct.pack("<H", int(p)) for p in positions)
    if len(payload) != POSITION_LEN:
        raise ValueError(f"payload must be {POSITION_LEN} bytes, got {len(payload)}")

    chk = checksum(payload)

    # body: [type][payload][chk]
    body = bytes([msg_type]) + payload + bytes([chk])

    return cobs.encode(body) + COBS_DELIM


def parse_u16_le(payload: bytes) -> List[int]:
    return [struct.unpack("<H", payload[i : i + 2])[0] for i in range(0, len(payload), 2)]


def parse_body(body: bytes) -> Optional[Tuple[int, List[int], List[int]]]:
    # body: [type][payload][chk]
    if len(body) < 3:
        return None

    msg_type = body[0]
    payload = body[1:-1]
    chk = body[-1]

    if checksum(payload) != chk:
        return None

    if msg_type == MSG_TYPE_POSITION:
        if len(payload) != POSITION_LEN:
            return None
        positions = parse_u16_le(payload)
        return (msg_type, positions, [])

    if msg_type == MSG_TYPE_TELEMETRY:
        if len(payload) != POSITION_LEN + FORCE_LEN:
            return None
        positions = parse_u16_le(payload[:POSITION_LEN])
        forces = parse_u16_le(payload[POSITION_LEN:])
        return (msg_type, positions, forces)

    return None


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__("banana_serial_bridge")

        # Parameters
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("timeout_s", 0.02)
        self.declare_parameter("publish_rate_hz", 200.0)
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
        self.pub_force = self.create_publisher(UInt16MultiArray, "rx_force", 10)
        self.sub_cmd = self.create_subscription(UInt16MultiArray, "tx_positions", self.on_cmd, 10)

        # Serial
        self.ser = serial.Serial(self.port, baudrate=self.baud, timeout=self.timeout_s)
        self.get_logger().info(f"Opened serial {self.port} @ {self.baud}")

        # RX buffer (encoded bytes, delimiter-split)
        self._rx_buf = bytearray()

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

    def _send_positions(self, positions: List[int]):
        if len(positions) != 8:
            self.get_logger().warn(f"tx_positions must have 8 values; got {len(positions)}")
            return

        frame = build_frame(MSG_TYPE_POSITION, positions)

        try:
            self.ser.write(frame)
            self.get_logger().info(f"Sent tx_positions: {positions}")
            try:
                self.ser.flush()
            except Exception:
                pass
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def on_cmd(self, msg: UInt16MultiArray):
        self._send_positions([int(x) for x in msg.data])

    def poll_serial(self):
        # Read whatever is available, append to buffer
        try:
            n = self.ser.in_waiting
            if n > 0:
                self._rx_buf += self.ser.read(n)
        except Exception as e:
            self.get_logger().error(f"Serial read failed: {e}")
            return

        # Extract all complete COBS frames (split by 0x00)
        while True:
            enc = self._extract_one_cobs_encoded_frame()
            if enc is None:
                break

            # Decode COBS
            try:
                body = cobs.decode(enc)
            except Exception:
                # Bad frame; keep going (we already resynced by delimiter)
                self.get_logger().warn(f"COBS decode failed (len={len(enc)}): {enc.hex()[:80]}...")
                continue

            parsed = parse_body(body)
            if parsed is None:
                continue

            msg_type, positions, forces = parsed

            if msg_type == MSG_TYPE_POSITION or msg_type == MSG_TYPE_TELEMETRY:
                js = JointState()
                js.header.stamp = self.get_clock().now().to_msg()
                js.name = self.joint_names
                js.position = [float(p) for p in positions[:POSITION_COUNT]]
                self.pub_js.publish(js)

            if msg_type == MSG_TYPE_TELEMETRY:
                force_msg = UInt16MultiArray()
                force_msg.data = [int(v) for v in forces]
                self.pub_force.publish(force_msg)

    def _extract_one_cobs_encoded_frame(self) -> Optional[bytes]:
        """
        Returns one encoded COBS frame (without delimiter), or None if no complete frame.
        Strategy:
        - Look for delimiter 0x00 in rx buffer
        - If found: take bytes before it as 'enc'
        - Drop delimiter and consumed bytes
        - Ignore empty frames (consecutive delimiters)
        - Cap buffer size to avoid runaway on noise
        """
        if not self._rx_buf:
            return None

        # Hard cap: if we somehow accumulate too much without a delimiter, drop + resync
        if len(self._rx_buf) > (MAX_ENC_FRAME * 4):
            self.get_logger().warn("RX buffer overflow; clearing to resync")
            self._rx_buf.clear()
            return None

        try:
            idx = self._rx_buf.index(0)  # delimiter position
        except ValueError:
            # No delimiter yet
            return None

        enc = bytes(self._rx_buf[:idx])
        # Remove [0..idx] inclusive (delimiter)
        del self._rx_buf[: idx + 1]

        if len(enc) == 0:
            # ignore empty frame (consecutive delimiters)
            return None

        if len(enc) > MAX_ENC_FRAME:
            # Drop garbage oversized frame
            self.get_logger().warn(f"Oversized encoded frame ({len(enc)} bytes); dropping")
            return None

        return enc


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
