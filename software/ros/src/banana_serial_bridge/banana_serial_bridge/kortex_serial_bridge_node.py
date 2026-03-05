#!/usr/bin/env python3
"""ROS 2 bridge between Banana COBS packets and Kinova Gen3 UART via Kortex API."""

import inspect
import struct
import collections
import collections.abc
import socket
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16MultiArray

from cobs import cobs


COBS_DELIM = b"\x00"
MSG_TYPE_POSITION = 0x01
MSG_TYPE_TELEMETRY = 0x03
POSITION_COUNT = 8
FORCE_COUNT = 10
POSITION_LEN = POSITION_COUNT * 2
FORCE_LEN = FORCE_COUNT * 2
MAX_ENC_FRAME = 512

# Kinova's `kortex_api` currently pins an old protobuf version that still
# references symbols removed from `collections` in Python 3.10+.
_COLLECTIONS_COMPAT_ATTRS = (
    "Mapping",
    "MutableMapping",
    "Sequence",
    "MutableSequence",
    "Set",
    "MutableSet",
    "Iterable",
)
for _name in _COLLECTIONS_COMPAT_ATTRS:
    if not hasattr(collections, _name) and hasattr(collections.abc, _name):
        setattr(collections, _name, getattr(collections.abc, _name))


def checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def build_frame(msg_type: int, positions: List[int]) -> bytes:
    if len(positions) != POSITION_COUNT:
        raise ValueError(f"expected {POSITION_COUNT} uint16 values, got {len(positions)}")

    payload = b"".join(struct.pack("<H", int(p)) for p in positions)
    if len(payload) != POSITION_LEN:
        raise ValueError(f"payload must be {POSITION_LEN} bytes, got {len(payload)}")

    chk = checksum(payload)
    body = bytes([msg_type]) + payload + bytes([chk])
    return cobs.encode(body) + COBS_DELIM


def parse_u16_le(payload: bytes) -> List[int]:
    return [struct.unpack("<H", payload[i : i + 2])[0] for i in range(0, len(payload), 2)]


def parse_body(body: bytes) -> Optional[Tuple[int, List[int], List[int]]]:
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


class KortexUartTransport:
    def __init__(self, node: Node):
        self._node = node
        self._transport = None
        self._router = None
        self._session_manager = None
        self._socket: Optional[socket.socket] = None
        self._bridge_id = None
        self._interconnect_device_id: Optional[int] = None

        self._connect()

    def _connect(self) -> None:
        try:
            from kortex_api.TCPTransport import TCPTransport
            from kortex_api.RouterClient import RouterClient
            from kortex_api.SessionManager import SessionManager
            from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
            from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
            from kortex_api.autogen.client_stubs.InterconnectConfigClientRpc import (
                InterconnectConfigClient,
            )
            from kortex_api.autogen.messages import (
                Base_pb2,
                Common_pb2,
                InterconnectConfig_pb2,
                Session_pb2,
            )
        except ImportError as exc:
            raise RuntimeError(
                "Kortex API Python package not found. Install Kinova Kortex API first."
            ) from exc

        self._Base_pb2 = Base_pb2
        self._Common_pb2 = Common_pb2
        self._InterconnectConfig_pb2 = InterconnectConfig_pb2

        robot_ip = str(self._node.get_parameter("robot_ip").value)
        port = int(self._node.get_parameter("tcp_port").value)
        username = str(self._node.get_parameter("username").value)
        password = str(self._node.get_parameter("password").value)
        session_inactivity_ms = int(self._node.get_parameter("session_inactivity_timeout_ms").value)
        connection_inactivity_ms = int(
            self._node.get_parameter("connection_inactivity_timeout_ms").value
        )

        self._transport = TCPTransport()

        def _on_error(exc: Exception) -> None:
            self._node.get_logger().error(f"Kortex router error: {exc}")

        self._router = RouterClient(self._transport, _on_error)
        self._transport.connect(robot_ip, port)

        session_info = Session_pb2.CreateSessionInfo()
        session_info.username = username
        session_info.password = password
        session_info.session_inactivity_timeout = session_inactivity_ms
        session_info.connection_inactivity_timeout = connection_inactivity_ms

        self._session_manager = SessionManager(self._router)
        self._session_manager.CreateSession(session_info)

        self._base_client = BaseClient(self._router)
        self._device_manager = DeviceManagerClient(self._router)
        self._interconnect_client = InterconnectConfigClient(self._router)
        self._interconnect_device_id = self._get_device_id(
            device_type=Common_pb2.INTERCONNECT,
            device_index=int(self._node.get_parameter("interconnect_index").value),
        )
        if self._interconnect_device_id is None:
            raise RuntimeError("Could not find INTERCONNECT device from DeviceManager.ReadAllDevices()")

        self._set_uart_enabled(
            enabled=True,
            port_id=int(self._node.get_parameter("uart_port_id").value),
            speed=int(self._node.get_parameter("uart_speed").value),
            word_length=int(self._node.get_parameter("uart_word_length").value),
            stop_bits=int(self._node.get_parameter("uart_stop_bits").value),
            parity=int(self._node.get_parameter("uart_parity").value),
        )
        bridge_cfg = Base_pb2.BridgeConfig()
        bridge_cfg.device_identifier = self._interconnect_device_id
        bridge_cfg.bridgetype = Base_pb2.BRIDGE_TYPE_UART
        target_port = int(self._node.get_parameter("bridge_target_port").value)
        out_port = int(self._node.get_parameter("bridge_out_port").value)
        if target_port or out_port:
            bridge_cfg.port_config.target_port = target_port
            bridge_cfg.port_config.out_port = out_port
        bridge_result = self._base_client.EnableBridge(bridge_cfg)
        self._bridge_id = bridge_result.bridge_id
        if bridge_result.status != Base_pb2.BRIDGE_STATUS_OK:
            raise RuntimeError(f"EnableBridge failed with status={bridge_result.status}")

        bridge_config = self._base_client.GetBridgeConfig(self._bridge_id)
        socket_port = int(bridge_config.port_config.out_port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.connect((robot_ip, socket_port))
        self._socket.setblocking(False)

        interconnect_port = int(bridge_config.port_config.target_port)
        self._node.get_logger().info(f"Connected to Kinova Gen3 @ {robot_ip}:{port}")
        self._node.get_logger().info(
            f"Enabled UART bridge id={self._bridge_id.bridge_id} "
            f"device={self._interconnect_device_id} target_port={interconnect_port} out_port={socket_port}"
        )

    def _get_device_id(self, device_type: int, device_index: int) -> Optional[int]:
        devices = self._device_manager.ReadAllDevices()
        cur = 0
        for device in devices.device_handle:
            if device.device_type != device_type:
                continue
            if cur == device_index:
                return int(device.device_identifier)
            cur += 1
        return None

    def _set_uart_enabled(
        self,
        enabled: bool,
        port_id: int,
        speed: int,
        word_length: int,
        stop_bits: int,
        parity: int,
    ) -> None:
        cfg = self._Common_pb2.UARTConfiguration()
        cfg.port_id = port_id
        cfg.enabled = enabled
        cfg.speed = speed
        cfg.word_length = word_length
        cfg.stop_bits = stop_bits
        cfg.parity = parity
        self._interconnect_client.SetUARTConfiguration(cfg, deviceId=self._interconnect_device_id)

    def close(self) -> None:
        try:
            if self._socket is not None:
                self._socket.close()
                self._socket = None
        except Exception:
            pass

        try:
            if self._bridge_id is not None:
                self._base_client.DisableBridge(self._bridge_id)
                self._bridge_id = None
        except Exception:
            pass

        try:
            if self._interconnect_device_id is not None:
                self._set_uart_enabled(
                    enabled=False,
                    port_id=int(self._node.get_parameter("uart_port_id").value),
                    speed=int(self._node.get_parameter("uart_speed").value),
                    word_length=int(self._node.get_parameter("uart_word_length").value),
                    stop_bits=int(self._node.get_parameter("uart_stop_bits").value),
                    parity=int(self._node.get_parameter("uart_parity").value),
                )
        except Exception:
            pass

        try:
            if self._session_manager is not None:
                self._session_manager.CloseSession()
        except Exception:
            pass

        try:
            if self._transport is not None:
                self._transport.disconnect()
        except Exception:
            pass

    def write(self, data: bytes) -> None:
        if self._socket is None:
            raise RuntimeError("UART bridge socket is not connected")
        self._socket.sendall(data)

    def read(self) -> bytes:
        if self._socket is None:
            return b""
        try:
            return self._socket.recv(4096)
        except BlockingIOError:
            return b""
        except socket.timeout:
            return b""


class KortexSerialBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("banana_kortex_serial_bridge")

        self.declare_parameter("robot_ip", "192.168.1.10")
        self.declare_parameter("tcp_port", 10000)
        self.declare_parameter("username", "admin")
        self.declare_parameter("password", "admin")
        self.declare_parameter("session_inactivity_timeout_ms", 60000)
        self.declare_parameter("connection_inactivity_timeout_ms", 2000)
        self.declare_parameter("publish_rate_hz", 200.0)
        self.declare_parameter("joint_names", [f"joint_{i}" for i in range(8)])

        # Defaults match Kinova example for expansion UART: 115200 8N1.
        self.declare_parameter("interconnect_index", 0)
        self.declare_parameter("uart_port_id", 1)
        self.declare_parameter("uart_speed", 6)
        self.declare_parameter("uart_word_length", 2)
        self.declare_parameter("uart_stop_bits", 2)
        self.declare_parameter("uart_parity", 1)
        # Keep 0/0 for auto bridge ports, as in the Kinova example.
        self.declare_parameter("bridge_target_port", 0)
        self.declare_parameter("bridge_out_port", 0)

        self.joint_names = list(self.get_parameter("joint_names").value)
        if len(self.joint_names) != POSITION_COUNT:
            self.get_logger().warn("joint_names is not length 8; forcing default names")
            self.joint_names = [f"joint_{i}" for i in range(POSITION_COUNT)]

        self.pub_js = self.create_publisher(JointState, "rx_positions", 10)
        self.pub_force = self.create_publisher(UInt16MultiArray, "rx_force", 10)
        self.sub_cmd = self.create_subscription(UInt16MultiArray, "tx_positions", self.on_cmd, 10)

        self._rx_buf = bytearray()

        self._transport = KortexUartTransport(self)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if publish_rate_hz <= 0.0:
            self.get_logger().warn("publish_rate_hz must be > 0; forcing 200 Hz")
            publish_rate_hz = 200.0

        self.timer = self.create_timer(1.0 / publish_rate_hz, self.poll_uart)

    def destroy_node(self) -> bool:
        try:
            if self._transport is not None:
                self._transport.close()
        except Exception:
            pass
        return super().destroy_node()

    def on_cmd(self, msg: UInt16MultiArray) -> None:
        positions = [int(x) for x in msg.data]
        if len(positions) != POSITION_COUNT:
            self.get_logger().warn(f"tx_positions must have 8 values; got {len(positions)}")
            return

        frame = build_frame(MSG_TYPE_POSITION, positions)

        try:
            self._transport.write(frame)
        except Exception as exc:
            self.get_logger().error(f"Kortex UART write failed: {exc}")

    def poll_uart(self) -> None:
        try:
            chunk = self._transport.read()
            if chunk:
                self._rx_buf += chunk
        except Exception as exc:
            self.get_logger().error(f"Kortex UART read failed: {exc}")
            return

        while True:
            enc = self._extract_one_cobs_encoded_frame()
            if enc is None:
                break

            try:
                body = cobs.decode(enc)
            except Exception:
                self.get_logger().warn(f"COBS decode failed (len={len(enc)}): {enc.hex()[:80]}...")
                continue

            parsed = parse_body(body)
            if parsed is None:
                continue

            msg_type, positions, forces = parsed

            if msg_type in (MSG_TYPE_POSITION, MSG_TYPE_TELEMETRY):
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
        if not self._rx_buf:
            return None

        if len(self._rx_buf) > (MAX_ENC_FRAME * 4):
            self.get_logger().warn("RX buffer overflow; clearing to resync")
            self._rx_buf.clear()
            return None

        try:
            idx = self._rx_buf.index(0)
        except ValueError:
            return None

        enc = bytes(self._rx_buf[:idx])
        del self._rx_buf[: idx + 1]

        if len(enc) == 0:
            return None

        if len(enc) > MAX_ENC_FRAME:
            self.get_logger().warn(f"Oversized encoded frame ({len(enc)} bytes); dropping")
            return None

        return enc


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = KortexSerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
