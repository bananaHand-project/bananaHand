#!/usr/bin/env python3
"""Drive the Banana Hand MuJoCo model and publish ROS state traces."""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

from ament_index_python.packages import get_package_share_directory
import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, UInt16MultiArray


ACTUATOR_COUNT = 6
POSITION_COUNT = 8
SETTLE_STEPS = 25


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class BananaHandMujocoVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("banana_hand_mujoco_visualizer")

        default_model_path = (
            Path(get_package_share_directory("banana_hand_mujoco")) / "mujoco" / "scene.xml"
        )
        self.declare_parameter("model_path", str(default_model_path))
        self.declare_parameter("state_publish_rate_hz", 30.0)
        self.declare_parameter("show_mujoco_viewer", False)
        self.declare_parameter("actuator_command_topic", "/banana_hand/actuator_commands")
        self.declare_parameter("command_topic", "/tx_positions")
        self.declare_parameter("feedback_topic", "/rx_positions")
        self.declare_parameter("joint_state_topic", "/banana_hand/mujoco_joint_states")
        self.declare_parameter("actuator_state_topic", "/banana_hand/actuator_state")
        self.declare_parameter("command_state_topic", "/banana_hand/command_state")
        self.declare_parameter(
            "actuator_map",
            [
                "indexpiston",
                "middlepiston",
                "ringpiston",
                "pinkypiston",
                "tmpiston",
                "thumbpiston",
                "",
                "",
            ],
        )
        self.declare_parameter(
            "direct_joint_map",
            ["", "", "", "", "", "", "indexrev4", "thumbrev4"],
        )
        self.declare_parameter(
            "motor_mins",
            [400.0, 400.0, 400.0, 400.0, 300.0, 150.0, 0.0, 0.0],
        )
        self.declare_parameter(
            "motor_maxs",
            [3900.0, 3900.0, 3900.0, 3900.0, 3900.0, 3900.0, 4095.0, 4095.0],
        )

        self._state_publish_rate_hz = float(self.get_parameter("state_publish_rate_hz").value)
        self._show_mujoco_viewer = bool(self.get_parameter("show_mujoco_viewer").value)
        self._actuator_map = list(self.get_parameter("actuator_map").value)
        self._direct_joint_map = list(self.get_parameter("direct_joint_map").value)
        self._motor_mins = [float(v) for v in self.get_parameter("motor_mins").value]
        self._motor_maxs = [float(v) for v in self.get_parameter("motor_maxs").value]

        if len(self._actuator_map) != POSITION_COUNT:
            raise ValueError("actuator_map must contain 8 entries")
        if len(self._direct_joint_map) != POSITION_COUNT:
            raise ValueError("direct_joint_map must contain 8 entries")
        if len(self._motor_mins) != POSITION_COUNT or len(self._motor_maxs) != POSITION_COUNT:
            raise ValueError("motor_mins and motor_maxs must contain 8 entries")

        model_path = Path(str(self.get_parameter("model_path").value)).expanduser().resolve()
        if not model_path.exists():
            raise FileNotFoundError(f"MuJoCo model not found: {model_path}")

        self._model_path = model_path
        self._model = mujoco.MjModel.from_xml_path(str(model_path))
        self._data = mujoco.MjData(self._model)
        mujoco.mj_forward(self._model, self._data)

        self._viewer = None
        if self._show_mujoco_viewer:
            self._viewer = mujoco.viewer.launch_passive(self._model, self._data)

        self._joint_name_to_id = {
            mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_JOINT, joint_id): joint_id
            for joint_id in range(self._model.njnt)
        }
        self._actuator_name_to_id = {
            mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id): actuator_id
            for actuator_id in range(self._model.nu)
        }

        self._last_positions = [0.0] * POSITION_COUNT

        joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        actuator_state_topic = str(self.get_parameter("actuator_state_topic").value)
        command_state_topic = str(self.get_parameter("command_state_topic").value)
        actuator_command_topic = str(self.get_parameter("actuator_command_topic").value)
        command_topic = str(self.get_parameter("command_topic").value)
        feedback_topic = str(self.get_parameter("feedback_topic").value)

        self._joint_pub = self.create_publisher(JointState, joint_state_topic, 10)
        self._actuator_state_pub = self.create_publisher(JointState, actuator_state_topic, 10)
        self._command_state_pub = self.create_publisher(JointState, command_state_topic, 10)
        self.create_subscription(
            Float32MultiArray, actuator_command_topic, self._on_actuator_commands, 10
        )
        self.create_subscription(UInt16MultiArray, command_topic, self._on_motor_positions, 10)
        self.create_subscription(JointState, feedback_topic, self._on_feedback_joint_state, 10)
        self.create_timer(1.0 / self._state_publish_rate_hz, self._publish_states)

        self.get_logger().info(f"Loaded MuJoCo model from {self._model_path}")

    def destroy_node(self) -> bool:
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
        return super().destroy_node()

    def _on_motor_positions(self, msg: UInt16MultiArray) -> None:
        if len(msg.data) < POSITION_COUNT:
            self.get_logger().warn(
                f"Expected {POSITION_COUNT} motor positions, got {len(msg.data)}"
            )
            return
        self._apply_motor_positions([float(v) for v in msg.data[:POSITION_COUNT]])

    def _on_actuator_commands(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < ACTUATOR_COUNT:
            self.get_logger().warn(
                f"Expected {ACTUATOR_COUNT} actuator command values, got {len(msg.data)}"
            )
            return

        normalized_values = [clamp(float(v), 0.0, 1.0) for v in msg.data[:ACTUATOR_COUNT]]
        for actuator_id, normalized in enumerate(normalized_values):
            ctrl_min, ctrl_max = self._model.actuator_ctrlrange[actuator_id]
            self._data.ctrl[actuator_id] = ctrl_min + normalized * (ctrl_max - ctrl_min)

        self._publish_command_state(normalized_values)
        for _ in range(SETTLE_STEPS):
            mujoco.mj_step(self._model, self._data)

    def _on_feedback_joint_state(self, msg: JointState) -> None:
        if len(msg.position) < POSITION_COUNT:
            return
        self._apply_motor_positions([float(v) for v in msg.position[:POSITION_COUNT]])

    def _apply_motor_positions(self, values: Sequence[float]) -> None:
        self._last_positions = list(values)

        for idx, raw_value in enumerate(values):
            min_raw = self._motor_mins[idx]
            max_raw = self._motor_maxs[idx]
            span = max_raw - min_raw
            normalized = 0.0 if span <= 0.0 else clamp((raw_value - min_raw) / span, 0.0, 1.0)

            actuator_name = self._actuator_map[idx]
            if actuator_name:
                actuator_id = self._actuator_name_to_id.get(actuator_name)
                if actuator_id is not None:
                    ctrl_min, ctrl_max = self._model.actuator_ctrlrange[actuator_id]
                    self._data.ctrl[actuator_id] = ctrl_min + normalized * (ctrl_max - ctrl_min)

            joint_name = self._direct_joint_map[idx]
            if joint_name:
                joint_id = self._joint_name_to_id.get(joint_name)
                if joint_id is not None:
                    qpos_adr = int(self._model.jnt_qposadr[joint_id])
                    qpos_min, qpos_max = self._model.jnt_range[joint_id]
                    self._data.qpos[qpos_adr] = qpos_min + normalized * (qpos_max - qpos_min)

        for _ in range(SETTLE_STEPS):
            mujoco.mj_step(self._model, self._data)

    def _publish_states(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._joint_pub.publish(self._build_joint_state_msg(stamp))
        self._actuator_state_pub.publish(self._build_actuator_state_msg(stamp))
        if self._viewer is not None:
            with self._viewer.lock():
                self._viewer.sync()

    def _publish_command_state(self, values: Sequence[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
            or f"actuator_{actuator_id}"
            for actuator_id in range(ACTUATOR_COUNT)
        ]
        msg.position = [float(value) for value in values]
        self._command_state_pub.publish(msg)

    def _build_joint_state_msg(self, stamp) -> JointState:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = []
        msg.position = []
        for joint_id in range(self._model.njnt):
            joint_name = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            if not joint_name:
                continue
            qpos_adr = int(self._model.jnt_qposadr[joint_id])
            msg.name.append(joint_name)
            msg.position.append(float(self._data.qpos[qpos_adr]))
        return msg

    def _build_actuator_state_msg(self, stamp) -> JointState:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = []
        msg.position = []
        for actuator_id in range(self._model.nu):
            actuator_name = mujoco.mj_id2name(
                self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id
            )
            msg.name.append(actuator_name or f"actuator_{actuator_id}")
            msg.position.append(float(self._data.ctrl[actuator_id]))
        return msg


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BananaHandMujocoVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
