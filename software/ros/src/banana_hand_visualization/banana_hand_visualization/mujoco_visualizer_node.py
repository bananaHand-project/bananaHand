#!/usr/bin/env python3
"""Drive the Banana Hand MuJoCo model and publish ROS state traces."""

from __future__ import annotations

import contextlib
from pathlib import Path
import threading
from typing import Sequence

from ament_index_python.packages import get_package_share_directory
import glfw
import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16MultiArray
from trajectory_msgs.msg import JointTrajectory


POSITION_COUNT = 8
SETTLE_STEPS = 25
FORCE_SENSOR_NAMES = [
    "thumb",
    "index",
    "middle",
    "ring",
    "pinky",
    "palm_1",
    "palm_2",
    "palm_3",
    "palm_4",
    "palm_5",
]


class EglGlfwMujocoViewer:
    """Minimal MuJoCo desktop viewer that forces GLFW EGL context creation."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, title: str) -> None:
        self._model = model
        self._data = data
        self._lock = threading.RLock()
        self._closed = False
        self._glfw_initialized = False
        self._window = None

        if not glfw.init():
            raise RuntimeError("GLFW initialization failed")
        self._glfw_initialized = True

        glfw.default_window_hints()
        if not hasattr(glfw, "EGL_CONTEXT_API"):
            raise RuntimeError("GLFW EGL context API is unavailable in this environment")
        glfw.window_hint(glfw.CONTEXT_CREATION_API, glfw.EGL_CONTEXT_API)
        self._window = glfw.create_window(1280, 720, title, None, None)
        if not self._window:
            raise RuntimeError("Failed to create EGL GLFW window")

        glfw.make_context_current(self._window)
        glfw.swap_interval(1)

        self._camera = mujoco.MjvCamera()
        self._option = mujoco.MjvOption()
        self._perturb = mujoco.MjvPerturb()
        self._scene = mujoco.MjvScene(self._model, maxgeom=10000)
        self._context = mujoco.MjrContext(self._model, mujoco.mjtFontScale.mjFONTSCALE_150.value)
        self._cat_mask = int(mujoco.mjtCatBit.mjCAT_ALL.value)

    @contextlib.contextmanager
    def lock(self):
        with self._lock:
            yield

    def sync(self) -> None:
        if self._closed:
            return
        if self._window is None:
            return
        if glfw.window_should_close(self._window):
            self.close()
            return

        glfw.make_context_current(self._window)
        width, height = glfw.get_framebuffer_size(self._window)
        viewport = mujoco.MjrRect(0, 0, max(1, width), max(1, height))
        mujoco.mjv_updateScene(
            self._model,
            self._data,
            self._option,
            self._perturb,
            self._camera,
            self._cat_mask,
            self._scene,
        )
        mujoco.mjr_render(viewport, self._scene, self._context)
        glfw.swap_buffers(self._window)
        glfw.poll_events()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True

        if self._window is not None:
            glfw.destroy_window(self._window)
            self._window = None
        if self._glfw_initialized:
            glfw.terminate()
            self._glfw_initialized = False


def glfw_native_context_available() -> bool:
    """Probe whether default GLFW (GLX on Linux) can create a context."""
    if not glfw.init():
        return False
    window = None
    try:
        glfw.default_window_hints()
        window = glfw.create_window(16, 16, "glfw_probe", None, None)
        return bool(window)
    finally:
        if window is not None:
            glfw.destroy_window(window)
        glfw.terminate()




def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def normalize_state_source(value: str) -> str:
    source = value.strip().lower()
    aliases = {
        "rx": "rx_positions",
        "feedback": "rx_positions",
        "rx_positions": "rx_positions",
        "teleop": "teleop",
        "teleop_joint_trajectory": "teleop",
    }
    if source not in aliases:
        raise ValueError("state_source must be one of: rx_positions, teleop")
    return aliases[source]


class BananaHandMujocoVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("banana_hand_visualization")

        default_model_path = (
            Path(get_package_share_directory("banana_hand_visualization")) / "mujoco" / "scene.xml"
        )
        self.declare_parameter("model_path", str(default_model_path))
        self.declare_parameter("state_publish_rate_hz", 30.0)
        self.declare_parameter("show_mujoco_viewer", False)
        self.declare_parameter("state_source", "rx_positions")
        self.declare_parameter("teleop_topic", "/hand/teleop_joint_trajectory")
        self.declare_parameter("feedback_topic", "/rx_positions")
        self.declare_parameter("teleop_input_min_ratio", 0.0)
        self.declare_parameter("teleop_input_max_ratio", 1.0)
        self.declare_parameter("teleop_source_indices", [0, 1, 2, 3, 4, 5, -1, -1])
        self.declare_parameter("teleop_fill_ratio", 0.0)
        self.declare_parameter("force_input_topic", "/rx_force")
        self.declare_parameter("joint_state_topic", "/banana_hand/mujoco_joint_states")
        self.declare_parameter("force_state_topic", "/banana_hand/force_state")
        self.declare_parameter("force_sensor_indices", list(range(len(FORCE_SENSOR_NAMES))))
        self.declare_parameter("force_scale", 1.0)
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
        self.declare_parameter(
            "position_invert_mask",
            [True, True, True, True, True, True, False, False],
        )

        self._state_publish_rate_hz = float(self.get_parameter("state_publish_rate_hz").value)
        self._show_mujoco_viewer = bool(self.get_parameter("show_mujoco_viewer").value)
        self._state_source = normalize_state_source(str(self.get_parameter("state_source").value))
        self._actuator_map = list(self.get_parameter("actuator_map").value)
        self._direct_joint_map = list(self.get_parameter("direct_joint_map").value)
        self._motor_mins = [float(v) for v in self.get_parameter("motor_mins").value]
        self._motor_maxs = [float(v) for v in self.get_parameter("motor_maxs").value]
        self._position_invert_mask = [
            bool(v) for v in self.get_parameter("position_invert_mask").value
        ]
        self._teleop_input_min = float(self.get_parameter("teleop_input_min_ratio").value)
        self._teleop_input_max = float(self.get_parameter("teleop_input_max_ratio").value)
        self._teleop_source_indices = [
            int(v) for v in self.get_parameter("teleop_source_indices").value
        ]
        self._teleop_fill_ratio = float(self.get_parameter("teleop_fill_ratio").value)
        self._force_sensor_indices = [
            int(v) for v in self.get_parameter("force_sensor_indices").value
        ]
        self._force_scale = float(self.get_parameter("force_scale").value)

        if len(self._actuator_map) != POSITION_COUNT:
            raise ValueError("actuator_map must contain 8 entries")
        if len(self._direct_joint_map) != POSITION_COUNT:
            raise ValueError("direct_joint_map must contain 8 entries")
        if len(self._motor_mins) != POSITION_COUNT or len(self._motor_maxs) != POSITION_COUNT:
            raise ValueError("motor_mins and motor_maxs must contain 8 entries")
        if len(self._position_invert_mask) != POSITION_COUNT:
            raise ValueError("position_invert_mask must contain 8 entries")
        if len(self._teleop_source_indices) != POSITION_COUNT:
            raise ValueError("teleop_source_indices must contain 8 entries")
        if self._teleop_input_max <= self._teleop_input_min:
            raise ValueError("teleop_input_max_ratio must be greater than teleop_input_min_ratio")
        if len(self._force_sensor_indices) != len(FORCE_SENSOR_NAMES):
            raise ValueError(f"force_sensor_indices must contain {len(FORCE_SENSOR_NAMES)} entries")

        model_path = Path(str(self.get_parameter("model_path").value)).expanduser().resolve()
        if not model_path.exists():
            raise FileNotFoundError(f"MuJoCo model not found: {model_path}")

        self._model_path = model_path
        self._model = mujoco.MjModel.from_xml_path(str(model_path))
        self._data = mujoco.MjData(self._model)
        mujoco.mj_forward(self._model, self._data)

        self._viewer = None
        if self._show_mujoco_viewer:
            if glfw_native_context_available():
                self._viewer = mujoco.viewer.launch_passive(self._model, self._data)
                self.get_logger().info("MuJoCo desktop viewer started with default backend")
            else:
                self.get_logger().warn(
                    "Default GLFW/GLX context is unavailable; using GLFW EGL fallback viewer"
                )
                self._viewer = EglGlfwMujocoViewer(self._model, self._data, "Banana Hand MuJoCo")
                self.get_logger().info("MuJoCo desktop viewer started with GLFW EGL fallback")

        self._joint_name_to_id = {
            mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_JOINT, joint_id): joint_id
            for joint_id in range(self._model.njnt)
        }
        self._actuator_name_to_id = {
            mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id): actuator_id
            for actuator_id in range(self._model.nu)
        }

        joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        teleop_topic = str(self.get_parameter("teleop_topic").value)
        feedback_topic = str(self.get_parameter("feedback_topic").value)
        force_input_topic = str(self.get_parameter("force_input_topic").value)
        force_state_topic = str(self.get_parameter("force_state_topic").value)

        self._joint_pub = self.create_publisher(JointState, joint_state_topic, 10)
        self._force_state_pub = self.create_publisher(JointState, force_state_topic, 10)
        self.create_subscription(JointTrajectory, teleop_topic, self._on_teleop_trajectory, 10)
        self.create_subscription(JointState, feedback_topic, self._on_feedback_joint_state, 10)
        self.create_subscription(UInt16MultiArray, force_input_topic, self._on_force, 10)
        self.create_timer(1.0 / self._state_publish_rate_hz, self._publish_states)

        self.get_logger().info(
            f"Loaded MuJoCo model from {self._model_path} using state_source='{self._state_source}'"
        )

    def destroy_node(self) -> bool:
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
        return super().destroy_node()

    def _on_feedback_joint_state(self, msg: JointState) -> None:
        if self._state_source != "rx_positions":
            return
        if len(msg.position) < POSITION_COUNT:
            return
        self._apply_motor_positions([float(v) for v in msg.position[:POSITION_COUNT]])

    def _on_teleop_trajectory(self, msg: JointTrajectory) -> None:
        if self._state_source != "teleop":
            return
        if not msg.points:
            return

        src_positions = list(msg.points[0].positions)
        adc_values = []
        for output_idx, src_idx in enumerate(self._teleop_source_indices):
            ratio = self._resolve_teleop_ratio(src_positions, src_idx)
            adc_values.append(self._teleop_ratio_to_adc(ratio, output_idx))

        self._apply_motor_positions(adc_values)

    def _apply_motor_positions(self, values: Sequence[float]) -> None:
        for idx, raw_value in enumerate(values):
            min_raw = self._motor_mins[idx]
            max_raw = self._motor_maxs[idx]
            span = max_raw - min_raw
            normalized = 0.0 if span <= 0.0 else clamp((raw_value - min_raw) / span, 0.0, 1.0)
            if self._position_invert_mask[idx]:
                normalized = 1.0 - normalized

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

    def _value_at(self, data: Sequence[int], index: int) -> float:
        if index < 0 or index >= len(data):
            return 0.0
        return float(data[index]) * self._force_scale

    def _resolve_teleop_ratio(self, source: Sequence[float], src_idx: int) -> float:
        if src_idx < 0 or src_idx >= len(source):
            return self._teleop_fill_ratio
        return float(source[src_idx])

    def _teleop_ratio_to_adc(self, ratio: float, output_idx: int) -> float:
        span = self._teleop_input_max - self._teleop_input_min
        normalized = 0.0
        if span > 0.0:
            normalized = clamp((ratio - self._teleop_input_min) / span, 0.0, 1.0)
        min_adc = self._motor_mins[output_idx]
        max_adc = self._motor_maxs[output_idx]
        return min_adc + normalized * (max_adc - min_adc)

    def _on_force(self, msg: UInt16MultiArray) -> None:
        values = [self._value_at(msg.data, sensor_idx) for sensor_idx in self._force_sensor_indices]

        state_msg = JointState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.name = list(FORCE_SENSOR_NAMES)
        state_msg.position = values
        self._force_state_pub.publish(state_msg)

    def _publish_states(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._joint_pub.publish(self._build_joint_state_msg(stamp))
        if self._viewer is not None:
            with self._viewer.lock():
                self._viewer.sync()

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
