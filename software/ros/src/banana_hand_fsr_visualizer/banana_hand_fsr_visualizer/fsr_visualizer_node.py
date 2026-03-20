#!/usr/bin/env python3
"""ROS2 + PySide6 2D fingertip force visualizer."""

from __future__ import annotations

import math
import signal
import sys
from dataclasses import dataclass
from typing import Dict, List

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt16MultiArray
try:
    from rclpy.exceptions import RCLError
except ImportError:
    from rclpy._rclpy_pybind11 import RCLError

from PySide6.QtCore import QPointF, QRectF, QTimer, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget

FINGER_NAMES = ["middle", "index", "ring", "pinky", "thumb"]
PALM_NAMES = ["palm_1", "palm_2", "palm_3", "palm_4", "palm_5"]
ALL_SENSOR_NAMES = FINGER_NAMES + PALM_NAMES
DISPLAY_NAMES = {
    "index": "Index",
    "middle": "Middle",
    "ring": "Ring",
    "pinky": "Pinky",
    "thumb": "Thumb",
    "palm_1": "Palm 1",
    "palm_2": "Palm 2",
    "palm_3": "Palm 3",
    "palm_4": "Palm 4",
    "palm_5": "Palm 5",
}


@dataclass
class FingerSample:
    raw: float = 0.0
    filtered: float = 0.0
    normalized: float = 0.0


class ForceNormalizer:
    def __init__(
        self,
        global_min: float,
        global_max: float,
        per_finger_min: Dict[str, float] | None = None,
        per_finger_max: Dict[str, float] | None = None,
    ):
        self.global_min = float(global_min)
        self.global_max = float(global_max)
        self.per_finger_min = per_finger_min or {}
        self.per_finger_max = per_finger_max or {}

    def normalize(self, finger: str, value: float) -> float:
        min_v = self.per_finger_min.get(finger, self.global_min)
        max_v = self.per_finger_max.get(finger, self.global_max)
        if max_v <= min_v:
            return 0.0
        ratio = (value - min_v) / (max_v - min_v)
        return max(0.0, min(1.0, ratio))


class ForceColorMapper:
    """Piecewise interpolation: low(blue-gray) -> mid(yellow-green) -> high(red)."""

    def __init__(self):
        self.low = QColor(60, 78, 100)
        self.mid = QColor(174, 222, 88)
        self.high = QColor(224, 74, 56)

    @staticmethod
    def _lerp(a: QColor, b: QColor, t: float) -> QColor:
        t = max(0.0, min(1.0, t))
        r = int(a.red() + (b.red() - a.red()) * t)
        g = int(a.green() + (b.green() - a.green()) * t)
        bch = int(a.blue() + (b.blue() - a.blue()) * t)
        return QColor(r, g, bch)

    def color_for(self, normalized: float) -> QColor:
        v = max(0.0, min(1.0, normalized))
        if v < 0.5:
            return self._lerp(self.low, self.mid, v / 0.5)
        return self._lerp(self.mid, self.high, (v - 0.5) / 0.5)


class HandVisualizerWidget(QWidget):
    def __init__(self, color_mapper: ForceColorMapper, contact_threshold: float, parent: QWidget | None = None):
        super().__init__(parent)
        self.color_mapper = color_mapper
        self.contact_threshold = max(0.0, min(1.0, contact_threshold))
        self.samples: Dict[str, FingerSample] = {
            name: FingerSample() for name in ALL_SENSOR_NAMES
        }
        self.has_data = False

        self.setMinimumSize(700, 540)

    def set_samples(self, samples: Dict[str, FingerSample], has_data: bool) -> None:
        self.samples = samples
        self.has_data = has_data
        self.update()

    def _finger_centers(self, w: int, h: int) -> Dict[str, QPointF]:
        cx = w * 0.52
        top = h * 0.17
        return {
            "thumb": QPointF(w * 0.22, h * 0.48),
            "index": QPointF(w * 0.40, top + 35),
            "middle": QPointF(cx, top),
            "ring": QPointF(w * 0.64, top + 22),
            "pinky": QPointF(w * 0.76, top + 58),
        }

    def _palm_centers(self, w: int, h: int) -> Dict[str, QPointF]:
        x0 = w * 0.36
        dx = w * 0.09
        y_mid = h * 0.56
        y_amp = h * 0.055
        y_offsets = [-y_amp, y_amp, -y_amp, y_amp, -y_amp]
        return {
            PALM_NAMES[i]: QPointF(x0 + i * dx, y_mid + y_offsets[i]) for i in range(5)
        }

    def paintEvent(self, event):
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        bg = QColor(26, 29, 34)
        painter.fillRect(self.rect(), bg)

        w = self.width()
        h = self.height()

        # Palm region.
        palm_color = QColor(66, 74, 83)
        painter.setPen(QPen(QColor(110, 118, 128), 2))
        painter.setBrush(palm_color)
        palm_x = w * 0.28
        palm_y = h * 0.28
        palm_w = w * 0.52
        palm_h = h * 0.46
        painter.drawRoundedRect(palm_x, palm_y, palm_w, palm_h, 70, 70)

        centers = self._finger_centers(w, h)
        palm_centers = self._palm_centers(w, h)
        tip_r = min(w, h) * 0.072
        palm_r = tip_r * 0.72

        label_font = QFont("Sans Serif", 10)
        value_font = QFont("Sans Serif", 11)
        value_font.setBold(True)

        for finger in FINGER_NAMES:
            c = centers[finger]
            sample = self.samples[finger]
            color = self.color_mapper.color_for(sample.normalized)

            painter.setPen(QPen(QColor(165, 172, 180), 2))
            painter.setBrush(color)
            painter.drawEllipse(c, tip_r, tip_r)

            if sample.normalized >= self.contact_threshold:
                painter.setPen(QPen(QColor(255, 236, 120), 3))
                painter.setBrush(Qt.NoBrush)
                painter.drawEllipse(c, tip_r + 6, tip_r + 6)

            painter.setPen(QColor(225, 230, 235))
            painter.setFont(label_font)
            label_rect = QRectF(c.x() - 64, c.y() + tip_r + 10, 128, 24)
            painter.drawText(label_rect, Qt.AlignHCenter | Qt.AlignVCenter, DISPLAY_NAMES[finger])

            painter.setFont(value_font)
            value_text = f"{sample.filtered:.1f}"
            value_rect = QRectF(c.x() - 46, c.y() - 14, 92, 28)
            painter.drawText(value_rect, Qt.AlignHCenter | Qt.AlignVCenter, value_text)

        for palm in PALM_NAMES:
            c = palm_centers[palm]
            sample = self.samples[palm]
            color = self.color_mapper.color_for(sample.normalized)

            painter.setPen(QPen(QColor(165, 172, 180), 2))
            painter.setBrush(color)
            painter.drawEllipse(c, palm_r, palm_r)

            if sample.normalized >= self.contact_threshold:
                painter.setPen(QPen(QColor(255, 236, 120), 3))
                painter.setBrush(Qt.NoBrush)
                painter.drawEllipse(c, palm_r + 5, palm_r + 5)

            painter.setPen(QColor(225, 230, 235))
            painter.setFont(label_font)
            label_rect = QRectF(c.x() - 58, c.y() + palm_r + 8, 116, 22)
            painter.drawText(
                label_rect,
                Qt.AlignHCenter | Qt.AlignVCenter,
                DISPLAY_NAMES[palm],
            )

            painter.setFont(value_font)
            value_text = f"{sample.filtered:.1f}"
            value_rect = QRectF(c.x() - 46, c.y() - 14, 92, 28)
            painter.drawText(value_rect, Qt.AlignHCenter | Qt.AlignVCenter, value_text)

        self._draw_legend(painter, w, h)

        if not self.has_data:
            painter.setPen(QColor(220, 220, 220))
            painter.setFont(QFont("Sans Serif", 10))
            painter.drawText(QRectF(0, 16, w, 24), Qt.AlignHCenter | Qt.AlignVCenter, "Waiting for data on topic...")

    def _draw_legend(self, painter: QPainter, w: int, h: int) -> None:
        lw = w * 0.62
        lx = (w - lw) / 2.0
        ly = h * 0.84
        lh = 18

        steps = 80
        for i in range(steps):
            t0 = i / max(1, steps - 1)
            x = lx + lw * t0
            color = self.color_mapper.color_for(t0)
            painter.setPen(QPen(color, max(1.0, lw / steps)))
            painter.drawLine(int(x), int(ly), int(x), int(ly + lh))

        painter.setPen(QColor(220, 220, 220))
        painter.setFont(QFont("Sans Serif", 9))
        labels_y = ly + lh + 6
        painter.drawText(QRectF(lx - 28, labels_y, 56, 20), Qt.AlignHCenter | Qt.AlignVCenter, "0")
        painter.drawText(QRectF(lx + lw * 0.5 - 38, labels_y, 76, 20), Qt.AlignHCenter | Qt.AlignVCenter, "2048")
        painter.drawText(QRectF(lx + lw - 38, labels_y, 76, 20), Qt.AlignHCenter | Qt.AlignVCenter, "4095")


class RosForceSubscriberNode(Node):
    def __init__(self, context: Context | None = None):
        super().__init__("banana_fsr_visualizer", context=context)

        self.declare_parameter("topic_name", "rx_force")
        self.declare_parameter("finger_indices", [0, 1, 2, 3, 4])
        self.declare_parameter("alpha", 0.25)
        self.declare_parameter("global_min", 0.0)
        self.declare_parameter("global_max", 4095.0)
        self.declare_parameter("per_finger_min", [])
        self.declare_parameter("per_finger_max", [])
        self.declare_parameter("refresh_hz", 30.0)
        self.declare_parameter("contact_threshold", 0.7)
        self.declare_parameter("simulate_if_no_data", False)

        self.topic_name = str(self.get_parameter("topic_name").value)
        self.finger_indices = list(self.get_parameter("finger_indices").value)
        if len(self.finger_indices) != 5:
            self.get_logger().warn("finger_indices must have length 5; falling back to [0,1,2,3,4].")
            self.finger_indices = [0, 1, 2, 3, 4]

        self.alpha = float(self.get_parameter("alpha").value)
        self.alpha = max(0.0, min(1.0, self.alpha))

        global_min = float(self.get_parameter("global_min").value)
        global_max = float(self.get_parameter("global_max").value)

        per_finger_min_vals = list(self.get_parameter("per_finger_min").value)
        per_finger_max_vals = list(self.get_parameter("per_finger_max").value)

        per_finger_min = {}
        per_finger_max = {}
        if len(per_finger_min_vals) == 5:
            per_finger_min = {k: float(v) for k, v in zip(FINGER_NAMES, per_finger_min_vals)}
        if len(per_finger_max_vals) == 5:
            per_finger_max = {k: float(v) for k, v in zip(FINGER_NAMES, per_finger_max_vals)}

        self.normalizer = ForceNormalizer(global_min, global_max, per_finger_min, per_finger_max)

        self.refresh_hz = float(self.get_parameter("refresh_hz").value)
        self.contact_threshold = float(self.get_parameter("contact_threshold").value)
        self.simulate_if_no_data = bool(self.get_parameter("simulate_if_no_data").value)

        self.samples: Dict[str, FingerSample] = {
            name: FingerSample() for name in ALL_SENSOR_NAMES
        }
        self.has_data = False
        self.last_rx_time = self.get_clock().now()
        self._sim_phase = 0.0

        latest_only_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.sub = self.create_subscription(
            UInt16MultiArray,
            self.topic_name,
            self._on_force_msg,
            latest_only_qos,
        )

        if self.simulate_if_no_data:
            self.create_timer(0.05, self._simulate_if_stale)

        self.get_logger().info(f"FSR visualizer listening on topic '{self.topic_name}'")

    def _on_force_msg(self, msg: UInt16MultiArray) -> None:
        finger_vals: List[float] = []
        for idx in self.finger_indices:
            finger_vals.append(float(msg.data[idx]) if idx < len(msg.data) else 0.0)

        tail_vals = [float(v) for v in list(msg.data)[-5:]]
        if len(tail_vals) < 5:
            tail_vals = ([0.0] * (5 - len(tail_vals))) + tail_vals

        for finger, raw in zip(FINGER_NAMES, finger_vals):
            filtered = raw
            norm = self.normalizer.normalize(finger, filtered)
            self.samples[finger] = FingerSample(raw=raw, filtered=filtered, normalized=norm)

        for palm, raw in zip(PALM_NAMES, tail_vals):
            filtered = raw
            norm = self.normalizer.normalize(palm, filtered)
            self.samples[palm] = FingerSample(raw=raw, filtered=filtered, normalized=norm)

        self.has_data = True
        self.last_rx_time = self.get_clock().now()

    def _simulate_if_stale(self) -> None:
        age_s = (self.get_clock().now() - self.last_rx_time).nanoseconds * 1e-9
        if age_s < 0.4:
            return

        self._sim_phase += 0.14
        base = [0.2, 0.45, 0.65, 0.4, 0.25]
        amps = [0.2, 0.22, 0.25, 0.2, 0.18]
        waves = [
            base[i] + amps[i] * (0.5 + 0.5 * math.sin(self._sim_phase * (1.0 + i * 0.13)))
            for i in range(5)
        ]

        for i, finger in enumerate(FINGER_NAMES):
            min_v = self.normalizer.per_finger_min.get(finger, self.normalizer.global_min)
            max_v = self.normalizer.per_finger_max.get(finger, self.normalizer.global_max)
            raw = min_v + waves[i] * (max_v - min_v)
            filtered = raw
            norm = self.normalizer.normalize(finger, filtered)
            self.samples[finger] = FingerSample(raw=raw, filtered=filtered, normalized=norm)

        palm_base = [0.35, 0.5, 0.7, 0.52, 0.4]
        palm_amp = [0.15, 0.18, 0.2, 0.16, 0.14]
        for i, palm in enumerate(PALM_NAMES):
            wave = palm_base[i] + palm_amp[i] * (
                0.5 + 0.5 * math.sin(self._sim_phase * (1.12 + i * 0.11))
            )
            raw = self.normalizer.global_min + wave * (
                self.normalizer.global_max - self.normalizer.global_min
            )
            filtered = raw
            norm = self.normalizer.normalize(palm, filtered)
            self.samples[palm] = FingerSample(raw=raw, filtered=filtered, normalized=norm)

        self.has_data = True


class MainWindow(QMainWindow):
    def __init__(self, ros_node: RosForceSubscriberNode):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle("Robotic Hand FSR Visualizer")
        self.resize(920, 680)

        self.color_mapper = ForceColorMapper()
        self.hand_widget = HandVisualizerWidget(self.color_mapper, ros_node.contact_threshold)
        self.executor = SingleThreadedExecutor(context=self.ros_node.context)
        self.executor.add_node(self.ros_node)

        self.status_label = QLabel("ROS: waiting for data")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("color: #c8c8c8; padding: 4px;")

        container = QWidget(self)
        layout = QVBoxLayout(container)
        layout.addWidget(self.hand_widget, 1)
        layout.addWidget(self.status_label)
        layout.setContentsMargins(10, 10, 10, 8)
        layout.setSpacing(4)
        self.setCentralWidget(container)

        self.setStyleSheet("QMainWindow, QWidget { background-color: #1a1d22; }")

        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self._spin_ros_once)
        self.spin_timer.start(max(5, int(1000.0 / max(10.0, ros_node.refresh_hz))))

        self.gui_timer = QTimer(self)
        self.gui_timer.timeout.connect(self._refresh_gui)
        self.gui_timer.start(max(15, int(1000.0 / max(10.0, ros_node.refresh_hz))))

    def _spin_ros_once(self) -> None:
        if not self.ros_node.context.ok():
            self.spin_timer.stop()
            self.gui_timer.stop()
            return
        try:
            self.executor.spin_once(timeout_sec=0.0)
        except RCLError:
            self.spin_timer.stop()
            self.gui_timer.stop()

    def _refresh_gui(self) -> None:
        self.hand_widget.set_samples(self.ros_node.samples, self.ros_node.has_data)

        topic = self.ros_node.topic_name
        status = "connected" if self.ros_node.has_data else "waiting"
        values = ", ".join(
            f"{DISPLAY_NAMES[k]}: {self.ros_node.samples[k].raw:.0f}/{self.ros_node.samples[k].filtered:.1f}"
            for k in ALL_SENSOR_NAMES
        )
        self.status_label.setText(f"ROS: {status}  |  topic: {topic}  |  raw/filtered -> {values}")

    def closeEvent(self, event):
        self.spin_timer.stop()
        self.gui_timer.stop()
        try:
            self.executor.remove_node(self.ros_node)
            self.executor.shutdown(timeout_sec=0.0)
        except Exception:
            pass
        super().closeEvent(event)


def main():
    context = Context()
    rclpy.init(context=context)
    node = RosForceSubscriberNode(context=context)

    app = QApplication(sys.argv)
    win = MainWindow(node)
    win.show()

    # Ensure Ctrl+C closes the Qt app cleanly.
    signal.signal(signal.SIGINT, lambda *_: app.quit())

    exit_code = 0
    try:
        exit_code = app.exec()
    except KeyboardInterrupt:
        exit_code = 0
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if context.ok():
            try:
                rclpy.shutdown(context=context)
            except RCLError:
                pass
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
