#!/usr/bin/env python3
"""
Robotic Hand FSR Visualizer (ROS2 + PySide6)

Dependencies:
- ROS2 with rclpy
- std_msgs
- PySide6

Run:
1) Build package:
   colcon build --packages-select banana_hand_fsr_visualizer
2) Source workspace:
   source install/setup.bash
3) Start visualizer:
   ros2 run banana_hand_fsr_visualizer fsr_visualizer

Where to change topic / scaling:
- ROS parameters on this node:
  - topic_name (default: rx_force)
  - alpha (default: 0.25)
  - global_min/global_max
  - per_finger_min/per_finger_max (optional length=5)
  - finger_indices (default: [0,1,2,3,4])

How to adapt message type:
- Replace the subscriber message type and callback parse logic in
  RosForceSubscriberNode._on_force_msg().
- The GUI consumes only {thumb,index,middle,ring,pinky} float values, so
  adapting is isolated to that callback.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from typing import Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray

from PySide6.QtCore import QPointF, QTimer, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget

FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]
DISPLAY_NAMES = {
    "thumb": "Thumb",
    "index": "Index",
    "middle": "Middle",
    "ring": "Ring",
    "pinky": "Pinky",
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
        self.samples: Dict[str, FingerSample] = {name: FingerSample() for name in FINGER_NAMES}
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
        tip_r = min(w, h) * 0.072

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
            painter.drawText(c.x() - 34, c.y() + tip_r + 22, DISPLAY_NAMES[finger])

            painter.setFont(value_font)
            value_text = f"{sample.filtered:.1f}"
            painter.drawText(c.x() - 20, c.y() + 6, value_text)

        self._draw_legend(painter, w, h)

        if not self.has_data:
            painter.setPen(QColor(220, 220, 220))
            painter.setFont(QFont("Sans Serif", 10))
            painter.drawText(18, 28, "Waiting for data on topic...")

    def _draw_legend(self, painter: QPainter, w: int, h: int) -> None:
        lx = w * 0.1
        ly = h * 0.84
        lw = w * 0.55
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
        painter.drawText(int(lx), int(ly + lh + 16), "Low")
        painter.drawText(int(lx + lw * 0.47), int(ly + lh + 16), "Medium")
        painter.drawText(int(lx + lw - 24), int(ly + lh + 16), "High")


class RosForceSubscriberNode(Node):
    def __init__(self):
        super().__init__("banana_fsr_visualizer")

        self.declare_parameter("topic_name", "rx_force")
        self.declare_parameter("finger_indices", [0, 1, 2, 3, 4])
        self.declare_parameter("alpha", 0.25)
        self.declare_parameter("global_min", 0.0)
        self.declare_parameter("global_max", 1200.0)
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

        self.samples: Dict[str, FingerSample] = {name: FingerSample() for name in FINGER_NAMES}
        self.has_data = False
        self.last_rx_time = self.get_clock().now()
        self._sim_phase = 0.0

        self.sub = self.create_subscription(UInt16MultiArray, self.topic_name, self._on_force_msg, 10)

        if self.simulate_if_no_data:
            self.create_timer(0.05, self._simulate_if_stale)

        self.get_logger().info(f"FSR visualizer listening on topic '{self.topic_name}'")

    def _on_force_msg(self, msg: UInt16MultiArray) -> None:
        # Easy message-type adaptation point: map incoming message -> 5 force values.
        vals: List[float] = []
        for idx in self.finger_indices:
            vals.append(float(msg.data[idx]) if idx < len(msg.data) else 0.0)

        for finger, raw in zip(FINGER_NAMES, vals):
            prev = self.samples[finger].filtered
            filtered = self.alpha * raw + (1.0 - self.alpha) * prev
            norm = self.normalizer.normalize(finger, filtered)
            self.samples[finger] = FingerSample(raw=raw, filtered=filtered, normalized=norm)

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
            prev = self.samples[finger].filtered
            filtered = self.alpha * raw + (1.0 - self.alpha) * prev
            norm = self.normalizer.normalize(finger, filtered)
            self.samples[finger] = FingerSample(raw=raw, filtered=filtered, normalized=norm)

        self.has_data = True


class MainWindow(QMainWindow):
    def __init__(self, ros_node: RosForceSubscriberNode):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle("Robotic Hand FSR Visualizer")
        self.resize(920, 680)

        self.color_mapper = ForceColorMapper()
        self.hand_widget = HandVisualizerWidget(self.color_mapper, ros_node.contact_threshold)

        self.status_label = QLabel("ROS: waiting for data")
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
        rclpy.spin_once(self.ros_node, timeout_sec=0.0)

    def _refresh_gui(self) -> None:
        self.hand_widget.set_samples(self.ros_node.samples, self.ros_node.has_data)

        topic = self.ros_node.topic_name
        status = "connected" if self.ros_node.has_data else "waiting"
        values = ", ".join(
            f"{DISPLAY_NAMES[k]}: {self.ros_node.samples[k].raw:.0f}/{self.ros_node.samples[k].filtered:.1f}"
            for k in FINGER_NAMES
        )
        self.status_label.setText(f"ROS: {status}  |  topic: {topic}  |  raw/filtered -> {values}")

    def closeEvent(self, event):
        self.spin_timer.stop()
        self.gui_timer.stop()
        super().closeEvent(event)


def main():
    rclpy.init()
    node = RosForceSubscriberNode()

    app = QApplication(sys.argv)
    win = MainWindow(node)
    win.show()

    exit_code = app.exec()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
