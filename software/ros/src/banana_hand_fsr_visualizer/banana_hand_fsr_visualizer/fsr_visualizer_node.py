#!/usr/bin/env python3
"""BananaHand ROS2 + PySide6 dashboard for force and position telemetry."""

from __future__ import annotations

import math
import signal
import sys
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Sequence

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16MultiArray

try:
    from rclpy.exceptions import RCLError
except ImportError:
    from rclpy._rclpy_pybind11 import RCLError

from PySide6.QtCore import QPointF, QRectF, QSize, QTimer, Qt
from PySide6.QtGui import QColor, QFont, QFontMetrics, QLinearGradient, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QVBoxLayout,
    QWidget,
)

FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]
PALM_NAMES = ["palm_1", "palm_2", "palm_3", "palm_4", "palm_5"]
POSITION_NAMES = ["thumb", "index", "middle", "ring", "pinky", "opposition"]
ALL_SENSOR_NAMES = FINGER_NAMES + PALM_NAMES
DISPLAY_NAMES = {
    "thumb": "Thumb",
    "index": "Index",
    "middle": "Middle",
    "ring": "Ring",
    "pinky": "Pinky",
    "opposition": "Opposition",
    "palm_1": "Palm A",
    "palm_2": "Palm B",
    "palm_3": "Palm C",
    "palm_4": "Palm D",
    "palm_5": "Palm E",
}


class BrandPalette:
    APP_BG = QColor("#F3F1EA")
    PANEL = QColor("#FCFBF7")
    PANEL_ALT = QColor("#E8E4DA")
    PANEL_DARK = QColor("#171715")
    BORDER = QColor("#1A1815")
    TEXT = QColor("#11100E")
    TEXT_MUTED = QColor("#5D5951")
    SOFT_TEXT = QColor("#817A70")
    BANANA = QColor("#EFE437")
    BANANA_DEEP = QColor("#D7BF19")
    BANANA_SOFT = QColor("#FFF4A5")
    ACCENT = QColor("#27241F")
    GRID = QColor("#3C3832")
    SUCCESS = QColor("#4B8D45")
    WARNING = QColor("#D48F19")
    DANGER = QColor("#A5432A")
    TRACE_COLORS = [
        QColor("#F1E63B"),
        QColor("#FFC84B"),
        QColor("#F79C2D"),
        QColor("#9ABF3D"),
        QColor("#6EC5A8"),
        QColor("#D7D0BC"),
    ]


@dataclass
class SensorSample:
    raw: float = 0.0
    filtered: float = 0.0
    normalized: float = 0.0


@dataclass
class TopicSnapshot:
    count: int = 0
    last_time_s: float = 0.0


class ForceNormalizer:
    def __init__(
        self,
        global_min: float,
        global_max: float,
        per_sensor_min: Dict[str, float] | None = None,
        per_sensor_max: Dict[str, float] | None = None,
    ):
        self.global_min = float(global_min)
        self.global_max = float(global_max)
        self.per_sensor_min = per_sensor_min or {}
        self.per_sensor_max = per_sensor_max or {}

    def normalize(self, sensor: str, value: float) -> float:
        min_v = self.per_sensor_min.get(sensor, self.global_min)
        max_v = self.per_sensor_max.get(sensor, self.global_max)
        if max_v <= min_v:
            return 0.0
        ratio = (value - min_v) / (max_v - min_v)
        return max(0.0, min(1.0, ratio))

    def denormalize(self, ratio: float) -> float:
        return self.global_min + max(0.0, min(1.0, ratio)) * (self.global_max - self.global_min)


class ForceColorMapper:
    def __init__(self):
        self.low = QColor("#2F6BFF")
        self.mid = QColor("#38C172")
        self.high = QColor("#E43D30")

    @staticmethod
    def _lerp(a: QColor, b: QColor, t: float) -> QColor:
        t = max(0.0, min(1.0, t))
        return QColor(
            int(a.red() + (b.red() - a.red()) * t),
            int(a.green() + (b.green() - a.green()) * t),
            int(a.blue() + (b.blue() - a.blue()) * t),
        )

    def color_for(self, normalized: float) -> QColor:
        value = max(0.0, min(1.0, normalized))
        if value < 0.45:
            return self._lerp(self.low, self.mid, value / 0.45)
        return self._lerp(self.mid, self.high, (value - 0.45) / 0.55)


class BananaLogoWidget(QWidget):
    PIXELS = [
        "0000000011000000",
        "0000000011000000",
        "0000000111100000",
        "0000001111110000",
        "0000001111110000",
        "0000001111110000",
        "0010001111110000",
        "0011101111110011",
        "0011111111111111",
        "0001111111111111",
        "0011011111111100",
        "0001111111111000",
        "0000111101111110",
    ]

    def sizeHint(self) -> QSize:
        return QSize(100, 84)

    def paintEvent(self, event) -> None:
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, False)
        painter.fillRect(self.rect(), Qt.transparent)

        rows = len(self.PIXELS)
        cols = len(self.PIXELS[0])
        cell = min(self.width() / cols, self.height() / rows)
        x0 = (self.width() - cols * cell) / 2.0
        y0 = (self.height() - rows * cell) / 2.0

        for row_index, row in enumerate(self.PIXELS):
            for col_index, value in enumerate(row):
                if value != "1":
                    continue
                rect = QRectF(x0 + col_index * cell, y0 + row_index * cell, cell + 0.5, cell + 0.5)
                painter.fillRect(rect, BrandPalette.BANANA)

        eye_size = cell * 1.25
        painter.fillRect(QRectF(x0 + cell * 7.3, y0 + cell * 5.2, eye_size, eye_size), BrandPalette.BORDER)
        painter.fillRect(QRectF(x0 + cell * 10.3, y0 + cell * 4.4, eye_size, eye_size), BrandPalette.BORDER)


class MetricCard(QFrame):
    def __init__(self, title: str, accent: QColor, parent: QWidget | None = None):
        super().__init__(parent)
        self.setObjectName("metricCard")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 14, 16, 14)
        layout.setSpacing(4)

        title_label = QLabel(title)
        title_label.setObjectName("metricTitle")
        title_label.setStyleSheet(f"color: {BrandPalette.TEXT_MUTED.name()};")

        self.value_label = QLabel("--")
        self.value_label.setObjectName("metricValue")

        self.meta_label = QLabel("")
        self.meta_label.setObjectName("metricMeta")
        self.meta_label.setStyleSheet(f"color: {BrandPalette.SOFT_TEXT.name()};")

        accent_bar = QFrame()
        accent_bar.setFixedHeight(4)
        accent_bar.setStyleSheet(
            f"background:{accent.name()}; border:none; border-radius:2px;"
        )

        layout.addWidget(accent_bar)
        layout.addWidget(title_label)
        layout.addWidget(self.value_label)
        layout.addWidget(self.meta_label)

    def set_content(self, value: str, meta: str) -> None:
        if self.value_label.text() != value:
            self.value_label.setText(value)
        if self.meta_label.text() != meta:
            self.meta_label.setText(meta)


class SensorChip(QFrame):
    def __init__(self, label: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.setObjectName("sensorChip")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(2)

        self.label = QLabel(label)
        self.label.setObjectName("sensorChipLabel")

        self.value = QLabel("0")
        self.value.setObjectName("sensorChipValue")

        self.meta = QLabel("idle")
        self.meta.setObjectName("sensorChipMeta")

        layout.addWidget(self.label)
        layout.addWidget(self.value)
        layout.addWidget(self.meta)

    def set_sample(self, sample: SensorSample, contact_threshold: float, mapper: ForceColorMapper) -> None:
        active = sample.normalized >= contact_threshold
        tint = mapper.color_for(sample.normalized)
        alpha = 70 if active else 26
        border = BrandPalette.BORDER.name() if active else "#C8C0B1"
        self.setStyleSheet(
            "QFrame#sensorChip {"
            f"background: rgba({tint.red()}, {tint.green()}, {tint.blue()}, {alpha}); "
            f"border: 1px solid {border}; border-radius: 16px; }}"
            f"QLabel#sensorChipLabel {{ color: {BrandPalette.TEXT_MUTED.name()}; font-size: 11px; font-weight: 600; }}"
            f"QLabel#sensorChipValue {{ color: {BrandPalette.TEXT.name()}; font-size: 22px; font-weight: 700; }}"
            f"QLabel#sensorChipMeta {{ color: {BrandPalette.SOFT_TEXT.name()}; font-size: 11px; }}"
        )
        self.value.setText(f"{sample.filtered:.0f}")
        self.meta.setText("contact" if active else "monitoring")


class SensorReadoutGrid(QFrame):
    def __init__(self, contact_threshold: float, mapper: ForceColorMapper, parent: QWidget | None = None):
        super().__init__(parent)
        self.contact_threshold = contact_threshold
        self.mapper = mapper
        self.chips: Dict[str, SensorChip] = {}
        self.setObjectName("sensorGrid")

        layout = QGridLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(10)

        for index, name in enumerate(FINGER_NAMES):
            chip = SensorChip(DISPLAY_NAMES[name])
            layout.addWidget(chip, 0, index)
            self.chips[name] = chip

        for index, name in enumerate(PALM_NAMES):
            chip = SensorChip(DISPLAY_NAMES[name])
            layout.addWidget(chip, 1, index)
            self.chips[name] = chip

    def update_samples(self, samples: Dict[str, SensorSample]) -> None:
        for name, chip in self.chips.items():
            chip.set_sample(samples[name], self.contact_threshold, self.mapper)


class TraceLegend(QFrame):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setObjectName("traceLegend")
        self.setStyleSheet(
            "QFrame#traceLegend { background: rgba(23, 23, 21, 0.18); border-radius: 14px; }"
        )

        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(10)
        layout.addStretch(1)

        for color, name in zip(BrandPalette.TRACE_COLORS, POSITION_NAMES):
            label = QLabel(f"● {DISPLAY_NAMES[name]}")
            label.setStyleSheet(
                f"color: {color.name()}; font-size: 12px; font-weight: 600; background: transparent;"
            )
            layout.addWidget(label)

        layout.addStretch(1)


class PositionPlotsPanel(QWidget):
    def __init__(self, window_seconds: float, y_min: float, y_max: float, parent: QWidget | None = None):
        super().__init__(parent)
        self.plots: Dict[str, PositionPlotWidget] = {}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        for color, name in zip(BrandPalette.TRACE_COLORS, POSITION_NAMES):
            plot = PositionPlotWidget(name, color, window_seconds, y_min, y_max)
            self.plots[name] = plot
            layout.addWidget(plot)

    def append_sample(self, timestamp_s: float, values: Sequence[float]) -> None:
        for name, value in zip(POSITION_NAMES, values):
            self.plots[name].append_sample(timestamp_s, value)


class HandVisualizerWidget(QWidget):
    def __init__(self, color_mapper: ForceColorMapper, contact_threshold: float, parent: QWidget | None = None):
        super().__init__(parent)
        self.color_mapper = color_mapper
        self.contact_threshold = max(0.0, min(1.0, contact_threshold))
        self.samples: Dict[str, SensorSample] = {name: SensorSample() for name in ALL_SENSOR_NAMES}
        self.has_data = False
        self.setMinimumHeight(600)

    def set_samples(self, samples: Dict[str, SensorSample], has_data: bool) -> None:
        self.samples = samples
        self.has_data = has_data
        self.update()

    def paintEvent(self, event) -> None:
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), Qt.transparent)

        bounds = self.rect().adjusted(34, 28, -34, -92)
        self._draw_hand_map(painter, bounds)
        self._draw_legend(painter, self.rect().adjusted(42, self.height() - 58, -42, -20))

        if not self.has_data:
            painter.setPen(BrandPalette.SOFT_TEXT)
            painter.setFont(QFont("DejaVu Sans", 11))
            painter.drawText(
                QRectF(0, 0, self.width(), 28),
                Qt.AlignHCenter | Qt.AlignVCenter,
                "Waiting for `rx_force` samples",
            )

    def _hand_specs(self, rect: QRectF) -> Dict[str, tuple[QPointF, QSize, float]]:
        return {
            "thumb": (QPointF(rect.left() + rect.width() * 0.85, rect.top() + rect.height() * 0.57), QSize(58, 58), 0.0),
            "index": (QPointF(rect.left() + rect.width() * 0.37, rect.top() + rect.height() * 0.17), QSize(62, 62), 0.0),
            "middle": (QPointF(rect.left() + rect.width() * 0.49, rect.top() + rect.height() * 0.17), QSize(66, 66), 0.0),
            "ring": (QPointF(rect.left() + rect.width() * 0.61, rect.top() + rect.height() * 0.17), QSize(62, 62), 0.0),
            "pinky": (QPointF(rect.left() + rect.width() * 0.72, rect.top() + rect.height() * 0.17), QSize(58, 58), 0.0),
            "palm_1": (QPointF(rect.left() + rect.width() * 0.36, rect.top() + rect.height() * 0.45), QSize(50, 50), 0.0),
            "palm_2": (QPointF(rect.left() + rect.width() * 0.45, rect.top() + rect.height() * 0.55), QSize(50, 50), 0.0),
            "palm_3": (QPointF(rect.left() + rect.width() * 0.54, rect.top() + rect.height() * 0.46), QSize(50, 50), 0.0),
            "palm_4": (QPointF(rect.left() + rect.width() * 0.63, rect.top() + rect.height() * 0.56), QSize(50, 50), 0.0),
            "palm_5": (QPointF(rect.left() + rect.width() * 0.70, rect.top() + rect.height() * 0.47), QSize(50, 50), 0.0),
        }

    def _draw_hand_map(self, painter: QPainter, rect: QRectF) -> None:
        specs = self._hand_specs(rect)
        self._draw_hand_underlay(painter, rect)

        for name in ALL_SENSOR_NAMES:
            center, size, angle = specs[name]
            self._draw_sensor_zone(painter, center, size, angle, self.samples[name], DISPLAY_NAMES[name])

    def _draw_hand_underlay(self, painter: QPainter, rect: QRectF) -> None:
        specs = self._hand_specs(rect)
        painter.save()
        guide_pen = QPen(QColor(BrandPalette.BORDER.red(), BrandPalette.BORDER.green(), BrandPalette.BORDER.blue(), 30), 2.0)
        painter.setPen(guide_pen)
        painter.setBrush(QColor(255, 255, 255, 24))

        for name in ["index", "middle", "ring", "pinky"]:
            center, size, angle = specs[name]
            shaft = QRectF(
                center.x() - size.width() * 0.48,
                center.y() - size.height() * 0.48,
                size.width() * 0.96,
                size.height() * 2.31,
            )
            painter.save()
            painter.translate(center)
            painter.rotate(angle)
            painter.translate(-center)
            painter.drawRoundedRect(shaft, 28, 28)
            painter.restore()

        palm_path = QPainterPath()
        palm_path.addRoundedRect(
            QRectF(rect.left() + rect.width() * 0.28, rect.top() + rect.height() * 0.34, rect.width() * 0.50, rect.height() * 0.52),
            72,
            72,
        )
        painter.drawPath(palm_path)

        thumb_center, thumb_size, thumb_angle = specs["thumb"]
        thumb_rect = QRectF(
            thumb_center.x() - thumb_size.width() * 1.75,
            thumb_center.y() - thumb_size.height() * 0.52,
            thumb_size.width() * 2.00,
            thumb_size.height() * 1.04,
        )
        painter.save()
        painter.translate(thumb_center)
        painter.rotate(-28.0)
        painter.translate(-thumb_center)
        painter.drawRoundedRect(thumb_rect, 28, 28)
        painter.restore()

        painter.restore()

    def _draw_sensor_zone(
        self,
        painter: QPainter,
        center: QPointF,
        size: QSize,
        angle: float,
        sample: SensorSample,
        label: str,
    ) -> None:
        diameter = float(min(size.width(), size.height()))
        base_rect = QRectF(-diameter / 2.0, -diameter / 2.0, diameter, diameter)
        outer_color = QColor("#ECE7DB")
        border = QColor("#B8B0A2")
        heat = self.color_mapper.color_for(sample.normalized)

        painter.save()
        painter.translate(center)
        painter.rotate(angle)
        painter.setPen(QPen(border, 1.6))
        painter.setBrush(outer_color)
        painter.drawEllipse(base_rect)

        inset = 6.0
        heat_rect = base_rect.adjusted(inset, inset, -inset, -inset)
        painter.setClipRect(heat_rect)
        painter.setPen(Qt.NoPen)
        fill_gradient = QLinearGradient(heat_rect.topLeft(), heat_rect.bottomRight())
        fill_gradient.setColorAt(0.0, QColor(255, 255, 255, 120))
        fill_gradient.setColorAt(0.18, self.color_mapper._lerp(QColor("#FFFFFF"), heat, 0.35))
        fill_gradient.setColorAt(1.0, heat)
        painter.setBrush(fill_gradient)
        painter.drawEllipse(heat_rect)

        painter.setPen(QPen(QColor(255, 255, 255, 54), 1.0))
        for band_index in range(1, 4):
            y = heat_rect.top() + heat_rect.height() * (band_index / 4.0)
            painter.drawLine(QPointF(heat_rect.left() + 4, y), QPointF(heat_rect.right() - 4, y))
        painter.setClipping(False)
        painter.restore()

        if sample.normalized >= self.contact_threshold:
            painter.setPen(QPen(QColor(BrandPalette.BANANA.red(), BrandPalette.BANANA.green(), BrandPalette.BANANA.blue(), 190), 2.5))
            painter.setBrush(Qt.NoBrush)
            ring_r = diameter * 0.56
            painter.drawEllipse(center, ring_r, ring_r)

        painter.setPen(BrandPalette.TEXT)
        painter.setFont(QFont("DejaVu Sans", 8, QFont.DemiBold))
        painter.drawText(
            QRectF(center.x() - 46, center.y() - 14, 92, 16),
            Qt.AlignHCenter | Qt.AlignVCenter,
            label.upper(),
        )
        painter.setFont(QFont("Trebuchet MS", 13, QFont.Bold))
        painter.drawText(
            QRectF(center.x() - 40, center.y() + 2, 80, 20),
            Qt.AlignHCenter | Qt.AlignVCenter,
            f"{sample.filtered:.0f}",
        )

    def _draw_legend(self, painter: QPainter, rect: QRectF) -> None:
        painter.save()
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(255, 255, 255, 175))
        painter.drawRoundedRect(rect, 16, 16)

        bar = rect.adjusted(14, 10, -14, -28)
        painter.setBrush(QColor("#F0ECE1"))
        painter.drawRoundedRect(bar, 12, 12)
        gradient = QLinearGradient(bar.topLeft(), bar.topRight())
        gradient.setColorAt(0.0, self.color_mapper.color_for(0.0))
        gradient.setColorAt(0.5, self.color_mapper.color_for(0.5))
        gradient.setColorAt(1.0, self.color_mapper.color_for(1.0))
        painter.setBrush(gradient)
        color_bar = bar.adjusted(2, 2, -2, -2)
        painter.drawRoundedRect(color_bar, 10, 10)

        threshold_x = bar.left() + bar.width() * self.contact_threshold
        painter.setPen(QPen(BrandPalette.BORDER, 2.6))
        painter.drawLine(QPointF(threshold_x, bar.top() - 4), QPointF(threshold_x, bar.bottom() + 4))

        painter.setPen(BrandPalette.TEXT_MUTED)
        painter.setFont(QFont("DejaVu Sans", 9))
        painter.drawText(QRectF(bar.left(), rect.bottom() - 20, 60, 16), Qt.AlignLeft | Qt.AlignVCenter, "Low")
        painter.drawText(QRectF(bar.center().x() - 36, rect.bottom() - 20, 72, 16), Qt.AlignHCenter | Qt.AlignVCenter, "Medium")
        painter.drawText(QRectF(bar.right() - 60, rect.bottom() - 20, 60, 16), Qt.AlignRight | Qt.AlignVCenter, "High")
        painter.drawText(
            QRectF(threshold_x - 48, rect.top() - 2, 96, 12),
            Qt.AlignHCenter | Qt.AlignVCenter,
            "contact",
        )
        painter.restore()


class PositionPlotWidget(QWidget):
    def __init__(self, channel_name: str, color: QColor, window_seconds: float, y_min: float, y_max: float, parent: QWidget | None = None):
        super().__init__(parent)
        self.channel_name = channel_name
        self.color = color
        self.window_seconds = max(2.0, float(window_seconds))
        self.y_min = float(y_min)
        self.y_max = max(self.y_min + 1.0, float(y_max))
        self.history: Deque[tuple[float, float]] = deque()
        self.latest_value = 0.0
        self.setMinimumHeight(98)

    def append_sample(self, timestamp_s: float, value: float) -> None:
        self.history.append((timestamp_s, float(value)))
        self.latest_value = float(value)
        cutoff = timestamp_s - self.window_seconds
        while self.history and self.history[0][0] < cutoff:
            self.history.popleft()
        self.update()

    def paintEvent(self, event) -> None:
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        rect = self.rect()
        painter.fillRect(rect, Qt.transparent)

        outer = rect.adjusted(0, 0, -1, -1)
        painter.setPen(QPen(QColor(255, 255, 255, 16), 1))
        painter.setBrush(BrandPalette.PANEL_DARK)
        painter.drawRoundedRect(outer, 24, 24)

        label_width = self._label_strip_width(rect)
        plot_rect = rect.adjusted(label_width + 20, 16, -22, -18)
        painter.setPen(Qt.NoPen)
        inner_gradient = QLinearGradient(plot_rect.topLeft(), plot_rect.bottomLeft())
        inner_gradient.setColorAt(0.0, QColor("#22201C"))
        inner_gradient.setColorAt(1.0, QColor("#141311"))
        painter.setBrush(inner_gradient)
        painter.drawRoundedRect(plot_rect, 18, 18)

        self._draw_grid(painter, plot_rect)
        self._draw_label_strip(painter, rect, plot_rect, label_width)
        self._draw_axes_labels(painter, plot_rect)
        self._draw_trace(painter, plot_rect)

    def _label_strip_width(self, rect: QRectF) -> float:
        label_font = QFont("DejaVu Sans", 10, QFont.Bold)
        value_font = QFont("Trebuchet MS", 16, QFont.Bold)
        label_metrics = QFontMetrics(label_font)
        value_metrics = QFontMetrics(value_font)
        min_width = max(label_metrics.horizontalAdvance(DISPLAY_NAMES[self.channel_name]), value_metrics.horizontalAdvance("4095"))
        target = min_width + 28
        return max(98.0, min(rect.width() * 0.28, float(target)))

    def _draw_label_strip(self, painter: QPainter, rect: QRectF, plot_rect: QRectF, label_width: float) -> None:
        label_rect = QRectF(rect.left() + 10, plot_rect.top(), label_width, plot_rect.height())
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(255, 255, 255, 10))
        painter.drawRoundedRect(label_rect, 14, 14)
        painter.setPen(self.color)
        painter.setFont(QFont("DejaVu Sans", 10, QFont.Bold))
        painter.drawText(label_rect.adjusted(10, 14, -8, -24), Qt.AlignLeft | Qt.AlignTop, DISPLAY_NAMES[self.channel_name])
        painter.setPen(QColor("#EAE1B8"))
        painter.setFont(QFont("Trebuchet MS", 16, QFont.Bold))
        painter.drawText(label_rect.adjusted(10, 34, -8, -10), Qt.AlignLeft | Qt.AlignTop, f"{self.latest_value:.0f}")

    def _draw_grid(self, painter: QPainter, plot_rect: QRectF) -> None:
        painter.save()
        painter.setPen(QPen(QColor(255, 255, 255, 26), 1))
        for i in range(1, 5):
            y = plot_rect.top() + plot_rect.height() * i / 5.0
            painter.drawLine(QPointF(plot_rect.left(), y), QPointF(plot_rect.right(), y))
        for i in range(1, 6):
            x = plot_rect.left() + plot_rect.width() * i / 6.0
            painter.drawLine(QPointF(x, plot_rect.top()), QPointF(x, plot_rect.bottom()))
        painter.restore()

    def _draw_axes_labels(self, painter: QPainter, plot_rect: QRectF) -> None:
        current_y_max = self._effective_y_max()
        painter.setPen(QColor("#BDB6A7"))
        painter.setFont(QFont("DejaVu Sans", 8))
        painter.drawText(QRectF(plot_rect.left(), plot_rect.top() - 14, 72, 12), Qt.AlignLeft, f"{current_y_max:.0f}")
        painter.drawText(QRectF(plot_rect.left(), plot_rect.bottom() + 2, 56, 12), Qt.AlignLeft, f"{self.y_min:.0f}")
        painter.drawText(
            QRectF(plot_rect.right() - 90, plot_rect.bottom() + 2, 90, 12),
            Qt.AlignRight,
            f"{self.window_seconds:.0f}s",
        )

    def _draw_trace(self, painter: QPainter, plot_rect: QRectF) -> None:
        now = self.history[-1][0] if self.history else 0.0
        if now <= 0.0:
            painter.setPen(QColor("#8B8578"))
            painter.setFont(QFont("DejaVu Sans", 10))
            painter.drawText(plot_rect, Qt.AlignCenter, "Waiting for `rx_positions` samples")
            return

        x_min = now - self.window_seconds
        y_min = self.y_min
        y_max = self._effective_y_max()
        span = max(1e-6, y_max - y_min)

        if len(self.history) < 2:
            return

        path = QPainterPath()
        first = True
        last_point = QPointF(plot_rect.left(), plot_rect.bottom())
        for t, value in self.history:
            x_ratio = (t - x_min) / self.window_seconds
            x = plot_rect.left() + max(0.0, min(1.0, x_ratio)) * plot_rect.width()
            y_ratio = (value - y_min) / span
            y = plot_rect.bottom() - max(0.0, min(1.0, y_ratio)) * plot_rect.height()
            last_point = QPointF(x, y)
            if first:
                path.moveTo(x, y)
                first = False
            else:
                path.lineTo(x, y)

        painter.setPen(QPen(QColor(self.color.red(), self.color.green(), self.color.blue(), 72), 6, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        painter.drawPath(path)
        painter.setPen(QPen(self.color, 2.0, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        painter.drawPath(path)

        painter.setPen(Qt.NoPen)
        painter.setBrush(self.color)
        painter.drawEllipse(last_point, 4.5, 4.5)

    def _effective_y_max(self) -> float:
        max_seen = self.y_max
        for _, value in self.history:
            if value > max_seen:
                max_seen = value
        return max(self.y_min + 1.0, max_seen * 1.08)


class RosDashboardNode(Node):
    def __init__(self, context: Context | None = None):
        super().__init__("banana_fsr_visualizer", context=context)

        self.declare_parameter("topic_name", "rx_force")
        self.declare_parameter("position_topic_name", "rx_positions")
        self.declare_parameter("position_message_type", "joint_state")
        self.declare_parameter("finger_indices", [0, 1, 2, 3, 4])
        self.declare_parameter("palm_indices", [-5, -4, -3, -2, -1])
        self.declare_parameter("position_indices", [0, 1, 2, 3, 4, 5])
        self.declare_parameter("alpha", 0.0)
        self.declare_parameter("position_alpha", 0.0)
        self.declare_parameter("global_min", 0.0)
        self.declare_parameter("global_max", 4095.0)
        self.declare_parameter("position_min", 0.0)
        self.declare_parameter("position_max", 4095.0)
        self.declare_parameter("plot_window_seconds", 8.0)
        self.declare_parameter("per_finger_min", [])
        self.declare_parameter("per_finger_max", [])
        self.declare_parameter("refresh_hz", 30.0)
        self.declare_parameter("ui_refresh_hz", 25.0)
        self.declare_parameter("status_refresh_hz", 4.0)
        self.declare_parameter("contact_threshold", 0.7)
        self.declare_parameter("simulate_if_no_data", False)

        self.topic_name = str(self.get_parameter("topic_name").value)
        self.position_topic_name = str(self.get_parameter("position_topic_name").value)
        self.position_message_type = str(self.get_parameter("position_message_type").value).strip().lower()

        self.finger_indices = self._sanitize_indices("finger_indices", self.get_parameter("finger_indices").value, 5)
        self.palm_indices = self._sanitize_indices("palm_indices", self.get_parameter("palm_indices").value, 5)
        self.position_indices = self._sanitize_indices("position_indices", self.get_parameter("position_indices").value, 6)

        self.alpha = max(0.0, min(1.0, float(self.get_parameter("alpha").value)))
        self.position_alpha = max(0.0, min(1.0, float(self.get_parameter("position_alpha").value)))
        self.refresh_hz = max(5.0, float(self.get_parameter("refresh_hz").value))
        self.ui_refresh_hz = min(30.0, max(20.0, float(self.get_parameter("ui_refresh_hz").value)))
        self.status_refresh_hz = min(10.0, max(1.0, float(self.get_parameter("status_refresh_hz").value)))
        self.contact_threshold = max(0.0, min(1.0, float(self.get_parameter("contact_threshold").value)))
        self.plot_window_seconds = max(2.0, float(self.get_parameter("plot_window_seconds").value))
        self.position_min = float(self.get_parameter("position_min").value)
        self.position_max = float(self.get_parameter("position_max").value)
        self.simulate_if_no_data = bool(self.get_parameter("simulate_if_no_data").value)

        global_min = float(self.get_parameter("global_min").value)
        global_max = float(self.get_parameter("global_max").value)
        per_sensor_min, per_sensor_max = self._build_per_sensor_limits()
        self.normalizer = ForceNormalizer(global_min, global_max, per_sensor_min, per_sensor_max)

        self.samples: Dict[str, SensorSample] = {name: SensorSample() for name in ALL_SENSOR_NAMES}
        self.raw_position_values = [0.0] * len(POSITION_NAMES)
        self.position_values = [0.0] * len(POSITION_NAMES)
        self.position_sequence = 0
        self.force_dirty = False
        self.position_dirty = False

        self.force_snapshot = TopicSnapshot()
        self.position_snapshot = TopicSnapshot()
        self.start_time = self._now_seconds()
        self._sim_phase = 0.0

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.force_sub = self.create_subscription(UInt16MultiArray, self.topic_name, self._on_force_msg, qos)
        self.position_sub = self._create_position_subscription(qos)

        if self.simulate_if_no_data:
            self.create_timer(0.05, self._simulate_if_stale)

        self.get_logger().info(
            f"BananaHand dashboard listening on force='{self.topic_name}', positions='{self.position_topic_name}' ({self.position_message_type})"
        )

    def _sanitize_indices(self, name: str, values: Sequence[int], expected_length: int) -> List[int]:
        cleaned = [int(v) for v in values]
        if len(cleaned) != expected_length:
            fallback = list(range(expected_length))
            if name == "palm_indices":
                fallback = [-5, -4, -3, -2, -1]
            self.get_logger().warn(f"{name} must have length {expected_length}; using {fallback}")
            return fallback
        return cleaned

    def _build_per_sensor_limits(self) -> tuple[Dict[str, float], Dict[str, float]]:
        min_vals = [float(v) for v in self.get_parameter("per_finger_min").value]
        max_vals = [float(v) for v in self.get_parameter("per_finger_max").value]
        per_sensor_min: Dict[str, float] = {}
        per_sensor_max: Dict[str, float] = {}
        if len(min_vals) == 5:
            per_sensor_min = {name: value for name, value in zip(FINGER_NAMES, min_vals)}
        if len(max_vals) == 5:
            per_sensor_max = {name: value for name, value in zip(FINGER_NAMES, max_vals)}
        return per_sensor_min, per_sensor_max

    def _create_position_subscription(self, qos: QoSProfile):
        if self.position_message_type == "uint16_multi_array":
            return self.create_subscription(
                UInt16MultiArray,
                self.position_topic_name,
                self._on_position_array_msg,
                qos,
            )
        return self.create_subscription(JointState, self.position_topic_name, self._on_position_joint_state, qos)

    def _extract_values(self, data: Sequence[float], indices: Sequence[int]) -> List[float]:
        values: List[float] = []
        length = len(data)
        for index in indices:
            resolved = index if index >= 0 else length + index
            if 0 <= resolved < length:
                values.append(float(data[resolved]))
            else:
                values.append(0.0)
        return values

    def _apply_smoothing(self, name: str, raw: float) -> SensorSample:
        normalized = self.normalizer.normalize(name, raw)
        sample = SensorSample(raw=raw, filtered=raw, normalized=normalized)
        self.samples[name] = sample
        return sample

    def _on_force_msg(self, msg: UInt16MultiArray) -> None:
        values = [float(v) for v in msg.data]
        finger_values = self._extract_values(values, self.finger_indices)
        palm_values = self._extract_values(values, self.palm_indices)

        for name, value in zip(FINGER_NAMES, finger_values):
            self._apply_smoothing(name, value)
        for name, value in zip(PALM_NAMES, palm_values):
            self._apply_smoothing(name, value)

        self.force_snapshot.count += 1
        self.force_snapshot.last_time_s = self._now_seconds()
        self.force_dirty = True

    def _on_position_joint_state(self, msg: JointState) -> None:
        self._update_positions(msg.position)

    def _on_position_array_msg(self, msg: UInt16MultiArray) -> None:
        self._update_positions(msg.data)

    def _update_positions(self, values: Sequence[float]) -> None:
        extracted = self._extract_values(values, self.position_indices)
        self.raw_position_values = list(extracted)
        self.position_values = list(extracted)

        self.position_snapshot.count += 1
        self.position_snapshot.last_time_s = self._now_seconds()
        self.position_sequence += 1
        self.position_dirty = True

    def _simulate_if_stale(self) -> None:
        now = self._now_seconds()
        if now - self.force_snapshot.last_time_s > 0.45:
            self._sim_phase += 0.12
            force_base = [0.72, 0.58, 0.76, 0.49, 0.41]
            force_amp = [0.18, 0.26, 0.16, 0.23, 0.20]
            for index, name in enumerate(FINGER_NAMES):
                ratio = force_base[index] + force_amp[index] * math.sin(self._sim_phase * (0.9 + index * 0.13))
                value = self.normalizer.denormalize(max(0.0, min(1.0, ratio)))
                self._apply_smoothing(name, value)

            palm_base = [0.36, 0.50, 0.61, 0.45, 0.55]
            palm_amp = [0.14, 0.15, 0.18, 0.12, 0.11]
            for index, name in enumerate(PALM_NAMES):
                ratio = palm_base[index] + palm_amp[index] * math.sin(self._sim_phase * (1.1 + index * 0.09) + 0.7)
                value = self.normalizer.denormalize(max(0.0, min(1.0, ratio)))
                self._apply_smoothing(name, value)

            self.force_snapshot.count += 1
            self.force_snapshot.last_time_s = now
            self.force_dirty = True
        if now - self.position_snapshot.last_time_s > 0.45:
            self._sim_phase += 0.06
            self.raw_position_values = [
                1800.0 + 900.0 * (0.5 + 0.5 * math.sin(self._sim_phase * (1.0 + idx * 0.11) + idx * 0.5))
                for idx in range(len(POSITION_NAMES))
            ]
            self.position_values = list(self.raw_position_values)
            self.position_snapshot.count += 1
            self.position_snapshot.last_time_s = now
            self.position_sequence += 1
            self.position_dirty = True

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def seconds_since(self, snapshot: TopicSnapshot) -> float | None:
        if snapshot.last_time_s <= 0.0:
            return None
        return max(0.0, self._now_seconds() - snapshot.last_time_s)

    def is_live(self, snapshot: TopicSnapshot, timeout_s: float = 1.0) -> bool:
        age = self.seconds_since(snapshot)
        return age is not None and age <= timeout_s


class MainWindow(QMainWindow):
    def __init__(self, ros_node: RosDashboardNode):
        super().__init__()
        self.ros_node = ros_node
        self.color_mapper = ForceColorMapper()
        self.executor = SingleThreadedExecutor(context=self.ros_node.context)
        self.executor.add_node(self.ros_node)
        self._last_position_sequence = -1
        self._last_force_status = ""
        self._last_position_status = ""
        self._last_latest_status = ""
        self._last_position_snapshot_text = ""
        self._last_force_badge_style = ""
        self._last_position_badge_style = ""
        self._last_latest_badge_style = ""

        self.setWindowTitle("BananaHand Telemetry Dashboard")
        self.resize(1500, 920)
        self.setMinimumSize(1500, 920)
        self.setMaximumSize(1500, 920)
        self._build_ui()
        self._apply_styles()

        spin_interval_ms = max(5, int(1000.0 / self.ros_node.refresh_hz))
        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self._spin_ros_once)
        self.spin_timer.start(spin_interval_ms)

        self.gui_timer = QTimer(self)
        self.gui_timer.timeout.connect(self._refresh_gui)
        self.gui_timer.start(int(1000.0 / self.ros_node.ui_refresh_hz))

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._update_status_badges)
        self.status_timer.start(int(1000.0 / self.ros_node.status_refresh_hz))

    def _build_ui(self) -> None:
        central = QWidget(self)
        root = QVBoxLayout(central)
        root.setContentsMargins(24, 24, 24, 24)
        root.setSpacing(18)

        header = QFrame()
        header.setObjectName("headerCard")
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(20, 16, 20, 16)
        header_layout.setSpacing(18)

        logo = BananaLogoWidget()
        logo.setFixedSize(110, 90)
        header_layout.addWidget(logo, 0, Qt.AlignTop)

        title_stack = QVBoxLayout()
        title_stack.setSpacing(3)

        eyebrow = QLabel("BANANAHAND ROBOTICS")
        eyebrow.setObjectName("eyebrow")

        title = QLabel("Force + Position Telemetry")
        title.setObjectName("windowTitle")

        subtitle = QLabel("Dashboard for force sensors and live actuator positions.")
        subtitle.setObjectName("subtitle")
        subtitle.setWordWrap(True)
        subtitle.setMaximumWidth(520)

        title_stack.addWidget(eyebrow)
        title_stack.addWidget(title)
        title_stack.addWidget(subtitle)
        header_layout.addLayout(title_stack, 1)

        badge_column = QVBoxLayout()
        badge_column.setSpacing(8)
        badge_column.setAlignment(Qt.AlignRight | Qt.AlignTop)

        self.force_badge = QLabel()
        self.position_badge = QLabel()
        self.latest_badge = QLabel()
        for badge in [self.force_badge, self.position_badge, self.latest_badge]:
            badge.setObjectName("statusBadge")
            badge.setMinimumWidth(320)
            badge.setMaximumWidth(420)
            badge_column.addWidget(badge)

        header_layout.addLayout(badge_column)
        root.addWidget(header)

        body = QHBoxLayout()
        body.setSpacing(18)

        left_column = QVBoxLayout()
        left_column.setSpacing(18)

        metrics_row = QHBoxLayout()
        metrics_row.setSpacing(12)
        self.max_force_card = MetricCard("Max Force", BrandPalette.BANANA)
        self.active_contacts_card = MetricCard("Active Contacts", BrandPalette.WARNING)
        self.avg_force_card = MetricCard("Average Force", BrandPalette.BANANA_DEEP)
        self.loaded_finger_card = MetricCard("Most Loaded Finger", BrandPalette.ACCENT)
        for card in [
            self.max_force_card,
            self.active_contacts_card,
            self.avg_force_card,
            self.loaded_finger_card,
        ]:
            metrics_row.addWidget(card, 1)

        left_column.addLayout(metrics_row)

        force_card = QFrame()
        force_card.setObjectName("forceCard")
        force_layout = QVBoxLayout(force_card)
        force_layout.setContentsMargins(18, 18, 18, 18)
        force_layout.setSpacing(14)

        force_header = QHBoxLayout()
        force_header.setSpacing(12)
        force_title = QLabel("Force Dashboard")
        force_title.setObjectName("sectionTitle")
        force_desc = QLabel("Branded hand schematic with fingertip and palm FSR pads.")
        force_desc.setObjectName("sectionMeta")
        force_header_col = QVBoxLayout()
        force_header_col.setSpacing(2)
        force_header_col.addWidget(force_title)
        force_header_col.addWidget(force_desc)
        force_header.addLayout(force_header_col, 1)
        force_layout.addLayout(force_header)

        self.hand_widget = HandVisualizerWidget(self.color_mapper, self.ros_node.contact_threshold)
        force_layout.addWidget(self.hand_widget, 1)

        left_column.addWidget(force_card, 1)

        right_column = QVBoxLayout()
        right_column.setSpacing(18)

        plot_card = QFrame()
        plot_card.setObjectName("plotCard")
        plot_layout = QVBoxLayout(plot_card)
        plot_layout.setContentsMargins(18, 18, 18, 18)
        plot_layout.setSpacing(14)

        plot_header = QHBoxLayout()
        plot_title_col = QVBoxLayout()
        plot_title_col.setSpacing(2)
        plot_title = QLabel("Position Traces")
        plot_title.setObjectName("sectionTitle")
        plot_meta = QLabel("First six `rx_positions` channels: thumb, index, middle, ring, pinky, opposition.")
        plot_meta.setObjectName("sectionMeta")
        plot_meta.setWordWrap(True)
        plot_title_col.addWidget(plot_title)
        plot_title_col.addWidget(plot_meta)

        self.position_snapshot_label = QLabel("Latest values will appear here")
        self.position_snapshot_label.setObjectName("snapshotLabel")
        self.position_snapshot_label.setAlignment(Qt.AlignRight | Qt.AlignTop)
        self.position_snapshot_label.setWordWrap(True)
        self.position_snapshot_label.setFixedWidth(260)

        plot_header.addLayout(plot_title_col, 1)
        plot_header.addWidget(self.position_snapshot_label, 0, Qt.AlignTop)
        plot_layout.addLayout(plot_header)

        self.plot_legend = TraceLegend()
        plot_layout.addWidget(self.plot_legend)

        self.plot_widget = PositionPlotsPanel(
            self.ros_node.plot_window_seconds,
            self.ros_node.position_min,
            self.ros_node.position_max,
        )
        plot_layout.addWidget(self.plot_widget, 1)
        right_column.addWidget(plot_card, 1)

        body.addLayout(left_column, 8)
        body.addLayout(right_column, 4)
        root.addLayout(body, 1)

        self.setCentralWidget(central)

    def _apply_styles(self) -> None:
        self.setStyleSheet(
            f"""
            QMainWindow, QWidget {{
                background: {BrandPalette.APP_BG.name()};
                color: {BrandPalette.TEXT.name()};
                font-family: 'DejaVu Sans';
            }}
            QLabel {{
                background: transparent;
            }}
            QFrame#headerCard, QFrame#forceCard, QFrame#plotCard, QFrame#metricCard {{
                background: {BrandPalette.PANEL.name()};
                border: 1px solid rgba(17, 16, 14, 32);
                border-radius: 24px;
            }}
            QLabel#eyebrow {{
                color: {BrandPalette.TEXT_MUTED.name()};
                font-size: 12px;
                font-weight: 700;
                letter-spacing: 1px;
            }}
            QLabel#windowTitle {{
                color: {BrandPalette.TEXT.name()};
                font-family: 'Trebuchet MS';
                font-size: 30px;
                font-weight: 800;
            }}
            QLabel#subtitle {{
                color: {BrandPalette.TEXT_MUTED.name()};
                font-size: 13px;
            }}
            QLabel#statusBadge {{
                background: {BrandPalette.ACCENT.name()};
                color: #FFF9E4;
                border-radius: 13px;
                padding: 9px 12px;
                font-size: 12px;
                font-weight: 700;
            }}
            QLabel#sectionTitle {{
                color: {BrandPalette.TEXT.name()};
                font-family: 'Trebuchet MS';
                font-size: 22px;
                font-weight: 800;
            }}
            QLabel#sectionMeta {{
                color: {BrandPalette.TEXT_MUTED.name()};
                font-size: 12px;
            }}
            QLabel#snapshotLabel {{
                color: {BrandPalette.SOFT_TEXT.name()};
                font-size: 12px;
                padding-top: 3px;
            }}
            QLabel#metricTitle {{
                font-size: 11px;
                font-weight: 700;
                text-transform: uppercase;
            }}
            QLabel#metricValue {{
                color: {BrandPalette.TEXT.name()};
                font-family: 'Trebuchet MS';
                font-size: 26px;
                font-weight: 900;
            }}
            QLabel#metricMeta {{
                font-size: 11px;
            }}
            """
        )

    def _spin_ros_once(self) -> None:
        if not self.ros_node.context.ok():
            self.spin_timer.stop()
            self.gui_timer.stop()
            self.status_timer.stop()
            return
        try:
            self.executor.spin_once(timeout_sec=0.0)
        except RCLError:
            self.spin_timer.stop()
            self.gui_timer.stop()
            self.status_timer.stop()

    def _refresh_gui(self) -> None:
        if self.ros_node.force_dirty:
            self.hand_widget.set_samples(self.ros_node.samples, self.ros_node.is_live(self.ros_node.force_snapshot))
            self._update_metrics()
            self.ros_node.force_dirty = False
        if self.ros_node.position_dirty:
            self._update_positions()
            self.ros_node.position_dirty = False

    def _update_metrics(self) -> None:
        force_values = [self.ros_node.samples[name].filtered for name in ALL_SENSOR_NAMES]
        normalized = [self.ros_node.samples[name].normalized for name in ALL_SENSOR_NAMES]
        max_index = max(range(len(force_values)), key=lambda idx: force_values[idx]) if force_values else 0
        max_name = ALL_SENSOR_NAMES[max_index]
        avg_force = sum(force_values) / max(1, len(force_values))
        active_contacts = sum(1 for value in normalized if value >= self.ros_node.contact_threshold)
        max_force = force_values[max_index] if force_values else 0.0

        finger_loads = {name: self.ros_node.samples[name].filtered for name in FINGER_NAMES}
        loaded_name = max(finger_loads, key=finger_loads.get) if finger_loads else "thumb"

        self.max_force_card.set_content(f"{max_force:.0f}", DISPLAY_NAMES[max_name])
        self.active_contacts_card.set_content(str(active_contacts), f"threshold {self.ros_node.contact_threshold:.2f}")
        self.avg_force_card.set_content(f"{avg_force:.0f}", "across 10 sensors")
        self.loaded_finger_card.set_content(DISPLAY_NAMES[loaded_name], f"{finger_loads.get(loaded_name, 0.0):.0f} units")

    def _update_status_badges(self) -> None:
        force_age = self._format_age(self.ros_node.seconds_since(self.ros_node.force_snapshot))
        position_age = self._format_age(self.ros_node.seconds_since(self.ros_node.position_snapshot))
        force_live = self.ros_node.is_live(self.ros_node.force_snapshot)
        position_live = self.ros_node.is_live(self.ros_node.position_snapshot)
        available_ages = [
            age
            for age in [
                self.ros_node.seconds_since(self.ros_node.force_snapshot),
                self.ros_node.seconds_since(self.ros_node.position_snapshot),
            ]
            if age is not None
        ]
        latest_age = min(available_ages) if available_ages else None

        force_text = f"Force topic  {'LIVE' if force_live else 'IDLE'}  |  {self.ros_node.topic_name}  |  {force_age}"
        position_text = f"Position topic  {'LIVE' if position_live else 'IDLE'}  |  {self.ros_node.position_topic_name}  |  {position_age}"
        latest_text = f"Latest update  |  {self._format_age(latest_age)}  |  mode: {self.ros_node.position_message_type}"

        active_bg = BrandPalette.ACCENT.name()
        idle_bg = "#CFC9BE"
        active_fg = "#FFF9E0"
        idle_fg = BrandPalette.TEXT_MUTED.name()

        force_style = f"QLabel#statusBadge {{ background: {active_bg if force_live else idle_bg}; color: {active_fg if force_live else idle_fg}; border-radius: 13px; padding: 9px 12px; font-size: 12px; font-weight: 700; }}"
        position_style = f"QLabel#statusBadge {{ background: {active_bg if position_live else idle_bg}; color: {active_fg if position_live else idle_fg}; border-radius: 13px; padding: 9px 12px; font-size: 12px; font-weight: 700; }}"
        latest_style = f"QLabel#statusBadge {{ background: {BrandPalette.BANANA.name()}; color: {BrandPalette.TEXT.name()}; border-radius: 13px; padding: 9px 12px; font-size: 12px; font-weight: 800; }}"

        if self._last_force_status != force_text:
            self.force_badge.setText(force_text)
            self._last_force_status = force_text
        if self._last_position_status != position_text:
            self.position_badge.setText(position_text)
            self._last_position_status = position_text
        if self._last_latest_status != latest_text:
            self.latest_badge.setText(latest_text)
            self._last_latest_status = latest_text

        if self._last_force_badge_style != force_style:
            self.force_badge.setStyleSheet(force_style)
            self._last_force_badge_style = force_style
        if self._last_position_badge_style != position_style:
            self.position_badge.setStyleSheet(position_style)
            self._last_position_badge_style = position_style
        if self._last_latest_badge_style != latest_style:
            self.latest_badge.setStyleSheet(latest_style)
            self._last_latest_badge_style = latest_style

    def _update_positions(self) -> None:
        if self.ros_node.position_snapshot.count > 0 and self.ros_node.position_sequence != self._last_position_sequence:
            self.plot_widget.append_sample(self.ros_node.position_snapshot.last_time_s, self.ros_node.position_values)
            self._last_position_sequence = self.ros_node.position_sequence

        latest_text = "\n".join(
            f"{DISPLAY_NAMES[name]} {value:.0f}" for name, value in zip(POSITION_NAMES, self.ros_node.position_values)
        )
        if self._last_position_snapshot_text != latest_text:
            self.position_snapshot_label.setText(latest_text)
            self._last_position_snapshot_text = latest_text

    def _format_age(self, seconds: float | None) -> str:
        if seconds is None:
            return "waiting"
        if seconds < 1.0:
            return f"{int(seconds * 1000)} ms ago"
        return f"{seconds:.1f} s ago"

    def closeEvent(self, event) -> None:
        self.spin_timer.stop()
        self.gui_timer.stop()
        self.status_timer.stop()
        try:
            self.executor.remove_node(self.ros_node)
            self.executor.shutdown(timeout_sec=0.0)
        except Exception:
            pass
        super().closeEvent(event)


def main() -> None:
    context = Context()
    rclpy.init(context=context)
    node = RosDashboardNode(context=context)

    app = QApplication(sys.argv)
    app.setApplicationDisplayName("BananaHand Dashboard")
    window = MainWindow(node)
    window.show()

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
