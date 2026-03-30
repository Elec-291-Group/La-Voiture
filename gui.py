import sys
import math
import csv
import time
import re
from pathlib import Path
from datetime import datetime
from collections import deque

from PyQt5.QtCore import Qt, QTimer, QRectF
from PyQt5.QtGui import QColor, QPainter, QPen, QBrush, QLinearGradient, QFont
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QComboBox, QTextEdit, QPlainTextEdit,
    QFileDialog, QMessageBox,
    QGroupBox, QCheckBox, QSlider, QDoubleSpinBox, QTabWidget,
    QSizePolicy
)

ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.append(str(ROOT_DIR))

from ble_receiver import BleWorker, DEFAULT_DEVICE_NAME, scan_ble_devices
from pathfinder import PathfinderTab


# ── Dark theme stylesheet ────────────────────────────────────────────────────
LIGHT_STYLESHEET = """
/* ── Global ── */
QWidget {
    background-color: #f8f9fb;
    color: #1e293b;
    font-family: 'Segoe UI', 'Inter', 'Helvetica Neue', sans-serif;
    font-size: 22px;
}

/* ── Tab widget ── */
QTabWidget::pane {
    border: 1px solid #e2e8f0;
    border-radius: 8px;
    background: #f8f9fb;
    top: -1px;
}
QTabBar::tab {
    background: #eef1f5;
    color: #64748b;
    border: 1px solid #e2e8f0;
    border-bottom: none;
    padding: 14px 40px;
    margin-right: 2px;
    border-top-left-radius: 8px;
    border-top-right-radius: 8px;
    font-weight: 600;
    font-size: 20px;
}
QTabBar::tab:selected {
    background: #ffffff;
    color: #2563eb;
    border-bottom: 2px solid #2563eb;
}
QTabBar::tab:hover:!selected {
    background: #e8ecf1;
    color: #475569;
}

/* ── Group boxes ── */
QGroupBox {
    background: #ffffff;
    border: 1px solid #e2e8f0;
    border-radius: 10px;
    margin-top: 18px;
    padding: 18px 14px 12px 14px;
    font-weight: 700;
    font-size: 22px;
    color: #475569;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 16px;
    padding: 2px 10px;
    background: #eef2ff;
    border-radius: 4px;
    color: #2563eb;
    font-size: 18px;
    letter-spacing: 1px;
    text-transform: uppercase;
}

/* ── Buttons ── */
QPushButton {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #ffffff, stop:1 #f1f5f9);
    color: #334155;
    border: 1px solid #cbd5e1;
    border-radius: 7px;
    padding: 14px 24px;
    font-weight: 600;
    font-size: 20px;
    min-height: 29px;
}
QPushButton:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #f8fafc, stop:1 #e2e8f0);
    border-color: #94a3b8;
    color: #1e293b;
}
QPushButton:pressed {
    background: #e2e8f0;
    border-color: #2563eb;
}
QPushButton:disabled {
    background: #f1f5f9;
    color: #94a3b8;
    border-color: #e2e8f0;
}

/* ── Primary action buttons ── */
QPushButton#connectBtn {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #3b82f6, stop:1 #2563eb);
    color: #ffffff;
    border: 1px solid #2563eb;
}
QPushButton#connectBtn:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #60a5fa, stop:1 #3b82f6);
    border-color: #3b82f6;
}
QPushButton#connectBtn:disabled {
    background: #f1f5f9;
    color: #94a3b8;
    border-color: #e2e8f0;
}

QPushButton#disconnectBtn {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #fef2f2, stop:1 #fee2e2);
    color: #dc2626;
    border: 1px solid #fca5a5;
}
QPushButton#disconnectBtn:hover {
    background: #fee2e2;
    border-color: #f87171;
}
QPushButton#disconnectBtn:disabled {
    background: #f1f5f9;
    color: #94a3b8;
    border-color: #e2e8f0;
}

QPushButton#estopBtn {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #ef4444, stop:1 #dc2626);
    color: #ffffff;
    border: 2px solid #dc2626;
    border-radius: 8px;
    font-size: 22px;
    font-weight: 800;
    padding: 14px;
    letter-spacing: 1px;
}
QPushButton#estopBtn:hover {
    background: #f87171;
    border-color: #ef4444;
}

/* ── Combo box ── */
QComboBox {
    background: #ffffff;
    border: 1px solid #cbd5e1;
    border-radius: 7px;
    padding: 7px 12px;
    color: #1e293b;
    font-size: 22px;
    min-height: 28px;
}
QComboBox:hover {
    border-color: #94a3b8;
}
QComboBox::drop-down {
    border: none;
    width: 28px;
}
QComboBox::down-arrow {
    image: none;
    border-left: 5px solid transparent;
    border-right: 5px solid transparent;
    border-top: 6px solid #64748b;
    margin-right: 8px;
}
QComboBox QAbstractItemView {
    background: #ffffff;
    border: 1px solid #cbd5e1;
    border-radius: 6px;
    color: #1e293b;
    selection-background-color: #dbeafe;
    selection-color: #1d4ed8;
}

/* ── Text area / Console ── */
QTextEdit {
    background: #ffffff;
    color: #047857;
    border: 1px solid #e2e8f0;
    border-radius: 8px;
    padding: 10px;
    font-family: 'Cascadia Code', 'Fira Code', 'Consolas', monospace;
    font-size: 20px;
    selection-background-color: #bfdbfe;
}

/* ── Labels ── */
QLabel {
    color: #1e293b;
    background: transparent;
    border: none;
}

/* ── Checkboxes ── */
QCheckBox {
    color: #475569;
    spacing: 8px;
    font-size: 12px;
}
QCheckBox::indicator {
    width: 18px;
    height: 18px;
    border: 2px solid #cbd5e1;
    border-radius: 4px;
    background: #ffffff;
}
QCheckBox::indicator:checked {
    background: #2563eb;
    border-color: #3b82f6;
}
QCheckBox::indicator:hover {
    border-color: #3b82f6;
}

/* ── Sliders ── */
QSlider::groove:horizontal {
    height: 6px;
    background: #e2e8f0;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: #3b82f6;
    border: 2px solid #2563eb;
    width: 16px;
    height: 16px;
    margin: -6px 0;
    border-radius: 9px;
}
QSlider::handle:horizontal:hover {
    background: #60a5fa;
    border-color: #3b82f6;
}
QSlider::sub-page:horizontal {
    background: #3b82f6;
    border-radius: 3px;
}

/* ── Spin boxes ── */
QDoubleSpinBox {
    background: #ffffff;
    border: 1px solid #cbd5e1;
    border-radius: 6px;
    padding: 5px 8px;
    color: #1e293b;
    font-size: 12px;
    min-height: 18px;
}
QDoubleSpinBox:hover {
    border-color: #94a3b8;
}
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
    background: #f1f5f9;
    border: none;
    width: 18px;
}

/* ── Scrollbars ── */
QScrollBar:vertical {
    background: #f8f9fb;
    width: 8px;
    border-radius: 4px;
}
QScrollBar::handle:vertical {
    background: #cbd5e1;
    border-radius: 4px;
    min-height: 30px;
}
QScrollBar::handle:vertical:hover {
    background: #94a3b8;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}

/* ── Separator lines ── */
QFrame#separator {
    background: #e2e8f0;
    max-height: 1px;
    min-height: 1px;
}
"""


# ── Solarbotics GM4 physical constants ──────────────────────────────────────
_WHEELBASE_CM    = 10.6
_WHEEL_RADIUS_CM = 3.3
_MAX_RPM         = 77.0
_MAX_SPEED_CM_S  = _MAX_RPM * 2 * math.pi * _WHEEL_RADIUS_CM / 60.0  # ≈ 26.6 cm/s


def compute_motion_state(left: float, right: float, threshold: float = 5.0) -> dict:
    v_left  = (left  / 100.0) * _MAX_SPEED_CM_S
    v_right = (right / 100.0) * _MAX_SPEED_CM_S
    linear_vel  = (v_left + v_right) / 2.0
    angular_vel = (v_left - v_right) / _WHEELBASE_CM
    speed_pct   = abs(left + right) / 2.0
    diff        = left - right
    stopped     = abs(left) < threshold and abs(right) < threshold
    turn_angle  = math.degrees(math.atan2(diff, abs(left + right) + 1e-6))
    if stopped:
        direction = 'stopped'
    elif abs(diff) < threshold:
        direction = 'forward' if linear_vel > 0 else 'reverse'
    elif speed_pct < threshold:
        direction = 'spin_right' if diff > 0 else 'spin_left'
    else:
        direction = 'turn_right' if diff > 0 else 'turn_left'
    return {
        'direction':        direction,
        'linear_vel_cms':   round(linear_vel, 2),
        'angular_vel_rads': round(angular_vel, 4),
        'turn_angle_deg':   round(turn_angle, 1),
    }


def compute_roll_pitch(ax, ay, az):
    try:
        roll = math.degrees(math.atan2(ay, az))
        pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
    except Exception:
        roll = 0.0
        pitch = 0.0
    return roll, pitch


class RobotVisualizationWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(560, 640)
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.accel = (0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.connected = False
        self.moving = False
        self.left_power = 0
        self.right_power = 0
        self.road_offset = 0.0
        self._anim_timer = QTimer(self)
        self._anim_timer.timeout.connect(self._tick)
        self._anim_timer.start(30)

    def _tick(self):
        speed = (abs(self.left_power) + abs(self.right_power)) / 2.0
        self.road_offset = (self.road_offset + speed * 0.15) % 16
        self.update()

    def set_vehicle_state(self, roll_deg, pitch_deg, accel, gyro, connected, moving, left_power, right_power):
        self.roll_deg = roll_deg
        self.pitch_deg = pitch_deg
        self.accel = accel
        self.gyro = gyro
        self.connected = connected
        self.moving = moving
        self.left_power = left_power
        self.right_power = right_power
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        bg = QLinearGradient(0, 0, 0, self.height())
        bg.setColorAt(0.0, QColor(241, 245, 249))
        bg.setColorAt(0.5, QColor(234, 239, 245))
        bg.setColorAt(1.0, QColor(226, 232, 240))
        painter.fillRect(self.rect(), bg)

        w = self.width()
        h = self.height()
        cx = w / 2.0

        # Road surface glow
        road_glow = QLinearGradient(cx - 90, 0, cx + 90, 0)
        road_glow.setColorAt(0.0, QColor(255, 255, 255, 0))
        road_glow.setColorAt(0.3, QColor(203, 213, 225, 50))
        road_glow.setColorAt(0.5, QColor(186, 199, 214, 70))
        road_glow.setColorAt(0.7, QColor(203, 213, 225, 50))
        road_glow.setColorAt(1.0, QColor(255, 255, 255, 0))
        painter.fillRect(QRectF(cx - 90, 30, 180, h - 130), road_glow)

        # Lane lines
        painter.setPen(QPen(QColor(186, 199, 214), 2))
        lane_left = int(cx - 72)
        lane_right = int(cx + 72)
        painter.drawLine(lane_left, 40, lane_left, h - 110)
        painter.drawLine(lane_right, 40, lane_right, h - 110)

        center_pen = QPen(QColor(160, 175, 195), 2)
        center_pen.setStyle(Qt.CustomDashLine)
        center_pen.setDashPattern([8, 8])
        center_pen.setDashOffset(self.road_offset)
        painter.setPen(center_pen)
        painter.drawLine(int(cx), 40, int(cx), h - 110)

        car_w = 100.0
        car_h = 195.0
        cy = h / 2.0 - 20
        body_x = cx - car_w / 2.0 + max(min(self.roll_deg, 22.0), -22.0) * 0.7
        body_y = cy - car_h / 2.0 + max(min(self.pitch_deg, 22.0), -22.0) * 0.8
        body = QRectF(body_x, body_y, car_w, car_h)

        # Car body shadow
        shadow = QRectF(body.left() + 4, body.top() + 6, car_w, car_h)
        painter.setBrush(QColor(100, 116, 139, 40))
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(shadow, 24, 24)

        # Car body
        shell_grad = QLinearGradient(body.left(), body.top(), body.right(), body.bottom())
        shell_grad.setColorAt(0.0, QColor(59, 130, 246))
        shell_grad.setColorAt(0.3, QColor(37, 99, 235))
        shell_grad.setColorAt(0.7, QColor(29, 78, 216))
        shell_grad.setColorAt(1.0, QColor(30, 64, 175))
        painter.setBrush(QBrush(shell_grad))
        painter.setPen(QPen(QColor(96, 165, 250, 180), 1.5))
        painter.drawRoundedRect(body, 24, 24)

        # Cabin
        cabin = QRectF(body.left() + 12, body.top() + 20, car_w - 24, car_h - 64)
        cabin_grad = QLinearGradient(cabin.left(), cabin.top(), cabin.left(), cabin.bottom())
        cabin_grad.setColorAt(0.0, QColor(191, 219, 254, 200))
        cabin_grad.setColorAt(1.0, QColor(147, 197, 253, 180))
        painter.setBrush(QBrush(cabin_grad))
        painter.setPen(QPen(QColor(255, 255, 255, 120), 1.5))
        painter.drawRoundedRect(cabin, 18, 18)

        # Wheels
        wheel_color = QColor(51, 65, 85)
        wheel_glow = QColor(234, 88, 12) if self.moving else QColor(37, 99, 235)
        painter.setBrush(wheel_color)
        painter.setPen(QPen(wheel_glow, 2))
        wheel_rects = [
            QRectF(body.left() - 13, body.top() + 22, 13, 44),
            QRectF(body.right(), body.top() + 22, 13, 44),
            QRectF(body.left() - 13, body.bottom() - 66, 13, 44),
            QRectF(body.right(), body.bottom() - 66, 13, 44),
        ]
        for wheel in wheel_rects:
            painter.drawRoundedRect(wheel, 6, 6)

        # Energy ring
        accel_mag = min((self.accel[0] ** 2 + self.accel[1] ** 2 + self.accel[2] ** 2) ** 0.5, 2.0)
        ring_color = QColor(234, 88, 12, 60) if self.moving else QColor(37, 99, 235, 45)
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(ring_color, 5))
        painter.drawEllipse(QRectF(
            cx - 82 - accel_mag * 12, body.top() - 16 - accel_mag * 7,
            164 + accel_mag * 24, 228 + accel_mag * 14
        ))

        # Outer subtle ring
        painter.setPen(QPen(QColor(ring_color.red(), ring_color.green(), ring_color.blue(), 18), 10))
        painter.drawEllipse(QRectF(
            cx - 90 - accel_mag * 14, body.top() - 24 - accel_mag * 8,
            180 + accel_mag * 28, 244 + accel_mag * 16
        ))

        title_font_size = 18 if w >= 720 else 15
        status_font_size = 10 if w >= 720 else 9
        metric_value_font_size = 22 if w >= 720 else 17
        metric_unit_font_size = 9 if w >= 720 else 8

        # Title
        painter.setPen(QColor(30, 41, 59))
        painter.setFont(QFont("Segoe UI", title_font_size, QFont.Bold))
        painter.drawText(36, 42, "Vehicle Visualization")

        # Status badges
        painter.setFont(QFont("Segoe UI", status_font_size, QFont.DemiBold))
        status = "CONNECTED" if self.connected else "DISCONNECTED"
        motion = "MOVING" if self.moving else "STABLE"

        # Connected badge background
        badge_y = 52
        badge_h = 22
        badge_radius = 11

        status_color = QColor(22, 163, 74) if self.connected else QColor(220, 38, 38)
        status_bg = QColor(status_color.red(), status_color.green(), status_color.blue(), 30)
        status_w = max(110, len(status) * 10 + 24)
        painter.setPen(Qt.NoPen)
        painter.setBrush(status_bg)
        painter.drawRoundedRect(QRectF(36, badge_y, status_w, badge_h), badge_radius, badge_radius)
        painter.setPen(status_color)
        painter.drawText(QRectF(36, badge_y, status_w, badge_h), Qt.AlignCenter, status)

        motion_color = QColor(234, 88, 12) if self.moving else QColor(37, 99, 235)
        motion_bg = QColor(motion_color.red(), motion_color.green(), motion_color.blue(), 25)
        motion_w = max(90, len(motion) * 10 + 24)
        painter.setPen(Qt.NoPen)
        painter.setBrush(motion_bg)
        painter.drawRoundedRect(QRectF(36 + status_w + 10, badge_y, motion_w, badge_h), badge_radius, badge_radius)
        painter.setPen(motion_color)
        painter.drawText(QRectF(36 + status_w + 10, badge_y, motion_w, badge_h), Qt.AlignCenter, motion)

        # Direction badge
        motion = compute_motion_state(self.left_power, self.right_power)
        dir_label = motion['direction'].replace('_', ' ').upper()
        dir_color_map = {
            'FORWARD':    QColor(22, 163, 74),
            'REVERSE':    QColor(220, 38, 38),
            'TURN RIGHT': QColor(234, 88, 12),
            'TURN LEFT':  QColor(234, 88, 12),
            'SPIN RIGHT': QColor(168, 85, 247),
            'SPIN LEFT':  QColor(168, 85, 247),
            'STOPPED':    QColor(100, 116, 139),
        }
        dir_color = dir_color_map.get(dir_label, QColor(100, 116, 139))
        dir_bg = QColor(dir_color.red(), dir_color.green(), dir_color.blue(), 28)
        dir_w = max(110, len(dir_label) * 10 + 24)
        dir_x = 36 + status_w + 10 + motion_w + 10
        painter.setPen(Qt.NoPen)
        painter.setBrush(dir_bg)
        painter.drawRoundedRect(QRectF(dir_x, badge_y, dir_w, badge_h), badge_radius, badge_radius)
        painter.setPen(dir_color)
        painter.drawText(QRectF(dir_x, badge_y, dir_w, badge_h), Qt.AlignCenter, dir_label)

        # Metric cards
        card_y = h - 142
        card_h = 92
        card_w = (w - 92) / 4.0
        metrics = [
            ("Roll",  self.roll_deg,                 "deg",  QColor(124, 58, 237)),
            ("Pitch", self.pitch_deg,                "deg",  QColor(37, 99, 235)),
            ("Speed", motion['linear_vel_cms'],      "cm/s", QColor(5, 150, 105)),
            ("Angle", motion['turn_angle_deg'],      "deg",  QColor(220, 38, 38)),
        ]

        for index, (label, value, unit, accent) in enumerate(metrics):
            left = 20 + index * (card_w + 10)
            rect = QRectF(left, card_y, card_w, card_h)

            # Card background — white with subtle shadow effect
            card_grad = QLinearGradient(rect.left(), rect.top(), rect.left(), rect.bottom())
            card_grad.setColorAt(0.0, QColor(255, 255, 255, 240))
            card_grad.setColorAt(1.0, QColor(248, 250, 252, 240))
            painter.setBrush(QBrush(card_grad))
            painter.setPen(QPen(QColor(203, 213, 225), 1))
            painter.drawRoundedRect(rect, 14, 14)

            # Accent top line
            painter.setPen(QPen(accent, 2))
            painter.drawLine(
                int(rect.left() + 14), int(rect.top() + 1),
                int(rect.right() - 14), int(rect.top() + 1)
            )

            # Label
            painter.setPen(accent)
            painter.setFont(QFont("Segoe UI", metric_unit_font_size, QFont.DemiBold))
            painter.drawText(QRectF(rect.left() + 16, rect.top() + 12, rect.width() - 32, 18),
                             Qt.AlignLeft | Qt.AlignVCenter, label.upper())

            # Value
            painter.setPen(QColor(15, 23, 42))
            painter.setFont(QFont("Segoe UI", metric_value_font_size, QFont.Bold))
            value_rect = QRectF(rect.left() + 16, rect.top() + 34, rect.width() - 64, 36)
            painter.drawText(value_rect, Qt.AlignLeft | Qt.AlignVCenter, f"{value:.1f}")

            # Unit
            painter.setPen(QColor(100, 116, 139))
            painter.setFont(QFont("Segoe UI", metric_unit_font_size))
            unit_rect = QRectF(rect.right() - 48, rect.top() + 40, 32, 22)
            painter.drawText(unit_rect, Qt.AlignRight | Qt.AlignVCenter, unit)


class CsvLogger:
    def __init__(self):
        self.file = None
        self.writer = None
        self.active = False

    def start(self, filename):
        self.file = open(filename, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "timestamp",
            "ax", "ay", "az",
            "gx", "gy", "gz",
            "roll", "pitch",
            "imu_roll", "imu_pitch",
            "accel_roll", "accel_pitch",
            "packets_received"
        ])
        self.active = True

    def log(self, *row):
        if self.active and self.writer is not None:
            self.writer.writerow(row)

    def stop(self):
        if self.file:
            self.file.close()
        self.file = None
        self.writer = None
        self.active = False


def _styled_value_label():
    lbl = QLabel("0.00")
    lbl.setStyleSheet("color: #1e293b; font-weight: 600; font-size: 22px; font-family: 'Cascadia Code', 'Fira Code', 'Consolas', monospace;")
    return lbl


class ImuGui(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("STM32 Serial IMU Viewer")
        self.resize(2000, 1250)

        self.worker = None
        self.logger = CsvLogger()

        self.max_samples = 300
        self.sample_counter = 0
        self.packet_count = 0
        self.last_good_port = ""
        self.pending_packet = {}

        self.t = deque(maxlen=self.max_samples)

        self.ax_data = deque(maxlen=self.max_samples)
        self.ay_data = deque(maxlen=self.max_samples)
        self.az_data = deque(maxlen=self.max_samples)

        self.gx_data = deque(maxlen=self.max_samples)
        self.gy_data = deque(maxlen=self.max_samples)
        self.gz_data = deque(maxlen=self.max_samples)

        self.roll_data = deque(maxlen=self.max_samples)
        self.pitch_data = deque(maxlen=self.max_samples)

        self.accel_roll_data = deque(maxlen=self.max_samples)
        self.accel_pitch_data = deque(maxlen=self.max_samples)

        self.imu_roll_data = deque(maxlen=self.max_samples)
        self.imu_pitch_data = deque(maxlen=self.max_samples)

        self.left_power = 0
        self.right_power = 0

        self.filtered_roll = 0.0
        self.filtered_pitch = 0.0
        self.last_time = None

        self.accel_bias = {"ax": 0.0, "ay": 0.0, "az": 0.0}
        self.gyro_bias = {"gx": 0.0, "gy": 0.0, "gz": 0.0}

        self.latest_raw = None

        self.init_ui()
        self.data_log_timer = QTimer()
        self.data_log_timer.timeout.connect(self._append_data_log)
        self.data_log_timer.start(1000)
        self.pathfinder_tab.set_shared_serial_host(self)
        self.refresh_ports()
        self.update_alpha_label()
        self.update_plot_visibility()
        self.update_orientation_curve_visibility()



    def init_ui(self):
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(0)
        self.setLayout(main_layout)

        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        self.imu_tab = QWidget()
        self.pathfinder_tab = PathfinderTab()

        self.tabs.addTab(self.imu_tab, "  IMU  ")
        self.tabs.addTab(self.pathfinder_tab, "  Pathfinder  ")

        tab_outer = QHBoxLayout()
        tab_outer.setContentsMargins(8, 8, 8, 8)
        tab_outer.setSpacing(10)
        self.imu_tab.setLayout(tab_outer)

        left_widget = QWidget()
        tab_layout = QVBoxLayout()
        tab_layout.setContentsMargins(0, 0, 0, 0)
        tab_layout.setSpacing(10)
        left_widget.setLayout(tab_layout)
        tab_outer.addWidget(left_widget, stretch=3)

        self.robot_visual = RobotVisualizationWidget()
        tab_layout.addWidget(self.robot_visual)

        # ── Bluetooth Connection ──
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setInsertPolicy(QComboBox.NoInsert)
        self.port_combo.setEditText(DEFAULT_DEVICE_NAME)
        self.port_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        self.refresh_btn = QPushButton("Refresh Devices")
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setObjectName("connectBtn")
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setObjectName("disconnectBtn")
        self.disconnect_btn.setEnabled(False)
        self.reconnect_btn = QPushButton("Reconnect")
        self.reconnect_btn.setEnabled(False)

        self.log_btn = QPushButton("Start Log")
        self.stop_log_btn = QPushButton("Stop Log")
        self.stop_log_btn.setEnabled(False)

        self.status_label = QLabel("Status: Disconnected")
        self.status_label.setStyleSheet("font-weight: bold; color: #6b7280;")
        self.console = QTextEdit()
        self.console.setMaximumHeight(250)

        self.labels = {}
        rows = [
            "ax", "ay", "az", "gx", "gy", "gz",
            "roll", "pitch", "imu_roll", "imu_pitch",
            "accel_roll", "accel_pitch", "sample_rate",
            "packets", "last_packet",
            "left_power", "right_power",
        ]
        for key in rows:
            self.labels[key] = _styled_value_label()

        self.connection_indicator = QLabel("Disconnected")
        self.motion_indicator = QLabel("Unknown")
        self.tilt_indicator = QLabel("No Alarm")
        self.filter_indicator = QLabel("IMU Angles")

        self.ping_btn = QPushButton("Ping STM32")
        self.calib_accel_btn = QPushButton("Calibrate Accel")
        self.calib_gyro_btn = QPushButton("Calibrate Gyro")
        self.reset_orientation_btn = QPushButton("Reset Orientation")
        self.clear_plots_btn = QPushButton("Clear Plots")
        self.estop_btn = QPushButton("EMERGENCY STOP")
        self.estop_btn.setObjectName("estopBtn")
        self.send_start_btn = QPushButton("Send START")
        self.send_cal_btn = QPushButton("Send CALIBRATE")

        self.comp_filter_checkbox = QCheckBox("Enable Complementary Filter")
        self.comp_filter_checkbox.setChecked(False)
        self.alpha_slider = QSlider(Qt.Horizontal)
        self.alpha_slider.setMinimum(90)
        self.alpha_slider.setMaximum(99)
        self.alpha_slider.setValue(98)
        self.alpha_label = QLabel("Alpha: 0.98")

        self.tilt_alarm_checkbox = QCheckBox("Enable Tilt Alarm")
        self.tilt_alarm_checkbox.setChecked(True)
        self.roll_thresh = QDoubleSpinBox()
        self.roll_thresh.setRange(0.0, 180.0)
        self.roll_thresh.setValue(30.0)
        self.pitch_thresh = QDoubleSpinBox()
        self.pitch_thresh.setRange(0.0, 180.0)
        self.pitch_thresh.setValue(30.0)
        self.motion_thresh = QDoubleSpinBox()
        self.motion_thresh.setRange(0.0, 1000.0)
        self.motion_thresh.setValue(2.0)
        self.motion_thresh.setDecimals(3)
        # ── Bluetooth group ──
        bluetooth_group = QGroupBox("Bluetooth")
        bluetooth_layout = QGridLayout()
        bluetooth_layout.setSpacing(8)
        bluetooth_layout.setContentsMargins(12, 12, 12, 12)
        device_label = QLabel("Device")
        device_label.setStyleSheet("color: #8892a4; font-weight: 600;")
        bluetooth_layout.addWidget(device_label, 0, 0)
        bluetooth_layout.addWidget(self.port_combo, 0, 1, 1, 3)
        self.stream_btn = QPushButton("Stream ON")
        self.stream_btn.setEnabled(False)

        bluetooth_layout.addWidget(self.refresh_btn, 1, 0)
        bluetooth_layout.addWidget(self.connect_btn, 1, 1)
        bluetooth_layout.addWidget(self.disconnect_btn, 1, 2)
        bluetooth_layout.addWidget(self.reconnect_btn, 1, 3)
        bluetooth_layout.addWidget(self.stream_btn, 2, 0, 1, 4)
        bluetooth_layout.addWidget(self.status_label, 3, 0, 1, 4)
        bluetooth_group.setLayout(bluetooth_layout)
        tab_layout.addWidget(bluetooth_group)

        self.console.setReadOnly(True)
        tab_layout.addWidget(self.console)

        # ── Right-side data log panel ──
        log_group = QGroupBox("Data Log")
        log_vlayout = QVBoxLayout()
        log_vlayout.setContentsMargins(8, 8, 8, 8)
        self.data_log = QPlainTextEdit()
        self.data_log.setReadOnly(True)
        self.data_log.setMaximumBlockCount(300)
        self.data_log.setMinimumWidth(260)
        self.data_log.setPlaceholderText("Waiting for data…")
        log_vlayout.addWidget(self.data_log)
        log_group.setLayout(log_vlayout)
        tab_outer.addWidget(log_group, stretch=1)

        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.connect_serial)
        self.disconnect_btn.clicked.connect(self.disconnect_serial)
        self.reconnect_btn.clicked.connect(self.reconnect_serial)
        self.log_btn.clicked.connect(self.start_logging)
        self.stop_log_btn.clicked.connect(self.stop_logging)
        self.calib_accel_btn.clicked.connect(self.calibrate_accel)
        self.calib_gyro_btn.clicked.connect(self.calibrate_gyro)
        self.reset_orientation_btn.clicked.connect(self.reset_orientation)
        self.clear_plots_btn.clicked.connect(self.clear_plots)
        self.estop_btn.clicked.connect(self.emergency_stop)
        self.stream_btn.clicked.connect(self.toggle_stream)

    def append_console(self, msg):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.console.append(f"[{timestamp}] {msg}")

    def refresh_ports(self):
        current = self.get_selected_port()
        current_text = self.port_combo.currentText().strip()
        self.port_combo.clear()
        devices = scan_ble_devices(timeout=5.0)

        for label, address in devices:
            self.port_combo.addItem(label, address)

        if self.port_combo.findText(DEFAULT_DEVICE_NAME) < 0:
            self.port_combo.addItem(DEFAULT_DEVICE_NAME, "")

        if current:
            for i in range(self.port_combo.count()):
                if self.port_combo.itemData(i) == current:
                    self.port_combo.setCurrentIndex(i)
                    break
        elif current_text:
            self.port_combo.setEditText(current_text)
        else:
            self.port_combo.setEditText(DEFAULT_DEVICE_NAME)

        if devices:
            self.append_console("Bluetooth devices refreshed")
        else:
            self.append_console("No Bluetooth devices found during scan. You can still type the device name and press Connect.")

    def get_selected_port(self):
        data = self.port_combo.currentData()
        if data:
            return data
        return self.port_combo.currentText().strip()

    def connect_serial(self):
        if self.worker is not None and self.worker.isRunning():
            return

        port = self.get_selected_port()
        device_name = self.port_combo.currentText().strip() or DEFAULT_DEVICE_NAME
        if not port and not device_name:
            QMessageBox.warning(self, "No Device", "No Bluetooth device selected")
            return

        self.status_label.setText("Status: Connecting...")
        self.status_label.setStyleSheet("font-weight: bold; color: #f59e0b;")
        self.connect_btn.setEnabled(False)
        self.append_console("Connecting...")

        self.last_good_port = port

        self.worker = BleWorker(device_address=port or None, device_name=device_name)
        self.worker.line_received.connect(self.handle_ble_line)
        self.worker.status_message.connect(self.append_console)
        self.worker.connection_changed.connect(self.handle_connection_change)
        self.worker.start()

    def disconnect_serial(self):
        if self.worker is not None:
            try:
                self.worker.line_received.disconnect(self.handle_ble_line)
            except Exception:
                pass
            self.worker.stop()
            self.worker.wait()
            self.worker = None

    def reconnect_serial(self):
        if not self.last_good_port:
            self.append_console("No previous Bluetooth device to reconnect")
            return

        self.disconnect_serial()
        self.refresh_ports()

        for i in range(self.port_combo.count()):
            if self.port_combo.itemData(i) == self.last_good_port:
                self.port_combo.setCurrentIndex(i)
                break

        self.connect_serial()

    def auto_connect_serial(self):
        if self.worker is not None and self.worker.isRunning():
            return
        if self.port_combo.count() == 0:
            self.refresh_ports()
        if self.port_combo.count() == 0:
            self.append_console("Auto-connect skipped: no Bluetooth devices found")
            return
        if not self.get_selected_port():
            self.port_combo.setCurrentIndex(0)
        self.connect_serial()

    def handle_connection_change(self, connected):
        self.robot_visual.set_vehicle_state(
            self.filtered_roll,
            self.filtered_pitch,
            (
                self.latest_raw["ax"] if self.latest_raw else 0.0,
                self.latest_raw["ay"] if self.latest_raw else 0.0,
                self.latest_raw["az"] if self.latest_raw else 0.0,
            ),
            (
                self.latest_raw["gx"] if self.latest_raw else 0.0,
                self.latest_raw["gy"] if self.latest_raw else 0.0,
                self.latest_raw["gz"] if self.latest_raw else 0.0,
            ),
            connected,
            self.motion_indicator.text() == "Moving",
            self.left_power,
            self.right_power,
        )
        self.pathfinder_tab.handle_serial_status("Connected" if connected else "Disconnected")
        self.pathfinder_tab.sync_shared_serial_controls()
        if connected:
            self.status_label.setText("Status: Connected")
            self.status_label.setStyleSheet("font-weight: bold; color: #22c55e;")
            self.connection_indicator.setText("Connected")
            self.connection_indicator.setStyleSheet("font-weight: bold; color: #22c55e;")
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.reconnect_btn.setEnabled(True)
            self.stream_btn.setEnabled(True)
        else:
            self.status_label.setText("Status: Disconnected")
            self.status_label.setStyleSheet("font-weight: bold; color: #6b7280;")
            self.connection_indicator.setText("Disconnected")
            self.connection_indicator.setStyleSheet("font-weight: bold; color: #ef4444;")
            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)
            self.reconnect_btn.setEnabled(True)
            self.stream_btn.setEnabled(False)
            self.stream_btn.setText("Stream ON")

    def toggle_stream(self):
        if self.worker is None:
            return
        if self.stream_btn.text() == "Stream ON":
            self.worker.send_text("STREAM_ON\n")
            self.stream_btn.setText("Stream OFF")
            self.append_console("Sent STREAM_ON")
        else:
            self.worker.send_text("STREAM_OFF\n")
            self.stream_btn.setText("Stream ON")
            self.append_console("Sent STREAM_OFF")

    def handle_ble_line(self, line):
        self.pathfinder_tab.handle_serial_line(line)

        # Parse motor power: "pwr,<0|1>,<signed_int>"
        pwr_match = re.match(r"pwr,([01]),([-\d]+)", line)
        if pwr_match:
            which = int(pwr_match.group(1))
            value = int(pwr_match.group(2))
            if which == 0:
                self.left_power = value
                self.labels["left_power"].setText(str(value))
            else:
                self.right_power = value
                self.labels["right_power"].setText(str(value))
            return

        parsed = self.parse_line(line)
        if parsed is not None:
            self.handle_data(parsed)

    # LSM6DS33 default full-scale sensitivities
    ACCEL_SENSITIVITY = 0.000061    # ±2g  → 0.061 mg/LSB → g/LSB
    GYRO_SENSITIVITY  = 0.00875     # ±245dps → 8.75 mdps/LSB → dps/LSB

    # Mapping from imu register index to (packet_key, sensitivity)
    IMU_REG_MAP = {
        0: ("ax", ACCEL_SENSITIVITY),
        1: ("ay", ACCEL_SENSITIVITY),
        2: ("az", ACCEL_SENSITIVITY),
        3: ("gx", GYRO_SENSITIVITY),
        4: ("gy", GYRO_SENSITIVITY),
        5: ("gz", GYRO_SENSITIVITY),
    }

    @staticmethod
    def _uint16_to_int16(val):
        return val - 65536 if val >= 32768 else val

    def parse_line(self, line):
        # BLE bridge format: "imu,<reg>,<raw_uint16>"
        imu_match = re.match(r"imu,(\d+),(\d+)", line)
        if imu_match:
            reg = int(imu_match.group(1))
            raw_u16 = int(imu_match.group(2))
            if reg in self.IMU_REG_MAP:
                key, sensitivity = self.IMU_REG_MAP[reg]
                self.pending_packet[key] = self._uint16_to_int16(raw_u16) * sensitivity

                # Emit as soon as ax/ay/az are known — use last known gx/gy/gz if not yet updated
                if all(k in self.pending_packet for k in ["ax", "ay", "az"]):
                    ax = self.pending_packet["ax"]
                    ay = self.pending_packet["ay"]
                    az = self.pending_packet["az"]
                    imu_roll, imu_pitch = compute_roll_pitch(ax, ay, az)
                    return {
                        "ax": ax,
                        "ay": ay,
                        "az": az,
                        "gx": self.pending_packet.get("gx", 0.0),
                        "gy": self.pending_packet.get("gy", 0.0),
                        "gz": self.pending_packet.get("gz", 0.0),
                        "imu_roll": imu_roll,
                        "imu_pitch": imu_pitch,
                        "raw": line,
                    }
            return None

        # STM32 direct serial format: "ACC[g] X:... | GYRO[dps] X:..."
        acc_gyro_match = re.match(
            r"ACC\[g\]\s*X:([-\d.]+)\s*Y:([-\d.]+)\s*Z:([-\d.]+)\s*\|\s*GYRO\[dps\]\s*X:([-\d.]+)\s*Y:([-\d.]+)\s*Z:([-\d.]+)",
            line
        )

        if acc_gyro_match:
            self.pending_packet["ax"] = float(acc_gyro_match.group(1))
            self.pending_packet["ay"] = float(acc_gyro_match.group(2))
            self.pending_packet["az"] = float(acc_gyro_match.group(3))
            self.pending_packet["gx"] = float(acc_gyro_match.group(4))
            self.pending_packet["gy"] = float(acc_gyro_match.group(5))
            self.pending_packet["gz"] = float(acc_gyro_match.group(6))
            self.pending_packet["raw_acc_gyro"] = line
            return None

        angle_match = re.match(
            r"ANGLE\s*roll:([-\d.]+)\s*deg\s*\|\s*pitch:([-\d.]+)\s*deg",
            line
        )

        if angle_match:
            self.pending_packet["imu_roll"] = float(angle_match.group(1))
            self.pending_packet["imu_pitch"] = float(angle_match.group(2))
            self.pending_packet["raw_angle"] = line

            required = ["ax", "ay", "az", "gx", "gy", "gz", "imu_roll", "imu_pitch"]
            if all(k in self.pending_packet for k in required):
                packet = {
                    "ax": self.pending_packet["ax"],
                    "ay": self.pending_packet["ay"],
                    "az": self.pending_packet["az"],
                    "gx": self.pending_packet["gx"],
                    "gy": self.pending_packet["gy"],
                    "gz": self.pending_packet["gz"],
                    "imu_roll": self.pending_packet["imu_roll"],
                    "imu_pitch": self.pending_packet["imu_pitch"],
                    "raw": self.pending_packet.get("raw_acc_gyro", "") + " || " + self.pending_packet.get("raw_angle", "")
                }
                self.pending_packet = {}
                return packet

        return None

    def start_logging(self):
        if self.logger.active:
            self.append_console("Already logging")
            return

        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save IMU Log",
            f"imu_log_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv",
            "CSV Files (*.csv)"
        )

        if not filename:
            return

        try:
            self.logger.start(filename)
            self.append_console(f"Logging started: {filename}")
            self.log_btn.setEnabled(False)
            self.stop_log_btn.setEnabled(True)
        except Exception as e:
            QMessageBox.critical(self, "Logging Error", str(e))

    def stop_logging(self):
        if self.logger.active:
            self.logger.stop()
            self.append_console("Logging stopped")
            self.log_btn.setEnabled(True)
            self.stop_log_btn.setEnabled(False)

    def update_alpha_label(self):
        alpha = self.alpha_slider.value() / 100.0
        self.alpha_label.setText(f"Alpha: {alpha:.2f}")

    def on_filter_mode_changed(self):
        if self.comp_filter_checkbox.isChecked():
            self.filter_indicator.setText("Complementary Filter")
        else:
            self.filter_indicator.setText("IMU Angles")

    def update_plot_visibility(self):
        self.robot_visual.setVisible(True)

    def update_orientation_curve_visibility(self):
        self.robot_visual.update()

    def handle_data(self, d):
        now = time.time()

        if self.last_time is None:
            dt = 0.01
            sample_rate = 0.0
        else:
            dt = now - self.last_time
            if dt <= 0:
                dt = 0.01
            sample_rate = 1.0 / dt

        self.last_time = now

        self.packet_count += 1
        self.latest_raw = d

        ax_raw = d["ax"]
        ay_raw = d["ay"]
        az_raw = d["az"]
        gx_raw = d["gx"]
        gy_raw = d["gy"]
        gz_raw = d["gz"]

        imu_roll_raw = d.get("imu_roll", 0.0)
        imu_roll_raw += 180
        if imu_roll_raw > 180:
            imu_roll_raw -= 360
        imu_pitch_raw = d.get("imu_pitch", 0.0)

        ax = ax_raw - self.accel_bias["ax"]
        ay = -(ay_raw - self.accel_bias["ay"])   # invert Y: IMU is upside down
        az = az_raw - self.accel_bias["az"]

        gx = gx_raw - self.gyro_bias["gx"]
        gy = gy_raw - self.gyro_bias["gy"]
        gz = gz_raw - self.gyro_bias["gz"]

        accel_roll, accel_pitch = compute_roll_pitch(ax, ay, az)
        # Normalize roll: IMU upside down reads ±180° when flat → offset to 0°
        accel_roll += 180
        if accel_roll > 180:
            accel_roll -= 360

        if self.comp_filter_checkbox.isChecked():
            alpha = self.alpha_slider.value() / 100.0
            gyro_roll = self.filtered_roll + gx * dt
            gyro_pitch = self.filtered_pitch + gy * dt
            self.filtered_roll = alpha * gyro_roll + (1.0 - alpha) * accel_roll
            self.filtered_pitch = alpha * gyro_pitch + (1.0 - alpha) * accel_pitch
            roll = self.filtered_roll
            pitch = self.filtered_pitch
            self.filter_indicator.setText("Complementary Filter")
        else:
            roll = imu_roll_raw
            pitch = imu_pitch_raw
            self.filtered_roll = roll
            self.filtered_pitch = pitch
            self.filter_indicator.setText("IMU Angles")

        self.labels["ax"].setText(f"{ax:.3f}")
        self.labels["ay"].setText(f"{ay:.3f}")
        self.labels["az"].setText(f"{az:.3f}")
        self.labels["gx"].setText(f"{gx:.3f}")
        self.labels["gy"].setText(f"{gy:.3f}")
        self.labels["gz"].setText(f"{gz:.3f}")
        self.labels["roll"].setText(f"{roll:.2f}")
        self.labels["pitch"].setText(f"{pitch:.2f}")
        self.labels["imu_roll"].setText(f"{imu_roll_raw:.2f}")
        self.labels["imu_pitch"].setText(f"{imu_pitch_raw:.2f}")
        self.labels["accel_roll"].setText(f"{accel_roll:.2f}")
        self.labels["accel_pitch"].setText(f"{accel_pitch:.2f}")
        self.labels["sample_rate"].setText(f"{sample_rate:.1f} Hz")
        self.labels["packets"].setText(str(self.packet_count))
        self.labels["last_packet"].setText("0.00 s ago")

        self.sample_counter += 1
        self.t.append(self.sample_counter)

        self.ax_data.append(ax)
        self.ay_data.append(ay)
        self.az_data.append(az)

        self.gx_data.append(gx)
        self.gy_data.append(gy)
        self.gz_data.append(gz)

        self.roll_data.append(roll)
        self.pitch_data.append(pitch)

        self.imu_roll_data.append(imu_roll_raw)
        self.imu_pitch_data.append(imu_pitch_raw)

        self.accel_roll_data.append(accel_roll)
        self.accel_pitch_data.append(accel_pitch)

        self.update_motion_indicator(ax, ay, az, gx, gy, gz)
        self.check_tilt_alarm(roll, pitch)
        self.robot_visual.set_vehicle_state(
            roll,
            pitch,
            (ax, ay, az),
            (gx, gy, gz),
            self.connection_indicator.text() == "Connected",
            self.motion_indicator.text() == "Moving",
            self.left_power,
            self.right_power,
        )
        self.update_plots()

        if self.logger.active:
            ts = datetime.now().isoformat(timespec="milliseconds")
            self.logger.log(
                ts,
                ax, ay, az,
                gx, gy, gz,
                roll, pitch,
                imu_roll_raw, imu_pitch_raw,
                accel_roll, accel_pitch,
                self.packet_count
            )

    def update_motion_indicator(self, ax, ay, az, gx, gy, gz):
        acc_mag = math.sqrt(ax * ax + ay * ay + az * az)
        gyro_mag = math.sqrt(gx * gx + gy * gy + gz * gz)
        thresh = self.motion_thresh.value()

        if abs(acc_mag - 1.0) > 0.15 or gyro_mag > thresh:
            self.motion_indicator.setText("Moving")
            self.motion_indicator.setStyleSheet("font-weight: bold; color: #fb923c;")
        else:
            self.motion_indicator.setText("Stationary")
            self.motion_indicator.setStyleSheet("font-weight: bold; color: #22c55e;")

    def check_tilt_alarm(self, roll, pitch):
        if not self.tilt_alarm_checkbox.isChecked():
            self.tilt_indicator.setText("Alarm Disabled")
            self.tilt_indicator.setStyleSheet("font-weight: bold; color: #6b7280;")
            return

        roll_limit = self.roll_thresh.value()
        pitch_limit = self.pitch_thresh.value()

        if abs(roll) > roll_limit or abs(pitch) > pitch_limit:
            self.tilt_indicator.setText("TILT ALARM")
            self.tilt_indicator.setStyleSheet("font-weight: bold; color: #ef4444;")
            if self.connection_indicator.text() == "Connected":
                self.status_label.setText("Status: TILT ALARM")
                self.status_label.setStyleSheet("font-weight: bold; color: #ef4444;")
        else:
            self.tilt_indicator.setText("No Alarm")
            self.tilt_indicator.setStyleSheet("font-weight: bold; color: #22c55e;")
            if self.connection_indicator.text() == "Connected":
                self.status_label.setText("Status: Connected")
                self.status_label.setStyleSheet("font-weight: bold; color: #22c55e;")

    def update_plots(self):
        return

    def calibrate_accel(self):
        if self.latest_raw is None:
            self.append_console("No IMU data yet for accel calibration")
            return

        self.accel_bias["ax"] = self.latest_raw["ax"]
        self.accel_bias["ay"] = self.latest_raw["ay"]
        self.accel_bias["az"] = self.latest_raw["az"] - 1.0

        self.append_console(
            f"Accel calibrated: ax={self.accel_bias['ax']:.3f}, "
            f"ay={self.accel_bias['ay']:.3f}, az={self.accel_bias['az']:.3f}"
        )

    def calibrate_gyro(self):
        if self.latest_raw is None:
            self.append_console("No IMU data yet for gyro calibration")
            return

        self.gyro_bias["gx"] = self.latest_raw["gx"]
        self.gyro_bias["gy"] = self.latest_raw["gy"]
        self.gyro_bias["gz"] = self.latest_raw["gz"]

        self.append_console(
            f"Gyro calibrated: gx={self.gyro_bias['gx']:.3f}, "
            f"gy={self.gyro_bias['gy']:.3f}, gz={self.gyro_bias['gz']:.3f}"
        )

    def reset_orientation(self):
        self.filtered_roll = 0.0
        self.filtered_pitch = 0.0
        self.last_time = None
        self.append_console("Orientation reset")

    def clear_plots(self):
        self.sample_counter = 0
        self.packet_count = 0
        self.t.clear()

        self.ax_data.clear()
        self.ay_data.clear()
        self.az_data.clear()

        self.gx_data.clear()
        self.gy_data.clear()
        self.gz_data.clear()

        self.roll_data.clear()
        self.pitch_data.clear()

        self.imu_roll_data.clear()
        self.imu_pitch_data.clear()

        self.accel_roll_data.clear()
        self.accel_pitch_data.clear()

        self.labels["packets"].setText("0")
        self.labels["last_packet"].setText("N/A")

        self.update_plots()
        self.append_console("Plots cleared")

    def reset_link_status(self):
        self.packet_count = 0
        self.labels["packets"].setText("0")
        self.labels["last_packet"].setText("N/A")
        self.append_console("Serial link status reset")

    def send_serial_command(self, cmd):
        if self.worker is None:
            self.append_console("Cannot send command: not connected")
            return

        ok = self.worker.send_text(cmd)
        if ok:
            self.append_console(f"Sent command: {cmd.strip()}")
        else:
            self.append_console("Failed to send command")

    def emergency_stop(self):
        self.send_serial_command("STOP\n")

    def _append_data_log(self):
        if self.latest_raw is None or self.stream_btn.text() == "Stream ON":
            return
        ts = datetime.now().strftime("%H:%M:%S")
        ax = self.latest_raw.get("ax", 0.0)
        ay = self.latest_raw.get("ay", 0.0)
        az = self.latest_raw.get("az", 0.0)
        gx = self.latest_raw.get("gx", 0.0)
        gy = self.latest_raw.get("gy", 0.0)
        gz = self.latest_raw.get("gz", 0.0)
        entry = (
            f"[{ts}]\n"
            f"  Roll:{self.filtered_roll:+7.2f}°  Pitch:{self.filtered_pitch:+7.2f}°\n"
            f"  Ax:{ax:+.3f}  Ay:{ay:+.3f}  Az:{az:+.3f} g\n"
            f"  Gx:{gx:+.2f}  Gy:{gy:+.2f}  Gz:{gz:+.2f} dps\n"
            f"  L-Pwr:{self.left_power:+d}  R-Pwr:{self.right_power:+d}"
        )
        self.data_log.appendPlainText(entry)

    def closeEvent(self, event):
        try:
            self.stop_logging()
            self.disconnect_serial()
        except Exception:
            pass
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(LIGHT_STYLESHEET)
    window = ImuGui()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
