import sys
import math
import time
import json
from pathlib import Path

from PyQt5.QtCore import Qt, pyqtSignal, QPointF
from PyQt5.QtGui import QPainter, QPen, QColor, QBrush, QFont, QPolygonF
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QComboBox,
    QVBoxLayout, QHBoxLayout, QGridLayout, QMessageBox,
    QDoubleSpinBox, QCheckBox, QSizePolicy, QFrame
)

ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.append(str(ROOT_DIR))

from ble_receiver import BleWorker, DEFAULT_DEVICE_NAME, scan_ble_devices


class GridCanvas(QWidget):
    path_changed = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(600, 600)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)

        self.grid_spacing_px = 40
        self.cm_per_grid = 10.0
        self.px_per_cm = self.grid_spacing_px / self.cm_per_grid
        self.plot_margin_left = 48
        self.plot_margin_right = 20
        self.plot_margin_top = 20
        self.plot_margin_bottom = 40

        self.origin_set = False
        self.origin_world_x = 0.0
        self.origin_world_y = 0.0

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading_deg = 0.0

        self.path_points = []
        self.robot_trail = []

        self.drawing_enabled = True
        self.show_trail = True
        self.show_labels = True
        self.show_heading_vector = True

        self.mouse_world = None

        self.trail_min_step_cm = 0.5

    def set_origin(self, x_cm=0.0, y_cm=0.0):
        self.origin_world_x = x_cm
        self.origin_world_y = y_cm
        self.origin_set = True
        self.update()

    def clear_origin(self):
        self.origin_set = False
        self.origin_world_x = 0.0
        self.origin_world_y = 0.0
        self.update()

    def clear_path(self):
        self.path_points = []
        self.update()
        self.path_changed.emit()

    def clear_trail(self):
        self.robot_trail = []
        self.update()

    def set_robot_pose(self, x_cm, y_cm, heading_deg):
        old_x = self.robot_x
        old_y = self.robot_y

        self.robot_x = x_cm
        self.robot_y = y_cm
        self.robot_heading_deg = heading_deg

        moved = math.hypot(x_cm - old_x, y_cm - old_y)

        if not self.robot_trail:
            self.robot_trail.append((x_cm, y_cm))
        elif moved >= self.trail_min_step_cm:
            self.robot_trail.append((x_cm, y_cm))
            if len(self.robot_trail) > 2000:
                self.robot_trail = self.robot_trail[-2000:]

        self.update()

    def world_to_screen(self, x_cm, y_cm):
        sx = self.plot_margin_left + (x_cm - self.origin_world_x) * self.px_per_cm
        sy = self.height() - self.plot_margin_bottom - (
            (y_cm - self.origin_world_y) * self.px_per_cm
        )
        return sx, sy

    def screen_to_world(self, sx, sy):
        usable_x = max(0.0, sx - self.plot_margin_left)
        usable_y = max(0.0, (self.height() - self.plot_margin_bottom) - sy)
        x_cm = self.origin_world_x + usable_x / self.px_per_cm
        y_cm = self.origin_world_y + usable_y / self.px_per_cm
        return x_cm, y_cm

    @staticmethod
    def snap_waypoint(x_cm, y_cm):
        return int(round(x_cm)), int(round(y_cm))

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self.drawing_enabled:
            x_cm, y_cm = self.screen_to_world(event.x(), event.y())
            x_cm, y_cm = self.snap_waypoint(x_cm, y_cm)
            self.path_points.append((x_cm, y_cm))
            self.path_changed.emit()
            self.update()

        elif event.button() == Qt.RightButton:
            x_cm, y_cm = self.screen_to_world(event.x(), event.y())
            self.set_origin(x_cm, y_cm)

    def mouseMoveEvent(self, event):
        self.mouse_world = self.screen_to_world(event.x(), event.y())
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        painter.fillRect(self.rect(), QColor(10, 12, 20))

        self.draw_grid(painter)
        self.draw_axes(painter)
        self.draw_origin(painter)
        self.draw_path(painter)
        self.draw_robot_trail(painter)
        self.draw_robot(painter)
        self.draw_mouse_info(painter)

    def draw_grid(self, painter):
        pen_minor = QPen(QColor(35, 40, 60), 1)
        pen_major = QPen(QColor(55, 60, 90), 1)

        left = self.plot_margin_left
        right = self.width() - self.plot_margin_right
        top = self.plot_margin_top
        bottom = self.height() - self.plot_margin_bottom
        spacing = self.grid_spacing_px

        num_v = int(max(0, right - left) / spacing) + 1
        num_h = int(max(0, bottom - top) / spacing) + 1

        for i in range(num_v + 1):
            x = left + i * spacing
            painter.setPen(pen_major if i % 5 == 0 else pen_minor)
            painter.drawLine(int(x), int(top), int(x), int(bottom))

        for j in range(num_h + 1):
            y = bottom - j * spacing
            painter.setPen(pen_major if j % 5 == 0 else pen_minor)
            painter.drawLine(int(left), int(y), int(right), int(y))

    def draw_axes(self, painter):
        left = self.plot_margin_left
        right = self.width() - self.plot_margin_right
        top = self.plot_margin_top
        bottom = self.height() - self.plot_margin_bottom

        painter.setPen(QPen(QColor(120, 120, 150), 1))
        painter.drawLine(int(left), int(bottom), int(right), int(bottom))
        painter.drawLine(int(left), int(top), int(left), int(bottom))

        if self.show_labels:
            painter.setPen(QColor(220, 220, 220))
            painter.setFont(QFont("Arial", 11))
            painter.drawText(int(left + 8), int(top + 16), "+Y")
            painter.drawText(int(right - 28), int(bottom - 8), "+X")

    def draw_origin(self, painter):
        if not self.origin_set:
            return

        sx, sy = self.world_to_screen(self.origin_world_x, self.origin_world_y)

        painter.setPen(QPen(QColor(255, 210, 80), 2))
        painter.setBrush(QBrush(QColor(255, 210, 80)))
        painter.drawEllipse(QPointF(sx, sy), 6, 6)

        painter.setPen(QColor(255, 230, 150))
        painter.setFont(QFont("Arial", 11, QFont.Bold))
        painter.drawText(int(sx + 10), int(sy - 10), "ORIGIN")
        painter.drawText(int(sx + 10), int(sy + 12), f"({self.origin_world_x:.1f}, {self.origin_world_y:.1f}) cm")

    def draw_path(self, painter):
        if not self.path_points:
            return

        point_pen = QPen(QColor(80, 200, 255), 2)
        line_pen = QPen(QColor(80, 200, 255), 2)

        prev = None
        for i, (x, y) in enumerate(self.path_points):
            sx, sy = self.world_to_screen(x, y)

            painter.setPen(point_pen)
            painter.setBrush(QBrush(QColor(80, 200, 255)))
            painter.drawEllipse(QPointF(sx, sy), 5, 5)

            if prev is not None:
                painter.setPen(line_pen)
                painter.drawLine(int(prev[0]), int(prev[1]), int(sx), int(sy))

            if self.show_labels:
                painter.setPen(QColor(210, 240, 255))
                painter.setFont(QFont("Arial", 11, QFont.Bold))
                painter.drawText(int(sx + 8), int(sy - 8), f"P{i + 1} ({x}, {y})")

            prev = (sx, sy)

    def draw_robot_trail(self, painter):
        if not self.show_trail or len(self.robot_trail) < 2:
            return

        painter.setPen(QPen(QColor(100, 255, 140), 2))

        prev = None
        for x, y in self.robot_trail:
            sx, sy = self.world_to_screen(x, y)
            if prev is not None:
                if math.hypot(sx - prev[0], sy - prev[1]) > 1.0:
                    painter.drawLine(int(prev[0]), int(prev[1]), int(sx), int(sy))
            prev = (sx, sy)

    def draw_robot(self, painter):
        sx, sy = self.world_to_screen(self.robot_x, self.robot_y)

        size = 16.0
        theta = math.radians(self.robot_heading_deg)

        pts = [
            QPointF(size, 0),
            QPointF(-size * 0.65, size * 0.55),
            QPointF(-size * 0.65, -size * 0.55),
        ]

        rot_pts = []
        for p in pts:
            rx = p.x() * math.cos(theta) - p.y() * math.sin(theta)
            ry = p.x() * math.sin(theta) + p.y() * math.cos(theta)
            rot_pts.append(QPointF(sx + rx, sy - ry))

        painter.setPen(QPen(QColor(255, 120, 110), 2))
        painter.setBrush(QBrush(QColor(255, 120, 110, 200)))
        painter.drawPolygon(QPolygonF(rot_pts))

        if self.show_heading_vector:
            hx = sx + 35.0 * math.cos(theta)
            hy = sy - 35.0 * math.sin(theta)
            painter.setPen(QPen(QColor(255, 180, 150), 2))
            painter.drawLine(int(sx), int(sy), int(hx), int(hy))

        painter.setPen(QColor(255, 200, 190))
        painter.setFont(QFont("Arial", 10))
        painter.drawText(int(sx + 10), int(sy + 18), f"({self.robot_x:.1f}, {self.robot_y:.1f}) cm")
        painter.drawText(int(sx + 10), int(sy + 34), f"{self.robot_heading_deg:.1f} deg")

    def draw_mouse_info(self, painter):
        if self.mouse_world is None:
            return
        x, y = self.mouse_world
        snap_x, snap_y = self.snap_waypoint(x, y)
        txt = f"Mouse: X={snap_x} cm   Y={snap_y} cm"
        painter.setPen(QColor(220, 220, 220))
        painter.setFont(QFont("Consolas", 10))
        fm = painter.fontMetrics()
        tw = fm.horizontalAdvance(txt)
        painter.drawText(self.width() - tw - 12, self.height() - 12, txt)


class PathfinderTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.bt_thread = None
        self.shared_serial_host = None

        self.last_path_ack = None
        self.last_rx_cmd = None

        self.build_ui()

    @staticmethod
    def wrap_angle_deg(angle_deg):
        while angle_deg > 180.0:
            angle_deg -= 360.0
        while angle_deg < -180.0:
            angle_deg += 360.0
        return angle_deg

    def transform_inbound_pose(self, x, y, h):
        return (
            x,
            y,
            self.wrap_angle_deg(h),
        )

    def transform_inbound_origin(self, x, y):
        return (
            x,
            y,
        )

    def transform_outbound_waypoint(self, x, y):
        return (
            int(round(x)),
            int(round(y)),
        )

    def build_ui(self):
        root = QHBoxLayout(self)

        left_panel = QFrame()
        left_panel.setFrameShape(QFrame.StyledPanel)
        left_panel.setStyleSheet("""
            QLabel { font-size: 20px; }
            QCheckBox { font-size: 20px; }
            QDoubleSpinBox { font-size: 20px; }
            QComboBox { font-size: 20px; }
        """)
        left_layout = QVBoxLayout(left_panel)

        conn_layout = QGridLayout()
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setInsertPolicy(QComboBox.NoInsert)
        self.port_combo.setEditText(DEFAULT_DEVICE_NAME)

        self.refresh_btn = QPushButton("Refresh Devices")
        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")

        conn_layout.addWidget(QLabel("Device"), 0, 0)
        conn_layout.addWidget(self.port_combo, 0, 1)
        conn_layout.addWidget(self.refresh_btn, 2, 0)
        conn_layout.addWidget(self.connect_btn, 2, 1)
        conn_layout.addWidget(self.disconnect_btn, 3, 0, 1, 2)

        left_layout.addLayout(conn_layout)

        controls = QGridLayout()
        self.set_origin_btn = QPushButton("Set Origin = Robot")
        self.clear_origin_btn = QPushButton("Clear Origin")
        self.clear_path_btn = QPushButton("Clear Path")
        self.clear_trail_btn = QPushButton("Clear Trail")
        self.send_path_btn = QPushButton("Send Path")
        self.zero_yaw_btn = QPushButton("Send ZERO_YAW")
        self.start_btn = QPushButton("Send START")
        self.stop_btn = QPushButton("Send STOP")
        self.status_btn = QPushButton("Send STATUS")

        controls.addWidget(self.set_origin_btn, 0, 0)
        controls.addWidget(self.clear_origin_btn, 0, 1)
        controls.addWidget(self.clear_path_btn, 1, 0)
        controls.addWidget(self.clear_trail_btn, 1, 1)
        controls.addWidget(self.send_path_btn, 2, 0)
        controls.addWidget(self.zero_yaw_btn, 2, 1)
        controls.addWidget(self.start_btn, 3, 0)
        controls.addWidget(self.stop_btn, 3, 1)
        controls.addWidget(self.status_btn, 4, 0, 1, 2)

        left_layout.addSpacing(8)
        left_layout.addLayout(controls)

        settings = QGridLayout()
        self.grid_spin = QDoubleSpinBox()
        self.grid_spin.setRange(2.0, 100.0)
        self.grid_spin.setValue(10.0)
        self.grid_spin.setSuffix(" cm/grid")

        self.draw_enable_cb = QCheckBox("Enable Drawing")
        self.draw_enable_cb.setChecked(True)

        self.show_trail_cb = QCheckBox("Show Trail")
        self.show_trail_cb.setChecked(True)

        self.show_labels_cb = QCheckBox("Show Labels")
        self.show_labels_cb.setChecked(True)

        self.show_heading_cb = QCheckBox("Show Heading Vector")
        self.show_heading_cb.setChecked(True)

        settings.addWidget(QLabel("Grid Scale"), 0, 0)
        settings.addWidget(self.grid_spin, 0, 1)
        settings.addWidget(self.draw_enable_cb, 1, 0, 1, 2)
        settings.addWidget(self.show_trail_cb, 2, 0, 1, 2)
        settings.addWidget(self.show_labels_cb, 3, 0, 1, 2)
        settings.addWidget(self.show_heading_cb, 4, 0, 1, 2)

        left_layout.addSpacing(8)
        left_layout.addLayout(settings)

        left_layout.addStretch()

        self.canvas = GridCanvas()

        self.pose_x_label = QLabel("X: 0.0 cm")
        self.pose_y_label = QLabel("Y: 0.0 cm")
        self.pose_heading_label = QLabel("Heading: 0.0 deg")
        self.path_count_label = QLabel("Waypoints: 0")
        self.status_label = QLabel("Status: Idle")

        pose_bar = QHBoxLayout()
        pose_bar.setSpacing(20)
        pose_bar.addStretch()
        for lbl in (self.pose_x_label, self.pose_y_label,
                    self.pose_heading_label, self.path_count_label,
                    self.status_label):
            lbl.setStyleSheet("font-size: 14px;")
            pose_bar.addWidget(lbl)

        right_panel = QVBoxLayout()
        right_panel.setSpacing(6)
        right_panel.addWidget(self.canvas)
        right_panel.addLayout(pose_bar)

        left_panel.setMinimumWidth(340)
        root.addWidget(left_panel, 2)
        root.addLayout(right_panel, 3)

        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.connect_serial)
        self.disconnect_btn.clicked.connect(self.disconnect_serial)

        self.set_origin_btn.clicked.connect(self.set_origin_from_robot)
        self.clear_origin_btn.clicked.connect(self.canvas.clear_origin)
        self.clear_path_btn.clicked.connect(self.canvas.clear_path)
        self.clear_trail_btn.clicked.connect(self.canvas.clear_trail)
        self.send_path_btn.clicked.connect(self.send_path)
        self.zero_yaw_btn.clicked.connect(lambda: self.send_command("ZERO_YAW"))
        self.start_btn.clicked.connect(lambda: self.send_command("START"))
        self.stop_btn.clicked.connect(lambda: self.send_command("TRACK_OFF"))
        self.status_btn.clicked.connect(lambda: self.send_command("STATUS"))

        self.grid_spin.valueChanged.connect(self.on_scale_changed)
        self.draw_enable_cb.toggled.connect(self.on_draw_toggle)
        self.show_trail_cb.toggled.connect(self.on_trail_toggle)
        self.show_labels_cb.toggled.connect(self.on_labels_toggle)
        self.show_heading_cb.toggled.connect(self.on_heading_toggle)

        self.canvas.path_changed.connect(self.update_path_count)

        self.refresh_ports()
        self.update_pose_labels()

    def set_shared_serial_host(self, host):
        self.shared_serial_host = host
        self.sync_shared_serial_controls()

    def sync_shared_serial_controls(self):
        shared = self.shared_serial_host is not None
        self.port_combo.setEnabled(not shared)
        self.refresh_btn.setEnabled(not shared)
        self.connect_btn.setEnabled(not shared)
        self.disconnect_btn.setEnabled(not shared)

        if not shared:
            return

        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        for i in range(self.shared_serial_host.port_combo.count()):
            self.port_combo.addItem(self.shared_serial_host.port_combo.itemText(i))
        current_text = self.shared_serial_host.port_combo.currentText()
        if current_text:
            self.port_combo.setCurrentText(current_text)
        self.port_combo.blockSignals(False)

    def refresh_ports(self):
        if self.shared_serial_host is not None:
            self.shared_serial_host.refresh_ports()
            self.sync_shared_serial_controls()
            return

        current = self.port_combo.currentText()
        self.port_combo.blockSignals(True)
        self.port_combo.clear()

        devices = scan_ble_devices(timeout=5.0)

        for label, addr in devices:
            self.port_combo.addItem(label, addr)

        if self.port_combo.findText(DEFAULT_DEVICE_NAME) < 0:
            self.port_combo.addItem(DEFAULT_DEVICE_NAME, "")

        if current:
            index = self.port_combo.findText(current)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)
            else:
                self.port_combo.setEditText(current)
        else:
            self.port_combo.setEditText(DEFAULT_DEVICE_NAME)

        self.port_combo.blockSignals(False)

    def connect_serial(self):
        if self.shared_serial_host is not None:
            self.shared_serial_host.connect_serial()
            self.sync_shared_serial_controls()
            return

        if self.bt_thread is not None:
            return

        device_address = self.port_combo.currentData()
        device_name = self.port_combo.currentText().strip() or DEFAULT_DEVICE_NAME
        if not device_address and not device_name:
            QMessageBox.warning(self, "No Device", "Select or enter a Bluetooth device first.")
            return

        self.bt_thread = BleWorker(device_address=device_address or None, device_name=device_name)
        self.bt_thread.line_received.connect(self.handle_serial_line)
        self.bt_thread.status_message.connect(self.handle_serial_status)
        self.bt_thread.start()

    def disconnect_serial(self):
        if self.shared_serial_host is not None:
            self.shared_serial_host.disconnect_serial()
            self.sync_shared_serial_controls()
            return

        if self.bt_thread is not None:
            self.bt_thread.stop()
            self.bt_thread.wait(1000)
            self.bt_thread = None

    def handle_serial_status(self, text):
        self.status_label.setText(f"Status: {text}")

    def handle_serial_line(self, line):
        line = line.strip()
        if not line:
            return

        if line.startswith("{") and line.endswith("}"):
            try:
                pkt = json.loads(line)
                self.handle_json_packet(pkt)
                return
            except Exception:
                return

        parts = [p.strip() for p in line.split(",")]
        tag = parts[0].upper()

        if tag == "PATH_ACK":
            if len(parts) >= 2:
                self.last_path_ack = parts[1].upper()
                self.status_label.setText(f"Status: Path {parts[1].lower()}")
            return

        if tag == "RX_CMD" and len(parts) >= 2:
            self.last_rx_cmd = ",".join(parts[1:])
            return

        if tag == "ORIGIN":
            if len(parts) >= 3:
                try:
                    x = self.parse_scaled_int(parts[1])
                    y = self.parse_scaled_int(parts[2])
                except Exception:
                    x, y = 0.0, 0.0
            else:
                x, y = 0.0, 0.0

            x, y = self.transform_inbound_origin(x, y)
            self.canvas.set_origin(x, y)
            return

        if tag == "POSE" and len(parts) >= 4:
            try:
                x = self.parse_scaled_int(parts[1])
                y = self.parse_scaled_int(parts[2])
                h = self.parse_scaled_int(parts[3])
                x, y, h = self.transform_inbound_pose(x, y, h)
                self.apply_robot_pose(x, y, h, allow_large_jump=False)
            except Exception:
                pass
            return

    def handle_json_packet(self, pkt):
        ptype = str(pkt.get("type", "")).lower()

        if ptype == "origin":
            x = float(pkt.get("x", 0.0))
            y = float(pkt.get("y", 0.0))
            x, y = self.transform_inbound_origin(x, y)
            self.canvas.set_origin(x, y)
            return

        if ptype == "pose":
            x = float(pkt.get("x", 0.0))
            y = float(pkt.get("y", 0.0))
            h = float(pkt.get("heading", 0.0))
            x, y, h = self.transform_inbound_pose(x, y, h)
            self.apply_robot_pose(x, y, h, allow_large_jump=False)

    def apply_robot_pose(self, x, y, h, allow_large_jump=False):
        old_x = self.canvas.robot_x
        old_y = self.canvas.robot_y

        if not allow_large_jump:
            max_jump_cm = 100.0
            dx = x - old_x
            dy = y - old_y
            if math.hypot(dx, dy) > max_jump_cm:
                return

        self.canvas.set_robot_pose(x, y, h)
        self.update_pose_labels()

    def set_origin_from_robot(self):
        self.canvas.set_origin(self.canvas.robot_x, self.canvas.robot_y)

    def on_scale_changed(self, value):
        self.canvas.cm_per_grid = value
        self.canvas.px_per_cm = self.canvas.grid_spacing_px / self.canvas.cm_per_grid
        self.canvas.update()

    def on_draw_toggle(self, checked):
        self.canvas.drawing_enabled = checked

    def on_trail_toggle(self, checked):
        self.canvas.show_trail = checked
        self.canvas.update()

    def on_labels_toggle(self, checked):
        self.canvas.show_labels = checked
        self.canvas.update()

    def on_heading_toggle(self, checked):
        self.canvas.show_heading_vector = checked
        self.canvas.update()

    def update_path_count(self):
        self.path_count_label.setText(f"Waypoints: {len(self.canvas.path_points)}")

    def update_pose_labels(self):
        self.pose_x_label.setText(f"X: {self.canvas.robot_x:.1f} cm")
        self.pose_y_label.setText(f"Y: {self.canvas.robot_y:.1f} cm")
        self.pose_heading_label.setText(f"Heading: {self.canvas.robot_heading_deg:.1f} deg")
        self.update_path_count()

    @staticmethod
    def parse_scaled_int(text):
        return int(text) / 100.0

    @staticmethod
    def encode_int(value):
        return str(int(value))

    def send_command(self, cmd):
        if self.shared_serial_host is not None:
            self.shared_serial_host.send_serial_command(f"{cmd}\n")
            return

        if self.bt_thread is None:
            return

        ok = self.bt_thread.send_text(cmd + "\n")
        _ = ok

    def send_command_with_delay(self, cmd, delay_s=0.05):
        self.send_command(cmd)
        QApplication.processEvents()
        time.sleep(delay_s)
        QApplication.processEvents()

    def wait_for_ack(self, attr_name, expected_value, timeout_s=1.5):
        deadline = time.time() + timeout_s
        expected_value = expected_value.upper()
        while time.time() < deadline:
            QApplication.processEvents()
            if getattr(self, attr_name) == expected_value:
                return True
            time.sleep(0.01)
        return False

    def send_path_end_with_retry(self, retries=2):
        for attempt in range(retries):
            self.last_path_ack = None
            self.send_command_with_delay("PATH_END", delay_s=0.35)
            if self.wait_for_ack("last_path_ack", "LOADED", timeout_s=4.0):
                return True
            if attempt < (retries - 1):
                QApplication.processEvents()
                time.sleep(0.25)
                QApplication.processEvents()
        return False

    def wait_for_rx_cmd(self, expected_cmd, timeout_s=1.5):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            QApplication.processEvents()
            if self.last_rx_cmd == expected_cmd:
                return True
            time.sleep(0.01)
        return False

    def send_waypoint_with_retry(self, cmd, retries=2):
        for attempt in range(retries):
            self.last_rx_cmd = None
            self.send_command_with_delay(cmd, delay_s=0.20)
            if self.wait_for_rx_cmd(cmd, timeout_s=1.2):
                return True
            if attempt < (retries - 1):
                QApplication.processEvents()
                time.sleep(0.20)
                QApplication.processEvents()
        return False

    def send_path(self):
        if not self.canvas.path_points:
            QMessageBox.warning(self, "No Path", "Draw a path first.")
            return

        if self.shared_serial_host is None and self.bt_thread is None:
            QMessageBox.warning(self, "Not Connected", "Connect Bluetooth before sending the path.")
            return

        self.last_path_ack = None
        self.send_command_with_delay(f"PATH_BEGIN,{len(self.canvas.path_points)}")
        if not self.wait_for_ack("last_path_ack", "BEGIN"):
            return

        for i, (x, y) in enumerate(self.canvas.path_points):
            tx, ty = self.transform_outbound_waypoint(x, y)
            cmd = (
                f"WPT,{i},"
                f"{self.encode_int(tx)},{self.encode_int(ty)}"
            )
            if not self.send_waypoint_with_retry(cmd):
                return

        QApplication.processEvents()
        time.sleep(0.40)
        QApplication.processEvents()

        if not self.send_path_end_with_retry():
            return

    def closeEvent(self, event):
        self.disconnect_serial()
        event.accept()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyQt IMU Grid Path Planner")
        self.resize(1450, 860)
        self.pathfinder_tab = PathfinderTab(self)
        self.setCentralWidget(self.pathfinder_tab)


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
