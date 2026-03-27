import sys
import math
import time
import json

from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, QPointF
from PyQt5.QtGui import QPainter, QPen, QColor, QBrush, QFont, QPolygonF
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QComboBox,
    QVBoxLayout, QHBoxLayout, QGridLayout, QTextEdit, QMessageBox,
    QDoubleSpinBox, QCheckBox, QSizePolicy, QFrame
)

try:
    import serial
    import serial.tools.list_ports
except Exception:
    serial = None


class SerialReaderThread(QThread):
    line_received = pyqtSignal(str)
    status_changed = pyqtSignal(str)

    def __init__(self, port_name, baudrate=115200, parent=None):
        super().__init__(parent)
        self.port_name = port_name
        self.baudrate = baudrate
        self.running = False
        self.ser = None

    def run(self):
        if serial is None:
            self.status_changed.emit("pyserial not installed")
            return

        try:
            self.ser = serial.Serial(self.port_name, self.baudrate, timeout=0.1)
            self.running = True
            self.status_changed.emit(f"Connected to {self.port_name} @ {self.baudrate}")
        except Exception as e:
            self.status_changed.emit(f"Serial open failed: {e}")
            return

        buffer = b""
        while self.running:
            try:
                data = self.ser.read(256)
                if data:
                    buffer += data
                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        decoded = line.decode("utf-8", errors="ignore").strip()
                        if decoded:
                            self.line_received.emit(decoded)
            except Exception as e:
                self.status_changed.emit(f"Serial read error: {e}")
                break

        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass

        self.status_changed.emit("Disconnected")

    def send_line(self, text):
        try:
            if self.ser and self.ser.is_open:
                self.ser.write((text + "\n").encode("utf-8"))
                return True
        except Exception:
            pass
        return False

    def stop(self):
        self.running = False


class GridCanvas(QWidget):
    point_added = pyqtSignal(float, float)
    path_changed = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(900, 700)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)

        self.grid_spacing_px = 40
        self.cm_per_grid = 10.0
        self.px_per_cm = self.grid_spacing_px / self.cm_per_grid

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
        cx = self.width() / 2.0
        cy = self.height() / 2.0
        sx = cx + (x_cm - self.origin_world_x) * self.px_per_cm
        sy = cy - (y_cm - self.origin_world_y) * self.px_per_cm
        return sx, sy

    def screen_to_world(self, sx, sy):
        cx = self.width() / 2.0
        cy = self.height() / 2.0
        x_cm = self.origin_world_x + (sx - cx) / self.px_per_cm
        y_cm = self.origin_world_y + (cy - sy) / self.px_per_cm
        return x_cm, y_cm

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self.drawing_enabled:
            x_cm, y_cm = self.screen_to_world(event.x(), event.y())
            self.path_points.append((x_cm, y_cm))
            self.point_added.emit(x_cm, y_cm)
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

        w = self.width()
        h = self.height()
        cx = w / 2.0
        cy = h / 2.0
        spacing = self.grid_spacing_px

        num_v = int(w / spacing) + 3
        num_h = int(h / spacing) + 3

        for i in range(-num_v, num_v + 1):
            x = cx + i * spacing
            painter.setPen(pen_major if i % 5 == 0 else pen_minor)
            painter.drawLine(int(x), 0, int(x), h)

        for j in range(-num_h, num_h + 1):
            y = cy + j * spacing
            painter.setPen(pen_major if j % 5 == 0 else pen_minor)
            painter.drawLine(0, int(y), w, int(y))

    def draw_axes(self, painter):
        cx = self.width() / 2.0
        cy = self.height() / 2.0

        painter.setPen(QPen(QColor(120, 120, 150), 1))
        painter.drawLine(0, int(cy), self.width(), int(cy))
        painter.drawLine(int(cx), 0, int(cx), self.height())

        if self.show_labels:
            painter.setPen(QColor(220, 220, 220))
            painter.setFont(QFont("Arial", 11))
            painter.drawText(int(cx + 8), 18, "+Y")
            painter.drawText(self.width() - 28, int(cy - 8), "+X")

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
                painter.drawText(int(sx + 8), int(sy - 8), f"P{i + 1}")

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
        txt = f"Mouse: X={x:.1f} cm   Y={y:.1f} cm"
        painter.setPen(QColor(220, 220, 220))
        painter.setFont(QFont("Consolas", 10))
        painter.drawText(12, self.height() - 12, txt)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyQt IMU Grid Path Planner")
        self.resize(1450, 860)

        self.serial_thread = None

        self.last_good_pose = (0.0, 0.0, 0.0)
        self.last_path_ack = None
        self.last_track_ack = None
        self.last_rx_cmd = None

        self.build_ui()

        self.port_timer = QTimer(self)
        self.port_timer.timeout.connect(self.refresh_ports)
        self.port_timer.start(1500)

    def build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)

        root = QHBoxLayout(central)

        left_panel = QFrame()
        left_panel.setFrameShape(QFrame.StyledPanel)
        left_layout = QVBoxLayout(left_panel)

        conn_layout = QGridLayout()
        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["115200", "57600", "38400", "9600"])
        self.baud_combo.setCurrentText("115200")

        self.refresh_btn = QPushButton("Refresh Ports")
        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")

        conn_layout.addWidget(QLabel("Port"), 0, 0)
        conn_layout.addWidget(self.port_combo, 0, 1)
        conn_layout.addWidget(QLabel("Baud"), 1, 0)
        conn_layout.addWidget(self.baud_combo, 1, 1)
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

        pose_layout = QGridLayout()
        self.pose_x_label = QLabel("X: 0.0 cm")
        self.pose_y_label = QLabel("Y: 0.0 cm")
        self.pose_heading_label = QLabel("Heading: 0.0 deg")
        self.path_count_label = QLabel("Waypoints: 0")
        self.status_label = QLabel("Status: Idle")

        pose_layout.addWidget(self.pose_x_label, 0, 0)
        pose_layout.addWidget(self.pose_y_label, 1, 0)
        pose_layout.addWidget(self.pose_heading_label, 2, 0)
        pose_layout.addWidget(self.path_count_label, 3, 0)
        pose_layout.addWidget(self.status_label, 4, 0)

        left_layout.addSpacing(8)
        left_layout.addLayout(pose_layout)

        left_layout.addWidget(QLabel("Serial / Debug Log"))
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        left_layout.addWidget(self.log_box, 1)

        self.canvas = GridCanvas()

        root.addWidget(left_panel, 0)
        root.addWidget(self.canvas, 1)

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

        self.canvas.point_added.connect(self.on_point_added)
        self.canvas.path_changed.connect(self.update_path_count)

        self.refresh_ports()
        self.update_pose_labels()

    def log(self, text):
        self.log_box.append(text)

    def refresh_ports(self):
        current = self.port_combo.currentText()
        self.port_combo.blockSignals(True)
        self.port_combo.clear()

        if serial is not None:
            ports = [p.device for p in serial.tools.list_ports.comports()]
        else:
            ports = []

        self.port_combo.addItems(ports)
        if current in ports:
            self.port_combo.setCurrentText(current)

        self.port_combo.blockSignals(False)

    def connect_serial(self):
        if self.serial_thread is not None:
            self.log("Already connected")
            return

        port = self.port_combo.currentText().strip()
        if not port:
            QMessageBox.warning(self, "No Port", "Select a serial port first.")
            return

        baud = int(self.baud_combo.currentText())
        self.serial_thread = SerialReaderThread(port, baud)
        self.serial_thread.line_received.connect(self.handle_serial_line)
        self.serial_thread.status_changed.connect(self.handle_serial_status)
        self.serial_thread.start()

    def disconnect_serial(self):
        if self.serial_thread is not None:
            self.serial_thread.stop()
            self.serial_thread.wait(1000)
            self.serial_thread = None

    def handle_serial_status(self, text):
        self.status_label.setText(f"Status: {text}")
        self.log(text)

    def handle_serial_line(self, line):
        self.log(line)

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

        if tag == "TRACK_ACK":
            if len(parts) >= 2:
                self.last_track_ack = parts[1].upper()
                state = parts[1].replace("_", " ").title()
                self.status_label.setText(f"Status: Tracking {state}")
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

            self.canvas.set_origin(x, y)
            self.log(f"Origin set to ({x:.1f}, {y:.1f}) cm")
            return

        if tag == "POSE" and len(parts) >= 4:
            try:
                x = self.parse_scaled_int(parts[1])
                y = self.parse_scaled_int(parts[2])
                h = self.parse_scaled_int(parts[3])
                self.apply_robot_pose(x, y, h, allow_large_jump=False)
            except Exception:
                pass
            return

    def handle_json_packet(self, pkt):
        ptype = str(pkt.get("type", "")).lower()

        if ptype == "origin":
            x = float(pkt.get("x", 0.0))
            y = float(pkt.get("y", 0.0))
            self.canvas.set_origin(x, y)
            return

        if ptype == "pose":
            x = float(pkt.get("x", 0.0))
            y = float(pkt.get("y", 0.0))
            h = float(pkt.get("heading", 0.0))
            self.apply_robot_pose(x, y, h, allow_large_jump=False)

    def apply_robot_pose(self, x, y, h, allow_large_jump=False):
        old_x = self.canvas.robot_x
        old_y = self.canvas.robot_y

        if not allow_large_jump:
            max_jump_cm = 100.0
            dx = x - old_x
            dy = y - old_y
            if math.hypot(dx, dy) > max_jump_cm:
                self.log(f"Ignored pose jump: X={x:.2f}, Y={y:.2f}, H={h:.2f}")
                return

        self.canvas.set_robot_pose(x, y, h)
        self.last_good_pose = (x, y, h)
        self.update_pose_labels()

    def set_origin_from_robot(self):
        self.canvas.set_origin(self.canvas.robot_x, self.canvas.robot_y)
        self.log("Origin set from current robot pose")

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

    def on_point_added(self, x, y):
        self.log(f"Waypoint added: {x:.1f}, {y:.1f} cm")

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
    def encode_scaled_int(value):
        return str(int(round(value * 100.0)))

    def send_command(self, cmd):
        if self.serial_thread is None:
            self.log(f"[NOT SENT] {cmd}")
            return

        ok = self.serial_thread.send_line(cmd)
        if ok:
            self.log(f"[TX] {cmd}")
        else:
            self.log(f"[TX FAIL] {cmd}")

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
                self.log("[RETRY] PATH_END")
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
                self.log(f"[RETRY] {cmd}")
                QApplication.processEvents()
                time.sleep(0.20)
                QApplication.processEvents()
        return False

    def send_path(self):
        if not self.canvas.path_points:
            QMessageBox.warning(self, "No Path", "Draw a path first.")
            return

        if self.serial_thread is None:
            self.log("[NOT SENT] PATH_BEGIN / waypoints (not connected)")
            return

        self.last_path_ack = None
        self.send_command_with_delay(f"PATH_BEGIN,{len(self.canvas.path_points)}")
        if not self.wait_for_ack("last_path_ack", "BEGIN"):
            self.log("[ACK TIMEOUT] PATH_ACK,BEGIN")
            return

        for i, (x, y) in enumerate(self.canvas.path_points):
            cmd = f"WPT,{i},{self.encode_scaled_int(x)},{self.encode_scaled_int(y)}"
            if not self.send_waypoint_with_retry(cmd):
                self.log(f"[RX TIMEOUT] {cmd}")
                return

        QApplication.processEvents()
        time.sleep(0.40)
        QApplication.processEvents()

        if not self.send_path_end_with_retry():
            self.log("[ACK TIMEOUT] PATH_ACK,LOADED")

    def closeEvent(self, event):
        self.disconnect_serial()
        event.accept()


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
