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
    QLabel, QPushButton, QComboBox, QTextEdit, QFileDialog, QMessageBox,
    QGroupBox, QCheckBox, QSlider, QDoubleSpinBox, QTabWidget
)

ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.append(str(ROOT_DIR))

from ble_receiver import BleWorker, DEFAULT_DEVICE_NAME, scan_ble_devices
from pathfinder import PathfinderTab


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
        self.sample_rate_hz = 0.0

    def set_vehicle_state(self, roll_deg, pitch_deg, accel, gyro, connected, moving, sample_rate_hz):
        self.roll_deg = roll_deg
        self.pitch_deg = pitch_deg
        self.accel = accel
        self.gyro = gyro
        self.connected = connected
        self.moving = moving
        self.sample_rate_hz = sample_rate_hz
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        bg = QLinearGradient(0, 0, 0, self.height())
        bg.setColorAt(0.0, QColor(16, 20, 28))
        bg.setColorAt(1.0, QColor(5, 8, 14))
        painter.fillRect(self.rect(), bg)

        w = self.width()
        h = self.height()
        cx = w / 2.0

        painter.setPen(QPen(QColor(48, 60, 78), 2))
        lane_left = int(cx - 110)
        lane_right = int(cx + 110)
        painter.drawLine(lane_left, 40, lane_left, h - 110)
        painter.drawLine(lane_right, 40, lane_right, h - 110)

        painter.setPen(QPen(QColor(80, 94, 118), 2, Qt.DashLine))
        painter.drawLine(int(cx), 40, int(cx), h - 110)

        car_w = 150.0
        car_h = 290.0
        body_x = cx - car_w / 2.0 + max(min(self.roll_deg, 22.0), -22.0) * 1.1
        body_y = 120.0 + max(min(self.pitch_deg, 22.0), -22.0) * 1.2
        body = QRectF(body_x, body_y, car_w, car_h)

        shell_grad = QLinearGradient(body.left(), body.top(), body.right(), body.bottom())
        shell_grad.setColorAt(0.0, QColor(228, 232, 238))
        shell_grad.setColorAt(0.45, QColor(145, 153, 165))
        shell_grad.setColorAt(1.0, QColor(54, 60, 70))
        painter.setBrush(QBrush(shell_grad))
        painter.setPen(QPen(QColor(245, 248, 255), 2))
        painter.drawRoundedRect(body, 34, 34)

        cabin = QRectF(body.left() + 18, body.top() + 30, car_w - 36, car_h - 95)
        painter.setBrush(QColor(28, 37, 52, 220))
        painter.setPen(QPen(QColor(126, 180, 224), 2))
        painter.drawRoundedRect(cabin, 26, 26)

        wheel_color = QColor(70, 74, 80)
        wheel_glow = QColor(255, 164, 79) if self.moving else QColor(103, 187, 255)
        painter.setBrush(wheel_color)
        painter.setPen(QPen(wheel_glow, 3))
        wheel_rects = [
            QRectF(body.left() - 18, body.top() + 34, 18, 64),
            QRectF(body.right(), body.top() + 34, 18, 64),
            QRectF(body.left() - 18, body.bottom() - 98, 18, 64),
            QRectF(body.right(), body.bottom() - 98, 18, 64),
        ]
        for wheel in wheel_rects:
            painter.drawRoundedRect(wheel, 8, 8)

        accel_mag = min((self.accel[0] * self.accel[0] + self.accel[1] * self.accel[1] + self.accel[2] * self.accel[2]) ** 0.5, 2.0)
        ring_color = QColor(255, 140, 92, 120) if self.moving else QColor(83, 196, 255, 90)
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(ring_color, 8))
        painter.drawEllipse(QRectF(cx - 125 - accel_mag * 18, body.top() - 24 - accel_mag * 10, 250 + accel_mag * 36, 340 + accel_mag * 20))

        title_font_size = 18 if w >= 720 else 15
        status_font_size = 10 if w >= 720 else 9
        metric_value_font_size = 20 if w >= 720 else 16
        metric_unit_font_size = 9 if w >= 720 else 8

        painter.setPen(QColor(245, 247, 250))
        painter.setFont(QFont("Segoe UI", title_font_size, QFont.Bold))
        painter.drawText(36, 42, "Vehicle Visualization")

        painter.setFont(QFont("Segoe UI", status_font_size))
        status = "CONNECTED" if self.connected else "DISCONNECTED"
        motion = "MOVING" if self.moving else "STABLE"
        status_rect = QRectF(36, 50, max(120.0, w * 0.22), 24)
        motion_rect = QRectF(status_rect.right() + 14, 50, max(90.0, w * 0.16), 24)
        painter.setPen(QColor(115, 220, 129) if self.connected else QColor(255, 114, 114))
        painter.drawText(status_rect, Qt.AlignLeft | Qt.AlignVCenter, status)
        painter.setPen(QColor(255, 182, 92) if self.moving else QColor(133, 206, 255))
        painter.drawText(motion_rect, Qt.AlignLeft | Qt.AlignVCenter, motion)

        card_y = h - 138
        card_h = 88
        card_w = (w - 72) / 3.0
        metrics = [
            ("Roll", self.roll_deg, "deg"),
            ("Pitch", self.pitch_deg, "deg"),
            ("Rate", self.sample_rate_hz, "Hz"),
        ]

        for index, (label, value, unit) in enumerate(metrics):
            left = 24 + index * (card_w + 12)
            rect = QRectF(left, card_y, card_w, card_h)
            painter.setPen(QPen(QColor(54, 65, 84), 1))
            painter.setBrush(QColor(18, 24, 34, 220))
            painter.drawRoundedRect(rect, 18, 18)
            painter.setPen(QColor(133, 149, 173))
            painter.setFont(QFont("Segoe UI", metric_unit_font_size))
            painter.drawText(QRectF(rect.left() + 16, rect.top() + 10, rect.width() - 32, 18), Qt.AlignLeft | Qt.AlignVCenter, label)
            painter.setPen(QColor(247, 250, 255))
            painter.setFont(QFont("Segoe UI", metric_value_font_size, QFont.Bold))
            value_rect = QRectF(rect.left() + 16, rect.top() + 28, rect.width() - 64, 32)
            painter.drawText(value_rect, Qt.AlignLeft | Qt.AlignVCenter, f"{value:.1f}")
            painter.setPen(QColor(133, 149, 173))
            painter.setFont(QFont("Segoe UI", metric_unit_font_size))
            unit_rect = QRectF(rect.right() - 44, rect.top() + 34, 28, 20)
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


class ImuGui(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("STM32 Serial IMU Viewer")
        self.resize(1500, 950)

        self.worker = None
        self.logger = CsvLogger()

        self.max_samples = 300
        self.sample_counter = 0
        self.packet_count = 0
        self.last_packet_time = None
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

        self.filtered_roll = 0.0
        self.filtered_pitch = 0.0
        self.last_time = None

        self.accel_bias = {"ax": 0.0, "ay": 0.0, "az": 0.0}
        self.gyro_bias = {"gx": 0.0, "gy": 0.0, "gz": 0.0}

        self.latest_raw = None

        self.init_ui()
        self.pathfinder_tab.set_shared_serial_host(self)
        self.refresh_ports()
        self.update_alpha_label()
        self.update_plot_visibility()
        self.update_orientation_curve_visibility()
        QTimer.singleShot(0, self.auto_connect_serial)

        self.link_timer = QTimer(self)
        self.link_timer.timeout.connect(self.check_link_status)
        self.link_timer.start(250)

    def init_ui(self):
        main_layout = QVBoxLayout()
        self.setLayout(main_layout)

        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        self.imu_tab = QWidget()
        self.pathfinder_tab = PathfinderTab()

        self.tabs.addTab(self.imu_tab, "IMU")
        self.tabs.addTab(self.pathfinder_tab, "Pathfinder")

        tab_layout = QVBoxLayout()
        self.imu_tab.setLayout(tab_layout)

        self.robot_visual = RobotVisualizationWidget()
        tab_layout.addWidget(self.robot_visual)

        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setInsertPolicy(QComboBox.NoInsert)
        self.port_combo.setEditText(DEFAULT_DEVICE_NAME)

        self.refresh_btn = QPushButton("Refresh Devices")
        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)
        self.reconnect_btn = QPushButton("Reconnect")
        self.reconnect_btn.setEnabled(False)

        self.log_btn = QPushButton("Start Log")
        self.stop_log_btn = QPushButton("Stop Log")
        self.stop_log_btn.setEnabled(False)

        self.status_label = QLabel("Status: Disconnected")
        self.console = QTextEdit()

        self.labels = {}
        rows = [
            "ax", "ay", "az", "gx", "gy", "gz",
            "roll", "pitch", "imu_roll", "imu_pitch",
            "accel_roll", "accel_pitch", "sample_rate",
            "packets", "last_packet",
        ]
        for key in rows:
            self.labels[key] = QLabel("0.00")

        self.connection_indicator = QLabel("Disconnected")
        self.link_indicator = QLabel("No Link")
        self.motion_indicator = QLabel("Unknown")
        self.tilt_indicator = QLabel("No Alarm")
        self.filter_indicator = QLabel("IMU Angles")

        self.ping_btn = QPushButton("Ping STM32")
        self.reset_link_btn = QPushButton("Reset Link Status")
        self.calib_accel_btn = QPushButton("Calibrate Accel")
        self.calib_gyro_btn = QPushButton("Calibrate Gyro")
        self.reset_orientation_btn = QPushButton("Reset Orientation")
        self.clear_plots_btn = QPushButton("Clear Plots")
        self.estop_btn = QPushButton("EMERGENCY STOP")
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
        self.link_timeout = QDoubleSpinBox()
        self.link_timeout.setRange(0.1, 10.0)
        self.link_timeout.setValue(1.0)
        self.link_timeout.setDecimals(2)

        bluetooth_group = QGroupBox("Bluetooth")
        bluetooth_layout = QGridLayout()
        bluetooth_layout.addWidget(QLabel("Device"), 0, 0)
        bluetooth_layout.addWidget(self.port_combo, 0, 1, 1, 3)
        bluetooth_layout.addWidget(self.refresh_btn, 1, 0)
        bluetooth_layout.addWidget(self.connect_btn, 1, 1)
        bluetooth_layout.addWidget(self.disconnect_btn, 1, 2)
        bluetooth_layout.addWidget(self.reconnect_btn, 1, 3)
        bluetooth_layout.addWidget(self.status_label, 2, 0, 1, 4)
        bluetooth_group.setLayout(bluetooth_layout)
        tab_layout.addWidget(bluetooth_group)

        self.console.setReadOnly(True)
        tab_layout.addWidget(self.console)

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
        self.reset_link_btn.clicked.connect(self.reset_link_status)
        self.estop_btn.clicked.connect(self.emergency_stop)

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
            self.append_console("Already connected")
            return

        port = self.get_selected_port()
        device_name = self.port_combo.currentText().strip() or DEFAULT_DEVICE_NAME
        if not port and not device_name:
            QMessageBox.warning(self, "No Device", "No Bluetooth device selected")
            return

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
            0.0,
        )
        self.pathfinder_tab.handle_serial_status("Connected" if connected else "Disconnected")
        self.pathfinder_tab.sync_shared_serial_controls()
        if connected:
            self.status_label.setText("Status: Connected")
            self.status_label.setStyleSheet("font-weight: bold; color: green;")
            self.connection_indicator.setText("Connected")
            self.connection_indicator.setStyleSheet("font-weight: bold; color: green;")
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.reconnect_btn.setEnabled(True)
        else:
            self.status_label.setText("Status: Disconnected")
            self.status_label.setStyleSheet("font-weight: bold; color: black;")
            self.connection_indicator.setText("Disconnected")
            self.connection_indicator.setStyleSheet("font-weight: bold; color: red;")
            self.link_indicator.setText("No Link")
            self.link_indicator.setStyleSheet("font-weight: bold; color: red;")
            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)
            self.reconnect_btn.setEnabled(True)

    def handle_ble_line(self, line):
        self.pathfinder_tab.handle_serial_line(line)
        parsed = self.parse_line(line)
        if parsed is not None:
            self.handle_data(parsed)

    def parse_line(self, line):
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
        self.last_packet_time = now
        self.packet_count += 1
        self.latest_raw = d

        ax_raw = d["ax"]
        ay_raw = d["ay"]
        az_raw = d["az"]
        gx_raw = d["gx"]
        gy_raw = d["gy"]
        gz_raw = d["gz"]

        imu_roll_raw = d.get("imu_roll", 0.0)
        imu_pitch_raw = d.get("imu_pitch", 0.0)

        ax = ax_raw - self.accel_bias["ax"]
        ay = ay_raw - self.accel_bias["ay"]
        az = az_raw - self.accel_bias["az"]

        gx = gx_raw - self.gyro_bias["gx"]
        gy = gy_raw - self.gyro_bias["gy"]
        gz = gz_raw - self.gyro_bias["gz"]

        accel_roll, accel_pitch = compute_roll_pitch(ax, ay, az)

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

        self.link_indicator.setText("Link Alive")
        self.link_indicator.setStyleSheet("font-weight: bold; color: green;")

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
            sample_rate,
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
            self.motion_indicator.setStyleSheet("font-weight: bold; color: orange;")
        else:
            self.motion_indicator.setText("Stationary")
            self.motion_indicator.setStyleSheet("font-weight: bold; color: green;")

    def check_tilt_alarm(self, roll, pitch):
        if not self.tilt_alarm_checkbox.isChecked():
            self.tilt_indicator.setText("Alarm Disabled")
            self.tilt_indicator.setStyleSheet("font-weight: bold; color: gray;")
            return

        roll_limit = self.roll_thresh.value()
        pitch_limit = self.pitch_thresh.value()

        if abs(roll) > roll_limit or abs(pitch) > pitch_limit:
            self.tilt_indicator.setText("TILT ALARM")
            self.tilt_indicator.setStyleSheet("font-weight: bold; color: red;")
            if self.connection_indicator.text() == "Connected":
                self.status_label.setText("Status: TILT ALARM")
                self.status_label.setStyleSheet("font-weight: bold; color: red;")
        else:
            self.tilt_indicator.setText("No Alarm")
            self.tilt_indicator.setStyleSheet("font-weight: bold; color: green;")
            if self.connection_indicator.text() == "Connected":
                self.status_label.setText("Status: Connected")
                self.status_label.setStyleSheet("font-weight: bold; color: green;")

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
        self.last_packet_time = None
        self.packet_count = 0
        self.labels["packets"].setText("0")
        self.labels["last_packet"].setText("N/A")
        self.link_indicator.setText("Waiting")
        self.link_indicator.setStyleSheet("font-weight: bold; color: orange;")
        self.append_console("Serial link status reset")

    def check_link_status(self):
        if self.connection_indicator.text() != "Connected":
            self.labels["last_packet"].setText("N/A")
            return

        if self.last_packet_time is None:
            self.link_indicator.setText("Waiting")
            self.link_indicator.setStyleSheet("font-weight: bold; color: orange;")
            self.labels["last_packet"].setText("No data")
            return

        age = time.time() - self.last_packet_time
        self.labels["last_packet"].setText(f"{age:.2f} s ago")

        if age > self.link_timeout.value():
            self.link_indicator.setText("Link Lost")
            self.link_indicator.setStyleSheet("font-weight: bold; color: red;")
            if self.connection_indicator.text() == "Connected":
                self.status_label.setText("Status: Link Lost")
                self.status_label.setStyleSheet("font-weight: bold; color: red;")
        else:
            self.link_indicator.setText("Link Alive")
            self.link_indicator.setStyleSheet("font-weight: bold; color: green;")
            if self.tilt_indicator.text() != "TILT ALARM" and self.connection_indicator.text() == "Connected":
                self.status_label.setText("Status: Connected")
                self.status_label.setStyleSheet("font-weight: bold; color: green;")

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

    def closeEvent(self, event):
        try:
            self.stop_logging()
            self.disconnect_serial()
        except Exception:
            pass
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = ImuGui()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
