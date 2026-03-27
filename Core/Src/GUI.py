import sys
import math
import csv
import time
import re
from datetime import datetime
from collections import deque

import serial
import serial.tools.list_ports
import pyqtgraph as pg

from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QComboBox, QTextEdit, QFileDialog, QMessageBox,
    QGroupBox, QCheckBox, QSlider, QDoubleSpinBox, QTabWidget
)

from f1_dashboard import F1DashboardTab


class SerialWorker(QThread):
    data_received = pyqtSignal(dict)
    status_message = pyqtSignal(str)
    connection_changed = pyqtSignal(bool)

    def __init__(self, port, baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.ser = None
        self.pending_packet = {}

    def run(self):
        self.running = True

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(0.2)
            self.connection_changed.emit(True)
            self.status_message.emit(f"Connected to {self.port} at {self.baudrate} baud")
        except Exception as e:
            self.connection_changed.emit(False)
            self.status_message.emit(f"Serial open error: {e}")
            return

        while self.running:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        parsed = self.parse_line(line)
                        if parsed is not None:
                            self.data_received.emit(parsed)
                else:
                    self.msleep(5)
            except Exception as e:
                self.status_message.emit(f"Serial read error: {e}")
                break

        self.cleanup()

    def stop(self):
        self.running = False

    def cleanup(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

        self.connection_changed.emit(False)
        self.status_message.emit("Disconnected")

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

    def send_text(self, text):
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(text.encode("utf-8"))
                return True
        except Exception:
            pass
        return False


def compute_roll_pitch(ax, ay, az):
    try:
        roll = math.degrees(math.atan2(ay, az))
        pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
    except Exception:
        roll = 0.0
        pitch = 0.0
    return roll, pitch


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
        self.last_good_baud = "115200"

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
        self.refresh_ports()
        self.update_alpha_label()
        self.update_plot_visibility()
        self.update_orientation_curve_visibility()

        self.link_timer = QTimer(self)
        self.link_timer.timeout.connect(self.check_link_status)
        self.link_timer.start(250)

    def init_ui(self):
        main_layout = QVBoxLayout()
        self.setLayout(main_layout)

        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        self.imu_tab = QWidget()
        self.dashboard_tab = F1DashboardTab()

        self.tabs.addTab(self.imu_tab, "IMU")
        self.tabs.addTab(self.dashboard_tab, "F1 Dashboard")

        tab_layout = QVBoxLayout()
        self.imu_tab.setLayout(tab_layout)

        top_bar = QHBoxLayout()

        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200", "230400"])
        self.baud_combo.setCurrentText("115200")

        self.refresh_btn = QPushButton("Refresh Ports")
        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)
        self.reconnect_btn = QPushButton("Reconnect")
        self.reconnect_btn.setEnabled(False)

        self.log_btn = QPushButton("Start Log")
        self.stop_log_btn = QPushButton("Stop Log")
        self.stop_log_btn.setEnabled(False)

        self.status_label = QLabel("Status: Disconnected")
        self.status_label.setStyleSheet("font-weight: bold;")

        top_bar.addWidget(QLabel("Serial / COM Port"))
        top_bar.addWidget(self.port_combo)
        top_bar.addWidget(QLabel("Baud"))
        top_bar.addWidget(self.baud_combo)
        top_bar.addWidget(self.refresh_btn)
        top_bar.addWidget(self.connect_btn)
        top_bar.addWidget(self.disconnect_btn)
        top_bar.addWidget(self.reconnect_btn)
        top_bar.addWidget(self.log_btn)
        top_bar.addWidget(self.stop_log_btn)
        top_bar.addStretch()
        top_bar.addWidget(self.status_label)

        tab_layout.addLayout(top_bar)

        content = QHBoxLayout()
        tab_layout.addLayout(content)

        left_col = QVBoxLayout()
        mid_col = QVBoxLayout()
        right_col = QVBoxLayout()

        content.addLayout(left_col, 2)
        content.addLayout(mid_col, 4)
        content.addLayout(right_col, 3)

        values_group = QGroupBox("Live IMU Values")
        values_grid = QGridLayout()
        values_group.setLayout(values_grid)

        self.labels = {}
        rows = [
            ("ax", "Accel X [g]"),
            ("ay", "Accel Y [g]"),
            ("az", "Accel Z [g]"),
            ("gx", "Gyro X [dps]"),
            ("gy", "Gyro Y [dps]"),
            ("gz", "Gyro Z [dps]"),
            ("roll", "Displayed Roll [deg]"),
            ("pitch", "Displayed Pitch [deg]"),
            ("imu_roll", "IMU Roll [deg]"),
            ("imu_pitch", "IMU Pitch [deg]"),
            ("accel_roll", "Accel Roll [deg]"),
            ("accel_pitch", "Accel Pitch [deg]"),
            ("sample_rate", "Sample Rate"),
            ("packets", "Packets"),
            ("last_packet", "Last Packet"),
        ]

        for i, (key, title) in enumerate(rows):
            values_grid.addWidget(QLabel(title), i, 0)
            lbl = QLabel("0.00")
            lbl.setMinimumWidth(140)
            values_grid.addWidget(lbl, i, 1)
            self.labels[key] = lbl

        left_col.addWidget(values_group)

        indicator_group = QGroupBox("Indicators")
        indicator_grid = QGridLayout()
        indicator_group.setLayout(indicator_grid)

        self.connection_indicator = QLabel("Disconnected")
        self.link_indicator = QLabel("No Link")
        self.motion_indicator = QLabel("Unknown")
        self.tilt_indicator = QLabel("No Alarm")
        self.filter_indicator = QLabel("IMU Angles")

        indicator_grid.addWidget(QLabel("Connection"), 0, 0)
        indicator_grid.addWidget(self.connection_indicator, 0, 1)
        indicator_grid.addWidget(QLabel("Serial Link"), 1, 0)
        indicator_grid.addWidget(self.link_indicator, 1, 1)
        indicator_grid.addWidget(QLabel("Motion"), 2, 0)
        indicator_grid.addWidget(self.motion_indicator, 2, 1)
        indicator_grid.addWidget(QLabel("Tilt Status"), 3, 0)
        indicator_grid.addWidget(self.tilt_indicator, 3, 1)
        indicator_grid.addWidget(QLabel("Orientation Mode"), 4, 0)
        indicator_grid.addWidget(self.filter_indicator, 4, 1)

        left_col.addWidget(indicator_group)

        console_group = QGroupBox("Console")
        console_layout = QVBoxLayout()
        console_group.setLayout(console_layout)

        self.console = QTextEdit()
        self.console.setReadOnly(True)
        console_layout.addWidget(self.console)

        left_col.addWidget(console_group)

        self.accel_plot = pg.PlotWidget(title="Accelerometer")
        self.accel_plot.setLabel("left", "Acceleration [g]")
        self.accel_plot.setLabel("bottom", "Sample")
        self.accel_plot.addLegend()

        self.gyro_plot = pg.PlotWidget(title="Gyroscope")
        self.gyro_plot.setLabel("left", "Angular Rate [dps]")
        self.gyro_plot.setLabel("bottom", "Sample")
        self.gyro_plot.addLegend()

        self.orientation_plot = pg.PlotWidget(title="Orientation")
        self.orientation_plot.setLabel("left", "Degrees")
        self.orientation_plot.setLabel("bottom", "Sample")
        self.orientation_plot.addLegend()

        self.ax_curve = self.accel_plot.plot(name="ax")
        self.ay_curve = self.accel_plot.plot(name="ay")
        self.az_curve = self.accel_plot.plot(name="az")

        self.gx_curve = self.gyro_plot.plot(name="gx")
        self.gy_curve = self.gyro_plot.plot(name="gy")
        self.gz_curve = self.gyro_plot.plot(name="gz")

        self.roll_curve = self.orientation_plot.plot(name="displayed_roll")
        self.pitch_curve = self.orientation_plot.plot(name="displayed_pitch")
        self.accel_roll_curve = self.orientation_plot.plot(name="accel_roll")
        self.accel_pitch_curve = self.orientation_plot.plot(name="accel_pitch")

        mid_col.addWidget(self.accel_plot)
        mid_col.addWidget(self.gyro_plot)
        mid_col.addWidget(self.orientation_plot)

        controls_group = QGroupBox("Controls")
        controls_layout = QVBoxLayout()
        controls_group.setLayout(controls_layout)

        connection_group = QGroupBox("Serial Link Controls")
        connection_grid = QGridLayout()
        connection_group.setLayout(connection_grid)

        self.ping_btn = QPushButton("Ping STM32")
        self.reset_link_btn = QPushButton("Reset Link Status")

        connection_grid.addWidget(self.ping_btn, 0, 0)
        connection_grid.addWidget(self.reset_link_btn, 0, 1)

        controls_layout.addWidget(connection_group)

        calibration_group = QGroupBox("Calibration")
        calibration_grid = QGridLayout()
        calibration_group.setLayout(calibration_grid)

        self.calib_accel_btn = QPushButton("Calibrate Accel")
        self.calib_gyro_btn = QPushButton("Calibrate Gyro")
        self.reset_orientation_btn = QPushButton("Reset Orientation")
        self.clear_plots_btn = QPushButton("Clear Plots")

        calibration_grid.addWidget(self.calib_accel_btn, 0, 0)
        calibration_grid.addWidget(self.calib_gyro_btn, 0, 1)
        calibration_grid.addWidget(self.reset_orientation_btn, 1, 0)
        calibration_grid.addWidget(self.clear_plots_btn, 1, 1)

        controls_layout.addWidget(calibration_group)

        filter_group = QGroupBox("Filter / Visualization")
        filter_grid = QGridLayout()
        filter_group.setLayout(filter_grid)

        self.comp_filter_checkbox = QCheckBox("Enable Complementary Filter")
        self.comp_filter_checkbox.setChecked(False)

        self.alpha_slider = QSlider(Qt.Horizontal)
        self.alpha_slider.setMinimum(90)
        self.alpha_slider.setMaximum(99)
        self.alpha_slider.setValue(98)
        self.alpha_label = QLabel("Alpha: 0.98")

        self.show_raw_checkbox = QCheckBox("Show Accel Orientation")
        self.show_raw_checkbox.setChecked(True)

        self.show_filtered_checkbox = QCheckBox("Show Displayed Orientation")
        self.show_filtered_checkbox.setChecked(True)

        self.show_accel_plot_checkbox = QCheckBox("Show Accel Plot")
        self.show_accel_plot_checkbox.setChecked(True)

        self.show_gyro_plot_checkbox = QCheckBox("Show Gyro Plot")
        self.show_gyro_plot_checkbox.setChecked(True)

        self.show_orientation_plot_checkbox = QCheckBox("Show Orientation Plot")
        self.show_orientation_plot_checkbox.setChecked(True)

        filter_grid.addWidget(self.comp_filter_checkbox, 0, 0, 1, 2)
        filter_grid.addWidget(self.alpha_label, 1, 0)
        filter_grid.addWidget(self.alpha_slider, 1, 1)
        filter_grid.addWidget(self.show_raw_checkbox, 2, 0, 1, 2)
        filter_grid.addWidget(self.show_filtered_checkbox, 3, 0, 1, 2)
        filter_grid.addWidget(self.show_accel_plot_checkbox, 4, 0, 1, 2)
        filter_grid.addWidget(self.show_gyro_plot_checkbox, 5, 0, 1, 2)
        filter_grid.addWidget(self.show_orientation_plot_checkbox, 6, 0, 1, 2)

        controls_layout.addWidget(filter_group)

        alarm_group = QGroupBox("Tilt / Motion Alarm")
        alarm_grid = QGridLayout()
        alarm_group.setLayout(alarm_grid)

        self.tilt_alarm_checkbox = QCheckBox("Enable Tilt Alarm")
        self.tilt_alarm_checkbox.setChecked(True)

        self.roll_thresh = QDoubleSpinBox()
        self.roll_thresh.setRange(0.0, 180.0)
        self.roll_thresh.setValue(30.0)
        self.roll_thresh.setSuffix(" deg")

        self.pitch_thresh = QDoubleSpinBox()
        self.pitch_thresh.setRange(0.0, 180.0)
        self.pitch_thresh.setValue(30.0)
        self.pitch_thresh.setSuffix(" deg")

        self.motion_thresh = QDoubleSpinBox()
        self.motion_thresh.setRange(0.0, 1000.0)
        self.motion_thresh.setValue(2.0)
        self.motion_thresh.setDecimals(3)

        self.link_timeout = QDoubleSpinBox()
        self.link_timeout.setRange(0.1, 10.0)
        self.link_timeout.setValue(1.0)
        self.link_timeout.setDecimals(2)
        self.link_timeout.setSuffix(" s")

        alarm_grid.addWidget(self.tilt_alarm_checkbox, 0, 0, 1, 2)
        alarm_grid.addWidget(QLabel("Roll Limit"), 1, 0)
        alarm_grid.addWidget(self.roll_thresh, 1, 1)
        alarm_grid.addWidget(QLabel("Pitch Limit"), 2, 0)
        alarm_grid.addWidget(self.pitch_thresh, 2, 1)
        alarm_grid.addWidget(QLabel("Motion Threshold"), 3, 0)
        alarm_grid.addWidget(self.motion_thresh, 3, 1)
        alarm_grid.addWidget(QLabel("Link Timeout"), 4, 0)
        alarm_grid.addWidget(self.link_timeout, 4, 1)

        controls_layout.addWidget(alarm_group)

        robot_group = QGroupBox("Robot Commands")
        robot_layout = QVBoxLayout()
        robot_group.setLayout(robot_layout)

        self.estop_btn = QPushButton("EMERGENCY STOP")
        self.estop_btn.setStyleSheet(
            "font-size: 18px; font-weight: bold; background-color: red; color: white; padding: 14px;"
        )

        self.send_start_btn = QPushButton("Send START")
        self.send_cal_btn = QPushButton("Send CALIBRATE")

        robot_layout.addWidget(self.estop_btn)
        robot_layout.addWidget(self.send_start_btn)
        robot_layout.addWidget(self.send_cal_btn)

        controls_layout.addWidget(robot_group)
        controls_layout.addStretch()

        right_col.addWidget(controls_group)

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

        self.alpha_slider.valueChanged.connect(self.update_alpha_label)
        self.comp_filter_checkbox.stateChanged.connect(self.on_filter_mode_changed)

        self.show_accel_plot_checkbox.stateChanged.connect(self.update_plot_visibility)
        self.show_gyro_plot_checkbox.stateChanged.connect(self.update_plot_visibility)
        self.show_orientation_plot_checkbox.stateChanged.connect(self.update_plot_visibility)
        self.show_raw_checkbox.stateChanged.connect(self.update_orientation_curve_visibility)
        self.show_filtered_checkbox.stateChanged.connect(self.update_orientation_curve_visibility)

        self.estop_btn.clicked.connect(self.emergency_stop)
        self.send_start_btn.clicked.connect(lambda: self.send_serial_command("START\n"))
        self.send_cal_btn.clicked.connect(lambda: self.send_serial_command("CALIBRATE\n"))
        self.ping_btn.clicked.connect(lambda: self.send_serial_command("PING\n"))
        self.reset_link_btn.clicked.connect(self.reset_link_status)

    def append_console(self, msg):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.console.append(f"[{timestamp}] {msg}")

    def refresh_ports(self):
        current = self.get_selected_port()
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()

        for p in ports:
            label = p.device
            if p.description:
                label = f"{p.device} - {p.description}"
            self.port_combo.addItem(label, p.device)

        if current:
            for i in range(self.port_combo.count()):
                if self.port_combo.itemData(i) == current:
                    self.port_combo.setCurrentIndex(i)
                    break

        self.append_console("Ports refreshed")

    def get_selected_port(self):
        data = self.port_combo.currentData()
        if data:
            return data
        text = self.port_combo.currentText().strip()
        if " - " in text:
            return text.split(" - ")[0].strip()
        return text

    def connect_serial(self):
        if self.worker is not None and self.worker.isRunning():
            self.append_console("Already connected")
            return

        port = self.get_selected_port()
        if not port:
            QMessageBox.warning(self, "No Port", "No serial / COM port selected")
            return

        baud = int(self.baud_combo.currentText())
        self.last_good_port = port
        self.last_good_baud = self.baud_combo.currentText()

        self.worker = SerialWorker(port, baud)
        self.worker.data_received.connect(self.handle_data)
        self.worker.status_message.connect(self.append_console)
        self.worker.connection_changed.connect(self.handle_connection_change)
        self.worker.start()

    def disconnect_serial(self):
        if self.worker is not None:
            self.worker.stop()
            self.worker.wait()
            self.worker = None

    def reconnect_serial(self):
        if not self.last_good_port:
            self.append_console("No previous serial port to reconnect")
            return

        self.disconnect_serial()
        self.refresh_ports()

        for i in range(self.port_combo.count()):
            if self.port_combo.itemData(i) == self.last_good_port:
                self.port_combo.setCurrentIndex(i)
                break

        self.baud_combo.setCurrentText(self.last_good_baud)
        self.connect_serial()

    def handle_connection_change(self, connected):
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
        self.accel_plot.setVisible(self.show_accel_plot_checkbox.isChecked())
        self.gyro_plot.setVisible(self.show_gyro_plot_checkbox.isChecked())
        self.orientation_plot.setVisible(self.show_orientation_plot_checkbox.isChecked())

    def update_orientation_curve_visibility(self):
        show_raw = self.show_raw_checkbox.isChecked()
        show_filtered = self.show_filtered_checkbox.isChecked()

        self.accel_roll_curve.setVisible(show_raw)
        self.accel_pitch_curve.setVisible(show_raw)
        self.roll_curve.setVisible(show_filtered)
        self.pitch_curve.setVisible(show_filtered)

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
        self.update_plots()

        if hasattr(self, "dashboard_tab") and self.dashboard_tab is not None:
            dashboard_packet = dict(d)
            dashboard_packet["roll"] = roll
            dashboard_packet["pitch"] = pitch
            self.dashboard_tab.update_dashboard(dashboard_packet, roll, pitch, sample_rate)

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
        x = list(self.t)

        self.ax_curve.setData(x, list(self.ax_data))
        self.ay_curve.setData(x, list(self.ay_data))
        self.az_curve.setData(x, list(self.az_data))

        self.gx_curve.setData(x, list(self.gx_data))
        self.gy_curve.setData(x, list(self.gy_data))
        self.gz_curve.setData(x, list(self.gz_data))

        self.roll_curve.setData(x, list(self.roll_data))
        self.pitch_curve.setData(x, list(self.pitch_data))

        self.accel_roll_curve.setData(x, list(self.accel_roll_data))
        self.accel_pitch_curve.setData(x, list(self.accel_pitch_data))

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
    pg.setConfigOptions(antialias=True)
    window = ImuGui()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()