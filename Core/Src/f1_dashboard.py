import math
import time

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QGroupBox, QProgressBar, QFrame
)


class DashboardBar(QProgressBar):
    def __init__(self, minimum=0, maximum=100, fmt="%v"):
        super().__init__()
        self.setRange(minimum, maximum)
        self.setValue(minimum)
        self.setFormat(fmt)
        self.setTextVisible(True)
        self.setMinimumHeight(28)
        self.setStyleSheet("""
            QProgressBar {
                border: 2px solid #222;
                border-radius: 8px;
                background-color: #111;
                color: white;
                text-align: center;
                font-weight: bold;
            }
            QProgressBar::chunk {
                background-color: #00c853;
                border-radius: 6px;
            }
        """)


class ValueCard(QFrame):
    def __init__(self, title, value="0", unit=""):
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet("""
            QFrame {
                background-color: #121212;
                border: 2px solid #2a2a2a;
                border-radius: 12px;
            }
            QLabel {
                color: white;
            }
        """)

        layout = QVBoxLayout()
        self.setLayout(layout)

        self.title_label = QLabel(title)
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setStyleSheet("font-size: 14px; color: #bbbbbb;")

        self.value_label = QLabel(str(value))
        self.value_label.setAlignment(Qt.AlignCenter)
        self.value_label.setStyleSheet("font-size: 28px; font-weight: bold; color: #00e5ff;")

        self.unit_label = QLabel(unit)
        self.unit_label.setAlignment(Qt.AlignCenter)
        self.unit_label.setStyleSheet("font-size: 13px; color: #aaaaaa;")

        layout.addWidget(self.title_label)
        layout.addWidget(self.value_label)
        layout.addWidget(self.unit_label)

    def set_value(self, value_text):
        self.value_label.setText(str(value_text))


class F1DashboardTab(QWidget):
    def __init__(self):
        super().__init__()

        self.last_update_time = time.time()

        self.speed_kph = 0.0
        self.speed_ms = 0.0
        self.rpm = 0
        self.gear = "N"
        self.throttle_pct = 0
        self.brake_pct = 0
        self.steering_deg = 0.0
        self.lap_time_s = 0.0
        self.sector = 1
        self.battery_pct = 100
        self.g_force = 1.0
        self.yaw_rate = 0.0
        self.pitch_deg = 0.0
        self.roll_deg = 0.0

        self.ax_f = 0.0
        self.ay_f = 0.0
        self.az_f = 0.0
        self.gz_f = 0.0
        self.long_g = 0.0
        self.lat_g = 0.0
        self.vertical_g = 1.0

        self.stationary_timer = 0.0
        self.last_shift_time = 0.0

        self.init_ui()

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.animate_when_idle)
        self.ui_timer.start(100)

    def init_ui(self):
        self.setStyleSheet("""
            QWidget {
                background-color: #0a0a0a;
            }
            QGroupBox {
                color: white;
                font-size: 16px;
                font-weight: bold;
                border: 2px solid #333;
                border-radius: 12px;
                margin-top: 10px;
                padding-top: 12px;
                background-color: #101010;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px 0 6px;
            }
            QLabel {
                color: white;
            }
        """)

        root = QVBoxLayout()
        self.setLayout(root)

        title = QLabel("F1 CAR DASHBOARD")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 28px; font-weight: bold; color: #ff3d00; padding: 10px;")
        root.addWidget(title)

        top_row = QHBoxLayout()
        root.addLayout(top_row)

        self.speed_card = ValueCard("SPEED", "0", "km/h")
        self.rpm_card = ValueCard("RPM EST", "0", "rev/min")
        self.gear_card = ValueCard("GEAR", "N", "")
        self.gforce_card = ValueCard("G-FORCE", "1.00", "g")
        self.battery_card = ValueCard("BATTERY", "100", "%")

        top_row.addWidget(self.speed_card)
        top_row.addWidget(self.rpm_card)
        top_row.addWidget(self.gear_card)
        top_row.addWidget(self.gforce_card)
        top_row.addWidget(self.battery_card)

        middle_row = QHBoxLayout()
        root.addLayout(middle_row)

        control_group = QGroupBox("Driver Inputs (Estimated)")
        control_layout = QVBoxLayout()
        control_group.setLayout(control_layout)

        self.throttle_bar = DashboardBar(0, 100, "Throttle %p%")
        self.brake_bar = DashboardBar(0, 100, "Brake %p%")

        self.steering_label = QLabel("Steering: 0.0 deg")
        self.steering_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #ffd54f;")

        control_layout.addWidget(self.throttle_bar)
        control_layout.addWidget(self.brake_bar)
        control_layout.addWidget(self.steering_label)

        vehicle_group = QGroupBox("Vehicle Dynamics")
        vehicle_layout = QGridLayout()
        vehicle_group.setLayout(vehicle_layout)

        self.roll_label = QLabel("Roll: 0.0 deg")
        self.pitch_label = QLabel("Pitch: 0.0 deg")
        self.yaw_label = QLabel("Yaw Rate: 0.0 deg/s")
        self.status_label = QLabel("Status: READY")

        for lbl in [self.roll_label, self.pitch_label, self.yaw_label, self.status_label]:
            lbl.setStyleSheet("font-size: 18px; font-weight: bold;")

        vehicle_layout.addWidget(self.roll_label, 0, 0)
        vehicle_layout.addWidget(self.pitch_label, 0, 1)
        vehicle_layout.addWidget(self.yaw_label, 1, 0)
        vehicle_layout.addWidget(self.status_label, 1, 1)

        middle_row.addWidget(control_group, 2)
        middle_row.addWidget(vehicle_group, 2)

        bottom_row = QHBoxLayout()
        root.addLayout(bottom_row)

        race_group = QGroupBox("Race Info")
        race_layout = QGridLayout()
        race_group.setLayout(race_layout)

        self.lap_label = QLabel("Lap Time: 00:00.000")
        self.sector_label = QLabel("Sector: 1")
        self.mode_label = QLabel("Mode: TRACK")
        self.temp_label = QLabel("IMU Load: 0 C")
        self.traction_label = QLabel("Traction: NORMAL")

        for lbl in [self.lap_label, self.sector_label, self.mode_label, self.temp_label, self.traction_label]:
            lbl.setStyleSheet("font-size: 18px; font-weight: bold;")

        race_layout.addWidget(self.lap_label, 0, 0)
        race_layout.addWidget(self.sector_label, 0, 1)
        race_layout.addWidget(self.mode_label, 1, 0)
        race_layout.addWidget(self.temp_label, 1, 1)
        race_layout.addWidget(self.traction_label, 2, 0)

        alert_group = QGroupBox("Alerts")
        alert_layout = QVBoxLayout()
        alert_group.setLayout(alert_layout)

        self.alert_label = QLabel("No active alerts")
        self.alert_label.setAlignment(Qt.AlignCenter)
        self.alert_label.setStyleSheet("""
            font-size: 24px;
            font-weight: bold;
            color: #00e676;
            padding: 18px;
            border: 2px solid #1f1f1f;
            border-radius: 10px;
            background-color: #111;
        """)

        alert_layout.addWidget(self.alert_label)

        bottom_row.addWidget(race_group, 2)
        bottom_row.addWidget(alert_group, 2)

    def animate_when_idle(self):
        pass

    def clamp(self, value, low, high):
        return max(low, min(value, high))

    def lowpass(self, prev, new, alpha):
        return alpha * new + (1.0 - alpha) * prev

    def infer_gear_from_speed(self, speed_kph):
        if speed_kph < 5:
            return 1
        if speed_kph < 45:
            return 1
        if speed_kph < 80:
            return 2
        if speed_kph < 115:
            return 3
        if speed_kph < 155:
            return 4
        if speed_kph < 200:
            return 5
        if speed_kph < 245:
            return 6
        if speed_kph < 295:
            return 7
        return 8

    def gear_ratio_for(self, gear_num):
        ratios = {
            1: 3.20,
            2: 2.45,
            3: 1.90,
            4: 1.55,
            5: 1.32,
            6: 1.10,
            7: 0.92,
            8: 0.78,
        }
        return ratios.get(gear_num, 3.20)

    def update_dashboard(self, imu_data, roll, pitch, sample_rate=0.0):
        now = time.time()
        dt = now - self.last_update_time
        if dt <= 0.0 or dt > 0.25:
            dt = 0.02
        self.last_update_time = now

        ax = float(imu_data.get("ax", 0.0))
        ay = float(imu_data.get("ay", 0.0))
        az = float(imu_data.get("az", 1.0))
        gx = float(imu_data.get("gx", 0.0))
        gy = float(imu_data.get("gy", 0.0))
        gz = float(imu_data.get("gz", 0.0))

        alpha_acc = 0.18
        alpha_gyro = 0.22

        self.ax_f = self.lowpass(self.ax_f, ax, alpha_acc)
        self.ay_f = self.lowpass(self.ay_f, ay, alpha_acc)
        self.az_f = self.lowpass(self.az_f, az, alpha_acc)
        self.gz_f = self.lowpass(self.gz_f, gz, alpha_gyro)

        self.roll_deg = roll
        self.pitch_deg = pitch
        self.yaw_rate = self.gz_f

        roll_rad = math.radians(self.roll_deg)
        pitch_rad = math.radians(self.pitch_deg)

        g_long_gravity = -math.sin(pitch_rad)
        g_lat_gravity = math.sin(roll_rad)
        g_vert_gravity = math.cos(roll_rad) * math.cos(pitch_rad)

        long_g = self.ax_f - g_long_gravity
        lat_g = self.ay_f - g_lat_gravity
        vert_g = self.az_f - g_vert_gravity

        self.long_g = long_g
        self.lat_g = lat_g
        self.vertical_g = vert_g

        acc_mag = math.sqrt(self.ax_f * self.ax_f + self.ay_f * self.ay_f + self.az_f * self.az_f)
        self.g_force = acc_mag

        long_acc_ms2 = long_g * 9.81

        deadband_g = 0.025
        if abs(long_g) < deadband_g:
            long_acc_ms2 = 0.0

        aero_drag = 0.010 * self.speed_ms * self.speed_ms
        rolling_drag = 0.18 if self.speed_ms > 0.2 else 0.0

        if long_acc_ms2 >= 0.0:
            net_acc = long_acc_ms2 - aero_drag - rolling_drag
        else:
            net_acc = long_acc_ms2 + aero_drag + rolling_drag

        self.speed_ms += net_acc * dt
        self.speed_ms = max(self.speed_ms, 0.0)

        gyro_activity = math.sqrt(gx * gx + gy * gy + gz * gz)
        nearly_stationary = (
            abs(long_g) < 0.02 and
            abs(lat_g) < 0.03 and
            abs(vert_g) < 0.04 and
            gyro_activity < 3.0
        )

        if nearly_stationary:
            self.stationary_timer += dt
        else:
            self.stationary_timer = 0.0

        if self.stationary_timer > 0.6:
            self.speed_ms *= 0.85
            if self.speed_ms < 0.25:
                self.speed_ms = 0.0

        self.speed_kph = self.speed_ms * 3.6

        if self.speed_kph < 1.0 and abs(long_g) < 0.03:
            throttle_est = 0
            brake_est = 0
        else:
            throttle_est = int(self.clamp((long_g / 0.75) * 100.0, 0.0, 100.0))
            brake_est = int(self.clamp((-long_g / 1.10) * 100.0, 0.0, 100.0))

        steering_est = self.clamp(self.yaw_rate * 4.0, -180.0, 180.0)

        desired_gear = self.infer_gear_from_speed(self.speed_kph)
        current_gear_num = 1 if self.gear == "N" else int(self.gear)

        if now - self.last_shift_time > 0.25:
            if desired_gear > current_gear_num:
                current_gear_num += 1
                self.last_shift_time = now
            elif desired_gear < current_gear_num:
                current_gear_num -= 1
                self.last_shift_time = now

        if self.speed_kph < 3.0 and throttle_est == 0:
            self.gear = "N"
            gear_num_for_rpm = 1
        else:
            self.gear = str(current_gear_num)
            gear_num_for_rpm = current_gear_num

        ratio = self.gear_ratio_for(gear_num_for_rpm)
        wheel_rpm_est = self.speed_ms * 60.0 / 2.1
        rpm_est = int(self.clamp(wheel_rpm_est * ratio * 3.7, 2500.0 if self.speed_kph > 3 else 0.0, 15000.0))

        if self.gear == "N":
            rpm_est = 0

        battery_drop_rate = 0.0025 * self.speed_kph + 0.015 * abs(long_g) * 100.0
        self.battery_pct -= battery_drop_rate * dt * 0.1
        self.battery_pct = int(self.clamp(self.battery_pct, 5, 100))

        temp_est = int(35 + 18 * abs(long_g) + 10 * abs(lat_g) + 0.15 * abs(self.yaw_rate))
        temp_est = int(self.clamp(temp_est, 30, 115))

        self.rpm = rpm_est
        self.throttle_pct = throttle_est
        self.brake_pct = brake_est
        self.steering_deg = steering_est

        self.speed_card.set_value(f"{self.speed_kph:.0f}")
        self.rpm_card.set_value(f"{self.rpm}")
        self.gear_card.set_value(self.gear)
        self.gforce_card.set_value(f"{self.g_force:.2f}")
        self.battery_card.set_value(f"{self.battery_pct}")

        self.throttle_bar.setValue(self.throttle_pct)
        self.brake_bar.setValue(self.brake_pct)
        self.steering_label.setText(f"Steering: {self.steering_deg:.1f} deg")

        self.roll_label.setText(f"Roll: {self.roll_deg:.1f} deg")
        self.pitch_label.setText(f"Pitch: {self.pitch_deg:.1f} deg")
        self.yaw_label.setText(f"Yaw Rate: {self.yaw_rate:.1f} deg/s")

        self.lap_time_s += dt
        mins = int(self.lap_time_s // 60)
        secs = int(self.lap_time_s % 60)
        millis = int((self.lap_time_s - int(self.lap_time_s)) * 1000)
        self.lap_label.setText(f"Lap Time: {mins:02d}:{secs:02d}.{millis:03d}")

        if self.speed_kph < 90:
            self.sector = 1
        elif self.speed_kph < 190:
            self.sector = 2
        else:
            self.sector = 3

        self.sector_label.setText(f"Sector: {self.sector}")
        self.temp_label.setText(f"IMU Load: {temp_est} C")

        if abs(self.roll_deg) > 30 or abs(self.pitch_deg) > 30:
            self.status_label.setText("Status: STABILITY LIMIT")
            self.status_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #ff5252;")
            self.alert_label.setText("HIGH TILT WARNING")
            self.alert_label.setStyleSheet("""
                font-size: 24px;
                font-weight: bold;
                color: white;
                padding: 18px;
                border: 2px solid #b71c1c;
                border-radius: 10px;
                background-color: #d50000;
            """)
        elif abs(self.lat_g) > 0.9:
            self.status_label.setText("Status: HIGH CORNER LOAD")
            self.status_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #ffd54f;")
            self.alert_label.setText("LATERAL LOAD HIGH")
            self.alert_label.setStyleSheet("""
                font-size: 24px;
                font-weight: bold;
                color: black;
                padding: 18px;
                border: 2px solid #f9a825;
                border-radius: 10px;
                background-color: #ffeb3b;
            """)
        elif self.battery_pct < 20:
            self.status_label.setText("Status: LOW ENERGY")
            self.status_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #ffd54f;")
            self.alert_label.setText("LOW BATTERY")
            self.alert_label.setStyleSheet("""
                font-size: 24px;
                font-weight: bold;
                color: black;
                padding: 18px;
                border: 2px solid #f9a825;
                border-radius: 10px;
                background-color: #ffeb3b;
            """)
        else:
            self.status_label.setText("Status: PUSHING")
            self.status_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #00e676;")
            self.alert_label.setText("SYSTEM NOMINAL")
            self.alert_label.setStyleSheet("""
                font-size: 24px;
                font-weight: bold;
                color: #00e676;
                padding: 18px;
                border: 2px solid #1f1f1f;
                border-radius: 10px;
                background-color: #111;
            """)

        if self.throttle_pct > 65 and self.brake_pct < 8:
            self.mode_label.setText("Mode: ATTACK")
            self.traction_label.setText("Traction: SPORT")
        elif self.brake_pct > 35:
            self.mode_label.setText("Mode: BRAKING")
            self.traction_label.setText("Traction: HIGH")
        elif abs(self.lat_g) > 0.65:
            self.mode_label.setText("Mode: CORNER")
            self.traction_label.setText("Traction: BALANCED")
        else:
            self.mode_label.setText("Mode: TRACK")
            self.traction_label.setText("Traction: NORMAL")