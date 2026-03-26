#!/usr/bin/env python3
"""Live car dashboard UI for STM32 IMU telemetry.

Reads UART lines produced by Core/Src/main.c:
- ACC[g] X:... Y:... Z:... | GYRO[dps] X:... Y:... Z:...
- ANGLE roll:... deg | pitch:... deg

If serial is unavailable, runs in demo mode.
"""

from __future__ import annotations

import argparse
import math
import queue
import re
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import ttk

try:
    import serial  # type: ignore
except Exception:
    serial = None


ACC_GYRO_RE = re.compile(
    r"ACC\[g\]\s+X:(?P<ax>-?\d+(?:\.\d+)?)\s+Y:(?P<ay>-?\d+(?:\.\d+)?)\s+Z:(?P<az>-?\d+(?:\.\d+)?)"
    r"\s*\|\s*GYRO\[dps\]\s+X:(?P<gx>-?\d+(?:\.\d+)?)\s+Y:(?P<gy>-?\d+(?:\.\d+)?)\s+Z:(?P<gz>-?\d+(?:\.\d+)?)"
)
ANGLE_RE = re.compile(r"ANGLE\s+roll:(?P<roll>-?\d+(?:\.\d+)?)\s+deg\s*\|\s*pitch:(?P<pitch>-?\d+(?:\.\d+)?)")


@dataclass
class Telemetry:
    ax: float = 0.0
    ay: float = 0.0
    az: float = 1.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0


class SerialReader(threading.Thread):
    def __init__(self, port: str, baud: int, out_q: queue.Queue[str]) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out_q = out_q
        self._running = True

    def run(self) -> None:
        if serial is None:
            return
        try:
            with serial.Serial(self.port, self.baud, timeout=0.25) as ser:
                while self._running:
                    line = ser.readline().decode(errors="ignore").strip()
                    if line:
                        self.out_q.put(line)
        except Exception as e:
            self.out_q.put(f"__ERR__:{e}")

    def stop(self) -> None:
        self._running = False


class Dashboard:
    def __init__(self, root: tk.Tk, port: str | None, baud: int) -> None:
        self.root = root
        self.root.title("La Voiture Live Dashboard")
        self.root.geometry("1260x780")
        self.root.configure(bg="#08111f")

        self.telemetry = Telemetry()
        self.speed_est = 0.0
        self.queue: queue.Queue[str] = queue.Queue()
        self.reader: SerialReader | None = None
        self.demo_mode = port is None
        self.err_text = ""
        self.last_t = time.time()
        self.road_phase = 0.0

        if port is not None and serial is not None:
            self.reader = SerialReader(port, baud, self.queue)
            self.reader.start()
        else:
            self.demo_mode = True

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.tick()

    def _build_ui(self) -> None:
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#08111f")
        style.configure("Panel.TFrame", background="#101c2f")
        style.configure("TLabel", background="#08111f", foreground="#d7e7ff", font=("Avenir Next", 11))
        style.configure("Head.TLabel", background="#08111f", foreground="#f8fbff", font=("Avenir Next", 22, "bold"))

        top = ttk.Frame(self.root)
        top.pack(fill=tk.X, padx=14, pady=(12, 8))
        ttk.Label(top, text="LA VOITURE // IMU DASHBOARD", style="Head.TLabel").pack(side=tk.LEFT)

        self.mode_var = tk.StringVar(value="MODE: DEMO" if self.demo_mode else "MODE: LIVE")
        ttk.Label(top, textvariable=self.mode_var).pack(side=tk.RIGHT)

        content = ttk.Frame(self.root)
        content.pack(fill=tk.BOTH, expand=True, padx=14, pady=10)

        left = ttk.Frame(content, style="Panel.TFrame")
        center = ttk.Frame(content, style="Panel.TFrame")
        right = ttk.Frame(content, style="Panel.TFrame")
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        center.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        right.pack(side=tk.LEFT, fill=tk.Y)

        self.gauge_canvas = tk.Canvas(left, width=320, height=680, bg="#101c2f", highlightthickness=0)
        self.gauge_canvas.pack(fill=tk.BOTH, expand=True)

        self.road_canvas = tk.Canvas(center, width=600, height=680, bg="#0b1628", highlightthickness=0)
        self.road_canvas.pack(fill=tk.BOTH, expand=True)

        self.att_canvas = tk.Canvas(right, width=320, height=680, bg="#101c2f", highlightthickness=0)
        self.att_canvas.pack(fill=tk.BOTH, expand=True)

        self.raw_var = tk.StringVar(value="Waiting for telemetry...")
        ttk.Label(self.root, textvariable=self.raw_var, anchor="w").pack(fill=tk.X, padx=16, pady=(2, 10))

    def parse_line(self, line: str) -> None:
        m = ACC_GYRO_RE.search(line)
        if m:
            self.telemetry.ax = float(m.group("ax"))
            self.telemetry.ay = float(m.group("ay"))
            self.telemetry.az = float(m.group("az"))
            self.telemetry.gx = float(m.group("gx"))
            self.telemetry.gy = float(m.group("gy"))
            self.telemetry.gz = float(m.group("gz"))
            return

        m = ANGLE_RE.search(line)
        if m:
            self.telemetry.roll = float(m.group("roll"))
            self.telemetry.pitch = float(m.group("pitch"))

    def update_demo(self, dt: float) -> None:
        t = time.time()
        self.telemetry.ax = 0.05 * math.sin(t * 1.5)
        self.telemetry.ay = 0.16 * math.sin(t * 0.9)
        self.telemetry.az = 1.0 + 0.04 * math.sin(t * 1.2)
        self.telemetry.gz = 35.0 * math.sin(t * 0.7)
        self.telemetry.gx = 5.0 * math.sin(t * 1.4)
        self.telemetry.gy = 4.0 * math.cos(t * 1.1)
        self.telemetry.roll = 10.0 * math.sin(t * 0.8)
        self.telemetry.pitch = 6.0 * math.sin(t * 1.1)

    def draw_gauges(self) -> None:
        c = self.gauge_canvas
        c.delete("all")
        c.create_text(20, 20, text="Performance", fill="#d7e7ff", anchor="nw", font=("Avenir Next", 16, "bold"))

        dt = 0.05
        self.speed_est = max(0.0, self.speed_est + (self.telemetry.ax * 9.81) * dt)
        speed_cap = 6.0
        yaw_cap = 240.0

        self._draw_needle_dial(
            c, 160, 205, 112, min(self.speed_est, speed_cap), 0.0, speed_cap,
            "Speed", f"{self.speed_est:.2f} m/s", "#22d3ee"
        )
        self._draw_needle_dial(
            c, 160, 450, 112, self.telemetry.gz, -yaw_cap, yaw_cap,
            "Angular Velocity", f"{self.telemetry.gz:+.1f} dps", "#fb7185"
        )

        c.create_text(160, 635, text=f"GYRO X:{self.telemetry.gx:+.1f}  Y:{self.telemetry.gy:+.1f}",
                      fill="#fda4af", font=("Avenir Next", 12, "bold"))

    def _draw_needle_dial(
        self,
        c: tk.Canvas,
        cx: int,
        cy: int,
        r: int,
        value: float,
        vmin: float,
        vmax: float,
        label: str,
        value_text: str,
        color: str,
    ) -> None:
        c.create_oval(cx - r, cy - r, cx + r, cy + r, fill="#0f1b2e", outline="#2b3c5a", width=2)
        c.create_arc(cx - r, cy - r, cx + r, cy + r, start=210, extent=120, style="arc", outline="#2d415f", width=10)
        c.create_text(cx, cy - 18, text=label, fill="#93c5fd", font=("Avenir Next", 11, "bold"))
        c.create_text(cx, cy + 16, text=value_text, fill="#f8fbff", font=("Avenir Next", 14, "bold"))

        for i in range(7):
            frac = i / 6.0
            ang = math.radians(210 - frac * 240.0)
            x0 = cx + math.cos(ang) * (r - 16)
            y0 = cy - math.sin(ang) * (r - 16)
            x1 = cx + math.cos(ang) * (r - 4)
            y1 = cy - math.sin(ang) * (r - 4)
            c.create_line(x0, y0, x1, y1, fill="#7a8fad", width=2)

        if vmax <= vmin:
            frac = 0.0
        else:
            frac = (value - vmin) / (vmax - vmin)
            frac = max(0.0, min(1.0, frac))
        ang = math.radians(210 - frac * 240.0)
        xn = cx + math.cos(ang) * (r - 24)
        yn = cy - math.sin(ang) * (r - 24)
        c.create_line(cx, cy, xn, yn, fill=color, width=4)
        c.create_oval(cx - 6, cy - 6, cx + 6, cy + 6, fill="#e2e8f0", outline="")

    def draw_road(self) -> None:
        c = self.road_canvas
        c.delete("all")
        w = int(c.winfo_width())
        h = int(c.winfo_height())
        c.create_rectangle(0, 0, w, h, fill="#0a1321", outline="")

        road_w = int(w * 0.58)
        road_x0 = (w - road_w) // 2
        road_x1 = road_x0 + road_w
        c.create_polygon(road_x0 + 70, 0, road_x1 - 70, 0, road_x1, h, road_x0, h, fill="#17263a", outline="")

        self.road_phase = (self.road_phase + max(4.0, self.speed_est * 14.0)) % 80.0
        y = -80.0 + self.road_phase
        while y < h + 80:
            c.create_rectangle(w // 2 - 6, int(y), w // 2 + 6, int(y + 46), fill="#f8fafc", outline="")
            y += 80

        # Car movement from lateral accel and yaw rate
        offset = int(max(-90, min(90, self.telemetry.ay * 260 + self.telemetry.gz * 0.6)))
        car_cx = w // 2 + offset
        car_cy = int(h * 0.74)

        glow_color = "#34d399" if abs(self.telemetry.gz) < 40 else "#f59e0b"
        c.create_oval(car_cx - 78, car_cy - 45, car_cx + 78, car_cy + 45, fill="#000000", outline="", stipple="gray50")
        c.create_rectangle(car_cx - 58, car_cy - 25, car_cx + 58, car_cy + 25, fill="#cbd5e1", outline="#94a3b8", width=2)
        c.create_rectangle(car_cx - 32, car_cy - 18, car_cx + 32, car_cy + 18, fill="#1e293b", outline="#475569")
        c.create_oval(car_cx - 66, car_cy - 30, car_cx - 52, car_cy - 16, fill=glow_color, outline="")
        c.create_oval(car_cx + 52, car_cy - 30, car_cx + 66, car_cy - 16, fill=glow_color, outline="")

        c.create_text(16, 16, anchor="nw", fill="#d7e7ff", font=("Avenir Next", 14, "bold"), text="Track View")

    def draw_attitude(self) -> None:
        c = self.att_canvas
        c.delete("all")
        c.create_text(20, 20, text="Attitude", fill="#d7e7ff", anchor="nw", font=("Avenir Next", 16, "bold"))

        cx, cy, r = 160, 220, 120
        pitch_px = max(-70, min(70, self.telemetry.pitch * 2.0))
        roll = self.telemetry.roll

        for i in range(-5, 6):
            y = cy + pitch_px + i * 22
            c.create_line(cx - 100, y, cx + 100, y, fill="#1f2e45")

        # Horizon line (roll visual)
        rad = math.radians(roll)
        dx = int(math.cos(rad) * 90)
        dy = int(math.sin(rad) * 90)
        c.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#2d405f", width=3)
        c.create_line(cx - dx, cy + dy + pitch_px, cx + dx, cy - dy + pitch_px, fill="#38bdf8", width=3)
        c.create_line(cx - 26, cy, cx + 26, cy, fill="#f8fbff", width=3)
        c.create_line(cx, cy - 8, cx, cy + 8, fill="#f8fbff", width=2)

        c.create_text(160, 390, text=f"Roll {self.telemetry.roll:+.1f} deg", fill="#cbd5e1", font=("Avenir Next", 13, "bold"))
        c.create_text(160, 420, text=f"Pitch {self.telemetry.pitch:+.1f} deg", fill="#cbd5e1", font=("Avenir Next", 13, "bold"))

        # Live values
        c.create_text(30, 500, anchor="w", text=f"AX: {self.telemetry.ax:+.3f} g", fill="#7dd3fc", font=("Avenir Next", 12))
        c.create_text(30, 530, anchor="w", text=f"AY: {self.telemetry.ay:+.3f} g", fill="#7dd3fc", font=("Avenir Next", 12))
        c.create_text(30, 560, anchor="w", text=f"AZ: {self.telemetry.az:+.3f} g", fill="#7dd3fc", font=("Avenir Next", 12))
        c.create_text(30, 600, anchor="w", text=f"GX: {self.telemetry.gx:+.2f} dps", fill="#fda4af", font=("Avenir Next", 12))
        c.create_text(30, 630, anchor="w", text=f"GY: {self.telemetry.gy:+.2f} dps", fill="#fda4af", font=("Avenir Next", 12))
        c.create_text(30, 660, anchor="w", text=f"GZ: {self.telemetry.gz:+.2f} dps", fill="#fda4af", font=("Avenir Next", 12))

    def tick(self) -> None:
        now = time.time()
        dt = now - self.last_t
        self.last_t = now

        while not self.queue.empty():
            line = self.queue.get_nowait()
            if line.startswith("__ERR__:"):
                self.err_text = line
                self.mode_var.set("MODE: ERROR (fallback to demo)")
                self.demo_mode = True
            else:
                self.raw_var.set(line)
                self.parse_line(line)

        if self.demo_mode:
            self.update_demo(dt)
            if not self.err_text:
                self.raw_var.set("Demo telemetry active. Pass --port to use STM32 UART.")

        self.draw_gauges()
        self.draw_road()
        self.draw_attitude()

        self.root.after(33, self.tick)

    def on_close(self) -> None:
        if self.reader is not None:
            self.reader.stop()
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description="La Voiture IMU Dashboard")
    parser.add_argument("--port", default=None, help="Serial port (e.g. COM21 or /dev/cu.usbserial-XXXX)")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate")
    args = parser.parse_args()

    root = tk.Tk()
    Dashboard(root, args.port, args.baud)
    root.mainloop()


if __name__ == "__main__":
    main()
