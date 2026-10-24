"""Obstacle Detection: drives straight, slows for obstacles in the distance, stops when close.

Run:  python app.py              (first working camera)
      python app.py --sim        (no camera: set a pretend obstacle distance in the GUI)
Keys: Enter = start, Space = stop
"""
import argparse
import csv
import os
import subprocess
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import ttk

import cv2
import numpy as np
from PIL import Image, ImageTk

from controller import Config, ObstacleController, State
from detector import ObstacleDetector, draw_overlay
from esp32_link import Esp32Link, Esp32Panel
from vehicle import Esp32Vehicle

VIDEO_W, VIDEO_H = 760, 570
CAMERA_LOST_AFTER = 1.0  # seconds without a frame before stopping for safety
SONAR_LOST_AFTER = 0.5   # seconds without an ultrasonic reading before stopping
GPS_LOST_AFTER = 3.0     # seconds without a GPS sentence before showing "no data"
MAX_CAMERA_INDEX = 5     # camera indexes tried when no --source is given
LAPTOP_LOCATION_EVERY = 30  # seconds between Windows location lookups (used until the GPS has a fix)
CSV_EVERY = 1.0          # seconds between rows written to the detection CSV
CSV_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "detections")

STATE_COLORS = {
    State.CRUISING: "#1a7f37",
    State.SLOWING: "#d4a017",
    State.BLOCKED: "#e07b00",
    State.STOPPED: "#c62828",
}

# (Config field, label, min, max, step, GUI value / config value)
SETTINGS = [
    ("slow_distance", "Slow down: camera obstacle within (m)", 0.5, 50, 0.5, 1),
    ("stop_distance", "Stop: camera obstacle within (m)", 0.2, 20, 0.1, 1),
    ("sonar_stop_cm", "Stop: ultrasonic under (cm)", 5, 400, 5, 1),
    ("slow_speed", "Slow speed limit (%)", 0, 100, 5, 100),
    ("cruise_speed", "Cruise speed (%)", 0, 100, 5, 100),
    ("clear_time", "Resume after clear for (s)", 0.1, 10, 0.1, 1),
    ("accel_rate", "Speed up rate (%/s)", 5, 200, 5, 100),
    ("decel_rate", "Slow down rate (%/s)", 5, 500, 5, 100),
    ("hard_stop_cm", "ESP32 buzzer + hard stop under (cm)", 0, 400, 1, 1),
    ("min_duty", "ESP32 motor min PWM (0-255)", 0, 255, 5, 1),
]
REVERSE_SETTINGS = [("reverse_a", "Reverse motor A"), ("reverse_b", "Reverse motor B")]
ESP32_SETTINGS_EVERY = 1.0  # s; resent so an ESP32 that rebooted gets them back


def open_camera(source):
    """Open the given source, or the first camera index that delivers a frame."""
    for s in [source] if source is not None else range(MAX_CAMERA_INDEX):
        cap = cv2.VideoCapture(s)
        if cap.isOpened() and cap.read()[0]:
            return cap, s
        cap.release()
    return None, None


# Windows location service (Wi-Fi / IP based). Needs Settings > Privacy > Location turned on.
LAPTOP_LOCATION_PS = """
Add-Type -AssemblyName System.Device
$w = New-Object System.Device.Location.GeoCoordinateWatcher
[void]$w.TryStart($false, [TimeSpan]::FromSeconds(10))
$end = (Get-Date).AddSeconds(15)
while ($w.Position.Location.IsUnknown -and $w.Permission -ne 'Denied' -and (Get-Date) -lt $end) {
    Start-Sleep -Milliseconds 200
}
$c = $w.Position.Location
if ($c.IsUnknown) { "none $($w.Permission)" } else {
    [string]::Format([Globalization.CultureInfo]::InvariantCulture, '{0},{1},{2}',
                     $c.Latitude, $c.Longitude, $c.HorizontalAccuracy)
}
$w.Stop()
"""


class LaptopLocation(threading.Thread):
    """Asks Windows where the laptop is every LAPTOP_LOCATION_EVERY seconds."""

    def __init__(self):
        super().__init__(daemon=True)
        self.location = None  # (lat, lon, accuracy m)
        self.status = "looking up…"

    def run(self):
        if os.name != "nt":
            self.status = "Windows only"
            return
        while True:
            try:
                out = subprocess.run(["powershell", "-NoProfile", "-Command", LAPTOP_LOCATION_PS],
                                     capture_output=True, text=True, timeout=40,
                                     creationflags=subprocess.CREATE_NO_WINDOW).stdout.strip()
                lat, lon, accuracy = map(float, out.split(","))
                self.location, self.status = (lat, lon, accuracy), ""
            except (OSError, subprocess.TimeoutExpired, ValueError):
                self.status = "not available (turn on Windows location)"
            time.sleep(LAPTOP_LOCATION_EVERY)


class CameraWorker(threading.Thread):
    """Finds a camera, runs the detector on every frame, re-searches if the camera drops out."""

    def __init__(self, source, detector, config):
        super().__init__(daemon=True)
        self.source, self.detector, self.config = source, detector, config
        self.running = True
        self.status = "Searching for camera…"
        self.reading = (None, "", 0.0, None)  # (distance m, label, time, annotated frame)
        self.save_csv = True   # toggled from the GUI
        self.csv_path = ""     # set when the first detection is written
        self._csv_at = 0.0

    def run(self):
        while self.running:
            source = self.source  # the GUI dropdown changes self.source to switch cameras
            cap, found = open_camera(source)
            if cap is None:
                self.status = "No camera found, retrying…"
                time.sleep(2)
                continue
            self.status = f"Camera {found} ({self.detector.name})"
            # Video files would otherwise play as fast as they decode; webcams pace themselves.
            frame_time = 1 / (cap.get(cv2.CAP_PROP_FPS) or 30) if isinstance(found, str) else 0
            while self.running and self.source == source:
                started = time.monotonic()
                ok, frame = cap.read()
                if not ok:
                    break
                obstacles, others = self.detector.detect(frame)
                draw_overlay(frame, self.detector, obstacles, others, self.config)
                distance, label = obstacles[0][:2] if obstacles else (None, "")
                self.reading = (distance, label, time.monotonic(), frame)
                if self.save_csv and (obstacles or others):
                    self._write_csv(obstacles, others)
                time.sleep(max(0, frame_time - (time.monotonic() - started)))
            cap.release()
            self.status = "Camera lost, searching…" if self.source == source else "Switching camera…"

    def _write_csv(self, obstacles, others):
        """Append everything seen in this frame, at most once every CSV_EVERY seconds."""
        if time.monotonic() - self._csv_at < CSV_EVERY:
            return
        self._csv_at = time.monotonic()
        if not self.csv_path:
            os.makedirs(CSV_DIR, exist_ok=True)
            self.csv_path = os.path.join(CSV_DIR, f"detections_{datetime.now():%Y%m%d_%H%M%S}.csv")
            with open(self.csv_path, "w", newline="") as f:
                csv.writer(f).writerow(["time", "label", "confidence", "obstacle_in_path",
                                        "distance_m", "x1", "y1", "x2", "y2"])
        stamp = datetime.now().isoformat(timespec="milliseconds")
        rows = [[stamp, label, f"{conf:.2f}", "yes", f"{d:.2f}", *box] for d, label, conf, box in obstacles]
        rows += [[stamp, label, f"{conf:.2f}", "no", "", *box] for label, conf, box in others]
        with open(self.csv_path, "a", newline="") as f:
            csv.writer(f).writerows(rows)


class ObstacleApp:
    def __init__(self, root, source, sim, backend, config, port="", focal=1.2):
        self.root = root
        root.title("Obstacle Detection")
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.sim = sim
        self.controller = ObstacleController(config)
        self.esp32 = Esp32Link()
        self.vehicle = Esp32Vehicle(self.esp32)  # only sends while an ESP32 is connected
        self.prev_state = None
        self.setting_vars = {f: tk.DoubleVar(value=getattr(config, f) * k) for f, *_, k in SETTINGS}
        self.reverse_vars = {f: tk.BooleanVar(value=getattr(config, f)) for f, _ in REVERSE_SETTINGS}
        self.esp32_sent = ("", 0.0)  # (last settings line sent to the ESP32, time)
        self.sim_obstacle = tk.BooleanVar(value=False)
        self.sim_distance = tk.DoubleVar(value=10.0)
        self.save_csv = tk.BooleanVar(value=True)

        self._build_ui(port, source)
        self.laptop_location = LaptopLocation()
        self.laptop_location.start()
        self.worker = None
        if not sim:
            self.worker = CameraWorker(source, ObstacleDetector(backend, focal=focal), config)
            self.worker.start()
            self.log(f"Camera search started ({self.worker.detector.name} detector)")
        else:
            self.log("Simulation mode: set the obstacle distance with the slider")

        root.bind("<Return>", lambda e: self.start())
        root.bind("<space>", lambda e: self.controller.stop())
        self.tick()

    # ---- layout --------------------------------------------------------
    def _build_ui(self, port, source):
        self.video = tk.Label(self.root, bg="black")
        self.video.pack(side="left", anchor="n", padx=8, pady=8)
        self.shown_frame = None
        self.show(self._placeholder("Starting camera…"))

        side = ttk.Frame(self.root, padding=8)
        side.pack(side="left", fill="both", expand=True)

        self.banner = tk.Label(side, text="", font=("Helvetica", 18, "bold"), fg="white",
                               width=24, height=2, wraplength=340)
        self.banner.pack(fill="x")
        self.reason = ttk.Label(side, text="", foreground="#c62828", font=("Helvetica", 11))
        self.reason.pack(pady=(4, 0))

        ttk.Label(side, text="Speed").pack(anchor="w", pady=(10, 0))
        self.speed_bar = ttk.Progressbar(side, maximum=100, length=340)
        self.speed_bar.pack(fill="x")
        self.speed_label = ttk.Label(side, text="0 %", font=("Helvetica", 12))
        self.speed_label.pack(anchor="e")

        buttons = tk.Frame(side)
        buttons.pack(fill="x", pady=6)
        tk.Button(buttons, text="▶ Start (Enter)", bg="#1a7f37", fg="white", height=2,
                  font=("Helvetica", 12, "bold"), command=self.start).pack(
            side="left", expand=True, fill="x", padx=(0, 4))
        tk.Button(buttons, text="■ Stop (Space)", bg="#c62828", fg="white", height=2,
                  font=("Helvetica", 12, "bold"), command=self.controller.stop).pack(
            side="left", expand=True, fill="x", padx=(4, 0))

        readings = ttk.LabelFrame(side, text=" Sensors ", padding=6)
        readings.pack(fill="x", pady=(4, 0))
        self.camera_label = ttk.Label(readings, text="")
        self.camera_label.pack(anchor="w")
        self.nearest_label = ttk.Label(readings, text="", font=("Helvetica", 11, "bold"))
        self.nearest_label.pack(anchor="w")
        self.sonar_label = ttk.Label(readings, text="")
        self.sonar_label.pack(anchor="w")
        self.gps_label = ttk.Label(readings, text="")
        self.gps_label.pack(anchor="w")
        if not self.sim:
            row = ttk.Frame(readings)
            row.pack(fill="x", pady=(0, 4))
            ttk.Label(row, text="Camera:").pack(side="left")
            choices = ["Auto"] + [str(i) for i in range(MAX_CAMERA_INDEX)]
            if isinstance(source, str):
                choices.append(source)  # video file given with --source
            self.camera_choice = tk.StringVar(value="Auto" if source is None else str(source))
            picker = ttk.Combobox(row, textvariable=self.camera_choice, values=choices,
                                  state="readonly", width=12)
            picker.pack(side="left", padx=(4, 0))
            picker.bind("<<ComboboxSelected>>", lambda e: self.pick_camera())
            ttk.Checkbutton(readings, text="Save detections to CSV", variable=self.save_csv).pack(anchor="w")
            self.csv_label = ttk.Label(readings, text="", foreground="gray")
            self.csv_label.pack(anchor="w")
        if self.sim:
            row = ttk.Frame(readings)
            row.pack(fill="x")
            ttk.Checkbutton(row, text="Obstacle at", variable=self.sim_obstacle).pack(side="left")
            ttk.Scale(row, from_=0.5, to=20, variable=self.sim_distance).pack(
                side="left", fill="x", expand=True)

        settings = ttk.LabelFrame(side, text=" Settings ", padding=6)
        settings.pack(fill="x", pady=8)
        for i, (field, label, lo, hi, step, _) in enumerate(SETTINGS):
            ttk.Label(settings, text=label).grid(row=i, column=0, sticky="w")
            ttk.Spinbox(settings, from_=lo, to=hi, increment=step, width=6,
                        textvariable=self.setting_vars[field]).grid(row=i, column=1, padx=(6, 0))
        row = ttk.Frame(settings)
        row.grid(row=len(SETTINGS), column=0, columnspan=2, sticky="w")
        for field, label in REVERSE_SETTINGS:
            ttk.Checkbutton(row, text=label, variable=self.reverse_vars[field]).pack(side="left", padx=(0, 10))

        self.esp32_panel = Esp32Panel(side, self.esp32, port)
        self.esp32_panel.pack(fill="x", pady=(0, 8))

        ttk.Label(side, text="Event log").pack(anchor="w")
        self.log_box = tk.Listbox(side, height=6, font=("Courier", 9))
        self.log_box.pack(fill="both", expand=True)

    def pick_camera(self):
        choice = self.camera_choice.get()
        self.worker.source = None if choice == "Auto" else int(choice) if choice.isdigit() else choice
        self.log(f"Camera: {choice}")

    def start(self):
        self.controller.start()
        self.log("Pilot: start")

    # ---- main loop (runs ~20 times per second) -------------------------
    def tick(self):
        config = self.controller.config
        for field, _, lo, hi, _, k in SETTINGS:
            try:
                value = min(max(float(self.setting_vars[field].get()), lo), hi)  # typed values too
                setattr(config, field, value / k)
            except (tk.TclError, ValueError):
                pass  # half-typed value in a spinbox
        for field, _ in REVERSE_SETTINGS:
            setattr(config, field, self.reverse_vars[field].get())
        esp32_line = (f"C{config.hard_stop_cm:.0f},{config.min_duty:.0f},"
                      f"{int(config.reverse_a)},{int(config.reverse_b)}")
        if esp32_line != self.esp32_sent[0] or time.monotonic() - self.esp32_sent[1] > ESP32_SETTINGS_EVERY:
            if self.esp32.send(esp32_line):
                self.esp32_sent = (esp32_line, time.monotonic())

        now = time.monotonic()
        fault = ""
        if self.sim:
            camera_m = self.sim_distance.get() if self.sim_obstacle.get() else None
            self.camera_label.config(text="Camera: simulated")
            if self.shown_frame is None:
                self.shown_frame = self._placeholder("SIMULATION")
                self.show(self.shown_frame)
        else:
            camera_m, label, frame_time, frame = self.worker.reading
            self.worker.save_csv = self.save_csv.get()
            if self.worker.csv_path:
                self.csv_label.config(text=os.path.relpath(self.worker.csv_path))
            self.camera_label.config(text=f"Camera: {self.worker.status}")
            if now - frame_time > CAMERA_LOST_AFTER:
                fault, camera_m = "No camera signal", None
                frame = self._placeholder(self.worker.status)
            if frame is not self.shown_frame:
                self.shown_frame = frame
                self.show(frame)
        self.nearest_label.config(
            text="Nearest: none" if camera_m is None
            else f"Nearest: {'obstacle' if self.sim else label} at {camera_m:.1f} m")

        self.esp32.poll()
        self.esp32_panel.update_status()
        sonar_cm = None
        if not self.esp32.connected:
            self.sonar_label.config(text="Ultrasonic: ESP32 not connected")
            if not self.sim:
                fault = fault or "ESP32 not connected"
        elif self.esp32.sonar_time is None or now - self.esp32.sonar_time > SONAR_LOST_AFTER:
            self.sonar_label.config(text="Ultrasonic: no data")
            fault = fault or "No ultrasonic data"
        else:
            left, right = self.esp32.sonar
            sonar_cm = min(left, right)
            self.sonar_label.config(text=f"Ultrasonic: left {left:.0f} cm   right {right:.0f} cm")

        gps = self.esp32.gps
        if not self.esp32.connected:
            gps_text = "GPS: ESP32 not connected"
        elif self.esp32.gps_time is None or now - self.esp32.gps_time > GPS_LOST_AFTER:
            gps_text = "GPS: no data"
        elif not gps.get("fix"):
            gps_text = f"GPS: no fix yet ({gps.get('sats', 0)} satellites)"
        else:
            gps_text = (f"GPS: {gps['lat']:.6f}, {gps['lon']:.6f}   "
                        f"{gps['speed_kmh']:.1f} km/h   {gps.get('sats', '?')} sats")
        laptop = self.laptop_location.location
        if laptop and not (self.esp32.connected and gps.get("fix")):
            gps_text = f"GPS: {laptop[0]:.6f}, {laptop[1]:.6f}"  # laptop location until the GPS has a fix
        self.gps_label.config(text=gps_text)

        speed = self.controller.update(camera_m, sonar_cm, fault)
        self.vehicle.set_speed(speed)
        self.refresh_panel()
        self.root.after(50, self.tick)

    def refresh_panel(self):
        c = self.controller
        if c.state != self.prev_state:
            show_reason = c.state in (State.STOPPED, State.BLOCKED)
            self.log(c.state.value + (f": {c.reason}" if show_reason else ""))
            if c.state == State.BLOCKED:
                self.root.bell()
            self.prev_state = c.state
        self.banner.config(text=c.state.value.upper(), bg=STATE_COLORS[c.state])
        self.reason.config(text=c.reason if c.state in (State.STOPPED, State.BLOCKED) else "")
        self.speed_bar["value"] = c.speed * 100
        self.speed_label.config(text=f"{c.speed * 100:.0f} %")

    def _placeholder(self, text):
        img = np.zeros((VIDEO_H, VIDEO_W, 3), np.uint8)
        cv2.putText(img, text, (40, VIDEO_H // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (200, 200, 200), 2)
        return img

    def show(self, frame):
        h, w = frame.shape[:2]
        scale = min(VIDEO_W / w, VIDEO_H / h)
        frame = cv2.resize(frame, (int(w * scale), int(h * scale)))
        image = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)))
        self.video.configure(image=image)
        self.video.image = image

    def log(self, text):
        self.log_box.insert(0, f"{datetime.now():%H:%M:%S}  {text}")

    def on_close(self):
        self.vehicle.stop()
        self.esp32.close()
        if self.worker:
            self.worker.running = False
            self.worker.join(timeout=2)
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description="Obstacle detection with distance-based speed limits.")
    parser.add_argument("--source", default=None,
                        help="camera index or video file (default: first camera that works)")
    parser.add_argument("--sim", action="store_true", help="no camera; set obstacle distance in the GUI")
    parser.add_argument("--detector", choices=["auto", "yolo", "edges"], default="auto")
    parser.add_argument("--focal", type=float, default=1.2,
                        help="camera focal length / frame height, for distance estimates (see detector.py)")
    parser.add_argument("--port", default="", help="ESP32 serial port to connect at startup, e.g. COM3")
    args = parser.parse_args()

    source = int(args.source) if args.source and args.source.isdigit() else args.source
    pid_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "app.pid")
    with open(pid_file, "w") as f:
        f.write(str(os.getpid()))  # lets stop.bat find this process
    root = tk.Tk()
    ObstacleApp(root, source, args.sim, args.detector, Config(), args.port, args.focal)
    try:
        root.mainloop()
    finally:
        if os.path.exists(pid_file):
            os.remove(pid_file)


if __name__ == "__main__":
    main()
