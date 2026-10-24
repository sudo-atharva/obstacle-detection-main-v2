"""USB serial link to the ESP32. Sends one text command per line and reads back the
ultrasonic readings the firmware sends as "U<left cm>,<right cm>", plus the GPS module's
NMEA RMC/GGA sentences, which the firmware passes through unchanged.

Requires: pip install pyserial
"""
import threading
import time


def list_ports() -> list[str]:
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        return []
    return [p.device for p in lp.comports()]


def parse_sonar(line: bytes):
    """b"U123,85" -> (123.0, 85.0); anything else -> None."""
    try:
        text = line.decode().strip()
        if text[:1] == "U":
            left, right = map(float, text[1:].split(","))
            return left, right
    except (UnicodeDecodeError, ValueError):
        pass
    return None


def _degrees(value: str, hemisphere: str) -> float:
    """NMEA ddmm.mmmm -> decimal degrees, negative for S/W."""
    raw = float(value)
    degrees = int(raw // 100) + (raw % 100) / 60
    return -degrees if hemisphere in ("S", "W") else degrees


def parse_nmea(line: bytes):
    """b"$GNRMC,..." / b"$GPGGA,..." -> dict of the GPS fields it carries; anything else -> None.
    Sentences with a bad checksum are dropped."""
    try:
        text = line.decode().strip()
        if text[:1] != "$" or "*" not in text:
            return None
        body, checksum = text[1:].split("*", 1)
        calc = 0
        for ch in body:
            calc ^= ord(ch)
        if calc != int(checksum[:2], 16):
            return None
        f = body.split(",")
        if f[0][2:] == "RMC":
            if f[2] != "A":
                return {"fix": False}
            return {"fix": True, "lat": _degrees(f[3], f[4]), "lon": _degrees(f[5], f[6]),
                    "speed_kmh": float(f[7] or 0) * 1.852}
        if f[0][2:] == "GGA":
            return {"sats": int(f[7] or 0)}
    except (UnicodeDecodeError, ValueError, IndexError):
        pass
    return None


class Esp32Link:
    def __init__(self):
        self.conn = None
        self.lock = threading.Lock()
        self.error = ""
        self.sonar = None       # latest (left_cm, right_cm)
        self.sonar_time = None  # time.monotonic() of that reading
        self.gps = {}           # latest fix, lat, lon, speed_kmh, sats (whatever has arrived)
        self.gps_time = None    # time.monotonic() of the last GPS sentence
        self._buf = b""

    @property
    def connected(self) -> bool:
        return self.conn is not None

    def connect(self, port: str, baud: int = 115200):
        import serial
        self.close()
        # dsrdtr/rtscts off and DTR/RTS low so opening the port doesn't reset the ESP32.
        conn = serial.Serial()
        conn.port, conn.baudrate, conn.timeout, conn.write_timeout = port, baud, 0, 0.2
        conn.dtr = conn.rts = False
        conn.open()
        self.conn = conn
        self.error = ""
        self.sonar = self.sonar_time = None
        self.gps, self.gps_time = {}, None
        self._buf = b""

    def poll(self):
        """Read whatever the ESP32 sent; keeps the latest ultrasonic reading."""
        with self.lock:
            if not self.conn:
                return
            try:
                data = self.conn.read(self.conn.in_waiting)
            except Exception as exc:
                self.error = str(exc)
                self._close_locked()
                return
        *lines, rest = (self._buf + data).split(b"\n")
        self._buf = rest[-64:]
        for line in lines:
            reading = parse_sonar(line)
            if reading:
                self.sonar, self.sonar_time = reading, time.monotonic()
            gps = parse_nmea(line)
            if gps:
                self.gps.update(gps)
                self.gps_time = time.monotonic()

    def send(self, line: str) -> bool:
        """Send one command. On failure (cable unplugged) the link closes itself."""
        with self.lock:
            if not self.conn:
                return False
            try:
                self.conn.write((line + "\n").encode())
                return True
            except Exception as exc:
                self.error = str(exc)
                self._close_locked()
                return False

    def close(self):
        with self.lock:
            self._close_locked()

    def _close_locked(self):
        if self.conn:
            try:
                self.conn.close()
            except Exception:
                pass
            self.conn = None


class Esp32Panel:
    """Small GUI block: COM port picker + Connect button + status line."""

    def __init__(self, parent, link: Esp32Link, initial_port: str = ""):
        import tkinter as tk
        from tkinter import ttk

        self.link = link
        self.frame = ttk.LabelFrame(parent, text=" ESP32 (USB) ", padding=6)
        self.port_var = tk.StringVar(value=initial_port)
        row = ttk.Frame(self.frame)
        row.pack(fill="x")
        self.ports = ttk.Combobox(row, textvariable=self.port_var, width=14)
        self.ports.pack(side="left", fill="x", expand=True)
        ttk.Button(row, text="⟳", width=3, command=self.refresh).pack(side="left")
        self.button = ttk.Button(row, text="Connect", width=10, command=self.toggle)
        self.button.pack(side="left", padx=(4, 0))
        self.status = ttk.Label(self.frame, text="Not connected", foreground="gray")
        self.status.pack(anchor="w")
        self.refresh()
        if initial_port:
            self.toggle()

    def pack(self, **kw):
        self.frame.pack(**kw)

    def refresh(self):
        ports = list_ports()
        self.ports["values"] = ports
        if not self.port_var.get() and ports:
            self.port_var.set(ports[0])

    def toggle(self):
        if self.link.connected:
            self.link.close()
        else:
            port = self.port_var.get().strip()
            if not port:
                self.status.config(text="Pick a port first", foreground="#c62828")
                return
            try:
                self.link.connect(port)
            except ImportError:
                self.link.error = "pyserial is not installed"
            except Exception as exc:
                self.link.error = str(exc)
        self.update_status()

    def update_status(self):
        """Call regularly so an unplugged cable shows up in the GUI."""
        if self.link.connected:
            self.status.config(text=f"Connected to {self.port_var.get()}", foreground="#1a7f37")
            self.button.config(text="Disconnect")
        else:
            text = f"Error: {self.link.error}" if self.link.error else "Not connected"
            self.status.config(text=text[:45], foreground="#c62828" if self.link.error else "gray")
            self.button.config(text="Connect")
