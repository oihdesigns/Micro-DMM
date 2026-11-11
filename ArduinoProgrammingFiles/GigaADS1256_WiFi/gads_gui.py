"""ADS1256 control panel with triggering, for a Giga R1 WiFi running GigaADS1256_WiFi.

The Wi-Fi version of "Raspberry Pi Project/ADS1256 GUI V3_triggering.py": same
panels, same channel map, DMM math, trigger modes and CSV format. The board does
the SPI, the triggering and the pre-trigger buffering; this program only sends
settings and draws what comes back.

    py -3 gads_gui.py            (needs matplotlib + numpy; pyserial for USB)

Type the board's IP address (it is in $NET / the status line over USB) or a COM
port in the Board box and press Connect. "127.0.0.1" talks to gads_sim.py.
"""

import csv
import datetime
import json
import math
import os
import queue
import socket
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import filedialog, scrolledtext, ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

try:
    import serial
    import serial.tools.list_ports
except ImportError:          # Wi-Fi only without pyserial
    serial = None

# ---------------- ADS1256 constants (as the Pi GUI) ----------------

# Volts = code * 2 * VREF / (gain * 0x7FFFFF). The Pi GUI wrote this as
# code * 5.0 / (0x7FFFFF * gain) with "VREF = 5.000": the same number for the
# usual 2.5 V reference, so the DMM math constants below carry over unchanged.
VREF = 2.500
FULL_CODE = 0x7FFFFF

DRATE_TABLE = {
    "30k SPS": 0xF0,
    "15k SPS": 0xE0,
    "7.5k SPS": 0xD0,
    "3.75k SPS": 0xC0,
    "2k SPS": 0xB0,
    "1k SPS": 0xA1,
    "500 SPS": 0x92,
    "100 SPS": 0x82,
    "60 SPS": 0x72,
    "50 SPS": 0x63,
    "30 SPS": 0x53,
    "25 SPS": 0x43,
    "15 SPS": 0x33,
    "10 SPS": 0x23,
    "5 SPS": 0x13,
    "2.5 SPS": 0x03,
}
DRATE_NAMES = {v: k for k, v in DRATE_TABLE.items()}

GAIN_TABLE = {
    "1x": 0,
    "2x": 1,
    "4x": 2,
    "8x": 3,
    "16x": 4,
    "32x": 5,
    "64x": 6,
}

# Channel map: label, (pos, neg), type. 8 = AINCOM. Up to 8 entries; the board
# takes whatever this says (!CHMAP).
CHANNEL_MAP = [
    ("CH0 (SE)", (0, 8), "SE"),
    ("CH1 (SE)", (1, 8), "SE"),
    ("CH2 (SE)", (2, 8), "SE"),
    ("CH3 (SE)", (3, 8), "SE"),
    ("CH4-5 (Diff)", (4, 5), "DIFF"),
    ("CH6-7 (Diff)", (6, 7), "DIFF"),
]

TRIGGER_MODES = ("Benchmark", "Multi")
TRIGGER_SLOPES = ("Rising", "Falling")
TRIGGER_POSITIONS = ("Start", "Mid", "End")

RANGE_PIN = "D6"
RESERVED_TEXT = "Reserved: D2 DRDY, D6 range, D7 RESET, D10-D13 SPI"
GPIO_MODES = ("Input", "Output Low", "Output High", "Input Pullup")

DEFAULT_WIFI_ADDR = "192.168.40.31"     # the test Giga on DHCP, 2026-11-11
TCP_PORT = 8080
LIVE_PERIOD_MS = 500
MAX_POINTS = 300
SETTINGS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gads_gui_settings.json")

HEADER_FONT = ("TkDefaultFont", 16, "bold")
CHANNEL_FONT = ("TkDefaultFont", 12)
VALUE_FONT = ("Courier New", 15, "bold")
SD_FONT = ("Courier New", 10)
UNIT_FONT = ("TkDefaultFont", 11, "bold")
BUTTON_FONT = ("TkDefaultFont", 12)


# ---------------- link to the board ----------------

def is_network_addr(s):
    s = s.strip()
    return s.lower().startswith("localhost") or "." in s or (":" in s and not s.upper().startswith("COM"))


class TcpPort:
    """The board over Wi-Fi with a serial port's read/write/close.
    read() returns b"" on a timeout and raises once the board hangs up."""

    def __init__(self, target, timeout=0.05):
        host, _, port = target.strip().partition(":")
        self.sock = socket.create_connection((host, int(port or TCP_PORT)), timeout=5)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(timeout)

    def read(self, n):
        try:
            data = self.sock.recv(n)
        except socket.timeout:
            return b""
        if not data:
            raise ConnectionError("the board closed the connection")
        return data

    def write(self, data):
        self.sock.sendall(data)
        return len(data)

    def close(self):
        self.sock.close()


class SerialPort:
    def __init__(self, port):
        if serial is None:
            raise RuntimeError("pyserial is not installed (py -3 -m pip install pyserial)")
        self.ser = serial.Serial(port, 115200, timeout=0.05, write_timeout=1.0)

    def read(self, n):
        return self.ser.read(max(1, min(n, self.ser.in_waiting or 1)))

    def write(self, data):
        return self.ser.write(data)

    def close(self):
        self.ser.close()


class Capture:
    """One capture as it came from the board: raw codes, frame times."""

    def __init__(self, frames, order, trig_idx, gaps, flags, dt_us, drate, pga, buf):
        self.frames = frames
        self.order = order                       # channel index per column
        self.trig_idx = trig_idx                 # frame of the trigger, -1 = none
        self.gaps = gaps
        self.flags = flags
        self.dt_us = dt_us
        self.drate = drate
        self.pga = pga
        self.buf = buf
        self.raw = np.full((frames, len(order)), np.nan)
        self.t_ms = np.full(frames, np.nan)
        self.got = 0
        self.bad_lines = 0
        # filled in by the GUI when the capture was asked for
        self.mode = ""
        self.keep = list(order)                  # columns to show (multi drops an unselected trigger channel)
        self.range_high = True

    def add_line(self, start, blob):
        nch = len(self.order)
        width = 4 + 3 * nch
        data = np.frombuffer(bytes.fromhex(blob), dtype=np.uint8)
        if data.size % width:
            self.bad_lines += 1
            return
        m = data.reshape(-1, width).astype(np.int64)
        k = m.shape[0]
        if start < 0 or start + k > self.frames:
            self.bad_lines += 1
            return
        t = (m[:, 0] << 24) | (m[:, 1] << 16) | (m[:, 2] << 8) | m[:, 3]
        self.t_ms[start:start + k] = t / 1000.0
        for c in range(nch):
            b = m[:, 4 + 3 * c:7 + 3 * c]
            v = (b[:, 0] << 16) | (b[:, 1] << 8) | b[:, 2]
            v = np.where(v & 0x800000, v - 0x1000000, v)
            self.raw[start:start + k, c] = v
        self.got += k


class Link:
    """Line protocol over TCP or USB serial. A reader thread parses lines and
    posts events to self.events; captures are assembled here, off the GUI thread."""

    def __init__(self, target):
        self.target = target
        self.port = TcpPort(target) if is_network_addr(target) else SerialPort(target)
        self.events = queue.Queue()
        self._alive = True
        self._txlock = threading.Lock()
        self._cap = None
        self._last_progress = 0.0
        self._rx = threading.Thread(target=self._reader, daemon=True, name="gads-rx")
        self._rx.start()

    def send(self, cmd):
        with self._txlock:
            self.port.write((cmd + "\n").encode("ascii"))
        self.events.put(("tx", cmd))

    def close(self):
        self._alive = False
        try:
            self.port.close()
        except Exception:                                   # noqa: BLE001
            pass

    def _reader(self):
        buf = b""
        while self._alive:
            try:
                data = self.port.read(4096)
            except Exception as exc:                        # noqa: BLE001
                if self._alive:
                    self.events.put(("closed", str(exc)))
                return
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip()
                if line:
                    try:
                        self._line(line)
                    except Exception as exc:                # noqa: BLE001
                        self.events.put(("log", f"parse error: {exc}: {line[:80]}"))

    def _line(self, line):
        if line.startswith("$CD,"):
            cap = self._cap
            if cap is None:
                return
            _, start, blob = line.split(",", 2)
            cap.add_line(int(start), blob)
            now = time.monotonic()
            if now - self._last_progress > 0.25:
                self._last_progress = now
                self.events.put(("progress", cap.got, cap.frames))
            return
        self.events.put(("line", line))
        f = line.split(",")
        tag = f[0]
        if tag == "$LIVE":
            n = int(f[3])
            chans = {}
            for i in range(n):
                b = 4 + 6 * i
                chans[int(f[b])] = dict(mean=float(f[b + 1]), sd=float(f[b + 2]),
                                        min=int(f[b + 3]), max=int(f[b + 4]), clip=int(f[b + 5]))
            self.events.put(("live", int(f[1]), float(f[2]), chans))
        elif tag == "$CAP":
            order = [int(x) for x in f[3].split(":")]
            self._cap = Capture(int(f[1]), order, int(f[4]), int(f[5]), int(f[6]), float(f[7]),
                                int(f[8], 0), int(f[9]), int(f[10]))
            self.events.put(("cap_start", self._cap))
        elif tag == "$CEND":
            cap, self._cap = self._cap, None
            if cap is not None:
                self.events.put(("capture", cap))
        elif tag == "$STATUS":
            d = {}
            for kv in f[1:]:
                k, _, v = kv.partition("=")
                d[k] = v
            self.events.put(("status", d))
        elif tag == "$GPIOLIST":
            self.events.put(("gpiolist", [int(x) for x in f[1:] if x]))
        elif tag == "$GPIOS":
            st = []
            for item in f[1:]:
                p, m, lv = item.split(":")
                st.append((int(p), int(m), int(lv)))
            self.events.put(("gpios", st))
        elif tag == "$GPIO":
            self.events.put(("gpios", [(int(f[1]), int(f[2]), int(f[3]))]))
        else:
            self.events.put((tag, f[1:]))


# ---------------- unit conversions ----------------

def code_to_volts(code, pga):
    return code * (2.0 * VREF) / ((1 << pga) * FULL_CODE)


def volts_to_code(volts, pga):
    return volts * ((1 << pga) * FULL_CODE) / (2.0 * VREF)


def dmm_formula(ch_index, volts, range_high):
    """The Pi GUI's DMM math, unclamped (numpy or float)."""
    with np.errstate(divide="ignore", invalid="ignore"):
        if ch_index == 5:
            return volts * -68.36437
        if ch_index == 3:
            if range_high:
                return 22000.0 * (volts / (5.001 - volts))
            return volts / (0.02016 - (volts / 330.0))
    return volts


def dmm_units(ch_index):
    return "Ω" if ch_index == 3 else "Calc"


def clamp_over(x):
    """NaN where the Pi GUI showed OVER."""
    x = np.asarray(x, dtype=float)
    with np.errstate(invalid="ignore"):
        return np.where(np.isfinite(x) & (np.abs(x) <= 10_000_000), x, np.nan)


# ---------------- the GUI ----------------

class App:
    def __init__(self, root):
        self.root = root
        self.link = None
        self.connecting = False
        self.status_dict = {}
        self.is_capturing = False
        self.capture = None
        self.cap_request = None
        self.pga = 0                       # applied gain, for conversions
        self.channel_units = [""] * len(CHANNEL_MAP)
        self.data_buffers = [deque(maxlen=MAX_POINTS) for _ in CHANNEL_MAP]
        self.gpio_rows = {}                # pin -> (var, combo, level label)
        self.gpio_busy_until = 0.0
        self.console = None
        self.ui_q = queue.Queue()          # results from worker threads (Tk is main-thread only)
        self.settings = self._load_settings()

        root.title("ADS1256 Control Panel with Triggering (Giga Wi-Fi)")
        root.geometry("1400x860")
        root.configure(padx=12, pady=8)
        root.columnconfigure(1, weight=1)
        root.rowconfigure(1, weight=1)

        self.status_var = tk.StringVar(value="Not connected")
        self.conn_var = tk.StringVar(value="Not connected")

        self._build_connection_bar()
        left = ttk.Frame(root)
        left.grid(row=1, column=0, sticky="nsw", padx=(0, 12))
        right = ttk.Frame(root)
        right.grid(row=1, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=1)
        right.rowconfigure(1, weight=1)
        self._build_channels(left)
        self._build_options(left)
        self._build_math(left)
        self._build_gpio(left)
        self._build_live(right)
        self._build_logger(right)

        self.trigger_channel_var.trace_add("write", self.update_trigger_units)
        self.update_trigger_units()
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.after(30, self._poll)
        root.after(1000, self._tick)

    # ---------- settings file ----------
    def _load_settings(self):
        try:
            with open(SETTINGS_FILE, encoding="utf-8") as fh:
                return json.load(fh)
        except (OSError, ValueError):
            return {}

    def _save_settings(self):
        s = dict(
            addr=self.addr_var.get().strip(),
            drate=self.drate_var.get(), gain=self.gain_var.get(), buffer=self.buffer_var.get(),
            channels=[v.get() for v in self.channel_vars], points=self.points_var.get(),
            mode=self.trigger_mode_var.get(), trig_ch=self.trigger_channel_var.get(),
            level=self.trigger_level_var.get(), slope=self.trigger_slope_var.get(),
            position=self.trigger_position_var.get(), timeout=self.timeout_var.get(),
            dmm=self.dmm_math_var.get(), range_high=self.dmm_range_var.get(), raw=self.raw_var.get(),
        )
        try:
            with open(SETTINGS_FILE, "w", encoding="utf-8") as fh:
                json.dump(s, fh, indent=1)
        except OSError:
            pass

    # ---------- layout ----------
    def _build_connection_bar(self):
        bar = ttk.Frame(self.root)
        bar.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))
        bar.columnconfigure(5, weight=1)
        ttk.Label(bar, text="Board (IP or COM port):", font=CHANNEL_FONT).grid(row=0, column=0, padx=(0, 6))
        self.addr_var = tk.StringVar(value=self.settings.get("addr", DEFAULT_WIFI_ADDR))
        self.addr_combo = ttk.Combobox(bar, textvariable=self.addr_var, width=22, font=CHANNEL_FONT,
                                       postcommand=self._refresh_ports)
        self.addr_combo.grid(row=0, column=1, padx=4)
        self._refresh_ports()
        self.connect_button = tk.Button(bar, text="Connect", font=BUTTON_FONT, width=11, command=self.toggle_connect)
        self.connect_button.grid(row=0, column=2, padx=4)
        tk.Button(bar, text="Console", font=BUTTON_FONT, command=self.open_console).grid(row=0, column=3, padx=4)
        self.cal_button = tk.Button(bar, text="Self-Cal", font=BUTTON_FONT, command=self.request_calibration)
        self.cal_button.grid(row=0, column=4, padx=4)
        ttk.Label(bar, textvariable=self.conn_var, font=CHANNEL_FONT).grid(row=0, column=5, sticky="w", padx=12)

    def _refresh_ports(self):
        vals = []
        for a in (self.addr_var.get().strip(), DEFAULT_WIFI_ADDR, "127.0.0.1"):
            if a and a not in vals:
                vals.append(a)
        if serial is not None:
            try:
                vals += [p.device for p in serial.tools.list_ports.comports()]
            except Exception:                               # noqa: BLE001
                pass
        self.addr_combo["values"] = vals

    def _build_channels(self, parent):
        frame = ttk.LabelFrame(parent, text="Channel Controls")
        frame.grid(row=0, column=0, sticky="nw", pady=(0, 10))
        self.channel_vars, self.value_labels, self.unit_labels, self.sd_labels = [], [], [], []
        saved = self.settings.get("channels", [])
        for row, (label_txt, _, _) in enumerate(CHANNEL_MAP):
            var = tk.BooleanVar(value=saved[row] if row < len(saved) else True)
            tk.Checkbutton(frame, text=label_txt, font=CHANNEL_FONT, variable=var, anchor="w",
                           justify="left", command=self.on_channels_changed).grid(row=row, column=0, sticky="w", padx=4, pady=2)
            val = tk.Label(frame, text="---", font=VALUE_FONT, width=14, anchor="e")
            val.grid(row=row, column=1, padx=6, pady=2, sticky="e")
            unit = tk.Label(frame, text="", font=UNIT_FONT, width=5, anchor="w")
            unit.grid(row=row, column=2, padx=2, pady=2, sticky="w")
            sd = tk.Label(frame, text="", font=SD_FONT, width=13, anchor="w", fg="#666666")
            sd.grid(row=row, column=3, padx=2, pady=2, sticky="w")
            self.channel_vars.append(var)
            self.value_labels.append(val)
            self.unit_labels.append(unit)
            self.sd_labels.append(sd)

    def _build_options(self, parent):
        frame = ttk.LabelFrame(parent, text="Acquisition Options")
        frame.grid(row=1, column=0, sticky="new")
        self.buffer_var = tk.BooleanVar(value=self.settings.get("buffer", False))
        self.buffer_check = tk.Checkbutton(frame, text="Enable Buffer", variable=self.buffer_var, font=CHANNEL_FONT,
                                           command=self.on_buffer)
        self.buffer_check.grid(row=0, column=0, sticky="w", padx=4, pady=4)
        self.raw_var = tk.BooleanVar(value=self.settings.get("raw", False))
        tk.Checkbutton(frame, text="Show Raw Integer", variable=self.raw_var, font=CHANNEL_FONT,
                       command=self.on_display_mode).grid(row=0, column=1, sticky="w", padx=4, pady=4)
        ttk.Label(frame, text="Sample Rate:", font=CHANNEL_FONT).grid(row=1, column=0, sticky="w", padx=4, pady=4)
        self.drate_var = tk.StringVar(value=self.settings.get("drate", "30k SPS"))
        self.drate_menu = ttk.Combobox(frame, textvariable=self.drate_var, values=list(DRATE_TABLE.keys()),
                                       state="readonly", width=12)
        self.drate_menu.grid(row=1, column=1, sticky="w", padx=4, pady=4)
        self.drate_menu.bind("<<ComboboxSelected>>", lambda _e: self.on_drate())
        ttk.Label(frame, text="Gain:", font=CHANNEL_FONT).grid(row=2, column=0, sticky="w", padx=4, pady=4)
        self.gain_var = tk.StringVar(value=self.settings.get("gain", "1x"))
        self.gain_menu = ttk.Combobox(frame, textvariable=self.gain_var, values=list(GAIN_TABLE.keys()),
                                      state="readonly", width=12)
        self.gain_menu.grid(row=2, column=1, sticky="w", padx=4, pady=4)
        self.gain_menu.bind("<<ComboboxSelected>>", lambda _e: self.on_gain())
        self.pga = GAIN_TABLE.get(self.gain_var.get(), 0)

    def _build_math(self, parent):
        frame = ttk.LabelFrame(parent, text="DMM Math")
        frame.grid(row=2, column=0, sticky="new", pady=10)
        self.dmm_math_var = tk.BooleanVar(value=self.settings.get("dmm", False))
        self.dmm_range_var = tk.BooleanVar(value=self.settings.get("range_high", True))
        tk.Checkbutton(frame, text="Enable DMM Math", font=CHANNEL_FONT, variable=self.dmm_math_var,
                       command=self.on_dmm_math_toggle).grid(row=0, column=0, sticky="w", padx=4, pady=4)
        self.range_check = tk.Checkbutton(frame, text=f"High Range ({RANGE_PIN})", font=CHANNEL_FONT,
                                          variable=self.dmm_range_var, command=self.on_range_toggle,
                                          state="normal" if self.dmm_math_var.get() else "disabled")
        self.range_check.grid(row=1, column=0, sticky="w", padx=4, pady=4)

    def _build_gpio(self, parent):
        self.gpio_frame = ttk.LabelFrame(parent, text="GPIO Controls")
        self.gpio_frame.grid(row=3, column=0, sticky="new")
        ttk.Label(self.gpio_frame, text=RESERVED_TEXT, font=("TkDefaultFont", 10, "italic")).grid(
            row=0, column=0, columnspan=6, sticky="w", padx=4, pady=(4, 6))
        self.gpio_hint = ttk.Label(self.gpio_frame, text="(connect to list the board's pins)", font=CHANNEL_FONT)
        self.gpio_hint.grid(row=1, column=0, columnspan=6, sticky="w", padx=4)

    def _build_live(self, parent):
        frame = ttk.LabelFrame(parent, text="Live Graphs")
        frame.grid(row=0, column=0, sticky="nsew", pady=(0, 10))
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)
        frame.rowconfigure(1, weight=1)
        self.live_values_var = tk.BooleanVar(value=True)
        self.live_graph_var = tk.BooleanVar(value=True)
        tk.Checkbutton(frame, text="Update Channel Readings", font=CHANNEL_FONT, variable=self.live_values_var,
                       command=self.on_live_values_toggle).grid(row=0, column=0, sticky="w", padx=4, pady=(4, 0))
        tk.Checkbutton(frame, text="Live Graph Enabled", font=CHANNEL_FONT, variable=self.live_graph_var,
                       command=self.on_live_graph_toggle).grid(row=0, column=1, sticky="w", padx=4, pady=(4, 0))
        self.fig = Figure(figsize=(8, 3.2), dpi=96)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.get_tk_widget().grid(row=1, column=0, columnspan=2, sticky="nsew")

    def _build_logger(self, parent):
        frame = ttk.LabelFrame(parent, text="Data Logger & Triggering")
        frame.grid(row=1, column=0, sticky="nsew")
        for c in range(6):
            frame.columnconfigure(c, weight=1 if c == 5 else 0)
        frame.rowconfigure(5, weight=1)
        s = self.settings
        self.points_var = tk.StringVar(value=s.get("points", "1000"))
        self.trigger_level_var = tk.StringVar(value=s.get("level", "0.0"))
        self.trigger_mode_var = tk.StringVar(value=s.get("mode", TRIGGER_MODES[0]))
        self.trigger_slope_var = tk.StringVar(value=s.get("slope", TRIGGER_SLOPES[0]))
        self.trigger_position_var = tk.StringVar(value=s.get("position", TRIGGER_POSITIONS[0]))
        self.timeout_var = tk.StringVar(value=s.get("timeout", "0"))
        self.channel_names = [c[0] for c in CHANNEL_MAP]
        self.channel_lookup = {name: idx for idx, name in enumerate(self.channel_names)}
        tc = s.get("trig_ch", self.channel_names[0])
        self.trigger_channel_var = tk.StringVar(value=tc if tc in self.channel_lookup else self.channel_names[0])

        def lab(r, c, text):
            ttk.Label(frame, text=text, font=CHANNEL_FONT).grid(row=r, column=c, sticky="w", padx=4, pady=3)

        lab(0, 0, "Points to Capture:")
        ttk.Entry(frame, textvariable=self.points_var, width=12, font=CHANNEL_FONT).grid(row=0, column=1, sticky="w", padx=4)
        lab(0, 2, "Mode:")
        ttk.Combobox(frame, textvariable=self.trigger_mode_var, values=list(TRIGGER_MODES), state="readonly",
                     width=12).grid(row=0, column=3, sticky="w", padx=4)
        lab(1, 0, "Trigger Channel:")
        ttk.Combobox(frame, textvariable=self.trigger_channel_var, values=self.channel_names, state="readonly",
                     width=16).grid(row=1, column=1, sticky="w", padx=4)
        lab(1, 2, "Slope:")
        ttk.Combobox(frame, textvariable=self.trigger_slope_var, values=list(TRIGGER_SLOPES), state="readonly",
                     width=12).grid(row=1, column=3, sticky="w", padx=4)
        lab(2, 0, "Trigger Level:")
        lvl = ttk.Frame(frame)
        lvl.grid(row=2, column=1, sticky="w", padx=4)
        ttk.Entry(lvl, textvariable=self.trigger_level_var, width=12, font=CHANNEL_FONT).pack(side="left")
        self.trigger_units_label = ttk.Label(lvl, text="Units: V", font=CHANNEL_FONT)
        self.trigger_units_label.pack(side="left", padx=6)
        lab(2, 2, "Trigger Position:")
        ttk.Combobox(frame, textvariable=self.trigger_position_var, values=list(TRIGGER_POSITIONS),
                     state="readonly", width=12).grid(row=2, column=3, sticky="w", padx=4)
        lab(3, 0, "Timeout (s, 0 = none):")
        ttk.Entry(frame, textvariable=self.timeout_var, width=12, font=CHANNEL_FONT).grid(row=3, column=1, sticky="w", padx=4)

        bf = ttk.Frame(frame)
        bf.grid(row=3, column=2, columnspan=4, sticky="w", padx=4, pady=4)
        self.arm_button = tk.Button(bf, text="Arm Trigger", font=BUTTON_FONT, command=lambda: self.perform_capture(True))
        self.arm_button.grid(row=0, column=0, padx=4)
        self.capture_button = tk.Button(bf, text="Capture Now", font=BUTTON_FONT, command=lambda: self.perform_capture(False))
        self.capture_button.grid(row=0, column=1, padx=4)
        self.abort_button = tk.Button(bf, text="Abort", font=BUTTON_FONT, command=self.abort_capture, state="disabled")
        self.abort_button.grid(row=0, column=2, padx=4)
        self.export_button = tk.Button(bf, text="Export CSV", font=BUTTON_FONT, command=self.export_csv)
        self.export_button.grid(row=0, column=3, padx=4)

        ttk.Label(frame, textvariable=self.status_var, font=CHANNEL_FONT).grid(
            row=4, column=0, columnspan=6, sticky="w", padx=4, pady=4)

        self.log_fig = Figure(figsize=(8, 2.6), dpi=96)
        self.log_ax = self.log_fig.add_subplot(111)
        self.log_canvas = FigureCanvasTkAgg(self.log_fig, master=frame)
        self.log_canvas.get_tk_widget().grid(row=5, column=0, columnspan=6, sticky="nsew", padx=4, pady=4)
        self.stats_frame = ttk.Frame(frame)
        self.stats_frame.grid(row=6, column=0, columnspan=6, sticky="new", padx=4, pady=(0, 6))

    # ---------- connection ----------
    def toggle_connect(self):
        if self.link is not None:
            self.disconnect("Disconnected")
            return
        if self.connecting:
            return
        target = self.addr_var.get().strip()
        if not target:
            return
        self.connecting = True
        self.connect_button.config(text="Connecting...", state="disabled")
        self.conn_var.set(f"Connecting to {target} ...")

        def worker():
            try:
                self.ui_q.put(("connected", Link(target)))
            except Exception as exc:                        # noqa: BLE001
                self.ui_q.put(("failed", str(exc)))

        threading.Thread(target=worker, daemon=True).start()

    def _connect_failed(self, msg):
        self.connecting = False
        self.connect_button.config(text="Connect", state="normal")
        self.conn_var.set(f"Connect failed: {msg}")

    def _connected(self, link):
        self.connecting = False
        self.link = link
        self.connect_button.config(text="Disconnect", state="normal")
        self.conn_var.set(f"Connected to {link.target}")
        self.status_var.set("Idle")
        self.push_settings()

    def push_settings(self):
        """The GUI is the source of truth for the ADC settings: send them all."""
        mux = ",".join(f"0x{(p << 4) | n:02X}" for _, (p, n), _ in CHANNEL_MAP)
        self.send("!LIVE,0")
        self.send(f"!CHMAP,{mux}")
        self.send(f"!BUF,{1 if self.buffer_var.get() else 0}")
        self.send(f"!GAIN,{GAIN_TABLE.get(self.gain_var.get(), 0)}")
        self.send(f"!DRATE,0x{DRATE_TABLE.get(self.drate_var.get(), 0xF0):02X}")
        self.send(f"!CH,{self.channel_mask()}")
        self.send(f"!RANGE,{1 if self.dmm_range_var.get() else 0}")
        self.send(f"!LIVE,1,{LIVE_PERIOD_MS}")
        self.send("!ID")
        self.send("!STATUS")
        self.send("!GPIOLIST")
        self.send("!GPIOS")
        self.pga = GAIN_TABLE.get(self.gain_var.get(), 0)

    def disconnect(self, why):
        link, self.link = self.link, None
        if link is not None:
            try:
                link.send("!LIVE,0")
            except Exception:                               # noqa: BLE001
                pass
            link.close()
        self.connect_button.config(text="Connect", state="normal")
        self.conn_var.set(why)
        if self.is_capturing:
            self.finish_capture(f"Capture failed: {why}")
        for i, lbl in enumerate(self.value_labels):
            lbl.config(text="---")
            self.sd_labels[i].config(text="")

    def send(self, cmd):
        if self.link is None:
            return False
        try:
            self.link.send(cmd)
            return True
        except Exception as exc:                            # noqa: BLE001
            self.disconnect(f"Link lost: {exc}")
            return False

    # ---------- event pump ----------
    def _poll(self):
        while True:
            try:
                kind, arg = self.ui_q.get_nowait()
            except queue.Empty:
                break
            if kind == "connected":
                self._connected(arg)
            else:
                self._connect_failed(arg)
        link = self.link
        if link is not None:
            t_end = time.monotonic() + 0.05
            while time.monotonic() < t_end:
                try:
                    ev = link.events.get_nowait()
                except queue.Empty:
                    break
                try:
                    self._event(ev)
                except Exception as exc:                    # noqa: BLE001
                    self._log(f"! GUI error handling {ev[0]}: {exc}")
                if self.link is not link:
                    break
        self.root.after(30, self._poll)

    def _tick(self):
        # input levels on the GPIO panel, once a second
        if self.link is not None and self.gpio_rows and time.monotonic() > self.gpio_busy_until:
            self.send("!GPIOS")
        self.root.after(1000, self._tick)

    def _event(self, ev):
        kind = ev[0]
        if kind == "line":
            line = ev[1]
            if not line.startswith("$LIVE"):
                self._log(line)
            return
        if kind == "tx":
            if not ev[1].startswith("!GPIOS"):
                self._log("> " + ev[1])
            return
        if kind == "log":
            self._log(ev[1])
        elif kind == "closed":
            self.disconnect(f"Connection closed: {ev[1]}")
        elif kind == "live":
            self.on_live(ev[1], ev[2], ev[3])
        elif kind == "status":
            self.status_dict = ev[1]
            self.update_conn_label()
        elif kind == "gpiolist":
            self.build_gpio_rows(ev[1])
        elif kind == "gpios":
            self.update_gpio_rows(ev[1])
        elif kind == "$READY":
            self.status_dict["fw"] = ev[1][1] if len(ev[1]) > 1 else "?"
            self.update_conn_label()
        elif kind == "$NET":
            self.status_dict["ip"] = ev[1][0]
            self.status_dict["rssi"] = ev[1][2]
            self.update_conn_label()
        elif kind == "$ADC":
            self.status_dict["adc"] = ev[1][0]
            self.update_conn_label()
            if ev[1][0] != "OK":
                self.status_var.set(f"ADS1256 not answering (STATUS register {ev[1][1]}): check the wiring")
        elif kind == "$CAL":
            ok, ms = ev[1][0] == "1", ev[1][1]
            if not self.is_capturing:
                self.status_var.set(f"Self-calibration done ({ms} ms)" if ok else "Self-calibration timed out (no DRDY)")
        elif kind == "$OK":
            self.on_ok(ev[1])
        elif kind == "$ERR":
            msg = ",".join(ev[1])
            # only the capture's own errors end it ("busy: capture running" is a refused setting)
            if self.is_capturing and msg.startswith(("capture", "usage: !CAP", "busy: a capture")):
                self.finish_capture(f"Capture failed: {msg}")
            else:
                self.status_var.set(f"Board: {msg}")
        elif kind == "$CAPARM":
            req = self.cap_request or {}
            self.status_var.set(f"Waiting for {req.get('slope', '').lower()} trigger on {req.get('trig_name', '')}"
                                f" ({req.get('position', '')})")
        elif kind == "$CAPTRIG":
            self.status_var.set(f"Trigger detected after {int(ev[1][0]) / 1000:.2f} s - capturing...")
        elif kind == "$CAPTMO":
            self.finish_capture("Capture aborted" if ev[1][0] == "ABORT" else "Trigger timed out")
        elif kind == "cap_start":
            cap = ev[1]
            self.status_var.set(f"Receiving {cap.frames} frames ...")
        elif kind == "progress":
            got, tot = ev[1], ev[2]
            self.status_var.set(f"Receiving capture: {got}/{tot} frames ({100 * got / max(tot, 1):.0f}%)")
        elif kind == "capture":
            self.on_capture(ev[1])
        elif kind == "$REG":
            self._log("registers: " + " ".join(ev[1]))

    def on_ok(self, f):
        what = f[0] if f else ""
        if what in ("DRATE", "GAIN", "BUF", "CAL"):
            if not self.is_capturing:
                self.status_var.set("Self-calibrating ...")
        elif what == "CAP":
            pass

    def update_conn_label(self):
        d = self.status_dict
        parts = [f"Connected to {self.link.target}" if self.link else "Not connected"]
        if "fw" in d:
            parts.append(f"fw {d['fw']}")
        if "adc" in d:
            parts.append(f"ADS1256 {d['adc']}")
        if d.get("ip") and self.link and not is_network_addr(self.link.target):
            parts.append(f"IP {d['ip']}")
        if d.get("rssi") not in (None, "", "0"):
            parts.append(f"RSSI {d['rssi']} dBm")
        if d.get("conv_s"):
            parts.append(f"{float(d['conv_s']):.0f} conv/s")
        self.conn_var.set("  ·  ".join(parts))

    # ---------- console ----------
    def open_console(self):
        if self.console is not None and self.console.winfo_exists():
            self.console.lift()
            return
        win = tk.Toplevel(self.root)
        win.title("Board console")
        win.geometry("820x420")
        txt = scrolledtext.ScrolledText(win, font=("Courier New", 10), height=20)
        txt.pack(fill="both", expand=True)
        row = ttk.Frame(win)
        row.pack(fill="x")
        entry = ttk.Entry(row, font=("Courier New", 11))
        entry.pack(side="left", fill="x", expand=True, padx=4, pady=4)

        def do_send(_e=None):
            cmd = entry.get().strip()
            if cmd:
                self.send(cmd)
                entry.delete(0, "end")

        entry.bind("<Return>", do_send)
        tk.Button(row, text="Send", command=do_send).pack(side="left", padx=4)
        tk.Button(row, text="!STATUS", command=lambda: self.send("!STATUS")).pack(side="left", padx=2)
        tk.Button(row, text="!REG", command=lambda: self.send("!REG")).pack(side="left", padx=2)
        win.txt = txt
        self.console = win
        entry.focus_set()

    def _log(self, line):
        win = self.console
        if win is None or not win.winfo_exists():
            return
        if line.startswith("$CD"):
            return
        txt = win.txt
        txt.insert("end", line + "\n")
        if int(txt.index("end-1c").split(".")[0]) > 2000:
            txt.delete("1.0", "500.0")
        txt.see("end")

    # ---------- option handlers ----------
    def channel_mask(self):
        return sum(1 << i for i, v in enumerate(self.channel_vars) if v.get())

    def on_channels_changed(self):
        self.send(f"!CH,{self.channel_mask()}")

    def on_buffer(self):
        self.send(f"!BUF,{1 if self.buffer_var.get() else 0}")

    def on_drate(self):
        self.send(f"!DRATE,0x{DRATE_TABLE.get(self.drate_var.get(), 0xF0):02X}")

    def on_gain(self):
        self.pga = GAIN_TABLE.get(self.gain_var.get(), 0)
        self.send(f"!GAIN,{self.pga}")
        self.update_trigger_units()

    def request_calibration(self):
        if self.send("!CAL"):
            self.status_var.set("Self-calibrating ...")

    def on_display_mode(self):
        self.update_trigger_units()
        if self.capture is not None:
            self.update_logger_plot()
            self.update_stats()

    def on_range_toggle(self):
        self.send(f"!RANGE,{1 if self.dmm_range_var.get() else 0}")
        self.update_trigger_units()

    def on_dmm_math_toggle(self):
        self.range_check.configure(state="normal" if self.dmm_math_var.get() else "disabled")
        self.on_display_mode()

    def on_live_values_toggle(self):
        if self.live_values_var.get():
            self.status_var.set("Live readings resumed")
            for i, lbl in enumerate(self.value_labels):
                if self.channel_vars[i].get():
                    lbl.config(text="---")
        else:
            if not self.is_capturing:
                self.status_var.set("Channel readings paused")
            for i, lbl in enumerate(self.value_labels):
                if self.channel_vars[i].get():
                    lbl.config(text="PAUSED")

    def on_live_graph_toggle(self):
        if self.live_graph_var.get():
            self.status_var.set("Live graph enabled")
            self.update_graph()
        elif not self.is_capturing:
            self.status_var.set("Live graph paused")

    def set_options_state(self, enabled):
        st = "readonly" if enabled else "disabled"
        self.drate_menu.config(state=st)
        self.gain_menu.config(state=st)
        self.buffer_check.config(state="normal" if enabled else "disabled")
        self.cal_button.config(state="normal" if enabled else "disabled")

    # ---------- value conversion (as the Pi GUI) ----------
    def compute_dmm_value(self, ch_index, volts):
        if ch_index not in (3, 5):
            return volts, f"{volts:.6f}"
        scaled = float(dmm_formula(ch_index, volts, self.dmm_range_var.get()))
        if math.isnan(scaled) or math.isinf(scaled) or abs(scaled) > 10_000_000:
            return None, "OVER"
        return scaled, f"{scaled:.6f}"

    def compute_sample_value(self, ch_index, raw_value):
        volts = code_to_volts(raw_value, self.pga)
        if self.dmm_math_var.get():
            value, text = self.compute_dmm_value(ch_index, volts)
            units = dmm_units(ch_index) if ch_index in (3, 5) else "V"
            return value, text, units
        if self.raw_var.get():
            return float(raw_value), f"{raw_value:.1f}", "Raw"
        return volts, f"{volts:.6f}", "V"

    def determine_units(self, ch_index):
        if self.dmm_math_var.get():
            return dmm_units(ch_index) if ch_index in (3, 5) else "V"
        if self.raw_var.get():
            return "Raw"
        return "V"

    def update_trigger_units(self, *_):
        idx = self.channel_lookup.get(self.trigger_channel_var.get(), 0)
        self.trigger_units_label.config(text=f"Units: {self.determine_units(idx)}")

    def convert_array(self, ch_index, raw, pga, range_high):
        """Raw codes -> the current display units; NaN for OVER."""
        raw = np.asarray(raw, dtype=float)
        if self.dmm_math_var.get():
            volts = code_to_volts(raw, pga)
            if ch_index in (3, 5):
                return clamp_over(dmm_formula(ch_index, volts, range_high))
            return volts
        if self.raw_var.get():
            return raw
        return code_to_volts(raw, pga)

    def level_to_code(self, ch_index, level):
        """A trigger level in the display units -> (raw code, slope inverted).
        The board compares raw codes; DMM math that falls with voltage (CH6-7)
        turns a rising trigger into a falling one."""
        if self.raw_var.get() and not self.dmm_math_var.get():
            return int(round(level)), False
        if not self.dmm_math_var.get() or ch_index not in (3, 5):
            return int(round(volts_to_code(level, self.pga))), False
        fs = 2.0 * VREF / (1 << self.pga)
        rng = self.dmm_range_var.get()

        def f(v):
            return float(dmm_formula(ch_index, v, rng))

        lo, hi = -fs, fs
        flo, fhi = f(lo), f(hi)
        rising = fhi > flo
        if not (min(flo, fhi) <= level <= max(flo, fhi)):
            raise ValueError(f"level is outside this channel's range ({min(flo, fhi):.6g} .. {max(flo, fhi):.6g})")
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            if (f(mid) < level) == rising:
                lo = mid
            else:
                hi = mid
        return int(round(volts_to_code(0.5 * (lo + hi), self.pga))), not rising

    # ---------- live readings ----------
    def on_live(self, frames, period_ms, chans):
        if self.is_capturing:
            return
        readings = self.live_values_var.get()
        for i in range(len(CHANNEL_MAP)):
            if not self.channel_vars[i].get():
                self.value_labels[i].config(text="OFF")
                self.unit_labels[i].config(text="")
                self.sd_labels[i].config(text="")
                self.data_buffers[i].append(float("nan"))
                continue
            c = chans.get(i)
            if not readings:
                if self.value_labels[i].cget("text") != "PAUSED":
                    self.value_labels[i].config(text="PAUSED")
                self.data_buffers[i].append(float("nan"))
                continue
            if c is None:
                self.data_buffers[i].append(float("nan"))
                continue
            value, text, units = self.compute_sample_value(i, c["mean"])
            if c["clip"]:
                text = "CLIP " + text if value is not None else text
            self.value_labels[i].config(text=text)
            self.unit_labels[i].config(text=units)
            self.channel_units[i] = units
            # noise, in the same units: the conversion's slope times sd
            if value is not None and c["sd"] > 0:
                v2, _, _ = self.compute_sample_value(i, c["mean"] + c["sd"])
                sd = abs(v2 - value) if v2 is not None else float("nan")
                self.sd_labels[i].config(text=f"σ {sd:.3g}")
            else:
                self.sd_labels[i].config(text="")
            self.data_buffers[i].append(float("nan") if value is None else value)
        if self.live_graph_var.get():
            self.update_graph()
        if frames and period_ms > 0:
            self.status_dict["conv_s"] = f"{frames * len(chans) * 1000.0 / period_ms:.0f}"
            self.update_conn_label()

    def update_graph(self):
        ax = self.ax
        ax.clear()
        ax.set_title("Live Channel Data", fontsize=13)
        ax.set_xlabel("Samples")
        ax.set_ylabel("DMM Value" if self.dmm_math_var.get() else "Raw" if self.raw_var.get() else "Volts")
        for i, (label_txt, _, _) in enumerate(CHANNEL_MAP):
            if not self.channel_vars[i].get():
                continue
            y = list(self.data_buffers[i])
            if y:
                ax.plot(range(len(y)), y, label=label_txt)
        if ax.has_data():
            ax.legend(loc="upper right", fontsize=8)
        self.fig.tight_layout()
        self.canvas.draw_idle()

    # ---------- GPIO ----------
    def build_gpio_rows(self, pins):
        if self.gpio_rows and sorted(self.gpio_rows) == sorted(pins):
            return
        for w in self.gpio_frame.winfo_children()[1:]:
            w.destroy()
        self.gpio_rows = {}
        cols = 3
        for k, pin in enumerate(pins):
            r, c = 1 + k // cols, (k % cols) * 3
            ttk.Label(self.gpio_frame, text=f"D{pin}", font=CHANNEL_FONT).grid(row=r, column=c, sticky="w", padx=(4, 2), pady=1)
            var = tk.StringVar(value="Input")
            combo = ttk.Combobox(self.gpio_frame, textvariable=var, values=GPIO_MODES, state="readonly", width=11)
            combo.grid(row=r, column=c + 1, sticky="ew", padx=2, pady=1)
            combo.bind("<<ComboboxSelected>>", lambda _e, p=pin: self.on_gpio_mode_change(p))
            lv = tk.Label(self.gpio_frame, text="-", font=("Courier New", 11, "bold"), width=2)
            lv.grid(row=r, column=c + 2, sticky="w", padx=(0, 4))
            self.gpio_rows[pin] = (var, combo, lv)

    def on_gpio_mode_change(self, pin):
        var, _, _ = self.gpio_rows[pin]
        mode = GPIO_MODES.index(var.get())
        self.gpio_busy_until = time.monotonic() + 1.0
        if not self.send(f"!GPIO,{pin},{mode}"):
            self.status_var.set(f"Failed to configure D{pin}: not connected")

    def update_gpio_rows(self, states):
        for pin, mode, level in states:
            row = self.gpio_rows.get(pin)
            if row is None:
                continue
            var, _, lv = row
            if 0 <= mode < len(GPIO_MODES) and var.get() != GPIO_MODES[mode]:
                var.set(GPIO_MODES[mode])
            lv.config(text=str(level), fg="#00A000" if level else "#888888")

    # ---------- capture ----------
    def perform_capture(self, armed):
        if self.is_capturing:
            return
        if self.link is None:
            self.status_var.set("Not connected")
            return
        try:
            n_points = int(self.points_var.get())
            if n_points <= 0:
                raise ValueError
        except ValueError:
            self.status_var.set("Invalid point count")
            return
        try:
            threshold = float(self.trigger_level_var.get())
        except ValueError:
            if armed:
                self.status_var.set("Invalid trigger level")
                return
            threshold = 0.0
        try:
            timeout_s = float(self.timeout_var.get() or 0)
        except ValueError:
            self.status_var.set("Invalid timeout")
            return
        trig_name = self.trigger_channel_var.get()
        trig = self.channel_lookup.get(trig_name, 0)
        selected = [i for i, v in enumerate(self.channel_vars) if v.get()]
        if not selected:
            self.status_var.set("Select at least one channel")
            return
        mode = self.trigger_mode_var.get()
        slope = self.trigger_slope_var.get()
        position = self.trigger_position_var.get()
        code, invert = 0, False
        if armed:
            try:
                code, invert = self.level_to_code(trig, threshold)
            except ValueError as exc:
                self.status_var.set(f"Trigger level: {exc}")
                return
        slope_bit = TRIGGER_SLOPES.index(slope) ^ (1 if invert else 0)
        mode_bit = 0 if mode == "Benchmark" else 1
        mask = sum(1 << i for i in selected)
        cmd = (f"!CAP,{mode_bit},{n_points},{mask},{trig},{1 if armed else 0},{code},{slope_bit},"
               f"{TRIGGER_POSITIONS.index(position)},{int(timeout_s * 1000)}")
        self.cap_request = dict(mode=mode, trig=trig, trig_name=trig_name, selected=selected, slope=slope,
                                position=position, armed=armed, range_high=self.dmm_range_var.get(),
                                pga=self.pga, t0=time.monotonic())
        self.is_capturing = True
        self.arm_button.config(state="disabled")
        self.capture_button.config(state="disabled")
        self.export_button.config(state="disabled")
        self.abort_button.config(state="normal")
        self.set_options_state(False)
        self.status_var.set("Awaiting trigger..." if armed else "Capturing now...")
        if not self.send(cmd):
            self.finish_capture("Not connected")

    def abort_capture(self):
        if self.is_capturing:
            self.send("!ABORT")
            self.status_var.set("Aborting ...")

    def finish_capture(self, message):
        self.is_capturing = False
        self.arm_button.config(state="normal")
        self.capture_button.config(state="normal")
        self.export_button.config(state="normal")
        self.abort_button.config(state="disabled")
        self.set_options_state(True)
        self.status_var.set(message)

    def on_capture(self, cap):
        req = self.cap_request or {}
        cap.mode = req.get("mode", "Benchmark" if len(cap.order) == 1 else "Multi")
        cap.range_high = req.get("range_high", True)
        if cap.mode == "Multi":
            sel = set(req.get("selected", cap.order))
            cap.keep = [ch for ch in cap.order if ch in sel] or list(cap.order)
        else:
            cap.keep = list(cap.order)
        self.capture = cap
        self.update_logger_plot()
        self.update_stats()
        took = time.monotonic() - req.get("t0", time.monotonic())
        rate = 1e6 / cap.dt_us if cap.dt_us > 0 else 0
        msg = f"Capture complete ({cap.got} samples, {rate:,.1f} frames/s, {took:.1f} s)"
        if cap.gaps:
            msg += f"  -  {cap.gaps} conversions lost (board couldn't keep up: try a lower rate)"
        if cap.got < cap.frames or cap.bad_lines:
            msg += f"  -  {cap.frames - cap.got} frames missing in transfer"
        if cap.flags & 2:
            msg += "  -  input at full scale"
        self.finish_capture(msg)

    def capture_columns(self):
        """[(channel index, values in display units)] for the shown columns."""
        cap = self.capture
        out = []
        for ch in cap.keep:
            col = cap.order.index(ch)
            out.append((ch, self.convert_array(ch, cap.raw[:, col], cap.pga, cap.range_high)))
        return out

    def update_logger_plot(self):
        cap = self.capture
        ax = self.log_ax
        ax.clear()
        if cap is None:
            self.log_canvas.draw_idle()
            return
        ax.set_title(f"{cap.mode} capture", fontsize=12)
        x = cap.t_ms if np.isfinite(cap.t_ms).any() else np.arange(cap.frames)
        ax.set_xlabel("Time (ms)")
        ax.set_ylabel("DMM Value" if self.dmm_math_var.get() else "Raw" if self.raw_var.get() else "Volts")
        for ch, y in self.capture_columns():
            ax.plot(x, y, label=CHANNEL_MAP[ch][0])
        if 0 <= cap.trig_idx < cap.frames and np.isfinite(x[cap.trig_idx]):
            ax.axvline(x[cap.trig_idx], color="#888888", linestyle="--", linewidth=1)
        if ax.has_data():
            ax.legend(loc="upper right", fontsize=8)
        self.log_fig.tight_layout()
        self.log_canvas.draw_idle()

    def update_stats(self):
        for w in self.stats_frame.winfo_children():
            w.destroy()
        cap = self.capture
        if cap is None:
            return
        for col, (ch, y) in enumerate(self.capture_columns()):
            valid = y[np.isfinite(y)]
            unit = self.determine_units(ch)
            if valid.size == 0:
                summary = "No valid samples"
            else:
                summary = (f"Min: {valid.min():.6f}\nMax: {valid.max():.6f}\n"
                           f"Avg: {valid.mean():.6f}\nStd: {valid.std():.6f}")
            ttk.Label(self.stats_frame, text=f"{CHANNEL_MAP[ch][0]} ({unit})\n{summary}", font=CHANNEL_FONT,
                      anchor="w", justify="left").grid(row=0, column=col, sticky="nw", padx=(0, 24))

    def export_csv(self):
        cap = self.capture
        if cap is None:
            self.status_var.set("No capture data to export")
            return
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = filedialog.asksaveasfilename(defaultextension=".csv", initialfile=f"ads1256_log_{ts}.csv",
                                             filetypes=[("CSV files", "*.csv")])
        if not fname:
            return
        cols = self.capture_columns()
        with open(fname, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["Time_ms"] + [CHANNEL_MAP[ch][0] for ch, _ in cols])
            for i in range(cap.frames):
                t = cap.t_ms[i]
                row = [f"{t:.7g}" if np.isfinite(t) else i]
                for _, y in cols:
                    v = y[i]
                    row.append("OVER" if not np.isfinite(v) else f"{v:.7g}")
                w.writerow(row)
        self.status_var.set(f"Exported {cap.frames} samples to CSV")

    def on_close(self):
        self._save_settings()
        if self.link is not None:
            self.disconnect("Closing")
        self.root.destroy()


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
