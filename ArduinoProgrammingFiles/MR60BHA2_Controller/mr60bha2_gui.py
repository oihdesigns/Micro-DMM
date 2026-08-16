#!/usr/bin/env python3
"""
mr60bha2_gui.py — host control panel for MR60BHA2_Controller.ino

Full control surface for a Seeed XIAO 60GHz mmWave Human Breathing and Heartbeat
Sensor (MR60BHA2) driven by the XIAO ESP32-C6 that ships plugged into it.

Tabs
  Live        rolling phase + rate traces, presence/flag readouts, CSV logging
  Controls    every lever: acquisition, validity gating, filtering, LED, capture
  Capture     timed capture of the raw phase waveform, with an FFT that re-derives
              breathing and heart rate independently of the module's own numbers
  Protocol    per-report-type frame statistics, raw frame monitor, manual frame
              transmission and a type-range probe

Requires: pyserial, matplotlib, numpy
    pip install pyserial matplotlib numpy
"""

import csv
import os
import queue
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import filedialog, messagebox, ttk

import numpy as np
import serial
import serial.tools.list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

BAUD = 115200

# ── $R field layout — must match sendReading() in the sketch ────────────────
R_FIELDS = ["t_ms", "present", "rflag", "dist", "br", "hr",
            "total", "breath", "heart", "br_raw", "hr_raw", "dist_raw",
            "human", "ntgt", "age_ms"]

# ── Report types — must match kTypes[] in the sketch ────────────────────────
# (repmask bit, key, hex type, human label)
REPORT_TYPES = [
    (0, "phase",  0x0A13, "phase (total/breath/heart)"),
    (1, "breath", 0x0A14, "breath rate"),
    (2, "heart",  0x0A15, "heart rate"),
    (3, "dist",   0x0A16, "range flag + distance"),
    (4, "human",  0x0F09, "human presence"),
    (5, "pcdet",  0x0A08, "point cloud detections"),
    (6, "pctgt",  0x0A04, "tracked targets"),
    (7, "fwver",  0xFFFF, "firmware version"),
]

LED_MODES = ["0 off", "1 presence", "2 breath phase", "3 heart pulse", "4 frame activity"]
RAW_MODES = ["0 off", "1 unknown types only", "2 every frame"]

# Capture columns, in the order the sketch's $CH header names them.
CAP_COLS = ["total", "breath", "heart", "dist", "br", "hr"]
CAP_COLOR = {"total": "#444444", "breath": "#1f77b4", "heart": "#d62728",
             "dist": "#2ca02c", "br": "#9467bd", "hr": "#ff7f0e"}

# Search bands for the FFT rate estimate, in cycles per minute.
BAND = {"breath": (6.0, 40.0), "heart": (40.0, 150.0)}

LIVE_WINDOW_S = 30.0


def _f(s):
    """Parse a float field; the firmware prints nan for 'never received'."""
    try:
        return float(s)
    except (TypeError, ValueError):
        return float("nan")


class SerialManager:
    """Background reader thread; hands complete lines to a queue."""

    def __init__(self, line_queue):
        self.q = line_queue
        self.ser = None
        self._stop = threading.Event()
        self._thread = None

    def connect(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.2)
        self._stop.clear()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def disconnect(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self.ser:
            try:
                self.ser.close()
            except OSError:
                pass
            self.ser = None

    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def send(self, text):
        if not self.is_open():
            return False
        self.ser.write((text + "\n").encode("ascii", "ignore"))
        return True

    def _read_loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(4096)
            except (OSError, serial.SerialException):
                self.q.put(("__closed__", None))
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.q.put(("line", line.decode("ascii", "replace").strip()))


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("MR60BHA2 Controller")
        self.geometry("1200x860")

        self.q = queue.Queue()
        self.sm = SerialManager(self.q)
        # Tk is not thread-safe; worker threads hand callables here and the
        # main-thread pump runs them.
        self._call_q = queue.Queue()

        # Device state mirrored from $CFG / $R
        self.cfg = {}            # key -> int value
        self.limits = {}         # key -> (min, max)
        self.vars = {}           # config key -> StringVar for the plain spin rows
        self._suppress_cfg_push = False
        self._cfg_event = threading.Event()

        self.last_r = {}         # field name -> value from the newest $R

        # Rolling live traces: parallel lists, trimmed by time
        self.tr_t = []
        self.tr = {k: [] for k in ("total", "breath", "heart", "br", "hr", "dist")}
        self._live_dirty = False
        self._t0 = None

        # Capture reception state
        self._cap_active = False
        self._cap_header = None
        self._cap_names = []
        self._cap_rows = []
        self.capture = None
        self._cap_event = threading.Event()

        # CSV logging of the live stream
        self._log_path = None
        self._log_file = None
        self._log_writer = None
        self._log_rows = 0

        # Protocol tab state
        self._stat_rows = {}
        self._raw_paused = False
        self._probe_rows = []

        self._build_ui()
        self.after(40, self._pump)
        self.after(200, self._live_redraw_tick)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ────────────────────────────────────────────────────────────── UI build

    def _build_ui(self):
        top = ttk.Frame(self, padding=6)
        top.pack(fill="x")

        ttk.Label(top, text="Port:").pack(side="left")
        self.port_cb = ttk.Combobox(top, width=22, values=self._ports())
        self.port_cb.pack(side="left", padx=4)
        if self.port_cb["values"]:
            self.port_cb.current(0)
        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side="left")
        self.connect_btn = ttk.Button(top, text="Connect", command=self._toggle_connect)
        self.connect_btn.pack(side="left", padx=6)
        self.conn_lbl = ttk.Label(top, text="disconnected", foreground="#a00")
        self.conn_lbl.pack(side="left", padx=6)

        ttk.Separator(top, orient="vertical").pack(side="left", fill="y", padx=8)
        self.fw_lbl = ttk.Label(top, text="fw: --", font=("Consolas", 10))
        self.fw_lbl.pack(side="left", padx=4)
        self.mod_lbl = ttk.Label(top, text="module: --", font=("Consolas", 10))
        self.mod_lbl.pack(side="left", padx=8)
        self.present_lbl = ttk.Label(top, text="target: --",
                                     font=("Consolas", 11, "bold"))
        self.present_lbl.pack(side="left", padx=8)

        self.nb = ttk.Notebook(self)
        self.nb.pack(fill="both", expand=True, padx=6, pady=4)
        self.live_tab = ttk.Frame(self.nb)
        self.ctrl_tab = ttk.Frame(self.nb)
        self.cap_tab = ttk.Frame(self.nb)
        self.proto_tab = ttk.Frame(self.nb)
        self.nb.add(self.live_tab, text="Live")
        self.nb.add(self.ctrl_tab, text="Controls")
        self.nb.add(self.cap_tab, text="Capture")
        self.nb.add(self.proto_tab, text="Protocol")

        self._build_live_tab()
        self._build_ctrl_tab()
        self._build_cap_tab()
        self._build_proto_tab()

        logf = ttk.LabelFrame(self, text="Serial log", padding=4)
        logf.pack(fill="x", padx=6, pady=(0, 6))
        self.log = tk.Text(logf, height=6, font=("Consolas", 9), wrap="none")
        self.log.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(logf, command=self.log.yview)
        sb.pack(side="right", fill="y")
        self.log.config(yscrollcommand=sb.set)

        cmdf = ttk.Frame(self, padding=(6, 0, 6, 6))
        cmdf.pack(fill="x")
        ttk.Label(cmdf, text="Manual:").pack(side="left")
        self.cmd_var = tk.StringVar()
        e = ttk.Entry(cmdf, textvariable=self.cmd_var, width=40)
        e.pack(side="left", padx=4)
        e.bind("<Return>", lambda _ev: self._send_manual())
        ttk.Button(cmdf, text="Send", command=self._send_manual).pack(side="left")
        self.quiet_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(cmdf, text="hide $R / $F in log",
                        variable=self.quiet_var).pack(side="left", padx=12)

    # ── Live tab ────────────────────────────────────────────────────────────

    def _build_live_tab(self):
        ctl = ttk.Frame(self.live_tab, padding=6)
        ctl.pack(fill="x")

        ttk.Button(ctl, text="Single Read", command=lambda: self._send("!READ")).pack(side="left")
        self.stream_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(ctl, text="Stream", variable=self.stream_var,
                        command=self._on_stream).pack(side="left", padx=10)

        ttk.Label(ctl, text="mode:").pack(side="left")
        self.strmode_var = tk.IntVar(value=0)
        ttk.Radiobutton(ctl, text="timer", value=0, variable=self.strmode_var,
                        command=lambda: self._set_key("strmode", 0)).pack(side="left")
        ttk.Radiobutton(ctl, text="per phase frame", value=1, variable=self.strmode_var,
                        command=lambda: self._set_key("strmode", 1)).pack(side="left")

        ttk.Label(ctl, text="interval ms:").pack(side="left", padx=(10, 2))
        self.streamms_var = tk.StringVar(value="200")
        ttk.Entry(ctl, width=7, textvariable=self.streamms_var).pack(side="left")
        ttk.Button(ctl, text="Set",
                   command=lambda: self._set_key("streamms", self.streamms_var.get())
                   ).pack(side="left", padx=2)

        ttk.Separator(ctl, orient="vertical").pack(side="left", fill="y", padx=10)
        ttk.Button(ctl, text="Reset filters (!ZERO)",
                   command=lambda: self._send("!ZERO")).pack(side="left")
        ttk.Button(ctl, text="Clear plot", command=self._clear_traces).pack(side="left", padx=6)
        ttk.Label(ctl, text="window s:").pack(side="left")
        self.window_var = tk.StringVar(value=str(int(LIVE_WINDOW_S)))
        ttk.Entry(ctl, width=6, textvariable=self.window_var).pack(side="left", padx=2)

        # ── numeric readouts ──
        rd = ttk.Frame(self.live_tab, padding=(8, 2))
        rd.pack(fill="x")
        self.big = {}
        for key, label, unit in (("br", "Breathing", "brpm"),
                                 ("hr", "Heart", "bpm"),
                                 ("dist", "Distance", "m")):
            f = ttk.LabelFrame(rd, text=f"{label} ({unit})", padding=4)
            f.pack(side="left", padx=6)
            lbl = ttk.Label(f, text="--", font=("Consolas", 26, "bold"),
                            foreground="#046", width=7, anchor="center")
            lbl.pack()
            self.big[key] = lbl

        stat = ttk.LabelFrame(rd, text="Module state", padding=4)
        stat.pack(side="left", fill="both", expand=True, padx=6)
        self.state_lbl = ttk.Label(stat, text="no data", font=("Consolas", 10),
                                   justify="left")
        self.state_lbl.pack(anchor="w")

        # ── CSV log of the live stream ──
        lf = ttk.LabelFrame(self.live_tab, text="CSV log", padding=6)
        lf.pack(fill="x", padx=6)
        self.logfile_var = tk.StringVar(value="")
        ttk.Button(lf, text="File...", command=self._choose_log_file).pack(side="left")
        ttk.Entry(lf, textvariable=self.logfile_var, width=52).pack(side="left", padx=4)
        self.logbtn = ttk.Button(lf, text="Start logging", command=self._toggle_logging)
        self.logbtn.pack(side="left", padx=6)
        self.logstat_lbl = ttk.Label(lf, text="idle", font=("Consolas", 10))
        self.logstat_lbl.pack(side="left", padx=8)
        ttk.Label(lf, text="(one row per $R — turn Stream on to fill it)").pack(side="left")

        # ── plots ──
        body = ttk.Frame(self.live_tab)
        body.pack(fill="both", expand=True)
        self.live_fig = Figure(figsize=(9.0, 4.6), dpi=100)
        self.ax_phase = self.live_fig.add_subplot(211)
        self.ax_rate = self.live_fig.add_subplot(212, sharex=self.ax_phase)
        self.live_canvas = FigureCanvasTkAgg(self.live_fig, master=body)
        self.live_canvas.get_tk_widget().pack(side="left", fill="both", expand=True,
                                              padx=6, pady=6)

        side = ttk.Frame(body)
        side.pack(side="right", fill="y", padx=(0, 8), pady=6)
        pf = ttk.LabelFrame(side, text="Phase traces", padding=4)
        pf.pack(fill="x")
        self.trace_vars = {}
        for name, default in (("total", False), ("breath", True), ("heart", True)):
            v = tk.BooleanVar(value=default)
            self.trace_vars[name] = v
            ttk.Checkbutton(pf, text=name, variable=v,
                            command=self._mark_live_dirty).pack(anchor="w")
        rf = ttk.LabelFrame(side, text="Rate traces", padding=4)
        rf.pack(fill="x", pady=(8, 0))
        for name, default in (("br", True), ("hr", True), ("dist", False)):
            v = tk.BooleanVar(value=default)
            self.trace_vars[name] = v
            ttk.Checkbutton(rf, text=name, variable=v,
                            command=self._mark_live_dirty).pack(anchor="w")

        self._draw_live()

    # ── Controls tab ────────────────────────────────────────────────────────

    def _build_ctrl_tab(self):
        outer = ttk.Frame(self.ctrl_tab, padding=6)
        outer.pack(fill="both", expand=True)
        left = ttk.Frame(outer)
        left.pack(side="left", fill="y", padx=(0, 10))
        mid = ttk.Frame(outer)
        mid.pack(side="left", fill="y", padx=(0, 10))
        right = ttk.Frame(outer)
        right.pack(side="left", fill="both", expand=True)

        # ── Acquisition ──
        acq = ttk.LabelFrame(left, text="Acquisition", padding=6)
        acq.pack(fill="x", pady=(0, 8))
        self._spin_row(acq, 0, "pump ms:", "pumpms", 0, 200)
        ttk.Label(acq, text="0 = non-blocking drain.\nHigher values block the loop\n"
                            "for that long every pass —\nsweep it to see the cost.",
                  foreground="#555", font=("Segoe UI", 8)).grid(row=1, column=0,
                                                                columnspan=2,
                                                                sticky="w", pady=(0, 4))
        self._spin_row(acq, 2, "stream ms:", "streamms", 20, 60000)

        # ── Validity gating ──
        gat = ttk.LabelFrame(left, text="Validity gating", padding=6)
        gat.pack(fill="x", pady=(0, 8))
        self._spin_row(gat, 0, "dist min mm:", "dminmm", 0, 10000)
        self._spin_row(gat, 1, "dist max mm:", "dmaxmm", 0, 10000)
        self._spin_row(gat, 2, "breath min:", "brmin", 0, 100)
        self._spin_row(gat, 3, "breath max:", "brmax", 0, 100)
        self._spin_row(gat, 4, "heart min:", "hrmin", 0, 250)
        self._spin_row(gat, 5, "heart max:", "hrmax", 0, 250)
        self._spin_row(gat, 6, "presence hold ms:", "holdms", 0, 60000)
        ttk.Label(gat, text="Readings outside a window are dropped;\n"
                            "the *_raw fields still show them.",
                  foreground="#555", font=("Segoe UI", 8)).grid(row=7, column=0,
                                                                columnspan=2, sticky="w")

        # ── Filtering ──
        flt = ttk.LabelFrame(left, text="Filtering", padding=6)
        flt.pack(fill="x")
        self._spin_row(flt, 0, "median N:", "medn", 1, 9)
        self._spin_row(flt, 1, "EMA breath /1000:", "emabr", 1, 1000)
        self._spin_row(flt, 2, "EMA heart /1000:", "emahr", 1, 1000)
        self._spin_row(flt, 3, "EMA dist /1000:", "emad", 1, 1000)
        ttk.Label(flt, text="1000 = passthrough. The module\nalready smooths hard, so this is\n"
                            "mostly for making it worse on purpose.",
                  foreground="#555", font=("Segoe UI", 8)).grid(row=4, column=0,
                                                                columnspan=2, sticky="w")

        # ── Indicator ──
        led = ttk.LabelFrame(mid, text="Indicator LED (D1)", padding=6)
        led.pack(fill="x", pady=(0, 8))
        ttk.Label(led, text="mode:").grid(row=0, column=0, sticky="e")
        self.ledmode_var = tk.StringVar(value=LED_MODES[1])
        cb = ttk.Combobox(led, width=18, state="readonly", values=LED_MODES,
                          textvariable=self.ledmode_var)
        cb.grid(row=0, column=1, sticky="w", padx=4)
        cb.bind("<<ComboboxSelected>>",
                lambda _e: self._set_key("ledmode", LED_MODES.index(self.ledmode_var.get())))
        self._spin_row(led, 1, "brightness:", "ledbri", 0, 255)

        # ── Capture ──
        capf = ttk.LabelFrame(mid, text="Capture", padding=6)
        capf.pack(fill="x", pady=(0, 8))
        self._spin_row(capf, 0, "window ms:", "capms", 100, 60000)
        ttk.Label(capf, text="mode:").grid(row=1, column=0, sticky="e")
        self.capmode_var = tk.IntVar(value=0)
        mf = ttk.Frame(capf)
        mf.grid(row=1, column=1, sticky="w")
        ttk.Radiobutton(mf, text="per frame", value=0, variable=self.capmode_var,
                        command=lambda: self._set_key("capmode", 0)).pack(side="left")
        ttk.Radiobutton(mf, text="grid", value=1, variable=self.capmode_var,
                        command=lambda: self._set_key("capmode", 1)).pack(side="left")
        self._spin_row(capf, 2, "grid dt ms:", "capdtms", 5, 5000)

        # ── Protocol switches ──
        pf = ttk.LabelFrame(mid, text="Protocol", padding=6)
        pf.pack(fill="x", pady=(0, 8))
        self.txck_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(pf, text="append checksum to empty frames",
                        variable=self.txck_var,
                        command=lambda: self._set_key("txck", int(self.txck_var.get()))
                        ).pack(anchor="w")
        self.unsafe_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(pf, text="allow unknown frame types (unsafe)",
                        variable=self.unsafe_var, command=self._on_unsafe).pack(anchor="w")
        self._spin_row_packed(pf, "module reset pin (255 = none):", "rstpin", 0, 255)
        ttk.Button(pf, text="Pulse reset (!RESET)",
                   command=lambda: self._send("!RESET")).pack(fill="x", pady=(4, 0))

        # ── Device config buttons ──
        dev = ttk.LabelFrame(mid, text="Device config", padding=6)
        dev.pack(fill="x")
        ttk.Button(dev, text="Re-read from device",
                   command=lambda: self._send("!CFG")).pack(fill="x", pady=1)
        ttk.Button(dev, text="Save to NVS", command=lambda: self._send("!SAVE")).pack(fill="x", pady=1)
        ttk.Button(dev, text="Load from NVS", command=self._load_nvs).pack(fill="x", pady=1)
        ttk.Button(dev, text="Restore defaults", command=self._restore_defaults).pack(fill="x", pady=1)
        ttk.Button(dev, text="Module firmware (!ID)",
                   command=lambda: self._send("!ID")).pack(fill="x", pady=1)

        # ── Report mask ──
        rep = ttk.LabelFrame(right, text="Report types processed (repmask)", padding=6)
        rep.pack(fill="x")
        self.rep_vars = {}
        for row, (bit, key, tid, label) in enumerate(REPORT_TYPES):
            v = tk.BooleanVar(value=True)
            self.rep_vars[bit] = v
            ttk.Checkbutton(rep, text=f"0x{tid:04X}  {key:<7s} {label}", variable=v,
                            command=self._push_repmask).grid(row=row, column=0, sticky="w")
        bf = ttk.Frame(rep)
        bf.grid(row=len(REPORT_TYPES), column=0, sticky="w", pady=(6, 0))
        ttk.Button(bf, text="All", command=lambda: self._rep_preset(True)).pack(side="left", padx=2)
        ttk.Button(bf, text="None", command=lambda: self._rep_preset(False)).pack(side="left", padx=2)
        self.repmask_lbl = ttk.Label(bf, text="repmask 0xFF", font=("Consolas", 10))
        self.repmask_lbl.pack(side="left", padx=12)
        ttk.Label(rep, text="Unticking a type stops it being interpreted — it is still counted\n"
                            "in the Protocol tab's statistics, so you can tell 'filtered' from 'absent'.",
                  foreground="#555", font=("Segoe UI", 8)).grid(
                      row=len(REPORT_TYPES) + 1, column=0, sticky="w", pady=(4, 0))

        # ── Raw config dump ──
        rawf = ttk.LabelFrame(right, text="Device config (as reported)", padding=6)
        rawf.pack(fill="both", expand=True, pady=(8, 0))
        self.cfg_tree = ttk.Treeview(rawf, columns=("val", "min", "max"),
                                     show="tree headings", height=16)
        self.cfg_tree.heading("#0", text="key")
        self.cfg_tree.heading("val", text="value")
        self.cfg_tree.heading("min", text="min")
        self.cfg_tree.heading("max", text="max")
        self.cfg_tree.column("#0", width=150)
        for c in ("val", "min", "max"):
            self.cfg_tree.column(c, width=90, anchor="e")
        self.cfg_tree.pack(fill="both", expand=True)

    def _spin_row(self, parent, row, label, key, lo, hi):
        var = tk.StringVar(value="0")
        self.vars[key] = var
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="e", pady=2)
        sp = ttk.Spinbox(parent, from_=lo, to=hi, width=10, textvariable=var)
        sp.grid(row=row, column=1, sticky="w", padx=4)
        sp.configure(command=lambda: self._set_key(key, var.get()))
        sp.bind("<Return>", lambda _e: self._set_key(key, var.get()))
        sp.bind("<FocusOut>", lambda _e: self._set_key(key, var.get()))
        return sp

    def _spin_row_packed(self, parent, label, key, lo, hi):
        f = ttk.Frame(parent)
        f.pack(fill="x", pady=2)
        var = tk.StringVar(value="0")
        self.vars[key] = var
        ttk.Label(f, text=label).pack(side="left")
        sp = ttk.Spinbox(f, from_=lo, to=hi, width=6, textvariable=var)
        sp.pack(side="left", padx=4)
        sp.configure(command=lambda: self._set_key(key, var.get()))
        sp.bind("<Return>", lambda _e: self._set_key(key, var.get()))
        sp.bind("<FocusOut>", lambda _e: self._set_key(key, var.get()))

    # ── Capture tab ─────────────────────────────────────────────────────────

    def _build_cap_tab(self):
        ctl = ttk.LabelFrame(self.cap_tab, text="Capture", padding=6)
        ctl.pack(fill="x", padx=6, pady=6)

        ttk.Label(ctl, text="Window ms:").pack(side="left")
        self.capms_var = tk.StringVar(value="20000")
        ttk.Entry(ctl, width=8, textvariable=self.capms_var).pack(side="left", padx=4)

        ttk.Label(ctl, text="Mode:").pack(side="left", padx=(10, 2))
        self.capmode2_var = tk.IntVar(value=0)
        ttk.Radiobutton(ctl, text="one sample per phase frame", value=0,
                        variable=self.capmode2_var,
                        command=lambda: self._set_key("capmode", 0)).pack(side="left")
        ttk.Radiobutton(ctl, text="uniform grid", value=1,
                        variable=self.capmode2_var,
                        command=lambda: self._set_key("capmode", 1)).pack(side="left")

        self.cap_btn = ttk.Button(ctl, text="Run Capture", command=self._run_capture)
        self.cap_btn.pack(side="left", padx=12)
        ttk.Button(ctl, text="Abort", command=lambda: self._send("!ABORT")).pack(side="left")
        self.savecsv_btn = ttk.Button(ctl, text="Save CSV", command=self._save_capture_csv,
                                      state="disabled")
        self.savecsv_btn.pack(side="left", padx=6)

        self.cap_result = ttk.Label(self.cap_tab, text="no capture yet",
                                    font=("Consolas", 10, "bold"), foreground="#046")
        self.cap_result.pack(anchor="w", padx=12)

        anaf = ttk.LabelFrame(self.cap_tab, text="Rate from the waveform", padding=6)
        anaf.pack(fill="x", padx=6, pady=4)
        ttk.Label(anaf, text="Signal:").pack(side="left")
        self.fft_src = tk.StringVar(value="breath")
        cb = ttk.Combobox(anaf, width=10, state="readonly", textvariable=self.fft_src,
                          values=["breath", "heart", "total"])
        cb.pack(side="left", padx=4)
        cb.bind("<<ComboboxSelected>>", lambda _e: self._draw_capture())
        ttk.Label(anaf, text="Band:").pack(side="left", padx=(10, 2))
        self.fft_band = tk.StringVar(value="breath")
        cb2 = ttk.Combobox(anaf, width=10, state="readonly", textvariable=self.fft_band,
                           values=["breath", "heart"])
        cb2.pack(side="left", padx=4)
        cb2.bind("<<ComboboxSelected>>", lambda _e: self._draw_capture())
        self.detrend_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(anaf, text="detrend", variable=self.detrend_var,
                        command=self._draw_capture).pack(side="left", padx=10)
        self.fft_lbl = ttk.Label(anaf, text="--", font=("Consolas", 11, "bold"),
                                 foreground="#804")
        self.fft_lbl.pack(side="left", padx=12)

        body = ttk.Frame(self.cap_tab)
        body.pack(fill="both", expand=True, padx=6, pady=4)

        self.cap_fig = Figure(figsize=(9.0, 5.0), dpi=100)
        self.ax_cap = self.cap_fig.add_subplot(211)
        self.ax_fft = self.cap_fig.add_subplot(212)
        self.cap_canvas = FigureCanvasTkAgg(self.cap_fig, master=body)
        self.cap_canvas.get_tk_widget().pack(side="left", fill="both", expand=True)

        side = ttk.Frame(body)
        side.pack(side="right", fill="y", padx=(8, 0))
        plotf = ttk.LabelFrame(side, text="Plot", padding=4)
        plotf.pack(fill="x")
        self.cap_plot_vars = {}
        for name in CAP_COLS:
            v = tk.BooleanVar(value=name in ("breath", "heart"))
            self.cap_plot_vars[name] = v
            ttk.Checkbutton(plotf, text=name, variable=v,
                            command=self._draw_capture).pack(anchor="w")

    # ── Protocol tab ────────────────────────────────────────────────────────

    def _build_proto_tab(self):
        outer = ttk.Frame(self.proto_tab, padding=6)
        outer.pack(fill="both", expand=True)
        left = ttk.Frame(outer)
        left.pack(side="left", fill="both", expand=True, padx=(0, 8))
        right = ttk.Frame(outer)
        right.pack(side="left", fill="both", expand=True)

        # ── Frame statistics ──
        sf = ttk.LabelFrame(left, text="Frame statistics (!STATS)", padding=6)
        sf.pack(fill="both", expand=True)
        bar = ttk.Frame(sf)
        bar.pack(fill="x")
        ttk.Button(bar, text="Refresh", command=lambda: self._send("!STATS")).pack(side="left")
        ttk.Button(bar, text="Reset counters",
                   command=self._reset_stats).pack(side="left", padx=6)
        self.autostat_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar, text="auto every 2 s", variable=self.autostat_var,
                        command=self._on_autostat).pack(side="left", padx=6)
        self.stat_tree = ttk.Treeview(sf, columns=("type", "count", "hz", "age"),
                                      show="headings", height=12)
        for c, w, t in (("type", 70, "type"), ("count", 90, "frames"),
                        ("hz", 80, "Hz"), ("age", 90, "age ms")):
            self.stat_tree.heading(c, text=t)
            self.stat_tree.column(c, width=w, anchor="e")
        self.stat_tree.pack(fill="both", expand=True, pady=(6, 0))
        ttk.Label(sf, text="'unkN' rows are frame types the sketch does not decode; "
                           "'bad' counts frames whose payload length made no sense.",
                  foreground="#555", font=("Segoe UI", 8)).pack(anchor="w", pady=(4, 0))

        # ── Targets ──
        tf = ttk.LabelFrame(left, text="Tracked targets (!TGT)", padding=6)
        tf.pack(fill="x", pady=(8, 0))
        ttk.Button(tf, text="Read targets", command=self._read_targets).pack(anchor="w")
        self.tgt_tree = ttk.Treeview(tf, columns=("x", "y", "dop", "cl", "v"),
                                     show="headings", height=4)
        for c, t in (("x", "x (m)"), ("y", "y (m)"), ("dop", "dop"),
                     ("cl", "cluster"), ("v", "cm/s")):
            self.tgt_tree.heading(c, text=t)
            self.tgt_tree.column(c, width=70, anchor="e")
        self.tgt_tree.pack(fill="x", pady=(4, 0))

        # ── Raw monitor ──
        rf = ttk.LabelFrame(right, text="Raw frame monitor", padding=6)
        rf.pack(fill="both", expand=True)
        bar2 = ttk.Frame(rf)
        bar2.pack(fill="x")
        ttk.Label(bar2, text="rawmode:").pack(side="left")
        self.rawmode_var = tk.StringVar(value=RAW_MODES[0])
        cb = ttk.Combobox(bar2, width=22, state="readonly", values=RAW_MODES,
                          textvariable=self.rawmode_var)
        cb.pack(side="left", padx=4)
        cb.bind("<<ComboboxSelected>>",
                lambda _e: self._set_key("rawmode", RAW_MODES.index(self.rawmode_var.get())))
        self.rawpause_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar2, text="pause", variable=self.rawpause_var).pack(side="left", padx=8)
        ttk.Button(bar2, text="Clear",
                   command=lambda: self.raw_txt.delete("1.0", "end")).pack(side="left")
        self.raw_txt = tk.Text(rf, height=12, font=("Consolas", 9), wrap="none")
        self.raw_txt.pack(fill="both", expand=True, pady=(6, 0))

        # ── Manual TX ──
        txf = ttk.LabelFrame(right, text="Send a frame (!TX)", padding=6)
        txf.pack(fill="x", pady=(8, 0))
        ttk.Label(txf, text="type hex:").grid(row=0, column=0, sticky="e")
        self.tx_type = tk.StringVar(value="0A13")
        ttk.Entry(txf, width=8, textvariable=self.tx_type).grid(row=0, column=1, sticky="w", padx=4)
        ttk.Label(txf, text="payload hex:").grid(row=0, column=2, sticky="e")
        self.tx_data = tk.StringVar(value="")
        ttk.Entry(txf, width=34, textvariable=self.tx_data).grid(row=0, column=3, sticky="w", padx=4)
        ttk.Button(txf, text="Send", command=self._send_tx).grid(row=0, column=4, padx=6)
        ttk.Label(txf, text="Types outside 0x0A00-0x0AFF / 0x0F00-0x0FFF need the 'unsafe' switch\n"
                            "on the Controls tab. Nothing published maps this module's command space,\n"
                            "so treat every reply as a discovery and write down what you sent.",
                  foreground="#555", font=("Segoe UI", 8)).grid(row=1, column=0, columnspan=5,
                                                                sticky="w", pady=(4, 0))

        # ── Probe ──
        prf = ttk.LabelFrame(right, text="Probe a type range (!PROBE)", padding=6)
        prf.pack(fill="both", expand=True, pady=(8, 0))
        pb = ttk.Frame(prf)
        pb.pack(fill="x")
        ttk.Label(pb, text="from:").pack(side="left")
        self.pr_from = tk.StringVar(value="0A00")
        ttk.Entry(pb, width=7, textvariable=self.pr_from).pack(side="left", padx=2)
        ttk.Label(pb, text="to:").pack(side="left")
        self.pr_to = tk.StringVar(value="0A2F")
        ttk.Entry(pb, width=7, textvariable=self.pr_to).pack(side="left", padx=2)
        ttk.Label(pb, text="gap ms:").pack(side="left", padx=(8, 2))
        self.pr_gap = tk.StringVar(value="200")
        ttk.Entry(pb, width=6, textvariable=self.pr_gap).pack(side="left")
        self.probe_btn = ttk.Button(pb, text="Run probe", command=self._run_probe)
        self.probe_btn.pack(side="left", padx=8)
        self.probe_tree = ttk.Treeview(prf, columns=("sent", "n", "types"),
                                       show="headings", height=6)
        for c, w, t in (("sent", 70, "sent"), ("n", 50, "replies"), ("types", 200, "types seen")):
            self.probe_tree.heading(c, text=t)
            self.probe_tree.column(c, width=w, anchor="w")
        self.probe_tree.pack(fill="both", expand=True, pady=(6, 0))

    # ─────────────────────────────────────────────────────── connection glue

    @staticmethod
    def _ports():
        return [p.device for p in serial.tools.list_ports.comports()]

    def _refresh_ports(self):
        self.port_cb["values"] = self._ports()

    def _toggle_connect(self):
        if self.sm.is_open():
            self.sm.disconnect()
            self.connect_btn.config(text="Connect")
            self.conn_lbl.config(text="disconnected", foreground="#a00")
            return
        port = self.port_cb.get().strip()
        if not port:
            messagebox.showwarning("No port", "Pick a serial port first.")
            return
        try:
            self.sm.connect(port)
        except (OSError, serial.SerialException) as e:
            messagebox.showerror("Connect failed", str(e))
            return
        self.connect_btn.config(text="Disconnect")
        self.conn_lbl.config(text=f"connected {port}", foreground="#070")
        self.after(700, lambda: (self._send("!PING"), self._send("!CFG"),
                                 self._send("!STATS")))

    def _send(self, text):
        if not self.sm.send(text):
            self._log("! not connected")
            return False
        self._log("> " + text)
        return True

    def _send_manual(self):
        t = self.cmd_var.get().strip()
        if t:
            self._send(t)
            self.cmd_var.set("")

    def _log(self, text):
        self.log.insert("end", text + "\n")
        if float(self.log.index("end")) > 500:
            self.log.delete("1.0", "100.0")
        self.log.see("end")

    def _on_close(self):
        self._stop_logging()
        try:
            self.sm.send("!STREAM,0")
        except Exception:
            pass
        self.sm.disconnect()
        self.destroy()

    # ───────────────────────────────────────────────────────── config wiring

    def _set_key(self, key, value):
        """Push one config key to the device, unless we're mirroring a device push."""
        if self._suppress_cfg_push:
            return
        try:
            v = int(float(str(value).strip()))
        except (TypeError, ValueError):
            return
        if key in self.limits:
            lo, hi = self.limits[key]
            v = max(lo, min(hi, v))
        if self.cfg.get(key) == v:
            return
        self._send(f"!SET,{key},{v}")

    def _push_repmask(self):
        mask = 0
        for bit, var in self.rep_vars.items():
            if var.get():
                mask |= 1 << bit
        self.repmask_lbl.config(text=f"repmask 0x{mask:02X}")
        self._set_key("repmask", mask)

    def _rep_preset(self, on):
        for var in self.rep_vars.values():
            var.set(on)
        self._push_repmask()

    def _on_unsafe(self):
        if self.unsafe_var.get():
            ok = messagebox.askyesno(
                "Unsafe frame types",
                "This lets !TX and !PROBE send frame types outside the range the "
                "module's own reports use.\n\nNothing published documents this "
                "module's command space, so an unknown type could put it into a "
                "state only a power cycle clears. Firmware update frames, if they "
                "exist, live somewhere in that space too.\n\nEnable anyway?")
            if not ok:
                self.unsafe_var.set(False)
                return
        self._set_key("unsafe", int(self.unsafe_var.get()))

    def _load_nvs(self):
        self._send("!LOAD")
        self.after(300, lambda: self._send("!CFG"))

    def _restore_defaults(self):
        self._send("!DEFAULTS")
        self.after(300, lambda: self._send("!CFG"))

    def _apply_cfg_to_widgets(self):
        """Mirror device state into the widgets without echoing it back."""
        self._suppress_cfg_push = True
        try:
            for key, var in self.vars.items():
                if key in self.cfg:
                    var.set(str(self.cfg[key]))
            if "streamms" in self.cfg:
                self.streamms_var.set(str(self.cfg["streamms"]))
            if "strmode" in self.cfg:
                self.strmode_var.set(self.cfg["strmode"])
            if "capmode" in self.cfg:
                self.capmode_var.set(self.cfg["capmode"])
                self.capmode2_var.set(self.cfg["capmode"])
            if "capms" in self.cfg:
                self.capms_var.set(str(self.cfg["capms"]))
            if "ledmode" in self.cfg and 0 <= self.cfg["ledmode"] < len(LED_MODES):
                self.ledmode_var.set(LED_MODES[self.cfg["ledmode"]])
            if "rawmode" in self.cfg and 0 <= self.cfg["rawmode"] < len(RAW_MODES):
                self.rawmode_var.set(RAW_MODES[self.cfg["rawmode"]])
            if "txck" in self.cfg:
                self.txck_var.set(bool(self.cfg["txck"]))
            if "unsafe" in self.cfg:
                self.unsafe_var.set(bool(self.cfg["unsafe"]))
            if "repmask" in self.cfg:
                mask = self.cfg["repmask"]
                for bit, var in self.rep_vars.items():
                    var.set(bool(mask & (1 << bit)))
                self.repmask_lbl.config(text=f"repmask 0x{mask:02X}")
        finally:
            self._suppress_cfg_push = False

        fwmod = self.cfg.get("fwmod", 0)
        if fwmod:
            self.mod_lbl.config(text="module: %d.%d.%d.%d" % (
                fwmod & 0xFF, (fwmod >> 8) & 0xFF, (fwmod >> 16) & 0xFF, (fwmod >> 24) & 0xFF))

        self.cfg_tree.delete(*self.cfg_tree.get_children())
        for key in self.cfg:
            lo, hi = self.limits.get(key, ("", ""))
            self.cfg_tree.insert("", "end", text=key, values=(self.cfg[key], lo, hi))

    # ─────────────────────────────────────────────────────────── line pumping

    def _call_soon(self, fn):
        """Schedule `fn` on the Tk main thread. Safe to call from any thread."""
        self._call_q.put(fn)

    def _pump(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "__closed__":
                    self.conn_lbl.config(text="port closed", foreground="#a00")
                    self.connect_btn.config(text="Connect")
                    self._stop_logging()
                    continue
                self._handle_line(payload)
        except queue.Empty:
            pass

        try:
            while True:
                self._call_q.get_nowait()()
        except queue.Empty:
            pass

        self.after(40, self._pump)

    def _handle_line(self, line):
        if not line:
            return

        # Capture body lines are high volume — keep them out of the log.
        if line.startswith("$S,"):
            if self._cap_active:
                self._cap_rows.append(line[3:])
            return

        parts = line.split(",")
        tag = parts[0]
        noisy = tag in ("$R", "$F")
        if not (noisy and self.quiet_var.get()):
            self._log(line)

        if tag == "$HELLO" and len(parts) >= 3:
            self.fw_lbl.config(text=f"fw: {parts[1]} v{parts[2]}")

        elif tag == "$CFG" and len(parts) >= 5:
            key, val = parts[1], parts[2]
            try:
                self.cfg[key] = int(val)
                self.limits[key] = (int(parts[3]), int(parts[4]))
            except ValueError:
                pass

        elif tag == "$CFGEND":
            self._apply_cfg_to_widgets()
            self._cfg_event.set()

        elif tag in ("$OK", "$VAL") and len(parts) >= 3:
            # Only config keys land in the mirror — save/load/tx/zero acks share
            # the same shape but are not settings.
            key = parts[1]
            if key in self.limits:
                try:
                    self.cfg[key] = int(parts[2])
                except ValueError:
                    pass
                self._refresh_cfg_tree_row(key)

        elif tag == "$R" and len(parts) >= len(R_FIELDS) + 1:
            self._on_reading(parts[1:])

        elif tag == "$T" and len(parts) >= 7:
            self.tgt_tree.insert("", "end", values=(parts[2], parts[3], parts[4],
                                                    parts[5], parts[6]))

        elif tag == "$TEND":
            pass

        elif tag == "$F" and len(parts) >= 5:
            if not self.rawpause_var.get():
                self.raw_txt.insert("end", f"{parts[1]:>9} ms  type {parts[2]}  "
                                           f"len {parts[3]:>3}  {parts[4]}\n")
                if float(self.raw_txt.index("end")) > 400:
                    self.raw_txt.delete("1.0", "100.0")
                self.raw_txt.see("end")

        elif tag == "$EVT" and len(parts) >= 4:
            pass   # already shown in the serial log

        elif tag == "$STAT" and len(parts) >= 6:
            if parts[1] == "phase":      # always the first row of a dump
                self._stat_rows.clear()
            self._stat_rows[parts[1]] = parts[1:]

        elif tag == "$STATEND":
            self._refresh_stats()

        elif tag == "$PRB" and len(parts) >= 3:
            self._probe_rows.append(parts)
            self.probe_tree.insert("", "end",
                                   values=(parts[1], parts[2], " ".join(parts[3:])))

        elif tag == "$PRBEND":
            self.probe_btn.config(state="normal")
            self._log(f"# probe finished, {len(self._probe_rows)} responsive types")

        elif tag == "$CAPB":
            self._cap_active = True
            self._cap_rows = []
            self._cap_names = []
            self._cap_header = parts

        elif tag == "$CH":
            if self._cap_active:
                self._cap_names = parts[2:]      # skip the t_us column name

        elif tag == "$CAPE":
            self._cap_active = False
            self._finish_capture()

    def _refresh_cfg_tree_row(self, key):
        if key not in self.cfg:
            return
        for iid in self.cfg_tree.get_children():
            if self.cfg_tree.item(iid, "text") == key:
                lo, hi = self.limits.get(key, ("", ""))
                self.cfg_tree.item(iid, values=(self.cfg[key], lo, hi))
                return

    # ────────────────────────────────────────────────────────── live readings

    def _on_reading(self, fields):
        r = dict(zip(R_FIELDS, fields))
        self.last_r = r

        for k in ("br", "hr", "dist"):
            v = _f(r.get(k))
            self.big[k].config(text="--" if not np.isfinite(v) else f"{v:.2f}")

        present = r.get("present") == "1"
        self.present_lbl.config(text="target: PRESENT" if present else "target: none",
                                foreground="#070" if present else "#a00")
        human = int(r.get("human", "-1"))
        self.state_lbl.config(text=(
            f"range flag {r.get('rflag'):>3}   human {'?' if human < 0 else human}   "
            f"targets {r.get('ntgt')}   age {r.get('age_ms')} ms\n"
            f"raw:  breath {r.get('br_raw')}  heart {r.get('hr_raw')}  "
            f"dist {r.get('dist_raw')}\n"
            f"phase: total {r.get('total')}  breath {r.get('breath')}  "
            f"heart {r.get('heart')}"))

        t = _f(r.get("t_ms")) / 1000.0
        if self._t0 is None:
            self._t0 = t
        self.tr_t.append(t - self._t0)
        for k in self.tr:
            self.tr[k].append(_f(r.get(k)))

        try:
            window = float(self.window_var.get())
        except ValueError:
            window = LIVE_WINDOW_S
        while self.tr_t and self.tr_t[-1] - self.tr_t[0] > window:
            self.tr_t.pop(0)
            for k in self.tr:
                self.tr[k].pop(0)

        self._live_dirty = True
        self._write_log_row(r)

    def _clear_traces(self):
        self.tr_t = []
        for k in self.tr:
            self.tr[k] = []
        self._t0 = None
        self._live_dirty = True

    def _mark_live_dirty(self):
        self._live_dirty = True

    def _live_redraw_tick(self):
        # Streaming can arrive at 20 Hz; redraw at 5 Hz instead of per sample.
        if self._live_dirty:
            self._live_dirty = False
            self._draw_live()
        self.after(200, self._live_redraw_tick)

    def _draw_live(self):
        self.ax_phase.clear()
        self.ax_rate.clear()

        t = self.tr_t
        if t:
            for name in ("total", "breath", "heart"):
                if self.trace_vars[name].get():
                    self.ax_phase.plot(t, self.tr[name], lw=1.0,
                                       color=CAP_COLOR[name], label=name)
            for name in ("br", "hr", "dist"):
                if self.trace_vars[name].get():
                    self.ax_rate.plot(t, self.tr[name], lw=1.2, marker=".", ms=2,
                                      color=CAP_COLOR[name], label=name)
        self.ax_phase.set_ylabel("phase")
        self.ax_phase.grid(alpha=0.3)
        if t:
            self.ax_phase.legend(fontsize=8, loc="upper left", ncol=3)
        self.ax_rate.set_ylabel("rate / m")
        self.ax_rate.set_xlabel("t (s)")
        self.ax_rate.grid(alpha=0.3)
        if t:
            self.ax_rate.legend(fontsize=8, loc="upper left", ncol=3)
        self.live_fig.tight_layout()
        self.live_canvas.draw_idle()

    def _on_stream(self):
        self._send("!STREAM," + ("1" if self.stream_var.get() else "0"))

    # ── CSV log ─────────────────────────────────────────────────────────────
    #
    # One row per $R line, written as it arrives — no separate timer, because the
    # device already decides the cadence (`strmode` / `streamms`).

    def _choose_log_file(self):
        default = f"mr60bha2_{datetime.now():%Y%m%d_%H%M%S}.csv"
        path = filedialog.asksaveasfilename(defaultextension=".csv",
                                            initialfile=default,
                                            filetypes=[("CSV", "*.csv")])
        if path:
            self.logfile_var.set(path)

    def _toggle_logging(self):
        if self._log_file:
            self._stop_logging()
            return
        path = self.logfile_var.get().strip()
        if not path:
            self._choose_log_file()
            path = self.logfile_var.get().strip()
            if not path:
                return
        new = not os.path.exists(path) or os.path.getsize(path) == 0
        try:
            self._log_file = open(path, "a", newline="")
        except OSError as e:
            messagebox.showerror("Cannot open log", str(e))
            self._log_file = None
            return
        self._log_writer = csv.writer(self._log_file)
        if new:
            self._log_writer.writerow(["timestamp"] + R_FIELDS)
        self._log_rows = 0
        self._log_path = path
        self.logbtn.config(text="Stop logging")
        self.logstat_lbl.config(text=f"logging to {os.path.basename(path)}")

    def _write_log_row(self, r):
        if not self._log_writer:
            return
        self._log_writer.writerow([datetime.now().isoformat(timespec="milliseconds")]
                                  + [r.get(k, "") for k in R_FIELDS])
        self._log_rows += 1
        if self._log_rows % 20 == 0:
            self._log_file.flush()
            self.logstat_lbl.config(
                text=f"{self._log_rows} rows -> {os.path.basename(self._log_path)}")

    def _stop_logging(self):
        if not self._log_file:
            return
        try:
            self._log_file.flush()
            self._log_file.close()
        except OSError:
            pass
        self._log_file = None
        self._log_writer = None
        self.logbtn.config(text="Start logging")
        self.logstat_lbl.config(text=f"stopped after {self._log_rows} rows")

    # ───────────────────────────────────────────────────────────────  capture

    def _run_capture(self):
        if not self.sm.is_open():
            messagebox.showwarning("Not connected", "Connect to the board first.")
            return
        try:
            ms = int(self.capms_var.get())
        except ValueError:
            messagebox.showerror("Bad window", "Capture window must be an integer (ms).")
            return
        self.cap_btn.config(state="disabled")
        self.cap_result.config(text="capturing...")
        self._cap_event.clear()
        self._send(f"!CAP,{ms}")
        self.after(ms + 60000, self._capture_watchdog)

    def _capture_watchdog(self):
        if str(self.cap_btn["state"]) == "disabled" and not self._cap_event.is_set():
            self._cap_active = False
            self.cap_btn.config(state="normal")
            self.cap_result.config(text="capture timed out — no $CAPE from device")

    def _finish_capture(self):
        hdr = self._cap_header or []
        names = self._cap_names or CAP_COLS
        t_us, data = [], []
        for row in self._cap_rows:
            f = row.split(",")
            if len(f) != len(names) + 1:
                continue
            try:
                t_us.append(int(f[0]))
            except ValueError:
                continue
            data.append([_f(x) for x in f[1:]])

        meta = {}
        keys = ["n", "elapsed_us", "rate_hz", "mode", "dt_ms", "dropped"]
        for k, v in zip(keys, hdr[1:7]):
            meta[k] = v

        self.capture = {
            "names": names,
            "t_us": np.array(t_us, dtype=np.int64),
            "data": (np.array(data, dtype=float) if data
                     else np.zeros((0, len(names)), float)),
            "meta": meta,
        }

        n = len(t_us)
        rate = float(meta.get("rate_hz", 0) or 0)
        elapsed = float(meta.get("elapsed_us", 0) or 0) / 1e6
        jitter = ""
        if n > 2:
            d = np.diff(self.capture["t_us"]) / 1000.0
            jitter = (f"   interval {d.mean():.1f} ms "
                      f"(min {d.min():.1f} / max {d.max():.1f} / sd {d.std():.1f})")
        drop = meta.get("dropped", "0")
        self.cap_result.config(
            text=(f"{n} samples over {elapsed:.2f} s  ->  {rate:.3f} Hz   "
                  f"[mode {meta.get('mode','?')}, grid {meta.get('dt_ms','?')} ms, "
                  f"dropped {drop}]{jitter}"))

        self.cap_btn.config(state="normal")
        self.savecsv_btn.config(state="normal" if n else "disabled")
        self._draw_capture()
        self._cap_event.set()

    def _cap_series(self, name):
        """Return (t_s, values) for one capture column, or (None, None)."""
        cap = self.capture
        if not cap or cap["data"].shape[0] == 0 or name not in cap["names"]:
            return None, None
        j = cap["names"].index(name)
        return cap["t_us"] / 1e6, cap["data"][:, j]

    def _draw_capture(self):
        self.ax_cap.clear()
        self.ax_fft.clear()
        cap = self.capture
        if not cap or cap["data"].shape[0] == 0:
            self.ax_cap.set_title("no capture data")
            self.fft_lbl.config(text="--")
            self.cap_canvas.draw_idle()
            return

        t = cap["t_us"] / 1e6
        for j, name in enumerate(cap["names"]):
            var = self.cap_plot_vars.get(name)
            if var is None or not var.get():
                continue
            self.ax_cap.plot(t, cap["data"][:, j], lw=1.0,
                             color=CAP_COLOR.get(name, "#333"), label=name)
        self.ax_cap.set_xlabel("t (s)")
        self.ax_cap.set_ylabel("value")
        self.ax_cap.grid(alpha=0.3)
        self.ax_cap.legend(fontsize=8, ncol=3)
        self.ax_cap.set_title(f"Capture — {cap['data'].shape[0]} samples "
                              f"@ {cap['meta'].get('rate_hz','?')} Hz")

        self._draw_fft()
        self.cap_fig.tight_layout()
        self.cap_canvas.draw_idle()

    def _draw_fft(self):
        """Re-derive a rate from the phase waveform, independent of the module."""
        src = self.fft_src.get()
        t, y = self._cap_series(src)
        if t is None or len(t) < 32:
            self.ax_fft.set_title("need at least 32 samples for a spectrum")
            self.fft_lbl.config(text="--")
            return

        good = np.isfinite(y)
        if good.sum() < 32:
            self.ax_fft.set_title(f"{src}: not enough finite samples")
            self.fft_lbl.config(text="--")
            return

        # Capture mode 0 timestamps every frame as it arrived, so the series is
        # only nominally uniform — resample onto an even grid before transforming.
        n = int(good.sum())
        tg = np.linspace(t[good][0], t[good][-1], n)
        yg = np.interp(tg, t[good], y[good])
        dt = (tg[-1] - tg[0]) / (n - 1)
        if dt <= 0:
            self.fft_lbl.config(text="--")
            return

        if self.detrend_var.get():
            yg = yg - np.polyval(np.polyfit(tg, yg, 1), tg)
        else:
            yg = yg - yg.mean()
        spec = np.abs(np.fft.rfft(yg * np.hanning(n)))
        freq = np.fft.rfftfreq(n, dt)
        cpm = freq * 60.0

        lo, hi = BAND[self.fft_band.get()]
        band = (cpm >= lo) & (cpm <= hi)
        peak_txt = "no energy in band"
        if band.any() and spec[band].max() > 0:
            k = np.argmax(np.where(band, spec, 0.0))
            peak = cpm[k]
            # Parabolic interpolation on the three bins around the peak — the bin
            # spacing is 60/(n*dt) cpm, which is coarse on a short capture.
            if 0 < k < len(spec) - 1:
                a, b, c = spec[k - 1], spec[k], spec[k + 1]
                den = a - 2 * b + c
                if den != 0:
                    peak += 0.5 * (a - c) / den * (cpm[1] - cpm[0])
            ref_key = "br" if self.fft_band.get() == "breath" else "hr"
            _, ref = self._cap_series(ref_key)
            ref_txt = ""
            if ref is not None and np.isfinite(ref).any():
                ref_txt = f"   module said {np.nanmean(ref):.1f}"
            peak_txt = (f"{src} peak {peak:.2f} /min   "
                        f"(bin {cpm[1] - cpm[0]:.2f} /min){ref_txt}")
        self.fft_lbl.config(text=peak_txt)

        self.ax_fft.plot(cpm, spec, lw=1.0, color=CAP_COLOR.get(src, "#333"))
        self.ax_fft.axvspan(lo, hi, color="#ffd", alpha=0.6, zorder=0)
        self.ax_fft.set_xlim(0, max(hi * 1.6, 60))
        self.ax_fft.set_xlabel("cycles per minute")
        self.ax_fft.set_ylabel("|FFT|")
        self.ax_fft.grid(alpha=0.3)
        self.ax_fft.set_title(f"{src} spectrum, {self.fft_band.get()} band shaded "
                              f"({1 / dt:.1f} Hz effective sample rate)")

    def _save_capture_csv(self):
        cap = self.capture
        if not cap or cap["data"].shape[0] == 0:
            return
        default = f"mr60bha2_cap_{datetime.now():%Y%m%d_%H%M%S}.csv"
        path = filedialog.asksaveasfilename(defaultextension=".csv",
                                            initialfile=default,
                                            filetypes=[("CSV", "*.csv")])
        if not path:
            return
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            for k, v in cap["meta"].items():
                w.writerow([f"# {k}", v])
            w.writerow(["t_us"] + cap["names"])
            for i in range(cap["data"].shape[0]):
                w.writerow([int(cap["t_us"][i])] + list(cap["data"][i]))
        self._log(f"# saved {os.path.basename(path)}")

    # ─────────────────────────────────────────────────────────────── protocol

    def _refresh_stats(self):
        # row = [name, type, count, hz, age]; the tree hides #0, so the name and
        # its hex type share the first column.
        self.stat_tree.delete(*self.stat_tree.get_children())
        for name, row in self._stat_rows.items():
            self.stat_tree.insert("", "end",
                                  values=(f"{name} {row[1]}", row[2], row[3], row[4]))

    def _reset_stats(self):
        self._stat_rows.clear()
        self.stat_tree.delete(*self.stat_tree.get_children())
        self._send("!STATS,0")

    def _read_targets(self):
        self.tgt_tree.delete(*self.tgt_tree.get_children())
        self._send("!TGT")

    def _on_autostat(self):
        if self.autostat_var.get():
            self._autostat_tick()

    def _autostat_tick(self):
        if not self.autostat_var.get():
            return
        if self.sm.is_open():
            self.sm.send("!STATS")      # bypass _send: no log spam every 2 s
        self.after(2000, self._autostat_tick)

    def _send_tx(self):
        t = self.tx_type.get().strip()
        d = self.tx_data.get().strip().replace(" ", "")
        if not t:
            return
        self._send(f"!TX,{t},{d}" if d else f"!TX,{t}")

    def _run_probe(self):
        if not self.sm.is_open():
            messagebox.showwarning("Not connected", "Connect to the board first.")
            return
        if not self.unsafe_var.get():
            messagebox.showwarning("Probe blocked",
                                   "Probing needs the 'unsafe' switch on the "
                                   "Controls tab — read the warning there first.")
            return
        try:
            a = int(self.pr_from.get(), 16)
            b = int(self.pr_to.get(), 16)
            gap = int(self.pr_gap.get())
        except ValueError:
            messagebox.showerror("Bad range", "From/to are hex, gap is an integer.")
            return
        span = abs(b - a) + 1
        if not messagebox.askyesno(
                "Run probe",
                f"Send {min(span, 256)} frames, {gap} ms apart "
                f"(about {min(span, 256) * gap / 1000.0:.0f} s).\n\n"
                "The board ignores commands while probing."):
            return
        self.probe_tree.delete(*self.probe_tree.get_children())
        self._probe_rows = []
        self.probe_btn.config(state="disabled")
        self._send(f"!PROBE,{a:04X},{b:04X},{gap}")


if __name__ == "__main__":
    App().mainloop()
