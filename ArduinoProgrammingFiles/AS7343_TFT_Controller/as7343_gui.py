#!/usr/bin/env python3
"""
as7343_gui.py — host control panel for AS7343_TFT_Controller.ino

Full control surface for an Adafruit AS7343 on a Feather RP2040 + 2.4" TFT
FeatherWing.  The touchscreen carries a reduced control set; everything the
sensor exposes lives here.

Tabs
  Live        live spectrum bar chart, single read / continuous stream
  Controls    every lever: gain, ATIME, ASTEP, WTIME, SMUX, LED, auto-zero,
              thresholds, per-channel store mask, EEPROM save/load
  Speed Test  timed burst captures — measured sample rate plus per-channel
              traces, and a sweep mode that walks one parameter and tabulates
              the achieved rate at each setting

Requires: pyserial, matplotlib, numpy
    pip install pyserial matplotlib numpy
"""

import csv
import os
import queue
import threading
import tkinter as tk
from datetime import datetime
from tkinter import filedialog, messagebox, ttk

import numpy as np
import serial
import serial.tools.list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

BAUD = 115200

# ── Channel table — must match kChan[] in the sketch ────────────────────────
# (index, name, centre wavelength in nm or None for the broadband VIS diodes)
CHANNELS = [
    ("FZ",   450), ("FY",   555), ("FXL",  600),
    ("NIR",  855), ("VTL0", None), ("VBR0", None),
    ("F2",   425), ("F3",   475), ("F4",   515),
    ("F6",   640), ("VTL1", None), ("VBR1", None),
    ("F1",   405), ("F7",   690), ("F8",   745),
    ("F5",   550), ("VTL2", None), ("VBR2", None),
]
NAME_TO_IDX = {n: i for i, (n, _) in enumerate(CHANNELS)}

# The 12 true spectral channels in wavelength order, for the bar chart.
SPECTRUM_ORDER = [12, 6, 0, 7, 8, 15, 1, 2, 9, 13, 14, 3]

CHAN_COLOR = {
    "F1": "#8B00FF", "F2": "#4B0FE0", "FZ": "#0000FF", "F3": "#00BFFF",
    "F4": "#00E000", "F5": "#7FE000", "FY": "#BFE000", "FXL": "#FF8C00",
    "F6": "#FF0000", "F7": "#B00000", "F8": "#800000", "NIR": "#555555",
    "VTL0": "#999999", "VBR0": "#AAAAAA", "VTL1": "#999999",
    "VBR1": "#AAAAAA", "VTL2": "#999999", "VBR2": "#AAAAAA",
}

GAIN_LABELS = ["0.5x", "1x", "2x", "4x", "8x", "16x", "32x", "64x",
               "128x", "256x", "512x", "1024x", "2048x"]

# Sweep presets: parameter key -> (label, default value list)
SWEEP_PRESETS = {
    "astep":  ("ASTEP",  "49, 99, 199, 399, 599, 999, 1999"),
    "atime":  ("ATIME",  "0, 1, 3, 7, 15, 29, 59"),
    "gain":   ("GAIN idx", "0, 2, 4, 6, 8, 10, 12"),
    "smux":   ("SMUX ch", "6, 12, 18"),
}


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
        self.title("AS7343 Controller")
        self.geometry("1150x820")

        self.q = queue.Queue()
        self.sm = SerialManager(self.q)
        # Tk is not thread-safe; worker threads hand callables here and the
        # main-thread pump runs them.
        self._call_q = queue.Queue()

        # Device state mirrored from $CFG / $R / $CAP*
        self.cfg = {}           # key -> int value
        self.limits = {}        # key -> (min, max)
        self.tint_ms = 0.0
        self.enabled_names = []  # channel names currently streamed, in order
        self.live_vals = {}      # name -> value
        self._chan_resync = False
        self._ymax_is_full = False  # live y-axis is tracking ADC full scale
        self.live_asat = False
        self.live_dsat = False

        # Capture reception state
        self._cap_active = False
        self._cap_header = None
        self._cap_names = []
        self._cap_rows = []
        self.capture = None      # dict: names, t_us, data (np array), meta
        self._cap_event = threading.Event()

        self._cfg_event = threading.Event()
        self._suppress_cfg_push = False   # don't echo device->widget updates back

        self._build_ui()
        self.after(40, self._pump)
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
        self.tint_lbl = ttk.Label(top, text="tint: -- ms", font=("Consolas", 10))
        self.tint_lbl.pack(side="left", padx=8)
        self.sat_lbl = ttk.Label(top, text="sat: --", font=("Consolas", 10, "bold"))
        self.sat_lbl.pack(side="left", padx=8)

        self.nb = ttk.Notebook(self)
        self.nb.pack(fill="both", expand=True, padx=6, pady=4)
        self.live_tab = ttk.Frame(self.nb)
        self.ctrl_tab = ttk.Frame(self.nb)
        self.cap_tab = ttk.Frame(self.nb)
        self.nb.add(self.live_tab, text="Live")
        self.nb.add(self.ctrl_tab, text="Controls")
        self.nb.add(self.cap_tab, text="Speed Test")

        self._build_live_tab()
        self._build_ctrl_tab()
        self._build_cap_tab()

        logf = ttk.LabelFrame(self, text="Serial log", padding=4)
        logf.pack(fill="x", padx=6, pady=(0, 6))
        self.log = tk.Text(logf, height=7, font=("Consolas", 9), wrap="none")
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

    # ── Live tab ────────────────────────────────────────────────────────────

    def _build_live_tab(self):
        ctl = ttk.Frame(self.live_tab, padding=6)
        ctl.pack(fill="x")

        ttk.Button(ctl, text="Single Read", command=lambda: self._send("!READ")).pack(side="left")
        self.stream_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(ctl, text="Stream", variable=self.stream_var,
                        command=self._on_stream).pack(side="left", padx=10)
        ttk.Label(ctl, text="interval ms:").pack(side="left")
        self.streamms_var = tk.StringVar(value="250")
        ttk.Entry(ctl, width=7, textvariable=self.streamms_var).pack(side="left", padx=2)
        ttk.Button(ctl, text="Set",
                   command=lambda: self._set_key("streamms", self.streamms_var.get())).pack(side="left")

        ttk.Separator(ctl, orient="vertical").pack(side="left", fill="y", padx=10)
        self.logscale_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(ctl, text="Log scale", variable=self.logscale_var,
                        command=self._draw_live).pack(side="left")

        self.ylock_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(ctl, text="Lock Y", variable=self.ylock_var,
                        command=self._on_ylock).pack(side="left", padx=(10, 2))
        self.ymax_var = tk.StringVar(value="")
        ymax_e = ttk.Entry(ctl, width=8, textvariable=self.ymax_var)
        ymax_e.pack(side="left", padx=2)
        ymax_e.bind("<Return>", lambda _e: self._apply_ymax())
        ymax_e.bind("<FocusOut>", lambda _e: self._apply_ymax())
        self.full_btn = ttk.Button(ctl, text="Full scale", command=self._on_full_scale)
        self.full_btn.pack(side="left", padx=2)

        ttk.Button(ctl, text="Flicker detect",
                   command=lambda: self._send("!FLICKER")).pack(side="left", padx=10)
        self.flicker_lbl = ttk.Label(ctl, text="flicker: --", font=("Consolas", 10))
        self.flicker_lbl.pack(side="left", padx=4)

        body = ttk.Frame(self.live_tab)
        body.pack(fill="both", expand=True)

        self.live_fig = Figure(figsize=(7.5, 4.2), dpi=100)
        self.live_ax = self.live_fig.add_subplot(111)
        self.live_canvas = FigureCanvasTkAgg(self.live_fig, master=body)
        self.live_canvas.get_tk_widget().pack(side="left", fill="both", expand=True,
                                              padx=6, pady=6)

        tblf = ttk.LabelFrame(body, text="Readings", padding=4)
        tblf.pack(side="right", fill="y", padx=6, pady=6)
        self.live_tree = ttk.Treeview(tblf, columns=("nm", "val"), show="headings",
                                      height=20)
        self.live_tree.heading("nm", text="nm")
        self.live_tree.heading("val", text="counts")
        self.live_tree.column("nm", width=70, anchor="e")
        self.live_tree.column("val", width=90, anchor="e")
        self.live_tree.pack(fill="y", expand=True)

        self._refresh_full_scale()
        self._draw_live()

    # ── Controls tab ────────────────────────────────────────────────────────

    def _build_ctrl_tab(self):
        outer = ttk.Frame(self.ctrl_tab, padding=6)
        outer.pack(fill="both", expand=True)

        left = ttk.Frame(outer)
        left.pack(side="left", fill="y", padx=(0, 10))
        right = ttk.Frame(outer)
        right.pack(side="left", fill="both", expand=True)

        # ── Integration / gain ──
        intf = ttk.LabelFrame(left, text="Integration & gain", padding=6)
        intf.pack(fill="x", pady=(0, 8))

        ttk.Label(intf, text="Gain:").grid(row=0, column=0, sticky="e", pady=2)
        self.gain_var = tk.StringVar(value="256x")
        gcb = ttk.Combobox(intf, width=10, values=GAIN_LABELS, state="readonly",
                           textvariable=self.gain_var)
        gcb.grid(row=0, column=1, sticky="w", padx=4)
        gcb.bind("<<ComboboxSelected>>",
                 lambda _e: self._set_key("gain", GAIN_LABELS.index(self.gain_var.get())))

        self.atime_var = tk.StringVar(value="29")
        self._spin_row(intf, 1, "ATIME:", self.atime_var, 0, 255, "atime")
        self.astep_var = tk.StringVar(value="599")
        self._spin_row(intf, 2, "ASTEP:", self.astep_var, 0, 65534, "astep")

        ttk.Label(intf, text="tint/cycle:").grid(row=3, column=0, sticky="e", pady=2)
        self.tint_ctrl_lbl = ttk.Label(intf, text="-- ms", font=("Consolas", 10, "bold"))
        self.tint_ctrl_lbl.grid(row=3, column=1, sticky="w", padx=4)

        ttk.Label(intf, text="SMUX:").grid(row=4, column=0, sticky="e", pady=2)
        smf = ttk.Frame(intf)
        smf.grid(row=4, column=1, columnspan=2, sticky="w")
        self.smux_var = tk.IntVar(value=18)
        for v in (6, 12, 18):
            ttk.Radiobutton(smf, text=f"{v} ch", value=v, variable=self.smux_var,
                            command=lambda: self._set_key("smux", self.smux_var.get())
                            ).pack(side="left")

        self.wtime_var = tk.StringVar(value="0")
        self._spin_row(intf, 5, "WTIME:", self.wtime_var, 0, 255, "wtime")
        self.waiten_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(intf, text="wait enable", variable=self.waiten_var,
                        command=lambda: self._set_key("waiten", int(self.waiten_var.get()))
                        ).grid(row=6, column=1, sticky="w", pady=2)

        self.az_var = tk.StringVar(value="255")
        self._spin_row(intf, 7, "Auto-zero:", self.az_var, 0, 255, "az")

        # ── LED ──
        ledf = ttk.LabelFrame(left, text="On-board LED", padding=6)
        ledf.pack(fill="x", pady=(0, 8))
        self.led_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(ledf, text="LED on", variable=self.led_var,
                        command=lambda: self._set_key("led", int(self.led_var.get()))
                        ).grid(row=0, column=0, columnspan=2, sticky="w")
        self.ledma_var = tk.StringVar(value="12")
        self._spin_row(ledf, 1, "Current mA:", self.ledma_var, 4, 258, "ledma")

        # ── Interrupt / thresholds ──
        thf = ttk.LabelFrame(left, text="Spectral threshold", padding=6)
        thf.pack(fill="x", pady=(0, 8))
        self.thlow_var = tk.StringVar(value="0")
        self._spin_row(thf, 0, "Low:", self.thlow_var, 0, 65535, "thlow")
        self.thhigh_var = tk.StringVar(value="65535")
        self._spin_row(thf, 1, "High:", self.thhigh_var, 0, 65535, "thhigh")
        self.thch_var = tk.StringVar(value="0")
        self._spin_row(thf, 2, "Channel idx:", self.thch_var, 0, 17, "thch")
        self.pers_var = tk.StringVar(value="0")
        self._spin_row(thf, 3, "Persistence:", self.pers_var, 0, 15, "pers")
        self.spint_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(thf, text="spectral interrupt", variable=self.spint_var,
                        command=lambda: self._set_key("spint", int(self.spint_var.get()))
                        ).grid(row=4, column=0, columnspan=2, sticky="w", pady=2)

        # ── EEPROM ──
        eef = ttk.LabelFrame(left, text="Device config", padding=6)
        eef.pack(fill="x")
        ttk.Button(eef, text="Re-read from device",
                   command=lambda: self._send("!CFG")).pack(fill="x", pady=1)
        ttk.Button(eef, text="Save to EEPROM",
                   command=lambda: self._send("!SAVE")).pack(fill="x", pady=1)
        ttk.Button(eef, text="Load from EEPROM",
                   command=self._load_eeprom).pack(fill="x", pady=1)
        ttk.Button(eef, text="Restore defaults",
                   command=self._restore_defaults).pack(fill="x", pady=1)
        ttk.Button(eef, text="Read silicon ID",
                   command=lambda: self._send("!ID")).pack(fill="x", pady=1)

        # ── Channel mask ──
        chf = ttk.LabelFrame(right, text="Channel store / stream mask "
                                        "(which channels are recorded — SMUX mode sets "
                                        "which the sensor actually measures)", padding=6)
        chf.pack(fill="x")

        self.chan_vars = []
        for i, (name, nm) in enumerate(CHANNELS):
            v = tk.BooleanVar(value=True)
            self.chan_vars.append(v)
            txt = f"{i:2d}  {name:<5s} {nm} nm" if nm else f"{i:2d}  {name:<5s} clear"
            cb = ttk.Checkbutton(chf, text=txt, variable=v, command=self._push_chmask)
            cb.grid(row=i % 6, column=i // 6, sticky="w", padx=6, pady=1)

        bf = ttk.Frame(chf)
        bf.grid(row=6, column=0, columnspan=3, sticky="w", pady=(6, 0))
        ttk.Button(bf, text="All", command=lambda: self._mask_preset("all")).pack(side="left", padx=2)
        ttk.Button(bf, text="None", command=lambda: self._mask_preset("none")).pack(side="left", padx=2)
        ttk.Button(bf, text="Spectral only",
                   command=lambda: self._mask_preset("spectral")).pack(side="left", padx=2)
        ttk.Button(bf, text="Clear/VIS only",
                   command=lambda: self._mask_preset("vis")).pack(side="left", padx=2)
        self.mask_lbl = ttk.Label(bf, text="mask 0x3FFFF", font=("Consolas", 10))
        self.mask_lbl.pack(side="left", padx=12)

        # ── Raw config dump ──
        rawf = ttk.LabelFrame(right, text="Device config (as reported)", padding=6)
        rawf.pack(fill="both", expand=True, pady=(8, 0))
        self.cfg_tree = ttk.Treeview(rawf, columns=("val", "min", "max"),
                                     show="tree headings", height=14)
        self.cfg_tree.heading("#0", text="key")
        self.cfg_tree.heading("val", text="value")
        self.cfg_tree.heading("min", text="min")
        self.cfg_tree.heading("max", text="max")
        self.cfg_tree.column("#0", width=140)
        for c in ("val", "min", "max"):
            self.cfg_tree.column(c, width=90, anchor="e")
        self.cfg_tree.pack(fill="both", expand=True)

    def _spin_row(self, parent, row, label, var, lo, hi, key):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="e", pady=2)
        sp = ttk.Spinbox(parent, from_=lo, to=hi, width=10, textvariable=var)
        sp.grid(row=row, column=1, sticky="w", padx=4)
        sp.configure(command=lambda: self._set_key(key, var.get()))
        sp.bind("<Return>", lambda _e: self._set_key(key, var.get()))
        sp.bind("<FocusOut>", lambda _e: self._set_key(key, var.get()))
        return sp

    # ── Speed Test tab ──────────────────────────────────────────────────────

    def _build_cap_tab(self):
        ctl = ttk.LabelFrame(self.cap_tab, text="Single capture", padding=6)
        ctl.pack(fill="x", padx=6, pady=6)

        ttk.Label(ctl, text="Window ms:").pack(side="left")
        self.capms_var = tk.StringVar(value="2000")
        ttk.Entry(ctl, width=8, textvariable=self.capms_var).pack(side="left", padx=4)

        ttk.Label(ctl, text="Mode:").pack(side="left", padx=(10, 2))
        self.capmode_var = tk.IntVar(value=0)
        ttk.Radiobutton(ctl, text="restart per sample", value=0,
                        variable=self.capmode_var,
                        command=lambda: self._set_key("capmode", 0)).pack(side="left")
        ttk.Radiobutton(ctl, text="free-run", value=1,
                        variable=self.capmode_var,
                        command=lambda: self._set_key("capmode", 1)).pack(side="left")

        self.cap_btn = ttk.Button(ctl, text="Run Capture", command=self._run_capture)
        self.cap_btn.pack(side="left", padx=12)
        self.savecsv_btn = ttk.Button(ctl, text="Save CSV", command=self._save_capture_csv,
                                      state="disabled")
        self.savecsv_btn.pack(side="left")

        self.cap_result = ttk.Label(self.cap_tab, text="no capture yet",
                                    font=("Consolas", 11, "bold"), foreground="#046")
        self.cap_result.pack(anchor="w", padx=12)

        # ── Sweep ──
        swf = ttk.LabelFrame(self.cap_tab, text="Parameter sweep", padding=6)
        swf.pack(fill="x", padx=6, pady=6)
        ttk.Label(swf, text="Parameter:").pack(side="left")
        self.sweep_key = tk.StringVar(value="astep")
        scb = ttk.Combobox(swf, width=10, state="readonly",
                           values=list(SWEEP_PRESETS.keys()), textvariable=self.sweep_key)
        scb.pack(side="left", padx=4)
        scb.bind("<<ComboboxSelected>>", self._on_sweep_key)
        ttk.Label(swf, text="Values:").pack(side="left", padx=(10, 2))
        self.sweep_vals = tk.StringVar(value=SWEEP_PRESETS["astep"][1])
        ttk.Entry(swf, width=42, textvariable=self.sweep_vals).pack(side="left", padx=4)
        ttk.Label(swf, text="Window ms:").pack(side="left", padx=(10, 2))
        self.sweep_ms = tk.StringVar(value="1000")
        ttk.Entry(swf, width=7, textvariable=self.sweep_ms).pack(side="left")
        self.sweep_btn = ttk.Button(swf, text="Run Sweep", command=self._run_sweep)
        self.sweep_btn.pack(side="left", padx=10)
        ttk.Button(swf, text="Save sweep CSV",
                   command=self._save_sweep_csv).pack(side="left")

        body = ttk.Frame(self.cap_tab)
        body.pack(fill="both", expand=True, padx=6, pady=4)

        self.cap_fig = Figure(figsize=(8.0, 4.0), dpi=100)
        self.cap_ax = self.cap_fig.add_subplot(111)
        self.cap_canvas = FigureCanvasTkAgg(self.cap_fig, master=body)
        self.cap_canvas.get_tk_widget().pack(side="left", fill="both", expand=True)

        side = ttk.Frame(body)
        side.pack(side="right", fill="y", padx=(8, 0))

        plotf = ttk.LabelFrame(side, text="Plot channels", padding=4)
        plotf.pack(fill="x")
        self.plot_vars = {}
        for i, (name, _nm) in enumerate(CHANNELS):
            v = tk.BooleanVar(value=name in ("F1", "FZ", "F4", "FY", "F6", "NIR"))
            self.plot_vars[name] = v
            ttk.Checkbutton(plotf, text=name, variable=v,
                            command=self._draw_capture).grid(row=i % 9, column=i // 9,
                                                             sticky="w")

        swres = ttk.LabelFrame(side, text="Sweep results", padding=4)
        swres.pack(fill="both", expand=True, pady=(8, 0))
        self.sweep_tree = ttk.Treeview(swres, columns=("val", "hz", "n"),
                                       show="headings", height=10)
        self.sweep_tree.heading("val", text="value")
        self.sweep_tree.heading("hz", text="Hz")
        self.sweep_tree.heading("n", text="n")
        for c, w in (("val", 70), ("hz", 80), ("n", 60)):
            self.sweep_tree.column(c, width=w, anchor="e")
        self.sweep_tree.pack(fill="both", expand=True)
        self.sweep_rows = []

    def _on_sweep_key(self, _ev=None):
        self.sweep_vals.set(SWEEP_PRESETS[self.sweep_key.get()][1])

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
        self.after(600, lambda: (self._send("!PING"), self._send("!CFG")))

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

    def _push_chmask(self):
        mask = 0
        for i, v in enumerate(self.chan_vars):
            if v.get():
                mask |= 1 << i
        self.mask_lbl.config(text=f"mask 0x{mask:05X}")
        self._set_key("chmask", mask)

    def _mask_preset(self, which):
        for i, (name, nm) in enumerate(CHANNELS):
            if which == "all":
                val = True
            elif which == "none":
                val = False
            elif which == "spectral":
                val = nm is not None
            else:  # vis
                val = nm is None
            self.chan_vars[i].set(val)
        self._push_chmask()

    def _load_eeprom(self):
        self._send("!LOAD")
        self.after(300, lambda: self._send("!CFG"))

    def _restore_defaults(self):
        self._send("!DEFAULTS")
        self.after(300, lambda: self._send("!CFG"))

    def _apply_cfg_to_widgets(self):
        """Mirror device state into the widgets without echoing it back."""
        self._suppress_cfg_push = True
        try:
            g = self.cfg.get("gain")
            if g is not None and 0 <= g < len(GAIN_LABELS):
                self.gain_var.set(GAIN_LABELS[g])
            for key, var in (("atime", self.atime_var), ("astep", self.astep_var),
                             ("wtime", self.wtime_var), ("az", self.az_var),
                             ("ledma", self.ledma_var), ("thlow", self.thlow_var),
                             ("thhigh", self.thhigh_var), ("thch", self.thch_var),
                             ("pers", self.pers_var), ("streamms", self.streamms_var)):
                if key in self.cfg:
                    var.set(str(self.cfg[key]))
            if "smux" in self.cfg:
                self.smux_var.set(self.cfg["smux"])
            if "capms" in self.cfg:
                self.capms_var.set(str(self.cfg["capms"]))
            if "capmode" in self.cfg:
                self.capmode_var.set(self.cfg["capmode"])
            for key, var in (("waiten", self.waiten_var), ("led", self.led_var),
                             ("spint", self.spint_var)):
                if key in self.cfg:
                    var.set(bool(self.cfg[key]))
            if "chmask" in self.cfg:
                mask = self.cfg["chmask"]
                for i, v in enumerate(self.chan_vars):
                    v.set(bool(mask & (1 << i)))
                self.mask_lbl.config(text=f"mask 0x{mask:05X}")
        finally:
            self._suppress_cfg_push = False

        self.tint_lbl.config(text=f"tint: {self.tint_ms:.2f} ms")
        self.tint_ctrl_lbl.config(text=f"{self.tint_ms:.2f} ms")
        self._refresh_full_scale()

        self.cfg_tree.delete(*self.cfg_tree.get_children())
        for key in self.cfg:
            lo, hi = self.limits.get(key, ("", ""))
            self.cfg_tree.insert("", "end", text=key,
                                 values=(self.cfg[key], lo, hi))

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

        self._log(line)
        parts = line.split(",")
        tag = parts[0]

        if tag == "$HELLO" and len(parts) >= 3:
            self.fw_lbl.config(text=f"fw: {parts[1]} v{parts[2]}")

        elif tag == "$CFG" and len(parts) >= 5:
            key, val = parts[1], parts[2]
            if key == "tint_ms":
                self.tint_ms = float(val)
            elif key == "nchan":
                pass
            else:
                try:
                    self.cfg[key] = int(val)
                    self.limits[key] = (int(parts[3]), int(parts[4]))
                except ValueError:
                    pass

        elif tag == "$CFGEND":
            self._apply_cfg_to_widgets()
            self._cfg_event.set()

        elif tag == "$CH":
            names = parts[1:]
            if self._cap_active:
                self._cap_names = names
            else:
                self.enabled_names = names
                self._refresh_live_tree()

        elif tag == "$OK" and len(parts) >= 3:
            try:
                self.cfg[parts[1]] = int(parts[2])
            except ValueError:
                pass
            # Anything affecting timing or channel set invalidates our mirror.
            if parts[1] in ("gain", "atime", "astep", "smux", "chmask", "waiten", "wtime"):
                self._send("!CFG")

        elif tag == "$VAL" and len(parts) >= 3:
            try:
                self.cfg[parts[1]] = int(parts[2])
            except ValueError:
                pass

        elif tag == "$R" and len(parts) >= 4:
            self.live_asat = parts[2] == "1"
            self.live_dsat = parts[3] == "1"
            vals = parts[4:]
            names = self.enabled_names
            if len(names) != len(vals):
                # Channel set changed under us — ask for a fresh header, but only
                # once, or a mismatch during streaming would spam !CFG.
                if not self._chan_resync:
                    self._chan_resync = True
                    self._send("!CFG")
            else:
                self._chan_resync = False
                self.live_vals = {n: int(v) for n, v in zip(names, vals) if v.isdigit()}
            self._update_sat()
            self._draw_live()

        elif tag == "$FLK" and len(parts) >= 3:
            self.flicker_lbl.config(text=f"flicker: {parts[2]} Hz (st {parts[1]})")

        elif tag == "$CAPB":
            self._cap_active = True
            self._cap_rows = []
            self._cap_names = []
            self._cap_header = parts

        elif tag == "$CAPE":
            self._cap_active = False
            self._finish_capture()

    # ───────────────────────────────────────────────────────────── live plot

    def _update_sat(self):
        if self.live_dsat:
            self.sat_lbl.config(text="sat: DIGITAL", foreground="#c00")
        elif self.live_asat:
            self.sat_lbl.config(text="sat: analog", foreground="#c60")
        else:
            self.sat_lbl.config(text="sat: ok", foreground="#070")

    def _refresh_live_tree(self):
        self.live_tree.delete(*self.live_tree.get_children())
        for n in self.enabled_names:
            idx = NAME_TO_IDX.get(n)
            nm = CHANNELS[idx][1] if idx is not None else None
            self.live_tree.insert("", "end", iid=n,
                                  values=(nm if nm else "clear", "--"))

    def _full_scale(self):
        """AS7343 ADC full scale = (ATIME+1) x (ASTEP+1), capped at the 16-bit
        register limit.

        This is well below 65535 for most useful settings — the stock
        ATIME=29 / ASTEP=599 gives 18000 — so scaling the axis to 65535 would
        waste most of the plot. Falls back to 65535 only before the device has
        reported a config.
        """
        atime = self.cfg.get("atime")
        astep = self.cfg.get("astep")
        if atime is None or astep is None:
            return 65535
        return min((atime + 1) * (astep + 1), 65535)

    def _locked_ymax(self):
        """Fixed y-axis top for the live chart, or None to let it autoscale."""
        if not self.ylock_var.get():
            return None
        try:
            top = float(self.ymax_var.get())
        except ValueError:
            return None
        return top if top > 0 else None

    def _on_ylock(self):
        """Ticking the box freezes the axis where it currently sits, so the view
        does not jump. Untick to go back to autoscaling."""
        if self.ylock_var.get() and not self.ymax_var.get().strip():
            top = self.live_ax.get_ylim()[1]
            if not top or top <= 0:
                top = max(self.live_vals.values(), default=0) or self._full_scale()
            self.ymax_var.set(f"{top:.0f}")
        self._ymax_is_full = False
        self._draw_live()

    def _on_full_scale(self):
        """Lock to the ADC full scale, and keep tracking it — ATIME/ASTEP changes
        (including those a sweep makes) move full scale, so a fixed number would
        go stale."""
        self._ymax_is_full = True
        self.ymax_var.set(str(self._full_scale()))
        self.ylock_var.set(True)
        self._draw_live()

    def _apply_ymax(self):
        """Typing a value implies you want it locked, at that value specifically —
        so stop tracking full scale."""
        if self.ymax_var.get().strip():
            self.ylock_var.set(True)
            if self.ymax_var.get().strip() != str(self._full_scale()):
                self._ymax_is_full = False
        self._draw_live()

    def _refresh_full_scale(self):
        """Called whenever the device reports a config: relabel the button with
        the current full scale, and follow it if we are locked to it."""
        fs = self._full_scale()
        self.full_btn.config(text=f"Full scale ({fs})")
        if self._ymax_is_full and self.ymax_var.get() != str(fs):
            self.ymax_var.set(str(fs))
            self._draw_live()

    def _draw_live(self):
        names = [CHANNELS[i][0] for i in SPECTRUM_ORDER]
        vals = [self.live_vals.get(n, 0) for n in names]
        colors = [CHAN_COLOR[n] for n in names]

        self.live_ax.clear()
        self.live_ax.bar(range(len(names)), vals, color=colors)
        self.live_ax.set_xticks(range(len(names)))
        labels = []
        for n in names:
            nm = CHANNELS[NAME_TO_IDX[n]][1]
            labels.append(f"{n}\n{nm}")
        self.live_ax.set_xticklabels(labels, fontsize=8)
        self.live_ax.set_ylabel("counts")
        self.live_ax.set_title("AS7343 spectrum")

        log = self.logscale_var.get()
        if log:
            self.live_ax.set_yscale("log")

        top = self._locked_ymax()
        if top is not None:
            self.live_ax.set_ylim(bottom=1 if log else 0, top=top)
        elif log:
            self.live_ax.set_ylim(bottom=1)
        self.live_ax.grid(axis="y", alpha=0.3)
        self.live_fig.tight_layout()
        self.live_canvas.draw_idle()

        for n in self.enabled_names:
            if self.live_tree.exists(n):
                self.live_tree.set(n, "val", self.live_vals.get(n, 0))

    # ─────────────────────────────────────────────────────────────── capture

    def _run_capture(self):
        if not self.sm.is_open():
            messagebox.showwarning("Not connected", "Connect to the board first.")
            return
        if self.stream_var.get():
            self.stream_var.set(False)
            self._send("!STREAM,0")
        try:
            ms = int(self.capms_var.get())
        except ValueError:
            messagebox.showerror("Bad window", "Capture window must be an integer (ms).")
            return
        self.cap_btn.config(state="disabled")
        self.cap_result.config(text="capturing...")
        self._cap_event.clear()
        self._send(f"!CAP,{ms}")
        # Watchdog: a very slow config plus the sample dump can take a while,
        # but never leave the button stuck if the board goes quiet.
        self.after(ms + 90000, self._capture_watchdog)

    def _capture_watchdog(self):
        if str(self.cap_btn["state"]) == "disabled" and not self._cap_event.is_set():
            self._cap_active = False
            self.cap_btn.config(state="normal")
            self.cap_result.config(text="capture timed out — no $CAPE from device")

    def _finish_capture(self):
        hdr = self._cap_header or []
        names = self._cap_names
        rows = self._cap_rows

        t_us, data = [], []
        for r in rows:
            f = r.split(",")
            if len(f) != len(names) + 1:
                continue
            try:
                t_us.append(int(f[0]))
                data.append([int(x) for x in f[1:]])
            except ValueError:
                continue

        meta = {}
        if len(hdr) >= 12:
            keys = ["n", "nchan", "mask", "elapsed_us", "rate_hz", "mode",
                    "gain", "atime", "astep", "smux", "tint_ms"]
            for k, v in zip(keys, hdr[1:12]):
                meta[k] = v

        self.capture = {
            "names": names,
            "t_us": np.array(t_us, dtype=np.int64),
            "data": np.array(data, dtype=np.int32) if data else np.zeros((0, len(names)), np.int32),
            "meta": meta,
        }

        n = len(t_us)
        rate = float(meta.get("rate_hz", 0) or 0)
        elapsed = float(meta.get("elapsed_us", 0) or 0) / 1000.0
        jitter = ""
        if n > 2:
            d = np.diff(self.capture["t_us"]) / 1000.0
            jitter = (f"  interval {d.mean():.3f} ms "
                      f"(min {d.min():.3f} / max {d.max():.3f} / sd {d.std():.3f})")
        self.cap_result.config(
            text=(f"{n} samples over {elapsed:.1f} ms  ->  {rate:.2f} Hz   "
                  f"[mode {meta.get('mode','?')}, gain idx {meta.get('gain','?')}, "
                  f"ATIME {meta.get('atime','?')}, ASTEP {meta.get('astep','?')}, "
                  f"SMUX {meta.get('smux','?')}, tint {meta.get('tint_ms','?')} ms]{jitter}"))

        self.cap_btn.config(state="normal")
        self.savecsv_btn.config(state="normal" if n else "disabled")
        self._draw_capture()
        self._cap_event.set()

    def _draw_capture(self):
        self.cap_ax.clear()
        cap = self.capture
        if not cap or cap["data"].shape[0] == 0:
            self.cap_ax.set_title("no capture data")
            self.cap_canvas.draw_idle()
            return

        t_ms = cap["t_us"] / 1000.0
        plotted = 0
        for j, name in enumerate(cap["names"]):
            var = self.plot_vars.get(name)
            if var is None or not var.get():
                continue
            self.cap_ax.plot(t_ms, cap["data"][:, j], lw=1.0,
                             color=CHAN_COLOR.get(name, "#333"), label=name)
            plotted += 1

        self.cap_ax.set_xlabel("t (ms)")
        self.cap_ax.set_ylabel("counts")
        rate = cap["meta"].get("rate_hz", "?")
        self.cap_ax.set_title(f"Capture — {cap['data'].shape[0]} samples @ {rate} Hz")
        if plotted:
            self.cap_ax.legend(fontsize=8, ncol=max(1, plotted // 6 + 1))
        self.cap_ax.grid(alpha=0.3)
        self.cap_fig.tight_layout()
        self.cap_canvas.draw_idle()

    def _save_capture_csv(self):
        cap = self.capture
        if not cap or cap["data"].shape[0] == 0:
            return
        default = f"as7343_cap_{datetime.now():%Y%m%d_%H%M%S}.csv"
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
                w.writerow([int(cap["t_us"][i])] + list(map(int, cap["data"][i])))
        self._log(f"# saved {os.path.basename(path)}")

    # ───────────────────────────────────────────────────────────────── sweep

    def _run_sweep(self):
        if not self.sm.is_open():
            messagebox.showwarning("Not connected", "Connect to the board first.")
            return
        key = self.sweep_key.get()
        try:
            values = [int(v.strip()) for v in self.sweep_vals.get().split(",") if v.strip()]
            window = int(self.sweep_ms.get())
        except ValueError:
            messagebox.showerror("Bad sweep", "Values must be a comma-separated integer list.")
            return
        if not values:
            return

        self.sweep_btn.config(state="disabled")
        self.cap_btn.config(state="disabled")
        self.sweep_tree.delete(*self.sweep_tree.get_children())
        self.sweep_rows = []
        threading.Thread(target=self._sweep_worker,
                         args=(key, values, window), daemon=True).start()

    def _sweep_worker(self, key, values, window):
        """Runs off the Tk thread; waits on events set by the main-thread pump."""
        original = self.cfg.get(key)
        for v in values:
            self._cfg_event.clear()
            self._call_soon(lambda k=key, val=v: self._send(f"!SET,{k},{val}"))
            self._call_soon(lambda: self._send("!CFG"))
            self._cfg_event.wait(timeout=5.0)

            self._cap_event.clear()
            self._call_soon(lambda w=window: self._send(f"!CAP,{w}"))
            # Generous: a slow config can take many seconds of integration, and
            # the board dumps every sample as a text line afterwards.
            if not self._cap_event.wait(timeout=window / 1000.0 + 90.0):
                self._call_soon(lambda: self._log("# sweep: capture timed out"))
                break
            cap = self.capture
            rate = float(cap["meta"].get("rate_hz", 0) or 0)
            n = cap["data"].shape[0]
            actual = self.cfg.get(key, v)
            self.sweep_rows.append((actual, rate, n))
            self._call_soon(lambda a=actual, r=rate, nn=n:
                            self.sweep_tree.insert("", "end", values=(a, f"{r:.2f}", nn)))

        if original is not None:
            self._call_soon(lambda: self._send(f"!SET,{key},{original}"))
        self._call_soon(self._sweep_done)

    def _sweep_done(self):
        self.sweep_btn.config(state="normal")
        self.cap_btn.config(state="normal")
        if not self.sweep_rows:
            return
        xs = [r[0] for r in self.sweep_rows]
        ys = [r[1] for r in self.sweep_rows]
        self.cap_ax.clear()
        self.cap_ax.plot(xs, ys, "o-", color="#1f77b4")
        self.cap_ax.set_xlabel(SWEEP_PRESETS[self.sweep_key.get()][0])
        self.cap_ax.set_ylabel("achieved rate (Hz)")
        self.cap_ax.set_title("Sample rate vs " + SWEEP_PRESETS[self.sweep_key.get()][0])
        self.cap_ax.grid(alpha=0.3)
        self.cap_fig.tight_layout()
        self.cap_canvas.draw_idle()
        self._send("!CFG")

    def _save_sweep_csv(self):
        if not self.sweep_rows:
            return
        default = f"as7343_sweep_{datetime.now():%Y%m%d_%H%M%S}.csv"
        path = filedialog.asksaveasfilename(defaultextension=".csv",
                                            initialfile=default,
                                            filetypes=[("CSV", "*.csv")])
        if not path:
            return
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([self.sweep_key.get(), "rate_hz", "n_samples"])
            w.writerows(self.sweep_rows)
        self._log(f"# saved {os.path.basename(path)}")

    # ───────────────────────────────────────────────────────────────── stream

    def _on_stream(self):
        self._send("!STREAM," + ("1" if self.stream_var.get() else "0"))


if __name__ == "__main__":
    App().mainloop()
