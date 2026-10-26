"""
monitor_gui.py  --  control panel for the 120VMonitor (RA4M1 bench mains monitor).

    py -3 monitor_gui.py            (pip install pyserial matplotlib numpy)

Port box: a COM port, or SIM for the built-in simulator (monitor_sim.py).

Tabs
  Meter        live readings + trends from the board's metering windows, CSV log
  Capture      fast raw captures (up to ~7k samples), triggers incl. relay-close
               inrush at a chosen phase, harmonics, CSV export
  Calibration  bench supply (Virtual Bench or direct SCPI) DC sweeps for V and I,
               phase from mains + resistive load, apply / save to EEPROM
  Config       every firmware key (built from the board's !CFG dump); snapshot
               export / apply by group (calibration, fuses, wiring …), and a
               snapshot logged to config_log/ on every connect and save
  Console      raw protocol log and a command line
"""

import csv
import os
import queue
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import filedialog, messagebox, ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

import bench_psu
import calibrate as cal
from monitor_link import (CLOSE_TRIGS, CURRENT_TRIGS, CFG_GROUPS, GROUP_INFO, GROUP_OF, RANGE_MODES, RANGE_NAMES, SAFE_WHY,
                          TRIG_MODES, LinkError, MonitorLink, Snapshot, capture_stats, flag_text,
                          log_snapshot, same_value)

HERE = os.path.dirname(os.path.abspath(__file__))
CFG_LOG_DIR = os.path.join(HERE, "config_log")   # a snapshot per board on connect and on save
TIMING_MODES = ["Rate + samples", "Rate + duration", "Samples + duration"]
CAP_MAX = 7168                  # int16 values in the XIAO's capture buffer; $STATUS capmax wins
RANGE_FULL = {0: "HIGH ±25 A", 1: "LOW ±2.5 A"}
CAP_RANGES = ["Follow mode", "HIGH", "LOW"]   # !CAP rng -1 / 0 / 1
CAL_IPTS = {0: "0.25, 0.5, 1.0, 1.5, 2.0, 2.5", 1: "0.1, 0.25, 0.5, 1.0, 1.5, 2.0"}

BG, FG, ACC = "#1A1A1A", "#00FF32", "#FFB000"


def list_ports():
    try:
        from serial.tools import list_ports as lp
        return [p.device for p in lp.comports()] + ["SIM"]
    except ImportError:
        return ["SIM"]


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("120V Monitor")
        self.geometry("1200x820")
        self.link = None
        self.psu = None
        self.busy = False
        self.hist = deque(maxlen=36000)        # $PWR records
        self.csv_fh = self.csv_w = None
        self.last_cap = None
        self.cal_v = self.cal_i = self.cal_ilo = None
        self.cal_phase = None
        self.stop_evt = threading.Event()
        self.repeat_job = None
        self.safe_msg = ""                     # sticky until the next relay action / capture
        self.uiq = queue.Queue()               # callbacks from worker threads
        self._build()
        self.after(100, self._pump)
        self.after(500, self._redraw_trends)
        self.protocol("WM_DELETE_WINDOW", self._quit)

    # ═══ layout ═════════════════════════════════════════════════════════════
    def _build(self):
        top = ttk.Frame(self, padding=4)
        top.pack(fill="x")
        ttk.Label(top, text="Port").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var, width=10, values=list_ports())
        self.port_cb.pack(side="left", padx=2)
        if self.port_cb["values"]:
            self.port_cb.current(0)
        ttk.Button(top, text="↻", width=3,
                   command=lambda: self.port_cb.configure(values=list_ports())).pack(side="left")
        self.conn_btn = ttk.Button(top, text="Connect", command=self._toggle_connect)
        self.conn_btn.pack(side="left", padx=4)
        self.relay_btn = tk.Button(top, text="LOAD OFF", width=10, bg="#444", fg="white",
                                   command=self._toggle_relay, state="disabled")
        self.relay_btn.pack(side="left", padx=8)
        self.stream_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text="Stream", variable=self.stream_var,
                        command=self._stream_changed).pack(side="left")
        ttk.Label(top, text="Range").pack(side="left", padx=(12, 2))
        self.rng_mode = tk.StringVar(value=RANGE_MODES[0])
        self.rng_cb = ttk.Combobox(top, textvariable=self.rng_mode, values=list(RANGE_MODES.values()),
                                   width=9, state="disabled")
        self.rng_cb.pack(side="left")
        self.rng_cb.bind("<<ComboboxSelected>>", self._range_selected)
        self.rng_lbl = tk.Label(top, text="", width=24, anchor="w", font=("Segoe UI", 10, "bold"))
        self.rng_lbl.pack(side="left", padx=4)
        self.flag_lbl = tk.Label(top, text="", fg="red", font=("Segoe UI", 10, "bold"))
        self.flag_lbl.pack(side="left", padx=10)
        self.status_lbl = ttk.Label(top, text="not connected")
        self.status_lbl.pack(side="right")

        self.nb = ttk.Notebook(self)
        self.nb.pack(fill="both", expand=True)
        self._build_meter()
        self._build_capture()
        self._build_cal()
        self._build_config()
        self._build_console()

    # ── Meter ───────────────────────────────────────────────────────────────
    def _build_meter(self):
        f = ttk.Frame(self.nb)
        self.nb.add(f, text="Meter")
        panel = tk.Frame(f, bg=BG)
        panel.pack(fill="x")
        self.read_vars = {}
        items = [("vrms", "V rms", "V", 2), ("irms", "I rms", "A", 4), ("p", "Power", "W", 2),
                 ("pf", "PF", "", 3), ("hz", "Freq", "Hz", 3), ("s", "Apparent", "VA", 2),
                 ("q", "Non-active", "var", 2), ("vpk", "V peak", "V", 1), ("ipk", "I peak", "A", 3),
                 ("crest", "I crest", "", 2), ("wh", "Energy", "Wh", 4), ("vdc", "V dc", "V", 3),
                 ("idc", "I dc", "A", 4)]
        for k, (key, label, unit, dp) in enumerate(items):
            r, c = divmod(k, 7)
            cell = tk.Frame(panel, bg=BG, padx=10, pady=4)
            cell.grid(row=r, column=c, sticky="w")
            tk.Label(cell, text=label, bg=BG, fg="#9a9a9a", font=("Segoe UI", 9)).pack(anchor="w")
            var = tk.StringVar(value="---")
            big = 22 if r == 0 and c < 3 else 15
            tk.Label(cell, textvariable=var, bg=BG, fg=FG, font=("Consolas", big, "bold")).pack(anchor="w")
            self.read_vars[key] = (var, unit, dp)
        bar = ttk.Frame(f, padding=4)
        bar.pack(fill="x")
        ttk.Label(bar, text="Trend span").pack(side="left")
        self.span_var = tk.StringVar(value="2 min")
        ttk.Combobox(bar, textvariable=self.span_var, width=8, state="readonly",
                     values=["30 s", "2 min", "10 min", "1 h", "all"]).pack(side="left", padx=4)
        ttk.Button(bar, text="Clear", command=lambda: self.hist.clear()).pack(side="left", padx=4)
        ttk.Button(bar, text="Reset Wh", command=lambda: self._cmd("!EZERO")).pack(side="left", padx=4)
        self.log_btn = ttk.Button(bar, text="Start CSV log…", command=self._toggle_log)
        self.log_btn.pack(side="left", padx=12)
        self.log_lbl = ttk.Label(bar, text="")
        self.log_lbl.pack(side="left")

        self.tfig = Figure(figsize=(8, 5), dpi=96)
        self.tax = self.tfig.subplots(2, 1, sharex=True)
        self.tax_i = self.tax[0].twinx()
        self.tax_pf = self.tax[1].twinx()
        self.tcanvas = FigureCanvasTkAgg(self.tfig, master=f)
        self.tcanvas.get_tk_widget().pack(fill="both", expand=True)

    # ── Capture ─────────────────────────────────────────────────────────────
    def _build_capture(self):
        f = ttk.Frame(self.nb)
        self.nb.add(f, text="Capture")
        ctl = ttk.Frame(f, padding=4)
        ctl.pack(fill="x")
        self.cap_vars = {}

        def field(row, col, label, key, default, width=8, values=None):
            ttk.Label(ctl, text=label).grid(row=row, column=col, sticky="e", padx=(8, 2))
            var = tk.StringVar(value=str(default))
            if values:
                w = ttk.Combobox(ctl, textvariable=var, values=values, width=width + 10, state="readonly")
            else:
                w = ttk.Entry(ctl, textvariable=var, width=width)
            w.grid(row=row, column=col + 1, sticky="w")
            self.cap_vars[key] = var
            return w

        self.cap_entries = {"rate": field(0, 0, "Rate (S/s, 0=max)", "rate", 20000),
                            "n": field(0, 2, "Samples", "n", 3000)}
        field(0, 4, "Channels", "ch", "V + I", values=["V + I", "V only", "I only"])
        field(0, 6, "Trigger", "trig", TRIG_MODES[2], values=list(TRIG_MODES.values()))
        field(1, 0, "Level (A, V; dropout A rms)", "level", 0.5)
        field(1, 2, "Pretrigger %", "pre", 10)
        field(1, 4, "Timeout (s, 0 = none)", "timeout", 5)
        field(1, 6, "Close delay after V rising zero (µs)", "relay_us", 0)
        field(0, 8, "Average N scans/sample", "avg", 7, width=4)
        field(1, 8, "Current range", "rng", CAP_RANGES[0], width=4, values=CAP_RANGES)
        # timing: any two of rate / samples / duration, the third is worked out
        field(2, 0, "Set by", "tmode", TIMING_MODES[0], width=8, values=TIMING_MODES)
        self.cap_entries["dur"] = field(2, 2, "Duration (ms)", "dur", 150)
        self.timing_note = ttk.Label(ctl, text="", foreground="#b05000")
        self.timing_note.grid(row=2, column=4, columnspan=6, sticky="w", padx=8)
        self.spike_lbl = ttk.Label(ctl, text="")
        self.spike_lbl.grid(row=3, column=0, columnspan=10, sticky="w", padx=8)
        self._timing_busy = False
        for k in ("rate", "n", "dur", "tmode", "ch", "avg"):
            self.cap_vars[k].trace_add("write", self._timing_update)
        self._timing_update()

        safe = ttk.LabelFrame(f, text="Safe start — the board opens the relay itself (recording continues)",
                              padding=4)
        safe.pack(fill="x", padx=4, pady=(4, 2))
        self.safe = {}

        def opt(col, key, label, entries):
            on = tk.BooleanVar(value=False)
            ttk.Checkbutton(safe, text=label, variable=on).grid(row=0, column=col, sticky="w", padx=(6, 2))
            self.safe[key] = on
            c = col + 1
            for ekey, default, unit in entries:
                var = tk.StringVar(value=str(default))
                ttk.Entry(safe, textvariable=var, width=6).grid(row=0, column=c, sticky="w")
                ttk.Label(safe, text=unit).grid(row=0, column=c + 1, sticky="w", padx=(1, 6))
                self.safe[ekey] = var
                c += 2
            return c

        c = opt(0, "ipk_on", "Instant trip above", [("ipk_a", 10, "A")])
        c = opt(c, "ov_on", "Over-current above", [("ov_a", 5, "A rms for"), ("ov_cyc", 3, "cycles")])
        c = opt(c, "auto_on", "Auto-open", [("auto_ms", 500, "ms after trigger")])
        self.safe["open_after"] = tk.BooleanVar(value=False)
        ttk.Checkbutton(safe, text="Open relay when capture completes",
                        variable=self.safe["open_after"]).grid(row=0, column=c, sticky="w", padx=6)
        btns = ttk.Frame(f, padding=(4, 0))
        btns.pack(fill="x")
        self.cap_btn = ttk.Button(btns, text="Capture", command=self._capture)
        self.cap_btn.pack(side="left")
        self.abort_btn = ttk.Button(btns, text="Abort", command=self._abort, state="disabled")
        self.abort_btn.pack(side="left", padx=(4, 0))
        self.mon_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(btns, text="Readings while armed (meter + log keep going; lower max rate)",
                        variable=self.mon_var).pack(side="left", padx=6)
        self.repeat_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(btns, text="Repeat", variable=self.repeat_var).pack(side="left", padx=6)
        ttk.Button(btns, text="Save CSV…", command=self._save_capture).pack(side="left", padx=6)
        self.show_harm = tk.BooleanVar(value=True)
        ttk.Checkbutton(btns, text="Harmonics", variable=self.show_harm,
                        command=lambda: self._plot_capture(self.last_cap)).pack(side="left", padx=6)
        self.rate_lbl = ttk.Label(btns, text="")
        self.rate_lbl.pack(side="left", padx=12)
        self.cap_stats = tk.StringVar(value="")
        tk.Label(f, textvariable=self.cap_stats, font=("Consolas", 10), justify="left",
                 anchor="w").pack(fill="x", padx=6)

        self.cfig = Figure(figsize=(8, 5), dpi=96)
        self.ccanvas = FigureCanvasTkAgg(self.cfig, master=f)
        NavigationToolbar2Tk(self.ccanvas, f).pack(fill="x")
        self.ccanvas.get_tk_widget().pack(fill="both", expand=True)

    # ── Calibration ─────────────────────────────────────────────────────────
    def _build_cal(self):
        f = ttk.Frame(self.nb)
        self.nb.add(f, text="Calibration")
        left = ttk.Frame(f, padding=6)
        left.pack(side="left", fill="y")

        box = ttk.LabelFrame(left, text="Bench supply", padding=6)
        box.pack(fill="x")
        self.psu_mode = tk.StringVar(value="auto")
        for txt, val in (("Auto (Virtual Bench, else direct)", "auto"), ("Virtual Bench only", "bench"),
                         ("Direct SCPI", "direct"), ("Simulated", "sim")):
            ttk.Radiobutton(box, text=txt, value=val, variable=self.psu_mode).pack(anchor="w")
        row = ttk.Frame(box)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text="Host").pack(side="left")
        self.psu_host = tk.StringVar(value=f"{bench_psu.PSU_HOST}:{bench_psu.PSU_PORT}")
        ttk.Entry(row, textvariable=self.psu_host, width=20).pack(side="left", padx=4)
        row = ttk.Frame(box)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text="Channel wired to the 120 V input").pack(side="left")
        self.psu_ch = tk.StringVar(value="2")
        ttk.Combobox(row, textvariable=self.psu_ch, values=["1", "2", "3", "4"], width=3,
                     state="readonly").pack(side="left", padx=4)
        self.psu_btn = ttk.Button(box, text="Connect supply", command=self._psu_connect)
        self.psu_btn.pack(anchor="w", pady=2)
        self.psu_lbl = ttk.Label(box, text="not connected", wraplength=300)
        self.psu_lbl.pack(anchor="w")
        self.mains_lim = tk.StringVar(value="5")
        self._entry_row(box, "Refuse above (VAC)", self.mains_lim, 8)

        box = ttk.LabelFrame(left, text="1. Voltage (mains DISCONNECTED, relay open)", padding=6)
        box.pack(fill="x", pady=6)
        self.vpts = tk.StringVar(value="0, 5, 10, 15, 20, 25, 30")
        self.vilim = tk.StringVar(value="0.05")
        self._entry_row(box, "Setpoints (V)", self.vpts, 26)
        self._entry_row(box, "Current limit (A)", self.vilim, 8)
        ttk.Button(box, text="Run voltage sweep", command=self._cal_voltage).pack(anchor="w", pady=2)

        box = ttk.LabelFrame(left, text="2. Current (short / power resistor in the outlet)", padding=6)
        box.pack(fill="x", pady=6)
        row = ttk.Frame(box)
        row.pack(fill="x", pady=1)
        ttk.Label(row, text="Range", width=16).pack(side="left")
        self.cal_rng = tk.StringVar(value=RANGE_FULL[0])
        cb = ttk.Combobox(row, textvariable=self.cal_rng, values=list(RANGE_FULL.values()), width=14,
                          state="readonly")
        cb.pack(side="left")
        cb.bind("<<ComboboxSelected>>",
                lambda _e: self.ipts.set(CAL_IPTS[self._cal_rng_idx()]))
        self.ipts = tk.StringVar(value=CAL_IPTS[0])
        self.ivlim = tk.StringVar(value="5")
        self._entry_row(box, "Setpoints (A)", self.ipts, 26)
        self._entry_row(box, "Voltage limit (V)", self.ivlim, 8)
        ttk.Button(box, text="Run current sweep", command=self._cal_current).pack(anchor="w", pady=2)

        box = ttk.LabelFrame(left, text="3. Phase (MAINS + resistive load, supply removed)", padding=6)
        box.pack(fill="x", pady=6)
        ttk.Button(box, text="Measure V/I skew", command=self._cal_phase).pack(anchor="w", pady=2)

        box = ttk.LabelFrame(left, text="Results", padding=6)
        box.pack(fill="x", pady=6)
        ttk.Button(box, text="Apply to board (RAM)", command=self._cal_apply).pack(anchor="w", pady=1)
        ttk.Button(box, text="Save to EEPROM", command=self._cal_save).pack(anchor="w", pady=1)
        ttk.Button(box, text="Stop", command=self.stop_evt.set).pack(anchor="w", pady=1)
        self.sim_mains = tk.BooleanVar(value=True)
        self.sim_cb = ttk.Checkbutton(box, text="(simulator) mains connected", variable=self.sim_mains,
                                      command=self._sim_mains)

        right = ttk.Frame(f, padding=6)
        right.pack(side="left", fill="both", expand=True)
        self.cal_txt = tk.Text(right, height=18, font=("Consolas", 9), wrap="none")
        self.cal_txt.pack(fill="x")
        self.kfig = Figure(figsize=(6, 3.6), dpi=96)
        self.kax = self.kfig.subplots(1, 2)
        self.kcanvas = FigureCanvasTkAgg(self.kfig, master=right)
        self.kcanvas.get_tk_widget().pack(fill="both", expand=True)

    def _entry_row(self, parent, label, var, width):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=1)
        ttk.Label(row, text=label, width=16).pack(side="left")
        ttk.Entry(row, textvariable=var, width=width).pack(side="left")

    # ── Config ──────────────────────────────────────────────────────────────
    def _build_config(self):
        f = ttk.Frame(self.nb)
        self.nb.add(f, text="Config")
        bar = ttk.Frame(f, padding=4)
        bar.pack(fill="x")
        for txt, fn in (("Refresh", self._cfg_refresh), ("Save to EEPROM", lambda: self._cmd("!SAVE")),
                        ("Reload EEPROM", lambda: self._cmd_refresh("!LOAD")),
                        ("Factory defaults", self._cfg_defaults),
                        ("Zero offsets (no mains, no load)", lambda: self._cmd_refresh("!ZERO"))):
            ttk.Button(bar, text=txt, command=fn).pack(side="left", padx=3)
        self.dirty_lbl = ttk.Label(bar, text="", foreground="#c60")
        self.dirty_lbl.pack(side="left", padx=10)
        bar2 = ttk.Frame(f, padding=(4, 0, 4, 4))
        bar2.pack(fill="x")
        ttk.Button(bar2, text="Export snapshot…", command=self._cfg_export).pack(side="left", padx=3)
        ttk.Button(bar2, text="Apply snapshot…", command=self._cfg_apply).pack(side="left", padx=3)
        ttk.Button(bar2, text="Open log folder",
                   command=lambda: (os.makedirs(CFG_LOG_DIR, exist_ok=True), os.startfile(CFG_LOG_DIR))
                   ).pack(side="left", padx=3)
        self.tree = ttk.Treeview(f, columns=("group", "value", "range", "help"), show="tree headings",
                                 height=20)
        self.tree.heading("#0", text="Key")
        self.tree.heading("group", text="Group")
        self.tree.column("group", width=70)
        self.tree.heading("value", text="Value")
        self.tree.heading("range", text="Range")
        self.tree.heading("help", text="Meaning")
        self.tree.column("#0", width=110)
        self.tree.column("value", width=140)
        self.tree.column("range", width=160)
        self.tree.column("help", width=400)
        self.tree.pack(fill="both", expand=True, padx=4)
        self.tree.bind("<Double-1>", self._cfg_edit)
        ttk.Label(f, text="Double-click a value to change it (RAM, takes effect at once); "
                          "Save to EEPROM keeps it.").pack(anchor="w", padx=6, pady=4)

    # ── Console ─────────────────────────────────────────────────────────────
    def _build_console(self):
        f = ttk.Frame(self.nb)
        self.nb.add(f, text="Console")
        self.con = tk.Text(f, font=("Consolas", 9), height=30)
        self.con.pack(fill="both", expand=True)
        row = ttk.Frame(f, padding=4)
        row.pack(fill="x")
        self.con_cmd = tk.StringVar()
        e = ttk.Entry(row, textvariable=self.con_cmd)
        e.pack(side="left", fill="x", expand=True)
        e.bind("<Return>", lambda _e: self._con_send())
        ttk.Button(row, text="Send", command=self._con_send).pack(side="left", padx=4)

    # ═══ connection ═════════════════════════════════════════════════════════
    def _toggle_connect(self):
        if self.link:
            self._disconnect()
            return
        port = self.port_var.get().strip()
        if not port:
            return
        self.conn_btn.configure(state="disabled")
        self.status_lbl.configure(text=f"opening {port}…")

        def work():
            try:
                link = MonitorLink(port)
                try:
                    ident = link.identify()
                except LinkError:
                    link.close()
                    raise LinkError(f"{port} opened but no 120VMonitor answered !ID")
                link.read_cfg()
                link.get_status()
                self.ui(self._connected, link, ident)
            except Exception as e:                         # noqa: BLE001
                self.ui(self._connect_failed, str(e))
        threading.Thread(target=work, daemon=True).start()

    def _connected(self, link, ident):
        self.link = link
        self.conn_btn.configure(state="normal", text="Disconnect")
        self.relay_btn.configure(state="normal")
        st = link.status
        self.status_lbl.configure(text=f"{self.port_var.get()}  {ident}  scan {st.get('scanus4', '?')} µs")
        self._spike_update()                  # now with the board's real scan times
        self._timing_update()                 # and its real capture buffer size
        try:
            s4 = float(st.get("scanus4", 0)); s2 = float(st.get("scanus2", 0))
            self.rate_lbl.configure(text=f"max ≈ {1e6 / s4:,.0f} S/s (V+I), {1e6 / s2:,.0f} S/s (one channel)"
                                    if s4 > 0 and s2 > 0 else "")
        except ValueError:
            pass
        self._set_relay_ui(st.get("relay") == "1")
        self._range_ui(st.get("rng"), st.get("rngmode"))
        self._fill_cfg()
        self._log_config("connect")
        if self.port_var.get().upper() == "SIM":
            self.sim_cb.pack(anchor="w", pady=4)
            self.psu_mode.set("sim")
        if self.stream_var.get():
            self._cmd("!START")

    def _connect_failed(self, msg):
        self.conn_btn.configure(state="normal")
        self.status_lbl.configure(text="not connected")
        messagebox.showerror("Connect", msg)

    def _disconnect(self):
        if self.repeat_job:
            self.after_cancel(self.repeat_job)
            self.repeat_job = None
        link, self.link = self.link, None
        try:
            link.relay(False)                 # leave the load off when the GUI lets go
        except Exception:                                  # noqa: BLE001
            pass
        link.close()
        self.conn_btn.configure(text="Connect")
        self.relay_btn.configure(state="disabled")
        self.rng_cb.configure(state="disabled")
        self.rng_lbl.configure(text="")
        self.status_lbl.configure(text="not connected")

    def _quit(self):
        if self.link:
            self._disconnect()
        if self.psu:
            self.psu.close()
        if self.csv_fh:
            self.csv_fh.close()
        self.destroy()

    def _cmd(self, cmd):
        if not self.link:
            return
        try:
            self.link.send(cmd)
        except Exception as e:                             # noqa: BLE001
            messagebox.showerror("Serial", str(e))

    def _cmd_refresh(self, cmd):
        self._cmd(cmd)
        self.after(400, self._cfg_refresh)

    def _stream_changed(self):
        self._cmd("!START" if self.stream_var.get() else "!STOP")

    def _toggle_relay(self):
        self.safe_msg = ""
        on = self.relay_btn.cget("text") == "LOAD OFF"
        self._cmd(f"!RELAY,{1 if on else 0}")

    # ── current range ───────────────────────────────────────────────────────
    def _range_selected(self, _e=None):
        mode = [k for k, n in RANGE_MODES.items() if n == self.rng_mode.get()][0]
        self._cmd(f"!RANGE,{mode}")

    def _range_ui(self, rng, mode=None, note=""):
        """rng/mode as reported by the board (str or int, None = unknown)."""
        if not self.link:
            return
        two = bool(self.link.cfg.get("RNGPIN", {}).get("value"))
        self.rng_cb.configure(state="readonly" if two else "disabled")
        if mode is not None:
            self.rng_mode.set(RANGE_MODES.get(int(mode), RANGE_MODES[0]))
        if not two:
            self.rng_lbl.configure(text="single range (RNGPIN 0)", fg="#777")
            return
        if rng is None:
            return
        rng = int(rng)
        if note or rng != getattr(self, "_rng_now", None):
            self.rng_lbl.configure(text=RANGE_FULL[rng] + (f"  {note}" if note else ""),
                                   fg="#1565c0" if rng else "#444")
        self._rng_now = rng

    def _set_relay_ui(self, on):
        self.relay_btn.configure(text="LOAD ON" if on else "LOAD OFF",
                                 bg="#c62828" if on else "#444")

    # ═══ event pump ═════════════════════════════════════════════════════════
    def ui(self, fn, *args):
        """Run fn(*args) on the Tk thread (Tk calls from workers are not safe)."""
        self.uiq.put((fn, args))

    def _pump(self):
        try:
            while True:
                fn, args = self.uiq.get_nowait()
                fn(*args)
        except queue.Empty:
            pass
        if self.link:
            try:
                for _ in range(500):
                    kind, obj = self.link.events.get_nowait()
                    if kind == "pwr":
                        self._on_pwr(obj)
                    elif kind == "line":
                        self._on_line(obj)
                    elif kind == "tx":
                        self._con_add("> " + obj)
                    elif kind == "error":
                        self._con_add("!! " + obj)
                        self.status_lbl.configure(text=obj)
            except queue.Empty:
                pass
        self.after(100, self._pump)

    def _on_line(self, line):
        self._con_add(line)
        if line.startswith("$RLY,"):
            self._set_relay_ui(line.endswith("1"))
        elif line.startswith("$RNG,"):
            p = line[5:].split(",")
            note = f"(overload {float(p[3]):.2f} A)" if p[1] == "OVER" and len(p) > 3 else \
                {"AUTO": "(auto)", "CLOSE": "(load closing)", "CAP": "(capture)"}.get(p[1], "")
            self._rng_now = None
            self._range_ui(p[0], p[2], note)
        elif line.startswith("$TRIP"):
            self.flag_lbl.configure(text="TRIPPED: " + line[6:])
        elif line.startswith("$REMOTE,"):
            ev = line[8:]
            self.safe_msg = {"PRIMED": "REMOTE: primed — press * to close",
                             "CLOSE": "", "CANCEL": "", "TIMEOUT": "",
                             "OPEN": "REMOTE opened the relay"}.get(ev, "REMOTE: " + ev)
            self.flag_lbl.configure(text=self.safe_msg)
        elif line.startswith("$SAFE,"):
            why, _, amps = line[6:].partition(",")
            self.safe_msg = (f"SAFE START opened relay: {why}"
                             + (f" ({float(amps):.2f} A)" if why in ("INST", "OVERCURRENT") else ""))
            self.flag_lbl.configure(text=self.safe_msg)
        elif line.startswith("$CFG,") or line.startswith("$OK"):
            self._fill_cfg()
            if line == "$OK,save":
                self._log_config("save")
        elif line.startswith("$ERR"):
            self.status_lbl.configure(text=line)

    def _con_add(self, s):
        self.con.insert("end", s + "\n")
        if int(self.con.index("end-1c").split(".")[0]) > 3000:
            self.con.delete("1.0", "1000.0")
        self.con.see("end")

    def _con_send(self):
        c = self.con_cmd.get().strip()
        if c:
            self._cmd(c)
            self.con_cmd.set("")

    def _on_pwr(self, r):
        r["crest"] = r["ipk"] / r["irms"] if r["irms"] > 1e-4 else float("nan")
        self.hist.append(r)
        for key, (var, unit, dp) in self.read_vars.items():
            v = r.get(key)
            if v is None or v != v:
                var.set("---")
            else:
                var.set(f"{v:,.{dp}f} {unit}".strip())
        self._set_relay_ui(r["relay"] >= 1)
        if "rng" in r:
            self._range_ui(r["rng"])
        fl = int(r["flags"])
        self.flag_lbl.configure(text="   ".join(x for x in (self.safe_msg, flag_text(fl)) if x))
        if self.csv_w:
            self.csv_w.writerow([time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(r["t"])),
                                 f"{r['t']:.3f}"] + [r[k] for k in
                                ("vrms", "irms", "p", "s", "q", "pf", "hz", "vpk", "ipk", "wh",
                                 "vdc", "idc", "relay", "flags")] + [RANGE_NAMES.get(int(r.get("rng", 0)), "")])

    def _redraw_trends(self):
        try:
            if self.nb.index(self.nb.select()) == 0 and self.hist:
                span = {"30 s": 30, "2 min": 120, "10 min": 600, "1 h": 3600}.get(self.span_var.get())
                now = time.time()
                rows = [r for r in self.hist if span is None or r["t"] >= now - span]
                if rows:
                    t = np.array([r["t"] - now for r in rows])
                    get = lambda k: np.array([r[k] for r in rows])   # noqa: E731
                    a0, a0b, a1, a1b = self.tax[0], self.tax_i, self.tax[1], self.tax_pf
                    for a in (a0, a0b, a1, a1b):
                        a.cla()
                    a0.plot(t, get("vrms"), color="tab:blue")
                    a0.set_ylabel("Vrms", color="tab:blue")
                    a0b.plot(t, get("irms"), color="tab:red")
                    a0b.set_ylabel("Irms (A)", color="tab:red")
                    a1.plot(t, get("p"), color="tab:green")
                    a1.set_ylabel("W", color="tab:green")
                    a1b.plot(t, get("pf"), color="tab:purple")
                    a1b.set_ylabel("PF", color="tab:purple")
                    a1b.set_ylim(-1.05, 1.05)
                    a1.set_xlabel("seconds")
                    a0.grid(alpha=.3)
                    a1.grid(alpha=.3)
                    self.tfig.tight_layout()
                    self.tcanvas.draw_idle()
        finally:
            self.after(700, self._redraw_trends)

    def _toggle_log(self):
        if self.csv_fh:
            self.csv_fh.close()
            self.csv_fh = self.csv_w = None
            self.log_btn.configure(text="Start CSV log…")
            self.log_lbl.configure(text="")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", initialdir=HERE,
                                            initialfile=time.strftime("monitor_%Y%m%d_%H%M%S.csv"))
        if not path:
            return
        self.csv_fh = open(path, "w", newline="")
        self.csv_w = csv.writer(self.csv_fh)
        self.csv_w.writerow(["time", "unix", "vrms", "irms", "w", "va", "var", "pf", "hz", "vpk",
                             "ipk", "wh", "vdc", "idc", "relay", "flags", "range"])
        self.log_btn.configure(text="Stop CSV log")
        self.log_lbl.configure(text=os.path.basename(path))

    # ═══ capture ════════════════════════════════════════════════════════════
    def _timing_update(self, *_):
        """Work out whichever of rate / samples / duration isn't being set."""
        if self._timing_busy:
            return
        v, e = self.cap_vars, self.cap_entries
        mode = TIMING_MODES.index(v["tmode"].get())
        derived = ("dur", "n", "rate")[mode]
        for k, w in e.items():
            w.configure(state="readonly" if k == derived else "normal")
        try:                          # the Feather's buffer is 32768
            cap_max = int(self.link.status.get("capmax", CAP_MAX)) if self.link else CAP_MAX
        except ValueError:
            cap_max = CAP_MAX
        max_n = cap_max // (2 if v["ch"].get() == "V + I" else 1)
        note = ""
        try:
            rate = float(v["rate"].get()) if derived != "rate" else None
            n = int(float(v["n"].get())) if derived != "n" else None
            dur = float(v["dur"].get()) if derived != "dur" else None
        except ValueError:
            self.timing_note.configure(text="")
            return
        if derived == "dur":
            out = f"{n / rate * 1000:.4g}" if rate > 0 else "?"
            if rate <= 0:
                note = "rate 0 = as fast as the ADC goes, so the duration isn't known in advance"
        elif derived == "n":
            if rate <= 0:
                out, note = "?", "set a rate: 0 (max) can't be combined with a duration"
            else:
                want = int(round(rate * dur / 1000))
                out = str(min(want, max_n))
                if want > max_n:
                    note = (f"buffer holds {max_n} samples here: {max_n / rate * 1000:.4g} ms "
                            f"at this rate")
        else:
            out = f"{n / dur * 1000:.6g}" if dur > 0 else "?"
        if derived != "n" and n is not None and n > max_n:
            note = f"buffer holds {max_n} samples with these channels; the board will clamp"
        self._timing_busy = True
        try:
            v[derived].set(out)
        finally:
            self._timing_busy = False
        self.timing_note.configure(text=note)
        self._spike_update()

    def _spike_update(self):
        """Shortest spike the averaged samples still show at full height.

        The N scans of a sample are spread evenly over its period T, so scans are
        d = T/N apart everywhere.  A spike covering 2N-1 consecutive scans must
        contain one whole sample's group of N, whatever its timing, so it needs to
        last (2N-1)·d.  Anything lasting d hits at least one scan, so it shows up,
        but at as little as 1/N of its height."""
        v = self.cap_vars
        try:
            rate = float(v["rate"].get())
            n_avg = int(float(v["avg"].get()))
        except ValueError:
            self.spike_lbl.configure(text="")
            return
        both = v["ch"].get() == "V + I"
        st = self.link.status if self.link else {}
        try:
            scan_us = float(st.get("scanus4" if both else "scanus2", 0))
        except ValueError:
            scan_us = 0
        est = "" if scan_us > 0 else "≈"
        scan_us = scan_us or (4.2 if both else 2.2)        # typical, until a board reports it
        n_avg = max(n_avg, 1)
        d = 1e6 / (rate * n_avg) if rate > 0 else scan_us
        late = d < scan_us
        d = max(d, scan_us)
        full = (2 * n_avg - 1) * d
        fmt = lambda us: f"{us / 1000:.3g} ms" if us >= 1000 else f"{us:.3g} µs"   # noqa: E731
        txt = (f"Spike fully captured if it lasts ≥ {est}{fmt(full)}"
               + (f"   (≥ {est}{fmt(d)} always shows, at ≥ 1/{n_avg} of its height)" if n_avg > 1 else "")
               + f"   max rate at N={n_avg}: {est}{1e6 / (scan_us * n_avg):,.0f} S/s")
        if late:
            txt += "   — rate × N is more than the ADC can do; the capture will run LATE"
        self.spike_lbl.configure(text=txt)

    def _cap_args(self):
        v = self.cap_vars
        if "?" in (v["rate"].get(), v["n"].get()):
            raise ValueError(self.timing_note.cget("text") or "incomplete timing")
        ch = {"V + I": 2, "V only": 0, "I only": 1}[v["ch"].get()]
        trig = [k for k, name in TRIG_MODES.items() if name == v["trig"].get()][0]
        sf = self.safe
        args = dict(rate=float(v["rate"].get()), n=int(float(v["n"].get())), ch=ch, trig=trig,
                    level=float(v["level"].get()), pre=float(v["pre"].get()),
                    timeout_ms=int(float(v["timeout"].get()) * 1000),
                    # rising zero: close relay_us after it; level trigger: close at once
                    relay_us=max(0, int(float(v["relay_us"].get()))) if trig == 3 else -1,
                    ipk_a=float(sf["ipk_a"].get()) if sf["ipk_on"].get() else 0.0,
                    ov_a=float(sf["ov_a"].get()) if sf["ov_on"].get() else 0.0,
                    ov_cyc=int(float(sf["ov_cyc"].get())),
                    auto_ms=int(float(sf["auto_ms"].get())) if sf["auto_on"].get() else 0,
                    open_after=sf["open_after"].get(),
                    avg=int(float(v["avg"].get())),
                    rng=CAP_RANGES.index(v["rng"].get()) - 1,
                    mon=self.mon_var.get())
        if trig in CURRENT_TRIGS and ch == 0:
            raise ValueError("a current trigger needs the current channel (V + I or I only)")
        if args["timeout_ms"] < 0:
            raise ValueError("timeout must be 0 (none) or more")
        if (args["ipk_a"] or args["ov_a"]) and ch == 0:
            raise ValueError("the current limits need the current channel (V + I or I only)")
        if not 1 <= args["avg"] <= 64:
            raise ValueError("Average N must be 1…64")
        return args

    def _capture(self):
        if not self.link or self.busy:
            return
        try:
            args = self._cap_args()
        except (ValueError, IndexError) as e:
            messagebox.showerror("Capture", f"bad setting: {e}")
            return
        closes = args["trig"] in CLOSE_TRIGS
        if closes and not self.repeat_var.get():
            if args["open_after"] or args["auto_ms"]:
                after = "opens it again " + ("when the capture completes" if args["open_after"]
                                             else f"{args['auto_ms']} ms after closing")
            else:
                after = "LEAVES THE LOAD SWITCHED ON"
            if not messagebox.askokcancel("Inrush capture", f"This CLOSES THE RELAY and {after}. "
                                          "If it is closed now it opens for 1 s first. Continue?"):
                return
        if closes and self.repeat_var.get():
            self.repeat_var.set(False)        # one inrush per click
        self.busy = True
        self.safe_msg = ""
        self.cap_btn.configure(state="disabled")
        self.abort_btn.configure(state="normal")
        if args["trig"]:
            self.cap_stats.set("armed — waiting for the trigger" + ("" if args["timeout_ms"] else " (no timeout)")
                               + (", readings continue" if args["mon"] else "") + "…")

        def work():
            try:
                # an inrush test needs the load off first: open, let it settle 1 s
                if closes and self.link.get_status().get("relay") == "1":
                    self.ui(self.cap_stats.set, "relay was closed: opened, waiting 1 s…")
                    self.link.relay(False)
                    time.sleep(1.0)
                cap = self.link.capture(**args)
                self.ui(self._capture_done, cap, None)
            except Exception as e:                         # noqa: BLE001
                self.ui(self._capture_done, None, str(e))
        threading.Thread(target=work, daemon=True).start()

    def _abort(self):
        self.repeat_var.set(False)
        if self.link:
            self.link.abort()

    def _capture_done(self, cap, err):
        self.busy = False
        self.cap_btn.configure(state="normal")
        self.abort_btn.configure(state="disabled")
        if err:
            self.cap_stats.set("capture: " + err)
            if not self.repeat_var.get():
                return
        else:
            self.last_cap = cap
            self._plot_capture(cap)
        if self.repeat_var.get() and self.link:
            self.repeat_job = self.after(200, self._capture)

    def _plot_capture(self, cap):
        if cap is None:
            return
        self.cfig.clear()
        harm = self.show_harm.get()
        ax = self.cfig.add_subplot(2 if harm else 1, 1, 1)
        st = capture_stats(cap)
        lines = []
        if cap.v is not None:
            ax.plot(cap.t_ms, cap.v, color="tab:blue", lw=1)
            ax.set_ylabel("V", color="tab:blue")
        axi = ax.twinx() if cap.v is not None else ax
        if cap.i is not None:
            axi.plot(cap.t_ms, cap.i, color="tab:red", lw=1)
            axi.set_ylabel("A", color="tab:red")
        # 0 V / 0 A on the centre line, however one-sided the waveform is
        for a, x in ((ax, cap.v), (axi, cap.i)):
            if x is not None and len(x):
                lim = float(np.nanmax(np.abs(x))) * 1.08 or 1.0
                a.set_ylim(-lim, lim)
        if cap.trig_idx >= 0:
            ax.axvline(0, color="k", ls="--", lw=.8)
        t0 = max(cap.trig_idx, 0)
        if cap.relay_idx >= 0:
            ax.axvline((cap.relay_idx - t0) * cap.dt_us / 1000, color=ACC, lw=1.2, label="relay close command")
        if cap.open_idx >= 0:
            ax.axvline((cap.open_idx - t0) * cap.dt_us / 1000, color="tab:purple", lw=1.2, ls="--",
                       label="relay open: " + SAFE_WHY.get(cap.open_why, "?"))
        if cap.rng_idx >= 0 and cap.i is not None:
            x0 = (cap.rng_idx - t0) * cap.dt_us / 1000
            axi.axvspan(x0, x0 + cap.settle_us / 1000, color="tab:orange", alpha=.15,
                        label="range relay moving (LOW → HIGH)")
            axi.legend(loc="upper right", fontsize=8)
        if cap.relay_idx >= 0 or cap.open_idx >= 0:
            ax.legend(loc="upper left", fontsize=8)
        ax.set_xlabel("ms from trigger" if cap.trig_idx >= 0 else "ms")
        ax.grid(alpha=.3)
        s = f"{cap.n} samples @ {st['rate']:,.0f} S/s ({cap.dt_us:.2f} µs)   f {st['hz']:.3f} Hz"
        if cap.avg > 1:
            s += f"   each the mean of {cap.avg} scans"
        if cap.i is not None and self.link and self.link.cfg.get("RNGPIN", {}).get("value"):
            s += f"   range {RANGE_NAMES.get(cap.rng, '?')}"
            if cap.rng_idx >= 0:
                s += (f" → HIGH (overload) at {(cap.rng_idx - max(cap.trig_idx, 0)) * cap.dt_us / 1000:.2f} ms;"
                      f" the next {cap.settle_us / 1000:g} ms is the relay moving")
        if "vrms" in st:
            lines.append(f"V  {st['vrms']:8.3f} Vrms  dc {st['vdc']:+8.3f} V  pk {st['vpk']:7.2f}  crest {st['vcrest']:.3f}  THD {st['vthd'] * 100:5.2f} %")
        if "irms" in st:
            lines.append(f"I  {st['irms']:8.4f} Arms  dc {st['idc']:+8.4f} A  pk {st['ipk']:7.3f}  crest {st['icrest']:.3f}  THD {st['ithd'] * 100:5.2f} %")
        if "p" in st:
            lines.append(f"P  {st['p']:8.3f} W     PF {st['pf']:.4f}")
        if cap.open_idx >= 0:
            lines.append(f"relay opened by {SAFE_WHY.get(cap.open_why, '?')} at "
                         f"{(cap.open_idx - max(cap.trig_idx, 0)) * cap.dt_us / 1000:.2f} ms")
        if cap.flags:
            lines.append("flags: " + flag_text(cap.flags))
        self.cap_stats.set(s + "\n" + "\n".join(lines))
        if harm:
            hx = self.cfig.add_subplot(2, 1, 2)
            for name, color, off in (("iharm", "tab:red", 0.2), ("vharm", "tab:blue", -0.2)):
                h = st.get(name)
                if h is not None and len(h) > 1 and h[0] > 0:
                    k = np.arange(1, len(h) + 1)
                    hx.bar(k + off, 100 * h / h[0], width=0.4, color=color,
                           label=("I" if name[0] == "i" else "V") + " % of fundamental")
            hx.set_yscale("log")
            hx.set_ylim(0.01, 150)
            hx.set_xlabel("harmonic")
            hx.legend(fontsize=8)
            hx.grid(alpha=.3, which="both")
        self.cfig.tight_layout()
        self.ccanvas.draw_idle()

    def _save_capture(self):
        cap = self.last_cap
        if cap is None:
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", initialdir=HERE,
                                            initialfile=time.strftime("capture_%Y%m%d_%H%M%S.csv"))
        if not path:
            return
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow([f"# dt_us={cap.dt_us}", f"trig_idx={cap.trig_idx}", f"relay_idx={cap.relay_idx}",
                        f"open_idx={cap.open_idx}", f"open_why={SAFE_WHY.get(cap.open_why, '')}",
                        f"flags={cap.flags}", f"scans_per_sample={cap.avg}",
                        f"range={RANGE_NAMES.get(cap.rng, '')}", f"range_high_from_idx={cap.rng_idx}"])
            w.writerow(["t_ms"] + (["v"] if cap.v is not None else []) + (["i"] if cap.i is not None else []))
            for k in range(cap.n):
                row = [f"{cap.t_ms[k]:.5f}"]
                if cap.v is not None:
                    row.append(f"{cap.v[k]:.4f}")
                if cap.i is not None:
                    row.append(f"{cap.i[k]:.5f}")
                w.writerow(row)

    # ═══ calibration ════════════════════════════════════════════════════════
    def _cal_log(self, s):
        self.ui(lambda: (self.cal_txt.insert("end", s + "\n"), self.cal_txt.see("end")))

    def _psu_connect(self):
        if self.psu:
            self.psu.close()
            self.psu = None
            self.psu_btn.configure(text="Connect supply")
            self.psu_lbl.configure(text="not connected")
            return
        hostport = self.psu_host.get().strip()
        host, _, port = hostport.partition(":")
        mode, ch = self.psu_mode.get(), int(self.psu_ch.get())

        def work():
            try:
                p = bench_psu.open_psu(mode, host=host or bench_psu.PSU_HOST,
                                       port=int(port or bench_psu.PSU_PORT))
                st = p.check_channel(ch)
                msg = (f"{p.name}\n{p.ident()}\nCH{ch}: {'ON' if st['on'] else 'off'}, "
                       f"set {st['vset']:g} V / {st['iset']:g} A")
                self.ui(self._psu_ok, p, msg)
            except Exception as e:                         # noqa: BLE001
                self.ui(messagebox.showerror, "Bench supply", str(e))
        threading.Thread(target=work, daemon=True).start()

    def _psu_ok(self, p, msg):
        self.psu = p
        self.psu_btn.configure(text="Disconnect supply")
        self.psu_lbl.configure(text=msg)

    def _sim_mains(self):
        import monitor_sim
        monitor_sim.WORLD.mains = self.sim_mains.get()

    def _run_cal(self, title, fn):
        if not self.link:
            messagebox.showerror(title, "connect the monitor first")
            return
        if self.busy:
            return
        self.busy = True
        self.stop_evt.clear()
        self._cal_log(f"── {title} ──")
        was_streaming = self.stream_var.get()

        def work():
            try:
                self.link.stream(False)
                fn()
            except Exception as e:                         # noqa: BLE001
                self._cal_log(f"FAILED: {e}")
            finally:
                self.busy = False
                if was_streaming and self.link:
                    try:
                        self.link.stream(True)
                    except Exception:                      # noqa: BLE001
                        pass
                self.ui(self._cal_plot)
        threading.Thread(target=work, daemon=True).start()

    @staticmethod
    def _floats(s):
        return [float(x) for x in s.replace(";", ",").split(",") if x.strip()]

    def _need_psu(self):
        if not self.psu:
            messagebox.showerror("Calibration", "connect the bench supply first")
            return None, None
        return self.psu, int(self.psu_ch.get())

    def _cal_voltage(self):
        if not messagebox.askokcancel("Voltage sweep",
                                      "Mains must be DISCONNECTED from the input and the bench supply "
                                      f"CH{self.psu_ch.get()} wired to it (+ to hot).\nThe relay will be "
                                      "opened. Continue?"):
            return

        psu, ch = self._need_psu()
        if not psu:
            return
        pts, ilim = self._floats(self.vpts.get()), float(self.vilim.get())
        lim = float(self.mains_lim.get())

        def fn():
            self.cal_v = cal.cal_voltage(self.link, psu, ch, pts, ilim, log=self._cal_log,
                                         stop=self.stop_evt, limit_vac=lim)
            self._cal_log(self.cal_v.report())
        self._run_cal("voltage sweep", fn)

    def _cal_rng_idx(self):
        return 1 if self.cal_rng.get() == RANGE_FULL[1] else 0

    def _cal_current(self):
        rng = self._cal_rng_idx()
        if not messagebox.askokcancel("Current sweep",
                                      "Mains DISCONNECTED, bench supply on the input, and a SHORT or a "
                                      "low-ohm power resistor plugged into the outlet.\nThe relay will be "
                                      f"closed and CH{self.psu_ch.get()} run in constant current up to "
                                      f"{self.ivlim.get()} V, on the {RANGE_FULL[rng]} range. Continue?"):
            return

        psu, ch = self._need_psu()
        if not psu:
            return
        pts, vlim = self._floats(self.ipts.get()), float(self.ivlim.get())
        lim = float(self.mains_lim.get())

        def fn():
            fit = cal.cal_current(self.link, psu, ch, pts, vlim, log=self._cal_log,
                                  stop=self.stop_evt, limit_vac=lim, rng=rng)
            if rng:
                self.cal_ilo = fit
            else:
                self.cal_i = fit
            self._cal_log(fit.report())
        self._run_cal(f"current sweep, {RANGE_FULL[rng]}", fn)

    def _cal_phase(self):
        if not messagebox.askokcancel("Phase", "Needs MAINS on the input (bench supply disconnected!) "
                                               "and a purely resistive load (heater, incandescent bulb) "
                                               "in the outlet. The relay will be closed. Continue?"):
            return

        def fn():
            self.link.relay(True)
            time.sleep(0.5)
            try:
                self.cal_phase = cal.cal_phase(self.link, log=self._cal_log)
            finally:
                self.link.relay(False)
            self._cal_log(f"PHASEUS -> {self.cal_phase:+.2f} us (Apply to use it)")
        self._run_cal("phase", fn)

    def _cal_values(self):
        out = {}
        if self.cal_v:
            out["VGAIN"], out["VOFF"] = self.cal_v.gain, self.cal_v.off
        if self.cal_i:
            out["IGAIN"], out["IOFF"] = self.cal_i.gain, self.cal_i.off
        if self.cal_ilo:
            out["IGAINLO"], out["IOFFLO"] = self.cal_ilo.gain, self.cal_ilo.off
        if self.cal_phase is not None:
            out["PHASEUS"] = self.cal_phase
        return out

    def _cal_apply(self):
        vals = self._cal_values()
        if not vals or not self.link:
            messagebox.showinfo("Apply", "nothing measured yet")
            return
        for k, v in vals.items():
            self.link.send(f"!SET,{k},{v:.7g}")
        self._cal_log("applied (RAM): " + ", ".join(f"{k}={v:.6g}" for k, v in vals.items()))
        self.after(400, self._cfg_refresh)

    def _cal_save(self):
        if not self.link:
            return
        if self._cal_values() and messagebox.askyesno("Save", "Apply the new results before saving?"):
            self._cal_apply()
        self.after(300, lambda: self._cmd("!SAVE"))
        self._cal_log("saved to EEPROM")

    def _cal_plot(self):
        for a in self.kax:
            a.cla()
        for a, fits in zip(self.kax, ((self.cal_v,), (self.cal_i, self.cal_ilo))):
            fits = [f for f in fits if f is not None]
            if not fits:
                a.set_title(("Voltage" if a is self.kax[0] else "Current") + ": not run", fontsize=9)
                continue
            for fit in fits:
                t = [p[0] for p in fit.points]
                a.plot(t, fit.resid, "o-", label=fit.what)
            a.axhline(0, color="k", lw=.6)
            a.set_title(f"{fits[0].what[0]} residuals ({fits[0].unit})", fontsize=9)
            a.set_xlabel(f"truth ({fits[0].unit})")
            if len(fits) > 1:
                a.legend(fontsize=8)
            a.grid(alpha=.3)
        self.kfig.tight_layout()
        self.kcanvas.draw_idle()

    # ═══ config ═════════════════════════════════════════════════════════════
    def _cfg_refresh(self):
        if self.link:
            self._cmd("!CFG")
            self._cmd("!STATUS")

    def _cfg_defaults(self):
        if messagebox.askyesno("Defaults", "Reset every key (including calibration) to factory defaults "
                                           "in RAM? Nothing is saved until you Save."):
            self._cmd_refresh("!DEFAULTS")

    def _fill_cfg(self):
        if not self.link:
            return
        cfg = self.link.cfg
        existing = set(self.tree.get_children())
        for key, row in cfg.items():
            v = row["value"]
            txt = f"{v:.6g}" if abs(v - round(v)) > 1e-9 or abs(v) > 1e5 else f"{int(round(v))}"
            vals = (GROUP_OF.get(key, ""), txt, f"{row['lo']:g} … {row['hi']:g}", row["help"])
            if key in existing:
                self.tree.item(key, values=vals)
            else:
                self.tree.insert("", "end", iid=key, text=key, values=vals)
        self.dirty_lbl.configure(text="unsaved changes" if self.link.status.get("dirty") == "1" else "")

    def _cfg_edit(self, event):
        item = self.tree.identify_row(event.y)
        if not item or not self.link:
            return
        x, y, w, h = self.tree.bbox(item, "value")
        var = tk.StringVar(value=self.tree.set(item, "value"))
        e = ttk.Entry(self.tree, textvariable=var)
        e.place(x=x, y=y, width=w, height=h)
        e.focus_set()

        def done(commit):
            if commit:
                self._cmd(f"!SET,{item},{var.get().strip()}")
                self.after(300, lambda: self._cmd("!STATUS"))
            e.destroy()
        e.bind("<Return>", lambda _e: done(True))
        e.bind("<Escape>", lambda _e: done(False))
        e.bind("<FocusOut>", lambda _e: done(False))

    # ── snapshots ───────────────────────────────────────────────────────────
    def _log_config(self, why):
        """Log the stored config to config_log/, unless it matches the newest
        snapshot of the same board."""
        link = self.link
        if not link:
            return

        def work():
            try:
                path = log_snapshot(link.dump(saved=True), CFG_LOG_DIR)
                msg = f"saved {os.path.basename(path)}" if path else "unchanged since the last snapshot"
            except (LinkError, OSError) as e:
                msg = str(e)
            self.ui(self._con_add, f"# config log ({why}): {msg}")
        threading.Thread(target=work, daemon=True).start()

    def _cfg_export(self):
        if not self.link:
            return
        saved = True
        if self.link.status.get("dirty") == "1":
            ans = messagebox.askyesnocancel("Export snapshot", "There are unsaved changes. Export the "
                                            "running values (Yes) or the saved ones (No)?")
            if ans is None:
                return
            saved = not ans
        os.makedirs(CFG_LOG_DIR, exist_ok=True)
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", initialdir=CFG_LOG_DIR, filetypes=[("Config snapshot", "*.csv")],
            initialfile=f"{self.link.status.get('board', 'RA4M1')}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        if not path:
            return
        link = self.link

        def work():
            try:
                link.dump(saved=saved).save(path)
                self.ui(self.status_lbl.configure, text=f"snapshot written: {os.path.basename(path)}")
            except (LinkError, OSError) as e:
                self.ui(messagebox.showerror, "Export snapshot", str(e))
        threading.Thread(target=work, daemon=True).start()

    def _cfg_apply(self):
        if not self.link:
            return
        path = filedialog.askopenfilename(initialdir=CFG_LOG_DIR if os.path.isdir(CFG_LOG_DIR) else HERE,
                                          filetypes=[("Config snapshot", "*.csv"), ("All files", "*.*")])
        if not path:
            return
        try:
            snap = Snapshot.load(path)
        except (OSError, ValueError) as e:
            messagebox.showerror("Apply snapshot", str(e))
            return
        link = self.link

        def work():
            try:
                here = link.dump()
            except LinkError as e:
                self.ui(messagebox.showerror, "Apply snapshot", str(e))
                return
            self.ui(ApplyDialog, self, snap, here, os.path.basename(path))
        threading.Thread(target=work, daemon=True).start()


class ApplyDialog(tk.Toplevel):
    """Copy chosen groups of a snapshot onto the connected board (RAM, then
    optionally saved). Only keys whose values differ are sent."""

    def __init__(self, app, snap, here, name):
        super().__init__(app)
        self.app = app
        self.title(f"Apply {name}")
        cur = here.values()
        self.diffs = [d for d in snap.diff(cur) if d[1] in cur]
        missing = sorted(k for _, k, _v in snap.rows if k not in cur)
        sm, hm = snap.meta, here.meta
        other = sm.get("board", "?") != hm.get("board", "?")
        same_unit = bool(sm.get("uid")) and sm.get("uid") == hm.get("uid")
        ttk.Label(self, justify="left", font=("Consolas", 9), text=(
            f"File : {sm.get('board', '?'):6s} uid {sm.get('uid') or '?'}  fw {sm.get('fw', '?')}  "
            f"{sm.get('src', '?')} values, taken {sm.get('taken', '?')}\n"
            f"Board: {hm.get('board', '?'):6s} uid {hm.get('uid') or '?'}  fw {hm.get('fw', '?')}"
            + ("   (same unit)" if same_unit else ""))).pack(anchor="w", padx=8, pady=6)
        if other:
            ttk.Label(self, foreground="#c60", wraplength=640, justify="left", text=(
                "Different board type. The CAL gains carry over if the isolation-amp front end moved "
                "with them, but zero the offsets again on this board and check PHASEUS. WIRING pin "
                "numbers do not carry over.")).pack(anchor="w", padx=8)
        if missing:
            ttk.Label(self, foreground="#888", text="Not on this firmware, skipped: " + ", ".join(missing)
                      ).pack(anchor="w", padx=8)

        box = ttk.LabelFrame(self, text="Copy these groups", padding=6)
        box.pack(fill="x", padx=8, pady=6)
        counts = {}
        for g, *_ in self.diffs:
            counts[g] = counts.get(g, 0) + 1
        self.on = {}
        for g in list(CFG_GROUPS) + sorted({r[0] for r in snap.rows} - set(CFG_GROUPS)):
            n = counts.get(g, 0)
            self.on[g] = tk.BooleanVar(value=g == "CAL" and n > 0)
            cb = ttk.Checkbutton(box, variable=self.on[g], command=self._refresh, text=(
                f"{g} — {GROUP_INFO.get(g, '')}: {n} differ"
                + ("   (pins from another board type!)" if g == "WIRING" and other and n else "")))
            cb.pack(anchor="w")
            if not n:
                cb.state(["disabled"])

        self.tree = ttk.Treeview(self, columns=("group", "board", "file"), show="tree headings", height=12)
        for col, txt, w in (("#0", "Key", 100), ("group", "Group", 70), ("board", "This board", 130),
                            ("file", "Snapshot", 130)):
            self.tree.heading(col, text=txt)
            self.tree.column(col, width=w)
        self.tree.tag_configure("off", foreground="#999")
        for g, k, c, v in self.diffs:
            self.tree.insert("", "end", iid=k, text=k, values=(g, f"{c:.7g}", f"{v:.7g}"))
        self.tree.pack(fill="both", expand=True, padx=8)

        row = ttk.Frame(self, padding=8)
        row.pack(fill="x")
        self.btns = [ttk.Button(row, text="Apply to RAM", command=lambda: self._apply(False)),
                     ttk.Button(row, text="Apply and save", command=lambda: self._apply(True))]
        for b in self.btns:
            b.pack(side="left", padx=3)
        ttk.Button(row, text="Close", command=self.destroy).pack(side="right")
        self.msg = ttk.Label(self, text="" if self.diffs else "This board already matches the snapshot.",
                             wraplength=640, justify="left")
        self.msg.pack(anchor="w", padx=8, pady=(0, 8))
        self._refresh()

    def _todo(self):
        return [(k, v) for g, k, _c, v in self.diffs if self.on[g].get()]

    def _refresh(self):
        for g, k, *_ in self.diffs:
            self.tree.item(k, tags=() if self.on[g].get() else ("off",))
        n = len(self._todo())
        for b in self.btns:
            b.state(["!disabled"] if n else ["disabled"])

    def _apply(self, save):
        todo = self._todo()
        for b in self.btns:
            b.state(["disabled"])
        self.msg.configure(text=f"sending {len(todo)} keys…")
        link = self.app.link

        def work():
            for k, v in todo:
                try:
                    link.set(k, v)
                except LinkError:
                    pass                          # a refused pin etc.; the read-back below reports it
            try:
                link.read_cfg()
                bad = [k for k, v in todo if not same_value(link.cfg.get(k, {}).get("value", float("nan")), v)]
                msg = f"{len(todo) - len(bad)} of {len(todo)} keys set in RAM."
                if bad:
                    msg += " Not taken (refused or clamped): " + ", ".join(bad) + "."
                if save and not bad:
                    link.save()
                    msg += " Saved."
                elif save:
                    msg += " Not saved — fix those first or save from the Config tab."
                else:
                    msg += " Save from the Config tab to keep them."
            except LinkError as e:
                msg = f"Error: {e}"
            self.app.ui(self._done, msg)
        threading.Thread(target=work, daemon=True).start()

    def _done(self, msg):
        if not self.winfo_exists():
            return
        self.msg.configure(text=msg)
        self.app._fill_cfg()
        self.app._cmd("!STATUS")


if __name__ == "__main__":
    App().mainloop()
