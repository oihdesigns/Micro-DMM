#!/usr/bin/env python3
"""
wirehawk_gui.py  --  Host GUI for the WireHawk firmware (Arduino Nano ESP32).

Talks to the board over USB serial (115200 baud).  Four tabs:

  Test
    - arm/close the relay (ON is gated behind an Arm checkbox; Esc = stop)
    - big live current readout, diagnosis banner, run timer
    - avg / min / max / peak / ripple statistics
    - D4/D3 sense-bit indicators and the decoded sense code
    - rolling current plot with the detection threshold and relay-on shading
    - one-button test sequence: idle check -> timed inrush capture -> verdict
    - optional CSV log of every streamed sample

  Inrush capture
    - high-rate A0 capture across a relay close, plotted in mA
    - inrush peak, steady-state current, time to steady, CSV export

  Calibration
    - !ZERO with the relay open, !CAL against a known current
    - live mV/mA readout so the two-point fit can be sanity checked

  Link supervision (all tabs)
    - switching the motor off arcs the relay contacts; that burst can brown
      out the board or drop the native-USB link, so the port vanishes mid-test
    - the GUI reconnects to the same port on its own (auto-reconnect, on by
      default) and sends !STOP once back, since a link loss is a fault
      condition and the relay may still be closed
    - CONFIRMED ON HARDWARE: the MCU does not reset, only the USB link drops --
      so the motor keeps running through the outage. While the relay is closed
      this GUI sends !PING every PING_MS, which arms the board's own HOSTTO
      cutoff so it de-energises without waiting for the host to come back
    - the banner turns to LINK LOST (amber when the relay was closed) rather
      than leaving a stale RUNNING on screen
    - MOTOR OFF / !STOP pressed during an outage is queued and replayed on
      reconnect, never silently dropped; commands that would energise the motor
      are refused instead of queued
    - a $BOOT line proves the board actually reset (and why: BROWNOUT/PANIC =
      the relay or motor is crashing the board, not just the USB link); no
      $BOOT after a reconnect means only the link dropped

  Configuration (EEPROM)
    - reads every config key from the device (!CFG) into an editable table
    - push to RAM (!SET), persist (!SAVE), reload (!LOAD), factory (!DEFAULTS)
    - keys the firmware reports but this GUI does not know about still appear
      (under "Other"), so new firmware tunables need no GUI change

Dependencies:
    pip install pyserial matplotlib

Run:
    python wirehawk_gui.py
"""

import csv
import os
import queue
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from tkinter import filedialog, messagebox, ttk

import serial
import serial.tools.list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

BAUD = 115200
LIVE_MAXLEN = 900          # samples kept in the rolling live plot
PLOT_REFRESH_MS = 150      # live plot redraw cadence
LOG_MAXLINES = 4000        # raw serial log trim

# Switching an inductive load off arcs the relay contacts, and that burst can
# brown-out the board or drop the native-USB link -- the port disappears and
# comes back a second later.  Rather than making the operator re-click
# Connect, reconnect automatically.  RECONNECT_TRIES * RECONNECT_MS is how
# long we keep trying.
RECONNECT_MS = 700
RECONNECT_TRIES = 30
STALE_MS = 3000            # no data at all for this long = link is dead
# A !CAP blocks the board for its whole duration and then dumps thousands of
# rows, so the stale-link watchdog has to stand down for it.
CAPTURE_GRACE_MS = 30000

# Keepalive sent while the relay is closed. The board arms its HOSTTO cutoff
# only after it sees a !PING, so these pings are what let it drop the relay by
# itself when the link dies mid-run. Must be comfortably under HOSTTO.
PING_MS = 400

# Commands that are safe to hold and replay once the link is back. Anything
# that ENERGISES the motor is deliberately absent: replaying a stale !ON after
# a reconnect would start the motor at a moment the operator did not choose.
QUEUEABLE = ("!OFF", "!STOP", "!CLEAR", "!RST")

# Diagnosis code -> (label, background colour, foreground colour).
# Must match STATE_NAMES / State in WireHawk.ino.
STATES = {
    0:  ("IDLE",        "#303030", "#c8c8c8"),
    1:  ("READY",       "#123a5a", "#7ec8ff"),
    2:  ("NO MOTOR",    "#5a3a12", "#ffcc7e"),
    3:  ("STARTING",    "#0d4a4a", "#7effe0"),
    4:  ("RUNNING",     "#0d5a1e", "#7dff8f"),
    5:  ("STALL",       "#6a0d0d", "#ff8a8a"),
    6:  ("NO VOLTAGE",  "#6a0d0d", "#ff8a8a"),
    7:  ("NO CURRENT",  "#5a3a12", "#ffcc7e"),
    8:  ("BACKFEED",    "#6a0d0d", "#ff8a8a"),
    9:  ("SENSE FAULT", "#6a0d0d", "#ff8a8a"),
    10: ("OVERCURRENT", "#7a0000", "#ffb0b0"),
    11: ("UNKNOWN",     "#303030", "#c8c8c8"),
}

# Sense code (D4<<1 | D3) -> meaning, per the load-side sense circuit.
IO_MEANING = {
    0: "00  unknown / not wired",
    1: "01  no motor",
    2: "10  motor present",
    3: "11  voltage present",
}

# Config-key metadata: (group, kind, description[, choices]).  The table is
# built from the DEVICE's !CFG dump, so a key missing here still shows up under
# "Other" -- this dict only makes known keys prettier.
KEY_META = {
    # --- Current sense ---
    "ZEROMV":    ("Current sense", "num",  "A0 reading at 0 A (mV) -- set by !ZERO"),
    "MVPERA":    ("Current sense", "num",  "Sense sensitivity (mV per amp) -- set by !CAL"),
    "CURTHRESH": ("Current sense", "num",  "Current at or above this = flowing (mA)"),
    "ABSCUR":    ("Current sense", "bool", "1 = compare |current| so either direction counts"),
    "ADCAVG":    ("Current sense", "num",  "A0 samples averaged per pass"),
    "ADCMV":     ("Current sense", "bool", "1 = analogReadMilliVolts (calibrated), 0 = raw x VREF"),
    "VREF":      ("Current sense", "num",  "Full-scale reference in mV (used when ADCMV=0)"),
    # --- Diagnosis ---
    "USEIO":     ("Diagnosis", "bool", "1 = use the D3/D4 sense bits (0 = current only)"),
    "IOINVERT":  ("Diagnosis", "bool", "1 = sense bits are active low"),
    "IOMODE":    ("Diagnosis", "choice", "D3/D4 input mode: 0=pulldown 1=pullup 2=plain",
                  ["0", "1", "2"]),
    "INRUSHMS":  ("Diagnosis", "num",  "Grace window after relay close (ms)"),
    "STALLMS":   ("Diagnosis", "num",  "Voltage-without-current before STALL (ms)"),
    "STABLEN":   ("Diagnosis", "num",  "Consecutive passes before a state change sticks"),
    # --- Safety ---
    "OCTRIP":    ("Safety", "num", "Over-current trip, latches relay open (mA, 0 = off)"),
    "MAXRUNMS":  ("Safety", "num", "Force the relay open after this run time (ms, 0 = off)"),
    "HOSTTO":    ("Safety", "num", "Open the relay if a pinging host goes quiet "
                                   "(ms, 0 = off) -- catches a dropped USB link mid-run"),
    # --- Capture / reporting ---
    "CAPPRE":    ("Capture / reporting", "num",  "Capture pre-trigger window (ms)"),
    "CAPMS":     ("Capture / reporting", "num",  "Default capture duration (ms)"),
    "CAPREST":   ("Capture / reporting", "bool", "1 = restore the relay state after a capture"),
    "RATEMS":    ("Capture / reporting", "num",  "$WH stream interval (ms)"),
    "STREAM":    ("Capture / reporting", "bool", "1 = stream $WH from boot"),
    "LED":       ("Capture / reporting", "bool", "1 = drive the onboard RGB LED"),
}
GROUP_ORDER = ["Current sense", "Diagnosis", "Safety", "Capture / reporting", "Other"]


class SerialManager:
    """Background serial reader. Pushes decoded lines onto a queue."""

    def __init__(self, line_queue):
        self.line_queue = line_queue
        self.ser = None
        self._stop = threading.Event()
        self._thread = None
        # Arrival is timestamped HERE, in the reader thread, not in the Tk
        # callback that drains the queue.  Otherwise a busy main loop (a big
        # matplotlib redraw, a modal dialog) delays the timestamp and a
        # perfectly healthy link looks dead to the stale-link watchdog.
        self.last_rx = 0.0

    def connect(self, port):
        self.disconnect()
        self.ser = serial.Serial(port, BAUD, timeout=0.1)
        self.last_rx = time.monotonic()
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
            except Exception:
                pass
            self.ser = None

    @property
    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def send(self, text):
        if self.is_open:
            if not text.endswith("\n"):
                text += "\n"
            try:
                self.ser.write(text.encode("ascii", errors="ignore"))
            except Exception as exc:
                self.line_queue.put(("__error__", str(exc)))

    def _read_loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                data = self.ser.read(256)
            except Exception as exc:
                self.line_queue.put(("__error__", str(exc)))
                break
            if not data:
                continue
            self.last_rx = time.monotonic()
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", errors="replace").strip()
                if line:
                    self.line_queue.put(("line", line))


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("WireHawk -- Motor Test & Fault Diagnosis")
        self.geometry("1180x830")

        self.line_queue = queue.Queue()
        self.serial = SerialManager(self.line_queue)

        # live sample history
        # One list of (board_ms, mA, relay).  Keeping the board's own timestamp
        # rather than a plot offset is what lets samples recovered after a
        # dropout (!HIST) be merged back into the right place.
        self.samples = deque(maxlen=LIVE_MAXLEN)
        self.last_board_ms = None
        self.hist_rows = []            # $H rows being collected
        self.backfilled = 0
        self.threshold_ma = 20.0

        # latest decoded telemetry
        self.state_code = 0
        self.relay = 0
        self.io_code = 0

        # link supervision
        self.port_in_use = None
        self.reconnecting = False
        self._reconnect_left = 0
        self._link_up_at = 0.0         # arrival time is owned by SerialManager
        self.stream_seen = False
        self.expect_stream = True
        self.deferred = []             # safe commands held while the link is down
        self.host_to_ms = None         # device HOSTTO, for the "unguarded" hint

        # capture accumulation
        self.cap_header = None
        self.cap_rows = []
        self.cap_active = False
        self.cap_started = 0.0
        self.cap_result = None
        self.cap_on_done = None        # callback for the scripted test sequence

        # link diagnostics from $WH: did the board stop sending, or did we stop
        # receiving? (see the "Is it the board or the host?" note in the .ino)
        self.last_seq = None
        self.last_tx_skips = 0
        self.lost_rows = 0
        self.skipped_rows = 0
        self.link_events = []          # board-recorded USB dropouts (!LINK)
        self.link_drops = 0
        self.board = ""                # nano_esp32 / xiao_ra4m1, from $BOOT
        self.sn_text = ""

        # config table state
        self.cfg_rows = {}             # key -> {"value", "editor", "var"}
        self.cfg_pending = False

        # CSV logging
        self.log_writer = None
        self.log_file = None

        self._build_ui()
        self.after(60, self._pump)
        self.after(PLOT_REFRESH_MS, self._redraw)
        self.after(PING_MS, self._ping_tick)
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.bind("<Escape>", lambda _e: self.cmd_off())

    # ─────────────────────────────────────────────────────────────
    #  UI
    # ─────────────────────────────────────────────────────────────
    def _build_ui(self):
        top = ttk.Frame(self, padding=(8, 6))
        top.pack(fill="x")

        ttk.Label(top, text="Port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=28)
        self.port_combo.pack(side="left", padx=4)
        ttk.Button(top, text="Refresh", command=self.refresh_ports).pack(side="left")
        self.connect_btn = ttk.Button(top, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=6)

        self.conn_lbl = ttk.Label(top, text="disconnected", foreground="#a05050")
        self.conn_lbl.pack(side="left", padx=10)
        self.auto_reconnect = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text="auto-reconnect",
                        variable=self.auto_reconnect).pack(side="left")
        self.sn_lbl = ttk.Label(top, text="SN: --")
        self.sn_lbl.pack(side="right")

        # always-visible diagnosis banner + emergency stop
        banner = tk.Frame(self, bg="#303030")
        banner.pack(fill="x", padx=8)
        self.state_lbl = tk.Label(banner, text="IDLE", bg="#303030", fg="#c8c8c8",
                                  font=("Segoe UI", 22, "bold"), pady=6)
        self.state_lbl.pack(side="left", padx=12)
        self.detail_lbl = tk.Label(banner, text="not connected", bg="#303030",
                                   fg="#909090", font=("Segoe UI", 10))
        self.detail_lbl.pack(side="left")
        self.stop_btn = tk.Button(banner, text="MOTOR OFF  (Esc)", bg="#8a1010",
                                  fg="white", font=("Segoe UI", 12, "bold"),
                                  activebackground="#b02020", relief="raised",
                                  command=self.cmd_off)
        self.stop_btn.pack(side="right", padx=8, pady=4)

        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=8, pady=8)
        self.tab_test = ttk.Frame(nb)
        self.tab_cap = ttk.Frame(nb)
        self.tab_cal = ttk.Frame(nb)
        self.tab_cfg = ttk.Frame(nb)
        self.tab_log = ttk.Frame(nb)
        nb.add(self.tab_test, text="Test")
        nb.add(self.tab_cap, text="Inrush capture")
        nb.add(self.tab_cal, text="Calibration")
        nb.add(self.tab_cfg, text="Configuration")
        nb.add(self.tab_log, text="Serial log")

        self._build_test_tab()
        self._build_capture_tab()
        self._build_cal_tab()
        self._build_config_tab()
        self._build_log_tab()

        self.refresh_ports()

    def _build_test_tab(self):
        left = ttk.Frame(self.tab_test, padding=6)
        left.pack(side="left", fill="y")

        # --- big current readout ---
        box = ttk.LabelFrame(left, text="Current", padding=8)
        box.pack(fill="x")
        self.cur_lbl = tk.Label(box, text="--", font=("Consolas", 34, "bold"))
        self.cur_lbl.pack()
        ttk.Label(box, text="mA").pack()
        self.mv_lbl = ttk.Label(box, text="A0: -- mV")
        self.mv_lbl.pack(pady=(4, 0))

        stats = ttk.LabelFrame(left, text="Statistics (inrush window excluded)", padding=8)
        stats.pack(fill="x", pady=6)
        self.stat_vars = {}
        for row, (key, label) in enumerate([("avg", "Average"), ("min", "Minimum"),
                                           ("max", "Maximum"), ("pk", "Peak |I|"),
                                           ("ripple", "Ripple (RMS)"),
                                           ("run", "Run time"),
                                           ("loop", "Board loop"),
                                           ("lost", "Rows lost/skip")]):
            ttk.Label(stats, text=label + ":").grid(row=row, column=0, sticky="w")
            var = tk.StringVar(value="--")
            ttk.Label(stats, textvariable=var, font=("Consolas", 10),
                      width=12, anchor="e").grid(row=row, column=1, sticky="e", padx=6)
            self.stat_vars[key] = var

        # --- sense bits ---
        io = ttk.LabelFrame(left, text="Load-side sense (D4 D3)", padding=8)
        io.pack(fill="x", pady=6)
        bits = ttk.Frame(io)
        bits.pack()
        self.bit_lbls = {}
        for col, name in enumerate(["D4", "D3"]):
            f = ttk.Frame(bits)
            f.grid(row=0, column=col, padx=8)
            ttk.Label(f, text=name).pack()
            lbl = tk.Label(f, text="?", width=3, font=("Consolas", 14, "bold"),
                           bg="#303030", fg="#909090")
            lbl.pack()
            self.bit_lbls[name] = lbl
        self.io_lbl = ttk.Label(io, text=IO_MEANING[0])
        self.io_lbl.pack(pady=(6, 0))
        self.useio_lbl = ttk.Label(io, text="", foreground="#a06000",
                                   wraplength=210, justify="left")
        self.useio_lbl.pack(pady=(4, 0))

        # --- relay control ---
        ctl = ttk.LabelFrame(left, text="Relay / motor", padding=8)
        ctl.pack(fill="x", pady=6)
        self.armed = tk.BooleanVar(value=False)
        ttk.Checkbutton(ctl, text="Armed (required to energise)",
                        variable=self.armed, command=self._update_arm).pack(anchor="w")
        self.on_btn = tk.Button(ctl, text="MOTOR ON", bg="#166a22", fg="white",
                                font=("Segoe UI", 11, "bold"), state="disabled",
                                command=self.cmd_on)
        self.on_btn.pack(fill="x", pady=(6, 2))

        timed = ttk.Frame(ctl)
        timed.pack(fill="x", pady=2)
        ttk.Label(timed, text="Timed run (ms):").pack(side="left")
        self.run_ms = tk.StringVar(value="2000")
        ttk.Entry(timed, textvariable=self.run_ms, width=8).pack(side="left", padx=4)
        self.run_btn = ttk.Button(timed, text="Run", state="disabled",
                                  command=self.cmd_run)
        self.run_btn.pack(side="left")

        self.seq_btn = ttk.Button(ctl, text="Run test sequence", state="disabled",
                                  command=self.run_sequence)
        self.seq_btn.pack(fill="x", pady=(6, 2))
        ttk.Button(ctl, text="Reset statistics",
                   command=lambda: self.send("!RST")).pack(fill="x", pady=2)
        ttk.Button(ctl, text="Clear latched fault",
                   command=lambda: self.send("!CLEAR")).pack(fill="x", pady=2)

        # --- CSV logging ---
        logf = ttk.LabelFrame(left, text="Sample log", padding=8)
        logf.pack(fill="x", pady=6)
        self.log_btn = ttk.Button(logf, text="Start CSV log", command=self.toggle_log)
        self.log_btn.pack(fill="x")
        self.log_lbl = ttk.Label(logf, text="not logging", foreground="#707070",
                                 wraplength=210, justify="left")
        self.log_lbl.pack(pady=(4, 0))

        # --- live plot ---
        right = ttk.Frame(self.tab_test)
        right.pack(side="left", fill="both", expand=True)
        self.fig = Figure(figsize=(7, 5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel("time (s)")
        self.ax.set_ylabel("current (mA)")
        self.ax.grid(alpha=0.3)
        # Build the artists ONCE and update their data on each tick.  Clearing
        # and rebuilding the axes 7x a second is slow enough to stall Tk's main
        # loop on a busy machine, which delays serial handling -- exactly the
        # kind of self-inflicted "serial problem" this GUI must not have.
        self.cur_line, = self.ax.plot([], [], lw=1.0, color="#1f77b4")
        self.thr_hi = self.ax.axhline(self.threshold_ma, color="#d62728",
                                      ls=":", lw=1)
        self.thr_lo = self.ax.axhline(-self.threshold_ma, color="#d62728",
                                      ls=":", lw=1)
        self.relay_fill = None
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        verdict = ttk.LabelFrame(right, text="Verdict", padding=6)
        verdict.pack(fill="x", padx=4, pady=(0, 4))
        self.verdict_txt = tk.Text(verdict, height=5, wrap="word")
        self.verdict_txt.pack(fill="x")

    def _build_capture_tab(self):
        bar = ttk.Frame(self.tab_cap, padding=6)
        bar.pack(fill="x")
        ttk.Label(bar, text="Duration (ms):").pack(side="left")
        self.cap_ms = tk.StringVar(value="400")
        ttk.Entry(bar, textvariable=self.cap_ms, width=8).pack(side="left", padx=4)
        self.cap_btn = ttk.Button(bar, text="Capture relay close", state="disabled",
                                  command=self.cmd_capture)
        self.cap_btn.pack(side="left", padx=6)
        ttk.Button(bar, text="Save CSV...", command=self.save_capture).pack(side="left")
        self.cap_lbl = ttk.Label(bar, text="no capture yet")
        self.cap_lbl.pack(side="left", padx=10)

        ttk.Label(self.tab_cap, padding=6, justify="left", wraplength=1100,
                  text="The board opens the relay, samples A0 as fast as it can for the "
                       "pre-trigger window (CAPPRE), closes the relay, and keeps sampling "
                       "to the end of the duration. Inrush peak, steady-state current and "
                       "time-to-steady come from that trace; a flat trace after the close "
                       "is the signature of a stalled or disconnected motor."
                  ).pack(fill="x")

        self.cap_fig = Figure(figsize=(9, 5), dpi=100)
        self.cap_ax = self.cap_fig.add_subplot(111)
        self.cap_ax.set_xlabel("time (ms)")
        self.cap_ax.set_ylabel("current (mA)")
        self.cap_ax.grid(alpha=0.3)
        self.cap_canvas = FigureCanvasTkAgg(self.cap_fig, master=self.tab_cap)
        self.cap_canvas.get_tk_widget().pack(fill="both", expand=True, padx=6, pady=6)

    def _build_cal_tab(self):
        f = ttk.Frame(self.tab_cal, padding=12)
        f.pack(fill="both", expand=True)

        ttk.Label(f, justify="left", wraplength=900, text=(
            "Two-point calibration.\n\n"
            "1. With the relay OPEN and no current flowing, press Capture zero. "
            "That stores the present A0 reading as 0 A (ZEROMV).\n"
            "2. Pass a known current through the sense element -- read it with a "
            "meter in series -- then type that value in mA and press Set "
            "sensitivity. That solves MVPERA = (mV - ZEROMV) * 1000 / mA.\n\n"
            "Prototype starting point: 1590 mV = 0 mA, 1630 mV = 80 mA, i.e. "
            "500 mV/A. Nothing is written to EEPROM until you press Save to "
            "EEPROM (or use !SAVE on the Configuration tab)."
        )).pack(anchor="w", pady=(0, 12))

        live = ttk.LabelFrame(f, text="Live reading", padding=10)
        live.pack(fill="x")
        self.cal_mv = tk.StringVar(value="--")
        self.cal_ma = tk.StringVar(value="--")
        ttk.Label(live, text="A0:").grid(row=0, column=0, sticky="e")
        ttk.Label(live, textvariable=self.cal_mv, font=("Consolas", 16),
                  width=12, anchor="e").grid(row=0, column=1)
        ttk.Label(live, text="mV").grid(row=0, column=2, sticky="w")
        ttk.Label(live, text="Current:").grid(row=1, column=0, sticky="e")
        ttk.Label(live, textvariable=self.cal_ma, font=("Consolas", 16),
                  width=12, anchor="e").grid(row=1, column=1)
        ttk.Label(live, text="mA").grid(row=1, column=2, sticky="w")

        step1 = ttk.LabelFrame(f, text="Step 1 -- zero", padding=10)
        step1.pack(fill="x", pady=8)
        ttk.Button(step1, text="Capture zero (!ZERO)",
                   command=self.cmd_zero).pack(side="left")
        self.zero_lbl = ttk.Label(step1, text="")
        self.zero_lbl.pack(side="left", padx=10)

        step2 = ttk.LabelFrame(f, text="Step 2 -- sensitivity", padding=10)
        step2.pack(fill="x")
        ttk.Label(step2, text="Known current now flowing (mA):").pack(side="left")
        self.cal_known = tk.StringVar(value="80")
        ttk.Entry(step2, textvariable=self.cal_known, width=10).pack(side="left", padx=4)
        ttk.Button(step2, text="Set sensitivity (!CAL)",
                   command=self.cmd_cal).pack(side="left", padx=6)
        self.cal_lbl = ttk.Label(step2, text="")
        self.cal_lbl.pack(side="left", padx=10)

        ttk.Button(f, text="Save to EEPROM (!SAVE)",
                   command=lambda: self.send("!SAVE")).pack(anchor="w", pady=12)

    def _build_config_tab(self):
        bar = ttk.Frame(self.tab_cfg, padding=6)
        bar.pack(fill="x")
        ttk.Button(bar, text="Read from device (!CFG)",
                   command=self.cfg_read).pack(side="left")
        ttk.Button(bar, text="Push changes (!SET)",
                   command=self.cfg_push).pack(side="left", padx=6)
        ttk.Button(bar, text="Save to EEPROM (!SAVE)",
                   command=self.cfg_save).pack(side="left")
        ttk.Button(bar, text="Reload EEPROM (!LOAD)",
                   command=lambda: self._cfg_cmd("!LOAD")).pack(side="left", padx=6)
        ttk.Button(bar, text="Factory defaults",
                   command=self.cfg_defaults).pack(side="left")
        self.cfg_lbl = ttk.Label(bar, text="")
        self.cfg_lbl.pack(side="left", padx=10)

        wrap = ttk.Frame(self.tab_cfg)
        wrap.pack(fill="both", expand=True, padx=6, pady=6)
        self.cfg_canvas = tk.Canvas(wrap, highlightthickness=0)
        sb = ttk.Scrollbar(wrap, orient="vertical", command=self.cfg_canvas.yview)
        self.cfg_inner = ttk.Frame(self.cfg_canvas)
        self.cfg_canvas.configure(yscrollcommand=sb.set)
        self.cfg_canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self.cfg_window = self.cfg_canvas.create_window((0, 0), window=self.cfg_inner,
                                                        anchor="nw")
        self.cfg_inner.bind("<Configure>", lambda _e: self.cfg_canvas.configure(
            scrollregion=self.cfg_canvas.bbox("all")))
        self.cfg_canvas.bind("<Configure>", lambda e: self.cfg_canvas.itemconfigure(
            self.cfg_window, width=e.width))

    def _build_log_tab(self):
        bar = ttk.Frame(self.tab_log, padding=6)
        bar.pack(fill="x")
        self.raw_cmd = tk.StringVar()
        e = ttk.Entry(bar, textvariable=self.raw_cmd, width=40)
        e.pack(side="left")
        e.bind("<Return>", lambda _ev: self.send_raw())
        ttk.Button(bar, text="Send", command=self.send_raw).pack(side="left", padx=4)
        ttk.Button(bar, text="!STATUS",
                   command=lambda: self.send("!STATUS")).pack(side="left")
        self.show_stream = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar, text="show $WH stream rows",
                        variable=self.show_stream).pack(side="left", padx=10)
        ttk.Button(bar, text="Clear", command=lambda: self.log_txt.delete(
            "1.0", "end")).pack(side="right")

        self.log_txt = tk.Text(self.tab_log, wrap="none", font=("Consolas", 9))
        self.log_txt.pack(fill="both", expand=True, padx=6, pady=6)

    # ─────────────────────────────────────────────────────────────
    #  Connection
    # ─────────────────────────────────────────────────────────────
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def toggle_connect(self):
        if self.serial.is_open or self.reconnecting:
            self.reconnecting = False          # cancel any retry in flight
            if self.serial.is_open:
                self.send("!OFF")              # never leave the motor running
            self.serial.disconnect()
            self.port_in_use = None
            self._set_connected(False)
            return
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("WireHawk", "Pick a serial port first.")
            return
        try:
            self.serial.connect(port)
        except Exception as exc:
            messagebox.showerror("WireHawk", "Could not open %s:\n%s" % (port, exc))
            return
        self.port_in_use = port
        self._reset_history()
        self._set_connected(True)
        self._after_link_up()

    def _after_link_up(self):
        self._link_up_at = time.monotonic()
        self.stream_seen = False
        self.after(600, lambda: self.send("!STATUS"))
        # Pull the board's dropout history: it survived the outage, so it can
        # say whether the device really left the bus and what the motor was
        # drawing at that moment.
        self.after(750, lambda: self.send("!LINK"))
        # Recover the samples the outage cost us, if we know where we left off.
        if self.last_board_ms is not None:
            self.after(900, lambda: self.send("!HIST,%d" % self.last_board_ms))
        self.after(1300, self.cfg_read)

    def _reset_history(self):
        """The board's millis() restarts on reset, so old history cannot be
        stitched onto new samples."""
        self.samples.clear()
        self.last_board_ms = None
        self.hist_rows = []
        self.relay = 0

    def _begin_reconnect(self, why):
        """The link died. Drop it and, unless the operator opted out, keep
        trying to reopen the same port."""
        if self.reconnecting:
            return
        was_energised = self.relay == 1
        self.serial.disconnect()
        self._set_connected(False)
        self._log("!! link lost: %s" % why)
        # The last telemetry is now stale. If the relay was closed the motor may
        # STILL be turning, so the banner must not keep showing a reassuring
        # RUNNING -- say plainly that we no longer know.
        self._paint_link_lost(was_energised, why)
        if not self.auto_reconnect.get() or not self.port_in_use:
            return
        self.reconnecting = True
        self._reconnect_left = RECONNECT_TRIES
        self.conn_lbl.config(text="reconnecting...", foreground="#a08000")
        self.after(RECONNECT_MS, self._try_reconnect)

    def _note_wedge(self):
        """Reopening the port cannot revive a board whose USB stack is stuck --
        only a reset can.  Say so rather than retrying silently forever."""
        self._log("!! reopened the port but the board is not talking. If this "
                  "persists, the board's USB stack is wedged and only a reset "
                  "clears it -- reflash with the current firmware, which never "
                  "spins inside Serial.write().")

    def _try_reconnect(self):
        if not self.reconnecting:
            return
        try:
            self.serial.connect(self.port_in_use)
        except Exception:
            self._reconnect_left -= 1
            if self._reconnect_left <= 0:
                self.reconnecting = False
                self.conn_lbl.config(text="disconnected", foreground="#a05050")
                self.detail_lbl.config(text="could not reopen %s -- press Connect"
                                            % self.port_in_use)
                self._log("!! gave up reconnecting to %s" % self.port_in_use)
                return
            self.after(RECONNECT_MS, self._try_reconnect)
            return
        self.reconnecting = False
        # Deliberately NOT clearing the history: the board keeps running through
        # these dropouts, so its millis() is still continuous and the samples we
        # missed can be recovered with !HIST.  Only a real reset ($BOOT) makes
        # the old history unusable.
        self._set_connected(True)
        self._log("== reconnected to %s" % self.port_in_use)
        # If the board never reset, the relay may STILL be closed -- an
        # unexpected link loss is a fault condition, so de-energise first and
        # make the operator re-arm deliberately.
        self.serial.send("!STOP")
        self._log(">> !STOP (safety, after reconnect)")
        self._flush_deferred()
        self._after_link_up()
        # If reopening the port did not actually revive the board, say why.
        self.after(2500, lambda: None if self.serial.last_rx > self._link_up_at
                   else self._note_wedge())

    def _set_connected(self, ok):
        self.connect_btn.config(text="Disconnect" if ok else "Connect")
        self.conn_lbl.config(text="connected" if ok else "disconnected",
                             foreground="#308030" if ok else "#a05050")
        for w in (self.cap_btn, self.seq_btn):
            w.config(state="normal" if ok else "disabled")
        if not ok:
            self.armed.set(False)
        self._update_arm()

    def _update_arm(self):
        ok = self.serial.is_open and self.armed.get()
        self.on_btn.config(state="normal" if ok else "disabled")
        self.run_btn.config(state="normal" if ok else "disabled")

    def send(self, text):
        if not self.serial.is_open:
            # Never silently swallow an operator action -- least of all MOTOR
            # OFF. Hold the safe ones and replay them once the link returns;
            # refuse the ones that would energise the motor.
            head = text.split(",")[0].upper()
            if head in QUEUEABLE:
                self.deferred.append(text)
                self._log("(link down) %s -- queued, will send on reconnect" % text)
            else:
                self._log("(link down) %s -- NOT SENT" % text)
                self.detail_lbl.config(text="link is down: %s was not sent" % text)
            return
        self._log(">> " + text)
        self.serial.send(text)

    def _flush_deferred(self):
        pending, self.deferred = self.deferred, []
        for cmd in pending:
            self._log(">> %s (queued while the link was down)" % cmd)
            self.serial.send(cmd)

    def send_raw(self):
        cmd = self.raw_cmd.get().strip()
        if cmd:
            self.send(cmd)
            self.raw_cmd.set("")

    # ─────────────────────────────────────────────────────────────
    #  Relay commands
    # ─────────────────────────────────────────────────────────────
    def cmd_on(self):
        if not self.armed.get():
            return
        self.send("!ON")

    def cmd_off(self):
        self.send("!OFF")

    def cmd_run(self):
        if not self.armed.get():
            return
        try:
            ms = int(float(self.run_ms.get()))
        except ValueError:
            messagebox.showwarning("WireHawk", "Timed run needs a number of ms.")
            return
        self.send("!RUN,%d" % ms)

    def cmd_capture(self):
        try:
            ms = int(float(self.cap_ms.get()))
        except ValueError:
            messagebox.showwarning("WireHawk", "Capture duration needs a number of ms.")
            return
        if not self.armed.get() and not messagebox.askyesno(
                "WireHawk", "A capture CLOSES THE RELAY and energises the motor for "
                            "%d ms.\n\nContinue?" % ms):
            return
        self.cap_rows = []
        self.cap_header = None
        self.cap_active = True
        self.cap_started = time.monotonic()
        self.cap_lbl.config(text="capturing...")
        self.send("!CAP,%d" % ms)

    def cmd_zero(self):
        self.send("!ZERO")

    def cmd_cal(self):
        try:
            ma = float(self.cal_known.get())
        except ValueError:
            messagebox.showwarning("WireHawk", "Known current needs a number in mA.")
            return
        self.send("!CAL,%g" % ma)

    # ─────────────────────────────────────────────────────────────
    #  Scripted test sequence
    # ─────────────────────────────────────────────────────────────
    def run_sequence(self):
        if not messagebox.askyesno(
                "WireHawk", "The test sequence energises the motor briefly.\n\n"
                            "1. Relay open -- record the idle sense state\n"
                            "2. Timed inrush capture across the relay close\n"
                            "3. Report a verdict\n\nContinue?"):
            return
        self._set_verdict("Running sequence...\n")
        self.send("!OFF")
        self.send("!RST")
        self.after(500, self._seq_step2)

    def _seq_step2(self):
        self.seq_idle_state = self.state_code
        self.seq_idle_io = self.io_code
        self._append_verdict("Idle: %s, sense %s\n" % (
            STATES.get(self.seq_idle_state, ("?",))[0],
            IO_MEANING.get(self.seq_idle_io, "?")))
        try:
            ms = max(100, int(float(self.cap_ms.get())))
        except ValueError:
            ms = 400
        self.cap_rows = []
        self.cap_header = None
        self.cap_active = True
        self.cap_started = time.monotonic()
        self.cap_on_done = self._seq_verdict
        self.cap_lbl.config(text="capturing (sequence)...")
        self.send("!CAP,%d" % ms)

    def _seq_verdict(self):
        r = self.cap_result
        if not r:
            self._append_verdict("Capture returned no data.\n")
            return
        self._append_verdict(
            "Capture: inrush peak %.1f mA, steady %.1f mA, time to steady %.0f ms\n"
            % (r["peak"], r["steady"], r["t_steady"]))

        thr = self.threshold_ma
        if self.seq_idle_state == 2:
            self._append_verdict("VERDICT: no motor -- open circuit before the test.")
        elif abs(r["steady"]) >= thr:
            self._append_verdict("VERDICT: motor running (steady current above the "
                                 "%.0f mA threshold)." % thr)
        elif abs(r["peak"]) >= thr:
            self._append_verdict(
                "VERDICT: current appeared then collapsed -- inrush without a sustained "
                "draw. Suspect a stall that tripped protection, or an intermittent "
                "connection.")
        else:
            self._append_verdict(
                "VERDICT: no current with the relay closed. If the voltage sense bit "
                "reads 11 this is a motor fault stall; if it reads 10 the load never "
                "got voltage (relay, fuse, supply). Sense at the end of the capture: "
                "%s." % IO_MEANING.get(r["io_end"], "?"))

    def _set_verdict(self, text):
        self.verdict_txt.delete("1.0", "end")
        self.verdict_txt.insert("end", text)

    def _append_verdict(self, text):
        self.verdict_txt.insert("end", text)
        self.verdict_txt.see("end")

    # ─────────────────────────────────────────────────────────────
    #  CSV logging of the live stream
    # ─────────────────────────────────────────────────────────────
    def toggle_log(self):
        if self.log_writer:
            self._stop_log()
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", filetypes=[("CSV", "*.csv")],
            initialfile="wirehawk_%s.csv" % datetime.now().strftime("%Y%m%d_%H%M%S"))
        if not path:
            return
        try:
            self.log_file = open(path, "w", newline="")
        except Exception as exc:
            messagebox.showerror("WireHawk", str(exc))
            return
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(["ms", "relay", "mA", "avgMA", "minMA", "maxMA",
                                  "peakMA", "rippleMA", "mV", "d3", "d4", "io",
                                  "state", "stateName", "runMs",
                                  "seq", "maxLoopUs", "txSkips", "src"])
        self.log_btn.config(text="Stop CSV log")
        self.log_lbl.config(text="logging to " + os.path.basename(path),
                            foreground="#308030")

    def _stop_log(self):
        if self.log_file:
            try:
                self.log_file.close()
            except Exception:
                pass
        self.log_file = None
        self.log_writer = None
        self.log_btn.config(text="Start CSV log")
        self.log_lbl.config(text="not logging", foreground="#707070")

    def save_capture(self):
        if not self.cap_rows:
            messagebox.showinfo("WireHawk", "No capture to save yet.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", filetypes=[("CSV", "*.csv")],
            initialfile="wirehawk_capture_%s.csv"
                        % datetime.now().strftime("%Y%m%d_%H%M%S"))
        if not path:
            return
        h = self.cap_header or {}
        try:
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["# trigger_us", h.get("trig_us", "")])
                w.writerow(["# zero_mv", h.get("zero_mv", ""),
                            "mv_per_amp", h.get("mv_per_amp", "")])
                w.writerow(["t_us", "t_ms", "raw", "mV", "mA", "io"])
                for t_us, raw, io in self.cap_rows:
                    mv = self._raw_to_mv(raw)
                    w.writerow([t_us, t_us / 1000.0, raw, "%.1f" % mv,
                                "%.2f" % self._mv_to_ma(mv), io])
        except Exception as exc:
            messagebox.showerror("WireHawk", str(exc))
            return
        self.cap_lbl.config(text="saved " + os.path.basename(path))

    # ─────────────────────────────────────────────────────────────
    #  Configuration table
    # ─────────────────────────────────────────────────────────────
    def _cfg_cmd(self, cmd):
        self.send(cmd)
        self.after(400, self.cfg_read)

    def cfg_read(self):
        if not self.serial.is_open:
            return
        self.cfg_pending = True
        self.cfg_lbl.config(text="reading...")
        self.send("!CFG")

    def cfg_push(self):
        n = 0
        for key, row in self.cfg_rows.items():
            new = row["var"].get().strip()
            if new != row["value"]:
                self.send("!SET,%s,%s" % (key, new))
                n += 1
        self.cfg_lbl.config(text="pushed %d change(s)" % n if n else "no changes")

    def cfg_save(self):
        self.cfg_push()
        self.after(300, lambda: self.send("!SAVE"))

    def cfg_defaults(self):
        if messagebox.askyesno("WireHawk", "Load factory defaults into device RAM?\n"
                                           "(EEPROM keeps its values until you Save.)"):
            self._cfg_cmd("!DEFAULTS")

    def _cfg_apply(self, values):
        """Rebuild the config table from a completed !CFG dump."""
        for w in self.cfg_inner.winfo_children():
            w.destroy()
        self.cfg_rows = {}

        groups = {}
        for key, val in values.items():
            meta = KEY_META.get(key)
            group = meta[0] if meta else "Other"
            groups.setdefault(group, []).append((key, val, meta))

        for group in GROUP_ORDER + [g for g in groups if g not in GROUP_ORDER]:
            items = groups.get(group)
            if not items:
                continue
            frame = ttk.LabelFrame(self.cfg_inner, text=group, padding=8)
            frame.pack(fill="x", pady=4)
            frame.columnconfigure(2, weight=1)
            for r, (key, val, meta) in enumerate(sorted(items)):
                ttk.Label(frame, text=key, font=("Consolas", 9),
                          width=12).grid(row=r, column=0, sticky="w")
                var = tk.StringVar(value=val)
                kind = meta[1] if meta else "num"
                if kind == "bool":
                    ed = ttk.Combobox(frame, textvariable=var, width=8,
                                      values=["0", "1"], state="readonly")
                elif kind == "choice":
                    ed = ttk.Combobox(frame, textvariable=var, width=8,
                                      values=meta[3], state="readonly")
                else:
                    ed = ttk.Entry(frame, textvariable=var, width=10)
                ed.grid(row=r, column=1, padx=6, sticky="w")
                desc = meta[2] if meta else "(unknown to this GUI -- firmware key)"
                ttk.Label(frame, text=desc, foreground="#707070").grid(
                    row=r, column=2, sticky="w")
                self.cfg_rows[key] = {"value": val, "editor": ed, "var": var}

        self.cfg_lbl.config(text="%d keys" % len(self.cfg_rows))
        self._sync_from_config()

    def _sync_from_config(self):
        """Pull the few config values the rest of the GUI needs."""
        try:
            self.threshold_ma = float(self.cfg_rows["CURTHRESH"]["value"])
        except (KeyError, ValueError):
            pass
        try:
            # with streaming off, silence is expected -- don't let the
            # stale-link watchdog mistake it for a dead link
            self.expect_stream = int(float(self.cfg_rows["STREAM"]["value"])) != 0
        except (KeyError, ValueError):
            pass
        try:
            self.host_to_ms = int(float(self.cfg_rows["HOSTTO"]["value"]))
        except (KeyError, ValueError):
            self.host_to_ms = None
        if self.host_to_ms == 0:
            self._log("!! HOSTTO=0: the board will NOT drop the relay by itself if "
                      "the link dies -- only MAXRUNMS will. Set HOSTTO=2000 on the "
                      "Configuration tab.")
        try:
            use_io = int(float(self.cfg_rows["USEIO"]["value"]))
        except (KeyError, ValueError):
            use_io = None
        if use_io == 0:
            self.useio_lbl.config(
                text="USEIO=0: sense bits reported but ignored. Set USEIO=1 on the "
                     "Configuration tab once D3/D4 are wired.")
        elif use_io == 1:
            self.useio_lbl.config(text="")

    # ─────────────────────────────────────────────────────────────
    #  Line handling
    # ─────────────────────────────────────────────────────────────
    def _pump(self):
        cfg_batch = getattr(self, "_cfg_batch", {})
        try:
            while True:
                kind, payload = self.line_queue.get_nowait()
                if kind == "__error__":
                    self._begin_reconnect(payload)
                    continue
                self._handle_line(payload, cfg_batch)
        except queue.Empty:
            pass
        self._cfg_batch = cfg_batch
        self._check_link()
        self.after(60, self._pump)

    def _check_link(self):
        """A dropped native-USB link does not always raise on read -- it can
        just go quiet.  If the board was streaming and everything stops, treat
        the link as dead."""
        if not (self.serial.is_open and not self.reconnecting
                and self.stream_seen and self.expect_stream):
            return

        # A capture legitimately blocks the board for its whole duration and
        # then dumps thousands of rows, so silence there is expected, not a
        # dead link.  Bounded, so a lost $CAPEND cannot disable the watchdog.
        limit = STALE_MS / 1000.0
        if self.cap_active:
            if time.monotonic() - self.cap_started < CAPTURE_GRACE_MS / 1000.0:
                return
            self.cap_active = False
            self._log("!! capture did not finish within %.0f s" % (CAPTURE_GRACE_MS / 1000.0))

        quiet = time.monotonic() - self.serial.last_rx
        if quiet > limit:
            self._begin_reconnect("no data for %.1f s" % quiet)

    def _handle_line(self, line, cfg_batch):
        parts = line.split(",")
        tag = parts[0]

        if tag == "$WH":
            self.stream_seen = True
            self._on_stream(parts)
            if self.show_stream.get():
                self._log(line)
            return

        if tag == "$H":
            try:
                self.hist_rows.append((int(parts[1]), float(parts[2]),
                                       int(parts[3]), int(parts[4]),
                                       int(parts[5])))
            except (IndexError, ValueError):
                pass
            return

        if tag == "$PONG":
            if self.show_stream.get():
                self._log(line)
            return

        if tag == "$CAP":
            if len(parts) >= 4:
                try:
                    self.cap_rows.append((int(parts[1]), int(parts[2]), int(parts[3])))
                except ValueError:
                    pass
            return

        self._log(line)

        if tag == "$CAPSTART":
            self.cap_rows = []
            self.cap_active = True
            self.cap_started = time.monotonic()
            try:
                self.cap_header = {
                    "n": int(parts[1]), "trig_us": int(parts[2]),
                    "dur_ms": int(parts[3]), "full_scale": int(parts[4]),
                    "vref_mv": int(parts[5]), "zero_mv": float(parts[6]),
                    "mv_per_amp": float(parts[7]),
                }
            except (IndexError, ValueError):
                self.cap_header = None
        elif tag == "$CAPEND":
            self.cap_active = False
            self._on_capture_done()
        elif tag == "$STATE":
            try:
                self.state_code = int(parts[1])
            except (IndexError, ValueError):
                return
            self._paint_state(parts[3] if len(parts) > 3 else "")
        elif tag == "$CFG":
            if len(parts) >= 3:
                cfg_batch[parts[1]] = ",".join(parts[2:])
        elif tag == "$CFGEND":
            if cfg_batch:
                self._cfg_apply(dict(cfg_batch))
                cfg_batch.clear()
        elif tag == "$BOOT":
            # The board restarted, so millis() and the relay both went back to
            # zero.  Say why -- BROWNOUT/PANIC means the relay or motor is
            # crashing the board, not just the USB link.
            reason = parts[1] if len(parts) > 1 else "?"
            if len(parts) > 3 and parts[3]:
                # one firmware serves both boards; show which one is connected
                self.board = parts[3]
                self.sn_lbl.config(text="%s  |  SN: %s" % (
                    parts[3], self.sn_text or "unassigned"))
            self._reset_history()
            self.armed.set(False)
            self._update_arm()
            self.detail_lbl.config(
                text="board RESET (%s) -- relay is open, motor de-energised" % reason)
            self._log("== board reset, reason %s" % reason)
            if reason in ("BROWNOUT", "PANIC", "INT_WDT", "TASK_WDT", "WDT"):
                self._append_verdict(
                    "\nBoard reset on %s: the relay/motor is disturbing the board's own "
                    "supply, not just the USB link. Check the coil flyback diode, add a "
                    "snubber across the contacts, and power the relay/motor from a "
                    "separate supply.\n" % reason)
        elif tag == "$SN":
            self.sn_text = parts[1] if len(parts) > 1 and parts[1] else ""
            label = "SN: " + (self.sn_text or "unassigned")
            if self.board:
                label = "%s  |  %s" % (self.board, label)
            self.sn_lbl.config(text=label)
        elif tag == "$OK" and len(parts) > 1:
            if parts[1] == "zero" and len(parts) > 2:
                self.zero_lbl.config(text="zero = %s mV" % parts[2])
                self.after(300, self.cfg_read)
            elif parts[1] == "cal" and len(parts) > 2:
                self.cal_lbl.config(text="sensitivity = %s mV/A" % parts[2])
                self.after(300, self.cfg_read)
            elif parts[1] in ("save", "load", "defaults"):
                self.cfg_lbl.config(text=parts[1] + " ok")
        elif tag == "$STATUS":
            self._diagnose_status(parts[1:])
        elif tag == "$LINK":
            self.link_events = []
            try:
                self.link_drops = int(parts[1])
            except (IndexError, ValueError):
                self.link_drops = 0
        elif tag == "$LINKEV":
            try:
                self.link_events.append(
                    (int(parts[2]), int(parts[3]), int(parts[4]),
                     float(parts[5]), int(parts[6])))
            except (IndexError, ValueError):
                pass
        elif tag == "$LINKEND":
            self._report_link_log()
        elif tag == "$HIST":
            self.hist_rows = []
        elif tag == "$HISTEND":
            self._merge_history()
        elif tag == "$ERR":
            self.detail_lbl.config(text=line)

    def _on_stream(self, p):
        # $WH,ms,relay,mA,avg,min,max,pk,ripple,mV,d3,d4,io,state,name,runMs
        if len(p) < 16:
            return
        try:
            ms = int(p[1]); relay = int(p[2])
            ma = float(p[3]); avg = float(p[4]); mn = float(p[5]); mx = float(p[6])
            pk = float(p[7]); ripple = float(p[8]); mv = float(p[9])
            d3 = int(p[10]); d4 = int(p[11]); io = int(p[12])
            state = int(p[13]); run_ms = int(p[15])
        except ValueError:
            return

        # Optional diagnostics (older firmware omits them)
        seq = max_loop_us = tx_skips = None
        if len(p) >= 19:
            try:
                seq = int(p[16]); max_loop_us = int(p[17]); tx_skips = int(p[18])
            except ValueError:
                pass
        if seq is not None:
            self._check_seq(seq, tx_skips, max_loop_us, ms)

        if self.log_writer:
            row = list(p[1:19])
            self.log_writer.writerow(row + [""] * (18 - len(row)) + ["live"])

        self.cur_lbl.config(text="%.1f" % ma)
        self.mv_lbl.config(text="A0: %.1f mV" % mv)
        self.cal_mv.set("%.1f" % mv)
        self.cal_ma.set("%.2f" % ma)
        self.stat_vars["avg"].set("%.1f mA" % avg)
        self.stat_vars["min"].set("%.1f mA" % mn)
        self.stat_vars["max"].set("%.1f mA" % mx)
        self.stat_vars["pk"].set("%.1f mA" % pk)
        self.stat_vars["ripple"].set("%.1f mA" % ripple)
        self.stat_vars["run"].set("%.1f s" % (run_ms / 1000.0) if relay else "--")
        if max_loop_us is not None:
            self.stat_vars["loop"].set("%.2f ms" % (max_loop_us / 1000.0))
            self.stat_vars["lost"].set("%d / %d" % (self.lost_rows, self.skipped_rows))

        for name, bit in (("D4", d4), ("D3", d3)):
            self.bit_lbls[name].config(
                text=str(bit),
                bg="#166a22" if bit else "#303030",
                fg="white" if bit else "#909090")
        self.io_code = io
        self.io_lbl.config(text=IO_MEANING.get(io, "?"))

        if state != self.state_code:
            self.state_code = state
            self._paint_state()

        self.samples.append((ms, ma, relay))
        self.last_board_ms = ms
        self.relay = relay

    def _check_seq(self, seq, tx_skips, max_loop_us, board_ms):
        """Attribute every missing row to a side.

        seq counts rows the BOARD decided to send, so a jump means rows that
        never reached us.  txSkips counts rows the board itself threw away
        because our end was not draining the CDC pipe.  The difference between
        the two is the number lost in transit.  maxLoopUs shows whether the
        firmware was stalled while all this happened.
        """
        prev, self.last_seq = self.last_seq, seq
        prev_skips, self.last_tx_skips = self.last_tx_skips, tx_skips or 0

        if prev is None or seq <= prev:      # first row, or the board restarted
            return
        gap = seq - prev - 1
        if gap <= 0:
            return

        # Clamp to the gap so the two causes always sum to the rows actually
        # missing -- the counters can disagree by one around a row boundary.
        skipped = min(gap, max(0, (tx_skips or 0) - prev_skips))
        lost = gap - skipped
        self.skipped_rows += skipped
        self.lost_rows += lost
        self.stat_vars["lost"].set("%d / %d" % (self.lost_rows, self.skipped_rows))

        parts = []
        if skipped:
            # Deliberately does NOT claim backpressure: the board also counts a
            # row as skipped when CDC reports no host at all, which is a
            # bus-level drop.  $LINK / txnohost say which it was.
            parts.append("%d the board could not transmit (see !LINK)" % skipped)
        if lost:
            parts.append("%d left the board and were LOST IN TRANSIT" % lost)
        loop_ms = (max_loop_us or 0) / 1000.0
        health = ("board loop healthy, max %.1f ms" % loop_ms if loop_ms < 50
                  else "BOARD LOOP STALLED, max %.1f ms" % loop_ms)
        self._log("!! %d stream row(s) missing at seq %d: %s (%s)"
                  % (gap, seq, "; ".join(parts) or "cause unclear", health))

    def _diagnose_status(self, fields):
        """Turn the board's own counters into a verdict about who dropped what."""
        vals = {}
        for f in fields:
            if "=" in f:
                k, v = f.split("=", 1)
                vals[k] = v

        def num(key):
            try:
                return int(float(vals.get(key, 0)))
            except ValueError:
                return 0

        nohost, miss, drops = num("txnohost"), num("txmiss"), num("usbdrops")
        if nohost and not miss:
            self._log(
                "!! %d row(s) dropped with the CDC link reporting NO HOST "
                "(usbdrops=%d, txmiss=0). The board was running fine; its USB "
                "device left the bus. Electrical, not data rate: motor return "
                "current in the shared ground shifts this board's ground away "
                "from the PC's, and USB tolerates very little common-mode. "
                "Star-ground the board to the supply 0V, snub the motor, then "
                "isolate the USB." % (nohost, drops))
        elif miss:
            self._log("!! %d write(s) hit the deadline with a host connected "
                      "(txmiss=%d): this end really was too slow to drain. "
                      "Raise RATEMS." % (miss, miss))

    def _merge_history(self):
        """Fold samples recovered with !HIST back into the live history.

        Merged on the board's own millis(), so rows that arrived late land in
        the right place even though they were received after newer ones.
        """
        rows, self.hist_rows = self.hist_rows, []
        if not rows:
            return

        have = {ms for ms, _ma, _r in self.samples}
        added = [(ms, ma, relay) for ms, ma, _st, _io, relay in rows
                 if ms not in have]
        if not added:
            return

        merged = sorted(list(self.samples) + added, key=lambda s: s[0])
        self.samples.clear()
        self.samples.extend(merged[-LIVE_MAXLEN:])
        self.backfilled += len(added)

        if self.log_writer:
            for ms, ma, _st, _io, relay in rows:
                row = [""] * 18
                row[0], row[1], row[2] = ms, relay, "%.2f" % ma
                self.log_writer.writerow(row + ["hist"])

        span = (added[-1][0] - added[0][0]) / 1000.0
        peak = max((abs(a) for _m, a, _r in added), default=0.0)
        self._log("== backfilled %d sample(s) from the board covering %.1f s "
                  "(peak %.0f mA) -- the dropout cost no data"
                  % (len(added), span, peak))

    def _report_link_log(self):
        """Print the board's record of dropouts it survived."""
        if not self.link_events:
            self._log("== board recorded NO USB dropouts. If this end saw the "
                      "device disappear, it never actually left the bus -- look "
                      "at the host driver, hub or cable, not the board.")
            return
        self._log("== board recorded %d USB dropout(s), most recent last:"
                  % self.link_drops)
        for at_ms, dur_ms, relay, ma, run_ms in self.link_events:
            self._log("     t=%.1fs  down %.0f ms  relay=%s  %.0f mA  "
                      "%.1fs into the run"
                      % (at_ms / 1000.0, dur_ms, "CLOSED" if relay else "open",
                         ma, run_ms / 1000.0))
        energised = [e for e in self.link_events if e[2]]
        if energised and len(energised) == len(self.link_events):
            self._log("     every dropout happened with the relay CLOSED -- the "
                      "running motor is causing them, via the shared ground "
                      "rather than the supply rail. Star-ground first: the "
                      "board's ground must reach the 24V 0V on its own "
                      "conductor, not through the motor's return path.")

    def _paint_link_lost(self, was_energised, why):
        bg = "#7a3000" if was_energised else "#3a3a3a"
        self.state_lbl.config(text="LINK LOST", bg=bg, fg="#ffd0a0")
        self.state_lbl.master.config(bg=bg)
        self.detail_lbl.config(
            bg=bg, fg="#ffd0a0",
            text=("%s -- relay was CLOSED, the motor may still be running "
                  "(board drops it after HOSTTO)" % why) if was_energised
                 else "%s -- relay was open" % why)
        # force the next real telemetry to repaint the banner
        self.state_code = -1

    def _paint_state(self, detail=None):
        label, bg, fg = STATES.get(self.state_code, ("?", "#303030", "#c8c8c8"))
        self.state_lbl.config(text=label, bg=bg, fg=fg)
        self.state_lbl.master.config(bg=bg)
        self.detail_lbl.config(bg=bg, fg=fg)
        if detail:
            self.detail_lbl.config(text=detail)

    def _ping_tick(self):
        """Keepalive while the motor is energised. The board arms its own
        HOSTTO relay cutoff off the back of these, so if this GUI (or the link,
        or the host) dies mid-run the board de-energises by itself."""
        if self.serial.is_open and self.relay == 1:
            self.serial.send("!PING")
        self.after(PING_MS, self._ping_tick)

    # ─────────────────────────────────────────────────────────────
    #  Capture analysis
    # ─────────────────────────────────────────────────────────────
    def _raw_to_mv(self, raw):
        h = self.cap_header or {}
        full = h.get("full_scale") or 4095
        vref = h.get("vref_mv") or 3300
        return raw * vref / float(full)

    def _mv_to_ma(self, mv):
        h = self.cap_header or {}
        zero = h.get("zero_mv", 1590.0)
        span = h.get("mv_per_amp", 500.0) or 500.0
        return (mv - zero) * 1000.0 / span

    def _on_capture_done(self):
        if not self.cap_rows:
            self.cap_lbl.config(text="capture returned no rows")
            if self.cap_on_done:
                cb, self.cap_on_done = self.cap_on_done, None
                cb()
            return

        trig_us = (self.cap_header or {}).get("trig_us", 0)
        t_ms = [(t - trig_us) / 1000.0 for t, _r, _io in self.cap_rows]
        ma = [self._mv_to_ma(self._raw_to_mv(r)) for _t, r, _io in self.cap_rows]

        post = [(t, i) for t, i in zip(t_ms, ma) if t >= 0]
        peak = max((abs(i) for _t, i in post), default=0.0)
        tail = [i for t, i in post if t >= post[-1][0] * 0.6] if post else []
        steady = sum(tail) / len(tail) if tail else 0.0

        # time to settle within 20% of the steady value
        t_steady = post[-1][0] if post else 0.0
        band = max(abs(steady) * 0.2, self.threshold_ma * 0.25, 1.0)
        for t, i in post:
            if abs(i - steady) <= band:
                t_steady = t
                break

        self.cap_result = {
            "peak": peak, "steady": steady, "t_steady": t_steady,
            "io_end": self.cap_rows[-1][2],
        }
        self.cap_lbl.config(
            text="%d samples  peak %.1f mA  steady %.1f mA  settle %.0f ms"
                 % (len(self.cap_rows), peak, steady, t_steady))

        ax = self.cap_ax
        ax.clear()
        ax.set_xlabel("time (ms, 0 = relay close)")
        ax.set_ylabel("current (mA)")
        ax.grid(alpha=0.3)
        ax.plot(t_ms, ma, lw=1.0, color="#1f77b4")
        ax.axvline(0, color="#666666", ls="--", lw=1, label="relay close")
        ax.axhline(self.threshold_ma, color="#d62728", ls=":", lw=1,
                   label="threshold %.0f mA" % self.threshold_ma)
        if steady:
            ax.axhline(steady, color="#2ca02c", ls="-.", lw=1,
                       label="steady %.1f mA" % steady)
        ax.legend(loc="best", fontsize=8)
        self.cap_canvas.draw_idle()

        if self.cap_on_done:
            cb, self.cap_on_done = self.cap_on_done, None
            cb()

    # ─────────────────────────────────────────────────────────────
    #  Live plot
    # ─────────────────────────────────────────────────────────────
    def _redraw(self):
        if self.samples:
            ax = self.ax
            ms0 = self.samples[0][0]
            t = [(s[0] - ms0) / 1000.0 for s in self.samples]
            i = [s[1] for s in self.samples]
            r = [s[2] for s in self.samples]
            self.cur_line.set_data(t, i)
            self.thr_hi.set_ydata([self.threshold_ma, self.threshold_ma])
            self.thr_lo.set_ydata([-self.threshold_ma, -self.threshold_ma])

            lo, hi = min(i), max(i)
            span = max(hi - lo, self.threshold_ma * 2, 10.0)
            mid = (hi + lo) / 2.0
            ax.set_ylim(mid - span * 0.7, mid + span * 0.7)
            if t[-1] > t[0]:
                ax.set_xlim(t[0], t[-1])

            # one collection, replaced per tick, instead of N axvspan patches
            if self.relay_fill is not None:
                self.relay_fill.remove()
            self.relay_fill = ax.fill_between(
                t, 0, 1, where=[v == 1 for v in r],
                transform=ax.get_xaxis_transform(),
                color="#2ca02c", alpha=0.10, step="post")
            self.canvas.draw_idle()
        self.after(PLOT_REFRESH_MS, self._redraw)

    # ─────────────────────────────────────────────────────────────
    #  Misc
    # ─────────────────────────────────────────────────────────────
    def _log(self, line):
        self.log_txt.insert("end", line + "\n")
        self.log_txt.see("end")
        if int(self.log_txt.index("end-1c").split(".")[0]) > LOG_MAXLINES:
            self.log_txt.delete("1.0", "500.0")

    def _on_close(self):
        self.reconnecting = False
        if self.serial.is_open:
            self.serial.send("!OFF")
        self._stop_log()
        self.serial.disconnect()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
