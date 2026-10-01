#!/usr/bin/env python3
"""
blinkyhawk_testsuite_gui.py  --  the BlinkyHawk bench GUI plus automated testing.

This IS the BlinkyHawk_Unified GUI (ArduinoProgrammingFiles/BlinkyHawk_Unified/
blinkyhawk_gui.py, imported and subclassed, not copied -- so its Diagnostics /
Configuration / Power tabs, and its per-unit CSV log, are the same ones), with
three more tabs.  (If that folder is missing it falls back to the older
BlinkyHawk_Bench GUI.)

  Test Rig        connect and drive the relay jig (and its AS7343 LED watcher),
                  the Rigol DG852 Pro and the Siglent scope by hand.  Type SIM in any port/IP box (or press
                  "Simulate everything") to dry-run with nothing plugged in.
  Test Sequence   the big test: every selected load / DC level / AC level in
                  turn, the BlinkyHawk's own readings + decisions logged per
                  detection pass, a scope capture per condition, a pass/fail
                  table, and a run folder under Runs/.  "Unit on battery"
                  judges by the LED watcher alone, with no serial link at all.
  Auto-Tune       finds REFCENTER/REFBAND for a target trip voltage and the
                  detection method + threshold that reads CLOSED at the chosen
                  resistance and OPEN above it -- or says it cannot, and shows
                  which loads CAN be told apart.  Applies to RAM only; Save or
                  Revert afterwards.  By default it measures ON BATTERY with the
                  unit floating (you unplug and replug USB when told; see
                  bh_battery.py) -- numbers taken over USB do not hold on battery.

Needs BlinkyHawk_Unified firmware (!DETLOG, and !SLEEPLOG,2 / !SLEEPLOG,D for
battery tuning).

Dependencies:  pip install pyserial matplotlib
Run:           py blinkyhawk_testsuite_gui.py
"""

import os
import queue
import sys
import threading
import time
import tkinter as tk
import traceback
from datetime import datetime
from tkinter import messagebox, ttk

import serial
import serial.tools.list_ports

HERE = os.path.dirname(os.path.abspath(__file__))
FW_DIR = os.path.normpath(os.path.join(HERE, "..", "..", "ArduinoProgrammingFiles"))
UNIFIED_DIR = os.path.join(FW_DIR, "BlinkyHawk_Unified")
BENCH_DIR = os.path.join(FW_DIR, "BlinkyHawk_Bench")
sys.path.insert(0, HERE)
if os.path.exists(os.path.join(UNIFIED_DIR, "blinkyhawk_gui.py")):
    sys.path.insert(1, UNIFIED_DIR)
    import blinkyhawk_gui as bench          # "bench" = the base GUI module
else:
    sys.path.insert(1, BENCH_DIR)
    try:
        import blinkyhawk_bench_gui as bench
    except ImportError as exc:
        raise SystemExit(f"Could not import a BlinkyHawk GUI from\n  {UNIFIED_DIR}\n"
                         f"or\n  {BENCH_DIR}\n({exc})")

import bh_sequencer as S
import bh_tuner as T
from bh_instruments import (JIG_LOADS, InstrumentError, RelayJig, RigolDG800, SiglentSDS,
                            RigolDL3000, SimLoad,
                            SimFGen, SimRelayJig, SimScope, fmt_ohms)
from bh_led import COLOURS, STATE_NAMES, LedDecoder
from bh_sim import SimBench, SimBlinkyHawk

# AS7343 gain codes (!LEDCFG) -> label
LED_GAINS = ["0.5x", "1x", "2x", "4x", "8x", "16x", "32x", "64x", "128x", "256x", "512x",
             "1024x", "2048x"]

INF = float("inf")
SIM = "SIM"
DEFAULT_FGEN_IP = "192.168.10.3"      # static IP set on the DG852 Pro (Utility > LAN)
DEFAULT_SCOPE_IP = "192.168.10.2"
DEFAULT_LOAD_IP = "192.168.10.4"      # static IP to set on the DL3000 (Utility > Interface > LAN)

CLOSED_CHOICES = [("SHORT", 0.0), ("10K", 10e3), ("150K", 150e3)]
OPEN_CHOICES = [("1M", 1e6), ("10.6M", 10.6e6), ("OPEN", INF)]

# pythonw has no console, so an exception in a Tk callback would otherwise
# vanish without a trace.  Every one is appended here AND shown in the log.
ERROR_LOG = os.path.join(HERE, "suite_errors.log")

# Seconds to wait for the BlinkyHawk's first $STATUS after the port opens.
REPLY_TIMEOUT_S = 3.0

ASLEEP_HELP = (
    "The port opened but the BlinkyHawk is not answering.\n\n"
    "Most likely it is ASLEEP.  With CHGINHIBIT=0, USB power no longer keeps it "
    "awake -- only a program holding the port open does.  So 8 s after a reset "
    "(or an upload) with the leads reading OPEN, it drops into Software Standby, "
    "and in Standby the USB port stops answering.  On this rig, open leads are the "
    "normal state: the relay jig rests on FGEN and the generator output is off.\n\n"
    "To wake it: close the leads (relay jig SHORT, or a jumper) or press reset, "
    "then connect.  Once the port is open it will not sleep again.")


class SafeSerialManager(bench.SerialManager):
    """The bench GUI's SerialManager, made unable to freeze the GUI.

    The base opens the port and writes to it on the Tk thread with no write
    timeout.  A BlinkyHawk in Software Standby stops servicing USB, so those
    calls block and the whole window hangs.  Here the port is opened by a
    worker thread (handed in via `preopened`), and a write that times out is
    reported as a link error -- the base's _tick then disconnects cleanly --
    instead of blocking forever.
    """

    def __init__(self, line_queue):
        super().__init__(line_queue)
        self.preopened = None
        self.dead = False        # a write timed out: stop writing until reconnected

    def connect(self, port):
        self.dead = False
        ser, self.preopened = self.preopened, None
        if ser is None:
            return super().connect(port)
        self.disconnect()
        self.ser = ser
        self._stop.clear()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def send(self, text):
        # Each blocked write costs the full write_timeout on the calling thread,
        # and the base sends five queries on connect -- so after the first one
        # times out, drop the rest rather than freeze the window for 5 s.
        if self.dead:
            return
        try:
            super().send(text)
        except serial.SerialTimeoutException:
            self.dead = True
            self.line_queue.put(("__error__", "write timed out -- the BlinkyHawk is not "
                                              "accepting data (asleep? see the Connect help)"))
        except serial.SerialException as exc:
            self.line_queue.put(("__error__", str(exc)))


def open_port(port):
    """Open a serial port with timeouts on both directions."""
    return serial.Serial(port, bench.BAUD, timeout=0.1, write_timeout=1.0)


def port_choices():
    """'COM7  --  USB Serial Device' labels; the first word is the port."""
    return [f"{p.device}  --  {p.description}" for p in serial.tools.list_ports.comports()]


# chart colours (validated default categorical slots 1-2, neutral for don't-care)
C_CLOSED, C_OPEN, C_OTHER, C_INK = "#2a78d6", "#eb6834", "#8a8a86", "#0b0b0b"


class BatteryHost:
    """What bh_battery needs from the GUI, callable from the worker thread."""

    def __init__(self, app, port):
        self.app = app
        self.port = port

    def notice(self, text):
        self.app.wq.put(("notice", text))

    def reconnect(self):
        """Open the port again if it is closed (never toggles an open one shut)."""
        ev = threading.Event()

        def go():
            try:
                a = self.app
                if not a.serial.is_open and not a._connecting:
                    a.port_cb.set(self.port)
                    a._toggle_connect()
            finally:
                ev.set()
        self.app.wq.put(("call", go))
        ev.wait(5)


class SuiteApp(bench.App):
    def __init__(self):
        super().__init__()
        # Swap in a tee'd queue so worker threads see every line immediately.
        # Nothing is connected yet, so replacing the serial manager is safe.
        self.line_queue = S.TeeQueue()
        self.serial = SafeSerialManager(self.line_queue)
        self._connecting = False
        self._status_seen_t = 0.0
        self._last_port = None
        # Keep both port lists current: refresh when a dropdown is opened.
        self.port_cb.configure(postcommand=self._refresh_ports)
        self.title("Blinky Hawk TEST SUITE -- bench GUI + relay jig / generator / scope")
        self.report_callback_exception = self._tk_error
        w = min(1280, self.winfo_screenwidth() - 60)
        h = min(940, self.winfo_screenheight() - 80)
        self.geometry(f"{w}x{h}+20+10")
        self.after(100, self._pump_worker)

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        # state for the new tabs (built here because the base __init__ calls this)
        self.sim_bench = SimBench()
        self.relay = None
        self.fgen = None
        self.scope = None
        self.load = None
        self.worker = None
        self.stop_event = threading.Event()
        self.wq = queue.Queue()
        self.det_count = 0
        self.last_run_dir = None
        self.tune_state = None
        self.decoder = LedDecoder()
        self.led_q = None               # GUI's own flash subscription (live readout)
        self.led_recent = []            # flashes of the last few seconds
        super()._build_ui()
        self.rig_tab = ttk.Frame(self.nb)
        self.seq_tab = ttk.Frame(self.nb)
        self.tune_tab = ttk.Frame(self.nb)
        self.nb.add(self.rig_tab, text="  Test Rig  ")
        self.nb.add(self.seq_tab, text="  Test Sequence  ")
        self.nb.add(self.tune_tab, text="  Auto-Tune  ")
        self._shared_vars()
        self._build_rig_tab()
        self._build_seq_tab()
        self._build_tune_tab()

    def _shared_vars(self):
        # pass/fail criteria: the same variables drive both the sequence and the tuner
        self.vt_var = tk.StringVar(value="0.5")
        self.closed_var = tk.StringVar(value="150K")
        self.open_var = tk.StringVar(value="1M")

    def _criteria(self, parent):
        f = ttk.LabelFrame(parent, text="Pass/fail criteria (shared by both tabs)", padding=6)
        ttk.Label(f, text="VOLTAGE at |V| >=").pack(side="left")
        ttk.Entry(f, width=6, textvariable=self.vt_var).pack(side="left", padx=2)
        ttk.Label(f, text="V      CLOSED at <=").pack(side="left")
        ttk.Combobox(f, width=7, state="readonly", textvariable=self.closed_var,
                     values=[n for n, _ in CLOSED_CHOICES]).pack(side="left", padx=2)
        ttk.Label(f, text="     OPEN at >=").pack(side="left")
        ttk.Combobox(f, width=7, state="readonly", textvariable=self.open_var,
                     values=[n for n, _ in OPEN_CHOICES]).pack(side="left", padx=2)
        ttk.Label(f, foreground="#555",
                  text="   (loads between the two are don't-care; DC within 10 % of the "
                       "trip voltage is don't-care)").pack(side="left")
        return f

    def _criteria_values(self):
        vt = float(self.vt_var.get())
        cm = dict(CLOSED_CHOICES)[self.closed_var.get()]
        om = dict(OPEN_CHOICES)[self.open_var.get()]
        if not vt > 0:
            raise ValueError("trip voltage must be positive")
        return vt, cm, om

    # ---------------------------------------------------------- test rig tab
    def _build_rig_tab(self):
        t = self.rig_tab
        top = ttk.Frame(t, padding=6)
        top.pack(fill="x")
        ttk.Button(top, text="Simulate everything (dry run, no hardware)",
                   command=self._sim_all).pack(side="left")
        ttk.Label(top, foreground="#555",
                  text="   Type SIM in any port/IP box to simulate just that instrument. "
                       "The BlinkyHawk port list has a SIM entry too.").pack(side="left")

        # --- relay jig ---
        rf = ttk.LabelFrame(t, text="Relay jig (RelayMountTestJig, USB)", padding=6)
        rf.pack(fill="x", padx=6, pady=4)
        row = ttk.Frame(rf)
        row.pack(fill="x")
        ttk.Label(row, text="Port:").pack(side="left")
        self.jig_port = ttk.Combobox(row, width=40, postcommand=self._jig_refresh)
        self.jig_port.pack(side="left", padx=4)
        ttk.Button(row, text="Refresh", command=self._jig_refresh).pack(side="left", padx=(0, 4))
        self._jig_refresh()
        ttk.Button(row, text="Connect", command=self._jig_connect).pack(side="left")
        ttk.Button(row, text="Disconnect", command=self._jig_disconnect).pack(side="left", padx=4)
        self.jig_lbl = ttk.Label(row, text="not connected -- runs will ask you to move the "
                                           "leads by hand", foreground="#a60")
        self.jig_lbl.pack(side="left", padx=10)
        row2 = ttk.Frame(rf)
        row2.pack(fill="x", pady=(6, 0))
        for name in ["FGEN"] + [n for n, _ in JIG_LOADS]:
            ttk.Button(row2, text=name, width=8,
                       command=lambda n=name: self._bg(lambda: self.relay.set_mode(n),
                                                       f"relay -> {n}", need="relay")
                       ).pack(side="left", padx=2)
        row3 = ttk.Frame(rf)
        row3.pack(fill="x", pady=(6, 0))
        ttk.Label(row3, text="BlinkyHawk USB (K8):").pack(side="left")
        ttk.Button(row3, text="Connect USB", command=lambda: self._usb_manual(True)
                   ).pack(side="left", padx=4)
        ttk.Button(row3, text="Disconnect USB", command=lambda: self._usb_manual(False)
                   ).pack(side="left")
        self.usb_auto = tk.BooleanVar(value=True)
        ttk.Checkbutton(row3, text="switch it automatically in battery runs (no unplugging "
                                   "by hand)", variable=self.usb_auto).pack(side="left", padx=12)
        self.usb_lbl = ttk.Label(row3, text="", font=("Consolas", 10, "bold"))
        self.usb_lbl.pack(side="left", padx=8)

        # --- LED watcher (AS7343 on the relay jig) ---
        lf = ttk.LabelFrame(t, text="LED watcher (AS7343 over the BlinkyHawk's LED, on the "
                                    "relay jig's I2C)", padding=6)
        lf.pack(fill="x", padx=6, pady=4)
        row = ttk.Frame(lf)
        row.pack(fill="x")
        self.led_on_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(row, text="report flashes", variable=self.led_on_var,
                        command=lambda: self._bg(lambda: self.relay.led_enable(
                            self.led_on_var.get()), "LED watcher on/off", need="relay")
                        ).pack(side="left")
        ttk.Label(row, text="   gain").pack(side="left")
        self.led_gain = ttk.Combobox(row, width=6, state="readonly", values=LED_GAINS)
        self.led_gain.set("16x")
        self.led_gain.pack(side="left", padx=2)
        self.led_atime = tk.StringVar(value="15")
        self.led_astep = tk.StringVar(value="256")
        self.led_thr = tk.StringVar(value="20")
        for txt, var, w in (("ATIME", self.led_atime, 4), ("ASTEP", self.led_astep, 6),
                            ("threshold (counts)", self.led_thr, 5)):
            ttk.Label(row, text=f"  {txt}").pack(side="left")
            ttk.Entry(row, width=w, textvariable=var).pack(side="left", padx=2)
        ttk.Button(row, text="Apply", command=self._led_apply).pack(side="left", padx=6)
        self.led_raw_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(row, text="raw counts (for aiming)", variable=self.led_raw_var,
                        command=lambda: self._bg(lambda: self.relay.led_raw(
                            200 if self.led_raw_var.get() else 0), "LED raw stream",
                            need="relay")).pack(side="left", padx=8)
        row = ttk.Frame(lf)
        row.pack(fill="x", pady=(6, 0))
        ttk.Label(row, text="Calibrate colours:").pack(side="left")
        ttk.Label(row, text="red from DC").pack(side="left", padx=(6, 0))
        self.led_cal_v = tk.StringVar(value="1.0")
        ttk.Entry(row, width=5, textvariable=self.led_cal_v).pack(side="left", padx=2)
        ttk.Label(row, text="V").pack(side="left")
        ttk.Button(row, text="Auto-calibrate (OPEN -> blue, SHORT -> green, DC -> red)",
                   command=self._led_calibrate).pack(side="left", padx=6)
        self.led_cal_lbl = ttk.Label(row, text="", foreground="#555")
        self.led_cal_lbl.pack(side="left", padx=6)
        self.led_lbl = ttk.Label(lf, text="relay jig not connected", font=("Consolas", 10),
                                 foreground="#555")
        self.led_lbl.pack(anchor="w", pady=(6, 0))
        self.led_live = ttk.Label(lf, text="", font=("Consolas", 11, "bold"))
        self.led_live.pack(anchor="w")
        self._led_cal_text()

        # --- generator ---
        gf = ttk.LabelFrame(t, text="Function generator (Rigol DG852 Pro, LAN SCPI)", padding=6)
        gf.pack(fill="x", padx=6, pady=4)
        row = ttk.Frame(gf)
        row.pack(fill="x")
        ttk.Label(row, text="IP:").pack(side="left")
        self.fg_ip = tk.StringVar(value=DEFAULT_FGEN_IP)
        ttk.Entry(row, width=16, textvariable=self.fg_ip).pack(side="left", padx=2)
        ttk.Label(row, text="port:").pack(side="left")
        self.fg_port = ttk.Combobox(row, width=6, values=["5025", "5555", "5000"])
        self.fg_port.set("5025")
        self.fg_port.pack(side="left", padx=2)
        ttk.Label(row, text="CH:").pack(side="left")
        self.fg_ch = ttk.Combobox(row, width=3, values=["1", "2"], state="readonly")
        self.fg_ch.set("1")
        self.fg_ch.pack(side="left", padx=2)
        ttk.Label(row, text="safety limit |V| peak:").pack(side="left", padx=(10, 0))
        self.fg_limit = tk.StringVar(value="8.0")
        ttk.Entry(row, width=5, textvariable=self.fg_limit).pack(side="left", padx=2)
        ttk.Button(row, text="Connect", command=self._fg_connect).pack(side="left", padx=6)
        ttk.Button(row, text="Disconnect", command=self._fg_disconnect).pack(side="left")
        self.fg_lbl = ttk.Label(row, text="not connected", foreground="#a60")
        self.fg_lbl.pack(side="left", padx=10)
        row2 = ttk.Frame(gf)
        row2.pack(fill="x", pady=(6, 0))
        self.fg_dc = tk.StringVar(value="0.8")
        ttk.Label(row2, text="DC V:").pack(side="left")
        ttk.Entry(row2, width=7, textvariable=self.fg_dc).pack(side="left", padx=2)
        ttk.Button(row2, text="Apply DC", command=lambda: self._bg(
            lambda: self.fgen.apply_dc(float(self.fg_dc.get())), "generator DC",
            need="fgen")).pack(side="left", padx=4)
        self.fg_vrms = tk.StringVar(value="5")
        self.fg_hz = tk.StringVar(value="60")
        ttk.Label(row2, text="   sine Vrms:").pack(side="left")
        ttk.Entry(row2, width=6, textvariable=self.fg_vrms).pack(side="left", padx=2)
        ttk.Label(row2, text="Hz:").pack(side="left")
        ttk.Entry(row2, width=6, textvariable=self.fg_hz).pack(side="left", padx=2)
        ttk.Button(row2, text="Apply sine", command=lambda: self._bg(
            lambda: self.fgen.apply_sine(float(self.fg_vrms.get()), float(self.fg_hz.get())),
            "generator sine", need="fgen")).pack(side="left", padx=4)
        ttk.Button(row2, text="Output ON", command=lambda: self._bg(
            lambda: self.fgen.output(True), "generator on", need="fgen")).pack(side="left", padx=(16, 2))
        ttk.Button(row2, text="Output OFF", command=lambda: self._bg(
            lambda: self.fgen.output(False), "generator off", need="fgen")).pack(side="left")
        ttk.Label(gf, foreground="#555", wraplength=1180,
                  text="Connecting forces the channel to HighZ load and Vpp units, and checks "
                       "it took: the Rigol's default 50-ohm setting would put TWICE the "
                       "programmed voltage on the BlinkyHawk's high-impedance input. Sine "
                       "levels are entered in Vrms (5 Vrms = 14.14 Vpp). The safety limit "
                       "refuses anything whose peak exceeds it.").pack(anchor="w", pady=(6, 0))

        # --- scope ---
        sf = ttk.LabelFrame(t, text="Scope (Siglent SDS800X HD, LAN SCPI)", padding=6)
        sf.pack(fill="x", padx=6, pady=4)
        row = ttk.Frame(sf)
        row.pack(fill="x")
        ttk.Label(row, text="IP:").pack(side="left")
        self.sc_ip = tk.StringVar(value=DEFAULT_SCOPE_IP)
        ttk.Entry(row, width=16, textvariable=self.sc_ip).pack(side="left", padx=2)
        ttk.Label(row, text="channels (blank = those on):").pack(side="left", padx=(8, 0))
        self.sc_ch = tk.StringVar(value="")
        ttk.Entry(row, width=12, textvariable=self.sc_ch).pack(side="left", padx=2)
        ttk.Label(row, text="max points/channel:").pack(side="left", padx=(8, 0))
        self.sc_max = tk.StringVar(value="100000")
        ttk.Entry(row, width=8, textvariable=self.sc_max).pack(side="left", padx=2)
        ttk.Button(row, text="Connect", command=self._sc_connect).pack(side="left", padx=6)
        ttk.Button(row, text="Disconnect", command=self._sc_disconnect).pack(side="left")
        ttk.Button(row, text="Grab now", command=self._sc_grab).pack(side="left", padx=6)
        self.sc_lbl = ttk.Label(row, text="not connected", foreground="#a60")
        self.sc_lbl.pack(side="left", padx=10)

        # --- electronic load ---
        ef = ttk.LabelFrame(t, text="Electronic load (Rigol DL3000, LAN SCPI) -- in parallel "
                                    "with the BlinkyHawk battery, for draining between levels",
                            padding=6)
        ef.pack(fill="x", padx=6, pady=4)
        row = ttk.Frame(ef)
        row.pack(fill="x")
        ttk.Label(row, text="IP:").pack(side="left")
        self.ld_ip = tk.StringVar(value=DEFAULT_LOAD_IP)
        ttk.Entry(row, width=16, textvariable=self.ld_ip).pack(side="left", padx=2)
        ttk.Label(row, text="port:").pack(side="left")
        self.ld_port = ttk.Combobox(row, width=6, values=["5555", "5025"])
        self.ld_port.set("5555")
        self.ld_port.pack(side="left", padx=2)
        ttk.Label(row, text="current cap (A):").pack(side="left", padx=(10, 0))
        self.ld_cap = tk.StringVar(value="1.0")
        ttk.Entry(row, width=5, textvariable=self.ld_cap).pack(side="left", padx=2)
        ttk.Button(row, text="Connect", command=self._ld_connect).pack(side="left", padx=6)
        ttk.Button(row, text="Disconnect", command=self._ld_disconnect).pack(side="left")
        self.ld_lbl = ttk.Label(row, text="not connected", foreground="#a60")
        self.ld_lbl.pack(side="left", padx=10)
        row = ttk.Frame(ef)
        row.pack(fill="x", pady=(6, 0))
        ttk.Label(row, text="CC (mA):").pack(side="left")
        self.ld_ma = tk.StringVar(value="250")
        ttk.Entry(row, width=6, textvariable=self.ld_ma).pack(side="left", padx=2)
        ttk.Label(row, text="Von floor (V):").pack(side="left", padx=(8, 0))
        self.ld_floor = tk.StringVar(value="3.0")
        ttk.Entry(row, width=5, textvariable=self.ld_floor).pack(side="left", padx=2)
        ttk.Button(row, text="Input ON", command=self._ld_on).pack(side="left", padx=(10, 2))
        ttk.Button(row, text="Input OFF", command=lambda: self._bg(
            lambda: self.load.input(False), "load input off", need="load")).pack(side="left")
        ttk.Button(row, text="Read V / I", command=self._ld_read).pack(side="left", padx=8)
        self.ld_meas = ttk.Label(row, text="", font=("Consolas", 11, "bold"))
        self.ld_meas.pack(side="left", padx=8)
        ttk.Label(ef, foreground="#555", wraplength=1180,
                  text="Set 'Von Latch' to OFF on the load's front panel once: the floor is sent "
                       "as Von, so the load itself stops sinking below it even if this PC or "
                       "script dies mid-drain -- but only with the latch off, and SCPI cannot "
                       "set the latch.").pack(anchor="w", pady=(6, 0))

        self.rig_log = tk.Text(t, height=10, wrap="none", font=("Consolas", 9))
        self.rig_log.pack(fill="both", expand=True, padx=6, pady=6)

    def _ports(self):
        return super()._ports() + [SIM]

    def _rig_say(self, text):
        self.rig_log.insert("end", text + "\n")
        self.rig_log.see("end")

    def _bg(self, fn, what, need=None, done=None):
        """Run an instrument call off the Tk thread; report the outcome."""
        if need and getattr(self, need) is None:
            self._rig_say(f"** {what}: {need} not connected")
            return

        def run():
            try:
                res = fn()
                self.wq.put(("rig", f"{what}: ok" + (f" -- {res}" if isinstance(res, str) else "")))
                if done:
                    self.wq.put(("call", lambda: done(res)))
            except Exception as exc:
                self.wq.put(("rig", f"** {what}: {exc}"))
            self.wq.put(("call", self._refresh_rig_labels))
        threading.Thread(target=run, daemon=True).start()

    def _refresh_rig_labels(self):
        if self.relay is not None:
            self.jig_lbl.config(text=f"{getattr(self.relay, 'ident', '')}   mode "
                                     f"{self.relay.mode}", foreground="#0a0")
        else:
            self.jig_lbl.config(text="not connected -- runs will ask you to move the "
                                     "leads by hand", foreground="#a60")
        if self.relay is None or not getattr(self.relay, "has_usb", False):
            self.usb_lbl.config(text="no USB switch (jig firmware 1.2 + K8 relays)",
                                foreground="#888")
        else:
            on = self.relay.usb_state
            self.usb_lbl.config(text={True: "USB CONNECTED", False: "USB DISCONNECTED",
                                      None: "USB ?"}[on],
                                foreground={True: "#0a0", False: "#c00", None: "#888"}[on])
        if self.fgen is not None:
            self.fg_lbl.config(text=f"{self.fgen.ident}   output: {self.fgen.state}",
                               foreground="#0a0")
        else:
            self.fg_lbl.config(text="not connected", foreground="#a60")
        self.sc_lbl.config(text=self.scope.ident if self.scope else "not connected",
                           foreground="#0a0" if self.scope else "#a60")
        self.ld_lbl.config(text=(f"{self.load.ident}   input: {self.load.state}"
                                 if self.load else "not connected"),
                           foreground="#0a0" if self.load else "#a60")
        info = getattr(self.relay, "led_info", {}) if self.relay is not None else {}
        if self.relay is None:
            self.led_lbl.config(text="relay jig not connected", foreground="#555")
        elif info.get("sensor") != "1":
            self.led_lbl.config(text="no AS7343 on the relay jig (needs jig firmware 1.1 and "
                                     "the sensor on I2C)", foreground="#a60")
        else:
            g = int(info.get("gain", 5))
            self.led_lbl.config(foreground="#0a0", text=(
                f"sensor OK   reporting {'ON' if info.get('on') == '1' else 'off'}   gain "
                f"{LED_GAINS[g] if 0 <= g < len(LED_GAINS) else g}   "
                f"{info.get('hz', '?')} samples/s   dark {info.get('base', '?')} of "
                f"{info.get('fullscale', '?')} counts   threshold {info.get('thr', '?')}"))

    def _jig_refresh(self):
        self.jig_port["values"] = port_choices() + [SIM]

    def _jig_connect(self):
        port = (self.jig_port.get().strip().split() or [""])[0]
        if not port:
            self._rig_say("** relay jig: pick a port first")
            return
        if self.serial.is_open and getattr(self.serial.ser, "port", None) == port:
            self._rig_say(f"** relay jig: {port} is the BlinkyHawk's port")
            return
        self._jig_disconnect()
        if port.upper() == SIM:
            self.relay = SimRelayJig(self.sim_bench)
            self._led_subscribe()
            self._refresh_rig_labels()
            return
        jig = RelayJig()

        def done(_):
            self.relay = jig
            self._led_subscribe()
        self._bg(lambda: jig.connect(port), f"relay jig on {port}", done=done)

    def _jig_disconnect(self):
        if self.relay is not None:
            if self.led_q is not None:
                self.relay.unsubscribe_flashes(self.led_q)
                self.led_q = None
            self.relay.close()
        self.relay = None
        self._refresh_rig_labels()

    def _led_apply(self):
        try:
            gain = LED_GAINS.index(self.led_gain.get())
            atime, astep = int(self.led_atime.get()), int(self.led_astep.get())
            thr = float(self.led_thr.get())
        except ValueError:
            self._rig_say("** LED: gain/ATIME/ASTEP/threshold must be numbers")
            return
        self._bg(lambda: self.relay.led_config(gain, atime, astep, thr), "LED sensor config",
                 need="relay")

    def _led_cal_text(self):
        cal = self.decoder.calibrated
        done = ", ".join(f"{k} {cal[k]}" for k in COLOURS if k in cal)
        self.led_cal_lbl.config(
            text=(f"calibrated: {done}   separation {self.decoder.separation():.3f}"
                  if done else "NOT calibrated -- using rough default colours"),
            foreground="#0a0" if len(cal) == 3 else "#a60")

    def _led_calibrate(self):
        if self.relay is None or not getattr(self.relay, "has_led", False):
            messagebox.showerror("LED calibration", "Connect the relay jig with its AS7343 first.")
            return
        try:
            v = float(self.led_cal_v.get())
        except ValueError:
            return
        if not messagebox.askokcancel(
                "LED calibration",
                "The jig will go OPEN (blue float alert), SHORT (green), then the generator "
                f"at {v:+g} V DC (red VDC+ alert), recording each for 5 s.\n\nThe unit must be "
                "alerting normally -- LED on, not locked out by charging -- and tuned so "
                f"{v:+g} V reads as voltage.  With serial connected, a condition the unit is "
                "not actually alerting is skipped rather than learned."):
            return
        self._start_job(lambda r: r.run_led_calibration(v, 5.0, 1.0),
                        need_serial=False, use_led=True)

    def _led_subscribe(self):
        if self.relay is not None and getattr(self.relay, "has_led", False):
            self.led_q = self.relay.subscribe_flashes()

    def _led_pump(self):
        """Live readout: last flash + what the last 3 s decode to."""
        if self.led_q is None:
            return
        got = False
        while True:
            try:
                f = self.led_q.get_nowait()
            except queue.Empty:
                break
            self.led_recent.append(self.decoder.classify(f))
            got = True
        now = time.time()
        # Age by when the flash ENDED: a long one (room light over the threshold,
        # or a solid-on LED the jig reports in 5 s pieces) STARTED more than 3 s
        # ago the moment it arrives.  Ageing by start emptied this list under
        # the [-1] below, and that exception stopped the whole GUI updating.
        self.led_recent = [f for f in self.led_recent
                           if now - (f.t + f.dur_ms / 1000.0) < 3.0]
        if got and self.led_recent:
            f = self.led_recent[-1]
            d = self.decoder.decode(self.led_recent)
            self.led_live.config(text=(
                f"last flash: {f.colour} (cos {f.cos:.2f}, {f.dur_ms} ms, peak {f.peak:.0f}"
                f"{', SATURATED' if f.sat else ''})     last 3 s: "
                f"{STATE_NAMES.get(d['state'], d['state'])}  [{d['colours']}]"),
                foreground="#c00" if f.sat else "#06a")
        raw = getattr(self.relay, "raw_last", None)
        if raw and self.led_raw_var.get():
            self.led_lbl.config(text="raw  FZ {1}  FY {2}  FXL {3}  NIR {4}  VIS {5}".format(*raw)
                                if len(raw) >= 6 else str(raw))

    def _fg_connect(self):
        ip = self.fg_ip.get().strip()
        self._fg_disconnect()
        try:
            limit = float(self.fg_limit.get())
        except ValueError:
            self._rig_say("** safety limit must be a number")
            return
        if ip.upper() == SIM:
            self.fgen = SimFGen(self.sim_bench, limit)
            self._refresh_rig_labels()
            return
        g = RigolDG800(int(self.fg_ch.get()), limit)

        def done(_):
            self.fgen = g
        self._bg(lambda: g.connect(ip, int(self.fg_port.get())), f"generator at {ip}",
                 done=done)

    def _fg_disconnect(self):
        if self.fgen is not None:
            self.fgen.safe_off()
            self.fgen.close()
        self.fgen = None
        self._refresh_rig_labels()

    def _sc_connect(self):
        ip = self.sc_ip.get().strip()
        self._sc_disconnect()
        if ip.upper() == SIM:
            self.scope = SimScope(self.sim_bench)
            self._refresh_rig_labels()
            return
        chans = [c for c in self.sc_ch.get().replace(",", " ").split() if c] or None
        s = SiglentSDS(chans, int(self.sc_max.get() or 0))

        def done(_):
            self.scope = s
        self._bg(lambda: s.connect(ip), f"scope at {ip}", done=done)

    def _usb_usable(self):
        return self.relay is not None and getattr(self.relay, "has_usb", False)

    def _usb_auto_on(self):
        return self.usb_auto.get() and self._usb_usable()

    def _usb_manual(self, on):
        if not self._usb_usable():
            self._rig_say("** the relay jig cannot switch USB (needs jig firmware 1.2 and the "
                          "K8 USB relays)")
            return
        if not on and self.serial.is_open:
            # Let go of the port first, rather than have it vanish under the reader.
            self._toggle_connect()
        port = self._last_port

        def done(_):
            if on and port and not self.serial.is_open:
                # Windows needs a moment to enumerate the BlinkyHawk again.
                self.after(2500, lambda: self._reconnect_to(port))
        self._bg(lambda: self.relay.usb(on), f"USB {'connect' if on else 'disconnect'}",
                 need="relay", done=done)

    def _reconnect_to(self, port):
        if self.serial.is_open or self._connecting:
            return
        if port not in [p.device for p in serial.tools.list_ports.comports()]:
            self._log(f"** {port} is not back yet -- press Connect when it is")
            return
        self.port_cb.set(port)
        self._toggle_connect()

    def _ld_connect(self):
        ip = self.ld_ip.get().strip()
        self._ld_disconnect()
        try:
            cap = float(self.ld_cap.get())
        except ValueError:
            self._rig_say("** load current cap must be a number")
            return
        if ip.upper() == SIM:
            self.load = SimLoad(self.sim_bench, cap)
            self._refresh_rig_labels()
            return
        ld = RigolDL3000(cap)

        def done(_):
            self.load = ld
        self._bg(lambda: ld.connect(ip, int(self.ld_port.get())), f"load at {ip}", done=done)

    def _ld_disconnect(self):
        if self.load is not None:
            self.load.safe_off()
            self.load.close()
        self.load = None
        self._refresh_rig_labels()

    def _ld_on(self):
        try:
            amps = float(self.ld_ma.get()) / 1000.0
            floor = float(self.ld_floor.get())
        except ValueError:
            self._rig_say("** load: mA and floor must be numbers")
            return

        def go():
            self.load.set_cc(amps, floor)
            self.load.input(True)
        self._bg(go, f"load CC {amps * 1000:.0f} mA ON (Von {floor:g} V)", need="load")

    def _ld_read(self):
        def rd():
            v, a = self.load.volts(), self.load.amps()
            self.wq.put(("battery", (v, a, self.load.state)))
            return f"{v:.4f} V  {a * 1000:.1f} mA"
        self._bg(rd, "load reading", need="load")

    def _sc_disconnect(self):
        if self.scope is not None:
            self.scope.restore()
            self.scope.close()
        self.scope = None
        self._refresh_rig_labels()

    def _sc_grab(self):
        def grab():
            tdiv, trig, waves = self.scope.capture("grab")
            return "; ".join(f"{w.channel}: mean {w.stats()['mean']:+.4g} V, "
                             f"AC rms {w.stats()['acrms']:.4g} V, pk-pk {w.stats()['pkpk']:.4g} V"
                             for w in waves)
        self._bg(grab, "scope grab", need="scope")

    def _sim_all(self):
        if self.serial.is_open:
            self._toggle_connect()
        self.port_cb.set(SIM)
        self._toggle_connect()
        self.jig_port.set(SIM)
        self._jig_connect()
        self.fg_ip.set(SIM)
        self._fg_connect()
        self.sc_ip.set(SIM)
        self._sc_connect()
        self.ld_ip.set(SIM)
        self._ld_connect()
        self._rig_say("everything simulated -- numbers from the simulator mean nothing about "
                      "hardware; this is for checking the flow")

    # BlinkyHawk connection.  SIM swaps in the simulated board; a real port is
    # opened by a worker thread so a sleeping board cannot freeze the window.
    def _toggle_connect(self):
        if self._connecting:
            return
        if self.serial.is_open:
            super()._toggle_connect()          # disconnect path, unchanged
            return
        port = self.port_cb.get().strip()
        if not port:
            self._log("** select a port first")
            return
        if port.upper() == SIM:
            self.serial = SimBlinkyHawk(self.line_queue, self.sim_bench)
            super()._toggle_connect()
            return
        jig_ser = getattr(self.relay, "ser", None)
        if jig_ser is not None and jig_ser.port == port:
            messagebox.showerror("Connect", f"{port} is the relay jig's port.")
            return
        self.serial = SafeSerialManager(self.line_queue)
        self._connecting = True
        self.conn_lbl.config(text=f"opening {port}...", foreground="#a60")

        def work():
            try:
                ser, err = open_port(port), None
            except Exception as exc:
                ser, err = None, exc
            self.wq.put(("call", lambda: self._finish_connect(port, ser, err)))
        threading.Thread(target=work, daemon=True).start()

    def _finish_connect(self, port, ser, err):
        self._connecting = False
        if err is not None:
            self.conn_lbl.config(text="disconnected", foreground="#a00")
            self._log(f"** connect failed: {err}")
            text = str(err).lower()
            busy = "denied" in text or "busy" in text
            messagebox.showerror(
                "Connect", f"Could not open {port}:\n{err}\n\n" + (
                    "Another program has the port open -- usually the Arduino IDE's "
                    "Serial Monitor (close its tab), an upload in progress, or another "
                    "copy of this GUI." if busy else
                    "Check the port still exists (press Refresh) -- the board may have "
                    "re-enumerated on a new COM number after the upload."))
            return
        self.serial.preopened = ser
        self._last_port = port
        t0 = time.time()
        super()._toggle_connect()            # base: attach, reset state, send the queries
        self.after(int(REPLY_TIMEOUT_S * 1000), lambda: self._check_reply(port, t0))

    def _check_reply(self, port, t0):
        if self._status_seen_t >= t0:
            return
        self._log(f"** no reply from {port} in {REPLY_TIMEOUT_S:g} s")
        if self.relay is not None:
            if messagebox.askyesno(
                    "BlinkyHawk not answering",
                    ASLEEP_HELP + "\n\nThe relay jig is connected.  Switch it to SHORT "
                                  "now to wake the board, then reconnect?"):
                self._wake_via_jig()
        else:
            messagebox.showwarning("BlinkyHawk not answering", ASLEEP_HELP)

    def _wake_via_jig(self):
        if self.serial.is_open:
            super()._toggle_connect()        # let go of the port first
        self._log("** relay jig -> SHORT to wake the BlinkyHawk; reconnecting in 2 s")

        def done(_):
            self.after(2000, self._toggle_connect)
        self._bg(lambda: self.relay.set_mode("SHORT"), "relay -> SHORT (wake)", need="relay",
                 done=done)

    def _handle_line(self, line):
        if line.startswith("$STATUS"):
            self._status_seen_t = time.time()
        if line.startswith("$DET,"):
            # 20 a second -- counted, not logged
            self.det_count += 1
            if self.det_count % 20 == 0:
                self.det_lbl.config(text=f"$DET passes: {self.det_count}")
            return
        super()._handle_line(line)

    # ------------------------------------------------------ sequence tab
    def _build_seq_tab(self):
        t = self.seq_tab
        self._criteria(t).pack(fill="x", padx=6, pady=4)

        cf = ttk.LabelFrame(t, text="Conditions (run in this order)", padding=6)
        cf.pack(fill="x", padx=6, pady=4)
        r = ttk.Frame(cf)
        r.pack(fill="x")
        ttk.Label(r, text="Relay loads:").pack(side="left")
        self.seq_loads = {}
        for name, ohms in JIG_LOADS:
            v = tk.BooleanVar(value=True)
            self.seq_loads[name] = v
            ttk.Checkbutton(r, text=f"{name}", variable=v).pack(side="left", padx=4)
        r = ttk.Frame(cf)
        r.pack(fill="x", pady=4)
        ttk.Label(r, text="Generator DC levels (V):").pack(side="left")
        self.seq_dc = tk.StringVar(value="0.8, -0.8")
        ttk.Entry(r, width=36, textvariable=self.seq_dc).pack(side="left", padx=4)
        ttk.Label(r, foreground="#555", text="list, or start:stop:step  e.g. -1:1:0.25"
                  ).pack(side="left")
        r = ttk.Frame(cf)
        r.pack(fill="x")
        self.seq_ac_on = tk.BooleanVar(value=True)
        ttk.Checkbutton(r, text="Generator sine", variable=self.seq_ac_on).pack(side="left")
        self.seq_ac = tk.StringVar(value="5")
        self.seq_hz = tk.StringVar(value="60")
        ttk.Entry(r, width=5, textvariable=self.seq_ac).pack(side="left", padx=2)
        ttk.Label(r, text="Vrms at").pack(side="left")
        ttk.Entry(r, width=5, textvariable=self.seq_hz).pack(side="left", padx=2)
        ttk.Label(r, text="Hz").pack(side="left")
        self.seq_off_on = tk.BooleanVar(value=True)
        ttk.Checkbutton(r, text="Generator path with output OFF (expect OPEN)",
                        variable=self.seq_off_on).pack(side="left", padx=20)
        r = ttk.Frame(cf)
        r.pack(fill="x", pady=(4, 0))
        ttk.Label(r, text="Judge by:").pack(side="left")
        self.seq_battery = tk.BooleanVar(value=False)
        ttk.Checkbutton(r, text="unit on battery -- NO serial link, LED watcher only",
                        variable=self.seq_battery).pack(side="left", padx=6)
        self.quiet_var = tk.BooleanVar(value=True)     # shared with the Auto-Tune tab
        ttk.Checkbutton(r, text="silence beeps during the run", variable=self.quiet_var
                        ).pack(side="left", padx=6)
        self.seq_use_led = tk.BooleanVar(value=True)
        ttk.Checkbutton(r, text="use the LED watcher when available",
                        variable=self.seq_use_led).pack(side="left", padx=6)
        ttk.Label(r, foreground="#555",
                  text="(LED: give >= 2 s dwell -- VAC is a 3-flash cycle; FLOAT may read dark)"
                  ).pack(side="left")

        bf = ttk.LabelFrame(t, text="Battery levels (electronic load drains the battery "
                                    "between rounds; USB stays out)", padding=6)
        bf.pack(fill="x", padx=6, pady=4)
        r = ttk.Frame(bf)
        r.pack(fill="x")
        self.bl_on = tk.BooleanVar(value=False)
        ttk.Checkbutton(r, text="run the plan at each level:", variable=self.bl_on
                        ).pack(side="left")
        self.bl_levels = tk.StringVar(value="as found, 3.9, 3.6, 3.3")
        ttk.Entry(r, width=24, textvariable=self.bl_levels).pack(side="left", padx=4)
        ttk.Label(r, text="V   judge by:").pack(side="left")
        self.bl_judge = tk.StringVar(value="led")
        ttk.Radiobutton(r, text="LED only (automatic)", value="led",
                        variable=self.bl_judge).pack(side="left", padx=2)
        ttk.Radiobutton(r, text="unit's battery log (replug USB each level)", value="log",
                        variable=self.bl_judge).pack(side="left", padx=2)
        r = ttk.Frame(bf)
        r.pack(fill="x", pady=(4, 0))
        self.bl_ma = tk.StringVar(value="250")
        self.bl_tol = tk.StringVar(value="20")
        self.bl_rest = tk.StringVar(value="60")
        self.bl_floor = tk.StringVar(value="3.0")
        self.bl_max = tk.StringVar(value="60")
        self.bl_samples = tk.StringVar(value="8")
        for txt, var, w in (("drain mA", self.bl_ma, 5), ("band +/- mV", self.bl_tol, 4),
                            ("rest up to s", self.bl_rest, 4), ("floor V", self.bl_floor, 4),
                            ("max drain min", self.bl_max, 4),
                            ("log samples/condition", self.bl_samples, 3)):
            ttk.Label(r, text=txt).pack(side="left", padx=(8, 2))
            ttk.Entry(r, width=w, textvariable=var).pack(side="left")
        self.bl_batt = ttk.Label(r, text="", font=("Consolas", 10, "bold"), foreground="#06a")
        self.bl_batt.pack(side="left", padx=12)

        of = ttk.LabelFrame(t, text="Timing, scope, run", padding=6)
        of.pack(fill="x", padx=6, pady=4)
        r = ttk.Frame(of)
        r.pack(fill="x")
        self.seq_dwell = tk.StringVar(value="3")
        self.seq_settle = tk.StringVar(value="0.5")
        ttk.Label(r, text="dwell s:").pack(side="left")
        ttk.Entry(r, width=5, textvariable=self.seq_dwell).pack(side="left", padx=2)
        ttk.Label(r, text="settle s:").pack(side="left", padx=(8, 0))
        ttk.Entry(r, width=5, textvariable=self.seq_settle).pack(side="left", padx=2)
        ttk.Label(r, text="   scope:").pack(side="left")
        self.seq_scope = tk.StringVar(value="grab")
        for txt, val in (("off", ""), ("grab screen", "grab"), ("single trigger", "single")):
            ttk.Radiobutton(r, text=txt, value=val, variable=self.seq_scope).pack(side="left")
        ttk.Label(r, text="timeout s:").pack(side="left", padx=(6, 0))
        self.seq_sto = tk.StringVar(value="5")
        ttk.Entry(r, width=4, textvariable=self.seq_sto).pack(side="left", padx=2)
        ttk.Label(r, text="   note:").pack(side="left")
        self.seq_note = tk.StringVar(value="")
        ttk.Entry(r, width=30, textvariable=self.seq_note).pack(side="left", padx=2)
        r = ttk.Frame(of)
        r.pack(fill="x", pady=(6, 0))
        ttk.Button(r, text="Run test", command=self._seq_run).pack(side="left")
        ttk.Button(r, text="Stop", command=self._stop_job).pack(side="left", padx=4)
        ttk.Button(r, text="Open run folder", command=self._open_run_dir).pack(side="left", padx=4)
        self.seq_prog = ttk.Progressbar(r, length=220, mode="determinate")
        self.seq_prog.pack(side="left", padx=10)
        self.seq_status = ttk.Label(r, text="idle", font=("Consolas", 10))
        self.seq_status.pack(side="left", padx=6)
        self.det_lbl = ttk.Label(r, text="$DET passes: 0", foreground="#555")
        self.det_lbl.pack(side="right")

        cols = ("cond", "exp", "res", "n", "C", "F", "V", "kind", "led", "ledc", "rest",
                "metric", "scope")
        heads = ("condition", "expect", "result", "passes", "CLOSED %", "OPEN %", "VOLT %",
                 "kind", "LED says", "LED flashes", "rest diff V", "metric", "scope")
        widths = (170, 55, 60, 50, 65, 60, 60, 55, 70, 120, 85, 70, 300)
        # operator instruction (unplug / replug USB) for battery-level runs
        self.seq_notice = tk.Label(t, text="", font=("Segoe UI", 14, "bold"),
                                   fg="#ffffff", bg="#c0392b", wraplength=1150, pady=6)
        tf = ttk.Frame(t)
        tf.pack(fill="both", expand=True, padx=6, pady=4)
        self.seq_tf = tf
        self.seq_tree = ttk.Treeview(tf, columns=cols, show="headings", height=12)
        for c, h, w in zip(cols, heads, widths):
            self.seq_tree.heading(c, text=h)
            self.seq_tree.column(c, width=w, anchor="w", stretch=(c == "scope"))
        self.seq_tree.tag_configure("FAIL", foreground="#c00")
        self.seq_tree.tag_configure("PASS", foreground="#070")
        self.seq_tree.tag_configure("LEVEL", background="#e8eef7")
        sb = ttk.Scrollbar(tf, command=self.seq_tree.yview)
        self.seq_tree.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.seq_tree.pack(side="left", fill="both", expand=True)

    def _seq_conditions(self):
        vt, cm, om = self._criteria_values()
        conds = S.load_conditions([n for n, v in self.seq_loads.items() if v.get()], cm, om)
        conds += [S.dc_condition(v, vt) for v in S.parse_levels(self.seq_dc.get())]
        if self.seq_ac_on.get():
            conds.append(S.ac_condition(float(self.seq_ac.get()), float(self.seq_hz.get())))
        if self.seq_off_on.get():
            conds.append(S.Cond("gen output off", "fgen_off", "FGEN", expected="F"))
        return conds

    def _seq_run(self):
        try:
            conds = self._seq_conditions()
            dwell, settle = float(self.seq_dwell.get()), float(self.seq_settle.get())
            sto = float(self.seq_sto.get())
        except (ValueError, KeyError) as exc:
            messagebox.showerror("Test", f"Check the settings: {exc}")
            return
        if not conds:
            return
        battery = self.seq_battery.get()
        use_led = self.seq_use_led.get()
        has_led = self.relay is not None and getattr(self.relay, "has_led", False)
        if battery and not (use_led and has_led):
            messagebox.showerror("Test", "Battery mode judges by the LED watcher alone, and "
                                         "the relay jig has no AS7343 connected (or it is "
                                         "switched off here).")
            return
        if battery and dwell < 2.0:
            if not messagebox.askyesno("Test", f"A {dwell:g} s dwell may not show a whole "
                                               "voltage pattern (VAC is three flashes, one "
                                               "per LEDVOLTPER), so some conditions could "
                                               "come out NO DATA or '?'.  Run anyway?"):
                return
        if self.fgen is None and any(c.kind in ("dc", "ac") for c in conds):
            if not messagebox.askyesno(
                    "Test", "The function generator is not connected, so the DC/AC "
                            "conditions will be skipped.  Run the rest?"):
                return
        if self.bl_on.get():
            self._levels_run(conds, dwell, settle)
            return
        self.seq_tree.delete(*self.seq_tree.get_children())
        self.seq_prog.config(maximum=len(conds), value=0)
        scope_mode = self.seq_scope.get() or None
        note = self.seq_note.get()
        self._start_job(lambda r: r.run_plan(conds, dwell, settle, scope_mode, sto, note),
                        need_serial=not battery, use_led=use_led,
                        serial_if_open=not battery)

    def _levels_run(self, conds, dwell, settle):
        """The whole plan at each battery level (bh_sequencer.run_battery_levels)."""
        try:
            levels = []
            for tok in self.bl_levels.get().replace(";", ",").split(","):
                tok = tok.strip().lower()
                if not tok:
                    continue
                levels.append(None if tok in ("as found", "asfound", "start", "full", "now")
                              else float(tok))
            p = {"levels": levels, "conds": conds, "dwell": dwell, "settle": settle,
                 "amps": float(self.bl_ma.get()) / 1000.0, "tol": float(self.bl_tol.get()) / 1000.0,
                 "rest_s": float(self.bl_rest.get()), "floor": float(self.bl_floor.get()),
                 "max_drain_s": float(self.bl_max.get()) * 60.0,
                 "samples": int(self.bl_samples.get()), "judge": self.bl_judge.get(),
                 "note": self.seq_note.get()}
        except ValueError as exc:
            messagebox.showerror("Battery levels", f"Check the settings: {exc}")
            return
        nums = [l for l in levels if l is not None]
        if not levels:
            return
        if nums != sorted(nums, reverse=True) or (None in levels and levels[0] is not None):
            messagebox.showerror("Battery levels", "Levels must go DOWN (the load can only "
                                                   "drain), with 'as found' first if used.")
            return
        if nums and min(nums) - p["tol"] <= p["floor"]:
            messagebox.showerror("Battery levels", f"{min(nums):g} V is too close to the "
                                                   f"{p['floor']:g} V floor.")
            return
        if self.load is None:
            messagebox.showerror("Battery levels", "Connect the electronic load first "
                                                   "(Test Rig tab).")
            return
        log_mode = p["judge"] == "log"
        port = getattr(getattr(self.serial, "ser", None), "port", None)
        if log_mode and not port:
            messagebox.showerror("Battery levels", "The battery-log mode arms each capture "
                                                   "over USB: connect the BlinkyHawk first.")
            return
        if not log_mode and not (self.seq_use_led.get() and self.relay is not None
                                 and getattr(self.relay, "has_led", False)):
            messagebox.showerror("Battery levels", "LED-only judging needs the relay jig's "
                                                   "LED watcher.")
            return
        if not messagebox.askokcancel(
                "Battery levels",
                "Before starting:\n\n"
                "  - the load is wired ACROSS THE BATTERY (+ to +, - to -)\n"
                "  - 'Von Latch' is OFF on the load's front panel\n"
                "  - every scope probe is OFF the BlinkyHawk\n"
                + ("  - the relay jig will DISCONNECT the USB"
                   + (" and reconnect it at each level to read the unit's log"
                      if log_mode else " for the whole run")
                   + ", and put it back at the end"
                   if self._usb_auto_on() else
                   "  - you will be told to UNPLUG the USB"
                   + (" and, at each level, to plug it back in to read the unit's log"
                      if log_mode else " -- it stays out for the whole run")) + "\n\n"
                f"Drain {p['amps'] * 1000:.0f} mA to within +/-{p['tol'] * 1000:.0f} mV of each "
                f"level (rested), never below {p['floor']:g} V."):
            return
        p["host"] = BatteryHost(self, port) if port else None
        self.seq_tree.delete(*self.seq_tree.get_children())
        self.seq_prog.config(maximum=len(conds), value=0)
        self._start_job(lambda r: r.run_battery_levels(p), need_serial=log_mode,
                        use_led=self.seq_use_led.get(), serial_if_open=log_mode)

    # ----------------------------------------------------------- tune tab
    def _build_tune_tab(self):
        t = self.tune_tab
        self._criteria(t).pack(fill="x", padx=6, pady=4)
        pf = ttk.LabelFrame(t, text="Auto-tune settings", padding=6)
        pf.pack(fill="x", padx=6, pady=4)

        r = ttk.Frame(pf)
        r.pack(fill="x")
        self.tu_do_v = tk.BooleanVar(value=True)
        self.tu_do_t = tk.BooleanVar(value=True)
        self.tu_do_x = tk.BooleanVar(value=True)
        ttk.Checkbutton(r, text="1. voltage (REFCENTER/REFBAND/VOLTFAST)",
                        variable=self.tu_do_v).pack(side="left")
        ttk.Checkbutton(r, text="2. open/closed (method + threshold)",
                        variable=self.tu_do_t).pack(side="left", padx=10)
        ttk.Checkbutton(r, text="3. verify", variable=self.tu_do_x).pack(side="left")
        ttk.Checkbutton(r, text="silence beeps during the run (restored after)",
                        variable=self.quiet_var).pack(side="left", padx=16)

        r = ttk.Frame(pf)
        r.pack(fill="x", pady=4)
        ttk.Label(r, text="DC sweep (V):").pack(side="left")
        self.tu_dc = tk.StringVar(value="-1.2:1.2:0.1")
        ttk.Entry(r, width=22, textvariable=self.tu_dc).pack(side="left", padx=2)
        self.tu_ac_on = tk.BooleanVar(value=True)
        ttk.Checkbutton(r, text="sine", variable=self.tu_ac_on).pack(side="left", padx=(12, 0))
        self.tu_ac = tk.StringVar(value="5")
        self.tu_hz = tk.StringVar(value="60")
        ttk.Entry(r, width=5, textvariable=self.tu_ac).pack(side="left", padx=2)
        ttk.Label(r, text="Vrms @").pack(side="left")
        ttk.Entry(r, width=5, textvariable=self.tu_hz).pack(side="left", padx=2)
        ttk.Label(r, text="Hz    verify also at DC:").pack(side="left")
        self.tu_vdc = tk.StringVar(value="0.8, -0.8")
        ttk.Entry(r, width=12, textvariable=self.tu_vdc).pack(side="left", padx=2)
        ttk.Label(r, text="   guard x:").pack(side="left")
        self.tu_guard = tk.StringVar(value="1.25")
        ttk.Entry(r, width=5, textvariable=self.tu_guard).pack(side="left", padx=2)

        r = ttk.Frame(pf)
        r.pack(fill="x")
        ttk.Label(r, text="DETBAND candidates:").pack(side="left")
        self.tu_bands = tk.StringVar(value="0.03, 0.05, 0.1")
        ttk.Entry(r, width=16, textvariable=self.tu_bands).pack(side="left", padx=2)
        self.tu_m0 = tk.BooleanVar(value=True)
        ttk.Checkbutton(r, text="also try method 0 (single |diff|)",
                        variable=self.tu_m0).pack(side="left", padx=8)
        ttk.Label(r, text="passes/condition:").pack(side="left", padx=(8, 0))
        self.tu_n = tk.StringVar(value="60")
        ttk.Entry(r, width=5, textvariable=self.tu_n).pack(side="left", padx=2)
        ttk.Label(r, text="dwell s:").pack(side="left")
        self.tu_dwell = tk.StringVar(value="3")
        ttk.Entry(r, width=4, textvariable=self.tu_dwell).pack(side="left", padx=2)
        ttk.Label(r, text="settle s:").pack(side="left")
        self.tu_settle = tk.StringVar(value="0.5")
        ttk.Entry(r, width=4, textvariable=self.tu_settle).pack(side="left", padx=2)

        r = ttk.Frame(pf)
        r.pack(fill="x", pady=(4, 0))
        self.tu_bat = tk.BooleanVar(value=True)
        ttk.Checkbutton(r, text="measure ON BATTERY (floating) -- recommended",
                        variable=self.tu_bat).pack(side="left")
        ttk.Label(r, text="   log entries/condition:").pack(side="left")
        self.tu_bn = tk.StringVar(value="6")
        ttk.Entry(r, width=4, textvariable=self.tu_bn).pack(side="left", padx=2)
        ttk.Label(r, text="dwell s:").pack(side="left")
        self.tu_bdwell = tk.StringVar(value="2.2")
        ttk.Entry(r, width=4, textvariable=self.tu_bdwell).pack(side="left", padx=2)
        ttk.Label(r, text="settle s:").pack(side="left")
        self.tu_bsettle = tk.StringVar(value="0.5")
        ttk.Entry(r, width=4, textvariable=self.tu_bsettle).pack(side="left", padx=2)
        self.tu_wake = tk.BooleanVar(value=True)
        ttk.Checkbutton(r, text="sleep/wake check in verify",
                        variable=self.tu_wake).pack(side="left", padx=10)
        ttk.Label(pf, foreground="#a33", wraplength=1150, text=(
            "Battery mode: take every scope probe OFF the unit (a ground clip earths it) and "
            "leave the relay jig + generator connected.  You will be asked to unplug and "
            "replug USB several times -- each capture then runs by itself.  The passes/"
            "condition and dwell above are for USB mode.")).pack(fill="x", pady=(2, 0))

        r = ttk.Frame(pf)
        r.pack(fill="x", pady=(6, 0))
        ttk.Button(r, text="Run auto-tune", command=self._tune_run).pack(side="left")
        ttk.Button(r, text="Stop (reverts)", command=self._stop_job).pack(side="left", padx=4)
        self.tu_save = ttk.Button(r, text="Save to EEPROM", state="disabled",
                                  command=self._tune_save)
        self.tu_save.pack(side="left", padx=(16, 2))
        self.tu_revert = ttk.Button(r, text="Revert to before", state="disabled",
                                    command=self._tune_revert)
        self.tu_revert.pack(side="left", padx=2)
        ttk.Button(r, text="Open run folder", command=self._open_run_dir).pack(side="left", padx=8)
        self.tu_status = ttk.Label(r, text="idle", font=("Consolas", 10))
        self.tu_status.pack(side="left", padx=10)
        # Operator instruction for battery captures (unplug / replug).  Big and
        # coloured, because the run waits on it and nothing else says so.
        self.tu_notice = tk.Label(t, text="", font=("Segoe UI", 14, "bold"),
                                  fg="#ffffff", bg="#c0392b", wraplength=1150, pady=6)

        body = ttk.Panedwindow(t, orient="horizontal")
        body.pack(fill="both", expand=True, padx=6, pady=4)
        self.tu_body = body
        lf = ttk.Frame(body)
        self.tu_text = tk.Text(lf, wrap="none", font=("Consolas", 9))
        sb = ttk.Scrollbar(lf, command=self.tu_text.yview)
        self.tu_text.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.tu_text.pack(side="left", fill="both", expand=True)
        body.add(lf, weight=3)
        rf = ttk.Frame(body)
        body.add(rf, weight=2)
        self.tu_fig = bench.Figure(figsize=(5, 6), dpi=90)
        self.tu_ax_v = self.tu_fig.add_subplot(211)
        self.tu_ax_m = self.tu_fig.add_subplot(212)
        self.tu_fig.tight_layout()
        self.tu_canvas = bench.FigureCanvasTkAgg(self.tu_fig, master=rf)
        self.tu_canvas.get_tk_widget().pack(fill="both", expand=True)
        self._tune_say("Auto-tune.  Order matters and is fixed: the open/closed metric "
                       "is measured against REFCENTER,\nso voltage is tuned first, then the "
                       "metric, then everything is verified for real.\nNothing is saved -- "
                       "the result sits in device RAM until you press Save or Revert.\n")

    def _tune_say(self, text):
        self.tu_text.insert("end", text + "\n")
        self.tu_text.see("end")

    def _tune_params(self):
        vt, cm, om = self._criteria_values()
        bands = [float(x) for x in self.tu_bands.get().replace(",", " ").split()]
        if not bands:
            raise ValueError("need at least one DETBAND")
        return {"note": f"trip{vt:g}V", "vt": vt, "closed_max": cm, "open_min": om,
                "dc_levels": S.parse_levels(self.tu_dc.get()),
                "verify_dc": S.parse_levels(self.tu_vdc.get()) if self.tu_vdc.get().strip() else [],
                "ac": [(float(self.tu_ac.get()), float(self.tu_hz.get()))]
                if self.tu_ac_on.get() else [],
                "passes": int(self.tu_n.get()), "dwell": float(self.tu_dwell.get()),
                "settle": float(self.tu_settle.get()), "guard": float(self.tu_guard.get()),
                "detbands": bands, "include_m0": self.tu_m0.get(),
                "loads": [n for n, _ in JIG_LOADS],
                "do_voltage": self.tu_do_v.get(), "do_threshold": self.tu_do_t.get(),
                "do_verify": self.tu_do_x.get(),
                "battery": self.tu_bat.get(), "wake_check": self.tu_wake.get(),
                "bat_samples": int(self.tu_bn.get()), "bat_dwell": float(self.tu_bdwell.get()),
                "bat_settle": float(self.tu_bsettle.get())}

    def _tune_run(self):
        try:
            p = self._tune_params()
        except (ValueError, KeyError) as exc:
            messagebox.showerror("Auto-tune", f"Check the settings: {exc}")
            return
        if p["do_voltage"] and self.fgen is None:
            if not messagebox.askyesno(
                    "Auto-tune", "Voltage tuning needs the function generator, which is not "
                                 "connected.  Continue with the other steps only?"):
                return
        self.tu_text.delete("1.0", "end")
        self.tu_ax_v.clear()
        self.tu_ax_m.clear()
        self.tu_canvas.draw_idle()
        self.tu_save.config(state="disabled")
        self.tu_revert.config(state="disabled")
        if p["battery"]:
            if self.relay is None:
                messagebox.showerror("Auto-tune", "Battery mode drives the leads by itself "
                                                  "while USB is unplugged, so it needs the relay "
                                                  "jig connected.")
                return
            port = getattr(getattr(self.serial, "ser", None), "port", None)
            if not port:
                messagebox.showerror("Auto-tune", "Connect the BlinkyHawk over USB first "
                                                  "(each capture is armed over USB).")
                return
            if not messagebox.askokcancel(
                    "Auto-tune on battery",
                    "Before starting:\n\n"
                    "  - every scope probe OFF the BlinkyHawk (a ground clip earths it)\n"
                    "  - battery fitted and charged\n"
                    "  - relay jig on the leads, generator connected\n\n"
                    + ("The relay jig switches the USB for each capture and puts it "
                       "back at the end -- fully automatic."
                       if self._usb_auto_on() else
                       "You will be told when to UNPLUG and when to PLUG BACK IN the USB "
                       "cable -- once per capture.  Everything else is automatic.")):
                return
            p["host"] = BatteryHost(self, port)
        self._start_job(lambda r: r.run_autotune(p))

    def _tune_save(self):
        self._cfg_save()             # base GUI: !SAVE + logs the unit row by SN
        self.tu_save.config(state="disabled")
        self._tune_say("** saved to EEPROM (and logged to the unit CSV if the unit has an SN)")

    def _tune_revert(self):
        st = self.tune_state
        if not st:
            return
        for k in st["applied"]:
            if k in st["original"]:
                self._send(f"!SET,{k},{st['original'][k]}")
        self._send("!CFG")
        self._send("!STATUS")
        self.tu_revert.config(state="disabled")
        self.tu_save.config(state="disabled")
        self._tune_say("** reverted: " + ", ".join(
            f"{k}={st['original'][k]}" for k in st["applied"] if k in st["original"]))

    def _plot_voltage(self, rv):
        ax = self.tu_ax_v
        ax.clear()
        tf = rv.get("transfer")
        if not tf or not tf.ok:
            return
        xs = [p[0] for p in tf.pts]
        ys = [p[1] for p in tf.pts]
        ax.plot(xs, ys, "-o", color=C_CLOSED, lw=2, ms=4, label="rest differential (median)")
        if "centre" in rv:
            c, b = rv["centre"], rv["band"]
            ax.axhspan(c - b, c + b, color=C_CLOSED, alpha=0.10, lw=0, label="no-voltage band")
            ax.axhline(c, color=C_INK, lw=0.8, ls="--")
            vt = float(self.vt_var.get())
            for v in (vt, -vt):
                ax.axvline(v, color=C_INK, lw=0.8, ls=":")
        ax.set_title("Front end: input V -> resting differential", fontsize=9)
        ax.set_xlabel("generator DC (V)", fontsize=8)
        ax.set_ylabel("differential (V)", fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=7, loc="upper left")
        self.tu_fig.tight_layout()
        self.tu_canvas.draw_idle()

    def _plot_metric(self, rt):
        ax = self.tu_ax_m
        ax.clear()
        best = rt.get("best")
        if not best:
            return
        sep = best["sep"]
        vt, cm, om = self._criteria_values()
        data = sorted(best["data"], key=lambda d: INF if d[1] is None else d[1])
        seen = set()
        for i, (lab, ohms, vals) in enumerate(data):
            r = INF if ohms is None else ohms
            grp, col = (("closed", C_CLOSED) if r <= cm else
                        ("open", C_OPEN) if r >= om else ("don't care", C_OTHER))
            vals = [v for v in vals if v is not None]
            jit = [i + (k % 9 - 4) * 0.03 for k in range(len(vals))]
            ax.scatter(jit, vals, s=10, color=col, alpha=0.7, linewidths=0,
                       label=None if grp in seen else f"must read {grp}"
                       if grp != "don't care" else grp)
            seen.add(grp)
        if sep.get("threshold") is not None:
            ax.axhline(sep["threshold"], color=C_INK, lw=1, ls="--", label="threshold")
        ax.set_xticks(range(len(data)))
        ax.set_xticklabels([d[0] for d in data], fontsize=7)
        u = T.METHOD_UNITS[best["method"]]
        ax.set_title(f"Metric per load: {best['label']} ({u})"
                     + ("" if sep["clean"] else "  -- NOT SEPARABLE"), fontsize=9)
        ax.tick_params(labelsize=7)
        ax.grid(True, axis="y", alpha=0.25)
        ax.legend(fontsize=7, loc="upper left")
        self.tu_fig.tight_layout()
        self.tu_canvas.draw_idle()

    # ------------------------------------------------------------ jobs
    def _start_job(self, job, need_serial=True, use_led=True, serial_if_open=True):
        """need_serial     the job cannot run without the BlinkyHawk's serial link
        serial_if_open  use the link when it happens to be open.  Battery mode
                        passes False: no $DET at all, the LED is the only judge."""
        if self.worker and self.worker.is_alive():
            messagebox.showinfo("Busy", "A run is already in progress.")
            return
        if need_serial and not self.serial.is_open:
            messagebox.showerror("Not connected", "Connect the BlinkyHawk first.")
            return
        self.stop_event.clear()
        dev = (S.DeviceClient(lambda: self.serial, self.line_queue, self.stop_event)
               if (need_serial or serial_if_open) and self.serial.is_open else None)
        relay = self.relay if self.relay is not None else S.ManualRelay(self._worker_prompt)
        runner = S.Runner(dev, relay, self.fgen, self.scope,
                          lambda k, p: self.wq.put((k, p)), self.stop_event,
                          use_led=use_led, decoder=self.decoder)
        runner.load = self.load
        runner.prompt = self._worker_prompt
        runner.usb_auto = self._usb_auto_on()
        runner.quiet_beeps = self.quiet_var.get()
        port = self._last_port

        def run():
            try:
                self.wq.put(("rundir", job(runner)))
                self.wq.put(("finished", "done"))
            except S.Aborted:
                self.wq.put(("finished", "stopped"))
            except Exception as exc:
                self.wq.put(("log", f"!! run failed: {exc}"))
                self.wq.put(("finished", f"failed: {exc}"))
            finally:
                # Whatever happened, put the USB back how the job found it --
                # a stopped or failed battery run must not leave the unit
                # disconnected -- and reopen the serial port if it came back.
                if runner.usb_restore() and port:
                    self.wq.put(("call", lambda: self.after(2500,
                                                            lambda: self._reconnect_to(port))))
                self.wq.put(("call", self._refresh_rig_labels))
            if runner.dir:
                self.wq.put(("rundir", runner.dir))
        self.worker = threading.Thread(target=run, daemon=True)
        self.worker.start()
        self.seq_status.config(text="running")
        self.tu_status.config(text="running")

    def _stop_job(self):
        self.stop_event.set()

    def _worker_prompt(self, text):
        ev = threading.Event()
        box = {}

        def ask():
            box["ok"] = messagebox.askokcancel("Move the leads", text)
            ev.set()
        self.wq.put(("call", ask))
        while not ev.wait(0.2):
            if self.stop_event.is_set():
                return False
        return box.get("ok", False)

    def _open_run_dir(self):
        d = self.last_run_dir or S.RUNS_DIR
        if os.path.isdir(d):
            os.startfile(d)

    def _pump_worker(self):
        """Drain the worker queue into the UI.

        Everything the background threads show -- result rows, status, the
        instrument connect replies -- comes through here, so this must survive
        any one bad message: each is handled in its own try, and the loop is
        rescheduled in a finally.  (An exception here once stopped all output
        for the rest of the session, which looked like the test and the scope
        connect doing nothing.)"""
        try:
            for _ in range(500):          # bounded, so a flood cannot starve Tk
                try:
                    kind, p = self.wq.get_nowait()
                except queue.Empty:
                    break
                try:
                    self._handle_wq(kind, p)
                except Exception:
                    self._report_error(f"handling '{kind}' from the worker")
            try:
                self._led_pump()
            except Exception:
                self._report_error("updating the LED readout")
        finally:
            self.after(100, self._pump_worker)

    def _handle_wq(self, kind, p):
        if kind == "log":
            self._log(p)
            self._tune_say(p)
        elif kind == "rig":
            self._rig_say(p)
        elif kind == "call":
            p()
        elif kind == "notice":
            if p:
                self.tu_notice.config(text=p)
                self.tu_notice.pack(fill="x", padx=6, pady=4, before=self.tu_body)
                self.seq_notice.config(text=p)
                self.seq_notice.pack(fill="x", padx=6, pady=4, before=self.seq_tf)
                self.bell()
                self._log("** " + p)
            else:
                self.tu_notice.pack_forget()
                self.seq_notice.pack_forget()
        elif kind == "battery":
            v, a, st = p
            txt = f"battery {v:.3f} V  {a * 1000:.0f} mA  (load {st})"
            self.bl_batt.config(text=txt)
            self.ld_meas.config(text=txt)
        elif kind == "level":
            self.seq_tree.insert("", "end", tags=("LEVEL",), values=(
                f"== level {p['level']}", "", f"{p['pass']}P {p['fail']}F", "",
                "", "", "", "", "", f"rest {p['rest_before_V']:.3f} V",
                f"{p['drained_mAh']:.1f} mAh", "", f"after tests {p['after_tests_V']:.3f} V"))
        elif kind in ("status", "stage"):
            self.seq_status.config(text=p)
            self.tu_status.config(text=p)
        elif kind == "progress":
            self.seq_prog.config(value=p[0], maximum=p[1])
        elif kind == "summary":
            self._add_summary(p)
        elif kind == "tune_voltage":
            self._plot_voltage(p)
        elif kind == "tune_threshold":
            self._plot_metric(p)
        elif kind == "report":
            self.tu_text.delete("1.0", "end")
            self._tune_say(p)
        elif kind == "tune_done":
            self.tune_state = p
            if p["applied"]:
                self.tu_save.config(state="normal")
                self.tu_revert.config(state="normal")
            self._send("!CFG")
        elif kind == "ledcal":
            self.decoder.load()
            self._led_cal_text()
            messagebox.showinfo("LED calibration", "\n".join(p))
        elif kind == "rundir":
            if p:
                self.last_run_dir = p
        elif kind == "finished":
            self.seq_status.config(text=p)
            self.tu_status.config(text=p)
            self._send("!STATUS")

    def _report_error(self, where):
        """Show a caught exception in the logs and append it to ERROR_LOG."""
        tb = traceback.format_exc()
        try:
            with open(ERROR_LOG, "a", encoding="utf-8") as fh:
                fh.write(f"--- {datetime.now():%Y-%m-%d %H:%M:%S}  {where}\n{tb}\n")
        except OSError:
            pass
        last = tb.strip().splitlines()[-1]
        msg = f"!! GUI error while {where}: {last}   (details: {ERROR_LOG})"
        for fn in (self._log, self._rig_say, self._tune_say):
            try:
                fn(msg)
            except Exception:
                pass
        try:
            self.seq_status.config(text="GUI error -- see log")
        except Exception:
            pass

    def _tk_error(self, exc, val, tb):
        """Tk callback exceptions (button handlers etc.) -- same treatment."""
        try:
            raise val
        except Exception:
            self._report_error("running a button/timer callback")

    def _add_summary(self, s):
        pct = lambda x: f"{x * 100:.0f}"
        num = lambda x, f: "" if x is None else format(x, f)
        has_serial = s["n"] > 0
        self.seq_tree.insert("", "end", tags=(s["result"],), values=(
            f"{s['phase']}: {s['label']}", s["expected"], s["result"],
            s["n"] if has_serial else "-",
            pct(s["lead_C"]) if has_serial else "", pct(s["lead_F"]) if has_serial else "",
            pct(s["lead_V"]) if has_serial else "",
            s["kind"] if has_serial and s["lead_V"] else "",
            STATE_NAMES.get(s.get("led_state", ""), s.get("led_state", "")),
            s.get("led_colours", ""),
            num(s["rest_mean"], "+.4f"), num(s["metric_mean"], ".4f"), s.get("scope", "")))
        kids = self.seq_tree.get_children()
        if kids:
            self.seq_tree.see(kids[-1])

    def _on_close(self):
        self.stop_event.set()
        for fn in (self._ld_disconnect, self._fg_disconnect, self._sc_disconnect,
                   self._jig_disconnect):
            try:
                fn()
            except Exception:
                pass
        super()._on_close()


if __name__ == "__main__":
    SuiteApp().mainloop()
