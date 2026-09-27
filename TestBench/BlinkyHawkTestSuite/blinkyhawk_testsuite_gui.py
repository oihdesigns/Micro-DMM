#!/usr/bin/env python3
"""
blinkyhawk_testsuite_gui.py  --  the BlinkyHawk bench GUI plus automated testing.

This IS blinkyhawk_bench_gui.py (imported from ArduinoProgrammingFiles/
BlinkyHawk_Bench and subclassed, not copied -- so its Diagnostics /
Configuration / Bench tabs, and its per-unit CSV log, are the same ones), with
three more tabs:

  Test Rig        connect and drive the relay jig, the Rigol DG852 Pro and the
                  Siglent scope by hand.  Type SIM in any port/IP box (or press
                  "Simulate everything") to dry-run with nothing plugged in.
  Test Sequence   the big test: every selected load / DC level / AC level in
                  turn, the BlinkyHawk's own readings + decisions logged per
                  detection pass, a scope capture per condition, a pass/fail
                  table, and a run folder under Runs/.
  Auto-Tune       finds REFCENTER/REFBAND for a target trip voltage and the
                  detection method + threshold that reads CLOSED at the chosen
                  resistance and OPEN above it -- or says it cannot, and shows
                  which loads CAN be told apart.  Applies to RAM only; Save or
                  Revert afterwards.

Needs the bench firmware with !DETLOG (BlinkyHawk_Bench.ino from the same
commit as this file).

Dependencies:  pip install pyserial matplotlib
Run:           py blinkyhawk_testsuite_gui.py
"""

import os
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import serial
import serial.tools.list_ports

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH_DIR = os.path.normpath(os.path.join(HERE, "..", "..", "ArduinoProgrammingFiles",
                                          "BlinkyHawk_Bench"))
sys.path.insert(0, HERE)
sys.path.insert(1, BENCH_DIR)
try:
    import blinkyhawk_bench_gui as bench
except ImportError as exc:
    raise SystemExit(f"Could not import blinkyhawk_bench_gui from\n  {BENCH_DIR}\n({exc})")

import bh_sequencer as S
import bh_tuner as T
from bh_instruments import (JIG_LOADS, InstrumentError, RelayJig, RigolDG800, SiglentSDS,
                            SimFGen, SimRelayJig, SimScope, fmt_ohms)
from bh_sim import SimBench, SimBlinkyHawk

INF = float("inf")
SIM = "SIM"
DEFAULT_FGEN_IP = "192.168.10.3"      # static IP set on the DG852 Pro (Utility > LAN)
DEFAULT_SCOPE_IP = "192.168.10.2"

CLOSED_CHOICES = [("SHORT", 0.0), ("10K", 10e3), ("150K", 150e3)]
OPEN_CHOICES = [("1M", 1e6), ("10.6M", 10.6e6), ("OPEN", INF)]

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


class SuiteApp(bench.App):
    def __init__(self):
        super().__init__()
        # Swap in a tee'd queue so worker threads see every line immediately.
        # Nothing is connected yet, so replacing the serial manager is safe.
        self.line_queue = S.TeeQueue()
        self.serial = SafeSerialManager(self.line_queue)
        self._connecting = False
        self._status_seen_t = 0.0
        # Keep both port lists current: refresh when a dropdown is opened.
        self.port_cb.configure(postcommand=self._refresh_ports)
        self.title("Blinky Hawk TEST SUITE -- bench GUI + relay jig / generator / scope")
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
        self.worker = None
        self.stop_event = threading.Event()
        self.wq = queue.Queue()
        self.det_count = 0
        self.last_run_dir = None
        self.tune_state = None
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
        if self.fgen is not None:
            self.fg_lbl.config(text=f"{self.fgen.ident}   output: {self.fgen.state}",
                               foreground="#0a0")
        else:
            self.fg_lbl.config(text="not connected", foreground="#a60")
        self.sc_lbl.config(text=self.scope.ident if self.scope else "not connected",
                           foreground="#0a0" if self.scope else "#a60")

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
            self._refresh_rig_labels()
            return
        jig = RelayJig()

        def done(_):
            self.relay = jig
        self._bg(lambda: jig.connect(port), f"relay jig on {port}", done=done)

    def _jig_disconnect(self):
        if self.relay is not None:
            self.relay.close()
        self.relay = None
        self._refresh_rig_labels()

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

        cols = ("cond", "exp", "res", "n", "C", "F", "V", "rest", "metric", "scope")
        heads = ("condition", "expect", "result", "passes", "CLOSED %", "OPEN %", "VOLT %",
                 "rest diff V", "metric", "scope")
        widths = (170, 55, 60, 55, 70, 70, 70, 95, 80, 420)
        tf = ttk.Frame(t)
        tf.pack(fill="both", expand=True, padx=6, pady=4)
        self.seq_tree = ttk.Treeview(tf, columns=cols, show="headings", height=12)
        for c, h, w in zip(cols, heads, widths):
            self.seq_tree.heading(c, text=h)
            self.seq_tree.column(c, width=w, anchor="w", stretch=(c == "scope"))
        self.seq_tree.tag_configure("FAIL", foreground="#c00")
        self.seq_tree.tag_configure("PASS", foreground="#070")
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
        if self.fgen is None and any(c.kind in ("dc", "ac") for c in conds):
            if not messagebox.askyesno(
                    "Test", "The function generator is not connected, so the DC/AC "
                            "conditions will be skipped.  Run the rest?"):
                return
        self.seq_tree.delete(*self.seq_tree.get_children())
        self.seq_prog.config(maximum=len(conds), value=0)
        scope_mode = self.seq_scope.get() or None
        note = self.seq_note.get()
        self._start_job(lambda r: r.run_plan(conds, dwell, settle, scope_mode, sto, note))

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

        body = ttk.Panedwindow(t, orient="horizontal")
        body.pack(fill="both", expand=True, padx=6, pady=4)
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
                "do_verify": self.tu_do_x.get()}

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
    def _start_job(self, job):
        if self.worker and self.worker.is_alive():
            messagebox.showinfo("Busy", "A run is already in progress.")
            return
        if not self.serial.is_open:
            messagebox.showerror("Not connected", "Connect the BlinkyHawk first.")
            return
        self.stop_event.clear()
        dev = S.DeviceClient(lambda: self.serial, self.line_queue, self.stop_event)
        relay = self.relay if self.relay is not None else S.ManualRelay(self._worker_prompt)
        runner = S.Runner(dev, relay, self.fgen, self.scope,
                          lambda k, p: self.wq.put((k, p)), self.stop_event)

        def run():
            try:
                self.wq.put(("rundir", job(runner)))
                self.wq.put(("finished", "done"))
            except S.Aborted:
                self.wq.put(("finished", "stopped"))
            except Exception as exc:
                self.wq.put(("log", f"!! run failed: {exc}"))
                self.wq.put(("finished", f"failed: {exc}"))
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
        try:
            while True:
                kind, p = self.wq.get_nowait()
                if kind == "log":
                    self._log(p)
                    self._tune_say(p)
                elif kind == "rig":
                    self._rig_say(p)
                elif kind == "call":
                    p()
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
                elif kind == "rundir":
                    if p:
                        self.last_run_dir = p
                elif kind == "finished":
                    self.seq_status.config(text=p)
                    self.tu_status.config(text=p)
                    self._send("!STATUS")
        except queue.Empty:
            pass
        self.after(100, self._pump_worker)

    def _add_summary(self, s):
        pct = lambda x: f"{x * 100:.0f}"
        num = lambda x, f: "" if x is None else format(x, f)
        self.seq_tree.insert("", "end", tags=(s["result"],), values=(
            f"{s['phase']}: {s['label']}", s["expected"], s["result"], s["n"],
            pct(s["lead_C"]), pct(s["lead_F"]), pct(s["lead_V"]),
            num(s["rest_mean"], "+.4f"), num(s["metric_mean"], ".4f"), s.get("scope", "")))
        kids = self.seq_tree.get_children()
        if kids:
            self.seq_tree.see(kids[-1])

    def _on_close(self):
        self.stop_event.set()
        for fn in (self._fg_disconnect, self._sc_disconnect, self._jig_disconnect):
            try:
                fn()
            except Exception:
                pass
        super()._on_close()


if __name__ == "__main__":
    SuiteApp().mainloop()
