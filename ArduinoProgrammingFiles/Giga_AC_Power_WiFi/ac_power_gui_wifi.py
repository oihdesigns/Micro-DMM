"""
Giga AC Power Monitor GUI, over Wi-Fi or USB.

Type the board's IP (e.g. 192.168.40.225, optionally :8080) or pick a COM port
in the Board box. The line protocol is the same either way.

What the board sends, and where it shows up here:
  $PWR   per window (default 12 cycles, 5/s)   -> tiles + Trends tab
  $CYC   per mains cycle (60/s)                -> Cycles tab (inrush, steps)
  $HARM  harmonics 1..N                        -> Shape tab, bottom
  $AVG   one cycle-averaged V and I cycle      -> Shape tab, top
  $WAVE  raw capture (manual or on an event)   -> Capture tab
  $INFO  link and acquisition health           -> Diagnostics tab
The board does the reductions, so a slow link only loses the bulk streams
($CYC $HARM $AVG); $PWR and replies always get through.

Requires: pip install pyserial matplotlib numpy
"""

import csv
import json
import os
import queue
import re
import socket
import threading
import time
from collections import deque

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import numpy as np
import serial
import serial.tools.list_ports
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

DEFAULT_ADDR = "192.168.40.225"
TCP_PORT = 8080
PROFILE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "ac_power_profile_wifi.json")

# $PWR / $CYC flag bits, in the sketch's order
FLAG_NAMES = [(0x01, "NO V CROSSING"), (0x02, "V CLIP"), (0x04, "I CLIP"),
              (0x08, "TRIPPED"), (0x10, "GAP"), (0x20, "ADC ERR"), (0x40, "SYNTH"),
              (0x80, "I RANGE SWITCH")]
INFO_FLAGS = 0x40 | 0x80          # shown, but not as a fault
SMP_LABELS = ["1.5", "2.5", "8.5", "16.5", "32.5", "64.5", "387.5", "810.5"]
BITS_LABELS = ["8", "10", "12", "14", "16"]

SM_PWR, SM_CYC, SM_HARM, SM_AVG, SM_INFO = 1, 2, 4, 8, 16

BG     = "#1a1a1a"
PANEL  = "#222222"
ACCENT = "#00ff32"   # voltage
AMBER  = "#ffaa00"   # current
CYAN   = "#00ccff"   # power
DIM    = "#888888"
WHITE  = "#e0e0e0"
RED    = "#ff4444"
SEP    = "#444444"


# ══ transport ════════════════════════════════════════════════════════════════

def is_network_addr(s: str) -> bool:
    return bool(re.match(r"^\d{1,3}(\.\d{1,3}){3}(:\d+)?$", s.strip()))


class Link:
    """A line link to the board, serial or TCP. A reader thread pushes complete
    lines into `rx`; None means the link closed."""

    def __init__(self, target: str):
        self.rx: queue.Queue = queue.Queue()
        self._alive = True
        self.target = target.strip()
        if is_network_addr(self.target):
            host, _, port = self.target.partition(":")
            self._sock = socket.create_connection((host, int(port or TCP_PORT)), timeout=5)
            self._sock.settimeout(0.5)
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._ser = None
            self.kind = "Wi-Fi"
        else:
            self._ser = serial.Serial(self.target, 115200, timeout=0.2)
            self._sock = None
            self.kind = "USB"
        self.bytes_in = 0
        self._th = threading.Thread(target=self._reader, daemon=True)
        self._th.start()

    def _reader(self):
        buf = b""
        while self._alive:
            try:
                if self._sock:
                    chunk = self._sock.recv(65536)
                    if not chunk:            # peer closed
                        break
                else:
                    chunk = self._ser.read(self._ser.in_waiting or 1)
            except socket.timeout:
                continue
            except Exception:
                break
            if not chunk:
                continue
            self.bytes_in += len(chunk)
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.rx.put(line.decode(errors="replace").strip())
        self._alive = False
        self.rx.put(None)

    def send(self, cmd: str):
        data = (cmd.strip() + "\n").encode()
        if self._sock:
            self._sock.sendall(data)
        else:
            self._ser.write(data)

    def close(self):
        self._alive = False
        try:
            if self._sock:
                self._sock.close()
            else:
                self._ser.close()
        except Exception:
            pass


# ══ application ══════════════════════════════════════════════════════════════

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Giga AC Power Monitor")
        self.configure(bg=BG)
        self.geometry("1360x860")

        self.link: Link | None = None
        self._relay = False
        self._smask = 0
        self._acfg: dict = {}
        self._adc: dict = {}

        H = 200_000
        self._tr = {k: deque(maxlen=H) for k in ("t", "vrms", "irms", "p", "pf", "s", "ithd")}
        self._cyc = {k: deque(maxlen=60 * 600) for k in ("t", "vrms", "irms", "p", "hz")}
        self._cyc_seq0 = None
        self._last_pwr_t = None
        self._seq_last = None
        self._pwr_missing = 0
        self._wave = None           # header dict while receiving
        self._wave_v: list = []
        self._wave_i: list = []
        self._dirty = set()
        self._last_draw = 0.0
        self._log_fh = None
        self._log_w = None
        self._log_rows = 0
        self._bytes_t = (time.monotonic(), 0)

        self._vcmd_float = self.register(lambda s: s in ("", "-", ".", "-.") or _isfloat(s))
        self._build_ui()
        self._load_profile()
        self._refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(40, self._poll)

    # ── widget helpers ───────────────────────────────────────────────────────

    def _lbl(self, parent, text, color=DIM):
        tk.Label(parent, text=text, bg=BG, fg=color, anchor="w").pack(fill="x", pady=(6, 1))

    def _note(self, parent, text):
        tk.Label(parent, text=text, bg=BG, fg=DIM, justify="left", anchor="w",
                 font=("Courier New", 7), wraplength=290).pack(fill="x")

    def _sep(self, parent):
        tk.Frame(parent, bg=SEP, height=1).pack(fill="x", pady=5)

    def _entry(self, parent, label, var, unit="", on_enter=None, width=9, label_w=11):
        row = tk.Frame(parent, bg=BG)
        row.pack(fill="x", pady=1)
        tk.Label(row, text=label, bg=BG, fg=WHITE, width=label_w, anchor="w").pack(side="left")
        e = tk.Entry(row, textvariable=var, width=width, bg=PANEL, fg=WHITE,
                     insertbackground=WHITE, relief="flat", validate="key",
                     validatecommand=(self._vcmd_float, "%P"))
        e.pack(side="left", padx=2)
        if on_enter:
            e.bind("<Return>", lambda _: on_enter())
        if unit:
            tk.Label(row, text=unit, bg=BG, fg=DIM).pack(side="left")
        return e

    def _combo(self, parent, label, var, values, on_change, width=8, label_w=11):
        row = tk.Frame(parent, bg=BG)
        row.pack(fill="x", pady=1)
        tk.Label(row, text=label, bg=BG, fg=WHITE, width=label_w, anchor="w").pack(side="left")
        cb = ttk.Combobox(row, textvariable=var, values=values, state="readonly", width=width)
        cb.pack(side="left")
        cb.bind("<<ComboboxSelected>>", lambda _: on_change())
        return cb

    def _check(self, parent, text, var, cmd):
        tk.Checkbutton(parent, text=text, variable=var, bg=BG, fg=WHITE, selectcolor=PANEL,
                       activebackground=BG, activeforeground=WHITE, anchor="w",
                       command=cmd).pack(fill="x")

    def _button(self, parent, text, cmd, fg=WHITE, bg=PANEL, height=1):
        b = tk.Button(parent, text=text, bg=bg, fg=fg, relief="flat", height=height, command=cmd)
        b.pack(fill="x", pady=(3, 1))
        return b

    # ── layout ───────────────────────────────────────────────────────────────

    def _build_ui(self):
        style = ttk.Style(self)
        style.theme_use("default")
        style.configure("TNotebook", background=BG, borderwidth=0)
        style.configure("TNotebook.Tab", background=PANEL, foreground=WHITE, padding=(10, 3))
        style.map("TNotebook.Tab", background=[("selected", "#333333")])

        self._build_topbar()
        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=8, pady=4)
        left = tk.Frame(body, bg=BG, width=310)
        left.pack(side="left", fill="y", padx=(0, 8))
        left.pack_propagate(False)
        right = tk.Frame(body, bg=BG)
        right.pack(side="left", fill="both", expand=True)
        self._build_controls(left)
        self._build_display(right)

    def _build_topbar(self):
        top = tk.Frame(self, bg=BG)
        top.pack(fill="x", padx=8, pady=6)
        tk.Label(top, text="Board:", bg=BG, fg=WHITE).pack(side="left")
        self._addr_var = tk.StringVar(value=DEFAULT_ADDR)
        self._addr_cb = ttk.Combobox(top, textvariable=self._addr_var, width=20)
        self._addr_cb.pack(side="left", padx=4)
        tk.Button(top, text="↺", bg=PANEL, fg=WHITE, relief="flat",
                  command=self._refresh_ports).pack(side="left")
        self._connect_btn = tk.Button(top, text="Connect", bg="#225522", fg=ACCENT,
                                      relief="flat", width=10, command=self._toggle_connect)
        self._connect_btn.pack(side="left", padx=8)
        self._status_lbl = tk.Label(top, text="Disconnected", bg=BG, fg=RED)
        self._status_lbl.pack(side="left")
        self._net_lbl = tk.Label(top, text="", bg=BG, fg=DIM, font=("Courier New", 9))
        self._net_lbl.pack(side="left", padx=10)

        # the load switch is the only control here that moves mains current
        self._relay_btn = tk.Button(top, text="LOAD  OFF", bg="#3a1414", fg=RED, relief="flat",
                                    width=14, height=2, font=("Courier New", 12, "bold"),
                                    command=self._toggle_relay)
        self._relay_btn.pack(side="right", padx=(8, 0))
        self._flag_lbl = tk.Label(top, text="", bg=BG, fg=RED, font=("Courier New", 9, "bold"))
        self._flag_lbl.pack(side="right", padx=8)

    def _build_controls(self, f):
        nb = ttk.Notebook(f)
        nb.pack(fill="both", expand=True)
        tabs = {}
        for name in ("Measure", "Calibrate", "Capture", "Device"):
            tabs[name] = tk.Frame(nb, bg=BG, padx=4)
            nb.add(tabs[name], text=name)
        self._build_measure_tab(tabs["Measure"])
        self._build_cal_tab(tabs["Calibrate"])
        self._build_capture_tab(tabs["Capture"])
        self._build_device_tab(tabs["Device"])

    def _build_measure_tab(self, f):
        self._lbl(f, "Streams", ACCENT)
        self._stream_btn = self._button(f, "▶  Start", self._toggle_stream, fg=ACCENT,
                                        bg="#224422", height=2)
        self._sm_cyc = tk.BooleanVar(value=True)
        self._sm_harm = tk.BooleanVar(value=True)
        self._sm_avg = tk.BooleanVar(value=True)
        self._sm_info = tk.BooleanVar(value=True)
        self._check(f, "per-cycle values ($CYC, 60/s)", self._sm_cyc, self._send_smask)
        self._check(f, "harmonics ($HARM, per window)", self._sm_harm, self._send_smask)
        self._check(f, "averaged cycle ($AVG, per window)", self._sm_avg, self._send_smask)
        self._check(f, "diagnostics ($INFO, every 2 s)", self._sm_info, self._send_smask)
        self._note(f, "$PWR always streams while started. On a slow link, untick the "
                      "bulk streams; the board sheds them first anyway.")

        self._sep(f)
        self._lbl(f, "Current range  (pin D6)", AMBER)
        self._irmode_var = tk.StringVar(value="0")
        row = tk.Frame(f, bg=BG)
        row.pack(fill="x")
        for val, txt in (("0", "Range 0"), ("1", "Range 1"), ("A", "Auto")):
            tk.Radiobutton(row, text=txt, value=val, variable=self._irmode_var, bg=BG, fg=WHITE,
                           selectcolor=PANEL, activebackground=BG, activeforeground=WHITE,
                           command=lambda: self._send(f"!IRANGE,{self._irmode_var.get()}")
                           ).pack(side="left")
        self._irng_lbl = tk.Label(f, text="active: —", bg=BG, fg=AMBER, anchor="w",
                                  font=("Courier New", 8))
        self._irng_lbl.pack(fill="x")
        self._irup_var, self._irdn_var = tk.StringVar(value="90"), tk.StringVar(value="60")
        self._irsettle_var = tk.StringVar(value="20")
        self._entry(f, "Auto up at:", self._irup_var, "% of FS", on_enter=self._send_irlim)
        self._entry(f, "Auto down at:", self._irdn_var, "% of FS", on_enter=self._send_irlim)
        self._entry(f, "Settle:", self._irsettle_var, "ms",
                    on_enter=lambda: self._send_num("!IRSETTLE", self._irsettle_var))
        self._irinv_var = tk.BooleanVar(value=False)
        self._check(f, "D6 HIGH selects range 0 (inverted)", self._irinv_var,
                    lambda: self._send(f"!IRINV,{int(self._irinv_var.get())}"))
        self._note(f, "Auto goes up at once on a sample over 'up' % of the sensitive "
                      "range's full scale (or clipping), and down after two windows "
                      "under 'down' %. Range 0 is selected at boot.")

        self._sep(f)
        self._hist_var = tk.StringVar(value="120")
        self._entry(f, "Trend history:", self._hist_var, "s")
        self._button(f, "Clear plots", self._clear_plots, fg=DIM)
        self._button(f, "Reset energy (Wh)", lambda: self._send("!EZERO"), fg=DIM)

        self._sep(f)
        self._lbl(f, "Protection", RED)
        self._trip_var = tk.StringVar(value="0")
        self._trippk_var = tk.StringVar(value="0")
        self._entry(f, "Trip, Irms:", self._trip_var, "A",
                    on_enter=lambda: self._send_num("!TRIP", self._trip_var))
        self._entry(f, "Trip, peak:", self._trippk_var, "A",
                    on_enter=lambda: self._send_num("!TRIPPK", self._trippk_var))
        self._note(f, "Irms is checked every mains cycle, peak on every sample. 0 disables. "
                      "A trip opens the relay, latches, and captures the waveform. "
                      "Not a substitute for a fuse.")

        self._sep(f)
        self._lbl(f, "CSV log ($PWR rows)")
        self._log_btn = self._button(f, "● Start logging", self._toggle_log)
        self._log_lbl = tk.Label(f, text="Not logging", bg=BG, fg=DIM, anchor="w",
                                 font=("Courier New", 8), wraplength=290, justify="left")
        self._log_lbl.pack(fill="x")

    def _build_cal_tab(self, f):
        self._lbl(f, "Voltage  A0 − A1", ACCENT)
        self._v_adc_var, self._v_fs_var = tk.StringVar(value="1.5"), tk.StringVar(value="170")
        self._entry(f, "ADC diff:", self._v_adc_var, "V", on_enter=self._send_vscale)
        self._entry(f, "equals:", self._v_fs_var, "V", on_enter=self._send_vscale)
        self._lbl(f, "Current  A2 − A3, per range", AMBER)
        self._i_adc_vars = [tk.StringVar(value="1.5"), tk.StringVar(value="1.5")]
        self._i_fs_vars = [tk.StringVar(value="25"), tk.StringVar(value="2.5")]
        self._pc_vars = [tk.StringVar(value="0"), tk.StringVar(value="0")]
        grid = tk.Frame(f, bg=BG)
        grid.pack(fill="x")
        for c, txt in enumerate(("", "Range 0", "Range 1", "")):
            tk.Label(grid, text=txt, bg=BG, fg=DIM).grid(row=0, column=c, sticky="w")
        for rI, (txt, vars_, unit, send) in enumerate((
                ("ADC diff:", self._i_adc_vars, "V", self._send_iscale),
                ("equals:", self._i_fs_vars, "A", self._send_iscale),
                ("PHASECAL:", self._pc_vars, "µs", self._send_phasecal)), start=1):
            tk.Label(grid, text=txt, bg=BG, fg=WHITE, width=11, anchor="w").grid(row=rI, column=0)
            for r in (0, 1):
                e = tk.Entry(grid, textvariable=vars_[r], width=8, bg=PANEL, fg=WHITE,
                             insertbackground=WHITE, relief="flat", validate="key",
                             validatecommand=(self._vcmd_float, "%P"))
                e.grid(row=rI, column=1 + r, padx=2, pady=1)
                e.bind("<Return>", lambda _, r=r, send=send: send(r))
            tk.Label(grid, text=unit, bg=BG, fg=DIM).grid(row=rI, column=3, sticky="w")
        self._irfs_lbl = tk.Label(f, text="full scale: —", bg=BG, fg=DIM, anchor="w",
                                  font=("Courier New", 8))
        self._irfs_lbl.pack(fill="x")
        self._note(f, "170 V is the peak of 120 Vrms. A negative 'equals' flips the "
                      "channel's polarity (e.g. a CT fitted backwards). PHASECAL: how much "
                      "later that range's front end is than the voltage one; trim on a "
                      "resistive load until PF = 1.000.")

        self._sep(f)
        self._button(f, "Zero V and both I ranges (no mains, no load)", self._zero, fg=AMBER)
        self._off_lbl = tk.Label(f, text="offsets: —", bg=BG, fg=DIM, anchor="w",
                                 font=("Courier New", 8))
        self._off_lbl.pack(fill="x")

        self._sep(f)
        self._cycles_var = tk.StringVar(value="12")
        self._mains_var = tk.StringVar(value="60")
        self._vmin_var = tk.StringVar(value="8.5")
        self._vref_var = tk.StringVar(value="3.3")
        self._harmn_var = tk.StringVar(value="40")
        self._entry(f, "Cycles/window:", self._cycles_var,
                    on_enter=lambda: self._send_int("!CYCLES", self._cycles_var))
        self._entry(f, "Mains:", self._mains_var, "Hz",
                    on_enter=lambda: self._send_num("!MAINS", self._mains_var))
        self._entry(f, "V crossing min:", self._vmin_var, "V pk",
                    on_enter=lambda: self._send_num("!VMIN", self._vmin_var))
        self._entry(f, "ADC VREF:", self._vref_var, "V",
                    on_enter=lambda: self._send_num("!VREF", self._vref_var))
        self._entry(f, "Harmonics:", self._harmn_var,
                    on_enter=lambda: self._send_int("!HARMN", self._harmn_var))
        self._dcrem_var = tk.BooleanVar(value=True)
        self._check(f, "remove DC per cycle", self._dcrem_var,
                    lambda: self._send(f"!DCREM,{int(self._dcrem_var.get())}"))
        self._note(f, "Press Enter in a box to send it.")

    def _build_capture_tab(self, f):
        self._lbl(f, "Waveform capture", AMBER)
        self._wlen_var = tk.StringVar(value="0.5")
        self._wpre_var = tk.StringVar(value="0.25")
        self._entry(f, "Length:", self._wlen_var, "s",
                    on_enter=lambda: self._send_num("!WLEN", self._wlen_var))
        self._entry(f, "Pre-trigger:", self._wpre_var, "of it",
                    on_enter=lambda: self._send_num("!WPRE", self._wpre_var))
        self._button(f, "⚡  Capture now", lambda: self._send("!WAVE"), fg=AMBER,
                     bg="#332200", height=2)
        self._note(f, "The board keeps ~8 s of samples, so 'now' includes the "
                      "pre-trigger part from before the click.")
        self._sep(f)
        self._lbl(f, "Capture on an event", AMBER)
        self._ev_di_var = tk.StringVar(value="0")
        self._ev_dv_var = tk.StringVar(value="0")
        self._entry(f, "I step >", self._ev_di_var, "A rms", on_enter=self._send_evt)
        self._entry(f, "V sag/swell >", self._ev_dv_var, "V rms", on_enter=self._send_evt)
        self._rlycap_var = tk.BooleanVar(value=False)
        self._check(f, "on every relay close (inrush)", self._rlycap_var,
                    lambda: self._send(f"!RLYCAP,{int(self._rlycap_var.get())}"))
        self._note(f, "Compared cycle by cycle against the previous window. 0 disables. "
                      "One capture at a time; trips capture too.")
        self._sep(f)
        self._cap_lbl = tk.Label(f, text="No capture yet", bg=BG, fg=DIM, anchor="w",
                                 font=("Courier New", 8), justify="left", wraplength=290)
        self._cap_lbl.pack(fill="x")
        self._button(f, "Save capture as CSV…", self._save_capture, fg=DIM)

    def _build_device_tab(self, f):
        self._lbl(f, "ADC", CYAN)
        self._rate_var = tk.StringVar(value="80000")
        self._decim_var = tk.StringVar(value="10")
        self._bits_var = tk.StringVar(value="16")
        self._smp_var = tk.StringVar(value=SMP_LABELS[4])
        self._entry(f, "Scan rate:", self._rate_var, "Hz",
                    on_enter=lambda: self._send_int("!RATE", self._rate_var))
        self._entry(f, "Average:", self._decim_var, "scans",
                    on_enter=lambda: self._send_int("!DECIM", self._decim_var))
        self._combo(f, "Bits:", self._bits_var, BITS_LABELS,
                    lambda: self._send(f"!BITS,{self._bits_var.get()}"))
        self._combo(f, "Sample time:", self._smp_var, SMP_LABELS,
                    lambda: self._send(f"!SMP,{SMP_LABELS.index(self._smp_var.get())}"))
        self._adc_lbl = tk.Label(f, text="", bg=BG, fg=DIM, anchor="w",
                                 font=("Courier New", 8), justify="left")
        self._adc_lbl.pack(fill="x")
        self._v_se_var = tk.BooleanVar(value=False)
        self._i_se_var = tk.BooleanVar(value=False)
        self._check(f, "voltage single-ended (ignore A1)", self._v_se_var, self._send_se)
        self._check(f, "current single-ended (ignore A3)", self._i_se_var, self._send_se)

        self._sep(f)
        self._lbl(f, "Synthetic source (test)", CYAN)
        self._syn_v, self._syn_i = tk.StringVar(value="120"), tk.StringVar(value="5")
        self._syn_deg, self._syn_h3 = tk.StringVar(value="30"), tk.StringVar(value="0")
        self._syn_hz = tk.StringVar(value="60")
        for lbl, var, unit in (("Vrms:", self._syn_v, "V"), ("Irms:", self._syn_i, "A"),
                               ("I lags V by:", self._syn_deg, "deg"),
                               ("3rd harmonic:", self._syn_h3, "% of I"),
                               ("Frequency:", self._syn_hz, "Hz")):
            self._entry(f, lbl, var, unit)
        row = tk.Frame(f, bg=BG)
        row.pack(fill="x")
        tk.Button(row, text="Synth ON", bg="#223344", fg=CYAN, relief="flat",
                  command=self._synth_on).pack(side="left", fill="x", expand=True, padx=(0, 2))
        tk.Button(row, text="Back to ADC", bg=PANEL, fg=WHITE, relief="flat",
                  command=lambda: self._send("!SYNTH,0")).pack(side="left", fill="x", expand=True)
        self._note(f, "Replaces the ADC data with an exact V/I pair. It still runs at the "
                      "ADC's pace, so it tests the maths and the link end to end.")

        self._sep(f)
        self._lbl(f, "Profile")
        self._push_var = tk.BooleanVar(value=True)
        self._check(f, "send settings to the board on connect", self._push_var, lambda: None)
        row = tk.Frame(f, bg=BG)
        row.pack(fill="x")
        tk.Button(row, text="Save profile", bg=PANEL, fg=WHITE, relief="flat",
                  command=self._save_profile).pack(side="left", fill="x", expand=True, padx=(0, 2))
        tk.Button(row, text="Send now", bg=PANEL, fg=WHITE, relief="flat",
                  command=self._push_profile).pack(side="left", fill="x", expand=True)
        self._note(f, "The board does not store settings; the profile is the record.")

    def _tile(self, parent, title, unit, color=WHITE, big=False):
        fr = tk.Frame(parent, bg=PANEL, padx=8, pady=4)
        fr.pack(side="left", fill="both", expand=True, padx=2)
        tk.Label(fr, text=title, bg=PANEL, fg=DIM, font=("Courier New", 8)).pack(anchor="w")
        v = tk.Label(fr, text="—", bg=PANEL, fg=color,
                     font=("Courier New", 20 if big else 14, "bold"))
        v.pack(anchor="w")
        tk.Label(fr, text=unit, bg=PANEL, fg=DIM, font=("Courier New", 8)).pack(anchor="w")
        return v

    def _build_display(self, f):
        r1 = tk.Frame(f, bg=BG)
        r1.pack(fill="x")
        self._t_v = self._tile(r1, "VOLTAGE", "V rms", ACCENT, True)
        self._t_i = self._tile(r1, "CURRENT", "A rms", AMBER, True)
        self._t_p = self._tile(r1, "REAL POWER", "W", CYAN, True)
        self._t_pf = self._tile(r1, "POWER FACTOR", "P / S", WHITE, True)
        r2 = tk.Frame(f, bg=BG)
        r2.pack(fill="x", pady=(4, 0))
        self._t_hz = self._tile(r2, "FREQUENCY", "Hz")
        self._t_s = self._tile(r2, "APPARENT", "VA")
        self._t_q = self._tile(r2, "NON-ACTIVE", "var")
        self._t_dpf = self._tile(r2, "DISPLACEMENT PF", "cos φ1")
        self._t_vthd = self._tile(r2, "V THD", "%", ACCENT)
        self._t_ithd = self._tile(r2, "I THD", "%", AMBER)
        self._t_ipk = self._tile(r2, "I PEAK", "A (crest —)", AMBER)
        self._t_wh = self._tile(r2, "ENERGY", "Wh", CYAN)

        nb = ttk.Notebook(f)
        nb.pack(fill="both", expand=True, pady=(6, 0))
        self._figs = {}
        for name in ("Trends", "Cycles", "Capture", "Shape", "Diagnostics"):
            fr = tk.Frame(nb, bg=BG)
            nb.add(fr, text=name)
            if name == "Diagnostics":
                self._build_diag(fr)
                continue
            fig = Figure(figsize=(8, 5), dpi=90, facecolor=BG)
            canvas = FigureCanvasTkAgg(fig, master=fr)
            canvas.get_tk_widget().pack(fill="both", expand=True)
            self._figs[name] = (fig, canvas)

        fig = self._figs["Trends"][0]
        self._ax_vi = fig.add_subplot(211); self._ax_vi2 = self._ax_vi.twinx()
        self._ax_p = fig.add_subplot(212, sharex=self._ax_vi); self._ax_p2 = self._ax_p.twinx()
        fig = self._figs["Cycles"][0]
        self._ax_ci = fig.add_subplot(211); self._ax_cv = fig.add_subplot(212, sharex=self._ax_ci)
        fig = self._figs["Capture"][0]
        self._ax_w = fig.add_subplot(111); self._ax_w2 = self._ax_w.twinx()
        fig = self._figs["Shape"][0]
        self._ax_avg = fig.add_subplot(211); self._ax_avg2 = self._ax_avg.twinx()
        self._ax_h = fig.add_subplot(212)
        for fig, _ in self._figs.values():
            fig.subplots_adjust(left=0.07, right=0.93, top=0.95, bottom=0.08, hspace=0.3)
        self._redraw_all()

    def _build_diag(self, f):
        top = tk.Frame(f, bg=BG)
        top.pack(fill="x", padx=6, pady=6)
        self._diag_lbl = tk.Label(top, text="No $INFO yet (tick diagnostics on Measure).",
                                  bg=BG, fg=WHITE, justify="left", anchor="w",
                                  font=("Courier New", 10))
        self._diag_lbl.pack(fill="x")
        row = tk.Frame(f, bg=BG)
        row.pack(fill="x", padx=6)
        self._cmd_var = tk.StringVar()
        tk.Label(row, text="Command:", bg=BG, fg=WHITE).pack(side="left")
        e = tk.Entry(row, textvariable=self._cmd_var, width=40, bg=PANEL, fg=WHITE,
                     insertbackground=WHITE, relief="flat")
        e.pack(side="left", padx=4)
        e.bind("<Return>", lambda _: (self._send(self._cmd_var.get()), self._cmd_var.set("")))
        for c in ("!CFG", "!STAT", "!SCAN", "!WIFI"):
            tk.Button(row, text=c, bg=PANEL, fg=DIM, relief="flat",
                      command=lambda c=c: self._send(c)).pack(side="left", padx=2)
        self._log_txt = tk.Text(f, bg=PANEL, fg=WHITE, font=("Courier New", 9), height=20,
                                relief="flat", insertbackground=WHITE)
        self._log_txt.pack(fill="both", expand=True, padx=6, pady=6)

    # ── connection ───────────────────────────────────────────────────────────

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        addrs = getattr(self, "_saved_addrs", []) or [DEFAULT_ADDR]
        self._addr_cb["values"] = addrs + ports

    def _toggle_connect(self):
        if self.link:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        target = self._addr_var.get().strip()
        if not target:
            return
        self._status_lbl.config(text=f"Connecting to {target}…", fg=AMBER)
        self.update_idletasks()
        try:
            self.link = Link(target)
        except Exception as e:
            self._status_lbl.config(text="Disconnected", fg=RED)
            messagebox.showerror("Connect", f"Could not open {target}:\n{e}")
            return
        if is_network_addr(target):
            addrs = getattr(self, "_saved_addrs", [])
            if target in addrs:
                addrs.remove(target)
            self._saved_addrs = [target] + addrs[:7]
            self._refresh_ports()
        self._status_lbl.config(text=f"Connected ({self.link.kind})", fg=ACCENT)
        self._connect_btn.config(text="Disconnect", bg="#552222", fg=RED)
        self._seq_last = None
        # the profile goes out once the board's $ADC has arrived, so the ADC
        # settings are only sent (each restarts acquisition) where they differ
        self._pending_push = bool(self._push_var.get())
        self._send("!CFG")

    def _disconnect(self, lost=False):
        if self.link:
            if not lost:
                try:
                    self.link.send("!SMASK,0")
                    self.link.send("!RELAY,0")     # same rule as the USB GUI: open on disconnect
                except Exception:
                    pass
            self.link.close()
        self.link = None
        self._smask = 0
        self._stream_btn.config(text="▶  Start", fg=ACCENT, bg="#224422")
        self._status_lbl.config(text="Link lost" if lost else "Disconnected", fg=RED)
        self._connect_btn.config(text="Connect", bg="#225522", fg=ACCENT)

    def _send(self, cmd: str):
        if not cmd:
            return
        if not self.link:
            self._log(f"(not connected) {cmd}")
            return
        try:
            self.link.send(cmd)
            self._log(f"> {cmd}")
        except Exception as e:
            self._log(f"send failed: {e}")
            self._disconnect(lost=True)

    def _send_num(self, cmd, var):
        s = var.get().strip()
        if _isfloat(s):
            self._send(f"{cmd},{s}")

    def _send_int(self, cmd, var):
        s = var.get().strip()
        if _isfloat(s):
            self._send(f"{cmd},{int(float(s))}")

    def _send_vscale(self):
        if _isfloat(self._v_adc_var.get()) and _isfloat(self._v_fs_var.get()):
            self._send(f"!VSCALE,{self._v_adc_var.get()},{self._v_fs_var.get()}")

    def _send_iscale(self, r):
        a, b = self._i_adc_vars[r].get(), self._i_fs_vars[r].get()
        if _isfloat(a) and _isfloat(b):
            self._send(f"!ISCALE,{a},{b},{r}")

    def _send_phasecal(self, r):
        if _isfloat(self._pc_vars[r].get()):
            self._send(f"!PHASECAL,{self._pc_vars[r].get()},{r}")

    def _send_irlim(self):
        if _isfloat(self._irup_var.get()) and _isfloat(self._irdn_var.get()):
            self._send(f"!IRLIM,{self._irup_var.get()},{self._irdn_var.get()}")

    def _send_evt(self):
        a, b = self._ev_di_var.get() or "0", self._ev_dv_var.get() or "0"
        if _isfloat(a) and _isfloat(b):
            self._send(f"!EVT,{a},{b}")

    def _send_se(self):
        self._send(f"!SE,{int(self._v_se_var.get())},{int(self._i_se_var.get())}")

    def _smask_wanted(self):
        m = SM_PWR
        if self._sm_cyc.get(): m |= SM_CYC
        if self._sm_harm.get(): m |= SM_HARM
        if self._sm_avg.get(): m |= SM_AVG
        if self._sm_info.get(): m |= SM_INFO
        return m

    def _send_smask(self):
        if self._smask:
            self._send(f"!SMASK,{self._smask_wanted()}")

    def _toggle_stream(self):
        if self._smask:
            self._send("!SMASK,0")
        else:
            self._send(f"!SMASK,{self._smask_wanted()}")

    def _toggle_relay(self):
        self._send(f"!RELAY,{0 if self._relay else 1}")

    def _zero(self):
        if messagebox.askokcancel("Zero", "Zero voltage and both current ranges now?\n\n"
                                  "The board switches the range pin to zero each range "
                                  "(about half a second). The front end should be powered "
                                  "with no mains and no load connected."):
            self._send("!ZERO")

    def _synth_on(self):
        vals = [self._syn_v.get(), self._syn_i.get(), self._syn_deg.get(),
                self._syn_h3.get(), self._syn_hz.get()]
        if all(_isfloat(v) for v in vals):
            self._send("!SYNTH,1," + ",".join(vals))

    # ── profile ──────────────────────────────────────────────────────────────

    def _profile_vars(self):
        return {
            "v_adc": self._v_adc_var, "v_fs": self._v_fs_var,
            "i_adc": self._i_adc_vars[0], "i_fs": self._i_fs_vars[0], "phasecal": self._pc_vars[0],
            "i_adc1": self._i_adc_vars[1], "i_fs1": self._i_fs_vars[1], "phasecal1": self._pc_vars[1],
            "irmode": self._irmode_var, "irup": self._irup_var, "irdn": self._irdn_var,
            "irsettle": self._irsettle_var, "cycles": self._cycles_var,
            "mains": self._mains_var, "vmin": self._vmin_var, "vref": self._vref_var,
            "harmn": self._harmn_var, "trip": self._trip_var, "trippk": self._trippk_var,
            "wlen": self._wlen_var, "wpre": self._wpre_var, "ev_di": self._ev_di_var,
            "ev_dv": self._ev_dv_var, "rate": self._rate_var, "decim": self._decim_var,
            "bits": self._bits_var, "smp": self._smp_var, "hist": self._hist_var,
        }

    def _profile_bools(self):
        return {"dcrem": self._dcrem_var, "rlycap": self._rlycap_var, "v_se": self._v_se_var,
                "i_se": self._i_se_var, "sm_cyc": self._sm_cyc, "sm_harm": self._sm_harm,
                "sm_avg": self._sm_avg, "sm_info": self._sm_info, "push": self._push_var,
                "irinv": self._irinv_var}

    def _save_profile(self):
        d = {k: v.get() for k, v in self._profile_vars().items()}
        d.update({k: bool(v.get()) for k, v in self._profile_bools().items()})
        d["addr"] = self._addr_var.get()
        d["addrs"] = getattr(self, "_saved_addrs", [])
        d["voff_uv"] = self._acfg.get("v_off_uv")
        ir = getattr(self, "_irng", {})
        d["ioff_uv"], d["ioff1_uv"] = ir.get("off0"), ir.get("off1")
        try:
            with open(PROFILE_FILE, "w") as fh:
                json.dump(d, fh, indent=2)
            self._log(f"profile saved: {PROFILE_FILE}")
        except OSError as e:
            messagebox.showerror("Profile", str(e))

    def _load_profile(self):
        try:
            with open(PROFILE_FILE) as fh:
                d = json.load(fh)
        except (OSError, ValueError):
            return
        for k, v in self._profile_vars().items():
            if k in d and d[k] is not None:
                v.set(str(d[k]))
        for k, v in self._profile_bools().items():
            if k in d:
                v.set(bool(d[k]))
        self._saved_addrs = d.get("addrs", [])
        if d.get("addr"):
            self._addr_var.set(d["addr"])
        self._prof_voff = d.get("voff_uv")
        self._prof_ioffs = [d.get("ioff_uv"), d.get("ioff1_uv")]

    def _push_profile(self):
        """Send every setting. The ADC settings restart acquisition, so only send
        them if they differ from what the board reported."""
        self._send_vscale()
        for r in (0, 1):
            self._send_iscale(r); self._send_phasecal(r)
        for cmd, var in (("!MAINS", self._mains_var), ("!IRSETTLE", self._irsettle_var),
                         ("!VMIN", self._vmin_var), ("!VREF", self._vref_var),
                         ("!TRIP", self._trip_var), ("!TRIPPK", self._trippk_var),
                         ("!WLEN", self._wlen_var), ("!WPRE", self._wpre_var)):
            self._send_num(cmd, var)
        self._send_int("!CYCLES", self._cycles_var)
        self._send_int("!HARMN", self._harmn_var)
        self._send(f"!DCREM,{int(self._dcrem_var.get())}")
        self._send(f"!RLYCAP,{int(self._rlycap_var.get())}")
        self._send_evt()
        self._send_se()
        vo = getattr(self, "_prof_voff", None)
        if vo is not None:
            self._send(f"!VOFF,{vo}")
        for r, io in enumerate(getattr(self, "_prof_ioffs", [None, None])):
            if io is not None:
                self._send(f"!IOFF,{io},{r}")
        self._send_irlim()
        self._send(f"!IRINV,{int(self._irinv_var.get())}")
        self._send(f"!IRANGE,{self._irmode_var.get()}")
        a = self._adc
        if a:
            if str(a.get("rate_req")) != self._rate_var.get():
                self._send_int("!RATE", self._rate_var)
            if str(a.get("decim")) != self._decim_var.get():
                self._send_int("!DECIM", self._decim_var)
            if str(a.get("bits")) != self._bits_var.get():
                self._send(f"!BITS,{self._bits_var.get()}")
            if a.get("smp") != SMP_LABELS.index(self._smp_var.get()):
                self._send(f"!SMP,{SMP_LABELS.index(self._smp_var.get())}")

    # ── CSV log ──────────────────────────────────────────────────────────────

    PWR_KEYS = ["vrms", "irms", "p", "s", "q", "pf", "hz", "vpk", "ipk", "wh", "t_s", "n", "fs",
                "relay", "flags", "vthd", "ithd", "dpf", "cycles", "seq", "irange"]

    def _toggle_log(self):
        if self._log_fh:
            self._log_fh.close(); self._log_fh = None
            self._log_btn.config(text="● Start logging")
            self._log_lbl.config(text=f"Stopped after {self._log_rows} rows")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv",
                                            initialfile=time.strftime("acpower_%Y%m%d_%H%M%S.csv"))
        if not path:
            return
        self._log_fh = open(path, "w", newline="")
        self._log_w = csv.writer(self._log_fh)
        self._log_w.writerow(["wall_time"] + self.PWR_KEYS + ["flag_text"])
        self._log_rows = 0
        self._log_btn.config(text="■ Stop logging")
        self._log_lbl.config(text=path)

    # ── receive ──────────────────────────────────────────────────────────────

    def _poll(self):
        if self.link:
            t_end = time.monotonic() + 0.03
            n = 0
            while time.monotonic() < t_end or n < 50:
                try:
                    line = self.link.rx.get_nowait()
                except queue.Empty:
                    break
                n += 1
                if line is None:
                    self._disconnect(lost=True)
                    break
                try:
                    self._handle(line)
                except Exception as e:
                    self._log(f"parse error: {e}: {line[:80]}")
        now = time.monotonic()
        if self._dirty and now - self._last_draw > 0.25:
            self._last_draw = now
            self._redraw(self._dirty)
            self._dirty = set()
        self.after(40, self._poll)

    def _handle(self, line: str):
        if not line.startswith("$"):
            self._log(line); return
        head, _, rest = line.partition(",")
        f = rest.split(",") if rest else []
        if head == "$PWR":   self._on_pwr(f)
        elif head == "$CYC": self._on_cyc(f)
        elif head == "$WD":
            if self._wave is not None:
                self._wave_v.append(float(f[0])); self._wave_i.append(float(f[1]))
        elif head == "$WAVE": self._on_wave_head(f)
        elif head == "$WEND": self._on_wave_end()
        elif head == "$HARM": self._on_harm(f)
        elif head == "$AVG":  self._on_avg(f)
        elif head == "$INFO": self._on_info(f)
        elif head == "$CFG":
            self._relay = f[1] == "1"; self._smask = int(f[2])
            self._set_relay_btn(); self._set_stream_btn()
        elif head == "$ADC":  self._on_adc(f)
        elif head == "$ACFG": self._on_acfg(f)
        elif head == "$IRNG": self._on_irng(f)
        elif head == "$NET":
            rssi = f"{f[2]} dBm" if f[2] not in ("0", "") else ""
            self._net_lbl.config(text=f"board IP {f[0]}:{f[1]}  {rssi}")
            self._log(line)
        elif head == "$RLY":
            self._relay = f[0] == "1"; self._set_relay_btn(); self._log(line)
        else:
            self._log(line)              # $READY $TRIP $EVT $ERR $SYNTH $SCAN $AP

    def _on_pwr(self, f):
        d = {k: float(v) for k, v in zip(self.PWR_KEYS, f)}
        seq = int(d["seq"])
        if self._seq_last is not None and seq > self._seq_last + 1:
            self._pwr_missing += seq - self._seq_last - 1
        self._seq_last = seq
        t = d["t_s"]
        for k, key in (("t", "t_s"), ("vrms", "vrms"), ("irms", "irms"), ("p", "p"),
                       ("pf", "pf"), ("s", "s"), ("ithd", "ithd")):
            self._tr[k].append(d[key])
        crest = d["ipk"] / d["irms"] if d["irms"] > 1e-6 else 0
        self._t_v.config(text=f"{d['vrms']:8.3f}")
        self._t_i.config(text=f"{d['irms']:8.4f}")
        self._t_p.config(text=f"{d['p']:9.2f}")
        self._t_pf.config(text=f"{d['pf']:6.4f}")
        self._t_hz.config(text=f"{d['hz']:.3f}" if d["hz"] > 0 else "—")
        self._t_s.config(text=f"{d['s']:.2f}")
        self._t_q.config(text=f"{d['q']:.2f}")
        self._t_dpf.config(text=f"{d['dpf']:.4f}")
        self._t_vthd.config(text=f"{d['vthd']:.2f}")
        self._t_ithd.config(text=f"{d['ithd']:.2f}")
        self._t_ipk.config(text=f"{d['ipk']:.3f}")
        self._t_ipk.master.winfo_children()[2].config(text=f"A (crest {crest:.2f})")
        self._t_wh.config(text=f"{d['wh']:.4f}")
        flags = int(d["flags"])
        self._flag_lbl.config(text="  ".join(n for b, n in FLAG_NAMES if flags & b),
                              fg=CYAN if not (flags & ~INFO_FLAGS) else RED)
        if "irange" in d:
            self._t_i.master.winfo_children()[2].config(text=f"A rms  (range {int(d['irange'])})")
        self._relay = d["relay"] > 0.5
        self._set_relay_btn()
        if self._log_fh:
            self._log_w.writerow([time.strftime("%Y-%m-%d %H:%M:%S")] + f[:len(self.PWR_KEYS)]
                                 + ["|".join(n for b, n in FLAG_NAMES if flags & b)])
            self._log_rows += 1
            if self._log_rows % 25 == 0:
                self._log_fh.flush()
                self._log_lbl.config(text=f"{self._log_rows} rows")
        self._dirty.add("Trends")

    def _on_cyc(self, f):
        seq = int(f[0])
        fs = float(self._adc.get("fs", 8000))
        hz = float(f[4])
        # x axis: cycle count / mains, so the strip reads in seconds
        mains = hz if hz > 0 else float(self._acfg.get("mains", 60))
        if self._cyc_seq0 is None or seq < self._cyc_seq0:
            self._cyc_seq0 = seq
            for dq in self._cyc.values():
                dq.clear()
        self._cyc["t"].append((seq - self._cyc_seq0) / max(mains, 1))
        self._cyc["vrms"].append(float(f[1])); self._cyc["irms"].append(float(f[2]))
        self._cyc["p"].append(float(f[3])); self._cyc["hz"].append(hz)
        self._dirty.add("Cycles")

    def _on_wave_head(self, f):
        keys = ["n", "dt_us", "kv", "bv", "ki", "bi", "alpha", "pre", "reason", "flags", "seq"]
        self._wave = dict(zip(keys, f))
        self._wave_v, self._wave_i = [], []
        self._cap_lbl.config(text=f"Receiving {self._wave['n']} samples "
                                  f"({self._wave['reason']})…", fg=AMBER)

    def _on_wave_end(self):
        w = self._wave
        if w is None:
            return
        n = int(w["n"])
        ok = len(self._wave_v) == n
        kv, bv, ki, bi = (float(w[k]) for k in ("kv", "bv", "ki", "bi"))
        v = np.array(self._wave_v) * kv + bv
        i = np.array(self._wave_i) * ki + bi
        dt = float(w["dt_us"]) * 1e-6
        alpha = float(w["alpha"])
        self._cap = {"t": (np.arange(len(v)) - int(w["pre"])) * dt, "v": v, "i": i,
                     "reason": w["reason"], "pre": int(w["pre"]), "dt": dt, "ok": ok,
                     "flags": int(w["flags"])}
        if len(v) > 4:
            vm, im = v - v.mean(), i - i.mean()
            vt = vm[:-1] + alpha * np.diff(vm)          # V at the current instants
            p = float(np.mean(vt * im[:-1]))
            vr, ir = float(np.sqrt(np.mean(vm ** 2))), float(np.sqrt(np.mean(im ** 2)))
            txt = (f"#{w['seq']} {w['reason']}  {len(v)}/{n} samples  {len(v) * dt * 1e3:.0f} ms\n"
                   f"Vrms {vr:.3f}  Irms {ir:.4f}  P {p:.2f} W\n"
                   f"I peak {np.max(np.abs(im)):.3f} A   (whole capture, not cycle-aligned)")
            if not ok:
                txt += "\nINCOMPLETE: samples were lost on the way"
            self._cap_lbl.config(text=txt, fg=WHITE if ok else RED)
        self._wave = None
        self._dirty.add("Capture")

    def _on_harm(self, f):
        H = int(f[1])
        vals = list(map(float, f[7:7 + 4 * H]))
        arr = np.array(vals).reshape(-1, 4) if vals else np.zeros((0, 4))
        self._harm = {"hz": float(f[2]), "vthd": float(f[3]), "ithd": float(f[4]),
                      "dpf": float(f[5]), "p1": float(f[6]), "a": arr}
        self._dirty.add("Shape")

    def _on_avg(self, f):
        M = int(f[1])
        vals = list(map(float, f[4:4 + 2 * M]))
        self._avg = {"n": int(f[2]), "hz": float(f[3]), "v": np.array(vals[:M]),
                     "i": np.array(vals[M:2 * M])}
        self._dirty.add("Shape")

    def _on_info(self, f):
        names = ["ADC rate meas", "gaps", "resyncs", "load %", "loop max ms", "backlog",
                 "tx drop tcp", "tx drop usb", "tcp busy", "heap max", "adc fixes",
                 "uptime s", "wifi rejoins", "lost ms", "bulk dropped"]
        now = time.monotonic()
        t0, b0 = self._bytes_t
        rate = (self.link.bytes_in - b0) / max(now - t0, 1e-3) / 1024 if self.link else 0
        self._bytes_t = (now, self.link.bytes_in if self.link else 0)
        rows = [f"{n:>14s}: {v}" for n, v in zip(names, f)]
        rows.append(f"{'GUI receive':>14s}: {rate:.1f} KB/s")
        rows.append(f"{'$PWR missing':>14s}: {self._pwr_missing}")
        half = (len(rows) + 1) // 2
        text = "\n".join(f"{a:<40s}{b}" for a, b in zip(rows[:half], rows[half:] + [""]))
        self._diag_lbl.config(text=text)

    def _on_adc(self, f):
        keys = ["id", "rate_req", "rate", "decim", "fs", "bits", "smp", "v_se", "i_se", "lag_us",
                "rate_max", "synth"]
        a = dict(zip(keys, f))
        self._adc = {"rate_req": a["rate_req"], "decim": a["decim"], "bits": a["bits"],
                     "smp": int(a["smp"]), "fs": float(a["fs"])}
        if getattr(self, "_pending_push", False):
            self._pending_push = False
            self._push_profile()          # compares the profile against self._adc
            return                        # the replies will show what the board took
        self._rate_var.set(a["rate_req"]); self._decim_var.set(a["decim"])
        self._bits_var.set(a["bits"]); self._smp_var.set(SMP_LABELS[int(a["smp"])])
        self._v_se_var.set(a["v_se"] == "1"); self._i_se_var.set(a["i_se"] == "1")
        self._adc_lbl.config(text=f"timer {float(a['rate']):.1f} Hz → {float(a['fs']):.1f} S/s analysed\n"
                                  f"V→I lag {a['lag_us']} µs, max scan {a['rate_max']} Hz"
                                  + ("\nSYNTHETIC SOURCE ON" if a["synth"] == "1" else ""))

    def _on_acfg(self, f):
        keys = ["v_adc", "v_fs", "i_adc", "i_fs", "phasecal", "cycles", "mains", "trip", "dcrem",
                "v_off_uv", "i_off_uv", "rly_inv", "wlen", "wch", "vref", "trippk", "wpre",
                "harmn", "vmin", "ev_di", "ev_dv", "rlycap"]
        a = dict(zip(keys, f))
        self._acfg = a
        # the current calibration comes per range in $IRNG
        for var, k in ((self._v_adc_var, "v_adc"), (self._v_fs_var, "v_fs"),
                       (self._cycles_var, "cycles"),
                       (self._mains_var, "mains"), (self._trip_var, "trip"),
                       (self._wlen_var, "wlen"), (self._vref_var, "vref"),
                       (self._trippk_var, "trippk"), (self._wpre_var, "wpre"),
                       (self._harmn_var, "harmn"), (self._vmin_var, "vmin"),
                       (self._ev_di_var, "ev_di"), (self._ev_dv_var, "ev_dv")):
            if k in a:
                var.set(_trim(a[k]))
        self._dcrem_var.set(a.get("dcrem") == "1")
        self._rlycap_var.set(a.get("rlycap") == "1")
        self._prof_voff = a.get("v_off_uv")
        self._show_offsets()

    def _show_offsets(self):
        ir = getattr(self, "_irng", {})
        self._off_lbl.config(text=f"offsets: V {self._acfg.get('v_off_uv', '—')} µV   "
                                  f"I {ir.get('off0', '—')} / {ir.get('off1', '—')} µV")

    def _on_irng(self, f):
        keys = ["range", "auto", "inv", "settle", "up", "dn", "switches", "fsa0", "fsa1",
                "adc0", "fs0", "off0", "pc0", "adc1", "fs1", "off1", "pc1"]
        a = dict(zip(keys, f))
        self._irng = a
        for r in (0, 1):
            self._i_adc_vars[r].set(_trim(a[f"adc{r}"]))
            self._i_fs_vars[r].set(_trim(a[f"fs{r}"]))
            self._pc_vars[r].set(_trim(a[f"pc{r}"]))
        self._prof_ioffs = [a["off0"], a["off1"]]
        self._irmode_var.set("A" if a["auto"] == "1" else a["range"])
        self._irinv_var.set(a["inv"] == "1")
        self._irsettle_var.set(_trim(a["settle"]))
        self._irup_var.set(_trim(a["up"])); self._irdn_var.set(_trim(a["dn"]))
        fs = [float(a["fsa0"]), float(a["fsa1"])]
        self._irfs_lbl.config(text=f"full scale: ±{fs[0]:.4g} A pk  /  ±{fs[1]:.4g} A pk")
        rng = int(a["range"])
        self._irng_lbl.config(text=f"active: range {rng} (±{fs[rng]:.4g} A pk)"
                                   + ("  AUTO" if a["auto"] == "1" else "")
                                   + f"   {a['switches']} switches")
        self._t_i.master.winfo_children()[2].config(text=f"A rms  (range {rng})")
        self._show_offsets()

    def _set_relay_btn(self):
        if self._relay:
            self._relay_btn.config(text="LOAD  ON", bg="#144a14", fg=ACCENT)
        else:
            self._relay_btn.config(text="LOAD  OFF", bg="#3a1414", fg=RED)

    def _set_stream_btn(self):
        if self._smask:
            self._stream_btn.config(text="■  Stop", fg=RED, bg="#442222")
        else:
            self._stream_btn.config(text="▶  Start", fg=ACCENT, bg="#224422")

    def _log(self, text):
        self._log_txt.insert("end", time.strftime("%H:%M:%S ") + text + "\n")
        if int(self._log_txt.index("end-1c").split(".")[0]) > 2000:
            self._log_txt.delete("1.0", "500.0")
        self._log_txt.see("end")

    # ── plots ────────────────────────────────────────────────────────────────

    def _style(self, ax, ylabel=None, color=DIM, xlabel=None, right=False):
        if right:                       # cla() puts a twin axis' ticks back on the left
            ax.yaxis.tick_right()
            ax.yaxis.set_label_position("right")
        ax.set_facecolor(BG)
        ax.tick_params(colors=DIM, labelsize=8)
        for s in ax.spines.values():
            s.set_color(SEP)
        ax.grid(True, color="#2c2c2c", lw=0.6)
        if ylabel:
            ax.set_ylabel(ylabel, color=color, fontsize=9)
        if xlabel:
            ax.set_xlabel(xlabel, color=DIM, fontsize=9)

    def _clear_plots(self):
        for dq in list(self._tr.values()) + list(self._cyc.values()):
            dq.clear()
        self._cyc_seq0 = None
        self._redraw_all()

    def _redraw_all(self):
        self._redraw({"Trends", "Cycles", "Capture", "Shape"})

    def _redraw(self, which):
        for name in which:
            getattr(self, f"_draw_{name.lower()}")()
            self._figs[name][1].draw_idle()

    def _draw_trends(self):
        for ax in (self._ax_vi, self._ax_vi2, self._ax_p, self._ax_p2):
            ax.cla()
        self._style(self._ax_vi, "Vrms", ACCENT); self._style(self._ax_vi2, "Irms", AMBER, right=True)
        self._style(self._ax_p, "W", CYAN, xlabel="board time, s"); self._style(self._ax_p2, "PF", WHITE, right=True)
        t = np.array(self._tr["t"])
        if len(t) > 1:
            try:
                hist = float(self._hist_var.get())
            except ValueError:
                hist = 120.0
            m = t >= t[-1] - hist
            step = max(1, int(m.sum() / 3000))
            tt = t[m][::step]
            sel = lambda k: np.array(self._tr[k])[m][::step]
            self._ax_vi.plot(tt, sel("vrms"), color=ACCENT, lw=1)
            self._ax_vi2.plot(tt, sel("irms"), color=AMBER, lw=1)
            self._ax_p.plot(tt, sel("p"), color=CYAN, lw=1)
            self._ax_p2.plot(tt, sel("pf"), color=WHITE, lw=0.8, alpha=0.8)

    def _draw_cycles(self):
        self._ax_ci.cla(); self._ax_cv.cla()
        self._style(self._ax_ci, "Irms per cycle", AMBER)
        self._style(self._ax_cv, "Vrms per cycle", ACCENT, xlabel="cycles / mains Hz, s")
        t = np.array(self._cyc["t"])
        if len(t) > 1:
            m = t >= t[-1] - 20.0              # last 20 s of cycles
            self._ax_ci.plot(t[m], np.array(self._cyc["irms"])[m], color=AMBER, lw=0.8)
            self._ax_cv.plot(t[m], np.array(self._cyc["vrms"])[m], color=ACCENT, lw=0.8)

    def _draw_capture(self):
        self._ax_w.cla(); self._ax_w2.cla()
        self._style(self._ax_w, "V", ACCENT, xlabel="ms from trigger")
        self._style(self._ax_w2, "A", AMBER, right=True)
        c = getattr(self, "_cap", None)
        if c is not None:
            self._ax_w.plot(c["t"] * 1e3, c["v"], color=ACCENT, lw=0.8)
            self._ax_w2.plot(c["t"] * 1e3, c["i"], color=AMBER, lw=0.8)
            self._ax_w.axvline(0, color=RED, lw=0.8, ls="--")

    def _draw_shape(self):
        self._ax_avg.cla(); self._ax_avg2.cla(); self._ax_h.cla()
        self._style(self._ax_avg, "V (averaged cycle)", ACCENT, xlabel="degrees from V rising crossing")
        self._style(self._ax_avg2, "A", AMBER, right=True)
        self._style(self._ax_h, "% of fundamental", WHITE, xlabel="harmonic")
        a = getattr(self, "_avg", None)
        if a is not None and a["n"] > 0:
            x = np.arange(len(a["v"])) * 360.0 / len(a["v"])
            self._ax_avg.plot(x, a["v"], color=ACCENT, lw=1.2)
            self._ax_avg2.plot(x, a["i"], color=AMBER, lw=1.2)
            self._ax_avg.set_title(f"mean of {a['n']} cycles at {a['hz']:.3f} Hz",
                                   color=DIM, fontsize=8)
        h = getattr(self, "_harm", None)
        if h is not None and len(h["a"]) > 1:
            arr = h["a"]
            n = np.arange(2, len(arr) + 1)
            v1 = arr[0, 0] or 1e-12
            i1 = arr[0, 2] or 1e-12
            self._ax_h.bar(n - 0.2, 100 * arr[1:, 0] / v1, width=0.4, color=ACCENT, label="V")
            self._ax_h.bar(n + 0.2, 100 * arr[1:, 2] / i1, width=0.4, color=AMBER, label="I")
            self._ax_h.set_title(f"THD V {h['vthd']:.2f} %   I {h['ithd']:.2f} %   "
                                 f"cos φ1 {h['dpf']:.4f}   P1 {h['p1']:.2f} W",
                                 color=DIM, fontsize=8)
            self._ax_h.legend(facecolor=PANEL, edgecolor=SEP, labelcolor=WHITE, fontsize=8)

    def _save_capture(self):
        c = getattr(self, "_cap", None)
        if c is None:
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv",
                                            initialfile=time.strftime("capture_%Y%m%d_%H%M%S.csv"))
        if not path:
            return
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t_s", "v", "i"])
            for t, v, i in zip(c["t"], c["v"], c["i"]):
                w.writerow([f"{t:.7f}", f"{v:.5f}", f"{i:.6f}"])

    def on_close(self):
        if self.link:
            self._disconnect()
        if self._log_fh:
            self._log_fh.close()
        self.destroy()


def _isfloat(s) -> bool:
    try:
        float(s)
        return True
    except (TypeError, ValueError):
        return False


def _trim(s: str) -> str:
    """'170.0000' -> '170', '1.500000' -> '1.5' for the entry boxes."""
    if "." in s:
        s = s.rstrip("0").rstrip(".")
    return s or "0"


if __name__ == "__main__":
    App().mainloop()
