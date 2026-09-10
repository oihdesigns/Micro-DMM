#!/usr/bin/env python3
"""
blinkyhawk_bench_gui.py  --  Host GUI for the BlinkyHawk_Bench firmware.

The bench counterpart of blinkyhawk_gui.py, for the experimental fork that
runs a XIAO RA4M1 and a Blinky Hawk PCB as separate parts on a breadboard.
Everything the production GUI does, plus a Bench tab for the two things that
fork exists to do: move a pin without recompiling, and gate part of the
circuit to find out what it costs.

Talks to the board over USB serial (115200 baud).  Three tabs:

  Diagnostics
    - enter/exit diagnostic mode, stream both ADC channels + differential
    - lock voltage mode ON or disable it, park the MOSFET
    - trigger a transient capture across a MOSFET toggle, plot + save CSV
    - re-enable the normal LED alerts while charging

  Configuration (EEPROM)
    - reads every config key from the device (!CFG) and shows it in a table
    - edit any value and push it to device RAM (!SET -- applied immediately)
    - persist RAM to EEPROM (!SAVE), reload EEPROM (!LOAD), factory
      defaults (!DEFAULTS); unsaved-changes state tracked from $STATUS dirty=
    - "Copy from unit..." clones a previously-saved unit's tuning onto the
      connected board, picked by serial number out of the master CSV log.
      Shows a full diff first and writes to RAM only, so it is reviewable
      and reversible until you Save.  HWREV is never copied (NEVER_COPY_KEYS)
    - "Restore this unit..." is the same dialog pointed at THIS unit's own
      row.  Copying between units must not carry wiring, but restoring a
      unit to its own last saved state must -- so the pin map and gate
      polarities ARE sent here, and only HWREV stays protected
    - unknown keys reported by newer firmware still appear (in "Other"), so
      this GUI does not need updating for every firmware tweak

  Bench (this fork only)
    - pin map: reads !PINS, shows what every function is wired to, and
      reassigns one from a dropdown of the pin names this core actually has
    - power gates: the live state of ANA / LEDG / AUX, and an Auto/On/Off
      hold (!GATE) that survives into sleep and !FLOOR -- which is how the
      sleeping current for a given gating scheme gets measured
    - two-stage sleep: the derived stage-2 schedule (!DEEP) including what
      the bridge MOSFET resolves to in each stage, and buttons to force the
      board between stages instead of waiting out the DEEPSEC countdown
    - power profile: runs !EXPT and keeps the schedule, so a PPK2 capture
      taken over the same window can be sliced into its ten steps.
      "Save plan CSV" writes those boundaries out as start/end seconds

Two warnings worth repeating from the firmware:
  - the pin map and gate polarities describe THIS breadboard, so they are in
    NEVER_COPY_KEYS and "Copy from unit..." will not clone them
  - the per-unit CSV log is blinkyhawk_bench_units.csv, deliberately separate
    from the production log: bench numbers are not unit tuning records

The top bar always shows charge/battery state and the live threshold
position ($DIP lines).  On HWREV 2 boards that position comes from the
physical DIP switches; on HWREV 3 (no DIP switches fitted) it is the
THRESHSEL config key.  Same four slots, same message, either way.

Descended from OpenLeadDetect_XIAO_Minimal/diagnostic_gui.py with the A5
potentiometer features (calibration sweep, offline battery log, !THRESH,
!NEGFIX) removed -- NEGFIX/NEGV are plain EEPROM keys now.

Dependencies:
    pip install pyserial matplotlib

Run:
    py blinkyhawk_bench_gui.py
"""

import csv
import os
import queue
import threading
import tkinter as tk
from collections import deque
from datetime import datetime
from tkinter import filedialog, messagebox, simpledialog, ttk

import serial
import serial.tools.list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

BAUD = 115200
LIVE_MAXLEN = 600          # samples kept in the rolling live plot
PLOT_REFRESH_MS = 80       # GUI redraw cadence

# Master per-unit config log: one row per serial number, updated in place each
# time that unit's RAM is saved to EEPROM.  Lives next to this script so it is
# found regardless of the working directory.
UNITS_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "blinkyhawk_bench_units.csv")
UNITS_CSV_FIXED = ["SN", "timestamp"]   # leading columns; config keys follow

# Keys that must NEVER be copied from one unit to another by "Copy from unit".
# HWREV is not a preference, it is a statement about which PCB the XIAO is
# plugged into.  Copying a V2 donor's HWREV=2 onto a V3 board silences its
# buzzer and freezes its threshold selector; copying a V3 donor's HWREV=3 onto
# a V2 board is worse -- the firmware would start driving D8 as a push-pull
# output into whatever that board's DIP switch is doing, which is a dead short
# to ground whenever the switch is closed.  The firmware protects itself the
# same way (its own !DEFAULTS preserves HWREV); this is the host-side half.
# BENCH: the pin map and the gate polarities are statements about how THIS
# breadboard is jumpered, in exactly the way HWREV is a statement about which
# PCB is underneath.  Copying them from another unit's row would silently point
# the firmware at pins that are not connected to anything here -- and, worse,
# start driving pins that are.  Modes and settle times ARE tuning and copy
# normally.
# NOTE the distinction this list does NOT make on its own: these are unsafe to
# copy BETWEEN units, but they are exactly what you want back when restoring a
# unit from its OWN row.  _copy_plan(restore=True) narrows the set to
# RESTORE_PROTECTED_KEYS for that case -- see _cfg_restore_this_unit.
NEVER_COPY_KEYS = {"HWREV",
                   "PINSENSEP", "PINSENSEN", "PINCHARGE", "PINMOSFET",
                   "PINSPKA", "PINSPKB", "PINLEDDAT", "PINDIPA", "PINDIPB",
                   "PINANA", "PINLEDG", "PINAUX",
                   "ANAPOL", "LEDGPOL", "AUXPOL"}

# Restoring a unit from its own row: the wiring came off this same board, so it
# is safe and wanted.  HWREV still is not -- if the XIAO has been moved to a
# different PCB since that row was written, writing a stale HWREV is the one
# mistake that can drive D8 into a short.  Set it deliberately, never by restore.
RESTORE_PROTECTED_KEYS = {"HWREV"}

# Bench pin-map keys, in the order the Bench tab lists them:
#   $PIN row function name -> config key
PIN_FUNC_KEYS = [
    ("SENSEP",  "PINSENSEP", "Differential + input"),
    ("SENSEN",  "PINSENSEN", "Differential - input (NC on V3)"),
    ("CHARGE",  "PINCHARGE", "VBUS/2 charge sense"),
    ("MOSFET",  "PINMOSFET", "Bridge MOSFET gate"),
    ("SPKA",    "PINSPKA",   "Buzzer hot leg"),
    ("SPKB",    "PINSPKB",   "Buzzer anti-phase leg"),
    ("LEDDATA", "PINLEDDAT", "SK6812 data"),
    ("DIPA",    "PINDIPA",   "Threshold DIP A (HWREV 2)"),
    ("DIPB",    "PINDIPB",   "Threshold DIP B (HWREV 2)"),
    ("GATEANA", "PINANA",    "Gate: analog front end"),
    ("GATELED", "PINLEDG",   "Gate: LED supply"),
    ("GATEAUX", "PINAUX",    "Gate: spare"),
]
GATE_NAMES = ["ANA", "LED", "AUX"]

# ---------------------------------------------------------------------------
# Config-key metadata: display grouping, label, and a short description.
# The table itself is built from whatever the DEVICE reports in its !CFG dump,
# so a key missing here still shows up (under "Other") -- this dict only makes
# known keys prettier.  kind "bool" renders as a 0/1 dropdown; kind "choice"
# renders as a dropdown of the values in the optional 4th tuple element.
# ---------------------------------------------------------------------------
KEY_META = {
    # --- Board ---
    # HWREV decides what D8 physically is, so it is the one key that must match
    # the PCB before anything else is trusted.  It survives !DEFAULTS.
    "HWREV":        ("Board", "choice", "PCB revision: 2 = DIP switches + single-ended buzzer, "
                                        "3 = no DIP (see THRESHSEL) + buzzer across D8/D9",
                     ["2", "3"]),
    # --- Detection ---
    "REFCENTER":    ("Detection", "num",  "Resting differential centre (V)"),
    "REFBAND":      ("Detection", "num",  "No-voltage window half-width (V)"),
    # THRESH units follow DETMETHOD: V (single) / ms (time-to-return) / V*ms (area).
    # Positions are named for the V2 DIP switches; on HWREV 3 the same four slots
    # are selected by THRESHSEL instead.
    "THRESH00":     ("Detection", "num",  "Threshold, position 00 = both switches ON (units per DETMETHOD)"),
    "THRESH01":     ("Detection", "num",  "Threshold, position 01 = D8 ON, D10 OFF (units per DETMETHOD)"),
    "THRESH10":     ("Detection", "num",  "Threshold, position 10 = D8 OFF, D10 ON (units per DETMETHOD)"),
    "THRESH11":     ("Detection", "num",  "Threshold, position 11 = both switches OFF (units per DETMETHOD)"),
    "THRESHSEL":    ("Detection", "choice", "HWREV 3 only: which THRESH slot is active "
                                            "(0=00, 1=01, 2=10, 3=11). Ignored on HWREV 2.",
                     ["0", "1", "2", "3"]),
    "VOLTFAST":     ("Detection", "num",  "Instant voltage-present multiplier (x REFBAND)"),
    "VOLTAVG":      ("Detection", "num",  "Reads averaged for voltage decision"),
    "TESTAGREE":    ("Detection", "num",  "Consecutive matching MOSFET tests required"),
    "STABLECOUNT":  ("Detection", "num",  "Passes before the alert state switches"),
    "SETTLEPREUS":  ("Detection", "num",  "Settle after MOSFET off, before read (us; method 0 only)"),
    "SETTLEPOSTMS": ("Detection", "num",  "Idle after read, before MOSFET on (ms)"),
    "NEGFIX":       ("Detection", "bool", "1 = fixed pseudo-reference instead of live neg pin"),
    "NEGV":         ("Detection", "num",  "Fixed pseudo-reference voltage (V)"),
    "DETMETHOD":    ("Detection", "choice", "Metric: 0=single |diff|(V)  1=time-to-return(ms)  2=tail-area(V*ms)",
                     ["0", "1", "2"]),
    "DETBAND":      ("Detection", "num",  "Method 1: |diff-centre| within this = 'returned' (V)"),
    "DETWINUS":     ("Detection", "num",  "Methods 1&2: max sample window / timeout (us)"),
    "DETAREAUS":    ("Detection", "num",  "Method 2: tail-area integration start (us from toggle)"),
    # --- Alerts ---
    "LED":          ("Alerts", "bool", "Master enable: detection LED alerts"),
    "BEEP":         ("Alerts", "bool", "Master enable: speaker"),
    "BOOTMUTE":     ("Alerts", "bool", "Leads shorted at boot mutes audio for session"),
    "PASSIVE":      ("Alerts", "bool", "1 = passive buzzer (tone), 0 = active buzzer"),
    "SPKDIFF":      ("Alerts", "bool", "HWREV 3 only: 1 = drive D8 anti-phase to D9 "
                                       "(2x swing across the piezo, ~+6 dB); 0 = park D8 low "
                                       "for the single-ended V2 drive"),
    "CONTFREQ":     ("Alerts", "num",  "Continuity beep pitch (Hz, passive only)"),
    "VOLTFREQ":     ("Alerts", "num",  "Voltage beep pitch (Hz, passive only)"),
    "CONTPULSES":   ("Alerts", "num",  "Pulses per continuity beep"),
    "VOLTPULSES":   ("Alerts", "num",  "Pulses per voltage beep"),
    "CONTREP":      ("Alerts", "bool", "Re-beep while continuity holds"),
    "VOLTREP":      ("Alerts", "bool", "Re-beep while voltage persists"),
    "CONTREPMS":    ("Alerts", "num",  "Continuity repeat period (ms)"),
    "VOLTREPMS":    ("Alerts", "num",  "Voltage repeat period (ms)"),
    "BEEPMIN":      ("Alerts", "num",  "Min gap between beep sequences (ms)"),
    "CONTONMS":     ("Alerts", "num",  "Continuity pulse on-time, first contact (ms)"),
    "CONTHOLDMS":   ("Alerts", "num",  "Continuity pulse on-time, ongoing re-beep while held (ms)"),
    "CONTOFFMS":    ("Alerts", "num",  "Gap between continuity pulses (ms)"),
    "VOLTONMS":     ("Alerts", "num",  "Voltage pulse on-time (ms)"),
    "VOLTOFFMS":    ("Alerts", "num",  "Gap between voltage pulses (ms)"),
    # --- Alert LED --- (hue is fixed in firmware: blue/green/red = the meaning)
    "LEDFLOATBR":   ("Alert LED", "num", "Floating (blue) brightness 0-255; 0 = this state dark"),
    "LEDCLOSEDBR":  ("Alert LED", "num", "Closed (green) brightness 0-255; 0 = this state dark"),
    "LEDVOLTBR":    ("Alert LED", "num", "Voltage (red) brightness 0-255; 0 = this state dark"),
    "LEDFLOATMS":   ("Alert LED", "num", "Floating flash on-time (ms)"),
    "LEDCLOSEDMS":  ("Alert LED", "num", "Closed flash on-time (ms)"),
    "LEDVOLTMS":    ("Alert LED", "num", "Voltage flash on-time (ms)"),
    "LEDFLOATPER":  ("Alert LED", "num", "Floating: min gap between flash STARTS (ms) = rate cap"),
    "LEDCLOSEDPER": ("Alert LED", "num", "Closed: min gap between flash STARTS (ms) = rate cap"),
    "LEDVOLTPER":   ("Alert LED", "num", "Voltage: min gap between flash STARTS (ms) = rate cap"),
    # --- Power / battery ---
    "CHGTHRESH":    ("Power / battery", "num", "VBUS/2 level meaning 'charging' (V)"),
    "BATTEMPTY":    ("Power / battery", "num", "Battery voltage mapped to 0% (V)"),
    "BATTFULL":     ("Power / battery", "num", "Battery voltage mapped to 100% (V)"),
    "BATTFULLPCT":  ("Power / battery", "num", "Charge % at which blink turns green"),
    "CHGINHIBIT":   ("Power / battery", "bool", "1 = normal: 5 V on the input blocks sleep and "
                                                "normal alerts (production behaviour). "
                                                "0 = bench: charge is still detected and shown, "
                                                "but blocks nothing -- for running the board off "
                                                "a bench supply, a boost converter, or primary "
                                                "cells behind a boost, where VBUS is just how it "
                                                "is powered and never goes away"),
    # --- Low power ---
    "SLEEPSEC":     ("Low power", "num",  "Open-lead idle time before sleeping (s; 0 = never sleep). "
                                          "Small values (1-2 s) + a fast SLEEPTICKMS = poll-while-asleep operation"),
    # The RTC ladder runs 2 s down to 1/256 s. The sub-125 ms rungs are rounded
    # here and in firmware (62.5 -> 63, 3.90625 -> 4); the interrupt itself is exact.
    "SLEEPTICKMS":  ("Low power", "choice", "Base wake period (ms); snapped to a rate the RTC can produce. "
                                            "63/31/16/8/4 are the 1/16..1/256 s rungs (rounded); below ~30 ms "
                                            "the probe dominates the tick and average current climbs to awake levels",
                     ["2000", "1000", "500", "250", "125", "63", "31", "16", "8", "4"]),
    "SLEEPTICKS":   ("Low power", "num",  "Probe every N wake ticks (N x SLEEPTICKMS between checks; 1-255)"),
    "SLEEPAVG":     ("Low power", "num",  "Reads per sleeping voltage check; ANY over VOLTFAST x REFBAND wakes (fewer = quieter)"),
    "SLEEPHB":      ("Low power", "num",  "Heartbeat flash every N ticks (counts SLEEPTICKMS ticks, not seconds)"),
    "SLEEPPARK":    ("Low power", "bool", "1 = park the bridge MOSFET OFF while asleep"),
    "SLEEPTHR00":   ("Low power", "num",  "Wake threshold, position 00 (0 = use THRESH00)"),
    "SLEEPTHR01":   ("Low power", "num",  "Wake threshold, position 01 (0 = use THRESH01)"),
    "SLEEPTHR10":   ("Low power", "num",  "Wake threshold, position 10 (0 = use THRESH10)"),
    "SLEEPTHR11":   ("Low power", "num",  "Wake threshold, position 11 (0 = use THRESH11)"),
    # --- Deep sleep, stage 2 (this fork only) ---
    # The board sleeps in two stages: SLEEPSEC gets it into stage 1 (the normal
    # operating mode), then DEEPSEC of uninterrupted open leads drops it to
    # stage 2, where the RTC wake period itself is slowed -- not just the probe
    # divisor -- because the standby wake is most of the cost at these rates.
    "DEEPSEC":      ("Low power", "num", "Seconds of stage-1 sleep, all reading open, before "
                                         "dropping to the deep stage (0 = never; single-stage, "
                                         "as production)"),
    "DEEPPARK":     ("Low power", "choice", "Bridge MOSFET in the deep stage: 0 = resting (ON), "
                                            "1 = parked OFF, 2 = follow SLEEPPARK (default). "
                                            "The bridge resting draws continuously through the "
                                            "100K sense leg, so 1 is the next real saving after "
                                            "the wake rate -- but the node then floats for a "
                                            "whole deep period between probes, so raise "
                                            "SETTLEPOSTMS if closed leads stop waking the board",
                     ["0", "1", "2"]),
    "DEEPHZ":       ("Low power", "num", "Deep-stage probe rate (Hz). Snapped to what the RTC "
                                         "ladder can produce, never faster than asked, and the "
                                         "achieved rate is written back -- so this box always "
                                         "shows the rate in force. Costs latency: at 1 Hz a "
                                         "closed lead waits up to a second to be noticed"),
    # --- Misc ---
    "LOOPMS":       ("Misc", "num", "Main-loop pacing / sleep (ms)"),
    # --- Bench pin map (this fork only) ---
    # Values are Arduino pin NUMBERS, not names.  The Bench tab has a dropdown
    # that does the lookup; these entries exist so the keys are still editable
    # from the config table if you already know the number.
    "PINSENSEP":    ("Bench pin map", "num", "Differential + input (default A2)"),
    "PINSENSEN":    ("Bench pin map", "num", "Differential - input (default A1; not connected on V3)"),
    "PINCHARGE":    ("Bench pin map", "num", "VBUS/2 charge sense (default A3)"),
    "PINMOSFET":    ("Bench pin map", "num", "Bridge MOSFET gate (default D7)"),
    "PINSPKA":      ("Bench pin map", "num", "Buzzer hot leg (default D9)"),
    "PINSPKB":      ("Bench pin map", "num", "Buzzer anti-phase leg (default D8; 255 = not wired)"),
    "PINLEDDAT":    ("Bench pin map", "num", "SK6812 data (default D6 = 6)"),
    "PINDIPA":      ("Bench pin map", "num", "Threshold DIP A, HWREV 2 only (default D8)"),
    "PINDIPB":      ("Bench pin map", "num", "Threshold DIP B, HWREV 2 only (default D10)"),
    # --- Bench power gates ---
    "PINANA":       ("Bench power gates", "num", "Analog-front-end load switch (255 = not wired)"),
    "ANAPOL":       ("Bench power gates", "bool", "1 = HIGH enables the analog rail, 0 = LOW enables it "
                                                  "(a P-FET high-side switch is usually active LOW)"),
    "ANAMODE":      ("Bench power gates", "choice", "0 = always on (control case)  1 = on awake, off asleep  "
                                                    "2 = pulsed: raised only for each measurement",
                     ["0", "1", "2"]),
    "ANAUS":        ("Bench power gates", "num", "Settle after raising the analog gate, before reading (us)"),
    "PINLEDG":      ("Bench power gates", "num", "LED1 supply load switch (255 = not wired)"),
    "LEDGPOL":      ("Bench power gates", "bool", "1 = HIGH enables LED1's supply, 0 = LOW enables it"),
    "LEDGMODE":     ("Bench power gates", "choice", "0 = always on  1 = on awake, off asleep  "
                                                    "2 = pulsed: up only while a colour is displayed",
                     ["0", "1", "2"]),
    "LEDGMS":       ("Bench power gates", "num", "Settle after raising the LED gate, before pixel data (ms)"),
    "PINAUX":       ("Bench power gates", "num", "Spare load switch (255 = not wired)"),
    "AUXPOL":       ("Bench power gates", "bool", "1 = HIGH enables the spare rail, 0 = LOW enables it"),
    "AUXMODE":      ("Bench power gates", "choice", "0 = always on  1 = on awake, off asleep  2 = pulsed",
                     ["0", "1", "2"]),
    "AUXUS":        ("Bench power gates", "num", "Settle after raising the spare gate (us)"),
}
GROUP_ORDER = ["Board", "Detection", "Alerts", "Alert LED", "Power / battery",
               "Low power", "Bench pin map", "Bench power gates", "Misc", "Other"]


class SerialManager:
    """Background serial reader. Pushes decoded lines onto a queue."""

    def __init__(self, line_queue):
        self.line_queue = line_queue
        self.ser = None
        self._stop = threading.Event()
        self._thread = None

    def connect(self, port):
        self.disconnect()
        self.ser = serial.Serial(port, BAUD, timeout=0.1)
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
            self.ser.write(text.encode("ascii", errors="ignore"))

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
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", errors="replace").strip()
                if line:
                    self.line_queue.put(("line", line))


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Blinky Hawk BENCH -- Diagnostics, Configuration & Power Gating")
        self.geometry("1100x860")

        self.line_queue = queue.Queue()
        self.serial = SerialManager(self.line_queue)

        # live rolling data
        self.t0 = None
        self.live_t = deque(maxlen=LIVE_MAXLEN)
        self.live_pos = deque(maxlen=LIVE_MAXLEN)
        self.live_neg = deque(maxlen=LIVE_MAXLEN)
        self.live_diff = deque(maxlen=LIVE_MAXLEN)

        # capture assembly
        self.cap_active = False
        self.cap_header = None
        self.cap_rows = []
        self.last_capture = None   # parsed rows of the most recent capture

        # config table: key -> row widgets/state (built lazily from the device)
        self.cfg_rows = {}
        self.cfg_pending_end = False

        # active detection method (from $STATUS) -> threshold display units
        self.detmethod = 0

        # unit identity + per-unit config logging
        self.sn = ""                    # serial number reported by the device
        self.sn_prompted = False        # asked to assign an SN this connection?
        self.pending_csv_log = False    # log to CSV once the post-save !CFG lands
        self.units_csv_path = UNITS_CSV

        # --- bench state ---
        self.pin_names = []             # [(name, num)] from $PINNAME
        self.pin_map = {}               # function -> (num_or_None, name)
        self.pin_rows_pending = {}      # accumulates $PIN rows until $PINEND
        self.gate_info = {}             # gate name -> dict from $GATE
        self.expt_plan = []             # [(idx, name, start_s, parked, deep)]
        self.expt_running = False
        self.chg_inhibit = True         # $STATUS chginhibit=; assume production

        self._build_ui()
        self.after(PLOT_REFRESH_MS, self._tick)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------------------------------------------------------- UI
    def _build_ui(self):
        # --- connection / device-state row ---
        top = ttk.Frame(self, padding=6)
        top.pack(fill="x")
        ttk.Label(top, text="Port:").pack(side="left")
        self.port_cb = ttk.Combobox(top, width=18, values=self._ports())
        self.port_cb.pack(side="left", padx=4)
        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side="left")
        self.connect_btn = ttk.Button(top, text="Connect", command=self._toggle_connect)
        self.connect_btn.pack(side="left", padx=4)
        self.conn_lbl = ttk.Label(top, text="disconnected", foreground="#a00")
        self.conn_lbl.pack(side="left", padx=8)

        self.sn_lbl = ttk.Label(top, text="SN: --", font=("Consolas", 10, "bold"))
        self.sn_lbl.pack(side="left", padx=12)

        self.dip_lbl = ttk.Label(top, text="THR POS: --", font=("Consolas", 10))
        self.dip_lbl.pack(side="right", padx=6)
        # Gate levels live in the top bar because they change under you: a
        # pulsed gate is down between measurements, and an experiment step
        # moves all three without anyone touching a control.
        self.gates_lbl = ttk.Label(top, text="gates: ---", font=("Consolas", 10))
        self.gates_lbl.pack(side="right", padx=6)
        self.sleep_lbl = ttk.Label(top, text="sleep: --", font=("Consolas", 10))
        self.sleep_lbl.pack(side="right", padx=6)
        self.batt_lbl = ttk.Label(top, text="batt: --")
        self.batt_lbl.pack(side="right", padx=6)
        self.charge_lbl = ttk.Label(top, text="charge: --")
        self.charge_lbl.pack(side="right", padx=6)

        # --- notebook: Diagnostics | Configuration ---
        self.nb = ttk.Notebook(self)
        self.nb.pack(fill="both", expand=True, padx=6, pady=4)
        self.diag_tab = ttk.Frame(self.nb)
        self.cfg_tab = ttk.Frame(self.nb)
        self.bench_tab = ttk.Frame(self.nb)
        self.nb.add(self.diag_tab, text="  Diagnostics  ")
        self.nb.add(self.cfg_tab, text="  Configuration (EEPROM)  ")
        self.nb.add(self.bench_tab, text="  Bench (pins / gates / power)  ")
        self._build_diag_tab()
        self._build_cfg_tab()
        self._build_bench_tab()

        # --- readouts + log (shared, below the notebook) ---
        self.status_lbl = ttk.Label(self, text="status: --", anchor="w",
                                    font=("Consolas", 9))
        self.status_lbl.pack(fill="x", padx=6)

        logf = ttk.LabelFrame(self, text="Serial log", padding=4)
        logf.pack(fill="both", padx=6, pady=4)
        self.log = tk.Text(logf, height=7, wrap="none", font=("Consolas", 8))
        self.log.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(logf, command=self.log.yview)
        sb.pack(side="right", fill="y")
        self.log["yscrollcommand"] = sb.set

    # ---------------------------------------------------- diagnostics tab
    def _build_diag_tab(self):
        ctl = ttk.LabelFrame(self.diag_tab, text="Controls", padding=6)
        ctl.pack(fill="x", padx=4, pady=4)

        self.diag_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(ctl, text="Diagnostic mode", variable=self.diag_var,
                        command=self._on_diag).grid(row=0, column=0, padx=4, sticky="w")
        self.stream_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(ctl, text="Stream", variable=self.stream_var,
                        command=self._on_stream).grid(row=0, column=1, padx=4, sticky="w")

        ttk.Label(ctl, text="Rate (ms):").grid(row=0, column=2, sticky="e")
        self.rate_var = tk.StringVar(value="20")
        ttk.Entry(ctl, width=6, textvariable=self.rate_var).grid(row=0, column=3, padx=2)
        ttk.Button(ctl, text="Set", command=self._on_rate).grid(row=0, column=4, padx=2)

        # voltage mode
        vm = ttk.LabelFrame(ctl, text="Voltage mode", padding=4)
        vm.grid(row=1, column=0, columnspan=3, padx=4, pady=4, sticky="w")
        self.vmode_var = tk.IntVar(value=0)
        for i, label in enumerate(["Auto", "Lock ON", "Disable"]):
            ttk.Radiobutton(vm, text=label, value=i, variable=self.vmode_var,
                            command=self._on_vmode).pack(side="left", padx=4)

        # mosfet hold
        mf = ttk.LabelFrame(ctl, text="MOSFET", padding=4)
        mf.grid(row=1, column=3, columnspan=2, padx=4, pady=4, sticky="w")
        self.mosfet_var = tk.IntVar(value=-1)
        for val, label in [(-1, "Auto"), (0, "Hold OFF"), (1, "Hold ON")]:
            ttk.Radiobutton(mf, text=label, value=val, variable=self.mosfet_var,
                            command=self._on_mosfet).pack(side="left", padx=4)

        # capture
        cap = ttk.LabelFrame(ctl, text="Capture", padding=4)
        cap.grid(row=1, column=5, padx=4, pady=4, sticky="w")
        ttk.Label(cap, text="ms:").pack(side="left")
        self.cap_ms_var = tk.StringVar(value="5")
        ttk.Entry(cap, width=5, textvariable=self.cap_ms_var).pack(side="left", padx=2)
        ttk.Button(cap, text="Capture", command=self._on_capture).pack(side="left", padx=2)
        self.save_btn = ttk.Button(cap, text="Save CSV", command=self._on_save_capture,
                                   state="disabled")
        self.save_btn.pack(side="left", padx=2)

        # charge lockout: while USB-powered the device suppresses normal alerts
        # and shows the charging blink.  These drive !ALERTS.
        cf = ttk.LabelFrame(ctl, text="Charge lockout", padding=4)
        cf.grid(row=2, column=0, columnspan=4, padx=4, pady=4, sticky="w")
        ttk.Button(cf, text="Re-enable LED alerts",
                   command=lambda: self._send("!ALERTS,1")).pack(side="left", padx=2)
        ttk.Button(cf, text="Restore charge blink",
                   command=lambda: self._send("!ALERTS,0")).pack(side="left", padx=2)

        # --- plots ---
        plots = ttk.Frame(self.diag_tab)
        plots.pack(fill="both", expand=True, padx=4, pady=4)

        self.live_fig = Figure(figsize=(5, 3), dpi=100)
        self.live_ax = self.live_fig.add_subplot(111)
        self.live_ax.set_title("Live stream")
        self.live_ax.set_xlabel("time (s)")
        self.live_ax.set_ylabel("volts")
        self.live_canvas = FigureCanvasTkAgg(self.live_fig, master=plots)
        self.live_canvas.get_tk_widget().pack(side="left", fill="both", expand=True)

        self.cap_fig = Figure(figsize=(5, 3), dpi=100)
        self.cap_ax = self.cap_fig.add_subplot(111)
        self.cap_ax.set_title("Last capture")
        self.cap_ax.set_xlabel("time from toggle (ms)")
        self.cap_ax.set_ylabel("volts")
        self.cap_canvas = FigureCanvasTkAgg(self.cap_fig, master=plots)
        self.cap_canvas.get_tk_widget().pack(side="left", fill="both", expand=True)

        self.read_lbl = ttk.Label(self.diag_tab, text="pos: --  neg: --  diff: --",
                                  anchor="w", font=("Consolas", 11))
        self.read_lbl.pack(fill="x", padx=4, pady=2)

    # --------------------------------------------------- configuration tab
    def _build_cfg_tab(self):
        bar = ttk.Frame(self.cfg_tab, padding=6)
        bar.pack(fill="x")
        ttk.Button(bar, text="Read all from device",
                   command=self._cfg_read_all).pack(side="left", padx=2)
        ttk.Button(bar, text="Save RAM -> EEPROM",
                   command=self._cfg_save).pack(side="left", padx=8)
        ttk.Button(bar, text="Reload EEPROM -> RAM",
                   command=self._cfg_load).pack(side="left", padx=2)
        ttk.Button(bar, text="Factory defaults",
                   command=self._cfg_defaults).pack(side="left", padx=8)
        ttk.Button(bar, text="Restore this unit...",
                   command=self._cfg_restore_this_unit).pack(side="left", padx=4)
        ttk.Button(bar, text="Copy from unit...",
                   command=self._cfg_copy_from_unit).pack(side="left", padx=2)
        self.dirty_lbl = ttk.Label(bar, text="")
        self.dirty_lbl.pack(side="right", padx=6)

        # --- unit identity row ---
        idbar = ttk.Frame(self.cfg_tab, padding=(6, 0))
        idbar.pack(fill="x")
        ttk.Label(idbar, text="Serial number:").pack(side="left")
        self.sn_cfg_lbl = ttk.Label(idbar, text="--", font=("Consolas", 10, "bold"),
                                    foreground="#06a")
        self.sn_cfg_lbl.pack(side="left", padx=(4, 8))
        ttk.Button(idbar, text="Write SN...",
                   command=self._write_sn_dialog).pack(side="left", padx=2)
        self.csv_path_lbl = ttk.Label(
            idbar, text=f"log: {self.units_csv_path}", foreground="#555")
        self.csv_path_lbl.pack(side="right", padx=6)

        hint = ("Edit a value and press Set (or Enter): the device applies it "
                "immediately, in RAM only.  'Save RAM -> EEPROM' makes the "
                "current RAM config permanent (survives power cycles) AND "
                "records this unit's full config to the master CSV, keyed by "
                "serial number.  Values outside the firmware's allowed range "
                "are clamped and echoed back.")
        ttk.Label(self.cfg_tab, text=hint, wraplength=980, foreground="#444",
                  padding=(8, 0)).pack(fill="x")

        # scrollable table: canvas + inner frame
        outer = ttk.Frame(self.cfg_tab)
        outer.pack(fill="both", expand=True, padx=6, pady=6)
        self.cfg_canvas = tk.Canvas(outer, highlightthickness=0)
        vsb = ttk.Scrollbar(outer, orient="vertical", command=self.cfg_canvas.yview)
        self.cfg_canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        self.cfg_canvas.pack(side="left", fill="both", expand=True)
        self.cfg_inner = ttk.Frame(self.cfg_canvas)
        self.cfg_inner_id = self.cfg_canvas.create_window(
            (0, 0), window=self.cfg_inner, anchor="nw")
        self.cfg_inner.bind(
            "<Configure>",
            lambda e: self.cfg_canvas.configure(scrollregion=self.cfg_canvas.bbox("all")))
        self.cfg_canvas.bind(
            "<Configure>",
            lambda e: self.cfg_canvas.itemconfigure(self.cfg_inner_id, width=e.width))
        # mouse-wheel scrolling while the pointer is over the table
        self.cfg_canvas.bind_all(
            "<MouseWheel>",
            lambda e: self._cfg_scroll(e), add="+")

        self.cfg_placeholder = ttk.Label(
            self.cfg_inner, foreground="#666", padding=10,
            text="Connect to a device -- the configuration table is built "
                 "from its !CFG dump.")
        self.cfg_placeholder.pack()

        self.cfg_next_row = 0
        self.cfg_groups_done = set()

    # --------------------------------------------------------- bench tab
    def _build_bench_tab(self):
        # --- pin map ---
        pf = ttk.LabelFrame(self.bench_tab, text="Pin map (!PINS)", padding=6)
        pf.pack(fill="x", padx=4, pady=4)

        bar = ttk.Frame(pf)
        bar.grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 4))
        ttk.Button(bar, text="Read pin map", command=lambda: self._send("!PINS")
                   ).pack(side="left")
        ttk.Label(bar, text="  Reassign:").pack(side="left", padx=(12, 2))
        self.pin_func_cb = ttk.Combobox(bar, width=26, state="readonly",
                                        values=[f"{f} -- {d}" for f, _, d in PIN_FUNC_KEYS])
        self.pin_func_cb.pack(side="left", padx=2)
        self.pin_to_cb = ttk.Combobox(bar, width=16, state="readonly", values=[])
        self.pin_to_cb.pack(side="left", padx=2)
        ttk.Button(bar, text="Set", command=self._bench_set_pin).pack(side="left", padx=4)
        ttk.Label(bar, text="(RAM only -- Save on the Configuration tab to keep it)"
                  ).pack(side="left", padx=6)

        self.pin_tree = ttk.Treeview(pf, columns=("pin", "name", "what"),
                                     show="headings", height=9)
        for col, txt, w in (("pin", "pin #", 60), ("name", "name", 80),
                            ("what", "function", 420)):
            self.pin_tree.heading(col, text=txt)
            self.pin_tree.column(col, width=w, anchor="w")
        self.pin_tree.grid(row=1, column=0, columnspan=4, sticky="ew")

        # --- gates ---
        gf = ttk.LabelFrame(self.bench_tab, text="Power gates (!GATE)", padding=6)
        gf.pack(fill="x", padx=4, pady=4)
        ttk.Label(gf, text="A hold overrides the configured MODE and stays in force through "
                           "sleep and !FLOOR, so it is what you set before unplugging to "
                           "measure a gating scheme. It is RAM only and clears on reset.",
                  wraplength=980, foreground="#555").grid(row=0, column=0, columnspan=6,
                                                          sticky="w", pady=(0, 6))
        self.gate_hold_vars = {}
        self.gate_lbls = {}
        for i, g in enumerate(GATE_NAMES):
            r = i + 1
            ttk.Label(gf, text=g, font=("Consolas", 10, "bold")).grid(row=r, column=0, padx=(2, 8))
            var = tk.StringVar(value="-1")
            self.gate_hold_vars[g] = var
            for j, (txt, val) in enumerate((("Auto", "-1"), ("Force on", "1"), ("Force off", "0"))):
                ttk.Radiobutton(gf, text=txt, variable=var, value=val,
                                command=lambda gg=g: self._bench_hold_gate(gg)
                                ).grid(row=r, column=1 + j, padx=2, sticky="w")
            lbl = ttk.Label(gf, text="not read yet", font=("Consolas", 9), foreground="#555")
            lbl.grid(row=r, column=4, padx=10, sticky="w")
            self.gate_lbls[g] = lbl
        ttk.Button(gf, text="Read gates", command=lambda: self._send("!GATE")
                   ).grid(row=len(GATE_NAMES) + 1, column=0, columnspan=2, pady=(6, 0), sticky="w")

        # --- two-stage sleep ---
        df = ttk.LabelFrame(self.bench_tab, text="Two-stage sleep (!DEEP)", padding=6)
        df.pack(fill="x", padx=4, pady=4)
        ttk.Label(df, text="Stage 1 is the normal sleeping mode. After DEEPSEC seconds of it "
                           "with every probe reading open, the board drops to stage 2 and the "
                           "RTC wake period itself slows to DEEPHZ. Forcing skips the countdown "
                           "so a meter reading can be taken now; it only works while the board "
                           "is already asleep, and any wake resets it to stage 1.",
                  wraplength=980, foreground="#555").pack(anchor="w", pady=(0, 6))
        drow = ttk.Frame(df)
        drow.pack(fill="x")
        ttk.Button(drow, text="Read schedule", command=lambda: self._send("!DEEP")
                   ).pack(side="left")
        ttk.Button(drow, text="Force stage 2", command=lambda: self._send("!DEEP,1")
                   ).pack(side="left", padx=4)
        ttk.Button(drow, text="Back to stage 1", command=lambda: self._send("!DEEP,0")
                   ).pack(side="left")
        self.deep_lbl = ttk.Label(df, text="not read yet", font=("Consolas", 9),
                                  foreground="#555")
        self.deep_lbl.pack(anchor="w", pady=(6, 0))

        # --- power profile ---
        ef = ttk.LabelFrame(self.bench_tab, text="Power profile (!EXPT)", padding=6)
        ef.pack(fill="both", expand=True, padx=4, pady=4)
        row = ttk.Frame(ef)
        row.pack(fill="x")
        ttk.Label(row, text="seconds per step:").pack(side="left")
        self.expt_sec = tk.StringVar(value="10")
        ttk.Spinbox(row, from_=1, to=600, width=6, textvariable=self.expt_sec
                    ).pack(side="left", padx=4)
        ttk.Button(row, text="Run", command=self._bench_run_expt).pack(side="left", padx=4)
        ttk.Button(row, text="Abort", command=lambda: self._send("!EXPT,0")).pack(side="left")
        ttk.Button(row, text="Save plan CSV...", command=self._bench_save_plan
                   ).pack(side="left", padx=12)
        self.expt_lbl = ttk.Label(row, text="idle", font=("Consolas", 10))
        self.expt_lbl.pack(side="left", padx=12)

        ttk.Label(ef, text="Ten steps: the ANA and LED gates swept while running, the same "
                           "sweep asleep in stage 1, then the best and worst of those repeated "
                           "in the deep stage. Arm it here, then unplug USB and let it run on "
                           "battery through the meter -- any serial byte aborts the run, so "
                           "replugging ends it. The deltas between steps are the answer; the "
                           "absolute number includes the breadboard. Note the deep steps take "
                           "their time from the deep tick, so the run is still eight-plus-two "
                           "equal-length steps.",
                  wraplength=980, foreground="#555").pack(anchor="w", pady=(6, 4))

        self.expt_text = tk.Text(ef, height=12, wrap="none", font=("Consolas", 9))
        self.expt_text.pack(fill="both", expand=True)

    def _bench_set_pin(self):
        sel = self.pin_func_cb.get()
        pin = self.pin_to_cb.get()
        if not sel or not pin:
            self._log("** pick a function and a pin first")
            return
        func = sel.split(" -- ")[0]
        key = dict((f, k) for f, k, _ in PIN_FUNC_KEYS).get(func)
        num = pin.split("(")[-1].rstrip(")")
        self._send(f"!SET,{key},{num}")
        self._send("!PINS")              # the device may have refused it
        self._send("!GATE")

    def _bench_hold_gate(self, gate):
        self._send(f"!GATE,{gate},{self.gate_hold_vars[gate].get()}")

    def _bench_run_expt(self):
        self.expt_plan = []
        self.expt_text.delete("1.0", "end")
        try:
            sec = int(self.expt_sec.get())
        except ValueError:
            self._log("** seconds per step must be a whole number")
            return
        self._send(f"!EXPT,{sec}")

    def _bench_save_plan(self):
        if not self.expt_plan:
            messagebox.showinfo("No plan", "Run !EXPT first -- the device prints the "
                                           "schedule and it is captured here.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", filetypes=[("CSV", "*.csv")],
            initialfile=f"expt_plan_{datetime.now():%Y%m%d_%H%M%S}.csv")
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["step", "name", "parked", "deep", "start_s", "end_s"])
            for idx, name, start, parked, deep in self.expt_plan:
                w.writerow([idx, name, parked, deep, start,
                            start + int(self.expt_sec.get() or 10)])
        self._log(f"** saved experiment plan to {path}")

    def _cfg_scroll(self, event):
        # only scroll when the Configuration tab is showing
        if self.nb.index(self.nb.select()) == 1:
            self.cfg_canvas.yview_scroll(int(-event.delta / 120), "units")

    # ----- config table construction -----
    def _cfg_group_of(self, key):
        return KEY_META.get(key, ("Other",))[0]

    def _cfg_add_group_header(self, group):
        hdr = ttk.Label(self.cfg_inner, text=group, font=("Segoe UI", 10, "bold"),
                        foreground="#0b6fb8")
        hdr.grid(row=self.cfg_next_row, column=0, columnspan=5,
                 sticky="w", padx=4, pady=(10, 2))
        self.cfg_next_row += 1

    def _cfg_add_row(self, key, value):
        """Create one table row for a key first seen in a device $CFG line."""
        if self.cfg_placeholder is not None:
            self.cfg_placeholder.destroy()
            self.cfg_placeholder = None

        group = self._cfg_group_of(key)
        if group not in self.cfg_groups_done:
            self.cfg_groups_done.add(group)
            self._cfg_add_group_header(group)

        meta = KEY_META.get(key)
        kind = meta[1] if meta else "num"
        desc = meta[2] if meta else ""

        r = self.cfg_next_row
        self.cfg_next_row += 1

        ttk.Label(self.cfg_inner, text=key, font=("Consolas", 10)).grid(
            row=r, column=0, sticky="w", padx=(16, 6), pady=1)
        dev_lbl = ttk.Label(self.cfg_inner, text=value, width=10,
                            font=("Consolas", 10), foreground="#06a")
        dev_lbl.grid(row=r, column=1, sticky="w", padx=6)

        var = tk.StringVar(value=value)
        if kind == "bool":
            editor = ttk.Combobox(self.cfg_inner, width=6, textvariable=var,
                                  values=["0", "1"], state="readonly")
        elif kind == "choice":
            choices = meta[3] if meta and len(meta) > 3 else ["0", "1"]
            editor = ttk.Combobox(self.cfg_inner, width=6, textvariable=var,
                                  values=choices, state="readonly")
        else:
            editor = ttk.Entry(self.cfg_inner, width=9, textvariable=var)
            editor.bind("<Return>", lambda e, k=key: self._cfg_set(k))
        editor.grid(row=r, column=2, sticky="w", padx=6)

        ttk.Button(self.cfg_inner, text="Set", width=4,
                   command=lambda k=key: self._cfg_set(k)).grid(
            row=r, column=3, sticky="w", padx=2)
        ttk.Label(self.cfg_inner, text=desc, foreground="#555").grid(
            row=r, column=4, sticky="w", padx=10)

        self.cfg_rows[key] = {"var": var, "dev_lbl": dev_lbl}

    def _cfg_update(self, key, value):
        """Apply a $CFG,<key>,<value> line to the table (create row if new)."""
        if key not in self.cfg_rows:
            self._cfg_add_row(key, value)
            return
        row = self.cfg_rows[key]
        row["dev_lbl"].config(text=value)
        row["var"].set(value)

    # ----- config commands -----
    def _cfg_read_all(self):
        self._send("!CFG")

    def _cfg_set(self, key):
        row = self.cfg_rows.get(key)
        if row is None:
            return
        val = row["var"].get().strip()
        try:
            float(val)
        except ValueError:
            self._log(f"** {key}: '{val}' is not a number")
            return
        self._send(f"!SET,{key},{val}")
        self._send("!STATUS")            # refresh the dirty flag display

    def _cfg_save(self):
        self._send("!SAVE")
        self._send("!STATUS")

    def _cfg_load(self):
        self._send("!LOAD")
        self._send("!CFG")               # re-sync the whole table
        self._send("!STATUS")

    def _cfg_defaults(self):
        if not messagebox.askyesno(
                "Factory defaults",
                "Load factory defaults into device RAM?\n\n"
                "The EEPROM is untouched until you press 'Save RAM -> EEPROM'."):
            return
        self._send("!DEFAULTS")
        self._send("!CFG")
        self._send("!STATUS")

    # ----- copy another unit's config onto this one -----
    def _read_units_csv(self):
        """Return {sn: rowdict} from the master per-unit log ({} if unusable)."""
        path = self.units_csv_path
        if not os.path.exists(path):
            return {}
        try:
            with open(path, newline="") as fh:
                return {(r.get("SN") or "").strip(): r
                        for r in csv.DictReader(fh)
                        if (r.get("SN") or "").strip()}
        except OSError as exc:
            messagebox.showerror("CSV log", f"Could not read\n{path}\n\n{exc}")
            return {}

    @staticmethod
    def _same_value(a, b):
        """Compare two config values numerically so 0.62 == 0.6200."""
        try:
            return abs(float(a) - float(b)) <= 1e-6
        except ValueError:
            return a == b

    def _copy_plan(self, donor, restore=False):
        """Work out what copying `donor`'s config onto this unit would do.

        `restore` = the donor row IS this unit, so the wiring keys are its own
        and get sent; only HWREV stays protected.

        Returns (changes, blocked, unknown, absent):
          changes  [(key, current, new)]  differing keys that will be sent
          blocked  [(key, value)]         protected -- deliberately kept
          unknown  [(key, value)]         donor has it, this firmware does not
          absent   [key]                  this unit has it, the donor row does not
        """
        protected = RESTORE_PROTECTED_KEYS if restore else NEVER_COPY_KEYS
        changes, blocked, unknown = [], [], []
        for key, raw in donor.items():
            if key in UNITS_CSV_FIXED:
                continue
            val = (raw or "").strip()
            if not val:
                continue
            if key in protected:
                blocked.append((key, val))
                continue
            row = self.cfg_rows.get(key)
            if row is None:
                unknown.append((key, val))
                continue
            cur = row["dev_lbl"].cget("text").strip()
            if not self._same_value(cur, val):
                changes.append((key, cur, val))
        absent = [k for k in self.cfg_rows
                  if k not in protected and not (donor.get(k) or "").strip()]
        return changes, blocked, unknown, absent

    def _cfg_copy_from_unit(self):
        self._cfg_copy_dialog(restore=False)

    def _cfg_restore_this_unit(self):
        """Re-send this unit's own last-saved row -- the wiring keys included.

        This is the path back from a CFG_VERSION bump that reseeded the EEPROM.
        The firmware migrates older layouts now, so it should be rare; it is
        still the only recovery when a config is lost some other way, and it is
        deliberately a separate button from 'Copy from unit...' so the two
        different safety rules cannot be confused for each other.
        """
        self._cfg_copy_dialog(restore=True)

    def _cfg_copy_dialog(self, restore=False):
        title = "Restore this unit" if restore else "Copy from unit"
        if not self.cfg_rows:
            messagebox.showinfo(
                title,
                "Connect to a device and read its config first -- the copy is "
                "computed against what this unit currently reports.")
            return
        units = self._read_units_csv()
        if restore:
            if not self.sn:
                messagebox.showinfo(
                    title,
                    "This unit has no serial number, so it has no row in the "
                    "log to restore from.\n\nThe log is keyed by SN -- assign "
                    "one with 'Write SN...' before the next save so this unit "
                    "can be recovered in future.")
                return
            donors = [sn for sn in units if sn == self.sn]
            if not donors:
                messagebox.showinfo(
                    title,
                    f"No saved row for SN '{self.sn}'.\n\n"
                    f"{self.units_csv_path}\n\n"
                    "A unit is recorded there when you press "
                    "'Save RAM -> EEPROM' while it has a serial number.")
                return
        else:
            donors = [sn for sn in units if sn != self.sn]
            if not donors:
                messagebox.showinfo(
                    title,
                    "No other units in the log yet.\n\n"
                    f"{self.units_csv_path}\n\n"
                    "A unit is recorded there when you press 'Save RAM -> EEPROM' "
                    "while it has a serial number assigned.\n\n"
                    "To put THIS unit back to its own last saved state, use "
                    "'Restore this unit...' instead.")
                return

        dlg = tk.Toplevel(self)
        dlg.title("Restore this unit from its own saved row" if restore
                  else "Copy config from another unit")
        dlg.transient(self)
        dlg.geometry("760x520")

        top = ttk.Frame(dlg, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text="Restore from:" if restore else "Copy from:").pack(side="left")
        donor_var = tk.StringVar(value=donors[0])
        donor_cb = ttk.Combobox(top, textvariable=donor_var, values=donors,
                                state="readonly", width=18)
        donor_cb.pack(side="left", padx=6)
        ttk.Label(top,
                  text=(f"back onto itself ({self.sn})" if restore
                        else f"onto this unit ({self.sn or 'unassigned SN'})"),
                  foreground="#555").pack(side="left")
        stamp_lbl = ttk.Label(top, text="", foreground="#555")
        stamp_lbl.pack(side="right")

        ttk.Label(dlg, padding=(8, 0), wraplength=730, foreground="#444",
                  text=("Values are written to device RAM only, exactly as if you "
                        "had typed each one. Nothing is permanent until you press "
                        "'Save RAM -> EEPROM' afterwards. " +
                        ("This row came off this same board, so the pin map and "
                         "gate polarities ARE restored. HWREV is still not -- if "
                         "the XIAO has moved to a different PCB since, a stale "
                         "HWREV is the one value that can do damage."
                         if restore else
                         "HWREV, the pin map and the gate polarities are never "
                         "copied between units -- they describe the board and "
                         "its wiring, not the tuning."))).pack(fill="x")

        table = ttk.Frame(dlg)
        table.pack(fill="both", expand=True, padx=8, pady=6)
        tree = ttk.Treeview(table, columns=("key", "cur", "new"),
                            show="headings", height=14)
        tree.heading("key", text="Key")
        tree.heading("cur", text="This unit")
        tree.heading("new", text="Donor")
        tree.column("key", width=150, anchor="w", stretch=False)
        tree.column("cur", width=300, anchor="w")
        tree.column("new", width=110, anchor="w", stretch=False)
        tsb = ttk.Scrollbar(table, orient="vertical", command=tree.yview)
        tree.configure(yscrollcommand=tsb.set)
        tsb.pack(side="right", fill="y")
        tree.pack(side="left", fill="both", expand=True)
        tree.tag_configure("note", foreground="#777")

        btns = ttk.Frame(dlg, padding=8)
        btns.pack(fill="x")
        summary = ttk.Label(btns, text="", foreground="#444")
        summary.pack(side="left")
        apply_btn = ttk.Button(btns, text="Apply to RAM")
        apply_btn.pack(side="right", padx=4)
        ttk.Button(btns, text="Cancel", command=dlg.destroy).pack(side="right")

        state = {"changes": []}

        def refresh(*_):
            donor = units.get(donor_var.get(), {})
            stamp_lbl.config(text=f"saved {donor.get('timestamp', '?')}")
            changes, blocked, unknown, absent = self._copy_plan(donor, restore)
            state["changes"] = changes
            tree.delete(*tree.get_children())
            for key, cur, new in changes:
                tree.insert("", "end", values=(key, cur, new))
            for key, val in blocked:
                tree.insert("", "end", tags=("note",),
                            values=(key, "-- not copied (describes the board) --", val))
            # A row written by older firmware has no column for keys added since;
            # those show up as "absent" and keep their defaults, which is right.
            for key, val in unknown:
                tree.insert("", "end", tags=("note",),
                            values=(key, "-- this firmware has no such key --", val))
            for key in absent:
                tree.insert("", "end", tags=("note",),
                            values=(key, "-- donor has no value, left alone --", ""))
            summary.config(
                text=f"{len(changes)} value(s) will change; "
                     f"{len(blocked)} protected, {len(unknown)} unknown, "
                     f"{len(absent)} absent from donor")
            apply_btn.config(state="normal" if changes else "disabled")

        def do_apply():
            changes = state["changes"]
            if not changes:
                return
            donor_sn = donor_var.get()
            if not messagebox.askyesno(
                    title,
                    f"Send {len(changes)} value(s) from SN '{donor_sn}' to this "
                    f"unit's RAM?\n\nThe EEPROM is untouched until you press "
                    f"'Save RAM -> EEPROM'.", parent=dlg):
                return
            dlg.destroy()
            self._log(f"** {'restoring' if restore else 'copying'} "
                      f"{len(changes)} value(s) from SN '{donor_sn}'")
            self._send_config_burst([f"!SET,{k},{v}" for k, _, v in changes])

        # Wired here, not at construction: do_apply is defined below the button.
        apply_btn.config(command=do_apply)
        donor_cb.bind("<<ComboboxSelected>>", refresh)
        refresh()          # fills the table and sets the button's enabled state
        dlg.grab_set()

    def _send_config_burst(self, cmds):
        """Send many !SET lines paced from the Tk event loop.

        The device drains its whole RX buffer each pass, but a pass only comes
        round every LOOPMS (50 ms default), so dumping ~60 lines at once leaves
        the host's USB write blocking on CDC back-pressure and the GUI frozen
        until the board catches up.  Trickling them keeps the UI alive and the
        board comfortably ahead.  Finishes with !CFG + !STATUS to re-sync the
        table from what the device actually accepted (values out of range are
        clamped, so the echo is the truth, not what we sent).
        """
        BATCH, PERIOD_MS = 3, 40
        queue_ = list(cmds)

        def pump():
            if not self.serial.is_open:
                self._log("** copy aborted: disconnected")
                return
            for cmd in queue_[:BATCH]:
                self.serial.send(cmd)
            del queue_[:BATCH]
            if queue_:
                self.after(PERIOD_MS, pump)
            else:
                self._send("!CFG")
                self._send("!STATUS")
                self._log("** copy complete -- review, then Save RAM -> EEPROM")

        pump()

    # ----- serial number + per-unit CSV log -----
    def _set_sn(self, sn):
        """Update the live SN and both on-screen labels."""
        self.sn = sn
        disp = sn if sn else "(unassigned)"
        self.sn_lbl.config(text=f"SN: {disp}")
        self.sn_cfg_lbl.config(text=disp)

    def _handle_sn(self, line):
        # $SN,<value>  (value may be empty when unassigned)
        parts = line.split(",", 1)
        self._set_sn(parts[1].strip() if len(parts) > 1 else "")

    def _write_sn(self, val):
        """Validate then push a new SN to the device (persists on the unit)."""
        val = (val or "").strip()
        if not val:
            return
        if "," in val:
            messagebox.showerror("Write SN", "Serial number cannot contain commas.")
            return
        if len(val) > 15:
            messagebox.showerror("Write SN", "Serial number too long (max 15 chars).")
            return
        self._send(f"!SN,{val}")
        self._send("!STATUS")

    def _write_sn_dialog(self):
        """Manual 'Write SN...' button: prompt, warning if overwriting."""
        if not self.serial.is_open:
            self._log("** not connected")
            return
        prompt = "Enter serial number to write (no commas, max 15 chars):"
        if self.sn:
            prompt = (f"This unit already has SN '{self.sn}'.\n\n"
                      "Writing will OVERWRITE it.\n\n" + prompt)
        val = simpledialog.askstring("Write serial number", prompt,
                                     parent=self, initialvalue=self.sn)
        if val is not None:
            self._write_sn(val)

    def _prompt_for_sn(self):
        """Auto-prompt fired once per connection when the unit has no SN."""
        if self.sn or not self.serial.is_open:
            return
        val = simpledialog.askstring(
            "Assign serial number",
            "This unit has no serial number.\n\nEnter one to write to the "
            "device now (no commas, max 15 chars), or Cancel to skip:",
            parent=self)
        if val is not None:
            self._write_sn(val)

    def _on_save_ok(self):
        """A !SAVE succeeded: re-sync the table, then log it to the master CSV."""
        if not self.sn:
            messagebox.showwarning(
                "No serial number",
                "Config was saved to EEPROM, but NOT logged to the CSV because "
                "this unit has no serial number.\n\nUse 'Write SN...' to assign "
                "one, then Save again.")
            return
        self.pending_csv_log = True     # _write_units_csv() runs on $CFGEND
        self._send("!CFG")              # fetch a fresh, complete snapshot

    def _write_units_csv(self):
        """Insert/replace this unit's row in the master per-unit config CSV."""
        sn = self.sn
        if not sn:
            return
        cfg_vals = {k: row["dev_lbl"].cget("text")
                    for k, row in self.cfg_rows.items()}
        ts = datetime.now().isoformat(timespec="seconds")
        path = self.units_csv_path

        # Read the existing log (preserving its column order and unit order).
        fieldnames = list(UNITS_CSV_FIXED)
        rows_by_sn = {}                 # dict keeps file order; new units append
        if os.path.exists(path):
            try:
                with open(path, newline="") as fh:
                    reader = csv.DictReader(fh)
                    if reader.fieldnames:
                        fieldnames = list(reader.fieldnames)
                    for row in reader:
                        key = (row.get("SN") or "").strip()
                        if key:
                            rows_by_sn[key] = row
            except OSError as exc:
                messagebox.showerror("CSV log", f"Could not read\n{path}\n\n{exc}")
                return

        # Guarantee the fixed columns lead, then append any new config keys.
        for i, col in enumerate(UNITS_CSV_FIXED):
            if col not in fieldnames:
                fieldnames.insert(i, col)
        for k in cfg_vals:
            if k not in fieldnames:
                fieldnames.append(k)

        # Update-or-insert this unit's row.
        row = rows_by_sn.get(sn, {})
        row["SN"] = sn
        row["timestamp"] = ts
        row.update(cfg_vals)
        rows_by_sn[sn] = row

        try:
            with open(path, "w", newline="") as fh:
                writer = csv.DictWriter(fh, fieldnames=fieldnames,
                                        extrasaction="ignore")
                writer.writeheader()
                for r in rows_by_sn.values():
                    writer.writerow(r)
        except OSError as exc:
            messagebox.showerror("CSV log", f"Could not write\n{path}\n\n{exc}")
            return
        self._log(f"** logged config for SN '{sn}' -> {path}")

    # ----------------------------------------------------------- serial
    def _ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def _refresh_ports(self):
        self.port_cb["values"] = self._ports()

    def _toggle_connect(self):
        if self.serial.is_open:
            self.serial.disconnect()
            self.conn_lbl.config(text="disconnected", foreground="#a00")
            self.connect_btn.config(text="Connect")
            self._set_sn("")
            return
        port = self.port_cb.get().strip()
        if not port:
            self._log("** select a port first")
            return
        try:
            self.serial.connect(port)
        except Exception as exc:
            self._log(f"** connect failed: {exc}")
            return
        self._reset_live()
        self._set_sn("")                 # cleared until the device reports it
        self.sn_prompted = False         # allow one auto-prompt this connection
        self.pending_csv_log = False
        self.conn_lbl.config(text=f"connected {port}", foreground="#0a0")
        self.connect_btn.config(text="Disconnect")
        self.serial.send("!STATUS")      # brings back sn= (may trigger prompt)
        self.serial.send("!CFG")         # populate the configuration table
        self.serial.send("!PINS")        # bench: pin map + this core's pin names
        self.serial.send("!GATE")        # bench: gate wiring and current state
        self.serial.send("!DEEP")        # bench: the two-stage sleep schedule

    def _reset_live(self):
        """Clear the rolling live-stream buffers and plot."""
        self.t0 = None
        self.live_t.clear()
        self.live_pos.clear()
        self.live_neg.clear()
        self.live_diff.clear()
        self.read_lbl.config(text="pos: --  neg: --  diff: --")
        self.live_ax.clear()
        self.live_ax.set_title("Live stream")
        self.live_ax.set_xlabel("time (s)")
        self.live_ax.set_ylabel("volts")
        self.live_canvas.draw_idle()

    def _send(self, cmd):
        if not self.serial.is_open:
            self._log("** not connected")
            return
        self.serial.send(cmd)
        self._log(f">> {cmd}")

    # --------------------------------------------------------- commands
    def _on_diag(self):
        self._send(f"!DIAG,{1 if self.diag_var.get() else 0}")

    def _on_stream(self):
        self._send(f"!STREAM,{1 if self.stream_var.get() else 0}")

    def _on_rate(self):
        self._send(f"!RATE,{self.rate_var.get().strip()}")

    def _on_vmode(self):
        self._send(f"!VMODE,{self.vmode_var.get()}")

    def _on_mosfet(self):
        self._send(f"!MOSFET,{self.mosfet_var.get()}")

    def _on_capture(self):
        self._send(f"!CAP,{self.cap_ms_var.get().strip()}")

    # ------------------------------------------------------------- loop
    def _tick(self):
        try:
            while True:
                kind, payload = self.line_queue.get_nowait()
                if kind == "__error__":
                    self._log(f"** serial error: {payload}")
                    self.serial.disconnect()
                    self.conn_lbl.config(text="disconnected", foreground="#a00")
                    self.connect_btn.config(text="Connect")
                else:
                    self._handle_line(payload)
        except queue.Empty:
            pass

        self._redraw_live()
        self.after(PLOT_REFRESH_MS, self._tick)

    def _handle_line(self, line):
        if line.startswith("$DIAG,"):
            self._handle_diag(line)
        elif line.startswith("$CFGEND"):
            self._log("<< config table synced")
            if self.pending_csv_log:        # follows a successful !SAVE
                self.pending_csv_log = False
                self._write_units_csv()
        elif line.startswith("$CFG,"):
            self._handle_cfg(line)
        elif line.startswith("$SN,"):
            self._handle_sn(line)
            self._log(f"<< {line}")
        elif line.startswith("$OK,save"):
            self._log(f"<< {line}")
            self._on_save_ok()
        elif line.startswith("$ERR,save"):
            self._log(f"<< {line}")
            messagebox.showerror(
                "Save failed",
                f"Device reported:\n{line}\n\nThe CSV log was not updated.")
        elif line.startswith("$DIP,"):
            self._handle_dip(line)
            self._log(f"<< {line}")
        elif line.startswith("$CAPSTART,"):
            self.cap_active = True
            self.cap_header = line
            self.cap_rows = []
        elif line.startswith("$CAP,"):
            if self.cap_active:
                self.cap_rows.append(line)
        elif line.startswith("$CAPEND"):
            self.cap_active = False
            self._finish_capture()
            self._log(f"<< capture: {len(self.cap_rows)} samples")
        elif line.startswith("$PINNAME,"):
            f = line.split(",")
            if len(f) >= 3:
                self.pin_names.append((f[1].strip(), f[2].strip()))
        elif line.startswith("$PIN,"):
            # $PIN,<func>,<num|none>,<name|-|fixed>
            f = line.split(",")
            if len(f) >= 4:
                self.pin_rows_pending[f[1].strip()] = (f[2].strip(), f[3].strip())
        elif line.startswith("$PINEND"):
            self._bench_apply_pins()
        elif line.startswith("$DEEP,"):
            self._handle_deep(line)
            self._log(f"<< {line}")
        elif line.startswith("$GATE,"):
            self._handle_gate(line)
            self._log(f"<< {line}")
        elif line.startswith("$EXPTPLAN,"):
            self._handle_expt_plan(line)
        elif line.startswith("$EXPTEND"):
            self.expt_running = False
            self.expt_lbl.config(text=line.split(",", 1)[-1], foreground="#555")
            self._bench_expt_log(line)
        elif line.startswith("$EXPT,"):
            self.expt_running = True
            f = line.split(",")
            if len(f) >= 3:
                self.expt_lbl.config(text=f"step {f[1]}: {f[2]}", foreground="#06a")
            self._bench_expt_log(f"{datetime.now():%H:%M:%S}  {line}")
        elif line.startswith("$STATUS,"):
            self._handle_status(line)
            self._log(f"<< {line}")
        else:
            self._log(f"<< {line}")

    # ------------------------------------------------------ bench handlers
    def _bench_expt_log(self, text):
        self.expt_text.insert("end", text + "\n")
        self.expt_text.see("end")

    def _bench_apply_pins(self):
        """A !PINS dump has finished: refresh the table and the pin dropdown."""
        self.pin_map = dict(self.pin_rows_pending)
        self.pin_rows_pending = {}
        descs = dict((f, d) for f, _, d in PIN_FUNC_KEYS)
        self.pin_tree.delete(*self.pin_tree.get_children())
        for func, (num, name) in self.pin_map.items():
            what = descs.get(func, "fixed on the XIAO module -- not remappable")
            self.pin_tree.insert("", "end", values=(num, name, f"{func}  --  {what}"))
        if self.pin_names:
            # Deduplicate by name but keep the aliases visible: on the XIAO an
            # analog and a digital name can share a pad, and knowing that is
            # half the point of reading this table.
            self.pin_to_cb["values"] = [f"{n} ({v})" for n, v in self.pin_names] + \
                                       ["none (255)"]
            self.pin_names = []
        self._log("<< pin map synced")

    def _handle_deep(self, line):
        # $DEEP,stage=..,deepsec=..,hz=..,tickms=..,ticks=..,probems=..,
        #       lightms=..,lightticks=..,lightprobems=..,forced=..
        kv = {}
        for tok in line.split(",")[1:]:
            if "=" in tok:
                k, v = tok.split("=", 1)
                kv[k.strip()] = v.strip()
        if not kv:
            return
        light = kv.get("lightprobems", "?")
        deep = kv.get("probems", "?")
        try:
            ratio = f"{int(deep) / int(light):.0f}x slower"
        except (ValueError, ZeroDivisionError):
            ratio = "?"
        # DEEPPARK defaults to "follow SLEEPPARK", so the key alone does not say
        # what the pin is doing -- the firmware resolves it and sends both.
        follow = "  (follows SLEEPPARK)" if kv.get("deeppark") == "2" else ""
        self.deep_lbl.config(
            text=(f"stage {kv.get('stage','?')}"
                  f"{'  (forced)' if kv.get('forced') == '1' else ''}"
                  f"   bridge now: {kv.get('bridgenow','?')}   |   "
                  f"stage 1: probe every {light} ms "
                  f"({kv.get('lightms','?')} ms tick x {kv.get('lightticks','?')}), "
                  f"bridge {kv.get('lightbridge','?')}   |   "
                  f"stage 2 after {kv.get('deepsec','?')} s: probe every {deep} ms "
                  f"({kv.get('tickms','?')} ms tick x {kv.get('ticks','?')}) "
                  f"= {kv.get('hz','?')} Hz, {ratio}, "
                  f"bridge {kv.get('deepbridge','?')}{follow}"),
            foreground="#06a")

    def _handle_gate(self, line):
        # $GATE,<NAME>,pin=..,pol=..,mode=..,settleus=..,force=..,state=..
        f = line.split(",")
        if len(f) < 3:
            return
        name = f[1].strip()
        kv = {}
        for tok in f[2:]:
            if "=" in tok:
                k, v = tok.split("=", 1)
                kv[k.strip()] = v.strip()
        self.gate_info[name] = kv
        if name in self.gate_lbls:
            self.gate_lbls[name].config(
                text=(f"pin={kv.get('pin','?'):<5} pol={kv.get('pol','?')} "
                      f"mode={kv.get('mode','?')} settle={kv.get('settleus','?')}us "
                      f"-> now {'ON' if kv.get('state') == '1' else 'off'}"),
                foreground="#0a0" if kv.get("state") == "1" else "#555")
        # Reflect the device's own idea of the hold, so the radio buttons stay
        # honest when an experiment step moves them.
        if name in self.gate_hold_vars and "force" in kv:
            self.gate_hold_vars[name].set(kv["force"])

    def _handle_expt_plan(self, line):
        # $EXPTPLAN,steps=..,sec=..,totalsec=..   or
        # $EXPTPLAN,<i>,<name>,start=<s>,parked=<0|1>
        self._bench_expt_log(line)
        f = line.split(",")
        if len(f) >= 5 and f[1].strip().isdigit():
            try:
                start = int(f[3].split("=")[1])
                parked = int(f[4].split("=")[1])
            except (IndexError, ValueError):
                return
            # deep= was added with the second sleep stage; tolerate its absence
            # so this GUI still reads a plan from an older bench build.
            try:
                deep = int(f[5].split("=")[1])
            except (IndexError, ValueError):
                deep = 0
            self.expt_plan.append((int(f[1]), f[2].strip(), start, parked, deep))

    def _handle_cfg(self, line):
        # $CFG,<key>,<value>
        f = line.split(",")
        if len(f) >= 3:
            self._cfg_update(f[1].strip(), f[2].strip())

    def _thr_unit(self):
        # Threshold units depend on the active detection method.
        return {0: "V", 1: "ms", 2: "V*ms"}.get(self.detmethod, "V")

    def _handle_dip(self, line):
        # $DIP,<idx>,<threshV>  -- live threshold-position change.
        # HWREV 2: someone moved a DIP switch.  HWREV 3: THRESHSEL was set.
        f = line.split(",")
        try:
            idx = int(f[1])
            thr = float(f[2])
        except (ValueError, IndexError):
            return
        self.dip_lbl.config(text=f"THR POS {idx:02b} -> {thr:.3f} {self._thr_unit()}",
                            foreground="#06a")

    def _handle_diag(self, line):
        # $DIAG,<ms>,<rawPos>,<rawNeg>,<posV>,<negV>,<diffV>
        f = line.split(",")
        try:
            ms = int(f[1])
            pv, nv, dv = float(f[4]), float(f[5]), float(f[6])
        except (ValueError, IndexError):
            return
        if self.t0 is None:
            self.t0 = ms
        t = (ms - self.t0) / 1000.0
        self.live_t.append(t)
        self.live_pos.append(pv)
        self.live_neg.append(nv)
        self.live_diff.append(dv)
        self.read_lbl.config(
            text=f"pos: {pv:+.4f} V   neg: {nv:+.4f} V   diff: {dv:+.4f} V")

    def _handle_status(self, line):
        # reflect device state back into the controls without re-sending
        kv = {}
        for tok in line.split(",")[1:]:
            if "=" in tok:
                k, v = tok.split("=", 1)
                kv[k] = v
        self.status_lbl.config(text="status: " + "  ".join(
            f"{k}={v}" for k, v in kv.items()))
        self._bench_status(kv)          # bench: gates= / gforce= / expt=
        # A charge readout that says "charging" while the board is happily
        # sleeping is confusing, so name the state rather than the rail.
        self.chg_inhibit = (kv.get("chginhibit", "1") == "1")
        try:
            if "diag" in kv:
                self.diag_var.set(kv["diag"] == "1")
            if "stream" in kv:
                self.stream_var.set(kv["stream"] == "1")
            if "vmode" in kv:
                self.vmode_var.set(int(kv["vmode"]))
            if "mosfet" in kv:
                self.mosfet_var.set(int(kv["mosfet"]))
            if "rate" in kv:
                self.rate_var.set(kv["rate"])
            if "capms" in kv:
                self.cap_ms_var.set(kv["capms"])
            if "detmethod" in kv:
                self.detmethod = int(kv["detmethod"])
        except ValueError:
            pass

        # unit serial number (may be empty). Auto-prompt once per connection
        # when unassigned, deferred so this queue-processing pass finishes first.
        if "sn" in kv:
            self._set_sn(kv["sn"])
            if not kv["sn"] and self.serial.is_open and not self.sn_prompted:
                self.sn_prompted = True
                self.after(150, self._prompt_for_sn)

        # Threshold position + active threshold (units follow the detection
        # method).  Still reported as "dip=" for both board revisions.
        if "dip" in kv:
            try:
                idx = int(kv["dip"])
                thr = float(kv.get("openthr", "nan"))
                self.dip_lbl.config(
                    text=f"THR POS {idx:02b} -> {thr:.3f} {self._thr_unit()}",
                    foreground="#06a")
            except ValueError:
                pass

        # unsaved-changes flag (device RAM vs EEPROM)
        if "dirty" in kv:
            if kv["dirty"] == "1":
                self.dirty_lbl.config(text="unsaved changes (RAM only)",
                                      foreground="#a60")
            else:
                self.dirty_lbl.config(text="config saved", foreground="#0a0")

        # charge lockout indicator: USB-power state + alert override
        if "charge" in kv:
            charging = kv["charge"] == "1"
            overridden = kv.get("alertovr") == "1"
            if not charging:
                txt, col = "charge: on battery", "#0a0"
            elif not self.chg_inhibit:
                # CHGINHIBIT=0: the rail is there and reported, but the board is
                # behaving as if on battery. Saying "USB (blink)" here would be
                # a lie -- it is neither blinking nor locked out.
                txt, col = "charge: 5V in (ignored)", "#06a"
            elif overridden:
                txt, col = "charge: USB (alerts on)", "#a60"
            else:
                txt, col = "charge: USB (blink)", "#a00"
            self.charge_lbl.config(text=txt, foreground=col)

        # battery state-of-charge
        if "battpct" in kv:
            pct = kv["battpct"]
            vtxt = f" ({kv['battv']}V)" if "battv" in kv else ""
            self.batt_lbl.config(text=f"batt: {pct}%{vtxt}", foreground="#0a0")

    # ------------------------------------------------------------ plots
    def _bench_status(self, kv):
        """Bench fields out of $STATUS: gate levels, holds, experiment step."""
        gates = kv.get("gates", "")
        force = kv.get("gforce", "")
        if len(gates) == len(GATE_NAMES):
            parts = []
            for i, g in enumerate(GATE_NAMES):
                held = force[i] if len(force) == len(GATE_NAMES) else "a"
                parts.append(f"{g}={'ON' if gates[i] == '1' else 'off'}"
                             f"{'' if held == 'a' else '*'}")
            self.gates_lbl.config(text="gates: " + " ".join(parts))
        stage = kv.get("lpstage")
        if stage is not None:
            txt, colour = {"0": ("awake", "#555"),
                           "1": ("light", "#06a"),
                           "2": ("DEEP", "#0a0")}.get(stage, (stage, "#555"))
            self.sleep_lbl.config(text=f"sleep: {txt}", foreground=colour)
        step = kv.get("expt")
        if step is not None and step != "-1":
            self.expt_running = True
        elif step == "-1" and self.expt_running:
            self.expt_running = False
            self.expt_lbl.config(text="idle", foreground="#555")

    def _redraw_live(self):
        if not self.live_t:
            return
        self.live_ax.clear()
        self.live_ax.set_title("Live stream")
        self.live_ax.set_xlabel("time (s)")
        self.live_ax.set_ylabel("volts")
        self.live_ax.plot(self.live_t, self.live_pos, label="pos", lw=0.9)
        self.live_ax.plot(self.live_t, self.live_neg, label="neg", lw=0.9)
        self.live_ax.plot(self.live_t, self.live_diff, label="diff", lw=1.2)
        self.live_ax.legend(loc="upper left", fontsize=8)
        self.live_ax.grid(True, alpha=0.3)
        self.live_canvas.draw_idle()

    @staticmethod
    def _gate_names(mask):
        """Gate bit mask -> names. Bit order matches the firmware's GATE_* enum."""
        names = [n for i, n in enumerate(GATE_NAMES) if mask & (1 << i)]
        return "+".join(names) if names else "?"

    def _finish_capture(self):
        if not self.cap_header:
            return
        h = self.cap_header.split(",")
        try:
            toggle_us = float(h[2])
            full_scale = float(h[4])
            vref = float(h[5])
        except (IndexError, ValueError):
            return
        # Appended by the bench firmware when a pulsed gate had to be raised for
        # the capture; absent on older builds, hence the tolerant parse.
        # gate_mask 0 means nothing was gated and gate_us is meaningless.
        try:
            gate_us = float(h[6])
            gate_mask = int(h[7])
            gate_settle_us = float(h[8])
        except (IndexError, ValueError):
            gate_us, gate_mask, gate_settle_us = 0.0, 0, 0.0

        # rows: $CAP,<t_us>,<rawPos>,<rawNeg>
        rows = []          # full per-sample records for CSV export
        t_ms, pos_v, neg_v, diff_v = [], [], [], []
        for row in self.cap_rows:
            f = row.split(",")
            try:
                t = float(f[1])
                rp = int(f[2])
                rn = int(f[3])
            except (ValueError, IndexError):
                continue
            pv = rp / full_scale * vref
            nv = rn / full_scale * vref
            dv = pv - nv
            t_rel = (t - toggle_us) / 1000.0
            # Was the gated rail up when this sample was taken?  1 for every
            # sample of an ungated capture, so the column always means the same
            # thing when the CSV is read back.
            rail_up = 1 if (not gate_mask or t >= gate_us) else 0
            rows.append((t, t_rel, rp, rn, pv, nv, dv, rail_up))
            t_ms.append(t_rel)
            pos_v.append(pv)
            neg_v.append(nv)
            diff_v.append(dv)

        # retain for CSV export and enable the save button
        self.last_capture = {"toggle_us": toggle_us, "rows": rows,
                             "gate_us": gate_us, "gate_mask": gate_mask,
                             "gate_settle_us": gate_settle_us}
        self.save_btn.config(state=("normal" if rows else "disabled"))

        self.cap_ax.clear()
        self.cap_ax.set_xlabel("time from toggle (ms)")
        self.cap_ax.set_ylabel("volts")
        if t_ms:
            self.cap_ax.plot(t_ms, pos_v, ".-", ms=2, lw=0.8, label="pos")
            self.cap_ax.plot(t_ms, neg_v, ".-", ms=2, lw=0.8, label="neg")
            self.cap_ax.plot(t_ms, diff_v, ".-", ms=2, lw=1.0, label="diff")
            if gate_mask:
                # Everything left of the gate edge was sampled with the rail
                # DOWN -- it is not signal, and shading it stops it being read
                # as one. The span between the edge and the toggle is the
                # settle: if the trace is still moving when it ends, ANAUS is
                # too short.
                gate_ms = (gate_us - toggle_us) / 1000.0
                self.cap_ax.axvspan(min(t_ms), gate_ms, color="#c00", alpha=0.07)
                self.cap_ax.axvline(gate_ms, color="#c00", ls=":", lw=1.2,
                                    label=f"{self._gate_names(gate_mask)} rail up")
                self.cap_ax.axvspan(gate_ms, 0.0, color="#0a0", alpha=0.06)
            self.cap_ax.axvline(0.0, color="k", ls="--", lw=0.8, label="toggle")
            self.cap_ax.legend(loc="upper right", fontsize=8)
            self.cap_ax.grid(True, alpha=0.3)
        title = "Last capture"
        if gate_mask:
            title += (f"   --   {self._gate_names(gate_mask)} gated, "
                      f"settle {gate_settle_us / 1000.0:.2f} ms (ANAUS)")
        self.cap_ax.set_title(title)
        self.cap_canvas.draw_idle()

    def _on_save_capture(self):
        if not self.last_capture or not self.last_capture["rows"]:
            messagebox.showinfo("Save capture", "No capture to save yet.")
            return
        path = filedialog.asksaveasfilename(
            title="Save capture CSV",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialfile="capture.csv",
        )
        if not path:
            return
        try:
            with open(path, "w", newline="") as fh:
                w = csv.writer(fh)
                # rail_up: 0 for samples taken before a gated analog rail came
                # up. Always 1 for an ungated capture, so the column means the
                # same thing in every file.
                w.writerow(["t_us", "t_ms_from_toggle",
                            "rawPos", "rawNeg", "posV", "negV", "diffV",
                            "rail_up"])
                for (t, t_rel, rp, rn, pv, nv, dv, up) in self.last_capture["rows"]:
                    w.writerow([f"{t:.0f}", f"{t_rel:.4f}", rp, rn,
                                f"{pv:.6f}", f"{nv:.6f}", f"{dv:.6f}", up])
        except OSError as exc:
            messagebox.showerror("Save capture", f"Could not write file:\n{exc}")
            return
        self._log(f"** saved capture -> {path}")

    # -------------------------------------------------------------- misc
    def _log(self, text):
        self.log.insert("end", text + "\n")
        self.log.see("end")
        # keep the log bounded
        if int(self.log.index("end-1c").split(".")[0]) > 500:
            self.log.delete("1.0", "100.0")

    def _on_close(self):
        self.serial.disconnect()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
