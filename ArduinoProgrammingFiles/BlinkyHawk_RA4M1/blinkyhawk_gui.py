#!/usr/bin/env python3
"""
blinkyhawk_gui.py  --  Host GUI for the BlinkyHawk_RA4M1 firmware.

Talks to the Blinky Hawk (Seeed XIAO RA4M1) over USB serial (115200 baud).
Two tabs:

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
    - unknown keys reported by newer firmware still appear (in "Other"), so
      this GUI does not need updating for every firmware tweak

The top bar always shows charge/battery state and the live DIP-switch
threshold position ($DIP lines).

Descended from OpenLeadDetect_XIAO_Minimal/diagnostic_gui.py with the A5
potentiometer features (calibration sweep, offline battery log, !THRESH,
!NEGFIX) removed -- NEGFIX/NEGV are plain EEPROM keys now.

Dependencies:
    pip install pyserial matplotlib

Run:
    python blinkyhawk_gui.py
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
                         "blinkyhawk_units.csv")
UNITS_CSV_FIXED = ["SN", "timestamp"]   # leading columns; config keys follow

# ---------------------------------------------------------------------------
# Config-key metadata: display grouping, label, and a short description.
# The table itself is built from whatever the DEVICE reports in its !CFG dump,
# so a key missing here still shows up (under "Other") -- this dict only makes
# known keys prettier.  kind "bool" renders as a 0/1 dropdown; kind "choice"
# renders as a dropdown of the values in the optional 4th tuple element.
# ---------------------------------------------------------------------------
KEY_META = {
    # --- Detection ---
    "REFCENTER":    ("Detection", "num",  "Resting differential centre (V)"),
    "REFBAND":      ("Detection", "num",  "No-voltage window half-width (V)"),
    # THRESH units follow DETMETHOD: V (single) / ms (time-to-return) / V*ms (area).
    "THRESH00":     ("Detection", "num",  "Threshold, DIP 00 = both switches ON (units per DETMETHOD)"),
    "THRESH01":     ("Detection", "num",  "Threshold, DIP 01 = D8 ON, D10 OFF (units per DETMETHOD)"),
    "THRESH10":     ("Detection", "num",  "Threshold, DIP 10 = D8 OFF, D10 ON (units per DETMETHOD)"),
    "THRESH11":     ("Detection", "num",  "Threshold, DIP 11 = both switches OFF (units per DETMETHOD)"),
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
    "CONTFREQ":     ("Alerts", "num",  "Continuity beep pitch (Hz, passive only)"),
    "VOLTFREQ":     ("Alerts", "num",  "Voltage beep pitch (Hz, passive only)"),
    "CONTPULSES":   ("Alerts", "num",  "Pulses per continuity beep"),
    "VOLTPULSES":   ("Alerts", "num",  "Pulses per voltage beep"),
    "CONTREP":      ("Alerts", "bool", "Re-beep while continuity holds"),
    "VOLTREP":      ("Alerts", "bool", "Re-beep while voltage persists"),
    "CONTREPMS":    ("Alerts", "num",  "Continuity repeat period (ms)"),
    "VOLTREPMS":    ("Alerts", "num",  "Voltage repeat period (ms)"),
    "BEEPMIN":      ("Alerts", "num",  "Min gap between beep sequences (ms)"),
    # --- Power / battery ---
    "CHGTHRESH":    ("Power / battery", "num", "VBUS/2 level meaning 'charging' (V)"),
    "BATTEMPTY":    ("Power / battery", "num", "Battery voltage mapped to 0% (V)"),
    "BATTFULL":     ("Power / battery", "num", "Battery voltage mapped to 100% (V)"),
    "BATTFULLPCT":  ("Power / battery", "num", "Charge % at which blink turns green"),
    # --- Misc ---
    "LOOPMS":       ("Misc", "num", "Main-loop pacing / sleep (ms)"),
}
GROUP_ORDER = ["Detection", "Alerts", "Power / battery", "Misc", "Other"]


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
        self.title("Blinky Hawk -- Diagnostics & Configuration")
        self.geometry("1024x800")

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

        self.dip_lbl = ttk.Label(top, text="DIP: --", font=("Consolas", 10))
        self.dip_lbl.pack(side="right", padx=6)
        self.batt_lbl = ttk.Label(top, text="batt: --")
        self.batt_lbl.pack(side="right", padx=6)
        self.charge_lbl = ttk.Label(top, text="charge: --")
        self.charge_lbl.pack(side="right", padx=6)

        # --- notebook: Diagnostics | Configuration ---
        self.nb = ttk.Notebook(self)
        self.nb.pack(fill="both", expand=True, padx=6, pady=4)
        self.diag_tab = ttk.Frame(self.nb)
        self.cfg_tab = ttk.Frame(self.nb)
        self.nb.add(self.diag_tab, text="  Diagnostics  ")
        self.nb.add(self.cfg_tab, text="  Configuration (EEPROM)  ")
        self._build_diag_tab()
        self._build_cfg_tab()

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
        elif line.startswith("$STATUS,"):
            self._handle_status(line)
            self._log(f"<< {line}")
        else:
            self._log(f"<< {line}")

    def _handle_cfg(self, line):
        # $CFG,<key>,<value>
        f = line.split(",")
        if len(f) >= 3:
            self._cfg_update(f[1].strip(), f[2].strip())

    def _thr_unit(self):
        # Threshold units depend on the active detection method.
        return {0: "V", 1: "ms", 2: "V*ms"}.get(self.detmethod, "V")

    def _handle_dip(self, line):
        # $DIP,<idx>,<threshV>  (live DIP-switch change)
        f = line.split(",")
        try:
            idx = int(f[1])
            thr = float(f[2])
        except (ValueError, IndexError):
            return
        self.dip_lbl.config(text=f"DIP: {idx:02b} -> {thr:.3f} {self._thr_unit()}",
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

        # DIP position + active threshold (units follow the detection method)
        if "dip" in kv:
            try:
                idx = int(kv["dip"])
                thr = float(kv.get("openthr", "nan"))
                self.dip_lbl.config(
                    text=f"DIP: {idx:02b} -> {thr:.3f} {self._thr_unit()}",
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
            rows.append((t, t_rel, rp, rn, pv, nv, dv))
            t_ms.append(t_rel)
            pos_v.append(pv)
            neg_v.append(nv)
            diff_v.append(dv)

        # retain for CSV export and enable the save button
        self.last_capture = {"toggle_us": toggle_us, "rows": rows}
        self.save_btn.config(state=("normal" if rows else "disabled"))

        self.cap_ax.clear()
        self.cap_ax.set_title("Last capture")
        self.cap_ax.set_xlabel("time from toggle (ms)")
        self.cap_ax.set_ylabel("volts")
        if t_ms:
            self.cap_ax.plot(t_ms, pos_v, ".-", ms=2, lw=0.8, label="pos")
            self.cap_ax.plot(t_ms, neg_v, ".-", ms=2, lw=0.8, label="neg")
            self.cap_ax.plot(t_ms, diff_v, ".-", ms=2, lw=1.0, label="diff")
            self.cap_ax.axvline(0.0, color="k", ls="--", lw=0.8, label="toggle")
            self.cap_ax.legend(loc="upper right", fontsize=8)
            self.cap_ax.grid(True, alpha=0.3)
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
                w.writerow(["t_us", "t_ms_from_toggle",
                            "rawPos", "rawNeg", "posV", "negV", "diffV"])
                for (t, t_rel, rp, rn, pv, nv, dv) in self.last_capture["rows"]:
                    w.writerow([f"{t:.0f}", f"{t_rel:.4f}", rp, rn,
                                f"{pv:.6f}", f"{nv:.6f}", f"{dv:.6f}"])
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
