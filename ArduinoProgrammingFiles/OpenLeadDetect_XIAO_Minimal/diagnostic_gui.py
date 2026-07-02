#!/usr/bin/env python3
"""
diagnostic_gui.py  --  Host GUI for OpenLeadDetect_XIAO_Minimal diagnostic mode.

Talks to the RA4M1 / XIAO open-lead detector over USB serial (115200 baud).
Lets you:
  - enter/exit diagnostic mode
  - lock voltage mode ON or disable it
  - park the MOSFET (auto / hold off / hold on)
  - set the open/closed differential threshold (CFG_OPEN_THRESH_V) live
    (shown read-only as "A5 auto" when the firmware derives it from A5)
  - pin the negative input (A2) to a fixed voltage to isolate board noise
  - re-enable the normal LED alerts while charging (RA4M1 charge lockout)
  - show battery presence and state-of-charge percentage (RA4M1)
  - stream both ADC pins independently plus the computed differential
  - trigger a transient capture across a MOSFET toggle and plot the result
  - "Calibrate A5..." window: sweep the pot and plot A5 voltage vs the
    post-toggle differential for a known threshold resistor (builds
    the firmware's A5_THRESH_TABLE)

Dependencies:
    pip install pyserial matplotlib

Run:
    python diagnostic_gui.py
"""

import csv
import queue
import threading
import tkinter as tk
from collections import deque
from tkinter import filedialog, messagebox, ttk

import serial
import serial.tools.list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

BAUD = 115200
LIVE_MAXLEN = 600          # samples kept in the rolling live plot
PLOT_REFRESH_MS = 80       # GUI redraw cadence


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
        self.title("OpenLeadDetect XIAO -- Diagnostics")
        self.geometry("1000x760")

        self.line_queue = queue.Queue()
        self.serial = SerialManager(self.line_queue)

        # live rolling data
        self.t0 = None
        self.has_se = True         # device sends separate single-ended channels?
        self.live_t = deque(maxlen=LIVE_MAXLEN)
        self.live_pos = deque(maxlen=LIVE_MAXLEN)
        self.live_neg = deque(maxlen=LIVE_MAXLEN)
        self.live_diff = deque(maxlen=LIVE_MAXLEN)

        # capture assembly
        self.cap_active = False
        self.cap_header = None
        self.cap_rows = []
        self.last_capture = None   # parsed rows of the most recent capture

        self.cal_win = None        # open CalibrationWindow, if any

        self._build_ui()
        self.after(PLOT_REFRESH_MS, self._tick)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------------------------------------------------------- UI
    def _build_ui(self):
        # --- connection row ---
        top = ttk.Frame(self, padding=6)
        top.pack(fill="x")
        ttk.Label(top, text="Port:").pack(side="left")
        self.port_cb = ttk.Combobox(top, width=22, values=self._ports())
        self.port_cb.pack(side="left", padx=4)
        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side="left")
        self.connect_btn = ttk.Button(top, text="Connect", command=self._toggle_connect)
        self.connect_btn.pack(side="left", padx=4)
        self.conn_lbl = ttk.Label(top, text="disconnected", foreground="#a00")
        self.conn_lbl.pack(side="left", padx=8)
        ttk.Button(top, text="Calibrate A5…",
                   command=self._open_calibration).pack(side="right", padx=4)

        # --- controls row ---
        ctl = ttk.LabelFrame(self, text="Controls", padding=6)
        ctl.pack(fill="x", padx=6, pady=4)

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

        # fixed-neg override: pin A2 to a constant so the diff isolates A0 /
        # board noise from A2-reference noise (sends !NEGFIX).
        nf = ttk.LabelFrame(ctl, text="Fixed neg (A2)", padding=4)
        nf.grid(row=2, column=0, columnspan=3, padx=4, pady=4, sticky="w")
        self.negfix_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(nf, text="Pin A2", variable=self.negfix_var,
                        command=self._on_negfix).pack(side="left", padx=4)
        ttk.Label(nf, text="V:").pack(side="left")
        self.negv_var = tk.StringVar(value="1.250")
        ttk.Entry(nf, width=7, textvariable=self.negv_var).pack(side="left", padx=2)
        ttk.Button(nf, text="Set", command=self._on_negv).pack(side="left", padx=2)

        # charge lockout (RA4M1): when USB-powered the device suppresses the
        # normal alerts and shows a slow dim-red blink.  These buttons drive
        # !ALERTS (re-enable normal alerts / restore the charging blink).
        cf = ttk.LabelFrame(ctl, text="Charge (RA4M1)", padding=4)
        cf.grid(row=2, column=3, columnspan=3, padx=4, pady=4, sticky="w")
        self.charge_lbl = ttk.Label(cf, text="charge: --", width=18)
        self.charge_lbl.pack(side="left", padx=4)
        self.batt_lbl = ttk.Label(cf, text="batt: --", width=18)
        self.batt_lbl.pack(side="left", padx=4)
        ttk.Button(cf, text="Re-enable LED alerts",
                   command=self._on_alerts_on).pack(side="left", padx=2)
        ttk.Button(cf, text="Restore charge blink",
                   command=self._on_alerts_off).pack(side="left", padx=2)

        # open/closed differential threshold (CFG_OPEN_THRESH_V): set the
        # voltage below which |A0-A2| reads as open vs at/above as closed.
        # Sends !THRESH,<v>; the device echoes it back as openthr= in $STATUS.
        th = ttk.LabelFrame(ctl, text="Open threshold", padding=4)
        th.grid(row=3, column=0, columnspan=3, padx=4, pady=4, sticky="w")
        ttk.Label(th, text="V:").pack(side="left")
        self.thresh_var = tk.StringVar(value="0.250")
        self.thresh_entry = ttk.Entry(th, width=7, textvariable=self.thresh_var)
        self.thresh_entry.pack(side="left", padx=2)
        self.thresh_set_btn = ttk.Button(th, text="Set", command=self._on_thresh)
        self.thresh_set_btn.pack(side="left", padx=2)
        # shown when the firmware derives the threshold from A5 (CFG_A5_THRESH):
        # openthr becomes read-only and tracks A5 live.
        self.thresh_mode_lbl = ttk.Label(th, text="")
        self.thresh_mode_lbl.pack(side="left", padx=4)

        # --- plots ---
        plots = ttk.Frame(self)
        plots.pack(fill="both", expand=True, padx=6, pady=4)

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

        # --- readouts + log ---
        self.status_lbl = ttk.Label(self, text="status: --", anchor="w",
                                    font=("Consolas", 9))
        self.status_lbl.pack(fill="x", padx=6)
        self.read_lbl = ttk.Label(self, text="pos: --  neg: --  diff: --",
                                  anchor="w", font=("Consolas", 11))
        self.read_lbl.pack(fill="x", padx=6, pady=2)

        logf = ttk.LabelFrame(self, text="Serial log", padding=4)
        logf.pack(fill="both", padx=6, pady=4)
        self.log = tk.Text(logf, height=8, wrap="none", font=("Consolas", 8))
        self.log.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(logf, command=self.log.yview)
        sb.pack(side="right", fill="y")
        self.log["yscrollcommand"] = sb.set

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
        self.conn_lbl.config(text=f"connected {port}", foreground="#0a0")
        self.connect_btn.config(text="Disconnect")
        self.serial.send("!STATUS")

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

    def _negv_str(self):
        # always emit a decimal point so the firmware parses arg as a voltage
        # (and thus sets the value) rather than as an on/off flag.
        try:
            return f"{float(self.negv_var.get().strip()):.3f}"
        except ValueError:
            return "1.250"

    def _on_negfix(self):
        # checkbox: ON sends the current voltage (sets value + enables),
        # OFF sends the disable flag.
        if self.negfix_var.get():
            self._send(f"!NEGFIX,{self._negv_str()}")
        else:
            self._send("!NEGFIX,0")

    def _on_negv(self):
        # Set button: push the voltage and enable the override.
        self.negfix_var.set(True)
        self._send(f"!NEGFIX,{self._negv_str()}")

    def _on_thresh(self):
        # always emit a decimal point so the firmware parses the arg as volts.
        try:
            v = float(self.thresh_var.get().strip())
        except ValueError:
            self._log("** invalid threshold value")
            return
        self._send(f"!THRESH,{v:.3f}")

    def _on_alerts_on(self):
        # force the normal LED alerts back on while charging (RA4M1)
        self._send("!ALERTS,1")

    def _on_alerts_off(self):
        # restore the default charging blink (respect the charge lockout)
        self._send("!ALERTS,0")

    # ------------------------------------------------------- calibration
    def _open_calibration(self):
        if self.cal_win is not None:
            self.cal_win.lift()
            return
        self.cal_win = CalibrationWindow(self)

    def _handle_cal(self, line):
        # $CAL,<ms>,<a5V>,<testV>
        f = line.split(",")
        try:
            a5 = float(f[2])
            testv = float(f[3])
        except (ValueError, IndexError):
            return
        if self.cal_win is not None:
            self.cal_win.add_point(a5, testv)

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
        elif line.startswith("$CAL,"):
            self._handle_cal(line)          # streamed fast -> not logged
        elif line.startswith("$STATUS,"):
            self._handle_status(line)
            self._log(f"<< {line}")
        else:
            self._log(f"<< {line}")

    def _handle_diag(self, line):
        # two-channel: $DIAG,<ms>,<rawPos>,<rawNeg>,<posV>,<negV>,<diffV>
        # diff-only:   $DIAG,<ms>,<rawDiff>,<diffV>
        f = line.split(",")
        try:
            ms = int(f[1])
            if len(f) >= 7:
                pv, nv, dv = float(f[4]), float(f[5]), float(f[6])
                self.has_se = True
            elif len(f) >= 4:
                dv = float(f[3])
                pv = nv = float("nan")
                self.has_se = False
            else:
                return
        except (ValueError, IndexError):
            return
        if self.t0 is None:
            self.t0 = ms
        t = (ms - self.t0) / 1000.0
        self.live_t.append(t)
        self.live_pos.append(pv)
        self.live_neg.append(nv)
        self.live_diff.append(dv)
        if self.has_se:
            self.read_lbl.config(
                text=f"pos: {pv:+.4f} V   neg: {nv:+.4f} V   diff: {dv:+.4f} V")
        else:
            self.read_lbl.config(text=f"diff: {dv:+.4f} V")

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
            if "negfix" in kv:
                self.negfix_var.set(kv["negfix"] == "1")
            if "negv" in kv:
                self.negv_var.set(kv["negv"])
            if "openthr" in kv:
                self.thresh_var.set(kv["openthr"])
        except ValueError:
            pass

        # adaptive-threshold mode (CFG_A5_THRESH): openthr is driven from A5, so
        # make the manual entry read-only and label it.
        if "adaptthr" in kv:
            if kv["adaptthr"] == "1":
                self.thresh_entry.config(state="disabled")
                self.thresh_set_btn.config(state="disabled")
                self.thresh_mode_lbl.config(text="(A5 auto)", foreground="#06a")
            else:
                self.thresh_entry.config(state="normal")
                self.thresh_set_btn.config(state="normal")
                self.thresh_mode_lbl.config(text="")

        # charge lockout indicator (RA4M1): show USB-power state and whether
        # the normal alerts have been forced back on.
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

        # battery state-of-charge (RA4M1)
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
        if self.has_se:
            self.live_ax.plot(self.live_t, self.live_pos, label="pos (A0)", lw=0.9)
            self.live_ax.plot(self.live_t, self.live_neg, label="neg (A2)", lw=0.9)
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

        # Row formats:
        #   two-channel: $CAP,<t_us>,<rawPos>,<rawNeg>
        #   diff-only:   $CAP,<t_us>,<rawDiff>
        se = None
        rows = []          # full per-sample records for CSV export
        t_ms, pos_v, neg_v, diff_v = [], [], [], []
        for row in self.cap_rows:
            f = row.split(",")
            try:
                t = float(f[1])
                if len(f) >= 4:
                    rp = int(f[2])
                    rn = int(f[3])
                    pv = rp / full_scale * vref
                    nv = rn / full_scale * vref
                    dv = pv - nv
                    if se is None:
                        se = True
                    t_rel = (t - toggle_us) / 1000.0
                    pos_v.append(pv)
                    neg_v.append(nv)
                    rows.append((t, t_rel, rp, rn, pv, nv, dv))
                elif len(f) >= 3:
                    rd = int(f[2])
                    dv = rd / full_scale * vref
                    if se is None:
                        se = False
                    t_rel = (t - toggle_us) / 1000.0
                    rows.append((t, t_rel, rd, dv))
                else:
                    continue
            except (ValueError, IndexError):
                continue
            t_ms.append(t_rel)
            diff_v.append(dv)

        se = bool(se)
        # retain for CSV export and enable the save button
        self.last_capture = {"toggle_us": toggle_us, "se": se, "rows": rows}
        self.save_btn.config(state=("normal" if rows else "disabled"))

        self.cap_ax.clear()
        self.cap_ax.set_title("Last capture")
        self.cap_ax.set_xlabel("time from toggle (ms)")
        self.cap_ax.set_ylabel("volts")
        if t_ms:
            if se:
                self.cap_ax.plot(t_ms, pos_v, ".-", ms=2, lw=0.8, label="pos (A0)")
                self.cap_ax.plot(t_ms, neg_v, ".-", ms=2, lw=0.8, label="neg (A2)")
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
                if self.last_capture["se"]:
                    w.writerow(["t_us", "t_ms_from_toggle",
                                "rawPos", "rawNeg", "posV", "negV", "diffV"])
                    for (t, t_rel, rp, rn, pv, nv, dv) in self.last_capture["rows"]:
                        w.writerow([f"{t:.0f}", f"{t_rel:.4f}", rp, rn,
                                    f"{pv:.6f}", f"{nv:.6f}", f"{dv:.6f}"])
                else:
                    w.writerow(["t_us", "t_ms_from_toggle", "rawDiff", "diffV"])
                    for (t, t_rel, rd, dv) in self.last_capture["rows"]:
                        w.writerow([f"{t:.0f}", f"{t_rel:.4f}", rd, f"{dv:.6f}"])
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
        if self.cal_win is not None:
            self.cal_win._on_close()
        self.serial.disconnect()
        self.destroy()


class CalibrationWindow(tk.Toplevel):
    """Sweep the A5 pot and capture A5 voltage vs the post-MOSFET-toggle
    differential, to build the firmware's A5_THRESH_TABLE for a known resistor.

    Drives the device: !DIAG,1 + !CAL,1 to start, !CAL,0 to stop.  Each incoming
    $CAL,<ms>,<a5V>,<testV> row is one scatter point (A5 vs |post-toggle diff|).

    Multiple sweeps can be overlaid: "Add sweep" freezes the current points on
    the chart and begins a fresh set (new colour) so you can change the resistor
    and characterise several resistances together.  Each sweep saves to its own
    pair of CSV columns.
    """

    # distinct colours cycled per sweep
    COLORS = ["#06a", "#c00", "#0a0", "#e08000", "#709", "#0aa", "#a06", "#556"]

    def __init__(self, app):
        super().__init__(app)
        self.app = app
        self.title("A5 threshold calibration")
        self.geometry("700x600")

        self.sweeping = False
        self._dirty = True

        self._build()
        # sweeps: list of {"res": str, "color": str, "a5": [], "testv": []}
        self.sweeps = []
        self._new_sweep()

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(150, self._refresh)

    def _build(self):
        top = ttk.Frame(self, padding=6)
        top.pack(fill="x")
        ttk.Label(top, text="Threshold resistor (ohms):").pack(side="left")
        self.res_var = tk.StringVar(value="330000")
        ttk.Entry(top, width=10, textvariable=self.res_var).pack(side="left", padx=4)
        self.sweep_btn = ttk.Button(top, text="Start sweep", command=self._toggle_sweep)
        self.sweep_btn.pack(side="left", padx=8)
        self.add_btn = ttk.Button(top, text="Add sweep", command=self._on_add_sweep)
        self.add_btn.pack(side="left", padx=2)
        ttk.Button(top, text="Clear all", command=self._clear).pack(side="left", padx=2)
        ttk.Button(top, text="Save CSV", command=self._save).pack(side="left", padx=2)
        self.count_lbl = ttk.Label(top, text="")
        self.count_lbl.pack(side="left", padx=8)

        hint = ("Connect the threshold resistor (e.g. 330k) across the leads, "
                "start the sweep, then rotate the pot slowly end-to-end a few "
                "times.  Each dot is one sample: A5 control voltage vs the "
                "post-toggle |differential| the detector compares to the "
                "threshold.  Use 'Add sweep' to keep the current dots, change "
                "the resistor, and lay down a new-coloured set.  Save the CSV, "
                "then read the A5_THRESH_TABLE points off the curve.")
        ttk.Label(self, text=hint, wraplength=670, foreground="#444",
                  padding=(6, 0)).pack(fill="x")

        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self._setup_axes()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=6, pady=4)

    def _setup_axes(self):
        self.ax.set_title("A5 voltage vs post-toggle differential")
        self.ax.set_xlabel("A5 (V)")
        self.ax.set_ylabel("post-toggle |diff| (V)")
        self.ax.grid(True, alpha=0.3)

    # ----- sweep bookkeeping -----
    def _new_sweep(self):
        color = self.COLORS[len(self.sweeps) % len(self.COLORS)]
        self.sweeps.append({"res": self.res_var.get().strip(),
                            "color": color, "a5": [], "testv": []})
        self._dirty = True

    @property
    def _active(self):
        return self.sweeps[-1]

    def _toggle_sweep(self):
        if not self.app.serial.is_open:
            messagebox.showinfo("Calibration", "Connect to the device first.")
            return
        self.sweeping = not self.sweeping
        if self.sweeping:
            # label the active sweep with the resistor in the box right now
            self._active["res"] = self.res_var.get().strip()
            self.app._send("!DIAG,1")
            self.app._send("!CAL,1")
            self.sweep_btn.config(text="Stop sweep")
        else:
            self.app._send("!CAL,0")
            self.sweep_btn.config(text="Start sweep")

    def _on_add_sweep(self):
        # freeze the current sweep and open a fresh one (new colour).  Stop any
        # in-progress capture first so points don't bleed into the new set.
        if self.sweeping:
            self.app._send("!CAL,0")
            self.sweeping = False
            self.sweep_btn.config(text="Start sweep")
        if not self._active["a5"]:
            return                          # active sweep is empty; nothing to freeze
        self._new_sweep()

    def add_point(self, a5, testv):
        s = self._active
        s["a5"].append(a5)
        s["testv"].append(abs(testv))       # threshold table uses magnitude
        self._dirty = True

    def _clear(self):
        self.sweeps = []
        self._new_sweep()
        self._dirty = True

    def _refresh(self):
        if self._dirty:
            self._dirty = False
            self.ax.clear()
            self._setup_axes()
            total = 0
            have_data = False
            for i, s in enumerate(self.sweeps):
                if not s["a5"]:
                    continue
                have_data = True
                total += len(s["a5"])
                self.ax.plot(s["a5"], s["testv"], ".", ms=3, color=s["color"],
                             label=f'{s["res"]} ohm')
            if have_data:
                self.ax.legend(loc="best", fontsize=8)
            self.canvas.draw_idle()
            self.count_lbl.config(
                text=f"{len([s for s in self.sweeps if s['a5']])} sweeps, {total} pts")
        self.after(150, self._refresh)

    def _save(self):
        # refresh the active sweep's label from the box, then keep non-empty sweeps
        self._active["res"] = self.res_var.get().strip()
        sweeps = [s for s in self.sweeps if s["a5"]]
        if not sweeps:
            messagebox.showinfo("Save calibration", "No points captured yet.")
            return
        path = filedialog.asksaveasfilename(
            title="Save calibration CSV",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialfile="a5_calibration.csv",
        )
        if not path:
            return
        # wide format: each sweep gets its own (a5, |diff|) column pair, each
        # sorted by A5 ascending and padded to the longest sweep.
        cols = [sorted(zip(s["a5"], s["testv"])) for s in sweeps]
        maxlen = max(len(c) for c in cols)
        header = []
        for s in sweeps:
            header += [f'a5_V_{s["res"]}ohm', f'absdiff_V_{s["res"]}ohm']
        try:
            with open(path, "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(header)
                for r in range(maxlen):
                    row = []
                    for c in cols:
                        if r < len(c):
                            row += [f"{c[r][0]:.4f}", f"{c[r][1]:.4f}"]
                        else:
                            row += ["", ""]
                    w.writerow(row)
        except OSError as exc:
            messagebox.showerror("Save calibration", f"Could not write file:\n{exc}")
            return
        self.app._log(
            f"** saved calibration ({len(sweeps)} sweeps, "
            f"{sum(len(c) for c in cols)} pts) -> {path}")

    def _on_close(self):
        if self.sweeping and self.app.serial.is_open:
            self.app._send("!CAL,0")
        self.sweeping = False
        self.app.cal_win = None
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
