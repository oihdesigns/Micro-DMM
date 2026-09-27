#!/usr/bin/env python3
"""
relay_jig_gui.py  --  Host GUI for the RelayMountTestJig firmware (UNO R4 WiFi).

Talks to the board over USB serial (115200 baud).

  Load selector
    - one button per load: FGEN / SHORT / OPEN / 1M / 10K / 10.6M / 150K
      (keys 0-6 do the same)
    - big readout of what the DUT is actually connected to, decoded by the
      firmware from the real relay states
    - the firmware walks every change through OPEN, so the DUT never sees a
      different resistance or a short in passing

  Sequence
    - step through any selection of loads with a fixed dwell, optionally
      looping; each step is logged with a timestamp

  Relays (manual)
    - K1..K8 indicators; with "Manual override" ticked they become toggles
      that drive single relays (!K) -- no sequencing, you are on your own

  Log
    - raw serial traffic and a free-form command box

Dependencies:
    pip install pyserial

Run:
    python relay_jig_gui.py
"""

import queue
import threading
import tkinter as tk
from datetime import datetime
from tkinter import ttk

import serial
import serial.tools.list_ports

BAUD = 115200
LOG_MAXLINES = 3000

# (firmware name, button label, description)
MODES = [
    ("FGEN",  "FGen",   "Function generator"),
    ("SHORT", "Short",  "0 Ω"),
    ("OPEN",  "Open",   "Open circuit"),
    ("1M",    "1 M",    "R1  1 MΩ"),
    ("10K",   "10 k",   "R2  10 kΩ"),
    ("10.6M", "10.6 M", "R3  10.6 MΩ"),
    ("150K",  "150 k",  "R4  150 kΩ"),
]
LOAD_TEXT = {m: d for m, _, d in MODES}
LOAD_TEXT["SPLIT"] = "SPLIT: one lead on FGen, one on network"

RELAY_ROLE = [
    "DUT+  FGen / network",
    "DUT-  FGen / network",
    "Short / continue",
    "1M branch / 10k branch",
    "1M to return / open",
    "10k / continue",
    "10.6M / 150k",
    "spare",
]

BG = "#1A1A1A"
FG = "#E0E0E0"
GREEN = "#00FF32"
AMBER = "#FFB000"
DIM = "#3A3A3A"


class SerialManager:
    """Owns the port and a reader thread that pushes whole lines to a queue."""

    def __init__(self, line_queue):
        self.q = line_queue
        self.ser = None
        self._stop = threading.Event()
        self._thread = None

    def connect(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.1)
        self._stop.clear()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def disconnect(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1)
        if self.ser:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None

    @property
    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def send(self, text):
        if not self.is_open:
            return False
        try:
            self.ser.write((text + "\n").encode("ascii"))
            return True
        except serial.SerialException as e:
            self.q.put(("#err", str(e)))
            return False

    def _read_loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(256)
            except (serial.SerialException, OSError) as e:
                self.q.put(("#err", str(e)))
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
        self.title("Relay Test Jig")
        self.minsize(720, 520)

        self.q = queue.Queue()
        self.link = SerialManager(self.q)

        self.port_var = tk.StringVar()
        self.mode_var = tk.StringVar(value="--")
        self.load_var = tk.StringVar(value="not connected")
        self.fw_var = tk.StringVar(value="")
        self.settle_var = tk.IntVar(value=25)
        self.manual_var = tk.BooleanVar(value=False)
        self.relay_bits = "0" * 8

        self.seq_vars = {m: tk.BooleanVar(value=(m != "FGEN")) for m, _, _ in MODES}
        self.dwell_var = tk.DoubleVar(value=2.0)
        self.loop_var = tk.BooleanVar(value=False)
        self.seq_status = tk.StringVar(value="idle")
        self._seq_job = None
        self._seq_list = []
        self._seq_idx = 0

        self._build_ui()
        self.refresh_ports()
        self.after(30, self._pump)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        for i, (m, _, _) in enumerate(MODES):
            self.bind(str(i), lambda e, m=m: self._key_mode(e, m))

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        top = ttk.Frame(self, padding=6)
        top.pack(fill="x")
        ttk.Label(top, text="Port").pack(side="left")
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=28)
        self.port_box.pack(side="left", padx=4)
        ttk.Button(top, text="↻", width=3, command=self.refresh_ports).pack(side="left")
        self.conn_btn = ttk.Button(top, text="Connect", command=self.toggle_connect)
        self.conn_btn.pack(side="left", padx=6)
        ttk.Label(top, textvariable=self.fw_var, foreground="#666").pack(side="left", padx=8)

        # Status banner: what the DUT is connected to right now.
        banner = tk.Frame(self, bg=BG, padx=12, pady=10)
        banner.pack(fill="x", padx=6)
        tk.Label(banner, text="DUT SEES", bg=BG, fg="#888",
                 font=("Segoe UI", 9)).pack(anchor="w")
        self.load_lbl = tk.Label(banner, textvariable=self.load_var, bg=BG, fg=GREEN,
                                 font=("Consolas", 28, "bold"), anchor="w")
        self.load_lbl.pack(fill="x")
        tk.Label(banner, textvariable=self.mode_var, bg=BG, fg="#888",
                 font=("Consolas", 10), anchor="w").pack(fill="x")

        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=6, pady=6)
        self._build_select_tab(nb)
        self._build_seq_tab(nb)
        self._build_relay_tab(nb)
        self._build_log_tab(nb)

    def _build_select_tab(self, nb):
        f = ttk.Frame(nb, padding=10)
        nb.add(f, text="Load selector")
        style = ttk.Style(self)
        style.configure("Mode.TButton", font=("Segoe UI", 13, "bold"), padding=10)
        style.configure("Active.TButton", font=("Segoe UI", 13, "bold"), padding=10,
                        foreground="#007A18")
        self.mode_btns = {}
        for i, (m, label, desc) in enumerate(MODES):
            b = ttk.Button(f, text=f"{label}\n[{i}]", style="Mode.TButton",
                           command=lambda m=m: self.set_mode(m))
            r, c = (0, 0) if i == 0 else (1 + (i - 1) // 3, (i - 1) % 3)
            b.grid(row=r, column=c, sticky="nsew", padx=4, pady=4)
            self.mode_btns[m] = b
        for c in range(3):
            f.columnconfigure(c, weight=1)

        opts = ttk.Frame(f)
        opts.grid(row=4, column=0, columnspan=3, sticky="w", pady=(12, 0))
        ttk.Label(opts, text="Relay settle (ms)").pack(side="left")
        ttk.Spinbox(opts, from_=0, to=2000, increment=5, width=6,
                    textvariable=self.settle_var).pack(side="left", padx=4)
        ttk.Button(opts, text="Apply", command=self.apply_settle).pack(side="left")
        ttk.Button(opts, text="Release all (!ALL)",
                   command=lambda: self.send("!ALL")).pack(side="left", padx=16)

    def _build_seq_tab(self, nb):
        f = ttk.Frame(nb, padding=10)
        nb.add(f, text="Sequence")
        steps = ttk.LabelFrame(f, text="Steps (in order)", padding=6)
        steps.grid(row=0, column=0, sticky="nw")
        for m, label, desc in MODES:
            ttk.Checkbutton(steps, text=f"{label:6s}  {desc}",
                            variable=self.seq_vars[m]).pack(anchor="w")

        ctl = ttk.Frame(f, padding=(16, 0))
        ctl.grid(row=0, column=1, sticky="nw")
        ttk.Label(ctl, text="Dwell per step (s)").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(ctl, from_=0.1, to=3600, increment=0.5, width=8,
                    textvariable=self.dwell_var).grid(row=0, column=1, padx=4)
        ttk.Checkbutton(ctl, text="Loop", variable=self.loop_var).grid(
            row=1, column=0, sticky="w", pady=4)
        ttk.Button(ctl, text="Start", command=self.seq_start).grid(
            row=2, column=0, sticky="ew", pady=4)
        ttk.Button(ctl, text="Stop", command=self.seq_stop).grid(
            row=2, column=1, sticky="ew", pady=4)
        ttk.Label(ctl, textvariable=self.seq_status).grid(
            row=3, column=0, columnspan=2, sticky="w", pady=8)

    def _build_relay_tab(self, nb):
        f = ttk.Frame(nb, padding=10)
        nb.add(f, text="Relays")
        ttk.Checkbutton(f, text="Manual override (drive single relays, no sequencing)",
                        variable=self.manual_var,
                        command=self._paint_relays).grid(row=0, column=0, columnspan=3,
                                                         sticky="w", pady=(0, 8))
        self.relay_lamps = []
        for k in range(8):
            lamp = tk.Label(f, text=f"K{k + 1}", width=5, bg=DIM, fg=FG,
                            font=("Consolas", 12, "bold"), relief="raised", bd=2)
            lamp.grid(row=k + 1, column=0, padx=4, pady=2)
            lamp.bind("<Button-1>", lambda e, k=k: self.toggle_relay(k))
            ttk.Label(f, text=RELAY_ROLE[k]).grid(row=k + 1, column=1, sticky="w", padx=8)
            self.relay_lamps.append(lamp)
        ttk.Label(f, foreground="#666",
                  text="Lit = relay moved to its NO contact.  Clicks only act "
                       "with Manual override ticked.").grid(row=10, column=0,
                                                            columnspan=3, sticky="w",
                                                            pady=(10, 0))

    def _build_log_tab(self, nb):
        f = ttk.Frame(nb, padding=6)
        nb.add(f, text="Log")
        self.log = tk.Text(f, height=12, bg=BG, fg=FG, insertbackground=FG,
                           font=("Consolas", 9), wrap="none")
        sb = ttk.Scrollbar(f, command=self.log.yview)
        self.log.configure(yscrollcommand=sb.set)
        self.log.grid(row=0, column=0, sticky="nsew")
        sb.grid(row=0, column=1, sticky="ns")
        self.log.tag_configure("tx", foreground="#6CB6FF")
        self.log.tag_configure("data", foreground=GREEN)
        self.log.tag_configure("err", foreground="#FF5555")
        row = ttk.Frame(f)
        row.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(4, 0))
        self.raw_var = tk.StringVar()
        e = ttk.Entry(row, textvariable=self.raw_var)
        e.pack(side="left", fill="x", expand=True)
        e.bind("<Return>", lambda _e: self.send_raw())
        ttk.Button(row, text="Send", command=self.send_raw).pack(side="left", padx=4)
        ttk.Button(row, text="Clear",
                   command=lambda: self.log.delete("1.0", "end")).pack(side="left")
        f.rowconfigure(0, weight=1)
        f.columnconfigure(0, weight=1)

    # ------------------------------------------------------------ link
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def toggle_connect(self):
        if self.link.is_open:
            self.seq_stop()
            self.link.disconnect()
            self._set_connected(False)
            return
        port = self.port_var.get().strip()
        if not port:
            return
        try:
            self.link.connect(port)
        except serial.SerialException as e:
            self._log(f"open {port} failed: {e}", "err")
            return
        self._set_connected(True)
        # Opening the port may reset the board; ask for identity/state after
        # it has had time to boot (a $BOOT line will also refresh state).
        self.after(1500, lambda: (self.send("!ID"), self.send("!STATE")))

    def _set_connected(self, ok):
        self.conn_btn.configure(text="Disconnect" if ok else "Connect")
        if not ok:
            self.fw_var.set("")
            self.load_var.set("not connected")
            self.mode_var.set("--")
            self.load_lbl.configure(fg="#888")

    def send(self, text):
        if self.link.send(text):
            self._log("> " + text, "tx")
            return True
        self._log("not connected: " + text, "err")
        return False

    def send_raw(self):
        t = self.raw_var.get().strip()
        if t:
            self.send(t)
            self.raw_var.set("")

    # --------------------------------------------------------- actions
    def set_mode(self, m):
        self.send(f"!MODE,{m}")

    def _key_mode(self, event, m):
        # Don't hijack digits typed into an entry/spinbox.
        if isinstance(event.widget, (tk.Entry, ttk.Entry, ttk.Spinbox, ttk.Combobox)):
            return
        self.set_mode(m)

    def apply_settle(self):
        try:
            self.send(f"!SETTLE,{int(self.settle_var.get())}")
        except (tk.TclError, ValueError):
            pass

    def toggle_relay(self, k):
        if not self.manual_var.get():
            return
        on = self.relay_bits[k] != "1"
        self.send(f"!K,{k + 1},{1 if on else 0}")

    # -------------------------------------------------------- sequence
    def seq_start(self):
        self.seq_stop()
        self._seq_list = [m for m, _, _ in MODES if self.seq_vars[m].get()]
        if not self._seq_list:
            self.seq_status.set("no steps selected")
            return
        self._seq_idx = 0
        self._seq_step()

    def _seq_step(self):
        if self._seq_idx >= len(self._seq_list):
            if not self.loop_var.get():
                self.seq_status.set("done")
                self._seq_job = None
                return
            self._seq_idx = 0
        m = self._seq_list[self._seq_idx]
        n = len(self._seq_list)
        if not self.send(f"!MODE,{m}"):
            self.seq_status.set("stopped: not connected")
            self._seq_job = None
            return
        self.seq_status.set(f"step {self._seq_idx + 1}/{n}: {LOAD_TEXT[m]}")
        self._seq_idx += 1
        try:
            dwell = max(0.1, float(self.dwell_var.get()))
        except (tk.TclError, ValueError):
            dwell = 2.0
        self._seq_job = self.after(int(dwell * 1000), self._seq_step)

    def seq_stop(self):
        if self._seq_job is not None:
            self.after_cancel(self._seq_job)
            self._seq_job = None
            self.seq_status.set("stopped")

    # ------------------------------------------------------- incoming
    def _pump(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "#err":
                    self._log("serial error: " + payload, "err")
                    self.seq_stop()
                    self.link.disconnect()
                    self._set_connected(False)
                else:
                    self._handle_line(payload)
        except queue.Empty:
            pass
        self.after(30, self._pump)

    def _handle_line(self, line):
        if not line:
            return
        tag = "err" if line.startswith("$ERR") else "data" if line.startswith("$") else None
        self._log(line, tag)
        fields = line.split(",")
        head = fields[0]
        if head == "$STATE" and len(fields) >= 5:
            self._on_state(fields[1], fields[2], fields[3], fields[4])
        elif head in ("$ID", "$BOOT") and len(fields) >= 3:
            self.fw_var.set(f"{fields[1]} v{fields[2]}")
            if head == "$BOOT":
                self._log("board reset -- all relays released", "err")

    def _on_state(self, mode, bits, load, settle):
        self.relay_bits = bits.ljust(8, "0")[:8]
        self.load_var.set(LOAD_TEXT.get(load, load))
        self.mode_var.set(f"mode {mode}   relays K1..K8 {self.relay_bits}   settle {settle} ms")
        self.load_lbl.configure(fg=AMBER if mode == "RAW" or load == "SPLIT" else GREEN)
        try:
            self.settle_var.set(int(settle))
        except ValueError:
            pass
        for m, b in self.mode_btns.items():
            b.configure(style="Active.TButton" if (m == mode) else "Mode.TButton")
        self._paint_relays()

    def _paint_relays(self):
        manual = self.manual_var.get()
        for k, lamp in enumerate(self.relay_lamps):
            on = self.relay_bits[k] == "1"
            lamp.configure(bg=GREEN if on else DIM, fg=BG if on else FG,
                           cursor="hand2" if manual else "")

    def _log(self, text, tag=None):
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log.insert("end", f"{stamp}  {text}\n", tag or ())
        n = int(self.log.index("end-1c").split(".")[0])
        if n > LOG_MAXLINES:
            self.log.delete("1.0", f"{n - LOG_MAXLINES}.0")
        self.log.see("end")

    def _on_close(self):
        self.seq_stop()
        self.link.disconnect()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
