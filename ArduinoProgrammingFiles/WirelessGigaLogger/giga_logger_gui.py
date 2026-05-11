"""
giga_logger_gui.py  —  Windows Python GUI for WirelessGigaLogger (Arduino Giga R1 WiFi)

Requirements:
    pip install bleak matplotlib

Usage:
    python giga_logger_gui.py

The GUI runs bleak in a background asyncio thread and communicates with
the tkinter main thread via thread-safe queues.
"""

import asyncio
import threading
import queue
import tkinter as tk
from tkinter import ttk, messagebox, filedialog, scrolledtext
import time
import csv
import os
import io
from datetime import datetime

# ─── BLE UUIDs (must match Arduino sketch) ────────────────────────────────────
DEVICE_NAME = "GigaLogger"
CMD_UUID    = "19b10001-e8f2-537e-4f6c-d104768a1214"
STREAM_UUID = "19b10002-e8f2-537e-4f6c-d104768a1214"

# ─── Thread-safe queues ───────────────────────────────────────────────────────
rx_q = queue.Queue()   # BLE RX  → GUI thread
tx_q = queue.Queue()   # GUI     → BLE TX thread

# ─── BLE manager (runs in background asyncio thread) ─────────────────────────
class BLEManager:
    def __init__(self):
        self.client   = None
        self.loop     = asyncio.new_event_loop()
        self.connected = False
        t = threading.Thread(target=self._run_loop, daemon=True)
        t.start()

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    # ── Scan ──
    def scan(self):
        """Return a Future whose result is a list of (name, address) tuples."""
        fut = asyncio.run_coroutine_threadsafe(self._scan(), self.loop)
        return fut

    async def _scan(self):
        from bleak import BleakScanner
        devices = await BleakScanner.discover(timeout=5.0)
        found = []
        for d in devices:
            if d.name and DEVICE_NAME in d.name:
                found.append((d.name, d.address))
        return found

    # ── Connect ──
    def connect(self, address):
        asyncio.run_coroutine_threadsafe(self._connect(address), self.loop)

    async def _connect(self, address):
        from bleak import BleakClient
        try:
            self.client = BleakClient(
                address,
                disconnected_callback=self._on_disconnect
            )
            await self.client.connect()
            await self.client.start_notify(STREAM_UUID, self._notify)
            self.connected = True
            rx_q.put(('connected', address))
            # Start TX pump
            asyncio.ensure_future(self._tx_pump(), loop=self.loop)
        except Exception as e:
            rx_q.put(('error', f"Connect failed: {e}"))

    def _on_disconnect(self, _client):
        self.connected = False
        rx_q.put(('disconnected', ''))

    def _notify(self, _sender, data: bytearray):
        try:
            text = data.decode('utf-8', errors='replace').strip()
            rx_q.put(('data', text))
        except Exception as e:
            rx_q.put(('error', str(e)))

    async def _tx_pump(self):
        while self.connected and self.client and self.client.is_connected:
            try:
                cmd = tx_q.get_nowait()
                payload = (cmd + '\n').encode('utf-8')
                await self.client.write_gatt_char(CMD_UUID, payload, response=False)
                # 40 ms pause after every write so ArduinoBLE's single-value
                # characteristic buffer is read before the next command lands.
                # Without this, rapid writes overwrite each other and commands are lost.
                await asyncio.sleep(0.04)
            except queue.Empty:
                await asyncio.sleep(0.02)
            except Exception as e:
                rx_q.put(('error', f"TX error: {e}"))
                break

    # ── Disconnect ──
    def disconnect(self):
        if self.client:
            asyncio.run_coroutine_threadsafe(self.client.disconnect(), self.loop)

    # ── Send command ──
    def send(self, cmd: str):
        tx_q.put(cmd)


# ─── Channel config dialog ────────────────────────────────────────────────────
class ChannelConfigDialog(tk.Toplevel):
    def __init__(self, parent, ch_data, on_apply):
        super().__init__(parent)
        self.title("Channel Configuration")
        self.resizable(False, False)
        self.on_apply = on_apply

        # ch_data: list of 8 dicts {active, offset, scale, unit, threshold}
        self.vars = []
        for c in range(8):
            d = ch_data[c]
            self.vars.append({
                'active':    tk.BooleanVar(value=d['active']),
                'offset':    tk.StringVar(value=f"{d['offset']:.4f}"),
                'scale':     tk.StringVar(value=f"{d['scale']:.4f}"),
                'unit':      tk.StringVar(value=d['unit']),
                'threshold': tk.StringVar(value=f"{d['threshold']:.4f}"),
            })

        hdr = ['En', 'Ch', 'Offset', 'Scale', 'Unit', 'Threshold']
        for col, h in enumerate(hdr):
            tk.Label(self, text=h, font=('Consolas', 9, 'bold'),
                     padx=6).grid(row=0, column=col, sticky='w')

        for c in range(8):
            v = self.vars[c]
            row = c + 1
            tk.Checkbutton(self, variable=v['active']).grid(row=row, column=0, padx=4)
            tk.Label(self, text=f"A{c}", font=('Consolas', 9)).grid(row=row, column=1, sticky='w', padx=4)
            tk.Entry(self, textvariable=v['offset'],    width=9,  font=('Consolas', 9)).grid(row=row, column=2, padx=4)
            tk.Entry(self, textvariable=v['scale'],     width=9,  font=('Consolas', 9)).grid(row=row, column=3, padx=4)
            ttk.Combobox(self, textvariable=v['unit'],  width=4,
                         values=['V', 'I'], state='readonly',
                         font=('Consolas', 9)).grid(row=row, column=4, padx=4)
            tk.Entry(self, textvariable=v['threshold'], width=9,  font=('Consolas', 9)).grid(row=row, column=5, padx=4)

        btn_frame = tk.Frame(self)
        btn_frame.grid(row=9, column=0, columnspan=6, pady=8)
        tk.Button(btn_frame, text="Apply", width=10,
                  command=self._apply).pack(side='left', padx=6)
        tk.Button(btn_frame, text="Cancel", width=10,
                  command=self.destroy).pack(side='left', padx=6)

    def _apply(self):
        result = []
        for c in range(8):
            v = self.vars[c]
            try:
                result.append({
                    'active':    v['active'].get(),
                    'offset':    float(v['offset'].get()),
                    'scale':     float(v['scale'].get()),
                    'unit':      v['unit'].get() or 'V',
                    'threshold': float(v['threshold'].get()),
                })
            except ValueError:
                messagebox.showerror("Invalid input", f"Channel {c}: check numeric fields")
                return
        self.on_apply(result)
        self.destroy()


# ─── Main application ─────────────────────────────────────────────────────────
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("GigaLogger")
        self.configure(bg='#1a1a1a')
        self.resizable(True, True)

        self.ble = BLEManager()
        self.scan_results = []   # list of (name, address)
        self.connected    = False
        self.active_mask  = 1    # bitmask of active channels
        self.ch_data      = [    # current channel config
            {'active': c == 0, 'offset': 0.0, 'scale': 1.0, 'unit': 'V', 'threshold': 1.0}
            for c in range(8)
        ]

        # Burst accumulator
        self.burst_samples   = []
        self.burst_total     = 0
        self.burst_mask      = 1
        self.burst_bits      = 16   # 12 or 16 — set by $BURST_START
        self.burst_epoch_ms  = 0    # epochMs() at capture start, from $BURST_START field 5
        self.burst_active    = False

        # File download accumulator
        self.dl_chunks      = {}   # seq → data string
        self.dl_total       = 0
        self.dl_filename    = ''
        self.dl_active      = False

        # Local CSV save (burst/trigger events)
        self.local_csv_path       = None
        self.local_csv_has_header = False

        self._build_ui()
        self._poll()   # start polling rx_q

    # ── UI construction ──────────────────────────────────────────────────────
    def _build_ui(self):
        COL_BG   = '#1a1a1a'
        COL_FG   = '#e0e0e0'
        COL_ACC  = '#00c864'
        COL_PANEL= '#252525'
        COL_BTN  = '#2d2d2d'

        style = ttk.Style(self)
        style.theme_use('clam')
        style.configure('TNotebook',       background=COL_BG,    borderwidth=0)
        style.configure('TNotebook.Tab',   background=COL_PANEL, foreground=COL_FG,
                        padding=[10, 4])
        style.map('TNotebook.Tab',         background=[('selected', COL_ACC)],
                                           foreground=[('selected', '#000000')])
        style.configure('TCombobox',       fieldbackground=COL_BTN, background=COL_BTN,
                        foreground=COL_FG)

        # ── Connection bar ──
        conn_frame = tk.Frame(self, bg=COL_PANEL, pady=6)
        conn_frame.pack(fill='x', padx=8, pady=(8, 0))

        tk.Button(conn_frame, text="Scan", width=7, command=self._scan,
                  bg=COL_BTN, fg=COL_FG, relief='flat').pack(side='left', padx=4)

        self.dev_var = tk.StringVar()
        self.dev_cb  = ttk.Combobox(conn_frame, textvariable=self.dev_var,
                                     width=30, state='readonly')
        self.dev_cb.pack(side='left', padx=4)

        tk.Button(conn_frame, text="Connect", width=8, command=self._connect,
                  bg=COL_BTN, fg=COL_FG, relief='flat').pack(side='left', padx=4)
        tk.Button(conn_frame, text="Disconnect", width=10, command=self._disconnect,
                  bg=COL_BTN, fg=COL_FG, relief='flat').pack(side='left', padx=4)

        self.conn_lbl = tk.Label(conn_frame, text="● Disconnected",
                                  bg=COL_PANEL, fg='#ff4444', font=('Consolas', 10))
        self.conn_lbl.pack(side='left', padx=12)

        # ── Notebook ──
        nb = ttk.Notebook(self)
        nb.pack(fill='both', expand=True, padx=8, pady=8)

        tab_main  = tk.Frame(nb, bg=COL_BG)
        tab_burst = tk.Frame(nb, bg=COL_BG)
        tab_dl    = tk.Frame(nb, bg=COL_BG)
        nb.add(tab_main,  text='  Live  ')
        nb.add(tab_burst, text='  Burst  ')
        nb.add(tab_dl,    text='  File DL  ')

        self._build_main_tab(tab_main,  COL_BG, COL_FG, COL_ACC, COL_PANEL, COL_BTN)
        self._build_burst_tab(tab_burst, COL_BG, COL_FG, COL_PANEL, COL_BTN)
        self._build_dl_tab(tab_dl,       COL_BG, COL_FG, COL_PANEL, COL_BTN)

    def _lbl(self, parent, text, bg, fg, font=('Consolas', 9), **kwargs):
        return tk.Label(parent, text=text, bg=bg, fg=fg, font=font, **kwargs)

    def _btn(self, parent, text, cmd, bg_btn, fg, width=12):
        return tk.Button(parent, text=text, command=cmd,
                         bg=bg_btn, fg=fg, relief='flat',
                         activebackground='#3d3d3d', width=width)

    def _build_main_tab(self, tab, BG, FG, ACC, PANEL, BTN):
        # Split: left=controls, right=live values
        left  = tk.Frame(tab,  bg=PANEL, padx=10, pady=10)
        right = tk.Frame(tab,  bg=BG,    padx=10, pady=10)
        left.pack(side='left', fill='y', padx=(0, 4))
        right.pack(side='left', fill='both', expand=True)

        # ── Controls ──
        self._lbl(left, "MODE", PANEL, ACC, ('Consolas', 9, 'bold')).pack(anchor='w')
        self.mode_var = tk.StringVar(value='TRIG')
        for m in ('TRIG', 'BURST', 'CONT'):
            tk.Radiobutton(left, text=m, variable=self.mode_var, value=m,
                           command=self._send_mode,
                           bg=PANEL, fg=FG, selectcolor=PANEL,
                           activebackground=PANEL,
                           font=('Consolas', 9)).pack(anchor='w')

        tk.Frame(left, bg='#444', height=1).pack(fill='x', pady=6)

        # Smooth N
        sf = tk.Frame(left, bg=PANEL)
        sf.pack(fill='x', pady=2)
        self._lbl(sf, "Smooth N:", PANEL, FG).pack(side='left')
        self.smooth_var = tk.StringVar(value='7')
        tk.Entry(sf, textvariable=self.smooth_var, width=5,
                 bg='#333', fg=FG, insertbackground=FG,
                 font=('Consolas', 9)).pack(side='left', padx=4)
        self._btn(sf, "Set", self._send_smooth, BTN, FG, width=4).pack(side='left')

        # Burst ms
        bf = tk.Frame(left, bg=PANEL)
        bf.pack(fill='x', pady=2)
        self._lbl(bf, "Burst ms: ", PANEL, FG).pack(side='left')
        self.burst_ms_var = tk.StringVar(value='1000')
        tk.Entry(bf, textvariable=self.burst_ms_var, width=7,
                 bg='#333', fg=FG, insertbackground=FG,
                 font=('Consolas', 9)).pack(side='left', padx=4)
        self._btn(bf, "Set", self._send_burst_ms, BTN, FG, width=4).pack(side='left')

        # Trig hold ms
        thf = tk.Frame(left, bg=PANEL)
        thf.pack(fill='x', pady=2)
        self._lbl(thf, "Trig hold ms:", PANEL, FG).pack(side='left')
        self.trig_hold_var = tk.StringVar(value='5')
        tk.Entry(thf, textvariable=self.trig_hold_var, width=6,
                 bg='#333', fg=FG, insertbackground=FG,
                 font=('Consolas', 9)).pack(side='left', padx=4)
        self._btn(thf, "Set", self._send_trig_hold, BTN, FG, width=4).pack(side='left')

        # Compact burst TX (12-bit)
        self.compact_var = tk.BooleanVar(value=False)
        tk.Checkbutton(left, text="Compact burst (12-bit TX)", variable=self.compact_var,
                       command=self._send_compact,
                       bg=PANEL, fg=FG, selectcolor=PANEL,
                       activebackground=PANEL,
                       font=('Consolas', 9)).pack(anchor='w', pady=(2, 0))

        tk.Frame(left, bg='#444', height=1).pack(fill='x', pady=6)

        # Action buttons
        for text, cmd in [
            ("Start",         self._cmd_start),
            ("Stop",          self._cmd_stop),
            ("Mark",          self._cmd_mark),
            ("Ch Config…",    self._open_ch_config),
        ]:
            self._btn(left, text, cmd, BTN, FG, width=14).pack(pady=2, fill='x')

        tk.Frame(left, bg='#444', height=1).pack(fill='x', pady=6)

        # USB toggle
        self.usb_var = tk.BooleanVar(value=False)
        tk.Checkbutton(left, text="USB Logging", variable=self.usb_var,
                       command=self._send_usb,
                       bg=PANEL, fg=FG, selectcolor=PANEL,
                       activebackground=PANEL,
                       font=('Consolas', 9)).pack(anchor='w')

        # Local PC save
        self.local_save_var = tk.BooleanVar(value=False)
        tk.Checkbutton(left, text="Save to PC CSV", variable=self.local_save_var,
                       command=self._toggle_local_save,
                       bg=PANEL, fg=FG, selectcolor=PANEL,
                       activebackground=PANEL,
                       font=('Consolas', 9)).pack(anchor='w')

        tk.Frame(left, bg='#444', height=1).pack(fill='x', pady=6)

        # Status counts
        self.log_count_lbl = self._lbl(left, "Logs: 0", PANEL, FG)
        self.log_count_lbl.pack(anchor='w')
        self.usb_status_lbl = self._lbl(left, "USB: --", PANEL, FG)
        self.usb_status_lbl.pack(anchor='w')

        # ── Live values ──
        self._lbl(right, "LIVE VALUES", BG, ACC, ('Consolas', 10, 'bold')).pack(anchor='w')

        self.live_frame = tk.Frame(right, bg=BG)
        self.live_frame.pack(fill='x', pady=4)
        self.live_lbls = []   # one label per channel
        for c in range(8):
            lbl = self._lbl(self.live_frame, f"A{c}: ---", BG, '#888',
                            ('Consolas', 11))
            lbl.grid(row=c, column=0, sticky='w', padx=4, pady=1)
            self.live_lbls.append(lbl)

        tk.Frame(right, bg='#444', height=1).pack(fill='x', pady=6)

        # Event log
        self._lbl(right, "EVENT LOG", BG, ACC, ('Consolas', 10, 'bold')).pack(anchor='w')
        self.event_log = scrolledtext.ScrolledText(
            right, height=14, bg='#111', fg='#00e060',
            font=('Consolas', 9), state='disabled',
            insertbackground='white', relief='flat')
        self.event_log.pack(fill='both', expand=True)

    def _build_burst_tab(self, tab, BG, FG, PANEL, BTN):
        self._lbl(tab, "BURST DATA", BG, '#00c864',
                  ('Consolas', 10, 'bold')).pack(anchor='w', padx=8, pady=(8, 2))

        hdr_frame = tk.Frame(tab, bg=PANEL, pady=4)
        hdr_frame.pack(fill='x', padx=8)
        self.burst_info_lbl = self._lbl(hdr_frame, "No burst received yet",
                                         PANEL, FG)
        self.burst_info_lbl.pack(side='left', padx=8)
        self.burst_progress = ttk.Progressbar(hdr_frame, length=200, mode='determinate')
        self.burst_progress.pack(side='left', padx=8)
        self._btn(hdr_frame, "Plot",     self._plot_burst,     BTN, FG, width=8
                  ).pack(side='right', padx=4)
        self._btn(hdr_frame, "Save CSV", self._save_burst_csv, BTN, FG, width=10
                  ).pack(side='right', padx=4)

        self.burst_text = scrolledtext.ScrolledText(
            tab, bg='#111', fg='#cccccc', font=('Consolas', 8),
            state='disabled', relief='flat')
        self.burst_text.pack(fill='both', expand=True, padx=8, pady=8)

    def _build_dl_tab(self, tab, BG, FG, PANEL, BTN):
        self._lbl(tab, "FILE DOWNLOAD (from USB stick over BLE)", BG, '#00c864',
                  ('Consolas', 10, 'bold')).pack(anchor='w', padx=8, pady=(8, 2))

        ctrl = tk.Frame(tab, bg=PANEL, pady=6)
        ctrl.pack(fill='x', padx=8)
        self._btn(ctrl, "Request File", self._cmd_dlfile, BTN, FG, width=14
                  ).pack(side='left', padx=8)
        self._btn(ctrl, "Save to PC", self._save_dl_file, BTN, FG, width=12
                  ).pack(side='left', padx=4)
        self.dl_info_lbl = self._lbl(ctrl, "No download yet", PANEL, FG)
        self.dl_info_lbl.pack(side='left', padx=12)
        self.dl_progress = ttk.Progressbar(ctrl, length=200, mode='determinate')
        self.dl_progress.pack(side='left', padx=8)

        self.dl_text = scrolledtext.ScrolledText(
            tab, bg='#111', fg='#cccccc', font=('Consolas', 8),
            state='disabled', relief='flat')
        self.dl_text.pack(fill='both', expand=True, padx=8, pady=8)

    # ── BLE actions ──────────────────────────────────────────────────────────
    def _scan(self):
        self.conn_lbl.config(text="● Scanning…", fg='#aaaaaa')
        fut = self.ble.scan()
        self.after(100, lambda: self._check_scan(fut))

    def _check_scan(self, fut):
        if not fut.done():
            self.after(200, lambda: self._check_scan(fut))
            return
        try:
            results = fut.result()
            self.scan_results = results
            names = [f"{name}  [{addr}]" for name, addr in results]
            self.dev_cb['values'] = names
            if names:
                self.dev_cb.current(0)
                self.conn_lbl.config(text=f"● {len(names)} found", fg='#aaaaaa')
            else:
                self.conn_lbl.config(text="● None found", fg='#ff8800')
        except Exception as e:
            self.conn_lbl.config(text="● Scan error", fg='#ff4444')
            self._log(f"Scan error: {e}")

    def _connect(self):
        idx = self.dev_cb.current()
        if idx < 0 or idx >= len(self.scan_results):
            messagebox.showwarning("GigaLogger", "Select a device first")
            return
        _, addr = self.scan_results[idx]
        self.conn_lbl.config(text="● Connecting…", fg='#aaaaaa')
        self.ble.connect(addr)

    def _disconnect(self):
        self.ble.disconnect()

    def send(self, cmd):
        if self.connected:
            self.ble.send(cmd)

    # ── Commands ─────────────────────────────────────────────────────────────
    def _send_mode(self):
        m = self.mode_var.get()
        self.send(f"!{m}_MODE")

    def _send_smooth(self):
        try:
            n = int(self.smooth_var.get())
            self.send(f"!SMOOTH,{n}")
        except ValueError:
            pass

    def _send_burst_ms(self):
        try:
            ms = int(self.burst_ms_var.get())
            self.send(f"!BURST_MS,{ms}")
        except ValueError:
            pass

    def _send_trig_hold(self):
        try:
            ms = int(self.trig_hold_var.get())
            self.send(f"!TRIG_HOLD,{ms}")
        except ValueError:
            pass

    def _cmd_start(self):
        self.send("!START")
        if self.mode_var.get() == 'BURST':
            self.burst_samples = []
            self.burst_active  = False
            self._update_burst_progress(0, 0)

    def _cmd_stop(self):  self.send("!STOP")
    def _cmd_mark(self):  self.send("!MARK")

    def _cmd_dlfile(self):
        self.dl_chunks  = {}
        self.dl_total   = 0
        self.dl_active  = False
        self._set_dl_text("")
        self.send("!DLFILE")

    def _send_usb(self):
        self.send(f"!USB,{1 if self.usb_var.get() else 0}")

    def _send_compact(self):
        self.send(f"!COMPACT,{1 if self.compact_var.get() else 0}")

    def _open_ch_config(self):
        ChannelConfigDialog(self, self.ch_data, self._apply_ch_config)

    def _apply_ch_config(self, new_cfg):
        self.ch_data = new_cfg
        # Queue all commands — the BLE TX pump spaces them 40 ms apart so
        # ArduinoBLE's single-value buffer is read before each next write lands.
        for c, d in enumerate(new_cfg):
            en = 1 if d['active'] else 0
            self.send(f"!CH,{c},{en},{d['offset']:.5f},{d['scale']:.5f},{d['unit']}")
            self.send(f"!THRES,{c},{d['threshold']:.5f}")
        # Request a full status echo once all config commands are queued.
        # This arrives after the 40 ms-spaced burst, so the Arduino is idle.
        self.send("!STATUS")

    # ── Local PC CSV ─────────────────────────────────────────────────────────
    def _toggle_local_save(self):
        if self.local_save_var.get():
            path = filedialog.asksaveasfilename(
                defaultextension='.csv',
                filetypes=[('CSV', '*.csv'), ('All', '*.*')],
                title="Save trigger/burst events to…")
            if not path:
                self.local_save_var.set(False)
                return
            self.local_csv_path       = path
            self.local_csv_has_header = False
            self._log(f"Saving bursts to {os.path.basename(path)}")
        else:
            self.local_csv_path       = None
            self.local_csv_has_header = False

    def _write_local_burst(self):
        """Append the most recently completed burst to the local CSV (open/write/close)."""
        if not self.local_save_var.get() or not self.local_csv_path:
            return
        if not self.burst_samples:
            return
        active_ch = [c for c in range(8) if (self.burst_mask >> c) & 1]
        _, ch_vals, _ = self._burst_to_physical()
        with open(self.local_csv_path, 'a', newline='') as f:
            w = csv.writer(f)
            if not self.local_csv_has_header:
                w.writerow(['datetime', 't_rel_us'] +
                           [f"A{c}_{self.ch_data[c]['unit']}" for c in active_ch])
                self.local_csv_has_header = True
            for i, s in enumerate(self.burst_samples):
                t_rel = s[1] if len(s) > 1 else '0'
                try:
                    t_us = int(t_rel)
                except ValueError:
                    t_us = 0
                if self.burst_epoch_ms:
                    abs_us = self.burst_epoch_ms * 1000 + t_us
                    dt = (datetime.fromtimestamp(abs_us / 1_000_000)
                          .strftime('%Y-%m-%d %H:%M:%S.') + f"{abs_us % 1_000_000:06d}")
                else:
                    dt = ''
                vals = [f"{ch_vals[c][i]:.5f}" if i < len(ch_vals.get(c, [])) else ''
                        for c in active_ch]
                w.writerow([dt, t_rel] + vals)
        self._log(f"Saved {len(self.burst_samples)} samples → {os.path.basename(self.local_csv_path)}")

    # ── Incoming message dispatcher ───────────────────────────────────────────
    def _poll(self):
        try:
            while True:
                kind, payload = rx_q.get_nowait()
                if kind == 'connected':
                    self._on_connected(payload)
                elif kind == 'disconnected':
                    self._on_disconnected()
                elif kind == 'data':
                    self._on_data(payload)
                elif kind == 'error':
                    self._log(f"[ERR] {payload}")
        except queue.Empty:
            pass
        self.after(40, self._poll)

    def _on_connected(self, addr):
        self.connected = True
        self.conn_lbl.config(text=f"● {addr}", fg='#00c864')
        self._log(f"Connected to {addr}")
        # Seed the clock
        epoch_ms = int(time.time() * 1000)
        self.send(f"!TIME,{epoch_ms}")
        self.send("!STATUS")

    def _on_disconnected(self):
        self.connected = False
        self.conn_lbl.config(text="● Disconnected", fg='#ff4444')
        self._log("Disconnected")

    def _on_data(self, text):
        if not text:
            return
        if text.startswith('$LOG,'):
            self._handle_log(text)
        elif text.startswith('$TRIG,'):
            self._handle_trig(text)
        elif text.startswith('$STATUS,'):
            self._handle_status(text)
        elif text.startswith('$CHCFG,'):
            self._handle_chcfg(text)
        elif text.startswith('$BURST_START,'):
            self._handle_burst_start(text)
        elif text.startswith('$BD,'):
            self._handle_burst_data(text)
        elif text.startswith('$BURST_END,'):
            self._handle_burst_end(text)
        elif text == '$BURST_STARTED':
            self._log("[BURST] Capturing…")
            self.burst_active = True
        elif text.startswith('$FILE_START,'):
            self._handle_file_start(text)
        elif text.startswith('$FC,'):
            self._handle_file_chunk(text)
        elif text.startswith('$FILE_END,'):
            self._handle_file_end(text)
        elif text.startswith('$ACK,'):
            self._log(f"[ACK] {text[5:]}")
        elif text.startswith('$ERR,'):
            self._log(f"[ERR] {text[5:]}")
        else:
            self._log(f"[?] {text}")

    # ── Message handlers ──────────────────────────────────────────────────────
    def _handle_log(self, text):
        # $LOG,<epoch_ms>,<ch0>,<ch1>,...  (active channels in order)
        parts = text.split(',')
        if len(parts) < 2:
            return
        active = [c for c in range(8) if self.ch_data[c]['active']]
        vals   = parts[2:]
        for i, c in enumerate(active):
            v = vals[i] if i < len(vals) else '---'
            unit = self.ch_data[c]['unit']
            self.live_lbls[c].config(
                text=f"A{c} ({unit}): {v}",
                fg='#00e060')
        # dim inactive
        for c in range(8):
            if not self.ch_data[c]['active']:
                self.live_lbls[c].config(text=f"A{c}: ---", fg='#555')

    def _handle_trig(self, text):
        # $TRIG,<epoch_ms>,<ch0>,...[,MARK]
        parts = text.split(',')
        if len(parts) < 2:
            return
        ts = parts[1]
        active = [c for c in range(8) if self.ch_data[c]['active']]
        vals   = parts[2:]
        mark   = vals and vals[-1] == 'MARK'
        if mark:
            vals = vals[:-1]

        dt = datetime.fromtimestamp(int(ts) / 1000).strftime('%H:%M:%S.%f')[:-3]
        pairs = ' '.join(
            f"A{active[i]}={vals[i]}" for i in range(min(len(active), len(vals)))
        )
        tag = '  ★ MARK' if mark else ''
        self._log(f"[TRIG] {dt}  {pairs}{tag}")

        # Write to local CSV
        row = [ts] + [vals[i] if i < len(vals) else '' for i in range(len(active))]
        self._write_local_event(row)

        # Update live display
        for i, c in enumerate(active):
            if i < len(vals):
                self.live_lbls[c].config(
                    text=f"A{c} ({self.ch_data[c]['unit']}): {vals[i]}",
                    fg='#ffdd00')

    def _handle_status(self, text):
        # $STATUS,<mode>,<mask>,<logActive>,<smoothN>,<burstMs>,<usbReady>,<usbLog>,<logs>
        parts = text.split(',')
        if len(parts) < 9:
            return
        try:
            mode_n      = int(parts[1])
            mask        = int(parts[2])
            smooth_n    = int(parts[4])
            burst_ms    = int(parts[5])
            usb_ready   = int(parts[6])
            usb_logging = int(parts[7])
            logs        = int(parts[8])
        except (ValueError, IndexError):
            return

        modes = ['TRIG', 'BURST', 'CONT']
        if 0 <= mode_n < len(modes):
            self.mode_var.set(modes[mode_n])

        self.active_mask = mask
        self.smooth_var.set(str(smooth_n))
        self.burst_ms_var.set(str(burst_ms))
        self.usb_var.set(bool(usb_logging))
        self.log_count_lbl.config(text=f"Logs: {logs}")
        self.usb_status_lbl.config(
            text=f"USB: {'Ready' if usb_ready else 'None'}  Log: {'ON' if usb_logging else 'off'}")

        # Sync channel active states from mask
        for c in range(8):
            self.ch_data[c]['active'] = bool((mask >> c) & 1)

    def _handle_chcfg(self, text):
        # $CHCFG,<n>,<en>,<offset>,<scale>,<unit>,<thres>
        parts = text.split(',')
        if len(parts) < 7:
            return
        try:
            n = int(parts[1])
            if n < 0 or n > 7:
                return
            self.ch_data[n]['active']    = bool(int(parts[2]))
            self.ch_data[n]['offset']    = float(parts[3])
            self.ch_data[n]['scale']     = float(parts[4])
            self.ch_data[n]['unit']      = parts[5] if parts[5] in ('V','I') else 'V'
            self.ch_data[n]['threshold'] = float(parts[6])
        except (ValueError, IndexError):
            pass

    # ── Burst ─────────────────────────────────────────────────────────────────
    def _handle_burst_start(self, text):
        # $BURST_START,<n>,<mask>,<smoothN>,<bits>,<epoch_ms>
        parts = text.split(',')
        try:
            self.burst_total    = int(parts[1])
            self.burst_mask     = int(parts[2])
            self.burst_bits     = int(parts[4]) if len(parts) > 4 else 16
            self.burst_epoch_ms = int(parts[5]) if len(parts) > 5 else 0
            self.burst_samples  = []
        except (ValueError, IndexError):
            self.burst_total    = 0
            self.burst_bits     = 16
            self.burst_epoch_ms = 0
        self.burst_active = True
        self._update_burst_progress(0, self.burst_total)
        active_ch = [c for c in range(8) if (self.burst_mask >> c) & 1]
        hdr = 'idx,t_rel_us,' + ','.join(
            f"A{c}({self.ch_data[c]['unit']})" for c in active_ch)
        self._set_burst_text(hdr + '\n')
        bits_str = f"{self.burst_bits}-bit"
        self._log(f"[BURST] Receiving {self.burst_total} samples ({bits_str})…")

    def _handle_burst_data(self, text):
        # $BD,<idx>,<t_rel_ms>,<raw0>,<raw1>,...  (raw ADC counts)
        parts = text.split(',')
        if len(parts) < 3:
            return
        self.burst_samples.append(parts[1:])   # [idx, t_rel_ms, raw0, raw1, ...]
        n = len(self.burst_samples)
        self._update_burst_progress(n, self.burst_total)
        # Show every 20th sample (with physical values) to avoid flooding the text box
        if n % 20 == 0 or n <= 3:
            active_ch = [c for c in range(8) if (self.burst_mask >> c) & 1]
            row = self._burst_row_to_str(parts[1:], active_ch)
            self._append_burst_text(row + '\n')

    def _burst_row_to_str(self, parts, active_ch):
        """Convert a burst sample [idx, t_rel_ms, raw0, ...] to a display string."""
        max_count = 4095 if self.burst_bits == 12 else 65535
        idx   = parts[0] if parts else '?'
        t_rel = parts[1] if len(parts) > 1 else '?'
        phys_vals = []
        for i, c in enumerate(active_ch):
            raw_idx = 2 + i
            try:
                raw  = int(parts[raw_idx])
                phys = ((raw / max_count) * 3.3 + self.ch_data[c]['offset']) * self.ch_data[c]['scale']
                phys_vals.append(f"{phys:.4f}")
            except (ValueError, IndexError):
                phys_vals.append('?')
        return f"{idx},{t_rel}," + ','.join(phys_vals)

    def _burst_to_physical(self):
        """Return (times_ms, {ch: [phys_values]}) for all received burst samples."""
        active_ch = [c for c in range(8) if (self.burst_mask >> c) & 1]
        max_count = 4095 if self.burst_bits == 12 else 65535
        times = []
        ch_vals = {c: [] for c in active_ch}
        for s in self.burst_samples:
            try:
                times.append(float(s[1]) / 1000.0)   # µs → ms for display
            except (ValueError, IndexError):
                times.append(0.0)
            for i, c in enumerate(active_ch):
                raw_idx = 2 + i
                try:
                    raw  = int(s[raw_idx])
                    phys = ((raw / max_count) * 3.3 + self.ch_data[c]['offset']) * self.ch_data[c]['scale']
                    ch_vals[c].append(phys)
                except (ValueError, IndexError):
                    ch_vals[c].append(0.0)
        return times, ch_vals, active_ch

    def _handle_burst_end(self, text):
        parts = text.split(',')
        total = int(parts[1]) if len(parts) > 1 else len(self.burst_samples)
        self.burst_active = False
        self._update_burst_progress(total, total)
        bits_str = f"{self.burst_bits}-bit"
        self._log(f"[BURST] Complete — {total} samples ({bits_str})")
        self.burst_info_lbl.config(text=f"{total} samples  ({bits_str})")
        # Rebuild text with all physical values (up to 1000 rows to keep UI responsive)
        active_ch = [c for c in range(8) if (self.burst_mask >> c) & 1]
        hdr = 'idx,t_rel_us,' + ','.join(
            f"A{c}({self.ch_data[c]['unit']})" for c in active_ch)
        rows = [hdr]
        for s in self.burst_samples[:1000]:
            rows.append(self._burst_row_to_str(s, active_ch))
        if total > 1000:
            rows.append(f"… {total - 1000} more rows (use Save CSV for full data)")
        self._set_burst_text('\n'.join(rows))
        self._write_local_burst()

    def _update_burst_progress(self, n, total):
        self.burst_progress['maximum'] = max(total, 1)
        self.burst_progress['value']   = n
        pct = int(100 * n / max(total, 1))
        self.burst_info_lbl.config(text=f"{n}/{total} ({pct}%)")

    def _set_burst_text(self, text):
        self.burst_text.config(state='normal')
        self.burst_text.delete('1.0', 'end')
        self.burst_text.insert('end', text)
        self.burst_text.config(state='disabled')

    def _append_burst_text(self, text):
        self.burst_text.config(state='normal')
        self.burst_text.insert('end', text)
        self.burst_text.see('end')
        self.burst_text.config(state='disabled')

    def _plot_burst(self):
        if not self.burst_samples:
            messagebox.showinfo("GigaLogger", "No burst data yet")
            return
        try:
            import matplotlib.pyplot as plt
            from matplotlib.ticker import AutoMinorLocator
        except ImportError:
            messagebox.showerror("GigaLogger",
                "matplotlib not installed.\nRun:  pip install matplotlib")
            return

        times, ch_vals, active_ch = self._burst_to_physical()
        n_ch = len(active_ch)
        if n_ch == 0:
            return

        COLORS = ['#00e060','#4488ff','#ff6644','#ffdd00',
                  '#cc44ff','#44ffcc','#ff4488','#88ff44']

        fig, axes = plt.subplots(
            n_ch, 1,
            figsize=(11, max(2.5 * n_ch, 4)),
            sharex=True, squeeze=False)
        fig.patch.set_facecolor('#1a1a1a')
        bits_str = f"{self.burst_bits}-bit"
        fig.suptitle(f"Burst Data  ({len(self.burst_samples)} samples, {bits_str})",
                     color='#e0e0e0', fontsize=11)

        for i, c in enumerate(active_ch):
            ax = axes[i][0]
            ax.set_facecolor('#111111')
            ax.plot(times, ch_vals[c],
                    color=COLORS[c % len(COLORS)], linewidth=0.9, label=f"A{c}")
            unit = self.ch_data[c]['unit']
            ax.set_ylabel(f"A{c} ({unit})", color='#cccccc', fontsize=9)
            ax.tick_params(colors='#888888', labelsize=8)
            ax.yaxis.set_minor_locator(AutoMinorLocator())
            ax.grid(True, which='major', color='#333333', linewidth=0.6)
            ax.grid(True, which='minor', color='#222222', linewidth=0.3)
            for spine in ax.spines.values():
                spine.set_edgecolor('#444444')

        axes[-1][0].set_xlabel('Time (ms)', color='#cccccc', fontsize=9)
        axes[-1][0].tick_params(axis='x', colors='#888888', labelsize=8)
        plt.tight_layout()
        plt.show()

    def _save_burst_csv(self):
        if not self.burst_samples:
            messagebox.showinfo("GigaLogger", "No burst data yet")
            return
        path = filedialog.asksaveasfilename(
            defaultextension='.csv',
            filetypes=[('CSV', '*.csv'), ('All', '*.*')],
            title="Save burst data…")
        if not path:
            return
        times, ch_vals, active_ch = self._burst_to_physical()
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['idx', 't_rel_us'] +
                       [f"A{c}_{self.ch_data[c]['unit']}" for c in active_ch])
            for i, s in enumerate(self.burst_samples):
                idx   = s[0] if s else i
                t_rel = s[1] if len(s) > 1 else ''
                vals  = [f"{ch_vals[c][i]:.5f}" if i < len(ch_vals[c]) else ''
                         for c in active_ch]
                w.writerow([idx, t_rel] + vals)
        self._log(f"Burst saved to {os.path.basename(path)}")

    # ── File download ─────────────────────────────────────────────────────────
    def _handle_file_start(self, text):
        # $FILE_START,<name>,<bytes>
        parts = text.split(',', 2)
        self.dl_filename = parts[1] if len(parts) > 1 else 'log.csv'
        self.dl_total    = int(parts[2]) if len(parts) > 2 else 0
        self.dl_chunks   = {}
        self.dl_active   = True
        self.dl_progress['maximum'] = 1
        self.dl_progress['value']   = 0
        self.dl_info_lbl.config(text=f"Downloading {self.dl_filename} ({self.dl_total} B)…")
        self._set_dl_text(f"# {self.dl_filename}  ({self.dl_total} bytes)\n")
        self._log(f"[DL] Starting {self.dl_filename}")

    def _handle_file_chunk(self, text):
        # $FC,<seq>,<total_chunks>,<csv_text>
        # Split only on first 3 commas — data may contain commas and newlines
        parts = text.split(',', 3)
        if len(parts) < 4:
            return
        seq   = int(parts[1])
        total = int(parts[2])
        data  = parts[3]
        self.dl_chunks[seq] = data
        self.dl_progress['maximum'] = max(total, 1)
        self.dl_progress['value']   = len(self.dl_chunks)
        pct = int(100 * len(self.dl_chunks) / max(total, 1))
        self.dl_info_lbl.config(text=f"Chunk {seq+1}/{total} ({pct}%)")

    def _handle_file_end(self, text):
        self.dl_active = False
        if not self.dl_chunks:
            self._log("[DL] Complete — no chunks received")
            return
        # Reassemble in sequence order
        ordered = [self.dl_chunks.get(i, '') for i in range(max(self.dl_chunks) + 1)]
        full = ''.join(ordered)
        self._set_dl_text(full)
        self._log(f"[DL] Complete — {len(full)} bytes")
        self.dl_info_lbl.config(text=f"Done — {len(full)} bytes")
        self.dl_progress['value'] = self.dl_progress['maximum']

    def _set_dl_text(self, text):
        self.dl_text.config(state='normal')
        self.dl_text.delete('1.0', 'end')
        self.dl_text.insert('end', text)
        self.dl_text.config(state='disabled')

    def _save_dl_file(self):
        content = self.dl_text.get('1.0', 'end').strip()
        if not content or content.startswith('No download'):
            messagebox.showinfo("GigaLogger", "No downloaded data yet")
            return
        default = self.dl_filename or 'download.csv'
        path = filedialog.asksaveasfilename(
            defaultextension='.csv',
            initialfile=default,
            filetypes=[('CSV', '*.csv'), ('All', '*.*')],
            title="Save downloaded file…")
        if not path:
            return
        with open(path, 'w', newline='') as f:
            f.write(content)
        self._log(f"Saved to {os.path.basename(path)}")

    # ── Utility ───────────────────────────────────────────────────────────────
    def _log(self, msg):
        ts  = datetime.now().strftime('%H:%M:%S')
        self.event_log.config(state='normal')
        self.event_log.insert('end', f"[{ts}] {msg}\n")
        self.event_log.see('end')
        self.event_log.config(state='disabled')

    def on_close(self):
        self.ble.disconnect()
        self.destroy()


if __name__ == '__main__':
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.geometry("980x720")
    app.mainloop()
