#!/usr/bin/env python3
"""
AMC_GUI.py  —  Serial GUI for AMC_testBoard (Feather RP2350)

METER tab  : live current + voltage with running min/max.
ANALYSIS tab: triggered capture, waveform/histogram plots, statistics.

Requirements
────────────
    pip install pyserial numpy matplotlib

Usage
─────
    python AMC_GUI.py
    python AMC_GUI.py --port COM3
    python AMC_GUI.py --port /dev/ttyACM0 --baud 115200

Serial protocol  (device → PC)
────────────────────────────────
    $AMC,I,V,Imin,Imax,Vmin,Vmax,gainI,gainV    (10 Hz, meter mode)
    $CAPTURE_START,N                              (capture about to begin)
    $S,idx,t_us,I_A,V_V                          (one line per sample)
    $CAPDONE,N,total_us,smoothN,gainI,gainV      (capture finished)

Commands  (PC → device)
─────────────────────────
    !RST                  reset min/max
    !ZERO / !UNZERO       current tare
    !CAPTURE,N[,smooth]   start capture (smooth=1 → raw, 0=current SMOOTH_N)
    !ABORT                cancel capture in progress
    !STATUS               immediate $AMC reply
"""

from __future__ import annotations

# ── DPI awareness — must be first, before any window is created ───────────────
import sys
if sys.platform == 'win32':
    try:
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(2)
    except Exception:
        try:
            windll.user32.SetProcessDPIAware()
        except Exception:
            pass

import argparse
import csv
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from typing import Dict, List, Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    raise SystemExit("pyserial not found.  pip install pyserial")

try:
    import numpy as np
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    MPL_OK = True
except ImportError:
    MPL_OK = False

# ── Colours ───────────────────────────────────────────────────────────────────
BG_ROOT    = '#111111'
BG_PANEL   = '#1A1A1A'
BG_HDR     = '#202020'
BG_BTN     = '#1a2a1a'
BG_CONN    = '#223322'
BG_TAB_ON  = '#1e2e1e'
BG_TAB_OFF = '#181818'

FG_LABEL   = '#505050'
FG_GAIN    = '#3a3a3a'
FG_GRAY    = '#888888'
FG_WHITE   = '#FFFFFF'
FG_I       = '#00FF32'   # green  — current
FG_V       = '#FFD700'   # yellow — voltage
FG_MINMAX  = '#00CCFF'   # cyan   — min/max
FG_BTN     = '#448844'
FG_CONN    = '#44aa44'
FG_RED     = '#FF4444'
FG_DIV     = '#2a2a2a'
FG_STAT_V  = '#cccccc'   # stat values

BAUD_DEFAULT = 115200
POLL_MS      = 50


# ── Font helper ───────────────────────────────────────────────────────────────

def _make_fonts(root: tk.Tk):
    """Return (font_dict, scale_factor) scaled to the display's real DPI."""
    dpi   = root.winfo_fpixels('1i')
    scale = dpi / 96.0

    def f(pt: int, *style: str) -> tuple:
        return ('Consolas', round(pt * scale), *style)

    fonts = {
        'hdr':    f(10, 'bold'),
        'label':  f(10),
        'gain':   f(9),
        'small':  f(9),
        'live':   f(38, 'bold'),
        'minmax': f(15),
        'btn':    f(10),
        'stat_k': f(9),
        'stat_v': f(9, 'bold'),
        'sec':    f(9, 'bold'),
    }
    return fonts, scale


# ── Main application ──────────────────────────────────────────────────────────

class AMCGui:
    def __init__(self, root: tk.Tk, initial_port: Optional[str] = None):
        self.root = root
        self.root.title('AMC Precision Meter')
        self.root.configure(bg=BG_ROOT)

        self._f, self._scale = _make_fonts(root)

        # Serial state
        self._ser: Optional[serial.Serial] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._running = False
        self._q: queue.Queue = queue.Queue()

        # Meter update rate
        self._upd_count = 0
        self._upd_t0    = time.monotonic()

        # Capture state
        self._cap_buf: List[Dict] = []
        self._cap_expected = 0
        self._cap_smooth   = 0
        self._cap_gainI    = ''
        self._cap_gainV    = ''
        self._capturing    = False

        # Analysis view mode  ('waveform' | 'histogram')
        self._view_mode = 'waveform'

        self._build_ui(initial_port)
        self._poll()

    # ═════════════════════════════════════════════════════════════════════════
    #  UI CONSTRUCTION
    # ═════════════════════════════════════════════════════════════════════════

    def _build_ui(self, initial_port: Optional[str]) -> None:
        f = self._f

        # ── Connection bar (row 0) ────────────────────────────────────────────
        conn = tk.Frame(self.root, bg=BG_HDR)
        conn.grid(row=0, column=0, sticky='ew')

        tk.Label(conn, text='Port:', bg=BG_HDR, fg=FG_GRAY,
                 font=f['label']).pack(side='left', padx=(10, 2), pady=7)

        self._port_var = tk.StringVar()
        self._port_cb  = ttk.Combobox(conn, textvariable=self._port_var,
                                      width=13, state='readonly',
                                      font=f['label'])
        self._port_cb.pack(side='left', pady=7)
        self._refresh_ports(initial_port)

        tk.Button(conn, text='↺', command=self._refresh_ports,
                  bg=BG_HDR, fg=FG_GRAY, font=f['label'],
                  relief='flat', bd=0, cursor='hand2',
                  activebackground=BG_HDR, activeforeground=FG_WHITE,
                  ).pack(side='left', padx=4)

        tk.Label(conn, text='Baud:', bg=BG_HDR, fg=FG_GRAY,
                 font=f['label']).pack(side='left', padx=(12, 2))
        self._baud_var = tk.StringVar(value=str(BAUD_DEFAULT))
        tk.Entry(conn, textvariable=self._baud_var, width=8,
                 bg='#2a2a2a', fg=FG_GRAY, insertbackground=FG_GRAY,
                 relief='flat', font=f['label'],
                 ).pack(side='left')

        self._conn_btn = tk.Button(
            conn, text='Connect', command=self._toggle_connection,
            bg=BG_CONN, fg=FG_CONN, font=f['label'],
            relief='flat', bd=0, padx=12, cursor='hand2',
            activebackground='#2a4a2a', activeforeground=FG_I)
        self._conn_btn.pack(side='right', padx=10, pady=7)

        # ── Tab bar (row 1) ───────────────────────────────────────────────────
        tab_bar = tk.Frame(self.root, bg=BG_HDR)
        tab_bar.grid(row=1, column=0, sticky='ew')

        btn_kw = dict(relief='flat', bd=0, padx=20, pady=5, cursor='hand2',
                      font=f['btn'])
        self._tab_meter_btn = tk.Button(tab_bar, text='METER',
                                        command=lambda: self._switch_tab('meter'),
                                        bg=BG_TAB_ON, fg=FG_I, **btn_kw)
        self._tab_meter_btn.pack(side='left')

        self._tab_analysis_btn = tk.Button(tab_bar, text='ANALYSIS',
                                           command=lambda: self._switch_tab('analysis'),
                                           bg=BG_TAB_OFF, fg=FG_LABEL, **btn_kw)
        self._tab_analysis_btn.pack(side='left')

        # ── Content area (row 2) ─────────────────────────────────────────────
        self._content = tk.Frame(self.root, bg=BG_PANEL)
        self._content.grid(row=2, column=0, sticky='nsew')
        self.root.rowconfigure(2, weight=1)
        self.root.columnconfigure(0, weight=1)

        self._meter_frame    = tk.Frame(self._content, bg=BG_PANEL)
        self._analysis_frame = tk.Frame(self._content, bg=BG_PANEL)

        self._build_meter_tab(self._meter_frame)
        self._build_analysis_tab(self._analysis_frame)

        self._meter_frame.pack(fill='both', expand=True)
        self._current_tab = 'meter'

        # ── Status bar (row 3) ───────────────────────────────────────────────
        status = tk.Frame(self.root, bg=BG_HDR)
        status.grid(row=3, column=0, sticky='ew')

        self._dot = tk.Label(status, text='●', fg=FG_RED,
                             bg=BG_HDR, font=f['small'])
        self._dot.pack(side='left', padx=(10, 3), pady=4)

        self._status_lbl = tk.Label(status, text='Disconnected',
                                    fg=FG_GRAY, bg=BG_HDR, font=f['small'])
        self._status_lbl.pack(side='left')

        self._rate_lbl = tk.Label(status, text='',
                                  fg=FG_GAIN, bg=BG_HDR, font=f['small'])
        self._rate_lbl.pack(side='right', padx=10)

    # ── METER TAB ─────────────────────────────────────────────────────────────

    def _build_meter_tab(self, parent: tk.Frame) -> None:
        f = self._f

        self._build_meter_section(
            parent, row=0,
            title='CURRENT', subtitle='AMC3301  ·  0.1 Ω shunt',
            color=FG_I, unit='A',
            attrs=('_lbl_I', '_lbl_Imin', '_lbl_Imax', '_lbl_gainI'))

        tk.Frame(parent, bg=FG_DIV, height=1).grid(
            row=1, column=0, sticky='ew', padx=14)

        self._build_meter_section(
            parent, row=2,
            title='VOLTAGE', subtitle='AMC3330  ·  50.5:1 divider',
            color=FG_V, unit='V',
            attrs=('_lbl_V', '_lbl_Vmin', '_lbl_Vmax', '_lbl_gainV'))

        tk.Frame(parent, bg=FG_DIV, height=1).grid(
            row=3, column=0, sticky='ew', padx=14)

        btn_row = tk.Frame(parent, bg=BG_PANEL)
        btn_row.grid(row=4, column=0, pady=12)

        tk.Button(btn_row, text='RESET  MIN / MAX',
                  command=lambda: self._send('!RST'),
                  bg=BG_BTN, fg=FG_BTN, font=f['btn'],
                  relief='flat', bd=0, padx=28, pady=8, cursor='hand2',
                  activebackground='#2a3a2a', activeforeground=FG_I,
                  ).pack(side='left', padx=6)

        tk.Button(btn_row, text='ZERO  CURRENT',
                  command=lambda: self._send('!ZERO'),
                  bg='#1a1a2a', fg='#4444aa', font=f['btn'],
                  relief='flat', bd=0, padx=20, pady=8, cursor='hand2',
                  activebackground='#2a2a3a', activeforeground='#8888ff',
                  ).pack(side='left', padx=6)

        tk.Button(btn_row, text='UN-ZERO',
                  command=lambda: self._send('!UNZERO'),
                  bg='#1a1a2a', fg='#333388', font=f['btn'],
                  relief='flat', bd=0, padx=14, pady=8, cursor='hand2',
                  activebackground='#2a2a3a', activeforeground='#8888ff',
                  ).pack(side='left', padx=6)

    def _build_meter_section(self, parent, row, title, subtitle,
                             color, unit, attrs) -> None:
        val_a, min_a, max_a, gain_a = attrs
        f = self._f

        frame = tk.Frame(parent, bg=BG_PANEL)
        frame.grid(row=row, column=0, sticky='ew')

        hdr = tk.Frame(frame, bg=BG_PANEL)
        hdr.grid(row=0, column=0, sticky='ew', padx=14, pady=(10, 0))
        tk.Label(hdr, text=title, fg=FG_LABEL, bg=BG_PANEL,
                 font=f['hdr']).pack(side='left')
        tk.Label(hdr, text=subtitle, fg=FG_GAIN, bg=BG_PANEL,
                 font=f['gain']).pack(side='left', padx=10)
        gain_lbl = tk.Label(hdr, text='Gain: ---', fg=FG_GAIN,
                            bg=BG_PANEL, font=f['gain'])
        gain_lbl.pack(side='right')
        setattr(self, gain_a, gain_lbl)

        live_lbl = tk.Label(frame, text=f'  -------- {unit}',
                            fg=color, bg=BG_PANEL,
                            font=f['live'], anchor='e', width=13)
        live_lbl.grid(row=1, column=0, sticky='ew', padx=14, pady=(2, 0))
        setattr(self, val_a, live_lbl)

        mm = tk.Frame(frame, bg=BG_PANEL)
        mm.grid(row=2, column=0, sticky='ew', padx=14, pady=(2, 10))
        tk.Label(mm, text='Min:', fg=FG_GAIN, bg=BG_PANEL,
                 font=f['minmax']).pack(side='left')
        min_lbl = tk.Label(mm, text=f'--------{unit}',
                           fg=FG_MINMAX, bg=BG_PANEL, font=f['minmax'],
                           anchor='e', width=12)
        min_lbl.pack(side='left', padx=(2, 22))
        setattr(self, min_a, min_lbl)

        tk.Label(mm, text='Max:', fg=FG_GAIN, bg=BG_PANEL,
                 font=f['minmax']).pack(side='left')
        max_lbl = tk.Label(mm, text=f'--------{unit}',
                           fg=FG_MINMAX, bg=BG_PANEL, font=f['minmax'],
                           anchor='e', width=12)
        max_lbl.pack(side='left', padx=(2, 0))
        setattr(self, max_a, max_lbl)

    # ── ANALYSIS TAB ──────────────────────────────────────────────────────────

    def _build_analysis_tab(self, parent: tk.Frame) -> None:
        f = self._f
        parent.columnconfigure(1, weight=1)
        parent.rowconfigure(0, weight=1)

        # ── Left controls panel ───────────────────────────────────────────────
        ctrl = tk.Frame(parent, bg=BG_PANEL, width=round(200 * self._scale))
        ctrl.grid(row=0, column=0, sticky='nsew', padx=(8, 4), pady=8)
        ctrl.grid_propagate(False)

        def _section_label(parent, text):
            tk.Label(parent, text=text, fg=FG_LABEL, bg=BG_PANEL,
                     font=f['sec']).pack(anchor='w', pady=(10, 2))
            tk.Frame(parent, bg=FG_DIV, height=1).pack(fill='x')

        # Capture settings
        _section_label(ctrl, 'CAPTURE SETTINGS')

        row_kw = dict(fill='x', pady=2)

        def _row(parent, label, widget_factory):
            r = tk.Frame(parent, bg=BG_PANEL)
            r.pack(**row_kw)
            tk.Label(r, text=label, fg=FG_GAIN, bg=BG_PANEL,
                     font=f['stat_k'], width=10, anchor='w').pack(side='left')
            widget_factory(r).pack(side='left')

        self._cap_n_var = tk.StringVar(value='500')
        _row(ctrl, 'Samples:', lambda p: tk.Spinbox(
            p, from_=50, to=2000, textvariable=self._cap_n_var,
            width=6, bg='#2a2a2a', fg=FG_GRAY, buttonbackground='#333333',
            relief='flat', font=f['stat_k']))

        self._cap_smooth_var = tk.StringVar(value='0')
        _row(ctrl, 'Smooth N:', lambda p: tk.Spinbox(
            p, from_=0, to=16, textvariable=self._cap_smooth_var,
            width=4, bg='#2a2a2a', fg=FG_GRAY, buttonbackground='#333333',
            relief='flat', font=f['stat_k']))

        tk.Label(ctrl, text='  (0 = use device default)',
                 fg=FG_GAIN, bg=BG_PANEL, font=('Consolas', round(7*self._scale))
                 ).pack(anchor='w')

        self._start_btn = tk.Button(
            ctrl, text='START CAPTURE',
            command=self._start_capture,
            bg='#1a3a1a', fg=FG_I, font=f['btn'],
            relief='flat', bd=0, pady=6, cursor='hand2',
            activebackground='#2a4a2a', activeforeground='#aaffaa')
        self._start_btn.pack(fill='x', pady=(6, 2))

        self._abort_btn = tk.Button(
            ctrl, text='ABORT',
            command=lambda: self._send('!ABORT'),
            bg='#2a1a1a', fg='#884444', font=f['btn'],
            relief='flat', bd=0, pady=4, cursor='hand2',
            activebackground='#3a2a2a', activeforeground=FG_RED,
            state='disabled')
        self._abort_btn.pack(fill='x', pady=(0, 4))

        self._cap_prog_var = tk.StringVar(value='')
        tk.Label(ctrl, textvariable=self._cap_prog_var,
                 fg=FG_GRAY, bg=BG_PANEL, font=f['stat_k']).pack(anchor='w')

        # Statistics
        _section_label(ctrl, 'CURRENT  (A)')
        self._stat_I = self._build_stat_block(ctrl,
            ['Mean', '\u03c3', 'Min', 'Max', 'Pk-Pk'])

        _section_label(ctrl, 'VOLTAGE  (V)')
        self._stat_V = self._build_stat_block(ctrl,
            ['Mean', '\u03c3', 'Min', 'Max', 'Pk-Pk'])

        _section_label(ctrl, 'CAPTURE INFO')
        self._stat_cap = self._build_stat_block(ctrl,
            ['N', 'Rate', 'Period', 'Smooth N', 'Gain I', 'Gain V'])

        # View toggle
        tk.Frame(ctrl, bg=FG_DIV, height=1).pack(fill='x', pady=(10, 4))
        vrow = tk.Frame(ctrl, bg=BG_PANEL)
        vrow.pack(fill='x')

        def _vtab(text, mode, color):
            return tk.Button(vrow, text=text,
                             command=lambda: self._set_view(mode),
                             bg=BG_TAB_OFF, fg=FG_LABEL, font=f['stat_k'],
                             relief='flat', bd=0, padx=8, pady=4, cursor='hand2')

        self._view_wave_btn = _vtab('WAVEFORM', 'waveform', FG_I)
        self._view_wave_btn.pack(side='left', padx=(0, 2))
        self._view_hist_btn = _vtab('HISTOGRAM', 'histogram', FG_V)
        self._view_hist_btn.pack(side='left')
        self._update_view_btns()

        # Save CSV
        tk.Button(ctrl, text='SAVE  CSV',
                  command=self._save_csv,
                  bg='#1a1a2a', fg='#4444aa', font=f['btn'],
                  relief='flat', bd=0, pady=6, cursor='hand2',
                  activebackground='#2a2a3a', activeforeground='#8888ff',
                  ).pack(fill='x', pady=(8, 0))

        # ── Right plot area ───────────────────────────────────────────────────
        plot_frame = tk.Frame(parent, bg=BG_PANEL)
        plot_frame.grid(row=0, column=1, sticky='nsew', padx=(4, 8), pady=8)
        plot_frame.rowconfigure(0, weight=1)
        plot_frame.columnconfigure(0, weight=1)

        if MPL_OK:
            fw = 7 * self._scale
            fh = 4.5 * self._scale
            self._fig = Figure(figsize=(fw, fh), facecolor='#141414')
            self._canvas = FigureCanvasTkAgg(self._fig, master=plot_frame)
            self._canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')
            self._draw_placeholder()
        else:
            tk.Label(plot_frame,
                     text='matplotlib / numpy not installed.\n\npip install numpy matplotlib',
                     fg=FG_GRAY, bg=BG_PANEL, font=f['label'],
                     justify='center').grid(row=0, column=0)

    def _build_stat_block(self, parent: tk.Frame,
                          keys: List[str]) -> Dict[str, tk.Label]:
        f = self._f
        labels = {}
        for k in keys:
            r = tk.Frame(parent, bg=BG_PANEL)
            r.pack(fill='x', pady=1)
            tk.Label(r, text=f'{k}:', fg=FG_GAIN, bg=BG_PANEL,
                     font=f['stat_k'], width=9, anchor='w').pack(side='left')
            lbl = tk.Label(r, text='---', fg=FG_STAT_V, bg=BG_PANEL,
                           font=f['stat_v'], anchor='w')
            lbl.pack(side='left')
            labels[k] = lbl
        return labels

    # ── Tab switching ─────────────────────────────────────────────────────────

    def _switch_tab(self, tab: str) -> None:
        if tab == self._current_tab:
            return
        if tab == 'meter':
            self._analysis_frame.pack_forget()
            self._meter_frame.pack(fill='both', expand=True)
            self._tab_meter_btn.config(bg=BG_TAB_ON, fg=FG_I)
            self._tab_analysis_btn.config(bg=BG_TAB_OFF, fg=FG_LABEL)
        else:
            self._meter_frame.pack_forget()
            self._analysis_frame.pack(fill='both', expand=True)
            self._tab_meter_btn.config(bg=BG_TAB_OFF, fg=FG_LABEL)
            self._tab_analysis_btn.config(bg=BG_TAB_ON, fg=FG_V)
        self._current_tab = tab

    # ── Port helpers ──────────────────────────────────────────────────────────

    def _refresh_ports(self, preselect: Optional[str] = None) -> None:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self._port_cb['values'] = ports
        current = self._port_var.get()
        if preselect and preselect in ports:
            self._port_var.set(preselect)
        elif current not in ports:
            self._port_var.set(ports[0] if ports else '')

    # ── Connection ────────────────────────────────────────────────────────────

    def _toggle_connection(self) -> None:
        if self._running:
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        port = self._port_var.get()
        if not port:
            self._status_lbl.config(text='No port selected')
            return
        try:
            baud = int(self._baud_var.get())
        except ValueError:
            baud = BAUD_DEFAULT
        try:
            self._ser = serial.Serial(port, baud, timeout=1)
        except serial.SerialException as exc:
            self._status_lbl.config(text=f'Error: {exc}')
            return

        self._running = True
        self._upd_count = 0
        self._upd_t0    = time.monotonic()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self._conn_btn.config(text='Disconnect', bg='#2a1a1a', fg=FG_RED)
        self._dot.config(fg=FG_I)
        self._status_lbl.config(text=f'Connected  {port}  {baud} baud')

    def _disconnect(self) -> None:
        self._running  = False
        self._capturing = False
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self._conn_btn.config(text='Connect', bg=BG_CONN, fg=FG_CONN)
        self._dot.config(fg=FG_RED)
        self._status_lbl.config(text='Disconnected')
        self._rate_lbl.config(text='')

    # ── Serial receive thread ─────────────────────────────────────────────────

    def _rx_loop(self) -> None:
        while self._running:
            try:
                line = self._ser.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue
                if line.startswith('$AMC,'):
                    self._q.put(('meter', line))
                elif line.startswith('$CAPTURE_START,'):
                    n = int(line.split(',')[1])
                    self._q.put(('cap_start', n))
                elif line.startswith('$S,'):
                    self._q.put(('sample', line))
                elif line.startswith('$CAPDONE,'):
                    self._q.put(('cap_done', line))
                elif line.startswith('$ABORT'):
                    self._q.put(('abort', None))
            except serial.SerialException:
                self._q.put(('error', 'Serial disconnected'))
                break
            except Exception:
                pass

    # ── GUI poll ──────────────────────────────────────────────────────────────

    def _poll(self) -> None:
        try:
            while True:
                kind, payload = self._q.get_nowait()
                if   kind == 'meter':
                    self._parse_meter(payload)
                elif kind == 'cap_start':
                    self._cap_expected = payload
                    self._cap_buf.clear()
                    self._cap_prog_var.set(f'0 / {payload}')
                elif kind == 'sample':
                    self._parse_sample(payload)
                elif kind == 'cap_done':
                    self._parse_capdone(payload)
                elif kind == 'abort':
                    self._capturing = False
                    self._set_capture_ui(False)
                    self._cap_prog_var.set('Aborted')
                elif kind == 'error':
                    self._disconnect()
                    self._status_lbl.config(text=payload)
        except queue.Empty:
            pass
        self.root.after(POLL_MS, self._poll)

    # ── Meter parsing ─────────────────────────────────────────────────────────

    def _parse_meter(self, line: str) -> None:
        parts = line.split(',')
        if len(parts) != 9:
            return
        try:
            I    = float(parts[1]); V    = float(parts[2])
            Imin = float(parts[3]); Imax = float(parts[4])
            Vmin = float(parts[5]); Vmax = float(parts[6])
            gI   = parts[7].strip(); gV  = parts[8].strip()
        except ValueError:
            return

        self._lbl_I.config(   text=f'{I:+.4f} A')
        self._lbl_Imin.config( text=f'{Imin:+.4f} A')
        self._lbl_Imax.config( text=f'{Imax:+.4f} A')
        self._lbl_gainI.config(text=f'Gain: \u00b1{gI}')
        self._lbl_V.config(   text=f'{V:+.3f} V')
        self._lbl_Vmin.config( text=f'{Vmin:+.3f} V')
        self._lbl_Vmax.config( text=f'{Vmax:+.3f} V')
        self._lbl_gainV.config(text=f'Gain: \u00b1{gV}')

        self._upd_count += 1
        now = time.monotonic()
        elapsed = now - self._upd_t0
        if elapsed >= 1.0:
            self._rate_lbl.config(text=f'{self._upd_count/elapsed:.1f} Hz')
            self._upd_count = 0
            self._upd_t0    = now

    # ── Capture parsing ───────────────────────────────────────────────────────

    def _parse_sample(self, line: str) -> None:
        # $S,idx,t_us,I_A,V_V
        parts = line.split(',')
        if len(parts) != 5:
            return
        try:
            t_us = int(parts[2])
            I    = float(parts[3])
            V    = float(parts[4])
        except ValueError:
            return
        self._cap_buf.append({'t': t_us, 'I': I, 'V': V})
        n = len(self._cap_buf)
        self._cap_prog_var.set(f'{n} / {self._cap_expected}')

    def _parse_capdone(self, line: str) -> None:
        # $CAPDONE,N,total_us,smoothN,gainI,gainV
        parts = line.split(',')
        if len(parts) >= 6:
            try:
                self._cap_smooth = int(parts[3])
                self._cap_gainI  = parts[4].strip()
                self._cap_gainV  = parts[5].strip()
            except (ValueError, IndexError):
                pass
        self._capturing = False
        self._set_capture_ui(False)
        self._cap_prog_var.set(f'\u2713 {len(self._cap_buf)} samples')
        self._finish_capture()

    def _finish_capture(self) -> None:
        if not MPL_OK or len(self._cap_buf) < 2:
            return
        self._compute_stats()
        self._redraw_plot()

    # ── Statistics ────────────────────────────────────────────────────────────

    def _compute_stats(self) -> None:
        buf = self._cap_buf
        t_arr = np.array([s['t'] for s in buf], dtype=np.float64)
        I_arr = np.array([s['I'] for s in buf], dtype=np.float64)
        V_arr = np.array([s['V'] for s in buf], dtype=np.float64)

        dt_ms     = np.diff(t_arr) / 1000.0          # µs → ms
        rate_hz   = 1000.0 / np.mean(dt_ms) if len(dt_ms) else 0.0
        period_ms = np.mean(dt_ms) if len(dt_ms) else 0.0

        def _fmt_I(v):
            if abs(v) < 0.001:  return f'{v*1e6:+.2f} µA'
            if abs(v) < 1.0:    return f'{v*1e3:+.3f} mA'
            return f'{v:+.5f} A'

        def _fmt_V(v):
            if abs(v) < 0.001:  return f'{v*1e3:+.3f} mV'
            return f'{v:+.4f} V'

        # Current stats
        si = self._stat_I
        si['Mean'].config( text=_fmt_I(np.mean(I_arr)))
        si['\u03c3'].config(text=_fmt_I(np.std(I_arr)))
        si['Min'].config(  text=_fmt_I(np.min(I_arr)))
        si['Max'].config(  text=_fmt_I(np.max(I_arr)))
        si['Pk-Pk'].config(text=_fmt_I(np.ptp(I_arr)))

        # Voltage stats
        sv = self._stat_V
        sv['Mean'].config( text=_fmt_V(np.mean(V_arr)))
        sv['\u03c3'].config(text=_fmt_V(np.std(V_arr)))
        sv['Min'].config(  text=_fmt_V(np.min(V_arr)))
        sv['Max'].config(  text=_fmt_V(np.max(V_arr)))
        sv['Pk-Pk'].config(text=_fmt_V(np.ptp(V_arr)))

        # Capture info
        sc = self._stat_cap
        sc['N'].config(      text=str(len(buf)))
        sc['Rate'].config(   text=f'{rate_hz:.1f} Hz')
        sc['Period'].config( text=f'{period_ms:.3f} ms')
        sc['Smooth N'].config(text=str(self._cap_smooth) if self._cap_smooth else 'device')
        sc['Gain I'].config( text=f'\u00b1{self._cap_gainI}')
        sc['Gain V'].config( text=f'\u00b1{self._cap_gainV}')

        # Store arrays for plotting
        self._t_arr = t_arr
        self._I_arr = I_arr
        self._V_arr = V_arr

    # ── Plotting ──────────────────────────────────────────────────────────────

    def _style_ax(self, ax, title: str, color: str) -> None:
        ax.set_facecolor('#0d0d0d')
        ax.set_title(title, color=color, fontsize=8, pad=3, loc='left')
        ax.tick_params(colors='#444444', labelsize=7)
        for spine in ax.spines.values():
            spine.set_color('#2a2a2a')
        ax.grid(True, color='#1a1a1a', linewidth=0.5, linestyle='--')

    def _draw_placeholder(self) -> None:
        self._fig.clear()
        ax = self._fig.add_subplot(1, 1, 1)
        ax.set_facecolor('#0d0d0d')
        ax.text(0.5, 0.5, 'No capture yet.\nSet parameters and press START CAPTURE.',
                transform=ax.transAxes,
                ha='center', va='center', color='#444444', fontsize=9)
        for spine in ax.spines.values():
            spine.set_color('#2a2a2a')
        ax.tick_params(colors='#333333')
        ax.set_xticks([]); ax.set_yticks([])
        self._fig.tight_layout(pad=1.0)
        self._canvas.draw()

    def _redraw_plot(self) -> None:
        if not MPL_OK or not hasattr(self, '_t_arr'):
            return
        if self._view_mode == 'waveform':
            self._plot_waveform()
        else:
            self._plot_histogram()

    def _plot_waveform(self) -> None:
        t_ms  = (self._t_arr - self._t_arr[0]) / 1000.0
        I_arr = self._I_arr
        V_arr = self._V_arr
        I_mean, I_std = np.mean(I_arr), np.std(I_arr)
        V_mean, V_std = np.mean(V_arr), np.std(V_arr)

        self._fig.clear()
        ax1 = self._fig.add_subplot(2, 1, 1)
        ax1.plot(t_ms, I_arr, color='#00CC28', linewidth=0.7, alpha=0.9)
        ax1.axhline(I_mean, color='#007a18', linewidth=0.8, linestyle='--')
        if I_std > 0:
            ax1.axhspan(I_mean - 2*I_std, I_mean + 2*I_std,
                        alpha=0.07, color='#00FF32', label=f'\u00b12\u03c3')
        ax1.set_ylabel('A', color='#404040', fontsize=7)
        self._style_ax(ax1, 'Current', FG_I)

        ax2 = self._fig.add_subplot(2, 1, 2, sharex=ax1)
        ax2.plot(t_ms, V_arr, color='#CCA800', linewidth=0.7, alpha=0.9)
        ax2.axhline(V_mean, color='#887000', linewidth=0.8, linestyle='--')
        if V_std > 0:
            ax2.axhspan(V_mean - 2*V_std, V_mean + 2*V_std,
                        alpha=0.07, color='#FFD700')
        ax2.set_xlabel('Time (ms)', color='#444444', fontsize=7)
        ax2.set_ylabel('V', color='#404040', fontsize=7)
        self._style_ax(ax2, 'Voltage', FG_V)

        self._fig.tight_layout(pad=1.2)
        self._canvas.draw()

    def _plot_histogram(self) -> None:
        I_arr = self._I_arr
        V_arr = self._V_arr
        I_mean, I_std = np.mean(I_arr), np.std(I_arr)
        V_mean, V_std = np.mean(V_arr), np.std(V_arr)
        bins = min(80, max(10, len(I_arr) // 8))

        self._fig.clear()

        ax1 = self._fig.add_subplot(1, 2, 1)
        ax1.hist(I_arr, bins=bins, color='#00CC28', alpha=0.75, density=True,
                 edgecolor='none')
        if I_std > 0:
            x = np.linspace(I_mean - 4*I_std, I_mean + 4*I_std, 300)
            ax1.plot(x, np.exp(-0.5*((x-I_mean)/I_std)**2) / (I_std*(2*np.pi)**0.5),
                     color='#aaffaa', linewidth=1.2, linestyle='--', label='Gaussian')
        ax1.set_xlabel('A', color='#444444', fontsize=7)
        ax1.set_ylabel('Density', color='#404040', fontsize=7)
        self._style_ax(ax1, 'Current Distribution', FG_I)

        ax2 = self._fig.add_subplot(1, 2, 2)
        ax2.hist(V_arr, bins=bins, color='#CCA800', alpha=0.75, density=True,
                 edgecolor='none')
        if V_std > 0:
            x = np.linspace(V_mean - 4*V_std, V_mean + 4*V_std, 300)
            ax2.plot(x, np.exp(-0.5*((x-V_mean)/V_std)**2) / (V_std*(2*np.pi)**0.5),
                     color='#ffeebb', linewidth=1.2, linestyle='--', label='Gaussian')
        ax2.set_xlabel('V', color='#444444', fontsize=7)
        ax2.set_ylabel('Density', color='#404040', fontsize=7)
        self._style_ax(ax2, 'Voltage Distribution', FG_V)

        self._fig.tight_layout(pad=1.2)
        self._canvas.draw()

    # ── View toggle ───────────────────────────────────────────────────────────

    def _set_view(self, mode: str) -> None:
        self._view_mode = mode
        self._update_view_btns()
        self._redraw_plot()

    def _update_view_btns(self) -> None:
        if self._view_mode == 'waveform':
            self._view_wave_btn.config(bg=BG_TAB_ON, fg=FG_I)
            self._view_hist_btn.config(bg=BG_TAB_OFF, fg=FG_LABEL)
        else:
            self._view_wave_btn.config(bg=BG_TAB_OFF, fg=FG_LABEL)
            self._view_hist_btn.config(bg=BG_TAB_ON, fg=FG_V)

    # ── Capture control ───────────────────────────────────────────────────────

    def _start_capture(self) -> None:
        if not (self._ser and self._running):
            messagebox.showwarning('Not connected', 'Connect to the device first.')
            return
        try:
            n      = int(self._cap_n_var.get())
            smooth = int(self._cap_smooth_var.get())
        except ValueError:
            return
        n = max(50, min(2000, n))
        cmd = f'!CAPTURE,{n}' if smooth == 0 else f'!CAPTURE,{n},{smooth}'
        self._cap_buf.clear()
        self._cap_expected = n
        self._capturing    = True
        self._set_capture_ui(True)
        self._cap_prog_var.set('Starting...')
        self._send(cmd)

    def _set_capture_ui(self, capturing: bool) -> None:
        self._start_btn.config(state='disabled' if capturing else 'normal')
        self._abort_btn.config(state='normal'   if capturing else 'disabled')

    # ── CSV export ────────────────────────────────────────────────────────────

    def _save_csv(self) -> None:
        if not self._cap_buf:
            messagebox.showinfo('No data', 'Run a capture first.')
            return
        path = filedialog.asksaveasfilename(
            defaultextension='.csv',
            filetypes=[('CSV files', '*.csv'), ('All files', '*.*')],
            initialfile='amc_capture.csv')
        if not path:
            return
        t0 = self._cap_buf[0]['t']
        with open(path, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['t_us_rel', 'I_A', 'V_V'])
            for s in self._cap_buf:
                w.writerow([s['t'] - t0, s['I'], s['V']])
        self._status_lbl.config(text=f'Saved {path}')

    # ── Serial send ───────────────────────────────────────────────────────────

    def _send(self, cmd: str) -> None:
        if self._ser and self._running:
            try:
                self._ser.write((cmd + '\n').encode('ascii'))
            except serial.SerialException:
                pass


# ── Helpers ───────────────────────────────────────────────────────────────────

def _apply_dark_combobox() -> None:
    style = ttk.Style()
    style.theme_use('default')
    style.configure('TCombobox',
                    fieldbackground='#2a2a2a', background='#2a2a2a',
                    foreground='#888888', selectbackground='#2a2a2a',
                    selectforeground='#888888', arrowcolor='#555555')


def main() -> None:
    ap = argparse.ArgumentParser(description='AMC Precision Meter GUI')
    ap.add_argument('--port', help='Serial port  e.g. COM3 or /dev/ttyACM0')
    ap.add_argument('--baud', type=int, default=BAUD_DEFAULT)
    args = ap.parse_args()

    root = tk.Tk()
    _apply_dark_combobox()

    app = AMCGui(root, initial_port=args.port)
    if args.port:
        app._baud_var.set(str(args.baud))
        app._connect()

    root.mainloop()


if __name__ == '__main__':
    main()
