"""
PC GUI for Oscilloscope_Giga — PyQtGraph-based scope and DMM viewer.

Connects over USB serial to an Arduino GIGA running the Oscilloscope_Giga sketch
and displays live waveforms (scope mode) and voltage readings (DMM mode).

Dependencies:
    pip install pyqtgraph pyserial PyQt6
"""

import sys
import time
import collections

from PyQt6.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QComboBox, QPushButton, QTabWidget, QFrame, QStatusBar,
    QDoubleSpinBox,
)
from PyQt6.QtGui import QFont
import pyqtgraph as pg
import serial
import serial.tools.list_ports


# ──────────────────────── Serial Reader Thread ────────────────────────

class SerialReader(QThread):
    """Reads lines from the serial port and emits parsed messages as signals."""

    scope_frame = pyqtSignal(dict)
    dmm_reading = pyqtSignal(dict)
    mode_changed = pyqtSignal(str)
    config_received = pyqtSignal(dict)
    connection_lost = pyqtSignal()

    def __init__(self, port: str, baud: int = 115200):
        super().__init__()
        self.port = port
        self.baud = baud
        self._running = True
        self._ser = None

    def run(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except serial.SerialException:
            self.connection_lost.emit()
            return

        while self._running:
            try:
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="ignore").strip()
                if not line:
                    continue
                self._parse(line)
            except serial.SerialException:
                break
            except Exception:
                continue

        try:
            self._ser.close()
        except Exception:
            pass
        self._ser = None
        self.connection_lost.emit()

    def stop(self):
        self._running = False

    def send_command(self, cmd: str):
        """Send a command string to the Arduino (e.g. '!TB+')."""
        ser = self._ser
        if ser and ser.is_open:
            try:
                ser.write((cmd + "\n").encode("ascii"))
            except serial.SerialException:
                pass

    # ── parsers ──

    def _parse(self, line: str):
        if line.startswith("$SCOPE,"):
            self._parse_scope(line)
        elif line.startswith("$DMM,"):
            self._parse_dmm(line)
        elif line.startswith("$MODE,"):
            mode = line.split(",", 1)[1].strip()
            self.mode_changed.emit(mode)
        elif line.startswith("$CFG,"):
            self._parse_cfg(line)

    def _parse_scope(self, line: str):
        try:
            parts = line.split(",")
            # $SCOPE,sampleRate,tbUsPerDiv,vPerDiv,trigLevel,freq,vpp,vmin,vmax,vrms,nSamples,s0..s619
            data = {
                "sampleRate": float(parts[1]),
                "tbUsPerDiv": int(parts[2]),
                "vPerDiv": float(parts[3]),
                "trigLevel": float(parts[4]),
                "freq": float(parts[5]),
                "vpp": float(parts[6]),
                "vmin": float(parts[7]),
                "vmax": float(parts[8]),
                "vrms": float(parts[9]),
                "nSamples": int(parts[10]),
            }
            n = data["nSamples"]
            data["samples"] = [int(parts[11 + i]) for i in range(n)]
            self.scope_frame.emit(data)
        except (IndexError, ValueError):
            pass

    def _parse_dmm(self, line: str):
        try:
            parts = line.split(",")
            data = {
                "avg": float(parts[1]),
                "min": float(parts[2]),
                "max": float(parts[3]),
                "vrms": float(parts[4]),
                "refV": float(parts[5]),
                "refSet": int(parts[6]) != 0,
                "showVac": int(parts[7]) != 0,
            }
            self.dmm_reading.emit(data)
        except (IndexError, ValueError):
            pass

    def _parse_cfg(self, line: str):
        try:
            parts = line.split(",")
            cfg = {
                "probeScale": float(parts[1]),
                "amc0330Offset": float(parts[2]),
                "vref": float(parts[3]),
                "adcMax": float(parts[4]),
                "plotW": int(parts[5]),
            }
            self.config_received.emit(cfg)
        except (IndexError, ValueError):
            pass


# ──────────────────────── Scope Tab ────────────────────────

class ScopeTab(QWidget):
    _CTRL_BTN_STYLE = (
        "QPushButton {{ background: {bg}; color: #dcdcdc; border: 1px solid #666; "
        "border-radius: 4px; padding: 4px 10px; font-weight: bold; }}"
        "QPushButton:hover {{ background: {hover}; }}"
    )

    def __init__(self):
        super().__init__()
        self.cfg = None  # will be set from $CFG
        self._reader = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        # ── Control bar ──
        ctrl_frame = QFrame()
        ctrl_frame.setStyleSheet("background: #252530; border-radius: 4px;")
        ctrl_layout = QHBoxLayout(ctrl_frame)
        ctrl_layout.setContentsMargins(6, 4, 6, 4)

        btn_defs = [
            ("TB<<",    "!TB-",  "#468cff"),
            ("TB>>",    "!TB+",  "#468cff"),
            ("V/d+",    "!VD+",  "#32c85a"),
            ("V/d-",    "!VD-",  "#32c85a"),
            ("Ofs Up",  "!OFS+", "#ffa500"),
            ("Ofs Dn",  "!OFS-", "#ffa500"),
            ("RUN/STOP","!RUN",  "#dc3c3c"),
            ("Auto",    "!AUTO", "#00c8ff"),
        ]
        for label, cmd, color in btn_defs:
            btn = QPushButton(label)
            hover = color.replace("ff", "cc") if len(color) == 7 else color
            btn.setStyleSheet(self._CTRL_BTN_STYLE.format(bg=color, hover=hover))
            btn.clicked.connect(lambda checked, c=cmd: self._send(c))
            ctrl_layout.addWidget(btn)

        # Trigger level spinbox
        ctrl_layout.addWidget(QLabel("Trig:"))
        self.trig_spin = QDoubleSpinBox()
        self.trig_spin.setRange(-50.0, 50.0)
        self.trig_spin.setSingleStep(0.1)
        self.trig_spin.setDecimals(2)
        self.trig_spin.setValue(0.0)
        self.trig_spin.setSuffix(" V")
        self.trig_spin.setStyleSheet(
            "QDoubleSpinBox { background: #2a2a2a; color: #ffdc00; "
            "border: 1px solid #666; border-radius: 3px; padding: 2px 6px; }"
        )
        self.trig_spin.valueChanged.connect(self._on_trig_changed)
        ctrl_layout.addWidget(self.trig_spin)

        layout.addWidget(ctrl_frame)

        # Plot
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground("#1a1a1a")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.25)
        self.plot_widget.setLabel("left", "Voltage", units="V")
        self.plot_widget.setLabel("bottom", "Time", units="s")
        self.curve = self.plot_widget.plot(pen=pg.mkPen("#00ff32", width=2))

        # Trigger level line
        self.trig_line = pg.InfiniteLine(
            pos=0, angle=0,
            pen=pg.mkPen("#ffdc00", width=1, style=Qt.PenStyle.DashLine),
        )
        self.plot_widget.addItem(self.trig_line)

        layout.addWidget(self.plot_widget, stretch=1)

        # Measurements bar
        meas_frame = QFrame()
        meas_frame.setStyleSheet("background: #222; border-radius: 4px;")
        meas_layout = QHBoxLayout(meas_frame)
        meas_layout.setContentsMargins(8, 4, 8, 4)

        self.meas_labels = {}
        for name in ("Freq", "Vpp", "Vmin", "Vmax", "Vrms", "TB", "V/div"):
            lbl = QLabel(f"{name}: ---")
            lbl.setFont(QFont("Consolas", 11))
            lbl.setStyleSheet("color: #ddd;")
            meas_layout.addWidget(lbl)
            self.meas_labels[name] = lbl

        layout.addWidget(meas_frame)

    def set_reader(self, reader):
        self._reader = reader

    def _send(self, cmd: str):
        if self._reader:
            self._reader.send_command(cmd)

    def _on_trig_changed(self, val: float):
        self._send(f"!TRIG,{val:.4f}")

    def update_frame(self, data: dict):
        cfg = self.cfg
        if cfg is None:
            return

        vref = cfg["vref"]
        adc_max = cfg["adcMax"]
        probe_scale = cfg["probeScale"]
        amc_offset = cfg["amc0330Offset"]
        samples = data["samples"]
        n = len(samples)
        sr = data["sampleRate"]

        # Convert raw ADC samples to offset-corrected probe-tip voltage
        # Matches dmmToVolts: subtract AMC0330 DC offset before scaling
        voltages = [((s / adc_max) * vref - amc_offset) * probe_scale for s in samples]

        # Time axis
        if sr > 0:
            dt = 1.0 / sr
        else:
            dt = 1e-6
        times = [i * dt for i in range(n)]

        self.curve.setData(times, voltages)

        # Trigger line (Arduino now sends offset-corrected probe-tip voltage)
        self.trig_line.setValue(data["trigLevel"])

        # Sync trigger spinbox
        self.trig_spin.blockSignals(True)
        self.trig_spin.setValue(data["trigLevel"])
        self.trig_spin.blockSignals(False)

        # Measurements
        tb_us = data["tbUsPerDiv"]
        if tb_us < 1000:
            tb_str = f"{tb_us}us"
        elif tb_us < 1_000_000:
            tb_str = f"{tb_us // 1000}ms"
        else:
            tb_str = f"{tb_us // 1_000_000}s"

        # Measurements (Arduino now sends offset-corrected values)
        self.meas_labels["Freq"].setText(f"Freq: {data['freq']:.1f} Hz")
        self.meas_labels["Vpp"].setText(f"Vpp: {data['vpp']:.3f} V")
        self.meas_labels["Vmin"].setText(f"Vmin: {data['vmin']:.3f} V")
        self.meas_labels["Vmax"].setText(f"Vmax: {data['vmax']:.3f} V")
        self.meas_labels["Vrms"].setText(f"Vrms: {data['vrms']:.3f} V")
        self.meas_labels["TB"].setText(f"TB: {tb_str}/div")
        self.meas_labels["V/div"].setText(f"V/d: {data['vPerDiv']:.1f} V")


# ──────────────────────── DMM Tab ────────────────────────

class DmmTab(QWidget):
    TREND_SECONDS = 60
    _CTRL_BTN_STYLE = (
        "QPushButton {{ background: {bg}; color: #dcdcdc; border: 1px solid #666; "
        "border-radius: 4px; padding: 4px 10px; font-weight: bold; }}"
        "QPushButton:hover {{ background: {hover}; }}"
    )

    def __init__(self):
        super().__init__()
        self.trend_data = collections.deque(maxlen=240)  # ~60s at 4Hz
        self.trend_times = collections.deque(maxlen=240)
        self._t0 = None
        self._reader = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)

        # ── Control bar ──
        ctrl_frame = QFrame()
        ctrl_frame.setStyleSheet("background: #252530; border-radius: 4px;")
        ctrl_layout = QHBoxLayout(ctrl_frame)
        ctrl_layout.setContentsMargins(6, 4, 6, 4)

        btn_defs = [
            ("Rst Min/Max", "!RST",  "#2e8b57"),
            ("Set Ref",     "!REF",  "#2e8b57"),
            ("VAC",         "!VAC",  "#00c8ff"),
        ]
        for label, cmd, color in btn_defs:
            btn = QPushButton(label)
            hover = color.replace("ff", "cc") if len(color) == 7 else color
            btn.setStyleSheet(self._CTRL_BTN_STYLE.format(bg=color, hover=hover))
            btn.clicked.connect(lambda checked, c=cmd: self._send(c))
            ctrl_layout.addWidget(btn)

        ctrl_layout.addStretch()

        # Calibration controls
        ctrl_layout.addWidget(QLabel("Cal:"))
        self.cal_combo = QComboBox()
        for v in ("1.0", "5.0", "10.0", "20.0"):
            self.cal_combo.addItem(f"{v} V", float(v))
        self.cal_combo.setStyleSheet(
            "QComboBox { background: #2a2a2a; color: #ffa500; "
            "border: 1px solid #666; border-radius: 3px; padding: 2px 6px; }"
        )
        ctrl_layout.addWidget(self.cal_combo)

        cal_btn = QPushButton("Apply Cal")
        cal_btn.setStyleSheet(self._CTRL_BTN_STYLE.format(bg="#ffa500", hover="#cc8400"))
        cal_btn.clicked.connect(self._on_apply_cal)
        ctrl_layout.addWidget(cal_btn)

        layout.addWidget(ctrl_frame)

        # Big voltage readout
        self.big_label = QLabel("+0.0000 V")
        self.big_label.setFont(QFont("Consolas", 52, QFont.Weight.Bold))
        self.big_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.big_label.setStyleSheet("color: #00ff32;")
        layout.addWidget(self.big_label, stretch=2)

        # Mode sub-label (VDC / VAC / DELTA)
        self.mode_label = QLabel("VDC")
        self.mode_label.setFont(QFont("Consolas", 16))
        self.mode_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.mode_label.setStyleSheet("color: #8c8c96;")
        layout.addWidget(self.mode_label)

        # Stats row
        stats_frame = QFrame()
        stats_frame.setStyleSheet("background: #222; border-radius: 4px;")
        stats_layout = QHBoxLayout(stats_frame)
        stats_layout.setContentsMargins(12, 6, 12, 6)

        self.stat_labels = {}
        for name in ("Min", "Max", "VRMS", "Delta"):
            lbl = QLabel(f"{name}: ---")
            lbl.setFont(QFont("Consolas", 13))
            lbl.setStyleSheet("color: #ddd;")
            stats_layout.addWidget(lbl)
            self.stat_labels[name] = lbl

        layout.addWidget(stats_frame)

        # Trend plot
        self.trend_widget = pg.PlotWidget(title="Trend (60s)")
        self.trend_widget.setBackground("#1a1a1a")
        self.trend_widget.showGrid(x=True, y=True, alpha=0.2)
        self.trend_widget.setLabel("left", "Voltage", units="V")
        self.trend_widget.setLabel("bottom", "Time", units="s")
        self.trend_curve = self.trend_widget.plot(pen=pg.mkPen("#00ff32", width=2))
        layout.addWidget(self.trend_widget, stretch=1)

    def update_reading(self, data: dict):
        avg = data["avg"]
        vmin = data["min"]
        vmax = data["max"]
        vrms = data["vrms"]
        ref_v = data["refV"]
        ref_set = data["refSet"]
        show_vac = data["showVac"]

        # Determine big reading
        if ref_set:
            big_value = avg - ref_v
            big_color = "#ffdc00"
            mode_text = "DELTA"
        elif show_vac:
            big_value = vrms
            big_color = "#00c8ff"
            mode_text = "VAC (RMS)"
        else:
            big_value = avg
            big_color = "#00ff32"
            mode_text = "VDC"

        self.big_label.setText(f"{big_value:+.4f} V")
        self.big_label.setStyleSheet(f"color: {big_color};")
        self.mode_label.setText(mode_text)

        # Stats
        self.stat_labels["Min"].setText(f"Min: {vmin:.4f} V")
        self.stat_labels["Max"].setText(f"Max: {vmax:.4f} V")
        self.stat_labels["VRMS"].setText(f"VRMS: {vrms:.4f} V")
        if ref_set:
            self.stat_labels["Delta"].setText(f"Ref: {ref_v:+.4f} V")
        else:
            self.stat_labels["Delta"].setText("Ref: ---")

        # Trend
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        self.trend_times.append(now - self._t0)
        self.trend_data.append(avg)
        self.trend_curve.setData(list(self.trend_times), list(self.trend_data))

    def set_reader(self, reader):
        self._reader = reader

    def _send(self, cmd: str):
        if self._reader:
            self._reader.send_command(cmd)

    def _on_apply_cal(self):
        val = self.cal_combo.currentData()
        if val is not None:
            self._send(f"!CAL,{val:.1f}")


# ──────────────────────── Main Window ────────────────────────

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Oscilloscope GIGA — PC Viewer")
        self.resize(1000, 700)
        self.reader = None
        self.cfg = None
        self._latest_scope = None
        self._latest_dmm = None

        # Display refresh timer — decouples serial data rate from rendering
        self._display_timer = QTimer(self)
        self._display_timer.setInterval(33)  # ~30 fps
        self._display_timer.timeout.connect(self._refresh_display)

        # Central widget
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(4, 4, 4, 4)

        # ── Connection toolbar ──
        conn_bar = QHBoxLayout()

        conn_bar.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(180)
        conn_bar.addWidget(self.port_combo)

        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self._refresh_ports)
        conn_bar.addWidget(self.refresh_btn)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._toggle_connect)
        conn_bar.addWidget(self.connect_btn)

        self.mode_btn = QPushButton("Mode")
        self.mode_btn.setStyleSheet(
            "QPushButton { background: #c864ff; color: #dcdcdc; border: 1px solid #666; "
            "border-radius: 4px; padding: 6px 16px; font-weight: bold; }"
            "QPushButton:hover { background: #a050d0; }"
        )
        self.mode_btn.clicked.connect(lambda: self._send_cmd("!MODE"))
        conn_bar.addWidget(self.mode_btn)

        self.conn_status = QLabel("Disconnected")
        self.conn_status.setStyleSheet("color: #aaa;")
        conn_bar.addWidget(self.conn_status)
        conn_bar.addStretch()

        main_layout.addLayout(conn_bar)

        # ── Tabs ──
        self.tabs = QTabWidget()
        self.scope_tab = ScopeTab()
        self.dmm_tab = DmmTab()
        self.tabs.addTab(self.scope_tab, "Scope")
        self.tabs.addTab(self.dmm_tab, "DMM")
        main_layout.addWidget(self.tabs)

        # ── Status bar ──
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Ready — select a port and click Connect")

        # Populate ports
        self._refresh_ports()

    # ── Port management ──

    def _refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in sorted(ports, key=lambda x: x.device):
            desc = f"{p.device} — {p.description}" if p.description != p.device else p.device
            self.port_combo.addItem(desc, p.device)

    # ── Connect / Disconnect ──

    def _toggle_connect(self):
        if self.reader is not None:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_combo.currentData()
        if not port:
            self.status_bar.showMessage("No port selected")
            return

        self.reader = SerialReader(port)
        self.reader.scope_frame.connect(self._on_scope_frame)
        self.reader.dmm_reading.connect(self._on_dmm_reading)
        self.reader.mode_changed.connect(self._on_mode_changed)
        self.reader.config_received.connect(self._on_config)
        self.reader.connection_lost.connect(self._on_connection_lost)
        self.reader.start()

        self.scope_tab.set_reader(self.reader)
        self.dmm_tab.set_reader(self.reader)
        self._display_timer.start()

        # Request config: immediately (if Arduino is already running) and
        # after 2s delay (if Arduino reset on connect and is still booting)
        QTimer.singleShot(100, lambda: self._send_cmd("!CFG"))
        QTimer.singleShot(2000, lambda: self._send_cmd("!CFG"))

        self.connect_btn.setText("Disconnect")
        self.conn_status.setText(f"Connected: {port}")
        self.conn_status.setStyleSheet("color: #00ff32;")
        self.status_bar.showMessage(f"Connected to {port}")

    def _disconnect(self):
        self._display_timer.stop()
        self._latest_scope = None
        self._latest_dmm = None
        if self.reader:
            self.reader.stop()
            self.reader.wait(2000)
            self.reader = None
        self.scope_tab.set_reader(None)
        self.dmm_tab.set_reader(None)
        self.connect_btn.setText("Connect")
        self.conn_status.setText("Disconnected")
        self.conn_status.setStyleSheet("color: #aaa;")
        self.status_bar.showMessage("Disconnected")

    def _on_connection_lost(self):
        self._display_timer.stop()
        self._latest_scope = None
        self._latest_dmm = None
        self.reader = None
        self.scope_tab.set_reader(None)
        self.dmm_tab.set_reader(None)
        self.connect_btn.setText("Connect")
        self.conn_status.setText("Connection lost")
        self.conn_status.setStyleSheet("color: #e04040;")
        self.status_bar.showMessage("Connection lost")

    def _send_cmd(self, cmd: str):
        if self.reader:
            self.reader.send_command(cmd)

    # ── Signal handlers ──

    def _on_config(self, cfg: dict):
        self.cfg = cfg
        self.scope_tab.cfg = cfg
        self.status_bar.showMessage(
            f"Config: probeScale={cfg['probeScale']:.2f}  Vref={cfg['vref']:.2f}  plotW={cfg['plotW']}"
        )

    def _on_mode_changed(self, mode: str):
        if mode == "SCOPE":
            self.tabs.setCurrentWidget(self.scope_tab)
            self.status_bar.showMessage("Mode: Scope")
        elif mode == "DMM":
            self.tabs.setCurrentWidget(self.dmm_tab)
            self.dmm_tab._t0 = None
            self.dmm_tab.trend_data.clear()
            self.dmm_tab.trend_times.clear()
            self.status_bar.showMessage("Mode: DMM")

    def _on_scope_frame(self, data: dict):
        self._latest_scope = data

    def _on_dmm_reading(self, data: dict):
        self._latest_dmm = data

    def _refresh_display(self):
        """Called by timer at ~30fps — renders latest data if available."""
        scope = self._latest_scope
        if scope is not None:
            self._latest_scope = None
            self.scope_tab.update_frame(scope)
        dmm = self._latest_dmm
        if dmm is not None:
            self._latest_dmm = None
            self.dmm_tab.update_reading(dmm)

    # ── Close ──

    def closeEvent(self, event):
        self._disconnect()
        event.accept()


# ──────────────────────── Dark Theme ────────────────────────

DARK_STYLESHEET = """
QMainWindow, QWidget {
    background-color: #1a1a1a;
    color: #dcdcdc;
}
QLabel {
    color: #dcdcdc;
}
QComboBox {
    background: #2a2a2a;
    color: #dcdcdc;
    border: 1px solid #444;
    border-radius: 3px;
    padding: 4px 8px;
}
QComboBox::drop-down {
    border: none;
}
QComboBox QAbstractItemView {
    background: #2a2a2a;
    color: #dcdcdc;
    selection-background-color: #444;
}
QPushButton {
    background: #333;
    color: #dcdcdc;
    border: 1px solid #555;
    border-radius: 4px;
    padding: 6px 16px;
}
QPushButton:hover {
    background: #444;
}
QPushButton:pressed {
    background: #555;
}
QTabWidget::pane {
    border: 1px solid #444;
    background: #1a1a1a;
}
QTabBar::tab {
    background: #2a2a2a;
    color: #aaa;
    border: 1px solid #444;
    padding: 6px 20px;
    margin-right: 2px;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
}
QTabBar::tab:selected {
    background: #1a1a1a;
    color: #00ff32;
    border-bottom: 1px solid #1a1a1a;
}
QStatusBar {
    background: #222;
    color: #aaa;
}
"""


# ──────────────────────── Entry Point ────────────────────────

def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(DARK_STYLESHEET)

    # Set default pyqtgraph options
    pg.setConfigOptions(antialias=True)

    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
