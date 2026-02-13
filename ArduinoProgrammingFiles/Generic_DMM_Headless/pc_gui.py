"""
PC GUI for Generic_DMM_Headless — PyQtGraph-based DMM viewer.

Connects over USB serial to any Arduino running the Generic_DMM_Headless sketch
and displays live voltage readings (DC, AC RMS, min/max, trend).

Dependencies:
    pip install pyqtgraph pyserial PyQt6
"""

import sys
import time
import collections

from PyQt6.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QComboBox, QPushButton, QFrame, QStatusBar,
)
from PyQt6.QtGui import QFont
import pyqtgraph as pg
import serial
import serial.tools.list_ports


# ──────────────────────── Serial Reader Thread ────────────────────────

class SerialReader(QThread):
    """Reads lines from the serial port and emits parsed messages as signals."""

    dmm_reading = pyqtSignal(dict)
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
        if line.startswith("$DMM,"):
            self._parse_dmm(line)
        elif line.startswith("$CFG,"):
            self._parse_cfg(line)

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
        self.ctrl_frame = ctrl_frame = QFrame()
        ctrl_frame.setStyleSheet("background: #252530; border-radius: 4px;")
        ctrl_layout = QHBoxLayout(ctrl_frame)
        ctrl_layout.setContentsMargins(6, 4, 6, 4)

        btn_defs = [
            ("Rst Min/Max", "!RST",  "#2e8b57"),
            ("Set Ref",     "!REF",  "#2e8b57"),
            ("VAC",         "!VAC",  "#00c8ff"),
            ("Set Zero",    "!ZERO", "#ffa500"),
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
        self.stats_frame = stats_frame = QFrame()
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

        # Trend plot (stored as instance var for compact toggle)
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
        self.setWindowTitle("Generic DMM — PC Viewer")
        self.resize(800, 600)
        self.reader = None
        self.cfg = None
        self._latest_dmm = None
        self._compact = False
        self._full_size = None  # remembered size before going compact

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
        self.conn_frame = QFrame()
        conn_bar = QHBoxLayout(self.conn_frame)
        conn_bar.setContentsMargins(4, 4, 4, 4)

        self.port_label = QLabel("Port:")
        conn_bar.addWidget(self.port_label)
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(180)
        conn_bar.addWidget(self.port_combo)

        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self._refresh_ports)
        conn_bar.addWidget(self.refresh_btn)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._toggle_connect)
        conn_bar.addWidget(self.connect_btn)

        self.conn_status = QLabel("Disconnected")
        self.conn_status.setStyleSheet("color: #aaa;")
        conn_bar.addWidget(self.conn_status)
        conn_bar.addStretch()

        self.compact_btn = QPushButton("Compact")
        self.compact_btn.setStyleSheet(
            "QPushButton { background: #555; color: #dcdcdc; border: 1px solid #666; "
            "border-radius: 4px; padding: 4px 10px; font-weight: bold; }"
            "QPushButton:hover { background: #666; }"
        )
        self.compact_btn.clicked.connect(self._toggle_compact)
        conn_bar.addWidget(self.compact_btn)

        main_layout.addWidget(self.conn_frame)

        # ── DMM panel ──
        self.dmm_tab = DmmTab()
        main_layout.addWidget(self.dmm_tab)

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
        self.reader.dmm_reading.connect(self._on_dmm_reading)
        self.reader.config_received.connect(self._on_config)
        self.reader.connection_lost.connect(self._on_connection_lost)
        self.reader.start()

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
        self._latest_dmm = None
        if self.reader:
            self.reader.stop()
            self.reader.wait(2000)
            self.reader = None
        self.dmm_tab.set_reader(None)
        self.connect_btn.setText("Connect")
        self.conn_status.setText("Disconnected")
        self.conn_status.setStyleSheet("color: #aaa;")
        self.status_bar.showMessage("Disconnected")

    def _on_connection_lost(self):
        self._display_timer.stop()
        self._latest_dmm = None
        self.reader = None
        self.dmm_tab.set_reader(None)
        self.connect_btn.setText("Connect")
        self.conn_status.setText("Connection lost")
        self.conn_status.setStyleSheet("color: #e04040;")
        self.status_bar.showMessage("Connection lost")

    def _send_cmd(self, cmd: str):
        if self.reader:
            self.reader.send_command(cmd)

    # ── Compact mode ──

    def _toggle_compact(self):
        self._compact = not self._compact
        dmm = self.dmm_tab

        if self._compact:
            self._full_size = self.size()
            # Hide controls, stats, trend, connection widgets, status bar
            dmm.ctrl_frame.hide()
            dmm.stats_frame.hide()
            dmm.trend_widget.hide()
            self.port_label.hide()
            self.port_combo.hide()
            self.refresh_btn.hide()
            self.connect_btn.hide()
            self.conn_status.hide()
            self.status_bar.hide()
            self.compact_btn.setText("Expand")
            # Shrink to just the voltage readout
            self.resize(340, 140)
        else:
            dmm.ctrl_frame.show()
            dmm.stats_frame.show()
            dmm.trend_widget.show()
            self.port_label.show()
            self.port_combo.show()
            self.refresh_btn.show()
            self.connect_btn.show()
            self.conn_status.show()
            self.status_bar.show()
            self.compact_btn.setText("Compact")
            if self._full_size:
                self.resize(self._full_size)

    # ── Signal handlers ──

    def _on_config(self, cfg: dict):
        self.cfg = cfg
        self.status_bar.showMessage(
            f"Config: probeScale={cfg['probeScale']:.4f}  offset={cfg['amc0330Offset']:.5f}  Vref={cfg['vref']:.2f}"
        )

    def _on_dmm_reading(self, data: dict):
        self._latest_dmm = data

    def _refresh_display(self):
        """Called by timer at ~30fps — renders latest data if available."""
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
