"""
PC GUI for PixelShieldController — PyQt6 NeoPixel Shield controller.

Connects over USB serial to an Arduino UNO R4 Minima running
PixelShieldController.ino and lets you pick solid colours or start a
rainbow cycle at a controllable speed.

Dependencies:
    pip install PyQt6 pyserial
"""

import sys
import math
import colorsys
import threading

from PyQt6.QtCore import Qt, QPointF, QRectF, QTimer, pyqtSignal, QObject
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QComboBox, QPushButton, QFrame, QSlider, QSpinBox,
    QStatusBar, QSizePolicy,
)
from PyQt6.QtGui import (
    QPainter, QConicalGradient, QRadialGradient,
    QColor, QBrush, QPen, QFont,
)
import serial
import serial.tools.list_ports


# ──────────────────────── Colour Wheel Widget ─────────────────────────

class ColorWheelWidget(QWidget):
    """
    Interactive HSV colour wheel drawn with QPainter.

    Hues (0–360°) sweep around the circumference via a QConicalGradient.
    Saturation fades from white at the centre to full colour at the edge
    via a QRadialGradient overlay.  Brightness is fixed at 1.0.

    Click or drag anywhere on the disc to pick a colour.
    The supplied callback is called with (r, g, b) ints 0–255.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(260, 260)
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Expanding,
        )
        self._hue = 0.0    # 0–360°
        self._sat = 1.0    # 0–1
        self._dragging = False
        self._callback = None

    def set_callback(self, fn):
        """fn(r, g, b) — called whenever the user selects a new colour."""
        self._callback = fn

    # ── Public helpers ──

    @property
    def current_rgb(self):
        r, g, b = colorsys.hsv_to_rgb(self._hue / 360.0, self._sat, 1.0)
        return int(r * 255), int(g * 255), int(b * 255)

    def set_color_rgb(self, r, g, b):
        """Move the indicator to the given colour without firing the callback."""
        h, s, _v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
        self._hue = h * 360.0
        self._sat = s
        self.update()

    # ── Internal geometry ──

    def _cx_cy(self):
        return self.width() / 2.0, self.height() / 2.0

    def _radius(self):
        return min(self.width(), self.height()) / 2.0 - 6

    # ── Paint ──

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        cx, cy = self._cx_cy()
        r = self._radius()
        rect = QRectF(cx - r, cy - r, 2 * r, 2 * r)

        # 1. Hue ring — conical gradient, 13 stops so colour[12]==colour[0]
        hue_grad = QConicalGradient(cx, cy, 0)
        for i in range(13):
            hue_grad.setColorAt(
                i / 12.0,
                QColor.fromHsvF((i * 30 % 360) / 360.0, 1.0, 1.0),
            )
        painter.setBrush(QBrush(hue_grad))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(rect)

        # 2. Saturation overlay — white centre → transparent edge
        sat_grad = QRadialGradient(cx, cy, r)
        sat_grad.setColorAt(0.0, QColor(255, 255, 255, 255))
        sat_grad.setColorAt(1.0, QColor(255, 255, 255, 0))
        painter.setBrush(QBrush(sat_grad))
        painter.drawEllipse(rect)

        # 3. Selection indicator at current hue/sat position
        # QConicalGradient sweeps CCW on screen (screen Y points down),
        # so the indicator must also go CCW: cos stays the same, sin is negated.
        sx = cx + self._sat * r * math.cos(math.radians(self._hue))
        sy = cy - self._sat * r * math.sin(math.radians(self._hue))
        pt = QPointF(sx, sy)

        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.setPen(QPen(QColor(0, 0, 0), 2))
        painter.drawEllipse(pt, 14.0, 14.0)
        painter.setPen(QPen(QColor(255, 255, 255), 3))
        painter.drawEllipse(pt, 11.0, 11.0)

    # ── Mouse input ──

    def _handle(self, x, y):
        cx, cy = self._cx_cy()
        r = self._radius()
        dx, dy = x - cx, y - cy
        dist = math.sqrt(dx * dx + dy * dy)
        if dist > r:
            return
        # Negate dy so angle increases CCW on screen, matching QConicalGradient
        self._hue = math.degrees(math.atan2(-dy, dx)) % 360.0
        self._sat = dist / r
        self.update()
        if self._callback:
            self._callback(*self.current_rgb)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._dragging = True
            self._handle(event.position().x(), event.position().y())

    def mouseMoveEvent(self, event):
        if self._dragging:
            self._handle(event.position().x(), event.position().y())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._dragging = False


# ──────────────────────── Background Serial Writer ───────────────────

class _WriterSignals(QObject):
    connection_lost = pyqtSignal()


class _SerialWriter:
    """
    Sends commands to the serial port from a background daemon thread so
    the GUI thread never blocks on a write.

    Only the most-recent pending command is sent — intermediate colour
    updates are dropped if they arrive faster than the port can consume
    them.  Control commands (OFF, R:hz) are sent immediately by bypassing
    the pending slot.
    """

    def __init__(self):
        self.signals = _WriterSignals()
        self._ser = None
        self._ser_lock = threading.Lock()
        self._pending = None
        self._pending_lock = threading.Lock()
        self._event = threading.Event()
        self._alive = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def attach(self, ser):
        with self._ser_lock:
            self._ser = ser

    def detach(self):
        with self._ser_lock:
            self._ser = None

    def send(self, cmd: str):
        """Queue a command; replaces any not-yet-sent previous command."""
        with self._pending_lock:
            self._pending = cmd
        self._event.set()

    def send_now(self, cmd: str):
        """Send immediately, bypassing the pending slot (for OFF / mode changes)."""
        with self._pending_lock:
            self._pending = cmd   # also update pending so retries are correct
        self._event.set()

    def send_batch(self, cmds: list):
        """Queue multiple commands joined with newlines; all arrive as one write."""
        with self._pending_lock:
            self._pending = "\n".join(cmds)
        self._event.set()

    def stop(self):
        self._alive = False
        self._event.set()
        self._thread.join(timeout=1.0)

    def _run(self):
        while self._alive:
            self._event.wait(timeout=0.2)
            self._event.clear()
            with self._pending_lock:
                cmd = self._pending
                self._pending = None
            if not cmd:
                continue
            with self._ser_lock:
                ser = self._ser
            if ser and ser.is_open:
                try:
                    ser.write((cmd + "\n").encode("ascii"))
                except Exception:
                    with self._ser_lock:
                        self._ser = None
                    self.signals.connection_lost.emit()


# ──────────────────────── Main Window ────────────────────────────────

class PixelShieldWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Pixel Shield Controller")
        self.resize(780, 620)

        self._ser = None
        self._writer = _SerialWriter()
        self._writer.signals.connection_lost.connect(self._on_connection_lost)
        self._mode = "SOLID"     # "SOLID" or "RAINBOW"
        self._rgb = (255, 34, 0) # last solid colour picked

        # ── Central layout ──
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(14)

        # ── Left column: connection bar + colour wheel ──
        left = QVBoxLayout()
        left.setSpacing(6)

        # Connection bar
        conn_row = QHBoxLayout()
        conn_row.setSpacing(6)
        conn_row.addWidget(QLabel("Port:"))

        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(170)
        conn_row.addWidget(self.port_combo)

        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self._refresh_ports)
        conn_row.addWidget(refresh_btn)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._toggle_connect)
        conn_row.addWidget(self.connect_btn)

        self.conn_label = QLabel("Disconnected")
        self.conn_label.setStyleSheet("color: #888;")
        conn_row.addWidget(self.conn_label)
        conn_row.addStretch()
        left.addLayout(conn_row)

        # Colour wheel
        self.wheel = ColorWheelWidget()
        self.wheel.set_callback(self._on_wheel_color)
        left.addWidget(self.wheel, stretch=1)

        root.addLayout(left, stretch=1)

        # ── Divider ──
        vline = QFrame()
        vline.setFrameShape(QFrame.Shape.VLine)
        vline.setStyleSheet("color: #333;")
        root.addWidget(vline)

        # ── Right column: controls ──
        right = QVBoxLayout()
        right.setSpacing(10)
        right.setContentsMargins(4, 4, 4, 4)

        # Selected colour swatch + hex value
        right.addWidget(self._section_lbl("Selected Colour"))
        swatch_row = QHBoxLayout()
        swatch_row.setSpacing(10)
        self.swatch = QFrame()
        self.swatch.setFixedSize(40, 40)
        self.hex_label = QLabel()
        self.hex_label.setFont(QFont("Consolas", 16, QFont.Weight.Bold))
        self._set_swatch(*self._rgb)   # needs both swatch and hex_label to exist
        swatch_row.addWidget(self.swatch)
        swatch_row.addWidget(self.hex_label)
        swatch_row.addStretch()
        right.addLayout(swatch_row)

        right.addWidget(self._separator())

        # Mode toggle
        right.addWidget(self._section_lbl("Mode"))
        mode_row = QHBoxLayout()
        mode_row.setSpacing(0)
        self.solid_btn   = QPushButton("SOLID")
        self.rainbow_btn = QPushButton("RAINBOW")
        for btn in (self.solid_btn, self.rainbow_btn):
            btn.setMinimumHeight(38)
            btn.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        self.solid_btn.clicked.connect(lambda: self._set_mode("SOLID"))
        self.rainbow_btn.clicked.connect(lambda: self._set_mode("RAINBOW"))
        mode_row.addWidget(self.solid_btn)
        mode_row.addWidget(self.rainbow_btn)
        right.addLayout(mode_row)

        right.addWidget(self._separator())

        # Speed slider (0.1–10.0 Hz, stored as ints 1–100)
        self.speed_label = QLabel("Speed: 1.0 Hz")
        self.speed_label.setFont(QFont("Consolas", 12))
        right.addWidget(self.speed_label)

        self.hz_slider = QSlider(Qt.Orientation.Horizontal)
        self.hz_slider.setRange(1, 100)   # 1→0.1 Hz, 100→10.0 Hz
        self.hz_slider.setValue(10)       # 1.0 Hz default
        self.hz_slider.setEnabled(False)
        self.hz_slider.valueChanged.connect(self._on_hz_changed)
        right.addWidget(self.hz_slider)

        right.addWidget(self._separator())

        # LED count spinbox
        right.addWidget(self._section_lbl("LED Count"))
        led_row = QHBoxLayout()
        led_row.setSpacing(8)
        self.led_count_spin = QSpinBox()
        self.led_count_spin.setRange(1, 1000)
        self.led_count_spin.setValue(40)
        self.led_count_spin.setFixedWidth(80)
        self.led_count_spin.valueChanged.connect(self._on_led_count_changed)
        led_row.addWidget(self.led_count_spin)
        led_row.addStretch()
        right.addLayout(led_row)

        # Brightness slider (0–255; default 50 ≈ 20%)
        self.brightness_label = QLabel("Brightness: 20%")
        self.brightness_label.setFont(QFont("Consolas", 12))
        right.addWidget(self.brightness_label)

        self.brightness_slider = QSlider(Qt.Orientation.Horizontal)
        self.brightness_slider.setRange(0, 255)
        self.brightness_slider.setValue(50)
        self.brightness_slider.valueChanged.connect(self._on_brightness_changed)
        right.addWidget(self.brightness_slider)

        right.addWidget(self._separator())

        # OFF button
        self.off_btn = QPushButton("OFF")
        self.off_btn.setMinimumHeight(52)
        self.off_btn.setFont(QFont("Arial", 17, QFont.Weight.Bold))
        self.off_btn.setStyleSheet(
            "QPushButton { background: #7a1515; color: #fff; border: 1px solid #a02020; "
            "border-radius: 5px; }"
            "QPushButton:hover { background: #a01c1c; }"
            "QPushButton:pressed { background: #5a0e0e; }"
        )
        self.off_btn.clicked.connect(self._on_off)
        right.addWidget(self.off_btn)

        right.addStretch()
        root.addLayout(right)

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Ready — select a port and click Connect")

        self._refresh_ports()
        self._update_mode_buttons()   # all widgets now exist

    # ── Layout helpers ──

    @staticmethod
    def _section_lbl(text):
        lbl = QLabel(text.upper())
        lbl.setFont(QFont("Arial", 9))
        lbl.setStyleSheet("color: #666; letter-spacing: 1px;")
        return lbl

    @staticmethod
    def _separator():
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet("color: #333;")
        return sep

    # ── Port management ──

    def _refresh_ports(self):
        self.port_combo.clear()
        for p in sorted(serial.tools.list_ports.comports(), key=lambda x: x.device):
            desc = (f"{p.device}  —  {p.description}"
                    if p.description and p.description != p.device
                    else p.device)
            self.port_combo.addItem(desc, p.device)

    # ── Connect / Disconnect ──

    def _toggle_connect(self):
        if self._ser and self._ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_combo.currentData()
        if not port:
            self.status_bar.showMessage("No port selected")
            return
        try:
            self._ser = serial.Serial(port, 115200, timeout=1)
            self._writer.attach(self._ser)
            self.connect_btn.setText("Disconnect")
            self.conn_label.setText(f"Connected: {port}")
            self.conn_label.setStyleSheet("color: #00ff32;")
            self.status_bar.showMessage(f"Connected to {port}")
            # Arduino resets on USB connect — wait for it to finish booting
            QTimer.singleShot(1500, self._send_current_state)
        except serial.SerialException as exc:
            self.status_bar.showMessage(f"Connection failed: {exc}")
            self._ser = None

    def _disconnect(self):
        self._writer.detach()
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self.connect_btn.setText("Connect")
        self.conn_label.setText("Disconnected")
        self.conn_label.setStyleSheet("color: #888;")
        self.status_bar.showMessage("Disconnected")

    def _on_connection_lost(self):
        self._ser = None
        self.connect_btn.setText("Connect")
        self.conn_label.setText("Connection lost")
        self.conn_label.setStyleSheet("color: #e04040;")
        self.status_bar.showMessage("Connection lost")

    # ── Serial write ──

    def _send(self, cmd: str):
        """Queue a command — safe to call from the GUI thread at any rate."""
        self._writer.send(cmd)

    def _send_now(self, cmd: str):
        """Send a priority command (OFF, mode switch) without dropping."""
        self._writer.send_now(cmd)

    def _send_current_state(self):
        # Bundle N:, B:, and the current mode command into one write so none
        # are dropped by the latest-wins pending slot.
        cmds = [
            f"N:{self.led_count_spin.value()}",
            f"B:{self.brightness_slider.value()}",
        ]
        if self._mode == "SOLID":
            r, g, b = self._rgb
            cmds.append(f"C:{r},{g},{b}")
        else:
            hz = self.hz_slider.value() / 10.0
            cmds.append(f"R:{hz:.2f}")
        self._writer.send_batch(cmds)

    # ── Colour wheel ──

    def _on_wheel_color(self, r, g, b):
        self._rgb = (r, g, b)
        self._set_swatch(r, g, b)
        if self._mode == "SOLID":
            self._send(f"C:{r},{g},{b}")
        else:
            # Picking a colour switches automatically to solid mode
            self._set_mode("SOLID")

    def _set_swatch(self, r, g, b):
        self.swatch.setStyleSheet(
            f"background: rgb({r},{g},{b}); "
            "border: 1px solid #555; border-radius: 3px;"
        )
        self.hex_label.setText(f"#{r:02X}{g:02X}{b:02X}")

    # ── Mode toggle ──

    def _set_mode(self, mode: str):
        self._mode = mode
        self._update_mode_buttons()
        if mode == "SOLID":
            r, g, b = self._rgb
            self._send_now(f"C:{r},{g},{b}")
        else:
            hz = self.hz_slider.value() / 10.0
            self._send_now(f"R:{hz:.2f}")

    def _update_mode_buttons(self):
        sel = (
            "QPushButton { background: #1a5fa0; color: #fff; "
            "border: 1px solid #1a5fa0; font-weight: bold; font-size: 12px; "
            "border-radius: 0; padding: 6px 18px; }"
        )
        unsel = (
            "QPushButton { background: #252525; color: #666; "
            "border: 1px solid #3a3a3a; font-size: 12px; "
            "border-radius: 0; padding: 6px 18px; }"
            "QPushButton:hover { background: #2e2e2e; color: #aaa; }"
        )
        self.solid_btn.setStyleSheet(sel if self._mode == "SOLID" else unsel)
        self.rainbow_btn.setStyleSheet(sel if self._mode == "RAINBOW" else unsel)

        rainbow_on = self._mode == "RAINBOW"
        self.hz_slider.setEnabled(rainbow_on)
        self.speed_label.setStyleSheet(
            "color: #dcdcdc;" if rainbow_on else "color: #444;"
        )

    # ── LED count / brightness ──

    def _on_led_count_changed(self, value: int):
        self._send(f"N:{value}")

    def _on_brightness_changed(self, value: int):
        pct = round(value / 255 * 100)
        self.brightness_label.setText(f"Brightness: {pct}%")
        self._send(f"B:{value}")

    # ── Hz slider ──

    def _on_hz_changed(self, value: int):
        hz = value / 10.0
        self.speed_label.setText(f"Speed: {hz:.1f} Hz")
        if self._mode == "RAINBOW":
            self._send(f"R:{hz:.2f}")

    # ── OFF button ──

    def _on_off(self):
        self._send_now("OFF")

    # ── Close ──

    def closeEvent(self, event):
        self._disconnect()
        self._writer.stop()
        event.accept()


# ──────────────────────── Dark Theme ─────────────────────────────────

DARK_STYLESHEET = """
QMainWindow, QWidget {
    background-color: #1a1a1a;
    color: #dcdcdc;
}
QLabel { color: #dcdcdc; }
QComboBox {
    background: #2a2a2a;
    color: #dcdcdc;
    border: 1px solid #444;
    border-radius: 3px;
    padding: 4px 8px;
}
QComboBox::drop-down { border: none; }
QComboBox QAbstractItemView {
    background: #2a2a2a;
    color: #dcdcdc;
    selection-background-color: #444;
}
QPushButton {
    background: #2e2e2e;
    color: #dcdcdc;
    border: 1px solid #555;
    border-radius: 4px;
    padding: 5px 14px;
}
QPushButton:hover  { background: #3a3a3a; }
QPushButton:pressed { background: #505050; }
QSlider::groove:horizontal {
    height: 6px;
    background: #2e2e2e;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: #1a5fa0;
    border: 1px solid #1a5fa0;
    width: 16px;
    height: 16px;
    margin: -5px 0;
    border-radius: 8px;
}
QSlider::sub-page:horizontal          { background: #1a5fa0; border-radius: 3px; }
QSlider::groove:horizontal:disabled   { background: #222; }
QSlider::handle:horizontal:disabled   { background: #383838; border-color: #383838; }
QSlider::sub-page:horizontal:disabled { background: #2a2a2a; }
QStatusBar { background: #141414; color: #888; }
"""


# ──────────────────────── Entry Point ────────────────────────────────

def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(DARK_STYLESHEET)
    win = PixelShieldWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
