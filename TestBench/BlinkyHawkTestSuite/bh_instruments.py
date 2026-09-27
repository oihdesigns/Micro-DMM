"""
bh_instruments.py  --  bench instrument drivers for the BlinkyHawk test suite.

  RelayJig      the RelayMount UNO R4 load selector (USB serial, !MODE / $STATE)
  RigolDG800    Rigol DG852 Pro function generator (SCPI over a raw LAN socket)
  SiglentSDS    Siglent SDS800X HD scope (SCPI over a raw LAN socket) -- a
                Python port of TestBench/Scope/ScopeLib.ps1, and
  write_scope_csv()  which writes the SAME CSV layout as Capture-Scope.ps1, so
                TestBench/Scope/Plot-Capture.ps1 plots a suite capture unchanged.

Every driver has a Sim* twin with the same methods, so a whole sequence can be
dry-run with nothing plugged in (see bh_sim.py for the simulated BlinkyHawk).
All drivers are safe to call from a worker thread; each holds its own lock.
"""

import math
import socket
import struct
import threading
import time

try:
    import serial
except ImportError:          # the GUI reports this; the sims still work
    serial = None

# Loads the relay jig can put across the DUT leads, in firmware names.
# Resistance in ohms (None = open circuit, 0 = short).  FGEN is not a load.
JIG_LOADS = [
    ("SHORT", 0.0),
    ("10K", 10e3),
    ("150K", 150e3),
    ("1M", 1e6),
    ("10.6M", 10.6e6),
    ("OPEN", None),
]
JIG_LOAD_OHMS = dict(JIG_LOADS)


def fmt_ohms(r):
    if r is None:
        return "open"
    if r == 0:
        return "short"
    if r >= 1e6:
        return f"{r / 1e6:g}M"
    if r >= 1e3:
        return f"{r / 1e3:g}k"
    return f"{r:g}R"


class InstrumentError(Exception):
    pass


# ---------------------------------------------------------------------------
# Relay jig
# ---------------------------------------------------------------------------
class RelayJig:
    """RelayMountTestJig firmware: !MODE,<name> -> $STATE,<mode>,<bits>,<load>,<settle>.

    The firmware walks every change through OPEN (break-before-make), so a mode
    change takes a few relay settle times; set_mode() waits for the $STATE that
    confirms the load the DUT actually sees.
    """
    name = "Relay jig"

    def __init__(self):
        self.ser = None
        self.lock = threading.Lock()
        self.ident = ""
        self.mode = None

    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open

    def connect(self, port):
        if serial is None:
            raise InstrumentError("pyserial is not installed (pip install pyserial)")
        self.close()
        self.ser = serial.Serial(port, 115200, timeout=0.1, write_timeout=1.0)
        # Opening the port resets the UNO R4: wait for $BOOT, then identify.
        self._read_until(lambda l: l.startswith("$BOOT"), 2.5, quiet=True)
        self.ident = self._query("!ID", "$ID", 2.0) or "RelayJig (no $ID)"
        st = self._query("!STATE", "$STATE", 2.0)
        if st:
            self.mode = st.split(",")[1]
        return self.ident

    def close(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def _read_until(self, pred, timeout, quiet=False):
        end = time.time() + timeout
        buf = b""
        while time.time() < end:
            chunk = self.ser.read(256)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip()
                if line.startswith("$ERR"):
                    raise InstrumentError(f"relay jig: {line}")
                if pred(line):
                    return line
        if quiet:
            return None
        raise InstrumentError("relay jig: no reply")

    def _query(self, cmd, head, timeout):
        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write((cmd + "\n").encode("ascii"))
            return self._read_until(lambda l: l.startswith(head), timeout, quiet=True)

    def set_mode(self, mode, timeout=5.0):
        if not self.connected:
            raise InstrumentError("relay jig not connected")
        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write(f"!MODE,{mode}\n".encode("ascii"))
            line = self._read_until(lambda l: l.startswith("$STATE"), timeout)
        f = line.split(",")
        # $STATE,<mode>,<bits>,<load>,<settle>: trust the decoded load, not the echo
        if len(f) < 4 or f[3] != mode:
            raise InstrumentError(f"relay jig did not reach {mode}: {line}")
        self.mode = mode
        return line


class SimRelayJig:
    name = "Relay jig (sim)"

    def __init__(self, bench):
        self.bench = bench
        self.ident = "RelayJig SIM"
        self.mode = "FGEN"
        self.connected = True

    def connect(self, port=None):
        return self.ident

    def close(self):
        pass

    def set_mode(self, mode, timeout=5.0):
        time.sleep(0.05)
        self.mode = mode
        self.bench.relay = mode
        return f"$STATE,{mode},sim,{mode},25"


# ---------------------------------------------------------------------------
# Raw-socket SCPI (both LAN instruments)
# ---------------------------------------------------------------------------
class ScpiSocket:
    def __init__(self):
        self.sock = None
        self.lock = threading.RLock()
        self._rx = b""

    @property
    def connected(self):
        return self.sock is not None

    def open(self, host, port, timeout=10.0):
        self.close()
        s = socket.create_connection((host, port), timeout=3.0)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.settimeout(timeout)
        self.sock = s
        self._rx = b""

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None

    def write(self, cmd):
        with self.lock:
            self.sock.sendall((cmd + "\n").encode("ascii"))

    def _fill(self):
        chunk = self.sock.recv(65536)
        if not chunk:
            raise InstrumentError("connection closed by instrument")
        self._rx += chunk

    def query(self, cmd):
        with self.lock:
            self.write(cmd)
            while True:
                # skip empty lines (e.g. the terminator left behind by a block read)
                self._rx = self._rx.lstrip(b"\r\n")
                if b"\n" in self._rx:
                    line, self._rx = self._rx.split(b"\n", 1)
                    return line.decode("ascii", "replace").strip()
                self._fill()

    def _read_exact(self, n):
        while len(self._rx) < n:
            self._fill()
        out, self._rx = self._rx[:n], self._rx[n:]
        return out

    def query_block(self, cmd):
        """IEEE 488.2 definite-length block: #<n><len><data>."""
        with self.lock:
            self.write(cmd)
            while b"#" not in self._rx:
                self._fill()
            self._rx = self._rx[self._rx.index(b"#") + 1:]
            nd = int(self._read_exact(1))
            ln = int(self._read_exact(nd))
            return self._read_exact(ln)


# ---------------------------------------------------------------------------
# Rigol DG852 Pro
# ---------------------------------------------------------------------------
class RigolDG800(ScpiSocket):
    """Rigol DG800 Pro / DG900 Pro over a raw LAN socket.

    Syntax from the DG800 Pro/DG900 Pro Programming Guide (PGB16101-1110):
      :SOURce<n>:APPLy:DC <freq>,<amp>,<offset>   freq/amp are placeholders
      :SOURce<n>:APPLy:SINusoid <freq>,<amp>,<offset>,<phase>
      :OUTPut<n>:LOAD INFinity                    HighZ
      :SOURce<n>:VOLTage:UNIT VPP
      :OUTPut<n>[:STATe] ON|OFF
    Port: 5025.  Confirmed on the bench unit (DG852 Pro, fw 00.01.00.00.22,
    2026-09-27): :SYST:COMM:LAN:CONT? returns 5025 and nothing listens on 5555 or
    5000 -- despite the guide's example reply of 5000.

    THE LOAD SETTING IS THE TRAP.  The generator defaults to a 50 ohm load and
    scales its output to hit the programmed level INTO 50 ohm -- into the
    BlinkyHawk's high-impedance input that is twice the voltage asked for.
    setup() forces HighZ on every connect, and every apply re-checks it.
    """
    name = "Function generator"

    # HighZ range from Table 3.66 (<= 50 MHz): 20 Vpp; |offset|*2 + amp <= 20.
    HIGHZ_MAX_VPP = 20.0

    def __init__(self, channel=1, max_abs_v=10.0):
        super().__init__()
        self.ch = channel
        self.max_abs_v = max_abs_v     # user safety limit on |instantaneous V|
        self.ident = ""
        self.state = "off"

    def connect(self, host, port=5025):
        self.open(host, port)
        self.ident = self.query("*IDN?")
        self.setup()
        return self.ident

    def setup(self):
        ch = self.ch
        self.write(f":OUTP{ch} OFF")
        self.write(f":OUTP{ch}:LOAD INF")
        self.write(f":SOUR{ch}:VOLT:UNIT VPP")
        self.check_error("setup")
        load = self.query(f":OUTP{ch}:LOAD?")
        try:
            if float(load) < 1e30:
                raise InstrumentError(
                    f"generator CH{ch} is still set to a {float(load):g} ohm load, not "
                    "HighZ -- refusing to drive the DUT at the wrong level")
        except ValueError:
            pass
        self.state = "off"

    def check_error(self, what):
        err = self.query(":SYST:ERR?")
        # "0,No error" / "+0,\"No error\""
        code = err.split(",", 1)[0].strip().lstrip("+")
        if code not in ("0", ""):
            raise InstrumentError(f"generator error after {what}: {err}")

    def _guard(self, peak):
        if abs(peak) > self.max_abs_v + 1e-9:
            raise InstrumentError(
                f"{peak:+.3f} V peak exceeds the {self.max_abs_v:g} V safety limit "
                "set on the Test Rig tab")

    def output(self, on):
        self.write(f":OUTP{self.ch} {'ON' if on else 'OFF'}")
        if not on:
            self.state = "off"

    def apply_dc(self, volts):
        self._guard(volts)
        self.write(f":SOUR{self.ch}:APPL:DC DEF,DEF,{volts:.6g}")
        self.check_error("APPL:DC")
        self.state = f"DC {volts:+.4g} V"

    def apply_sine(self, vrms, freq_hz, offset=0.0):
        vpp = vrms * 2.0 * math.sqrt(2.0)
        self._guard(abs(offset) + vpp / 2.0)
        if vpp + 2 * abs(offset) > self.HIGHZ_MAX_VPP:
            raise InstrumentError(f"{vpp:.2f} Vpp + offset exceeds the generator's "
                                  f"{self.HIGHZ_MAX_VPP:g} Vpp HighZ range")
        self.write(f":SOUR{self.ch}:APPL:SIN {freq_hz:.6g},{vpp:.6g},{offset:.6g},0")
        self.check_error("APPL:SIN")
        self.state = f"sine {vrms:g} Vrms {freq_hz:g} Hz"

    def safe_off(self):
        try:
            if self.connected:
                self.output(False)
        except Exception:
            pass


class SimFGen:
    name = "Function generator (sim)"

    def __init__(self, bench, max_abs_v=10.0):
        self.bench = bench
        self.max_abs_v = max_abs_v
        self.ident = "RIGOL,DG852 Pro,SIM"
        self.state = "off"
        self.connected = True
        self._pending = ("dc", 0.0, 0.0)

    def connect(self, host=None, port=None):
        return self.ident

    def close(self):
        pass

    def setup(self):
        self.output(False)

    def output(self, on):
        self.bench.fgen = self._pending if on else None
        if not on:
            self.state = "off"

    def apply_dc(self, volts):
        if abs(volts) > self.max_abs_v:
            raise InstrumentError("exceeds safety limit")
        self._pending = ("dc", volts, 0.0)
        if self.bench.fgen is not None:
            self.bench.fgen = self._pending
        self.state = f"DC {volts:+.4g} V"

    def apply_sine(self, vrms, freq_hz, offset=0.0):
        if vrms * math.sqrt(2) + abs(offset) > self.max_abs_v:
            raise InstrumentError("exceeds safety limit")
        self._pending = ("sine", vrms, freq_hz)
        if self.bench.fgen is not None:
            self.bench.fgen = self._pending
        self.state = f"sine {vrms:g} Vrms {freq_hz:g} Hz"

    def safe_off(self):
        self.output(False)


# ---------------------------------------------------------------------------
# Siglent SDS800X HD
# ---------------------------------------------------------------------------
class ScopeWave:
    __slots__ = ("channel", "volts", "t0", "dt", "sparse", "vdiv", "offset", "probe")

    def __init__(self, channel, volts, t0, dt, sparse, vdiv, offset, probe):
        self.channel, self.volts, self.t0, self.dt = channel, volts, t0, dt
        self.sparse, self.vdiv, self.offset, self.probe = sparse, vdiv, offset, probe

    def stats(self):
        v = self.volts
        n = len(v)
        if not n:
            return {}
        mean = sum(v) / n
        rms = math.sqrt(sum(x * x for x in v) / n)
        acrms = math.sqrt(max(0.0, rms * rms - mean * mean))
        return {"min": min(v), "max": max(v), "mean": mean, "rms": rms,
                "acrms": acrms, "pkpk": max(v) - min(v)}


class SiglentSDS(ScpiSocket):
    """Port of ScopeLib.ps1 (same descriptor offsets, same chunking)."""
    name = "Scope"

    def __init__(self, channels=None, max_points=100000):
        super().__init__()
        self.channels = channels       # None = whatever is switched on
        self.max_points = max_points
        self.ident = ""
        self.orig_mode = None
        self.was_running = False

    def connect(self, host, port=5025):
        self.open(host, port)
        self.ident = self.query("*IDN?")
        self.orig_mode = self.query(":TRIG:MODE?")
        self.was_running = self.query(":TRIG:STAT?") != "Stop"
        return self.ident

    def active_channels(self):
        if self.channels:
            return [c.upper() for c in self.channels]
        return [f"C{i}" for i in range(1, 5) if self.query(f":CHAN{i}:SWIT?") == "ON"]

    def run(self):
        if self.orig_mode:
            self.write(f":TRIG:MODE {self.orig_mode}")
        self.write(":TRIG:RUN")

    def restore(self):
        try:
            if self.orig_mode:
                self.write(f":TRIG:MODE {self.orig_mode}")
                if self.was_running:
                    self.write(":TRIG:RUN")
        except Exception:
            pass

    def wait_stopped(self, timeout=3.0):
        end = time.time() + timeout
        while self.query(":TRIG:STAT?") != "Stop":
            if time.time() > end:
                raise InstrumentError("scope did not stop")
            time.sleep(0.05)

    def grab(self):
        """Stop and take whatever is on screen."""
        self.write(":TRIG:STOP")
        self.wait_stopped()

    def single(self, timeout_s, stop_event=None):
        """Arm a single acquisition; False on timeout (no trigger)."""
        self.write(":TRIG:MODE SING")
        self.query("*OPC?")
        end = time.time() + timeout_s
        while self.query(":TRIG:STAT?") != "Stop":
            if (stop_event and stop_event.is_set()) or time.time() > end:
                self.write(":TRIG:STOP")
                return False
            time.sleep(0.05)
        return True

    def trigger_desc(self):
        typ = self.query(":TRIG:TYPE?")
        if typ != "EDGE":
            return typ
        return (f"{self.query(':TRIG:EDGE:SOUR?')} {self.query(':TRIG:EDGE:SLOP?')} at "
                f"{float(self.query(':TRIG:EDGE:LEV?')):.4g}V")

    def read_channel(self, ch, tdiv):
        self.write(f":WAV:SOUR {ch}")
        self.write(":WAV:WIDT WORD")
        self.write(":WAV:INT 1")
        self.write(":WAV:STAR 0")
        self.write(":WAV:POIN 0")
        pre = self.query_block(":WAV:PRE?")
        count = struct.unpack_from("<i", pre, 0x74)[0]
        if count <= 0:
            raise InstrumentError(f"no waveform on {ch} yet")
        probe = struct.unpack_from("<f", pre, 0x148)[0]
        vdiv = struct.unpack_from("<f", pre, 0x9C)[0] * probe
        offs = struct.unpack_from("<f", pre, 0xA0)[0] * probe
        code = struct.unpack_from("<f", pre, 0xA4)[0]
        dt = float(f"{struct.unpack_from('<f', pre, 0xB0)[0]:.7g}")   # float32 -> tidy
        delay = struct.unpack_from("<d", pre, 0xB4)[0]
        maxp = int(float(self.query(":WAV:MAXP?")))

        chunks = []
        sparse = 1
        if self.max_points and count > self.max_points:
            sparse = int(math.ceil(count / self.max_points))
            self.write(f":WAV:INT {sparse}")
            chunks.append(self.query_block(":WAV:DATA?"))
            self.write(":WAV:INT 1")
        else:
            start = 0
            while start < count:
                self.write(f":WAV:STAR {start}")
                self.write(f":WAV:POIN {min(maxp, count - start)}")
                chunks.append(self.query_block(":WAV:DATA?"))
                start += maxp
            self.write(":WAV:STAR 0")
            self.write(":WAV:POIN 0")
        scale = vdiv / code
        volts = []
        for c in chunks:
            n = len(c) // 2
            volts.extend(x * scale - offs for x in struct.unpack_from(f"<{n}h", c, 0))
        if not volts:
            raise InstrumentError(f"no waveform data on {ch} (scope did not trigger)")
        return ScopeWave(ch, volts, delay - 5 * tdiv, dt * sparse, sparse, vdiv, offs, probe)

    def capture(self, mode, timeout_s=5.0, stop_event=None):
        """mode 'grab' | 'single'.  Returns (tdiv, trigger_desc, [ScopeWave]) or None."""
        if mode == "single":
            if not self.single(timeout_s, stop_event):
                return None
            trig = self.trigger_desc()
        else:
            self.grab()
            trig = "screen grab"
        tdiv = float(self.query(":TIM:SCAL?"))
        waves = [self.read_channel(ch, tdiv) for ch in self.active_channels()]
        self.run()               # live again while the next condition is set up
        return tdiv, trig, waves


class SimScope:
    name = "Scope (sim)"

    def __init__(self, bench):
        self.bench = bench
        self.ident = "Siglent Technologies,SDS814X HD,SIM"
        self.connected = True

    def connect(self, host=None, port=None):
        return self.ident

    def close(self):
        pass

    def restore(self):
        pass

    def capture(self, mode, timeout_s=5.0, stop_event=None):
        import random
        n, tdiv = 2000, 5e-3
        dt = 10 * tdiv / n
        f = self.bench.fgen
        out = []
        for i in range(n):
            t = -5 * tdiv + i * dt
            if f is None:
                v = 0.0
            elif f[0] == "dc":
                v = f[1]
            else:
                v = f[1] * math.sqrt(2) * math.sin(2 * math.pi * f[2] * t)
            out.append(v + random.gauss(0, 0.003))
        return tdiv, "sim", [ScopeWave("C1", out, -5 * tdiv, dt, 1, 1.0, 0.0, 1.0)]


def _csv_cell(s):
    s = str(s)
    return '"' + s.replace('"', '""') + '"' if any(c in s for c in '",\r\n') else s


def write_scope_csv(path, idn, started, note, conds):
    """Write Capture-Scope.ps1's layout.  conds: [dict(label, note, time, tdiv, trigger, waves)].

    Kept byte-for-byte compatible in structure so Plot-Capture.ps1 renders it.
    """
    lines = [f"# Siglent capture,{_csv_cell(idn)}",
             f"# Session started,{started}",
             f"# Session note,{_csv_cell(note)}",
             "# Time is seconds relative to the trigger point"]
    for i, c in enumerate(conds, 1):
        w0 = c["waves"][0]
        lines.append(f"# Condition {i},{_csv_cell(c['label'])},captured {c['time']},"
                     f"{_csv_cell(c.get('note', ''))}")
        lines.append(f"#   timebase {c['tdiv']:.4g}s/div, sample interval {w0.dt:.4g}s, "
                     f"{len(w0.volts)} points"
                     + (f" (every {w0.sparse}th sample)" if w0.sparse > 1 else "")
                     + f", trigger {c['trigger']}")
        for w in c["waves"]:
            lines.append(f"#   {w.channel}: {w.vdiv:.4g}V/div, offset {w.offset:.4g}V, "
                         f"probe {w.probe:g}x")
    ref = conds[0]["waves"][0]
    shared = all(len(w.volts) == len(ref.volts) and abs(w.t0 - ref.t0) <= 1e-12
                 and abs(w.dt - ref.dt) <= 1e-15
                 for c in conds for w in c["waves"])
    hdr, cols, is_t = [], [], []
    if shared:
        hdr.append("Time (s)")
        cols.append([ref.t0 + i * ref.dt for i in range(len(ref.volts))])
        is_t.append(True)
    for c in conds:
        if not shared:
            w0 = c["waves"][0]
            hdr.append(_csv_cell(f"{c['label']} Time (s)"))
            cols.append([w0.t0 + i * w0.dt for i in range(len(w0.volts))])
            is_t.append(True)
        for w in c["waves"]:
            hdr.append(_csv_cell(f"{c['label']} {w.channel} (V)"))
            cols.append(w.volts)
            is_t.append(False)
    lines.append(",".join(hdr))
    rows = max(len(c) for c in cols)
    with open(path, "w", encoding="utf-8-sig", newline="\r\n") as fh:
        fh.write("\n".join(lines) + "\n")
        for r in range(rows):
            fh.write(",".join(
                (f"{col[r]:.9g}" if t else f"{col[r]:.6g}") if r < len(col) else ""
                for col, t in zip(cols, is_t)) + "\n")
