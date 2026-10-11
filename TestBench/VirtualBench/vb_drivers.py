"""
vb_drivers.py  --  instrument drivers for the Virtual Bench.

Every instrument here is a long-lived object with a poller thread that keeps
`state` (the instrument's settings and readings, as JSON-able dicts) fresh, and
`do_<action>(**args)` methods the web UI calls.  The SCPI plumbing (ScpiSocket)
and the scope's waveform decoder (SiglentSDS.read_channel) come from the
BlinkyHawk test suite, so there is one implementation of each, not two.

Ground rules, different from the test-suite drivers:
  * CONNECTING NEVER CHANGES ANYTHING.  The suite drivers switch outputs off and
    force settings on connect; here connect is *IDN? plus reads, so opening the
    bench never disturbs a setup made on the front panel.
  * A comms failure (timeout, closed socket) drops the socket and the poller
    reconnects -- after a timeout a late reply may still be in flight, and a
    fresh socket is the only way to be sure replies line up with queries.
  * These instruments serve ONE SCPI client at a time: while the bench holds an
    instrument, the test suite cannot use it.  Disconnect it here first.

Command syntax notes are per class; "probed" means confirmed against the bench
unit on 2026-10-10.
"""

import collections
import math
import os
import socket
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "BlinkyHawkTestSuite")))

from bh_instruments import InstrumentError, ScpiSocket, SiglentSDS   # noqa: E402
import bh_giga                                                        # noqa: E402


class ScpiError(Exception):
    """The instrument understood the link but rejected a command.  Unlike a
    comms failure this does not drop the connection."""


def _msg(e):
    if isinstance(e, (socket.timeout, TimeoutError)):
        return "timed out waiting for a reply"
    return str(e) or type(e).__name__


def _f(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


# ---------------------------------------------------------------------------
# base classes
# ---------------------------------------------------------------------------
class Instrument:
    kind = "?"
    poll_s = 1.0
    retry_s = 5.0
    history_len = 7200

    def __init__(self, key, name, host, port):
        self.key, self.name, self.host, self.port = key, name, host, port
        self.lock = threading.RLock()
        self.want = True               # the poller (re)connects while this is set
        self.connected = False
        self.ident = ""
        self.error = ""
        self.state = {}
        self.updated = 0.0
        self.history = collections.deque(maxlen=self.history_len)
        self._wake = threading.Event()
        self._thread = None

    # --- lifecycle ---
    def start(self):
        self._thread = threading.Thread(target=self._loop, name=f"poll-{self.key}", daemon=True)
        self._thread.start()

    def _loop(self):
        next_try = 0.0
        while True:
            if self.want and not self.connected and time.time() >= next_try:
                try:
                    self.connect()
                except Exception as e:                   # noqa: BLE001 -- shown in the UI
                    self.error = _msg(e)
                    next_try = time.time() + self.retry_s
            if self.connected:
                try:
                    self.poll()
                except (ScpiError, ValueError) as e:
                    self.error = _msg(e)
                except Exception as e:                   # noqa: BLE001
                    self._fail(e)
                    next_try = time.time() + 1.0
            self._wake.wait(self.poll_s)
            self._wake.clear()

    def connect(self):
        self.want = True
        with self.lock:
            self._close()
            self.ident = self._connect()
            self.connected = True
            self.error = ""
        self.poll()

    def disconnect(self):
        self.want = False
        with self.lock:
            self._close()
            self.connected = False
        self.error = ""

    def _fail(self, e):
        self.error = _msg(e)
        with self.lock:
            try:
                self._close()
            except Exception:                            # noqa: BLE001
                pass
            self.connected = False

    # --- polling / actions ---
    def poll(self):
        with self.lock:
            if not self.connected:
                return
            st = self._poll()
        self.state = st
        self.updated = time.time()

    def act(self, action, args):
        fn = getattr(self, "do_" + action, None)
        if fn is None:
            raise ValueError(f"{self.name} has no action {action!r}")
        if not self.connected and action not in self.offline_actions:
            raise ValueError(f"{self.name} is not connected")
        try:
            with self.lock:
                if not self.connected and action not in self.offline_actions:
                    # the poller dropped the link while we waited for the lock
                    # (e.g. the DMM's hung capacitance read): reconnect now
                    self.ident = self._connect()
                    self.connected = True
                if self.connected:
                    self._before_act()
                res = fn(**(args or {}))
        except (ScpiError, ValueError, TypeError):
            raise
        except Exception as e:                           # noqa: BLE001
            self._fail(e)
            raise
        self._wake.set()                                 # show the effect promptly
        return res

    offline_actions = ("clear_history",)

    def do_clear_history(self):
        self.history.clear()

    def record(self, values):
        self.history.append((round(time.time(), 3), values))

    def history_since(self, since):
        return [h for h in list(self.history) if h[0] > since]

    def snapshot(self):
        return {"key": self.key, "name": self.name, "kind": self.kind, "host": self.host,
                "port": self.port, "connected": self.connected, "want": self.want,
                "ident": self.ident, "error": self.error, "state": self.state,
                "updated": self.updated}

    def safe_off(self):
        """Switch every output off.  Only output instruments override this."""
        return None

    # --- subclass hooks ---
    def _before_act(self):
        pass

    def _connect(self):
        raise NotImplementedError

    def _close(self):
        pass

    def _poll(self):
        return {}


TRACE = None          # file object: virtual_bench.py --trace logs every SCPI line


def _trace(key, arrow, text):
    TRACE.write(f"{time.time():.3f} {threading.current_thread().name[:14]:>14} {key:>5} {arrow} {text}\n")
    TRACE.flush()


class ScpiInstrument(Instrument):
    io_class = ScpiSocket
    timeout = 5.0

    def __init__(self, key, name, host, port):
        super().__init__(key, name, host, port)
        self.io = self.io_class()
        if TRACE:
            # ScpiSocket.query/query_block call self.write, so this sees every line sent
            raw_write, raw_query = self.io.write, self.io.query

            def write(cmd):
                _trace(self.key, ">", cmd)
                raw_write(cmd)

            def query(cmd):
                reply = raw_query(cmd)
                _trace(self.key, "<", reply)
                return reply
            self.io.write, self.io.query = write, query

    def _connect(self):
        self.io.open(self.host, self.port, timeout=self.timeout)
        return self.io.identify(self.name.lower())

    def _close(self):
        self.io.close()

    def q(self, cmd):
        return self.io.query(cmd)

    def qf(self, cmd):
        return _f(self.q(cmd))

    def w(self, cmd):
        self.io.write(cmd)

    def _before_act(self):
        # Errors left by the console, the front panel or an earlier query would
        # otherwise be blamed on this action by check_error.
        self.w("*CLS")

    def check_error(self, what):
        err = self.q("SYST:ERR?")
        code = err.split(",", 1)[0].strip().lstrip("+")
        if code not in ("0", ""):
            raise ScpiError(f"{self.name} rejected {what}: {err}")

    def raw(self, cmd):
        """The SCPI console.  Anything ending in '?' is read as one line."""
        cmd = cmd.strip()
        if not cmd:
            return ""
        try:
            with self.lock:
                if not self.connected:
                    raise ValueError(f"{self.name} is not connected")
                head = cmd.split()[0]
                if head.endswith("?"):
                    return self.io.query(cmd)
                self.io.write(cmd)
                return ""
        except (ValueError, ScpiError):
            raise
        except Exception as e:                           # noqa: BLE001
            self._fail(e)
            raise


# ---------------------------------------------------------------------------
# Rigol DM858 multimeter
# ---------------------------------------------------------------------------
class RigolDM858(ScpiInstrument):
    """Keysight-style SCPI (probed): FUNC? -> "VOLT", CONF? -> "VOLT <range>,<res>",
    <func>:RANG? / :RANG:AUTO? / :NPLC?, READ?.  Port 5025 (5555 closed).
    Ranges and NPLC values are from the DM858 Programming Guide (PGC11101-1110).
    It has NO input-impedance command (no 10 Mohm / 10 Gohm switch over SCPI).
    Its SYST:ERR? queue is unreliable (reports -295/-100 against queries that
    answered), so it is not checked after commands; the UI shows what the
    meter reports back instead, which is the real confirmation."""
    kind = "dmm"
    poll_s = 0.25
    timeout = 8.0

    # key: (CONF command, FUNC? replies, SCPI prefix for range/NPLC, unit, label, ranges, has NPLC)
    FUNCS = {
        "VDC":  ("CONF:VOLT:DC", ("VOLT", "VOLT:DC"), "VOLT:DC", "V", "DC volts",
                 [0.1, 1, 10, 100, 1000], True),
        "VAC":  ("CONF:VOLT:AC", ("VOLT:AC",), "VOLT:AC", "V", "AC volts",
                 [0.1, 1, 10, 100, 750], False),
        "IDC":  ("CONF:CURR:DC", ("CURR", "CURR:DC"), "CURR:DC", "A", "DC current",
                 [1e-4, 1e-3, 1e-2, 0.1, 1, 3, 10], True),
        "IAC":  ("CONF:CURR:AC", ("CURR:AC",), "CURR:AC", "A", "AC current",
                 [1e-4, 1e-3, 1e-2, 0.1, 1, 3, 10], False),
        "RES":  ("CONF:RES", ("RES",), "RES", "Ω", "Resistance 2W",
                 [100, 1e3, 1e4, 1e5, 1e6, 1e7, 5e7], True),
        "FRES": ("CONF:FRES", ("FRES",), "FRES", "Ω", "Resistance 4W",
                 [100, 1e3, 1e4, 1e5, 1e6, 1e7, 5e7], True),
        "FREQ": ("CONF:FREQ", ("FREQ",), None, "Hz", "Frequency", None, False),
        "CAP":  ("CONF:CAP", ("CAP",), "CAP", "F", "Capacitance",
                 [1e-9, 1e-8, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2], False),
        "CONT": ("CONF:CONT", ("CONT",), None, "Ω", "Continuity", None, False),
        "DIOD": ("CONF:DIOD", ("DIOD",), None, "V", "Diode", None, False),
    }
    NPLCS = [0.4, 5, 20]             # the only values the guide allows: fast / medium / slow

    def _func_key(self, reply):
        r = reply.strip().strip('"').upper()
        for k, f in self.FUNCS.items():
            if r in f[1]:
                return k
        return None

    def _poll(self):
        fraw = self.q("FUNC?")
        key = self._func_key(fraw)
        st = {"func": key, "func_raw": fraw.strip('"'),
              "funcs": {k: {"label": f[4], "unit": f[3], "ranges": f[5], "nplc": f[6]}
                        for k, f in self.FUNCS.items()},
              "nplcs": self.NPLCS}
        if key:
            _, _, pre, unit, label, _, has_nplc = self.FUNCS[key]
            st.update(unit=unit, label=label)
            if pre:
                st["range"] = self.qf(f"{pre}:RANG?")
                st["auto"] = self.q(f"{pre}:RANG:AUTO?").strip() in ("1", "ON")
            if has_nplc:
                st["nplc"] = self.qf(f"{pre}:NPLC?")
        try:
            v = self.qf("READ?")
        except (socket.timeout, TimeoutError):
            # An open-lead capacitance reading never finishes, and the meter
            # answers nothing else until it does; only a fresh socket gets it
            # back.  Not an error -- report it and let the poller reconnect.
            self.io.close()
            self.connected = False
            st.update(value=None, overload=False,
                      note=f"no reading within {self.timeout:g} s (open leads?) -- reconnecting")
            return st
        overload = v is not None and abs(v) >= 9e37
        st["overload"] = overload
        st["value"] = None if overload else v
        if v is not None:
            self.record({"value": None if overload else v})
        return st

    def do_function(self, func):
        if func not in self.FUNCS:
            raise ValueError(f"unknown function {func}")
        self.history.clear()                 # a trend that mixes volts and ohms is nonsense
        self.w(self.FUNCS[func][0])

    def _prefix(self):
        key = self._func_key(self.q("FUNC?"))
        pre = key and self.FUNCS[key][2]
        if not pre:
            raise ValueError("this function has no range setting")
        return pre

    def do_range(self, value):
        pre = self._prefix()
        if value in ("AUTO", None, ""):
            self.w(f"{pre}:RANG:AUTO ON")
        else:
            self.w(f"{pre}:RANG {float(value):g}")

    def do_nplc(self, value):
        pre = self._prefix()
        self.w(f"{pre}:NPLC {float(value):g}")


# ---------------------------------------------------------------------------
# Siglent SPD4323X power supply
# ---------------------------------------------------------------------------
class SiglentSPD4000(ScpiInstrument):
    """Syntax (probed): VOLT? CH1, CURR? CH1, MEAS:VOLT? CH1, MEAS:CURR? CH1,
    MEAS:POWer? CH1, OUTP? CH1, OUTP:TRACK? (0 independent / 1 series /
    2 parallel, CH2+CH3).

    SETTING TAKES THE CHANNEL FIRST: VOLT CH2,9 / CURR CH2,0.4 / OUTP CH2,ON.
    The value-first form (VOLT 9,CH2, as the queries read) is ACCEPTED WITH NO
    ERROR AND IGNORED -- SYST:ERR? then answers "0,No error,VOLT" instead of
    the usual "+0, No error".  So every set is verified by reading it back.
    The `CHn:VOLT` form of the older SPD3303X is rejected outright.

    No channel ratings are hard-coded: the 'datasheet' 6 V for CH3 turned out
    wrong (it takes 9 V), and the read-back catches anything the supply clamps.

    Changing OUTP:TRACK (seen 2026-10-10) switches CH3's output OFF and copies
    CH2's setpoints onto CH3; in series mode CH2's setpoint reads the combined
    voltage (2x)."""
    kind = "psu"
    poll_s = 0.7
    CHANNELS = (1, 2, 3, 4)
    TRACK = {0: "Independent", 1: "Series", 2: "Parallel"}

    def _poll(self):
        chans, rec = [], {}
        for ch in self.CHANNELS:
            c = {"ch": ch,
                 "vset": self.qf(f"VOLT? CH{ch}"), "iset": self.qf(f"CURR? CH{ch}"),
                 "on": self.q(f"OUTP? CH{ch}").strip() in ("1", "ON"),
                 "v": self.qf(f"MEAS:VOLT? CH{ch}"), "i": self.qf(f"MEAS:CURR? CH{ch}")}
            c["p"] = c["v"] * c["i"] if c["v"] is not None and c["i"] is not None else None
            # in CC when the measured current has reached the limit
            c["cc"] = bool(c["on"] and c["i"] is not None and c["iset"]
                           and c["i"] >= 0.98 * c["iset"])
            chans.append(c)
            rec[f"V{ch}"], rec[f"I{ch}"] = c["v"], c["i"]
        self.record(rec)
        track = _f(self.q("OUTP:TRACK?"))
        return {"channels": chans, "track": None if track is None else int(track),
                "track_modes": self.TRACK}

    def _ch(self, ch):
        ch = int(ch)
        if ch not in self.CHANNELS:
            raise ValueError(f"no channel {ch}")
        return ch

    def _set_verified(self, what, ch, value, unit):
        if value < 0:
            raise ValueError(f"CH{ch} {what} can't be negative")
        self.w(f"{what} CH{ch},{value:.4f}")
        self.check_error(f"{what} CH{ch},{value:g}")
        got = self.qf(f"{what}? CH{ch}")
        if got is None or abs(got - value) > max(0.0015, 0.001 * value):
            raise ScpiError(f"CH{ch} kept {got:g} {unit} instead of {value:g} {unit} -- "
                            "outside the channel's range, or the channel is slaved in "
                            "series/parallel mode")

    def do_set(self, ch, volts=None, amps=None):
        ch = self._ch(ch)
        if volts is not None:
            self._set_verified("VOLT", ch, float(volts), "V")
        if amps is not None:
            self._set_verified("CURR", ch, float(amps), "A")

    def do_track(self, mode):
        mode = int(mode)
        if mode not in self.TRACK:
            raise ValueError("mode must be 0 (independent), 1 (series) or 2 (parallel)")
        self.w(f"OUTP:TRACK {mode}")
        self.check_error("OUTP:TRACK")
        if _f(self.q("OUTP:TRACK?")) != mode:
            raise ScpiError(f"the supply did not switch to {self.TRACK[mode]} mode")

    def do_output(self, ch, on):
        ch = self._ch(ch)
        self.w(f"OUTP CH{ch},{'ON' if on else 'OFF'}")

    def do_all_off(self):
        for ch in self.CHANNELS:
            self.w(f"OUTP CH{ch},OFF")

    def safe_off(self):
        self.do_all_off()


# ---------------------------------------------------------------------------
# Rigol DL3021A electronic load
# ---------------------------------------------------------------------------
class RigolDL3000(ScpiInstrument):
    """Syntax (probed): :SOUR:FUNC? -> CC|CV|CR|CP, :SOUR:<CURR|VOLT|RES|POW>:LEV:IMM,
    :SOUR:CURR:RANG (4 / 40 A), :SOUR:CURR:VON, :SOUR:INP:STAT, :MEAS:VOLT?/CURR?/POW?/RES?.
    Port 5555 (5025 closed).  Changing :SOUR:CURR:RANG resets the CC level to
    0 A (probed; mode changes don't).  Von only gates sinking with "Von Latch" OFF on
    the front panel (no SCPI command for the latch)."""
    kind = "load"
    poll_s = 0.5
    MODES = {"CC": ("CURR", "A"), "CV": ("VOLT", "V"), "CR": ("RES", "Ω"), "CP": ("POW", "W")}

    def _poll(self):
        st = {"mode": self.q(":SOUR:FUNC?").strip().upper()[:2],
              "input": self.q(":SOUR:INP:STAT?").strip() in ("1", "ON"),
              "levels": {m: self.qf(f":SOUR:{s}:LEV:IMM?") for m, (s, _) in self.MODES.items()},
              "range": self.qf(":SOUR:CURR:RANG?"),
              "von": self.qf(":SOUR:CURR:VON?"),
              "v": self.qf(":MEAS:VOLT?"), "i": self.qf(":MEAS:CURR?"),
              "p": self.qf(":MEAS:POW?"), "r": self.qf(":MEAS:RES?"),
              "units": {m: u for m, (_, u) in self.MODES.items()}}
        self.record({"V": st["v"], "I": st["i"], "P": st["p"]})
        return st

    def do_mode(self, mode):
        if mode not in self.MODES:
            raise ValueError(f"unknown mode {mode}")
        self.w(f":SOUR:FUNC {self.MODES[mode][0]}")
        self.check_error(f"mode {mode}")

    def do_level(self, mode, value):
        if mode not in self.MODES:
            raise ValueError(f"unknown mode {mode}")
        v = float(value)
        if v < 0:
            raise ValueError("level must be positive")
        self.w(f":SOUR:{self.MODES[mode][0]}:LEV:IMM {v:g}")
        self.check_error(f"{mode} level {v:g}")

    def do_von(self, value):
        self.w(f":SOUR:CURR:VON {float(value):g}")
        self.check_error("Von")

    def do_range(self, value):
        self.w(f":SOUR:CURR:RANG {float(value):g}")
        self.check_error("range")

    def do_input(self, on):
        self.w(f":SOUR:INP:STAT {'ON' if on else 'OFF'}")

    def safe_off(self):
        self.do_input(False)


# ---------------------------------------------------------------------------
# Rigol DG852 Pro function generator
# ---------------------------------------------------------------------------
class RigolDG800(ScpiInstrument):
    """DG800 Pro / DG900 Pro Programming Guide syntax: :SOUR<n>:FUNC, :FREQ,
    :VOLT, :VOLT:OFFS, :PHAS, :FUNC:SQU:DCYC, :FUNC:RAMP:SYMM, :VOLT:UNIT,
    :OUTP<n>[:STAT], :OUTP<n>:LOAD.  Port 5025 (5555/5000 closed on this unit).

    THE LOAD SETTING IS THE TRAP: at the default 50 ohm setting the generator
    doubles its output into a high-impedance input.  The panel shows the load
    setting on every channel and offers HighZ; it does not change it silently.
    Parameter-at-a-time commands (not APPLy) so changing one field leaves the
    others alone -- except DC (probed 2026-10-10, fw 00.01.00.00.22): `:FUNC DC`
    is REJECTED with a misleading -350 "Queue overflow", and after
    `:APPL:DC DEF,DEF,<V>` FUNC? answers "ARB" (DC is an arb internally).  So DC
    is set with APPL:DC and the waveform is read from APPL? ("DC,..." /
    "SIN,..."), which names it correctly."""
    kind = "fgen"
    poll_s = 1.5
    FUNCS = ["SIN", "SQU", "RAMP", "PULS", "NOIS", "DC"]

    @staticmethod
    def _func(reply):
        r = reply.strip().strip('"').upper()
        for f in RigolDG800.FUNCS:
            if r.startswith(f):
                return f
        return r

    def _poll(self):
        chans = []
        for n in (1, 2):
            c = {"ch": n,
                 "func": self._func(self.q(f":SOUR{n}:APPL?")),
                 "freq": self.qf(f":SOUR{n}:FREQ?"),
                 "amp": self.qf(f":SOUR{n}:VOLT?"),
                 "offset": self.qf(f":SOUR{n}:VOLT:OFFS?"),
                 "phase": self.qf(f":SOUR{n}:PHAS?"),
                 "unit": self.q(f":SOUR{n}:VOLT:UNIT?").strip().upper(),
                 "on": self.q(f":OUTP{n}?").strip() in ("1", "ON")}
            load = self.qf(f":OUTP{n}:LOAD?")
            c["highz"] = load is None or load >= 1e30
            c["load"] = None if c["highz"] else load
            if c["func"] == "SQU":
                c["duty"] = self.qf(f":SOUR{n}:FUNC:SQU:DCYC?")
            elif c["func"] == "RAMP":
                c["symmetry"] = self.qf(f":SOUR{n}:FUNC:RAMP:SYMM?")
            chans.append(c)
        return {"channels": chans, "funcs": self.FUNCS}

    def do_set(self, ch, func=None, freq=None, amp=None, offset=None, phase=None,
               duty=None, symmetry=None, unit=None):
        n = int(ch)
        if n not in (1, 2):
            raise ValueError(f"no channel {ch}")
        s = f":SOUR{n}"
        if func == "DC":
            if offset is None:
                offset = self.qf(f"{s}:VOLT:OFFS?") or 0.0
            self.w(f"{s}:APPL:DC DEF,DEF,{float(offset):.6g}")
            func = offset = None
        if func is not None:
            if func not in self.FUNCS:
                raise ValueError(f"unknown waveform {func}")
            self.w(f"{s}:FUNC {func}")
        if unit is not None:
            if unit not in ("VPP", "VRMS", "DBM"):
                raise ValueError("unit must be VPP, VRMS or DBM")
            self.w(f"{s}:VOLT:UNIT {unit}")
        if freq is not None:
            self.w(f"{s}:FREQ {float(freq):.9g}")
        if amp is not None:
            self.w(f"{s}:VOLT {float(amp):.6g}")
        if offset is not None:
            self.w(f"{s}:VOLT:OFFS {float(offset):.6g}")
        if phase is not None:
            self.w(f"{s}:PHAS {float(phase):.6g}")
        if duty is not None:
            self.w(f"{s}:FUNC:SQU:DCYC {float(duty):.6g}")
        if symmetry is not None:
            self.w(f"{s}:FUNC:RAMP:SYMM {float(symmetry):.6g}")
        self.check_error(f"CH{n} settings")

    def do_load(self, ch, highz):
        n = int(ch)
        self.w(f":OUTP{n}:LOAD {'INF' if highz else '50'}")
        self.check_error(f"CH{n} load")

    def do_output(self, ch, on):
        n = int(ch)
        self.w(f":OUTP{n} {'ON' if on else 'OFF'}")

    def safe_off(self):
        for n in (1, 2):
            self.w(f":OUTP{n} OFF")


# ---------------------------------------------------------------------------
# Siglent SDS800X HD oscilloscope
# ---------------------------------------------------------------------------
def _freq_estimate(v, dt):
    """Rising mean-crossings with 10 % hysteresis -> Hz, or None."""
    if len(v) < 8:
        return None
    lo, hi = min(v), max(v)
    if hi - lo <= 1e-12:
        return None
    mid, h = (lo + hi) / 2, (hi - lo) * 0.1
    armed, edges = False, []
    for i, x in enumerate(v):
        if x < mid - h:
            armed = True
        elif armed and x > mid + h:
            edges.append(i)
            armed = False
    if len(edges) < 2:
        return None
    return (len(edges) - 1) / ((edges[-1] - edges[0]) * dt)


def _wave_stats(v, dt):
    n = len(v)
    mean = sum(v) / n
    rms = math.sqrt(sum(x * x for x in v) / n)
    return {"min": min(v), "max": max(v), "pkpk": max(v) - min(v), "mean": mean, "rms": rms,
            "acrms": math.sqrt(max(0.0, rms * rms - mean * mean)), "freq": _freq_estimate(v, dt)}


class SiglentScope(ScpiInstrument):
    """Syntax (probed, SDS804X HD fw 3.8.12): :TRIG:STAT? (Stop/Arm/Ready/Auto/
    Trig'd), :TRIG:MODE AUTO|NORM|SING, :TRIG:RUN/:TRIG:STOP, :TIM:SCAL, :TIM:DEL,
    :CHANn:SWIT/SCAL/OFFS/COUP/PROB/BWL, :TRIG:EDGE:SOUR/LEV/SLOP, :ACQ:SRAT?/MDEP?,
    and :PRIN? PNG, which returns a RAW PNG (no block header) then a newline."""
    kind = "scope"
    poll_s = 1.5
    timeout = 8.0
    io_class = SiglentSDS
    WAVE_POINTS = 4000

    def __init__(self, key, name, host, port):
        super().__init__(key, name, host, port)
        self.io.max_points = self.WAVE_POINTS
        # NOTHING MAY FOLLOW A SETTING UNTIL THE SCOPE HAS APPLIED IT.  Anything
        # that arrives while it is still parsing one -- the next command in its
        # own packet (TCP_NODELAY), or in the same packet (Nagle coalescing) --
        # makes it apply a WRONG value: `:TIM:SCAL 1E-3` then `*OPC?` as two
        # packets landed on 10 ns/div in 20-30 of 30 tries (2026-10-10, fw
        # 3.8.12); on the bench 0.5 V/div -> 0.5 mV/div, trigger level -> 0.
        # The one form that never failed is the setting and *OPC? on ONE line,
        # answered before anything else is sent -- so every non-query write,
        # including SiglentSDS.read_channel's :WAV: setup, goes that way.
        plain_write, query = self.io.write, self.io.query

        def write(cmd):
            if "?" in cmd.split()[0] or cmd.rstrip().endswith("?"):   # a query (or ;*OPC?)
                plain_write(cmd)
            else:
                query(f"{cmd};*OPC?")
        self.io.write = write


    def _poll(self):
        st = {"trig_state": self.q(":TRIG:STAT?").strip(),
              "trig_mode": self.q(":TRIG:MODE?").strip().upper()[:4],
              "tdiv": self.qf(":TIM:SCAL?"), "delay": self.qf(":TIM:DEL?"),
              "srate": self.qf(":ACQ:SRAT?"), "mdepth": self.q(":ACQ:MDEP?").strip(),
              "trig_src": self.q(":TRIG:EDGE:SOUR?").strip(),
              "trig_level": self.qf(":TRIG:EDGE:LEV?"),
              "trig_slope": self.q(":TRIG:EDGE:SLOP?").strip().upper()[:3]}
        chans = []
        for n in range(1, 5):
            c = f":CHAN{n}"
            chans.append({"ch": n, "on": self.q(f"{c}:SWIT?").strip() == "ON",
                          "vdiv": self.qf(f"{c}:SCAL?"), "offset": self.qf(f"{c}:OFFS?"),
                          "coupling": self.q(f"{c}:COUP?").strip().upper(),
                          "probe": self.qf(f"{c}:PROB?"),
                          "bwl": self.q(f"{c}:BWL?").strip().upper() != "FULL"})
        st["channels"] = chans
        return st

    def do_run(self):
        mode = self.q(":TRIG:MODE?").strip().upper()
        if mode.startswith("SING"):
            self.w(":TRIG:MODE AUTO")
        self.w(":TRIG:RUN")

    def do_stop(self):
        self.w(":TRIG:STOP")

    def do_single(self):
        self.w(":TRIG:MODE SING")

    def do_autoset(self):
        self.w(":AUT")

    def do_timebase(self, tdiv=None, delay=None):
        if tdiv is not None:
            self.w(f":TIM:SCAL {float(tdiv):.4E}")
        if delay is not None:
            self.w(f":TIM:DEL {float(delay):.4E}")
        self.check_error("timebase")

    def do_channel(self, ch, on=None, vdiv=None, offset=None, coupling=None, probe=None, bwl=None):
        n = int(ch)
        if n not in (1, 2, 3, 4):
            raise ValueError(f"no channel {ch}")
        c = f":CHAN{n}"
        if on:                       # a channel that is off ignores scale/offset
            self.w(f"{c}:SWIT ON")
        if probe is not None:
            self.w(f"{c}:PROB VAL,{float(probe):g}")
        if vdiv is not None:
            self.w(f"{c}:SCAL {float(vdiv):.4E}")
        if offset is not None:
            self.w(f"{c}:OFFS {float(offset):.4E}")
        if coupling is not None:
            if coupling not in ("DC", "AC", "GND"):
                raise ValueError("coupling must be DC, AC or GND")
            self.w(f"{c}:COUP {coupling}")
        if bwl is not None:
            self.w(f"{c}:BWL {'20M' if bwl else 'FULL'}")
        if on is False:
            self.w(f"{c}:SWIT OFF")
        self.check_error(f"C{n} settings")

    def do_trigger(self, source=None, level=None, slope=None, mode=None):
        if source is not None:
            self.w(f":TRIG:EDGE:SOUR {source}")
        if level is not None:
            self.w(f":TRIG:EDGE:LEV {float(level):.4E}")
        if slope is not None:
            if slope not in ("RIS", "FALL", "ALT"):
                raise ValueError("slope must be RIS, FALL or ALT")
            self.w(f":TRIG:EDGE:SLOP {slope}")
        if mode is not None:
            if mode not in ("AUTO", "NORM", "SING"):
                raise ValueError("mode must be AUTO, NORM or SING")
            self.w(f":TRIG:MODE {mode}")
        self.check_error("trigger")

    def do_waveform(self):
        """The last acquisition of every displayed channel, decimated to
        ~WAVE_POINTS (the scope picks every Nth sample), plus measurements."""
        tdiv = self.qf(":TIM:SCAL?")
        out = []
        for ch in self.io.active_channels():
            w = self.io.read_channel(ch, tdiv)
            out.append({"ch": ch, "t0": w.t0, "dt": w.dt, "vdiv": w.vdiv, "offset": w.offset,
                        "probe": w.probe, "sparse": w.sparse,
                        "v": [round(x, 6) for x in w.volts], "stats": _wave_stats(w.volts, w.dt)})
        return {"tdiv": tdiv, "waves": out, "t": time.time()}

    def screenshot(self):
        try:
            with self.lock:
                if not self.connected:
                    raise ValueError("scope is not connected")
                io = self.io
                with io.lock:
                    io.write(":PRIN? PNG")
                    end = b"IEND\xaeB`\x82"
                    while end not in io._rx:
                        io._fill()
                    k = io._rx.index(end) + len(end)
                    png, io._rx = io._rx[:k], io._rx[k:].lstrip(b"\r\n")
        except ValueError:
            raise
        except Exception as e:                           # noqa: BLE001
            self._fail(e)
            raise
        return png[png.find(b"\x89PNG"):]


# ---------------------------------------------------------------------------
# GigaMeter (Giga_RemoteController_Dual over WiFi)
# ---------------------------------------------------------------------------
class GigaBench(Instrument):
    """The isolated Giga R1 meter.  No persistent connection: the board's
    socket pool is tiny, so every capture opens its own socket (bh_giga does
    that) and nothing is polled.  A live stream is the one held socket; while it
    runs the board answers nothing else, so a capture stops it first.
    Calibration comes from the Giga GUI's own giga_presets.json (diff pair)."""
    kind = "giga"
    poll_s = 2.0
    offline_actions = ("clear_history", "preset")
    PLOT_POINTS = 4000

    def __init__(self, key, name, host, port, preset=bh_giga.DEFAULT_PRESET):
        super().__init__(key, name, host, port)
        self.preset = preset
        self.meter = None
        self.last = None
        self._stream_stop = threading.Event()
        self._stream_thread = None
        self.stream_hz = 0

    @staticmethod
    def presets():
        import json
        with open(bh_giga.PRESETS, encoding="utf-8") as fh:
            return [k for k, p in json.load(fh).items() if p.get("diff_enable")]

    def _connect(self):
        self.meter = bh_giga.GigaMeter(f"{self.host}:{self.port}", self.preset)
        return self.meter.connect()

    def _close(self):
        self._stop_stream()

    def poll(self):
        # nothing to ask the board; just publish what we have
        m = self.meter
        self.state = {
            "preset": self.preset, "presets": self.presets(),
            "pair": m.label if m else None, "scale": m.d_scale if m else None,
            "rate": m.rate if m else None, "smooth": m.smooth if m else None,
            "bits": m.bits if m else None,
            "streaming": bool(self._stream_thread and self._stream_thread.is_alive()),
            "stream_hz": self.stream_hz, "last": self.last}
        self.updated = time.time()

    def do_preset(self, name):
        if name not in self.presets():
            raise ValueError(f"preset {name!r} has no differential pair")
        self._stop_stream()
        self.preset = name
        if self.meter:
            self.meter.preset_name = name
            self.meter.load_preset(name)
        self.history.clear()
        self.poll()

    def do_capture(self, ms, smooth=None):
        self._stop_stream()
        try:
            cap = self.meter.capture(float(ms), smooth=int(smooth) if smooth else None)
        except bh_giga.GigaError as e:
            raise ScpiError(str(e)) from e
        v, t = cap.v, cap.t
        n = len(v)
        if not n:
            raise ScpiError("the meter returned no samples")
        mean = sum(v) / n
        std = math.sqrt(max(0.0, sum(x * x for x in v) / n - mean * mean))
        stats = {"n": n, "mean": mean, "min": min(v), "max": max(v), "std": std,
                 "pkpk": max(v) - min(v), "elapsed_ms": cap.elapsed_ms, "dt_ms": cap.dt_ms,
                 "freq": _freq_estimate(v, cap.dt_ms / 1000.0)}
        self.last = stats
        # min/max envelope keeps sub-bucket pulses visible in the plot
        if n > self.PLOT_POINTS:
            b = n / (self.PLOT_POINTS / 2)
            pt, pv = [], []
            for k in range(self.PLOT_POINTS // 2):
                i0, i1 = int(k * b), max(int(k * b) + 1, int((k + 1) * b))
                seg = v[i0:i1]
                lo, hi = seg.index(min(seg)), seg.index(max(seg))
                for j in sorted((lo, hi)):
                    pt.append(t[i0 + j])
                    pv.append(seg[j])
            t, v = pt, pv
        return {"t": [round(x, 4) for x in t], "v": [round(x, 6) for x in v], "stats": stats,
                "label": cap.label}

    def do_stream_start(self, hz=4):
        self._stop_stream()
        self.history.clear()
        self.stream_hz = int(hz)
        self._stream_stop.clear()
        self._stream_thread = threading.Thread(target=self._stream, args=(self.stream_hz,),
                                               name="giga-stream", daemon=True)
        self._stream_thread.start()

    def do_stream_stop(self):
        self._stop_stream()

    def _stop_stream(self):
        self._stream_stop.set()
        t = self._stream_thread
        if t and t.is_alive() and t is not threading.current_thread():
            t.join(timeout=4)
        self._stream_thread = None

    def _stream(self, hz):
        """STREAM:D1|D1:<pos>-<neg>|... -> S|...|CH:D1|N:|MIN:|MAX:|MEAN:|STD:|T:
        lines (raw counts of pos-neg).  Scaled exactly as bh_giga.stream does."""
        m = self.meter
        step = 3.3 / (2 ** m.bits - 1)
        cmd = (f"STREAM:D1|D1:{m.pos}-{m.neg}|BITS:{m.bits}|RATE:{m.rate}"
               f"|SMOOTH:{m.smooth}|HZ:{hz}\n")
        conv = lambda raw: (float(raw) * step - m.d_off) * m.d_scale     # noqa: E731
        try:
            with socket.create_connection((m.host, m.port), timeout=5) as s:
                s.settimeout(1.0)
                s.sendall(cmd.encode())
                buf = b""
                try:
                    while not self._stream_stop.is_set():
                        try:
                            chunk = s.recv(65536)
                        except socket.timeout:
                            continue
                        if not chunk:
                            raise ScpiError("the meter closed the stream")
                        buf += chunk
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            line = line.decode(errors="ignore").strip()
                            if line.startswith("ERR:"):
                                raise ScpiError(f"meter: {line}")
                            if line.startswith("STREAMON") and f"D1:{m.pos}-{m.neg}" not in line:
                                raise ScpiError(f"meter streamed different pins: {line}")
                            if line.startswith("S|") and "|CH:D1|" in line:
                                kv = dict(x.split(":", 1) for x in line.split("|")[1:] if ":" in x)
                                row = {"mean": conv(kv["MEAN"]), "min": conv(kv["MIN"]),
                                       "max": conv(kv["MAX"]),
                                       "std": float(kv.get("STD", 0)) * step * abs(m.d_scale)}
                                self.record(row)
                finally:
                    try:
                        s.sendall(b"STOP\n")
                        s.settimeout(2)
                        s.recv(4096)                     # STREAMOFF, best effort
                    except OSError:
                        pass
        except Exception as e:                           # noqa: BLE001 -- shown in the UI
            self.error = f"stream: {_msg(e)}"
