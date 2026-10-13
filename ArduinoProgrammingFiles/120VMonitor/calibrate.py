"""
calibrate.py  --  bench-supply calibration of the 120VMonitor front end.

The bench supply's CH2 is wired to the monitor's 120 V input (+ to the hot
side, - to neutral).  The isolation amplifiers are DC coupled, so a DC sweep
pins down gain AND offset of each channel, and the signs come out right too:
"positive" ends up meaning hot above neutral and current flowing hot -> load,
so real power reads positive for a consuming load once mains is connected.

  Voltage   relay open, supply in CV, a sweep of setpoints; truth is the
            supply's own MEAS:VOLT? read either side of the monitor's !DC.
  Current   needs a short (or a low-ohm power resistor) in the outlet and
            the relay closed; supply in CC at a low voltage limit, a sweep of
            current setpoints; truth is MEAS:CURR?.  Run once per current
            range: HIGH (0.01 ohm, IGAIN/IOFF) and LOW (0.1 ohm, IGAINLO/IOFFLO).
  Phase     needs MAINS and a purely resistive load: the delay between the
            fundamentals of V and I in a raw capture IS the front ends' skew,
            and becomes PHASEUS.

Fits are real = gain * (adc - off), solved by least squares.  Nothing is
written to the board here -- the caller applies and saves.
"""

import time
from dataclasses import dataclass, field

import numpy as np

from monitor_link import capture_stats, fundamental


class CalError(Exception):
    pass


@dataclass
class Fit:
    what: str                       # "V" or "I"
    gain: float
    off: float                      # ADC volts
    points: list = field(default_factory=list)   # (truth, adc, sd)
    resid: list = field(default_factory=list)    # real units
    unit: str = "V"

    @property
    def worst(self):
        return max((abs(r) for r in self.resid), default=float("nan"))

    def report(self):
        lines = [f"{self.what}: gain {self.gain:.6g} {self.unit}/V(adc), offset {self.off * 1e3:+.4f} mV(adc)",
                 f"   {'truth':>10} {'adc V':>11} {'fit':>10} {'resid':>9}"]
        for (t, a, _), r in zip(self.points, self.resid):
            lines.append(f"   {t:10.4f} {a:11.6f} {t + r:10.4f} {r:+9.4f}")
        lines.append(f"   worst residual {self.worst:.4f} {self.unit}")
        return "\n".join(lines)


def fit_line(what, unit, points):
    truth = np.array([p[0] for p in points])
    adc = np.array([p[1] for p in points])
    if len(points) < 2 or np.ptp(truth) <= 0:
        raise CalError(f"{what}: need at least two distinct points")
    g, c = np.polyfit(adc, truth, 1)          # truth = g*adc + c
    if g == 0:
        raise CalError(f"{what}: the channel did not respond")
    off = -c / g
    resid = list(g * (adc - off) - truth)
    return Fit(what, float(g), float(off), list(points), [float(r) for r in resid], unit)


def _log(log, msg):
    if log:
        log(msg)


def _stopped(stop):
    if stop is not None and stop.is_set():
        raise CalError("stopped")


class _PsuSession:
    """Remembers the channel's setpoints and output state, and restores them
    (output OFF first) however the sweep ends."""

    def __init__(self, psu, ch, log):
        self.psu, self.ch, self.log = psu, ch, log

    def __enter__(self):
        st = self.psu.check_channel(self.ch)
        self.prev = st
        if st["on"]:
            _log(self.log, f"note: CH{self.ch} output was already ON ({st['vset']:g} V) -- switching it off first")
            self.psu.output(self.ch, False)
        return self

    def __exit__(self, *exc):
        try:
            self.psu.output(self.ch, False)
        finally:
            try:
                self.psu.set(self.ch, volts=self.prev["vset"], amps=self.prev["iset"])
            except Exception as e:                         # noqa: BLE001
                _log(self.log, f"could not restore CH{self.ch} setpoints: {e}")
        _log(self.log, f"CH{self.ch} output OFF, setpoints restored")
        return False


def check_no_mains(link, log=None, limit_vac=5.0):
    """A DC sweep onto live mains would be a very bad day for the supply.
    Refuses above limit_vac of AC (ripple RMS on the voltage channel, scaled
    by the board's present VGAIN -- the placeholder gain is close enough to
    tell 120 V mains from front-end noise)."""
    if "VGAIN" not in link.cfg:
        link.read_cfg()
    gain = abs(link.cfg["VGAIN"]["value"])
    d = link.dc(200)
    vac = d["v_sd"] * gain
    if vac > limit_vac:
        raise CalError(f"the voltage channel shows {vac:.2f} VAC (limit {limit_vac:g} V) -- is mains "
                       "connected? Disconnect it before running a supply sweep.")
    _log(log, f"input quiet ({vac:.3f} VAC, limit {limit_vac:g} V)")
    return d


def cal_voltage(link, psu, ch=2, points=(0, 5, 10, 15, 20, 25, 30), ilim=0.05,
                settle=1.0, dc_ms=500, log=None, stop=None, limit_vac=5.0):
    link.relay(False)
    check_no_mains(link, log, limit_vac)
    pts = []
    with _PsuSession(psu, ch, log):
        psu.set(ch, volts=float(points[0]), amps=float(ilim))
        psu.output(ch, True)
        for vset in points:
            _stopped(stop)
            psu.set(ch, volts=float(vset))
            time.sleep(settle)
            m1 = psu.meas_v(ch)
            d = link.dc(dc_ms)
            m2 = psu.meas_v(ch)
            truth = 0.5 * (m1 + m2)
            if abs(m1 - m2) > max(0.01, 0.002 * abs(truth)):
                _log(log, f"  warning: supply moved {m1:.4f} -> {m2:.4f} V during the reading")
            if d["flags"] & 2:
                raise CalError(f"voltage channel clipped at {truth:.3f} V")
            pts.append((truth, d["v_adc"], d["v_sd"]))
            _log(log, f"  {vset:6.2f} V set   {truth:9.4f} V meas   adc {d['v_adc']:+.6f} V  (sd {d['v_sd'] * 1e3:.2f} mV)")
    return fit_line("V", "V", pts)


def _check_range(d, rng, truth):
    if d.get("rng", 0) != rng or d.get("rng_changed"):
        raise CalError(f"the board left the {'LOW' if rng else 'HIGH'} range at {truth:.3f} A "
                       "(overload: lower the setpoints or raise RNGUPA)")


def cal_current(link, psu, ch=2, points=(0.25, 0.5, 1.0, 1.5, 2.0, 2.5), vlim=5.0,
                settle=1.5, dc_ms=500, log=None, stop=None, limit_vac=5.0, rng=0):
    """Needs a short / low-ohm load in the outlet; closes the relay itself.
    rng 0 calibrates the HIGH range (IGAIN/IOFF), 1 the LOW one (IGAINLO/IOFFLO)."""
    check_no_mains(link, log, limit_vac)
    if "RNGMODE" not in link.cfg:
        link.read_cfg()
    prev_mode = link.cfg.get("RNGMODE", {}).get("value")
    if rng and not link.cfg.get("RNGPIN", {}).get("value"):
        raise CalError("RNGPIN is 0: the board has no range relay configured")
    if prev_mode is not None:
        got = link.range_mode(2 if rng else 1)          # pin the range for the sweep
        if got != rng:
            raise CalError("the board would not switch range")
        _log(log, f"range pinned to {'LOW (IGAINLO)' if rng else 'HIGH (IGAIN)'}")
    try:
        fit = _sweep_current(link, psu, ch, points, vlim, settle, dc_ms, log, stop, rng)
    finally:
        if prev_mode is not None:
            link.range_mode(int(prev_mode))
    if rng:
        fit.what = "I LOW"
    return fit


def _sweep_current(link, psu, ch, points, vlim, settle, dc_ms, log, stop, rng):
    pts = []
    with _PsuSession(psu, ch, log):
        # 0 A point with the output genuinely off
        link.relay(True)
        time.sleep(0.3)
        d = link.dc(dc_ms)
        _check_range(d, rng, 0.0)
        pts.append((0.0, d["i_adc"], d["i_sd"]))
        _log(log, f"  output off      0.0000 A meas   adc {d['i_adc']:+.6f} V")
        try:
            psu.set(ch, volts=float(vlim), amps=float(points[0]))
            psu.output(ch, True)
            for iset in points:
                _stopped(stop)
                psu.set(ch, amps=float(iset))
                time.sleep(settle)
                m1 = psu.meas_i(ch)
                d = link.dc(dc_ms)
                m2 = psu.meas_i(ch)
                vout = psu.meas_v(ch)
                truth = 0.5 * (m1 + m2)
                if truth < 0.9 * iset:
                    _log(log, f"  warning: only {truth:.3f} A of {iset:g} A at {vout:.2f} V -- the load "
                              "is too high a resistance for the voltage limit (not in CC)")
                _check_range(d, rng, truth)
                if d["flags"] & 4:
                    raise CalError(f"current channel clipped at {truth:.3f} A")
                pts.append((truth, d["i_adc"], d["i_sd"]))
                _log(log, f"  {iset:6.3f} A set   {truth:9.4f} A meas   adc {d['i_adc']:+.6f} V   ({vout:.3f} V)")
        finally:
            psu.output(ch, False)
            link.relay(False)
    return fit_line("I", "A", pts)


def cal_phase(link, rate=20000, n=3000, cycles_min=5, log=None):
    """Mains + resistive load: the V->I delay of the fundamentals is PHASEUS."""
    cap = link.capture(rate=rate, n=n, ch=2, trig=2, pre=0, timeout_ms=2000)
    dt = cap.dt_us * 1e-6
    st = capture_stats(cap)
    if st["vrms"] < 20:
        raise CalError(f"only {st['vrms']:.1f} Vrms -- phase calibration needs mains")
    if st["irms"] < 0.05:
        raise CalError(f"only {st['irms'] * 1e3:.0f} mA -- connect a resistive load (heater, bulb)")
    if cap.n * dt * st["hz"] < cycles_min:
        raise CalError("capture too short for a phase measurement")
    fv, av, pv = fundamental(cap.v, dt, st["hz"])
    fi, ai, pi_ = fundamental(cap.i, dt, fv)
    dphi = (pv - pi_ + np.pi) % (2 * np.pi) - np.pi      # + : current lags voltage
    delay_us = dphi / (2 * np.pi * fv) * 1e6
    _log(log, f"V {av / np.sqrt(2):.2f} Vrms, I {ai / np.sqrt(2):.4f} Arms, f {fv:.3f} Hz, "
              f"I lags V by {np.degrees(dphi):+.3f} deg = {delay_us:+.2f} us "
              f"(raw PF {st.get('pf', float('nan')):.4f})")
    if abs(np.degrees(dphi)) > 10:
        raise CalError(f"{np.degrees(dphi):+.1f} deg is far too much for front-end skew -- "
                       "is the load really resistive? (or is a channel's polarity reversed?)")
    return delay_us
