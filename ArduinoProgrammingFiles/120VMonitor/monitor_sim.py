"""
monitor_sim.py  --  a pretend 120VMonitor + bench supply, for exercising the
GUI and the calibration flow without hardware.  Type SIM as the port.

The simulated front end deliberately disagrees with the firmware's default
gains (and has offsets and a 25 us current-path delay), so a calibration run
visibly has something to correct.  WORLD is shared between SimSerial and
SimPSU: the supply's CH2 drives the voltage input, and when the relay is
closed with a short plugged in, its current flows through the current sense.
"""

import math
import threading
import time

import numpy as np

# true front end: adc_volts = x / scale + offset
TRUE_V_SCALE, TRUE_V_OFF = 178.0, 0.0031
TRUE_I_SCALE, TRUE_I_OFF = 2.43, -0.0017
TRUE_I_DELAY = 25e-6
ADC_VREF = 3.3
LSB = ADC_VREF / 16384


class World:
    def __init__(self):
        self.mains = True           # mains on the input (ignored while the supply is on)
        self.load_ohm = 144.0       # what is plugged into the outlet (100 W bulb)
        self.inrush = True          # extra decaying current after a relay close
        self.psu_on = False
        self.psu_v = 0.0
        self.psu_i = 0.05
        self.short = 0.05           # ohms, used while the supply drives a closed relay
        self.relay = False
        self.relay_t = 0.0
        self.lock = threading.RLock()

    def psu_out(self):
        """(volts, amps) at the supply terminals."""
        if not self.psu_on:
            return 0.0, 0.0
        if self.relay:
            i = min(self.psu_i, self.psu_v / self.short)
            return i * self.short, i
        return self.psu_v, 0.0

    def vi(self, t):
        """True volts and amps at absolute times t (seconds)."""
        t = np.asarray(t, dtype=float)
        if self.psu_on:
            v, i = self.psu_out()
            return np.full_like(t, v), np.full_like(t, i)
        if not self.mains:
            return np.zeros_like(t), np.zeros_like(t)
        w = 2 * math.pi * 60.0
        v = 170.0 * np.sin(w * t) + 3.0 * np.sin(3 * w * t)
        if not self.relay:
            return v, np.zeros_like(t)
        i = v / self.load_ohm
        if self.inrush:
            age = np.clip(t - self.relay_t, 0, None)
            i = i * (1 + 6 * np.exp(-age / 0.04))
        return v, i


WORLD = World()


class SimSerial:
    def __init__(self):
        self.cfg = {"VGAIN": 170.0, "VOFF": 0.0, "IGAIN": 2.5, "IOFF": 0.0, "PHASEUS": 0.0,
                    "ADCVREF": 3.3, "MAINS": 60.0, "SPC": 128, "CYCLES": 10, "TRIPA": 0.0,
                    "TRIPPK": 0.0, "RLYPIN": 6, "RLYINV": 0, "OLED": 1, "OLEDADDR": 60,
                    "IRPIN": 9, "IRADDR": 0, "IROK": 28, "IRCONF": 22, "IRARMMS": 5000, "PIXEL": 40,
                    "LIVEVAC": 30.0}
        self.saved = dict(self.cfg)
        self.out = bytearray(b"$READY,120VMON,1.0-sim\r\n")
        self.lock = threading.Lock()
        self.streaming = False
        self.trip = False
        self.wh = 0.0
        self.last_win = time.time()
        self.t0 = time.time()
        self.is_open = True

    # ── pyserial surface ────────────────────────────────────────────────────
    def write(self, data):
        for line in data.decode().splitlines():
            if line.strip():
                self._cmd(line.strip())
        return len(data)

    def read(self, n=1):
        if self.streaming and time.time() - self.last_win >= 0.2:
            self._window()
        with self.lock:
            if not self.out:
                pass
            chunk = bytes(self.out[:n])
            del self.out[:n]
        if not chunk:
            time.sleep(0.02)
        return chunk

    def close(self):
        self.is_open = False

    # ── helpers ─────────────────────────────────────────────────────────────
    def _emit(self, s):
        with self.lock:
            self.out += (s + "\r\n").encode()

    def _adc(self, t, avg=1):
        v, i = WORLD.vi(t)
        _, i_late = WORLD.vi(np.asarray(t) - TRUE_I_DELAY)
        rng = np.random.default_rng()
        va = v / TRUE_V_SCALE + TRUE_V_OFF + rng.normal(0, 0.4e-3 / np.sqrt(avg), np.shape(t))
        ia = i_late / TRUE_I_SCALE + TRUE_I_OFF + rng.normal(0, 0.4e-3 / np.sqrt(avg), np.shape(t))
        vc = np.round(va / LSB)
        ic = np.round(ia / LSB)
        return vc, ic

    def _real(self, vc, ic):
        c = self.cfg
        return (vc * LSB - c["VOFF"]) * c["VGAIN"], (ic * LSB - c["IOFF"]) * c["IGAIN"]

    def _cfgrow(self, k):
        lo, hi = (-1e6, 1e6)
        return f"$CFG,{k},{self.cfg[k]:.6f},{lo},{hi},sim"

    def _relay(self, on):
        with WORLD.lock:
            if on and not WORLD.relay:
                WORLD.relay_t = time.time()
            WORLD.relay = on
        self._emit(f"$RLY,{1 if on else 0}")

    # ── metering ────────────────────────────────────────────────────────────
    def _window(self):
        now = time.time()
        dt_win = now - self.last_win
        self.last_win = now
        t = now + np.arange(0, 10 / 60.0, 1 / 7680.0)
        vc, ic = self._adc(t)
        v, i = self._real(vc, ic)
        vm, im = v.mean(), i.mean()
        vrms = float(np.sqrt(np.mean((v - vm) ** 2)))
        irms = float(np.sqrt(np.mean((i - im) ** 2)))
        p = float(np.mean((v - vm) * (i - im)))
        s = vrms * irms
        pf = p / s if s > 1e-9 else 0.0
        q = math.sqrt(max(s * s - p * p, 0))
        self.wh += p * dt_win / 3600
        flags = 0 if WORLD.mains and not WORLD.psu_on else 1
        if self.cfg["TRIPA"] > 0 and irms > self.cfg["TRIPA"] and WORLD.relay:
            self.trip = True
            self._relay(False)
            self._emit(f"$TRIP,RMS,{irms:.4f},{self.cfg['TRIPA']:.4f}")
        flags |= 8 if self.trip else 0
        hz = 60.0 if not flags & 1 else 0.0
        self._emit(f"$PWR,{vrms:.4f},{irms:.5f},{p:.4f},{s:.4f},{q:.4f},{pf:.4f},{hz:.3f},"
                   f"{np.max(np.abs(v - vm)):.3f},{np.max(np.abs(i - im)):.5f},{self.wh:.6f},"
                   f"{now - self.t0:.2f},1536,24,{1 if WORLD.relay else 0},{flags},{vm:.4f},{im:.5f}")

    # ── commands ────────────────────────────────────────────────────────────
    def _cmd(self, cmd):
        p = cmd.split(",")
        c = p[0].upper()
        if c == "!START":
            self.streaming = True; self._emit("$OK,start")
        elif c == "!STOP":
            self.streaming = False; self._emit("$OK,stop")
        elif c == "!ID":
            self._emit("$READY,120VMON,1.0-sim")
        elif c == "!STATUS":
            self._emit(f"$STATUS,fw=1.0-sim,stream={int(self.streaming)},relay={int(WORLD.relay)},"
                       f"trip={int(self.trip)},dirty={int(self.cfg != self.saved)},scanus4=4.2,"
                       f"scanus2=2.2,capmax=7168,oled=1,ch=9/0/1/2,wh={self.wh:.6f}")
        elif c == "!CFG":
            for k in self.cfg:
                self._emit(self._cfgrow(k))
            self._emit("$CFGEND")
        elif c == "!GET" and len(p) > 1 and p[1].upper() in self.cfg:
            self._emit(self._cfgrow(p[1].upper()))
        elif c == "!SET" and len(p) > 2 and p[1].upper() in self.cfg:
            self.cfg[p[1].upper()] = float(p[2])
            self._emit(self._cfgrow(p[1].upper()))
        elif c == "!SAVE":
            self.saved = dict(self.cfg); self._emit("$OK,save")
        elif c == "!LOAD":
            self.cfg = dict(self.saved); self._emit("$OK,load")
        elif c == "!RELAY" and len(p) > 1:
            on = p[1] != "0"
            if on:
                self.trip = False
            self._relay(on)
        elif c == "!ZERO":
            vc, ic = self._adc(time.time() + np.arange(0, 0.2, 1e-4))
            self.cfg["VOFF"], self.cfg["IOFF"] = vc.mean() * LSB, ic.mean() * LSB
            self._emit(self._cfgrow("VOFF")); self._emit(self._cfgrow("IOFF")); self._emit("$OK,zero")
        elif c == "!EZERO":
            self.wh = 0.0; self._emit("$OK,ezero")
        elif c == "!DC":
            ms = float(p[1]) if len(p) > 1 else 200
            time.sleep(ms / 1000)
            vc, ic = self._adc(time.time() + np.arange(0, ms / 1000, 1e-4))
            va, ia = vc.mean() * LSB, ic.mean() * LSB
            v, i = self._real(np.array([vc.mean()]), np.array([ic.mean()]))
            self._emit(f"$DC,{va:.7f},{ia:.7f},{v[0]:.5f},{i[0]:.6f},{vc.std() * LSB:.7f},"
                       f"{ic.std() * LSB:.7f},{len(vc) * 40},0")
        elif c == "!CAP":
            self._capture(p[1:])
        elif c == "!ABORT":
            pass
        else:
            self._emit(f"$ERR,unknown command: {cmd}")

    def _capture(self, a):
        f = [float(x) for x in a] + [None] * 14
        rate = f[0] or 200000.0
        n, ch, trig = int(f[1] or 4000), int(f[2] if f[2] is not None else 2), int(f[3] or 0)
        level, pre = f[4] if f[4] is not None else 0.5, f[5] if f[5] is not None else 10
        relay_us = f[7] if f[7] is not None else -1
        ipk_a, ov_a = f[8] or 0.0, f[9] or 0.0
        ov_cyc, auto_ms, open_after = int(f[10] or 3), f[11] or 0.0, bool(f[12])
        avg = min(max(int(f[13] or 1), 1), 64)
        n = min(n, 7168 // (2 if ch == 2 else 1))
        dt = 1.0 / rate
        pre_n = int(n * pre / 100) if trig else 0
        start = time.time()
        relay_idx = -1
        if trig == 3:
            # close at the next rising zero crossing + relay_us (or at once)
            t_close = start + pre_n * dt
            if relay_us >= 0:
                t_close = math.ceil(t_close * 60) / 60 + relay_us * 1e-6
            WORLD.relay_t = t_close
            WORLD.relay = True
            t = t_close + (np.arange(n) - pre_n) * dt
            relay_idx = pre_n
            v, _ = WORLD.vi(t)
            vc, ic = self._adc(t, avg)
            ic = np.where(np.arange(n) < pre_n, np.round(TRUE_I_OFF / LSB), ic)
        else:
            t = start + np.arange(n) * dt
            if trig == 2:
                t = t + (math.ceil(t[pre_n] * 60) / 60 - t[pre_n])
            vc, ic = self._adc(t, avg)
        trig_idx = pre_n if trig else 0
        # safe start: find where the relay would have opened, then zero the current after it
        open_idx, why = -1, 0
        if WORLD.relay and ch != 0:
            i_real = (ic * LSB - self.cfg["IOFF"]) * self.cfg["IGAIN"]
            k0 = trig_idx
            cands = []
            if ipk_a > 0:
                hit = np.nonzero(np.abs(i_real) > ipk_a)[0]
                if len(hit):
                    cands.append((int(hit[0]), 1))
            if ov_a > 0:
                per = int(round(rate / 60.0))
                run = 0
                for c in range((n - k0) // per):
                    seg = i_real[k0 + c * per:k0 + (c + 1) * per]
                    run = run + 1 if np.sqrt(np.mean(seg ** 2)) > ov_a else 0
                    if run >= ov_cyc:
                        cands.append((k0 + (c + 1) * per - 1, 2))
                        break
            if auto_ms > 0 and k0 + int(auto_ms / 1000 * rate) < n:
                cands.append((k0 + int(auto_ms / 1000 * rate), 3))
            if cands:
                open_idx, why = min(cands)
                ic = np.where(np.arange(n) > open_idx, np.round(TRUE_I_OFF / LSB), ic)
                WORLD.relay = False
        if open_idx < 0 and open_after and WORLD.relay:
            open_idx, why = n - 1, 4
            WORLD.relay = False
        time.sleep(n * dt)
        c = self.cfg
        hdr = (f"$CAP,{n},{ch},{dt * 1e6:.4f},{LSB * c['VGAIN']:.9f},{c['VOFF'] / LSB:.3f},"
               f"{LSB * c['IGAIN']:.9f},{c['IOFF'] / LSB:.3f},{trig_idx},0,{relay_idx},{open_idx},{why},{avg}")
        self._emit(hdr)
        if ch == 2:
            vals = np.column_stack([vc, ic]).ravel()
        else:
            vals = vc if ch == 0 else ic
        vals = np.clip(vals, -32768, 32767).astype(np.int16).view(np.uint16)
        for k in range(0, len(vals), 32):
            self._emit("$CD," + "".join(f"{x:04X}" for x in vals[k:k + 32]))
        if trig == 3:
            self._emit("$RLY,1")
        if open_idx >= 0:
            self._emit("$RLY,0")
            self._emit(f"$SAFE,{['NONE','INST','OVERCURRENT','TIMER','END'][why]},0")
        self._emit(f"$CEND,{n}")


class SimPSU:
    name = "simulated supply"

    def ident(self):
        return "Siglent,SPD4323X-SIM"

    def query(self, cmd):
        c = cmd.upper()
        if c.startswith("OUTP:TRACK?"):
            return "0"
        if c.startswith("OUTP?"):
            return "ON" if WORLD.psu_on else "OFF"
        if c.startswith("VOLT?"):
            return f"{WORLD.psu_v:.3f}"
        if c.startswith("CURR?"):
            return f"{WORLD.psu_i:.3f}"
        if c.startswith("MEAS:VOLT?"):
            return f"{WORLD.psu_out()[0] + np.random.normal(0, 0.0005):.4f}"
        if c.startswith("MEAS:CURR?"):
            return f"{WORLD.psu_out()[1] + np.random.normal(0, 0.0003):.4f}"
        return ""

    def qf(self, cmd):
        return float(self.query(cmd))

    def meas_v(self, ch):
        return self.qf(f"MEAS:VOLT? CH{ch}")

    def meas_i(self, ch):
        return self.qf(f"MEAS:CURR? CH{ch}")

    def setpoints(self, ch):
        return WORLD.psu_v, WORLD.psu_i

    def is_on(self, ch):
        return WORLD.psu_on

    def check_channel(self, ch):
        return {"on": WORLD.psu_on, "vset": WORLD.psu_v, "iset": WORLD.psu_i}

    def set(self, ch, volts=None, amps=None):
        if volts is not None:
            WORLD.psu_v = float(volts)
        if amps is not None:
            WORLD.psu_i = float(amps)

    def output(self, ch, on):
        WORLD.psu_on = bool(on)

    def close(self):
        pass
