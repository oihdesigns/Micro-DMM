"""A stand-in for the Giga + ADS1256, speaking GigaADS1256_WiFi's protocol over TCP.

    py -3 gads_sim.py [host:port]        (default 127.0.0.1:8080)

then type 127.0.0.1 in gads_gui.py's Board box. Each channel carries its own
test signal (volts):

    CH0  1 V sine, 5 Hz            CH3     2.0 V DC + noise (14.66 kOhm in DMM math)
    CH1  2.5 V DC + 50 uV noise    CH4-5   +-0.5 V square, 10 Hz
    CH2  0..2 V sawtooth, 2 Hz     CH6-7   50 mV sine, 50 Hz

Captures use the firmware's frame / trigger / pre-trigger rules (runCapture()
in the .ino) on a simulated time line, paced in real time (up to 10 s), so the
GUI sees $CAPARM, $CAPTRIG and the data in the same order as from the board.
Nothing here models the ADC's analog behaviour.
"""

import math
import socket
import sys
import threading
import time

import numpy as np

DRATES = {0xF0: 30000, 0xE0: 15000, 0xD0: 7500, 0xC0: 3750, 0xB0: 2000, 0xA1: 1000, 0x92: 500,
          0x82: 100, 0x72: 60, 0x63: 50, 0x53: 30, 0x43: 25, 0x33: 15, 0x23: 10, 0x13: 5, 0x03: 2.5}
# ADS1256 datasheet, Table 14: conversions per second when cycling the mux
CYCLE = {0xF0: 4374, 0xE0: 3817, 0xD0: 3043, 0xC0: 2165, 0xB0: 1356, 0xA1: 772, 0x92: 409,
         0x82: 88, 0x72: 54, 0x63: 45, 0x53: 27.8, 0x43: 23.3, 0x33: 14.2, 0x23: 9.5, 0x13: 4.9, 0x03: 2.5}
GPIO_PINS = [3, 4, 5, 8, 9] + list(range(22, 34))
VREF = 2.5
FULL = 0x7FFFFF
RNG = np.random.default_rng(1)


def pnoise(t, sd, seed):
    """Noise that is a fixed function of time (an integer hash of the time in
    0.1 us), so a sample regenerated for the capture output equals the one the
    trigger search saw. Uniform, scaled to standard deviation sd."""
    n = np.round(np.asarray(t, dtype=float) * 1e7).astype(np.uint64) + np.uint64(seed * 7919)
    with np.errstate(over="ignore"):                                  # wraps on purpose
        h = (n * np.uint64(0x9E3779B97F4A7C15)) >> np.uint64(40)      # 24 bits
    return sd * 3.4641 * (h.astype(float) / 16777216.0 - 0.5)


def signal(ch, t):
    t = np.asarray(t, dtype=float)
    if ch == 0:
        return 1.0 * np.sin(2 * np.pi * 5 * t)
    if ch == 1:
        return 2.5 + pnoise(t, 50e-6, 11)
    if ch == 2:
        return 2.0 * ((t * 2) % 1.0)
    if ch == 3:
        return 2.0 + pnoise(t, 20e-6, 13)
    if ch == 4:
        return np.where(np.sin(2 * np.pi * 10 * t) >= 0, 0.5, -0.5)
    if ch == 5:
        return 0.05 * np.sin(2 * np.pi * 50 * t)
    return np.zeros_like(t)


def codes(ch, t, pga):
    v = signal(ch, t) * (1 << pga) * FULL / (2 * VREF)
    v = v + pnoise(t, 2.0, ch)
    return np.clip(np.round(v), -0x800000, 0x7FFFFF).astype(np.int64)


class Board:
    def __init__(self):
        self.drate, self.pga, self.buf = 0xF0, 0, 0
        self.nmap, self.mask = 6, 0x3F
        self.live, self.live_ms = True, 500
        self.range_hi = 1
        self.gpio = {p: 0 for p in GPIO_PINS}
        self.client = None
        self.wlock = threading.Lock()
        self.capturing = False
        self.abort = False
        self.t0 = time.monotonic()

    # -- output --
    def out(self, line):
        c = self.client
        if c is None:
            return
        try:
            with self.wlock:
                c.sendall((line + "\r\n").encode())
        except OSError:
            self.client = None

    def banner(self):
        self.out("$READY,GIGA_ADS1256,sim")
        self.status()
        self.out("$NET,127.0.0.1,8080,-50,3,0,SIM,1,1,1")
        self.out("$GPIOLIST," + ",".join(map(str, GPIO_PINS)))

    def status(self):
        self.out(f"$STATUS,fw=sim,adc=OK,streg=0x30,drate=0x{self.drate:02X},sps={DRATES[self.drate]},"
                 f"pga={self.pga},buf={self.buf},nmap={self.nmap},mask=0x{self.mask:02X},live={int(self.live)},"
                 f"livems={self.live_ms},conv_s={CYCLE[self.drate]:.1f},drdy_tmo=0,cap={int(self.capturing)},"
                 f"capmax_frames=524288,capmax_vals=1048576,range={self.range_hi},cal_ms=1,ip=127.0.0.1,rssi=-50,"
                 f"ssid=SIM,net=1,uptime_s={int(time.monotonic() - self.t0)},loopms=0.1,txdrops=0,tcpbusy=0")

    # -- live --
    def live_loop(self):
        last = time.monotonic()
        while True:
            time.sleep(self.live_ms / 1000.0)
            if not self.live or self.capturing or self.client is None:
                last = time.monotonic()
                continue
            now = time.monotonic()
            chans = [i for i in range(self.nmap) if self.mask & (1 << i)]
            if not chans:
                continue
            period = now - last
            last = now
            frames = max(1, int(period * CYCLE[self.drate] / len(chans)))
            t = now - self.t0 + np.arange(frames) / (CYCLE[self.drate] / len(chans))
            parts = [f"$LIVE,{frames},{period * 1000:.1f},{len(chans)}"]
            for ch in chans:
                c = codes(ch, t, self.pga)
                parts.append(f"{ch},{c.mean():.2f},{c.std():.2f},{c.min()},{c.max()},"
                             f"{int(c.max() >= FULL or c.min() <= -0x800000)}")
            self.out(",".join(parts))

    # -- capture: the firmware's rules on a simulated time line --
    def capture(self, mode, n, mask, trig, armed, level, slope, pos, timeout_ms):
        order = [trig] + ([i for i in range(self.nmap) if mask & (1 << i) and i != trig] if mode else [])
        nch = len(order)
        pre = 0 if not armed else (n // 2 if pos == 1 else n - 1 if pos == 2 else 0)
        dt = 1.0 / DRATES[self.drate] if mode == 0 else 1.0 / CYCLE[self.drate]   # per conversion
        frame_dt = dt if mode == 0 else dt * nch
        self.out(f"$OK,CAP,{n},{nch},{':'.join(map(str, order))},{pre}")
        start = time.monotonic() - self.t0
        if armed:
            self.out("$CAPARM")
        # find the trigger frame: frames are recorded from f = 0; the trigger
        # may fire on frame f >= pre (and f >= 1, it needs a previous value)
        trig_f = 0
        if armed:
            block, f0, prev, trig_f = 65536, 0, None, -1
            limit = timeout_ms / 1000.0 if timeout_ms else 60.0
            while trig_f < 0:
                t = start + (f0 + np.arange(block)) * frame_dt
                v = codes(order[0], t, self.pga)
                p = np.concatenate([[v[0] if prev is None else prev], v[:-1]])
                hit = (p < level) & (level <= v) if slope == 0 else (p > level) & (level >= v)
                idx = f0 + np.arange(block)
                ok = hit & (idx >= pre) & (idx >= 1)
                if ok.any():
                    trig_f = int(idx[np.argmax(ok)])
                    break
                prev = v[-1]
                f0 += block
                if f0 * frame_dt > limit:
                    if not self.wait(min(limit, 10.0)):
                        return
                    self.out("$CAPTMO,TIMEOUT")
                    return
            if not self.wait(min(trig_f * frame_dt, 10.0)):
                return
            self.out(f"$CAPTRIG,{int(trig_f * frame_dt * 1000)}")
        first = trig_f - pre
        idx = first + np.arange(n)
        tf = start + idx * frame_dt
        cols = [codes(ch, tf + k * dt, self.pga) for k, ch in enumerate(order)]
        if not self.wait(min(n * frame_dt, 10.0)):
            return
        ts = np.round((tf - tf[0]) * 1e6 + RNG.normal(0, 0.5, n)).astype(np.int64)
        ts[0] = 0
        flags = 2 if any((c >= FULL).any() or (c <= -0x800000).any() for c in cols) else 0
        self.out(f"$CAP,{n},{nch},{':'.join(map(str, order))},{pre if armed else -1},0,{flags},"
                 f"{frame_dt * 1e6:.3f},0x{self.drate:02X},{self.pga},{self.buf}")
        fpl = max(1, 240 // (8 + 6 * nch))
        for s in range(0, n, fpl):
            if self.abort:
                self.out("$CAPTMO,ABORT")
                return
            e = min(n, s + fpl)
            chunks = []
            for i in range(s, e):
                chunks.append(f"{int(ts[i]) & 0xFFFFFFFF:08X}" + "".join(f"{int(c[i]) & 0xFFFFFF:06X}" for c in cols))
            self.out(f"$CD,{s}," + "".join(chunks))
        self.out(f"$CEND,{n}")

    def wait(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self.abort:
                self.out("$CAPTMO,ABORT")
                return False
            time.sleep(0.02)
        return True

    def run_capture(self, args):
        try:
            self.capture(*args)
        finally:
            self.capturing = False

    # -- commands --
    def command(self, line):
        name, _, rest = line.partition(",")
        a = [int(x, 0) for x in rest.split(",")] if rest else []
        busy = self.capturing
        if name == "!ID":
            self.out("$READY,GIGA_ADS1256,sim")
        elif name == "!STATUS":
            self.status()
        elif name == "!NET":
            self.out("$NET,127.0.0.1,8080,-50,3,0,SIM,1,1,1")
        elif name in ("!DRATE", "!GAIN", "!BUF", "!CAL"):
            if busy:
                self.out("$ERR,busy: capture running")
                return
            if name == "!DRATE":
                if a[0] not in DRATES:
                    self.out("$ERR,DRATE: not a DRATE code")
                    return
                self.drate = a[0]
                self.out(f"$OK,DRATE,0x{a[0]:02X}")
            elif name == "!GAIN":
                self.pga = a[0]
                self.out(f"$OK,GAIN,{a[0]}")
            elif name == "!BUF":
                self.buf = int(bool(a[0]))
                self.out(f"$OK,BUF,{self.buf}")
            else:
                self.out("$OK,CAL")
            self.out("$CAL,1,3")
        elif name == "!CHMAP":
            self.nmap = len(a)
            self.mask &= (1 << self.nmap) - 1
            self.out(f"$OK,CHMAP,{len(a)}")
        elif name == "!CH":
            self.mask = a[0] & ((1 << self.nmap) - 1)
            self.out(f"$OK,CH,0x{self.mask:02X}")
        elif name == "!LIVE":
            if a:
                self.live = bool(a[0])
            if len(a) > 1:
                self.live_ms = max(50, min(10000, a[1]))
            self.out(f"$OK,LIVE,{int(self.live)},{self.live_ms}")
        elif name == "!RANGE":
            self.range_hi = int(bool(a[0]))
            self.out(f"$OK,RANGE,{self.range_hi}")
        elif name == "!CAP":
            if busy:
                self.out("$ERR,busy: a capture is running")
                return
            if len(a) < 9:
                self.out("$ERR,usage: !CAP,mode,n,mask,trig_ch,armed,level,slope,pos,timeout_ms")
                return
            self.capturing, self.abort = True, False
            threading.Thread(target=self.run_capture, args=(a,), daemon=True).start()
        elif name == "!ABORT":
            self.abort = True
            if not busy:
                self.out("$OK,ABORT,idle")
        elif name == "!REG":
            self.out(f"$REG,{0x30 | (2 if self.buf else 0):02X},01,{self.pga:02X},{self.drate:02X},E1,00,00,00,00,00,00")
        elif name == "!REINIT":
            self.out("$OK,REINIT")
            self.out("$ADC,OK,0x30")
        elif name == "!GPIOLIST":
            self.out("$GPIOLIST," + ",".join(map(str, GPIO_PINS)))
        elif name == "!GPIOS":
            self.out("$GPIOS," + ",".join(f"{p}:{m}:{self.level(p)}" for p, m in self.gpio.items()))
        elif name == "!GPIO":
            if len(a) < 2 or a[0] not in self.gpio:
                self.out("$ERR,GPIO: not a user pin (see !GPIOLIST)")
                return
            self.gpio[a[0]] = a[1]
            self.out(f"$GPIO,{a[0]},{a[1]},{self.level(a[0])}")
        else:
            self.out(f"$ERR,unknown command: {line}")

    def level(self, pin):
        m = self.gpio[pin]
        return 1 if m in (2, 3) else 0

    def serve(self, host, port):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, port))
        srv.listen(2)
        threading.Thread(target=self.live_loop, daemon=True).start()
        print(f"gads_sim listening on {host}:{port}")
        while True:
            c, addr = srv.accept()
            print("client", addr)
            old, self.client = self.client, c          # newest connection wins, as on the board
            if old is not None:
                try:
                    old.close()
                except OSError:
                    pass
            self.banner()
            threading.Thread(target=self.reader, args=(c,), daemon=True).start()

    def reader(self, c):
        buf = b""
        while True:
            try:
                data = c.recv(4096)
            except OSError:
                break
            if not data:
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.decode(errors="replace").strip()
                if line and c is self.client:
                    try:
                        self.command(line)
                    except (ValueError, IndexError) as exc:
                        self.out(f"$ERR,bad command: {line} ({exc})")
        if self.client is c:
            self.client = None


if __name__ == "__main__":
    target = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1:8080"
    h, _, p = target.partition(":")
    Board().serve(h or "127.0.0.1", int(p or 8080))
