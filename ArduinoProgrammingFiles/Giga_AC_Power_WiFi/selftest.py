#!/usr/bin/env python3
"""
End-to-end self-test for Giga_AC_Power_WiFi, through the GUI's own link
(monitor_link.MonitorLink). It needs no wiring: it switches the board to its
synthetic source (!SYNTH), which still runs at the ADC's pace, and checks what
comes back against the values it asked for.

  py -3 selftest.py 192.168.40.225        # Wi-Fi (TCP port 8080)
  py -3 selftest.py COM36                 # USB serial

  1. accuracy: Vrms, Irms, P, PF, Hz at several phase angles, with a 3rd
     harmonic, at 60 and 50 Hz (and THD from !HGET)
  2. streaming: $PWR for a while with a capture in the middle; no window
     missed, no samples lost ($STATUS gaps)
  3. captures: V rising zero trigger lands on the crossing; an |I| trigger
     above the signal times out; one below it fires on the right sample
  4. inrush: "close the relay n us after a rising zero" closes at that phase
     (the relay_idx sample sits where the sine says), and "open when capture
     completes" opens it again. The relay clicks; keep the load off.
  5. safe start: an instant trip below the signal's peak opens the relay
     ($SAFE,INST) and the capture says where
  6. current range: auto drops to LOW on a small current after RNGHOLD and
     goes HIGH at once on a big one, reading right on both
  7. config: !SET round trip and a refused value, and !DC
It returns the board to the ADC (!SYNTH,0) with the relay open at the end.
"""
import math
import queue
import sys
import time

import numpy as np

from monitor_link import LinkError, MonitorLink, capture_stats


def pwr_rows(link, seconds):
    """$PWR records arriving within `seconds` (other events dropped)."""
    rows, t_end = [], time.time() + seconds
    while time.time() < t_end:
        try:
            kind, obj = link.events.get(timeout=0.1)
        except queue.Empty:
            continue
        if kind == "pwr":
            rows.append(obj)
    return rows


def drain(link):
    while True:
        try:
            link.events.get_nowait()
        except queue.Empty:
            return


def lines(link, seconds, prefix):
    out, t_end = [], time.time() + seconds
    while time.time() < t_end:
        try:
            kind, obj = link.events.get(timeout=0.1)
        except queue.Empty:
            continue
        if kind == "line" and obj.startswith(prefix):
            out.append(obj)
    return out


def check(name, got, want, tol_rel=None, tol_abs=None):
    err = got - want
    ok = not ((tol_abs is not None and abs(err) > tol_abs) or
              (tol_rel is not None and want != 0 and abs(err / want) > tol_rel))
    rel = f"{100 * err / want:+.4f} %" if want else f"{err:+.5f}"
    print(f"    {'ok  ' if ok else 'FAIL'} {name:6s} {got:12.5f}  want {want:12.5f}  ({rel})")
    return ok


def synth(link, vrms, irms, deg, h3=0.0, hz=60.0):
    link.send(f"!SYNTH,1,{vrms},{irms},{deg},{h3},{hz}")


def accuracy(link):
    print("1. accuracy, synthetic V/I through the full pipeline")
    allok = True
    cases = [  # vrms, irms, deg, h3 %, hz
        (120.0, 5.0, 0.0, 0.0, 60.0),
        (120.0, 5.0, 30.0, 0.0, 60.0),
        (120.0, 5.0, 60.0, 0.0, 60.0),
        (120.0, 5.0, -45.0, 0.0, 60.0),
        (120.0, 2.0, 0.0, 30.0, 60.0),
        (115.0, 10.0, 25.0, 0.0, 50.0),
        (120.0, 5.0, 30.0, 0.0, 59.7),
    ]
    for vrms, irms, deg, h3, hz in cases:
        link.set("MAINS", 50.0 if hz < 55 else 60.0)
        synth(link, vrms, irms, deg, h3, hz)
        pwr_rows(link, 1.2)                   # let the analyser lock and the range settle
        rows = pwr_rows(link, 2.0)
        if len(rows) < 3:
            print(f"  FAIL: only {len(rows)} $PWR lines"); allok = False; continue
        r = rows[-1]
        i1 = irms / math.sqrt(1 + (h3 / 100) ** 2)
        p = vrms * i1 * math.cos(math.radians(deg))
        print(f"  V {vrms} V, I {irms} A, {deg} deg, h3 {h3} %, {hz} Hz   flags {int(r['flags'])} rng {int(r['rng'])}")
        allok &= check("vrms", r["vrms"], vrms, tol_rel=2e-4)
        allok &= check("irms", r["irms"], irms, tol_rel=2e-4)
        allok &= check("P", r["p"], p, tol_abs=max(2e-4 * vrms * irms, 0.01))
        allok &= check("PF", r["pf"], p / (vrms * irms), tol_abs=3e-4)
        allok &= check("Hz", r["hz"], hz, tol_rel=1e-4)
        harm = link.request("!HGET", ("$HARM",), 2.0).split(",")
        allok &= check("ITHD", float(harm[5]), h3, tol_abs=0.05)
    link.set("MAINS", 60.0)
    return allok


def streaming(link, seconds=15):
    print(f"2. streaming: $PWR for {seconds} s, with a 20 kS/s capture in the middle")
    synth(link, 120, 5, 30, 10, 60)
    pwr_rows(link, 1.0)
    st0 = dict(link.get_status())
    drain(link)
    t0 = time.time()
    rows = pwr_rows(link, seconds / 2)
    cap = link.capture(rate=20000, n=8000, ch=2, trig=0, pre=0, timeout_ms=1000, avg=4)
    rows += pwr_rows(link, seconds / 2)
    el = time.time() - t0
    st1 = dict(link.get_status())
    ts = np.array([r["uptime"] for r in rows])
    d = np.diff(ts)
    win = 12 / 60.0
    missed = int(np.sum(np.round(d / win) - 1)) if len(d) else 0
    print(f"   {len(rows)} $PWR in {el:.1f} s ({len(rows) / el:.2f}/s), windows missed {missed}, "
          f"gaps {st0['gaps']} -> {st1['gaps']}, loop max {st1['loopms']} ms")
    s = capture_stats(cap)
    want_p = 120 * (5 / math.sqrt(1.01)) * math.cos(math.radians(30))
    print(f"   capture: {cap.n} samples, dt {cap.dt_us} us, avg {cap.avg}, P {s['p']:.2f} W (want {want_p:.2f})")
    ok = missed == 0 and st1["gaps"] == st0["gaps"] and cap.n == 8000
    ok &= abs(s["p"] - want_p) / want_p < 0.002
    return ok


def captures(link):
    print("3. capture triggers")
    synth(link, 120, 5, 30, 0, 60)
    pwr_rows(link, 0.8)
    ok = True
    cap = link.capture(rate=40000, n=2000, ch=2, trig=2, pre=25, timeout_ms=2000, avg=2)
    k = cap.trig_idx
    vpk = 120 * math.sqrt(2)
    slope = 2 * math.pi * 60 * vpk * cap.dt_us * 1e-6          # volts per sample at the crossing
    print(f"   V rising zero: trig_idx {k}, v {cap.v[k - 1]:+.2f} -> {cap.v[k]:+.2f} V "
          f"(a sample is {slope:.1f} V of the slope there)")
    ok &= bool(cap.v[k - 1] < 0 <= cap.v[k] + 1e-9 and cap.v[k] < 2 * slope)
    try:
        link.capture(rate=20000, n=1000, ch=2, trig=1, level=50.0, pre=10, timeout_ms=600)
        print("   FAIL: an |I| >= 50 A trigger fired on a 7 A peak"); ok = False
    except LinkError as e:
        print(f"   |I| >= 50 A on a 7 A peak: {e} (right)")
        ok &= "timed out" in str(e)
    cap = link.capture(rate=10000, n=3000, ch=2, trig=1, level=6.5, pre=50, timeout_ms=2000, avg=4)
    ipk = abs(cap.i[cap.trig_idx])
    print(f"   |I| >= 6.5 A: trig_idx {cap.trig_idx}, |i| there {ipk:.3f} A")
    ok &= bool(ipk >= 6.4)
    return ok


def inrush(link):
    print("4. inrush: close the relay n us after a rising zero")
    synth(link, 120, 1, 0, 0, 60)
    link.relay(False)
    pwr_rows(link, 1.5)
    ok = True
    vpk = 120 * math.sqrt(2)
    for us in (4167, 0, 2083):
        cap = link.capture(rate=40000, n=1600, ch=2, trig=3, relay_us=us, pre=25, timeout_ms=2000,
                           open_after=True, avg=2)
        k = cap.relay_idx
        want_ph = (360.0 * 60 * us * 1e-6) % 360
        # the phase of the V cycle at relay_idx, from the samples around it
        if k > 2:
            v, dv = cap.v[k], (cap.v[k + 1] - cap.v[k - 1]) / 2
            ph = math.degrees(math.atan2(v / vpk, dv / (vpk * 2 * math.pi * 60 * cap.dt_us * 1e-6))) % 360
        else:
            ph = float("nan")
        err = (ph - want_ph + 180) % 360 - 180
        print(f"   relay_us {us:4d}: relay_idx {k}, v there {cap.v[k]:+8.2f} V, phase {ph:6.1f} deg "
              f"(want {want_ph:5.1f}, off {err:+.2f} deg = {err / 360 / 60 * 1e6:+.0f} us), "
              f"open_idx {cap.open_idx} why {cap.open_why}")
        ok &= k >= 0 and abs(err) < 2.0 and cap.open_why == 4
    st = link.get_status()
    print(f"   relay after the captures: {st['relay']}")
    ok &= st["relay"] == "0"
    return ok


def safe_start(link):
    print("5. safe start: instant trip at 6 A on a 7.07 A peak")
    synth(link, 120, 5, 0, 0, 60)
    pwr_rows(link, 0.8)
    drain(link)
    cap = link.capture(rate=20000, n=2000, ch=2, trig=5, level=0.5, pre=10, timeout_ms=2000, ipk_a=6.0, avg=4)
    safe = lines(link, 0.5, "$SAFE")
    print(f"   relay_idx {cap.relay_idx}, open_idx {cap.open_idx}, why {cap.open_why}; {safe}")
    ok = cap.open_why == 1 and cap.open_idx >= cap.relay_idx >= 0
    st = link.get_status()
    ok &= st["relay"] == "0" and st["trip"] == "1"
    link.relay(False)
    return ok


def ranges(link):
    print("6. current range, auto (RNGUPA 1.6 A, RNGDNA 1.0 A, RNGHOLD 3 s)")
    link.range_mode(0)
    synth(link, 120, 0.5, 30, 0, 60)          # 0.71 A peak: under RNGDNA
    rows = pwr_rows(link, 5.0)
    seen = [int(r["rng"]) for r in rows]
    print(f"   0.5 A rms: ranges {sorted(set(seen))}, last {seen[-1] if rows else '-'} (1 = LOW), "
          f"irms {rows[-1]['irms']:.4f}")
    ok = bool(rows) and seen[-1] == 1 and abs(rows[-1]["irms"] - 0.5) < 1e-3
    drain(link)
    t0 = time.time()
    synth(link, 120, 3.0, 30, 0, 60)          # 4.2 A peak: an overload on LOW
    got, dt = None, None
    while time.time() - t0 < 1.5 and got is None:
        try:
            kind, obj = link.events.get(timeout=0.05)
        except queue.Empty:
            continue
        if kind == "line" and obj.startswith("$RNG,0"):
            got, dt = obj, time.time() - t0
    rows = pwr_rows(link, 1.5)
    print(f"   3 A rms: {got} after {dt if dt is None else round(dt, 3)} s (incl. the link), last rng {int(rows[-1]['rng'])}, "
          f"irms {rows[-1]['irms']:.4f}")
    ok &= got is not None and "OVER" in got and int(rows[-1]["rng"]) == 0 and abs(rows[-1]["irms"] - 3.0) < 2e-3
    return ok


def config(link):
    print("7. config and !DC")
    old = link.cfg["CYCLES"]["value"]
    link.set("CYCLES", 6)
    pwr_rows(link, 0.5)
    rows = pwr_rows(link, 2.0)
    link.set("CYCLES", old)
    print(f"   CYCLES 6: {len(rows) / 2.0:.1f} $PWR/s (want ~10)")
    ok = 8 <= len(rows) / 2.0 <= 12
    try:
        link.set("CYCLES", 999)
        print("   FAIL: CYCLES 999 accepted"); ok = False
    except LinkError as e:
        print(f"   CYCLES 999 refused: {e}")
    d = link.dc(300)
    print(f"   !DC: v {d['v']:+.3f} V, i {d['i']:+.4f} A, {d['n']} scans, rng {d['rng']}")
    ok &= d["n"] > 20000
    return ok


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else "192.168.40.225"
    link = MonitorLink(target)
    print(link.identify())
    link.read_cfg()
    st = link.get_status()
    print(f"fw {st['fw']}, scan {st['scanus4']} us, capmax {st['capmax']}, lincal {st['lincal']}")
    mode0 = int(link.cfg["RNGMODE"]["value"])
    link.relay(False)
    link.stream(True)
    res = {}
    try:
        for name, fn in (("accuracy", accuracy), ("streaming", streaming), ("captures", captures),
                         ("inrush", inrush), ("safe start", safe_start), ("ranges", ranges),
                         ("config", config)):
            try:
                res[name] = bool(fn(link))
            except LinkError as e:
                print(f"   FAIL: {e}")
                res[name] = False
    finally:
        link.send("!SYNTH,0")
        try:
            link.relay(False)
            link.range_mode(mode0)
            link.stream(False)
        except LinkError:
            pass
        link.close()
    print("  ".join(f"{k}: {'ok' if v else 'FAIL'}" for k, v in res.items()))
    ok = all(res.values())
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
