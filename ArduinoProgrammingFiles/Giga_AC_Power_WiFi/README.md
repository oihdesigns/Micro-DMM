# Giga AC Power Monitor (Wi-Fi)

A bench mains power monitor on an **Arduino Giga R1 WiFi**, controlled over
Wi-Fi or USB. The firmware speaks the **120VMonitor protocol** (fw 1.4), so the
120VMonitor GUI, its bench-supply calibration and its Android app all drive
it. The GUI files here are copies of 120VMonitor's, adapted for the Giga.

| File | |
|---|---|
| `Giga_AC_Power_WiFi.ino` + `ac_types.h`, `net_sink.h`, `adc_access.h` | firmware (M7, default "2MB M7 + M4 in SDRAM" split) |
| `arduino_secrets.example.h` | copy to `arduino_secrets.h` (git-ignored), fill in the Wi-Fi name and password |
| `monitor_gui.py` | the GUI: `py -3 monitor_gui.py` (pyserial, matplotlib, numpy) |
| `monitor_link.py` | protocol + capture analysis, usable from scripts; serial, TCP or SIM |
| `calibrate.py`, `bench_psu.py` | bench-supply calibration (SPD4323X through Virtual Bench or SCPI) |
| `monitor_sim.py` | the 120VMonitor simulator: type `SIM` as the port |
| `selftest.py` | end-to-end test, no wiring: `py -3 selftest.py <ip or COMx>` |

## Quick start

1. Fit the Wi-Fi antenna (U.FL). Without it the link was −84 dBm and barely usable.
2. Flash `arduino:mbed_giga:giga`. The IP is in `$NET` (USB) and `$STATUS`.
3. `py -3 monitor_gui.py`, type the IP (default `192.168.40.225`) or a COM
   port in the port box, Connect. Every tab works as on 120VMonitor; see
   [its README](../120VMonitor/README.md) for the Capture, Meter (live
   persistence), Calibration and Config tabs.

`!SYNTH,1,<vrms>,<irms>,<deg>,<h3%>,<hz>` (Console tab) replaces the ADC data
with an exact synthetic V/I, still paced by the ADC, to try everything with
nothing connected. `!SYNTH,0` goes back.

## Wiring

| Signal | Pin | Key |
|---|---|---|
| Voltage, isolation amp OUTP / OUTN | A0 (+) / A1 (−) | ADC1 / ADC2, sampled at one instant |
| Current, isolation amp OUTP / OUTN | A2 (+) / A3 (−) | ADC1 / ADC2, one conversion (1.28 µs) after V |
| Load relay driver | **D5** | `RLYPIN`, `RLYINV`. Driven open at boot |
| Current-range relay driver | **D6** | `RNGPIN`, `RNGINV` 0: pin HIGH = LOW range (as built: the NC contact shorts the 0.1 Ω shunt, so an unpowered board sits in HIGH) |
| Wi-Fi antenna | U.FL | |

Note the channel order differs from the XIAO build (there A0/A1 is current).

## How it differs from 120VMonitor

**Sampling.** ADC1 and ADC2 run dual regular-simultaneous at 80 kHz, timer
paced, DMA'd, never stopping. Every scan goes into a 6.5 s ring in SDRAM, and
every 10 scans are averaged into an 8 kS/s ring for metering. So:

- **Metering is gap-free**: every mains cycle is measured (no window is
  thrown away), and it never pauses for a capture. "Readings while armed"
  is always on.
- **A capture's pretrigger is always full**, and reaches back before the
  `!CAP` arrived. Any rate up to 80 kS/s: each stored sample is the mean of
  N scans spread over its period (the GUI's Average N), from scans 12.5 µs
  apart (`$STATUS scanus4`). If a period holds fewer than N scans it uses
  all of them, and `$CAP` reports the real N. Up to 262144 values
  (`capmax`), and the whole capture must fit in the ring (about 5.5 s,
  `capspan_ms`); a longer one is cut and flagged +16.
- **V and I in a capture are simultaneous.** The current scans are moved back
  by the ADC's V→I conversion lag before averaging, so `PHASEUS`
  (from the Calibration tab's phase step) is only the front ends' own
  difference.
- **Triggers and safe start** run on the live scan stream. The relay reacts
  within one DMA buffer (~2 ms) plus however long it takes the current to
  cross the limit, not within one sample as on the XIAO.
- **Inrush at a phase** (Close relay - rising zero) is scheduled on a hardware
  timer from the measured crossing and frequency: the close lands within
  ~15 µs (0.3°) of the requested phase (selftest, synthetic mains).

**Current ranges** follow 120VMonitor's `RNGMODE` (auto / HIGH only / LOW /
auto-delayed), `RNGUPA`, `RNGDNA`, `RNGHOLD`, `RNGCYC`, `RNGSETMS`. On top:

- Each scan is scaled by the range it was taken on, placed from the DMA
  buffers' timestamps, so a switch never mixes scales.
- For `RNGSETMS` (20 ms) after a switch, each current scan is replaced by the
  one a mains cycle earlier and flagged +128. Measured on this bench, the
  range relay shows 3–5 ms of the old path on the new scale, then up to 8 ms
  of bounce at ~60× the real current; without the replacement auto range
  hunted, each switch's spike looking like an overload.
- Defaults here: `RNGUPA` 1.6 A and `RNGDNA` 1.0 A, because LOW reads ±1.77 A
  peak at the ADC's ±VREF/2 on this front end.

**Settings** live in the last 128 KB sector of internal flash. `!SAVE`
stops sampling for about 1 s: mbed's flash driver erases with interrupts off
(reported as a gap in `$STATUS`), so `!SAVE` is refused while the load relay
is closed, as on the F405 build. A board that has never saved shows `$ERR,no
saved config` once at connect (the GUI asks for the saved block); Save to
flash makes it go away.

**Not on the Giga:** OLED, IR remote, NeoPixel (their keys don't exist),
`!DACTEST`. `SPC` doesn't apply: metering runs at a fixed 8 kS/s.

**Giga extras** (Console tab): `!SYNTH`, `!SMASK` (`$CYC` per cycle, `$HARM`
harmonics 1..40 with phases, `$AVG` the cycle-averaged waveform, `$INFO`
diagnostics), `!HGET`, `!AVGW`, `!STAT`, `!WIFI`, `!SCAN`. Config keys for the
ADC (`RATE`, `DECIM`, `BITS`, `SMP`, `VSE`, `ISE`) and the metering (`HARMN`,
`VMIN`, `DCREM`) are in the Config tab under ADC and METER.

**GUI changes** from 120VMonitor's copy: the port box takes an IP address
(the link opens TCP 8080 for one), the range label shows each range's full
scale from `$STATUS`, LOW calibration points stop at 1.25 A, "EEPROM" reads
"flash", and capture / save timeouts allow for Wi-Fi.

## Calibration on this bench

As 120VMonitor: PSU CH2 into the front end, `calibrate.py` (Calibration tab)
sweeps the supply. The current sweep needs something in the load position
that sinks whatever the supply pushes:

- The DL3021A in **CV** snaps on with a ~1.8 A, ~1 ms pulse when the supply
  output switches on. That is a real overload on LOW, so the board goes HIGH
  and the sweep stops ("left the LOW range"). Use **CR 1 Ω with Von lowered**
  (Von 3.1 V otherwise keeps it off at low voltage), or a real short.
- At 0.1–0.25 A the DL3021A in CR still doesn't hold the supply in CC; the
  sweep warns about it. Those points are worse truth than the rest.

Results through the GUI's own code (2026-11-08, Wi-Fi): voltage gain 102.648
(the stored 102.744 is from a hand sweep, 0.09 % apart, residuals ≤ 0.09 V
over 0–30 V); HIGH 11.895 (stored 11.723, 1.5 % apart: the range relay's
bypass contact drifts, see below); LOW 1.0668 (stored 1.0727, 0.55 %). The
stored defaults are the hand sweep's, made against the load's own current
reading.

## Verified

`selftest.py` over Wi-Fi, synthetic source (2026-11-08): PASS.

| | |
|---|---|
| Accuracy, 7 cases (0°/30°/60°/−45°, 30 % h3, 50 Hz, 59.7 Hz) | V, I, P within 4 ppm, PF to the 4th decimal, THD exact |
| Streaming 15 s with a 20 kS/s capture in the middle | 5.00 $PWR/s, no window missed, 0 gaps |
| Triggers | V rising zero lands on the crossing; \|I\| ≥ 50 A on 7 A times out; \|I\| ≥ 6.5 A fires at 6.6 A |
| Inrush at 0°, 45°, 90° | closes within 4–16 µs of the phase; "open when complete" opens it |
| Safe start, instant trip 6 A | `$SAFE,INST,6.0`, open marked 6.5 ms after the close |
| Auto range | 0.5 A → LOW after RNGHOLD; 3 A → `$RNG,0,OVER` within 0.06 s at the PC |
| Config, !DC | `!SET` round trip, out-of-range refused, $DC 24000 scans |

The GUI itself was run against the board over Wi-Fi: connect, Meter,
Capture (V 119.999, I 3.9999, THD 8.00 %, PF 0.8165 on a synthetic
120 V / 4 A / 35° / 8 % h3) and live persistence.

On the real front end (DC): voltage within −0.1 to −0.2 %, LOW within
±0.5 %, HIGH within ±0.8 % except a +2.8 % point; range switching at a
steady 0.5 A reads 0.496–0.531 A across the switch with no spike.

**Not yet tested:** anything at mains, so `PHASEUS` is still 0.

## Known limits

- **ADC2 code error.** Both channels read up to ±1 mV (about 20 LSB) off a
  straight line in a repeatable pattern, and the − pin (A3, ADC2) passing
  1.2375 V (code 0x6000) adds up to +11 mV. Same at 16 and 14 bits and both
  sample times, and the linearity calibration doesn't remove it: the
  converter's own. AC sweeps across thousands of codes and averages it out;
  a steady DC reading near that code shows it. A fix would be a per-ADC
  correction table, measured with the Giga's DAC swept into A1/A3.
- **Range HIGH drifts.** Its path includes the range relay's bypass contact,
  comparable to the 10 mΩ shunt (2.4 % and 1.5 % moves between sweeps,
  noisy per sample). Kelvin taps across the shunt alone would fix it.
- **Wi-Fi throughput** is ~20–80 KB/s (lwIP allows 2 × 536 B in flight). A
  capture is ~4.2 bytes a value as hex: 6000 values take ~1 s, the full
  262144 most of a minute. USB is much faster.

## Things found on this core (any Giga sketch)

- **`if (Serial)` runs `delay(1)`** (`operator bool`). Per output line it
  throttled the loop to 117 ms and overran the ADC. Read
  `_SerialUSB.connected()` once per loop.
- **`Serial.write()` blocks** per USB packet; `_SerialUSB.send_nb()` doesn't.
- **TCP sends block up to 1.5 s** (`SOCKET_TIMEOUT`); `client.setSocketTimeout(0)`.
- **`WiFi.begin()` blocks 5–20 s**: join on its own RTOS thread.
- **The radio's power-save is on** (pings 22–406 ms): `whd_wifi_disable_powersave()`
  after the join brought them to ~20 ms.
- **AdvancedADC never runs the H7's linearity calibration** (offset only). The
  sketch runs `ADC_CALIB_OFFSET_LINEARITY` before each start (`lincal` in
  `$STATUS`); zero offsets went from ~2.3 mV to 0.2–1.3 mV.
- **The trapezoid rule leaks into harmonics** (0.1 % fake THD with fractional
  cycle ends): piecewise-cubic weights, harmonics after removing the
  fundamental.
- **mbed `FlashIAP` erases with interrupts off** (~1 s stall). Calling the HAL
  erase from a thread instead hung the board.

## Protocol

The header of `Giga_AC_Power_WiFi.ino` lists every command and line;
120VMonitor's README explains each one in more depth. One TCP client at a
time on port 8080; a new connection replaces the old one. As on
120VMonitor, any command while a capture waits for its trigger aborts it.

## Safety

The relay opens at boot and the GUI opens it on disconnect. `TRIPA` checks
every mains cycle's Irms, `TRIPPK` every scan; both latch until the relay is
commanded closed. In auto range a relay close goes HIGH first and waits for
the range relay. None of this replaces a fuse. Never have mains and the
bench supply on the input at the same time.
