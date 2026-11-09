# Giga AC Power Monitor (Wi-Fi)

`ac_power_monitor` moved to an **Arduino Giga R1 WiFi** and controlled over
Wi-Fi. It keeps the same jobs (Vrms, Irms, real power, PF, energy, a load
relay, waveform capture) and the same scaling and calibration. The
acquisition underneath is rebuilt around what the Giga can do.

| | |
|---|---|
| `Giga_AC_Power_WiFi.ino` | firmware (M7 only, default flash split) |
| `ac_types.h`, `net_sink.h`, `adc_access.h` | types, the non-blocking output path, the ADC/DMA repair access |
| `arduino_secrets.example.h` | copy to `arduino_secrets.h` (git-ignored) and fill in the Wi-Fi name/password |
| `ac_power_gui_wifi.py` | GUI: `pip install pyserial matplotlib numpy` |
| `selftest.py` | end-to-end test with no wiring: `py -3 selftest.py <ip or COMx>` |

## Quick start

1. **Fit the Wi-Fi antenna** (the U.FL lead that came with the Giga). Without it
   this board saw the access point at −80 to −84 dBm. Joins often failed, and
   once joined it managed 0.1 KB/s with 66 % ping loss. With it: −65 dBm,
   21 ms average ping, and every stream at once with nothing dropped.
2. `arduino_secrets.h` is already here (copied from Giga_RemoteController_WiFi).
3. Flash: `arduino:mbed_giga:giga`, no split option needed.
4. Run `py -3 ac_power_gui_wifi.py`. Type the board's IP (or a COM port) in
   **Board** and click Connect, then Start. The IP is printed in `$NET` over
   USB, and shown in the GUI's top bar once connected.

Nothing is connected to the inputs yet. To see real numbers anyway, open
Device → **Synth ON**: the board replaces the ADC data with an exact V/I pair
of your choosing, still paced by the ADC, so the whole chain is exercised.

## Wiring

| Signal | Pins | ADC |
|---|---|---|
| Mains voltage | A0 (+) / A1 (−) | ADC1 rank 0 / ADC2 rank 0 |
| Load current | A2 (+) / A3 (−) | ADC1 rank 1 / ADC2 rank 1 |
| Relay driver | D5 | opened at boot; `!RLYINV,1` if active-low |
| Current range select | D6 | LOW = range 0 (selected at boot), HIGH = range 1; `!IRINV,1` swaps them |

Inputs are 0–3.3 V. The − pins are the bias reference of each pair, as on the
RA4M1 build. `!SE,1,0` / `!SE,0,1` makes a channel single-ended (its − pin
ignored, mid-scale subtracted instead). Any of A0–A6 can be used. To move a
pair, edit the two `AdvancedADC` lines at the top: keep + on `adcA` and − on
`adcB`.

## What changed, and why

**V and I are sampled simultaneously.** The old boards had one mux, so V and I
alternated and the maths interpolated across a half-sample gap. Here ADC1 and
ADC2 run in dual regular-simultaneous mode (the `Giga_RemoteController_WiFi`
lesson). Each pair's + and − are converted at the same instant, so common-mode
ripple cancels instead of becoming noise. V and I are one conversion apart
(1.28 µs at the defaults, 0.03° at 60 Hz). That lag is known exactly and is
removed.

**Gap-free.** The ADC never stops: DMA → boxcar → an 8-second ring in SDRAM.
Every mains cycle is measured and the energy count has no holes. A lost DMA
buffer is detected, counted in `$INFO`, and flagged (+16). It is never
silently spliced.

**Oversample and average.** The ADC scans at 80 kHz, and each 10 scans are
averaged into one analysed sample: 8 kS/s, 133 per 60 Hz cycle. (80 kHz
because the library's timer only hits rates that divide 120 MHz exactly;
76.8 kHz would really run at 76.82 kHz.) The boxcar's small, known droop
(0.009 % at 60 Hz) is divided out.

**Less data on the wire.** The board reduces, the link carries results:

| Stream | What | Size / rate |
|---|---|---|
| `$PWR` | the window result (12 cycles) + THD + displacement PF | ~150 B, 5/s |
| `$CYC` | every cycle's Vrms, Irms, W, Hz, Ipk (steps, inrush) | ~60 B, 60/s |
| `$HARM` | harmonics 1..40, V and I, magnitude + phase | ~1.5 KB per window |
| `$AVG` | one cycle-synchronous averaged cycle of V and I, 128 points each | ~2 KB per window |
| `$WAVE` | raw capture from the ring, **only when something happens** | 16 B/sample |

`$AVG` is the "see the waveform without streaming it" mode. It is the average
of the window's cycles, aligned on the V crossing, so noise averages out and
the shape stays. Captures are sent on `!WAVE` or on an event: an I step, a V
sag/swell, every relay close (inrush), or a trip. The ring holds 8 s, so a
capture includes the moments *before* its trigger.

**Transmit while sampling, on one core.** The idea was "one core samples, the
other transmits". The DMA already samples while the CPU works, so the real
job was to make sure nothing blocks the loop that drains it:

- Output goes into a ring per connection (USB, TCP) and is sent with
  non-blocking writes. A slow link sheds the bulk streams first (`$CYC`,
  `$HARM`, `$AVG`). `$PWR` and command replies always get through, and a
  capture streams out in the background between live lines.
- The Wi-Fi join (`WiFi.begin()` blocks 5–20+ s) runs on its own RTOS thread.
  The board measures from power-up, whether or not the network is there.

Measured on the −84 dBm link (no antenna): 0 gaps, worst loop 2.8 ms, 1461
bulk lines shed, nothing else lost. With the antenna, all streams at once
(~19 KB/s) arrive complete, and a capture of 8000 samples takes 1–2 s. A second core would only be worth it if `$INFO` ever shows
gaps on a healthy network. The engine talks to the rest of the sketch only
through the ring and `emit*()`, to keep that move to the M4 easy.

## Verified (2026-11-08, `selftest.py`, synthetic source)

| Case | Result |
|---|---|
| 120 V / 5 A at 0°, 30°, 60°, −45° (60 Hz) | V, I, P within 4 ppm; PF and cos φ1 to the 4th decimal |
| 115 V / 10 A at 25°, 50 Hz; 59.7 Hz | the same |
| pure sine, I THD | 0.001 % |
| 30 % 3rd harmonic on I | THD 30.000 %; I1 and I3 amplitude and phase exact |
| all streams + an 8000-sample capture, 20 s over USB | `$PWR` 5.0/s, `$CYC` 60.0/s, 0 gaps, capture complete |
| all of the above **over Wi-Fi** (−65 dBm, antenna fitted) | PASS: every case the same; `$CYC` 58.8/s, `$HARM`/`$AVG` 4.9/s, 0 lines skipped, 0 gaps, loop max 2.8 ms |

Things found and fixed along the way, worth knowing for any Giga sketch:

- **`if (Serial)` runs `delay(1)`** on this core (`operator bool`). Checked
  per output line, it throttled the loop to 117 ms and overran the ADC. The
  sketch reads `_SerialUSB.connected()` once per loop instead.
- **`Serial.write()` blocks** until each USB packet is sent. The sketch uses
  `_SerialUSB.send_nb()`.
- **TCP sends block for up to 1.5 s by default** (`SOCKET_TIMEOUT`). The sketch
  calls `client.setSocketTimeout(0)` and keeps unsent bytes queued.
- **The radio's power-save is on by default**, and with it every round trip
  waits for a beacon (pings 22–406 ms). This core's lwIP also allows only
  1072 bytes (2 × 536) unacknowledged per TCP connection, fixed in the
  prebuilt library, so throughput is about 1 KB per round trip. The sketch
  calls `whd_wifi_disable_powersave()` after each join (`$NET` reports it):
  pings dropped to 21 ms average.
- **Bursts need room.** A 4 KB shed threshold dropped 15–25 % of `$HARM`/`$AVG`
  over Wi-Fi at 12 KB/s, on a link that moves a capture at 85 KB/s: one
  window's `$HARM` + `$AVG` is a 3.5 KB burst. The threshold is now 12 KB, and
  a capture keeps the queue under 3 KB so live lines get through beside it.
- **AdvancedADC never runs the H7's linearity calibration** (it asks for
  `ADC_CALIB_OFFSET` only). The sketch runs `ADC_CALIB_OFFSET_LINEARITY` on
  ADC1 and ADC2 before each start; `$ADC` ends with `lincal` = 1 when both
  took it. It brought the zero offsets from ~2.3 mV down to ~0.2–1.3 mV.
- **The trapezoid rule leaks into harmonics.** With fractional cycle ends, its
  end error grows with h². A pure sine read 0.1 % THD whenever a cycle was not a
  whole number of samples. It is now a piecewise-cubic rule, plus harmonics
  computed after the fundamental is removed.

### On the bench (2026-11-08, isolated front end, DC)

PSU CH2 into the front end's voltage input and through its current shunts
to the DL3021A load, with the load behind the D5 relay. Calibrated from DC
sweeps (0–30 V with no current, 0–1.2 A on each range). The values are in
`ac_power_profile_wifi.json`, which the GUI sends on connect:

| | Scale | Offset | Check |
|---|---|---|---|
| Voltage | 1.5 V = 154.116 V (102.74 V/V) | 1267 µV | 20 V reads −0.1 to −0.2 % |
| Current, range 0 (D6 LOW) | 1.5 V = 17.585 A (±19.3 A pk) | 215 µV | −0.3 to −0.8 %, one point +2.8 % at 0.6 A |
| Current, range 1 (D6 HIGH) | 1.5 V = 1.609 A (±1.77 A pk) | 282 µV | +0.45 / −0.24 / −0.17 % at 0.3 / 0.6 / 1.0 A |

- D5 is the load relay and D6 the range relay (HIGH = sensitive range,
  10.8× range 0), as the firmware expects.
- A range switch at a steady 0.5 A reads 0.496–0.531 A (1 ms means) across
  it, with no spike. Auto range: range 1 at 0.5 A, up to range 0 at
  1.7 A, back down to range 1 at 0.5 A.
- Range 0 is noisy per sample (±0.06 A at 8 kS/s) and drifted 2.4 % between
  two sweeps. Its path includes the range relay's bypass contact, whose
  resistance is comparable to the 10 mΩ shunt. If the amplifier's sense
  taps sit outside that contact, moving them across the shunt alone would
  fix both.
- PHASECAL is still unmeasured: it needs AC and a resistive load.

**ADC2 code error.** Both channels read up to ±1 mV (about 20 LSB) off a
straight line in a repeatable pattern. Near 0.5 A on range 1 the error was
+11 mV. Reading each pin single-ended traced it to the − pin (A3, on ADC2)
passing 1.2375 V, which is code 0x6000. It is identical at 16 and 14 bits,
the same at both sample times tried, and it survives the linearity
calibration, so it is the converter's own code-transition error. An AC
signal sweeps across thousands of codes, so it mostly averages out there;
a steady DC reading near such a code shows it. The fix would be a
per-ADC correction table, measured with the Giga's DAC swept into A1/A3.

Not yet tested: anything at mains, and PHASECAL.

## Current range (D6)

D6 switches the current front end between two ranges (a shunt, a gain, a
burden resistor: whatever your hardware does with it). Each range has its
own scale, offset and PHASECAL, set on the Calibrate tab or with
`!ISCALE,adcV,A,range`, `!IOFF,uV,range` and `!PHASECAL,us,range`.
Defaults: range 0 = 1.5 V → 25 A, range 1 = 1.5 V → 2.5 A.

- **Manual or auto** (Measure tab, or `!IRANGE,0|1|A`). Auto goes up at
  once on the first sample over 90 % of the sensitive range's full scale
  (or any clipping). It comes down after two windows under 60 %, at most
  once a second, so it doesn't hunt. `!IRLIM,up,down` changes the limits.
- **No glitch at the switch.** The DMA buffers in flight still hold scans
  from the old range when the pin changes. Each buffer carries the time it
  filled, so the board finds the scan where the switch happened and scales
  every scan by its own range. The ring holds amps, so cycles, windows and
  captures that span a switch stay in one unit.
- **Settling.** A relay doesn't switch at once. On the bench (2026-11-08,
  steady 0.5 A DC) the front end's range relay showed 3–5 ms of the old path
  on the new scale, then up to 8 ms of contact bounce that saturated the
  amplifier at about 60× the real current. Within `IRSETTLE` (default 20 ms)
  of a switch, each current sample is replaced by the one a mains cycle
  earlier and flagged +128, as is any cycle or window with a switch inside.
  That is exact for DC and close for steady AC, and the RMS, the energy,
  the trips and auto range never see the spike. Before this, auto range
  hunted: each switch's spike looked like an over-range.
- **Zero** zeroes V and then each current range in turn (~0.5 s).
- **Safety:** range 0 is selected at boot. If your switch changes a shunt,
  make range 0 the high-current one, so a reboot never leaves a big current
  on the sensitive shunt.
- `$PWR` ends with the range the window was measured on, and `$IRNG`
  reports both ranges' settings after every change or switch.

To check the switch timing on the bench, run a wire from D6 to A2 and run
`py -3 selftest.py <ip> 20 --jumper`. It counts any scan scaled with the
wrong range, before or after the edge.

## Calibration (same order as before)

1. **Zero**: front end powered, no mains, no load → Calibrate → Zero.
2. **Voltage**: if it reads `V_shown` and a trusted meter reads `V_true`, set the
   "equals" box to `old × V_true / V_shown`.
3. **Current**: the same against a clamp meter.
4. **Phase**: on a purely resistive load, trim **PHASECAL** until PF = 1.000,
   on each current range.
   PHASECAL is now in **µs**, nominal **0**: the extra delay of the current
   front end (a CT, a filter) relative to the voltage one. The ADC's own
   V→I lag is already handled. (On the RA4M1 build it was a sample
   fraction, nominal 0.5.)
5. **Save profile** (Device tab). The board doesn't store settings; the GUI
   sends the profile on connect.

## Protocol

See the header of `Giga_AC_Power_WiFi.ino` for every command and field. In
short: commands are `!NAME,args`, results are `$NAME,fields`, the same over
USB and TCP port 8080. One TCP client at a time; a new connection replaces the
old one, so a stale socket can't hold the Giga's tiny socket pool.

## Safety

The relay opens at boot, and the GUI opens it on disconnect. `!TRIP` checks
every cycle's Irms and `!TRIPPK` checks every sample. A trip latches,
reports, and captures the waveform. Neither replaces a fuse or a breaker.
