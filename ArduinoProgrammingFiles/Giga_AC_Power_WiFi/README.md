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
- **The trapezoid rule leaks into harmonics.** With fractional cycle ends, its
  end error grows with h². A pure sine read 0.1 % THD whenever a cycle was not a
  whole number of samples. It is now a piecewise-cubic rule, plus harmonics
  computed after the fundamental is removed.

Not yet tested: a real signal on the inputs (the Rigol into A0 and A2 would
check the dual-ADC path and PHASECAL), and anything at mains.

## Calibration (same order as before)

1. **Zero**: front end powered, no mains, no load → Calibrate → Zero.
2. **Voltage**: if it reads `V_shown` and a trusted meter reads `V_true`, set the
   "equals" box to `old × V_true / V_shown`.
3. **Current**: the same against a clamp meter.
4. **Phase**: on a purely resistive load, trim **PHASECAL** until PF = 1.000.
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
