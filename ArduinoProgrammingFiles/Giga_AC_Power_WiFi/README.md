# Giga AC Power Monitor (Wi-Fi)

A bench mains power monitor on an **Arduino Giga R1 WiFi**, controlled over
Wi-Fi or USB. The firmware speaks the **120VMonitor protocol** (fw 1.4), so the
120VMonitor GUI, its bench-supply calibration and its Android app all drive
it. The GUI files here are copies of 120VMonitor's, adapted for the Giga.

| File | |
|---|---|
| `Giga_AC_Power_WiFi.ino` + `ac_types.h`, `net_sink.h`, `adc_access.h` | firmware (M7, default "2MB M7 + M4 in SDRAM" split) |
| `web_page.html` → `make_web_page.py` → `web_page.h` | the phone page the board serves; edit the .html, run `py -3 make_web_page.py`, compile |
| `arduino_secrets.example.h` | copy to `arduino_secrets.h` (git-ignored), fill in the Wi-Fi name and password (up to five networks, see below) |
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

## Web page (phone)

Open **`http://<ip>:8080/`** (e.g. `http://192.168.40.225:8080/`) in a browser
on the same network. No app or PC needed; the page is served by the board.

- **Readings**: V, I and W large; VA, var, PF, Hz, peaks, DC parts, Wh, THD and
  the current range below; tags for trip, clipping, lost samples, DC mode,
  synthetic data, a capture running, the GUI connected. In DC mode the cells
  swap as on the GUI (V dc / I dc large, the ripple below, Freq "DC").
- **Persistence map**: one mains cycle (0° = the rising voltage crossing), V
  green on the left scale, I amber on the right. The board folds **every**
  cycle (the 8 kS/s metering samples, so edges much shorter than 125 µs are
  smoothed) into 128 × 96 counts and sends a frame every 0.5 s; the page sums
  and fades them (1 s to no fade, chosen on the page). The scale follows the
  signal in 1-2-5 steps, so it jumps when the signal does and the map starts
  over. With no voltage crossings the spans are free-running and the phase
  axis means nothing; the page says so.
- **Load relay**: the one thing the page can change. Open is one tap. Close is
  two taps within 4 s, clears a latched trip, and, as the IR remote's close,
  goes HIGH range first in auto range and is refused while a capture owns the
  relay. Each switch is logged as `$WEB,RELAY,<0|1>,<OK|REFUSED>,<client ip>`.
- **No login yet.** Anyone on the network can open the page and switch the
  relay. Another web site can't do it through a visitor's browser: the switch
  is a POST that needs an `X-GACP` header, which a cross-site request can't
  add without a preflight the board refuses. `webAuthorized()` in the sketch
  is where a password check goes.

The page polls `GET /api/live[?since=<frame>]` once a second (JSON: the last
window's readings and any persistence frames newer than `since`). The relay
is `POST /api/relay?on=<0|1>` with `X-GACP: 1`. Timings on this bench: the
page loads in ~0.1 s (served gzipped), a poll takes ~60–130 ms, no samples
lost with the GUI and the page both connected.

**It shares port 8080 with the GUI.** The core's lwIP has 4 sockets and 4
netconns in all, a listener holds one of each, and lwIP takes a netconn for
each new connection before the sketch even sees it, resetting the connection
when none is left. With the page on its own port 80 only two connections
could exist at once, and a GUI connecting while two page requests ran was
reset every time. With one listener: the GUI plus two page connections at
once (a third is reset; the page retries next second). A new connection's
first byte decides what it is: `!` (the GUI and the Android app send `!ID`
straight away) makes it the GUI's connection, replacing the old one as
before; a letter makes it an HTTP request. A connection that sends nothing
within 1 s is closed and never taken for the GUI.

## Wiring

| Signal | Pin | Key |
|---|---|---|
| Voltage, isolation amp OUTP / OUTN | A0 (+) / A1 (−) | ADC1 / ADC2, sampled at one instant |
| Current, isolation amp OUTP / OUTN | A2 (+) / A3 (−) | ADC1 / ADC2, one conversion (1.28 µs) after V |
| Load relay driver | **D5** | `RLYPIN`, `RLYINV`. Driven open at boot |
| Current-range relay driver | **D6** | `RNGPIN`, `RNGINV` 0: pin HIGH = LOW range (as built: the NC contact shorts the 0.1 Ω shunt, so an unpowered board sits in HIGH) |
| OLED, SSD1306 128x64 (or 128x32) | **SDA D20, SCL D21**, 3.3 V, GND | `OLED` 1 = 128x64, 2 = 128x32, 0 off; `OLEDADDR` 0x3C; `OLEDROT` |
| IR receiver output (TSOP-style, active low) | **D9**, 3.3 V, GND | `IRPIN`, 0 = off. Not D4 or D8 (see below) |
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

**OLED and IR remote** work as on 120VMonitor (fw 2.1), with the same keys
in the Config tab's UI group (`OLED`, `OLEDADDR`, `OLEDROT`, `IRADDR`, `IROK`,
`IRCONF`, `IRARMMS`) and `IRPIN` under WIRING:

- The screen shows load on/off, the range, Hz, V, A, W, PF and Wh; the bottom
  line shows `PRESS * TO CLOSE`, `TRIPPED`, `CLIP` or `ARMED` when they apply,
  and the board's IP for 10 s after it joins Wi-Fi. The readings refresh
  once a second; a relay, remote or Wi-Fi change redraws at once. It is probed at boot:
  with nothing at `OLEDADDR` it stays off (`$STATUS oled=0`). Only the
  SSD1306 types; the SH1107 FeatherWing (120VMonitor's `OLED` 3) isn't
  supported here.
- The frame goes out 64 bytes per loop pass instead of Adafruit's one-shot
  `display()` (~25 ms at 400 kHz), so the screen never holds off the trips,
  safe start or the remote by more than ~1.6 ms.
- Remote: with the relay open, `IROK` (default 0x1C, OK) primes a close and
  `IRCONF` (0x16, *) within `IRARMMS` closes it; `IROK` again cancels. With
  the relay closed `IROK` opens it, within one loop pass (a few ms). Every
  frame is reported as `$IR,<addr>,<cmd>` in the Console, to learn a new
  remote's codes. A close is refused while a capture is running.
- `IRPIN` can be any header pin except **D4 and D8**: STM32 edge interrupts
  are per pin number across all ports, and the Wi-Fi chip's host-wake line
  is PI8, so a Px8 pin (D4 = PJ8, D8 = PB8) can't have one. `!SET` refuses
  them.

**Wi-Fi networks.** `arduino_secrets.h` can list up to five, tried in order:

```c
#define SECRET_SSID  "Lab"          // first choice
#define SECRET_PASS  "..."
#define SECRET_SSID2 "Home"         // optional fallbacks, up to SECRET_SSID5
#define SECRET_PASS2 "..."
```

Each join scans once, then tries the listed networks that are in range, in
that order; one out of range costs nothing, one that refuses the password
falls through to the next. If the link drops (or nothing was found) the whole
list is tried again every 20 s, and `!WIFI` starts over from the top. It
doesn't move back to a higher-priority network on its own while connected.
`$STATUS` reports `ssid=` and `net=` (1-5, 0 none); `!SCAN` marks each
network it sees with its place in the list. Hidden SSIDs aren't supported:
this core's `WiFi.begin()` only joins networks the scan saw.

**DC mode** (fw 2.2). A metering window with not one voltage zero crossing
(no AC above `VMIN` / 20 % of the peak) is reported as DC: `$PWR` gains a last
field `dc` = 1, `vdc` / `idc` are the readings and `vrms` / `irms` the AC
ripple on them, and W, VA, var, PF and Wh include the DC part (with `DCREM` on
they'd otherwise read ~0 on DC). `TRIPA` then acts on the total RMS current
and `TRIPPK` on |i|, so a DC overload trips too (checked: 2.09 A DC against
`TRIPA` 1.5 trips in one cycle). The GUI's meter shows V dc / I dc in the big
cells, the ripple where V dc / I dc normally are, Freq "DC" and a DC MODE
tag; trends plot the DC values and the CSV gets a `mode` column. The OLED
shows signed DC volts and amps with "DC" in place of Hz. A `$PWR` without the
field (older firmware) is treated as AC.

**Board RGB LED:** solid **blue** while a capture is waiting for its trigger;
otherwise solid **red** when the input's peak voltage (absolute, so DC
counts) is above 40 V and **green** once it is below 35 V. Between 35 and
40 V it keeps its last colour. It updates every mains cycle and is off
until the first cycle is measured. (The fw 2.0 colours - red tripped,
green client, blue no Wi-Fi - are gone; the OLED and `$STATUS` show those.)

**Not on the Giga:** NeoPixel (its keys don't exist), `!DACTEST`. `SPC`
doesn't apply: metering runs at a fixed 8 kS/s.

**Settings from fw 2.0** (config v1) carry over on the first boot of fw 2.1,
with the new keys at their defaults (`$STATUS cfgold=1` until the next
`!SAVE`).

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

- **Dead band above code 0x6000 (repaired in firmware).** Neither ADC ever
  outputs the codes just above 0x6000 (1.2375 V): ADC1 skips 0x6000-0x6137
  (~16 mV), ADC2 0x6000-~0x6230 (~28 mV), and every input in that band reads
  0x5FFF. The linearity calibration doesn't fix it, and the clocking is in
  spec (rev V, BOOST 11, 32 MHz at the converter), so it's the converters'
  own. The legs rest at ~1.48 V, so the band is 0.23 V below rest: on the
  HIGH range the I− leg (ADC2) is in it at about +4.7 to +5.5 A and I+ (ADC1)
  at about −5.0 to −5.8 A; the voltage legs at about ±48 V. Measured raw
  against a DM858 in series (2026-11-15): up to **+0.22 A** at +5.1 A and
  −0.06 A at −5.6 A, which is why calibrating at ±5.18 A gave different
  answers per polarity. DC parked there showed it; AC mostly averages it.
  - Repair (default on, `ADCRAW` 0): a leg that reads 0x5FFF is rebuilt from
    its partner, since an isolation amp's two legs are mirror images
    (p = a + b·n, b ≈ −1). It keeps rebuilding for 64 scans after the last
    hit, because the legs' noise (~6 mV rms) is mostly independent: rebuilding
    only the hit scans kept the ones whose own noise had missed the band, and
    read 110 mA off. The line is learned only from 64-scan block averages
    well clear of the band; b comes from a regression once the signal has
    spread over 10 mV (AC at once, DC after a few levels), else −1.
  - Result, same sweep: within **±20 mA** through both bands, following the
    board's normal ±25 mA error curve on either side.
  - `$STATUS` shows `mirv=` / `miri=` as `<scans rebuilt>:<b>[*]:<a mV>`,
    where `*` means b was learned. `ADCRAW,1` turns the repair off.
    `!LEGS` gives the four legs' raw codes and a code histogram, and
    `!ADCREG` the converters' clock and calibration registers.
  - Not yet checked on the voltage legs (needs ≥48 V or mains).
- **Small code errors elsewhere.** Both channels read up to ±1 mV (about 20
  LSB) off a straight line in a repeatable pattern, including a 64-code
  ripple. AC and noise average it out.
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
- **`micros()` is not the DMA timestamps' clock.** AdvancedADC stamps
  buffers with `us_ticker_read()`; `micros()` is a Timer started at boot, so
  the two differ by however long the bootloader ran (0.5 ms after a USB
  upload, more than the 20 ms settle window after a reset-button boot).
  Compared with each other, the range-settle window landed in the past and
  the relay's switching pulse went through: auto range hunted every ~3.5 s.
  Everything compared with a buffer time now uses `us_ticker_read()`;
  `$STATUS tickoff_us` shows the offset.
- **mbed `FlashIAP` erases with interrupts off** (~1 s stall). Calling the HAL
  erase from a thread instead hung the board.
- **4 sockets and 4 netconns, listeners included** (`MBED_CONF_LWIP_SOCKET_MAX`,
  `MEMP_NUM_NETCONN`, `MEMP_NUM_TCP_PCB`, all 4 and fixed in `libmbed.a`).
  lwIP resets a new connection when no netconn is free, and
  `WiFiServer::available()` resets the waiting connection when no socket is
  free: accept only when one is.
- **A short write followed by more data costs ~200 ms**: lwIP's Nagle holds
  the second segment until the first is acknowledged, and Windows delays
  that acknowledgement. Send a reply's header and body start in one write.
- **`WiFiClient::stop()` joins the client's reader thread** (up to 100 ms), and
  that thread polls every 100 ms, so a request can sit up to 100 ms before
  `available()` sees it. The sketch stops web connections on a reaper thread.
- **Closing right after the last `write()` loses nothing**: lwIP still
  delivers what it has queued (40 replies of 36 KB, all whole).

## Protocol

The header of `Giga_AC_Power_WiFi.ino` lists every command and line;
120VMonitor's README explains each one in more depth. One GUI client at a
time on port 8080; a new connection that starts with `!` replaces the old
one (the web page shares the port, see above). As on 120VMonitor, any
command while a capture waits for its trigger aborts it. `$STATUS web=` is
`<connections>:<page requests>:<relay switches>:<timeouts>:<taken as the GUI>`.

## Safety

The relay opens at boot and the GUI opens it on disconnect. `TRIPA` checks
every mains cycle's Irms, `TRIPPK` every scan; both latch until the relay is
commanded closed. In auto range a relay close goes HIGH first and waits for
the range relay. None of this replaces a fuse. Never have mains and the
bench supply on the input at the same time.
