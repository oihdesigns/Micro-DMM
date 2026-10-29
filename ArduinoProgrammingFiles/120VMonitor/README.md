# 120V Monitor

Bench mains monitor: RA4M1 (XIAO; UNO R4 builds too) or Adafruit Feather
STM32F405, isolation-amp front end, SSD1306 OLED, load relay, Python GUI,
bench-supply calibration. The sketch picks the board automatically; the
Feather's differences are in [Feather STM32F405](#feather-stm32f405).

| File | |
|---|---|
| `120VMonitor.ino` + `monitor_types.h` | firmware |
| `monitor_gui.py` | GUI: `py -3 monitor_gui.py` (pyserial, matplotlib, numpy) |
| `monitor_link.py` | serial protocol + capture analysis (usable from scripts) |
| `calibrate.py` | voltage / current / phase calibration logic |
| `bench_psu.py` | the SPD4323X, through the Virtual Bench or direct SCPI |
| `monitor_sim.py` | simulated board + supply: type `SIM` as the port |

## Wiring

| Signal | Pin |
|---|---|
| Current, isolation amp OUTP / OUTN | A0 (+) / A1 (−) |
| Voltage, isolation amp OUTP / OUTN | A2 (+) / A3 (−) |
| OLED SSD1306 128x64 | D4 SDA / D5 SCL, 0x3C (`OLEDADDR`). `OLED` picks the screen (below); `OLEDROT 1` flips it upside down |
| Relay driver | **D6 by default**, `RLYPIN` key; `RLYINV 1` if active-low |
| IR receiver (TSOP-style, active-low out) | **D9** by default, `IRPIN` key, 0 = off. It needs a pin with an edge interrupt; **D10 (P109) has none on the XIAO RA4M1**. D7, D8 and D9 all work |
| Status NeoPixel | the XIAO's built-in RGB (P112, enabled by P500) |
| Current-range relay driver | **D7** (`RNGPIN 7`, `RNGINV 0`). Its NC contact shorts the 0.1 Ω shunt, so energized (pin HIGH) = LOW range |

The relay is driven open in `setup()` before anything else runs, and the GUI
opens it on disconnect.

## Feather STM32F405

Build with **Generic STM32F4 series → Adafruit Feather STM32F405**, USB support
**CDC (generic 'Serial' supersede U(S)ART)**, upload method **STM32CubeProgrammer
(DFU)**. Firmware 1.4 and later.

| Signal | Feather pin |
|---|---|
| Current, isolation amp OUTP / OUTN | **A4 (+) / A5 (−)** |
| Voltage, isolation amp OUTP / OUTN | A2 (+) / A3 (−) |
| OLED | SDA / SCL. Default `OLED 2`: the Adafruit OLED FeatherWing 128x32 at 0x3C |
| Load (output on/off) relay driver | **D6** (`RLYPIN 6`, default) |
| IR receiver | **D9** (`IRPIN 9`, default); any pin can take the edge interrupt |
| Status NeoPixel | the Feather's own (D8, PC0) |
| Current-range relay driver | **D5** (`RNGPIN 5`, default on the Feather; there is no D7) |

These are the Feather's factory defaults (`!DEFAULTS`, or a blank flash).

**Screen (`OLED` key).** 0 off, 1 SSD1306 128x64 (the XIAO's default), 2 Adafruit
OLED FeatherWing 128x32 (SSD1306, the Feather's default), 3 Adafruit FeatherWing
128x64 (SH1107). The sketch needs the **Adafruit SH110X** library for type 3.
The 128x32 layout puts V large on the left, with A and W stacked beside it and
PF and Wh on the bottom line. Prompts, trips and clipping replace the bottom line.
**Don't press the FeatherWing's A/B/C buttons.** They are wired to D9/D6/D5, which
are the IR, load-relay and range-relay pins, and pressing one shorts that line to
GND. The firmware does not read them.
Pins for `RLYPIN` / `RNGPIN` / `IRPIN` must be D1, D5, D6 or D9–D13.
A0 and A1 are left free because they are the two DAC outputs.

**Sampling.** ADC1 and ADC2 run in dual injected-simultaneous mode: each scan
converts V+ and V− at the same instant, then I+ and I− at the next (~1.9 µs
later, 0.04° at 60 Hz, which `PHASEUS` covers). Each pair's halves are
simultaneous, so common-mode noise cancels at any frequency, not just below a
few kHz as on the XIAO. The ADC is 12-bit (the XIAO's is 14), which averaging
makes up for.

**Capture buffer.** The F405's 64 KB core-coupled RAM: **32768 values** (16384
V+I pairs) against 7168 on the XIAO. The GUI reads the size from `$STATUS
capmax`.

**Config storage.** The chip has no EEPROM, so the config lives in the last
flash sector. Erasing it stalls the board for 1–2 s, including every trip
check, so **`!SAVE` is refused while the load relay is closed**. A new board
starts uncalibrated: run the calibration on it.

**Loopback self-test.** `!DACTEST,<hz>,<deg>[,<amp>]` puts a "voltage" sine on
A0 and a "current" sine lagging it by `<deg>` on A1, `<amp>` V peak about
mid-rail (default 1.0). With the isolation amps disconnected, jumper A0 → A2
and A1 → A4, and hold A3 and A5 at mid-rail with a divider (two equal
resistors, 3.3 V to GND). With `VGAIN` and `IGAIN` set to 1 and `VOFF` /
`IOFF` at 0 (`!SET` only, don't save), the board should read `amp/√2` on both
channels with PF = cos(deg). `!DACTEST,0` stops it. This checks
the sampling and power maths end to end before mains is involved.

## How it samples

The core's `analogRead()` reopens the ADC on every call and its scan mask only
grows, so after four pins each call converts all four. The sketch routes the
pins once with `analogRead()` and then drives the ADC14 registers itself: one
single scan converts AN000 (A1), AN001 (A2), AN002 (A3), AN009 (A0) back to
back in a few µs (`$STATUS scanus4`). Both pairs are centred on the same
instant, so V and I are effectively simultaneous.

- **Metering** runs continuously, even with no PC attached, and drives the
  OLED. A window is `CYCLES`+2 mains cycles cut into `SPC` buckets per cycle,
  and every scan in a bucket is averaged into one sample, so about 25 scans per
  sample at the defaults. It integrates between interpolated V zero crossings
  with DC removed. `PHASEUS` corrects the skew between the two front ends.
- **Capture** paces single scans off the Cortex-M4 cycle counter into a
  7168-sample int16 buffer: 3584 V+I pairs, or 7168 of one channel. Scanning
  one pair roughly doubles the top rate. If you ask for more than the ADC can
  do, the `LATE` flag is set and the real rate is reported. Occasional samples
  carry a few µs of jitter from the USB interrupt.

**Timing** (Capture tab, `Set by`). Pick any two of rate, samples and
duration, and the GUI works out the third. That third field is greyed out.
When the buffer can't hold the requested duration at the chosen rate, the
sample count is capped and the note beside the field says how long you
actually get.

Triggers: immediate · |I| ≥ level · V rising zero crossing · |V| ≥ level ·
two inrush triggers that close the relay themselves:

- **Close relay - rising zero** closes *n* µs after a V rising zero crossing
  (0 = at the crossing, 4167 µs = the 60 Hz peak). The relay's own
  mechanical delay adds to that. The trigger is the close command.
- **Close relay - level trigger** closes as soon as the pretrigger is full,
  then triggers when |I| reaches the Level (A). The close is marked on the
  plot if it is still in the buffer. On a timeout the relay stays closed,
  unless "Open relay when capture completes" is ticked.

If the relay is already closed when you start either inrush trigger, the GUI
opens it and waits 1 s first. Pretrigger is a percentage of the buffer.

**Irms drops below level** is for catching a device's own over-current
protection tripping. The board works out the RMS current of each mains cycle.
A cycle at or above the Level (A rms) arms the trigger, and the first cycle
below it fires. The trigger mark is the start of that low cycle, so the
dropout is at most one cycle after the mark. Use a high pretrigger (80–95 %)
to keep what led up to the trip, and set the level well below the running
current but above what is left after the trip.

**Timeout 0** waits until the trigger fires or you abort. **Abort** (next to
Capture) stops a capture that is still waiting. Any command from the PC does
the same, including the relay button.

**Readings while armed.** By default a capture does nothing but sample until
it ends, which gives the best timing. Tick this to keep `$PWR` readings coming
from the capture's own samples, one per `CYCLES` mains cycles, so the Meter
tab, its trends and the CSV log keep running during a long wait. Each
reading costs a little time per sample. If a capture comes back flagged LATE,
lower the rate or Average N. Power and PF need V + I. Each of these readings
also does what a normal metering window does: it updates the board's screen
and status light, adds to Wh, lets auto-range step back down to LOW, and is
checked against TRIPA. A TRIPA trip during a capture opens the relay, and the
board reports it as `$SAFE,TRIPA,<A rms>`.

The capture plot always puts 0 V and 0 A on its centre line. The readout
gives each channel's DC mean beside its rms, and the Meter tab shows both
V dc and I dc.

**Cycle heat map.** "Time series / Cycle heat map" (desktop) or "Time / Heat
map" (app) shows the same capture two ways, and you can switch at any time. The heat map folds every
mains cycle onto one, like a scope's persistence display: 0–360° of the V
cycle (I's if only I was captured), with a log-scale count of how often the
waveform passed through each spot. Normal cycles pile up into one bright
trace. A sag, a spike, an inrush or a dropout shows as a dim trace beside
it. Each cycle is stretched between its own rising zero crossings, so drift in
the mains frequency doesn't smear the trace. A crossing that a dropout hides is
filled in at the median period. More cycles give a richer map: 1 s at
20 kS/s is 60 cycles.

**Live persistence** (Meter tab: "Trends / Live persistence"). This view runs
captures back to back and folds each one into a running cycle heat map, so
the map keeps building while you watch. Each capture is V + I at 15 kS/s,
4 scans a sample, with readings while armed on, so the meter numbers and the
CSV log keep updating. It's as many cycles as the buffer holds: 14 on the
XIAO, 30 on the Feather. The map is not gap-free. Nothing is sampled while
each capture is being sent, and the label shows what fraction of the time
was covered (about 80-90 % on the simulator). A one-off event that falls in a
gap is missed, so use a trigger capture for that. Fade: Infinite keeps
everything; 30 s / 5 s / 1 s let old cycles die away, like a scope's variable
persistence. Clear map starts over. The vertical range grows at once
(with 25 % headroom) when the signal gets bigger, and shrinks back once the
signal has stayed under half the range for 3 s; anything left off the new
scale is dropped. Capture is locked out until you switch back to Trends.

**Overlay V + I** draws both maps on one plot (V green to light blue on the
left scale, I in the usual colours on the right) so you can see where in the
voltage cycle the current flows. The **Voltage** dropdown (separate panels
only) picks what the voltage panel shows:

- **Actual**: the measured voltage.
- **Diff (Perfect)**: the measured voltage minus a perfect sine with the same
  RMS, lined up to the fundamental's phase. Harmonics already on the line
  show up here (the simulator's 3 V 3rd harmonic, for example).
- **Diff (Nulled)**: the measured voltage minus a reference you set with
  **Take Null**. That button averages the last 4 captures' voltage at each
  point in the cycle. Take it with the load off: the line's own harmonics
  cancel, and turning the load on shows only what the load does. A peak that
  sags 0.5 V shows as a 0.5 V dent on a scale of a few volts, instead of
  being lost on a ±200 V scale. The reference isn't rescaled, so a sag across
  the whole wave shows in real volts too. Clear map keeps the null; press
  Take Null again to replace it.

All the maps keep building the whole time, so switching views keeps the
history. The nulled map starts when the null is taken.
The meter values, the board's screen, the CSV log and the trend history all
keep going in this view. Switch back to Trends to see the history, including
the time spent in persistence. Wh also counts the gaps between captures.
Live captures have no trigger to wait for, so the board holds their meter
readings until after the last sample and does the maths and sending then. Only
the TRIPA check runs during sampling, so the readings can't make the sampling
fall behind (LATE).

**Timing slips.** If the board falls behind its sample clock it restarts the
clock and flags the capture LATE. The samples around that point aren't evenly
spaced, so that cycle won't line up with the others. For a LATE capture, both
heat maps (capture and live) leave out every cycle whose length is off by more
than half a sample from the median, plus the partial cycles at each end, and
say how many were left out. Captures without LATE are never trimmed, so a real
disturbance at a zero crossing always shows. The
board's screen is redrawn between captures, about 4 times a second. During a
long armed wait it can't be redrawn, because writing to the screen would stall
sampling.

**Averaging** (`Average N scans/sample`, default 7). This is oversampling,
done on the board: every stored sample is the mean of N consecutive ADC
scans, spaced evenly across the sample period (back to back at rate 0). The
Rate field is still the stored sample rate, so the ADC actually runs at
rate × N. If that's more than it can do, `LATE` is set and the real rate is
reported. Random noise drops by about √N. Triggers and the over-current check
work on the averaged samples. `TRIPPK` and the instant trip still check every
single scan. `$CAP` reports N as its last field, and the CSV header records
`scans_per_sample`.

Under the timing fields, the GUI shows the shortest spike you will capture at
full height whatever its timing. That is (2N−1)·T/N, where T is the sample
period: a spike that long covers 2N−1 scans in a row, and so always covers
every scan of at least one sample. A spike that lasts one scan spacing (T/N)
always lands on at least one scan, but it may only reach 1/N of its height.
The calculation uses the board's measured scan time (`$STATUS scanus4/2`) once
you're connected.

**Safe start** (Capture tab). These checks run on the board, inside the
capture loop, so they react within one sample. Recording continues after the
relay opens, and the plot marks the open point and its reason.

| Option | Opens the relay when |
|---|---|
| Instant trip | any single sample has \|i\| above X A |
| Over-current | the RMS current over each mains cycle stays above Y A for N consecutive cycles, counted from the trigger (instantaneous AC current crosses zero twice a cycle, so a continuous ">5 A" can never hold) |
| Auto-open | X ms after the trigger, which is the relay close for an inrush capture. If the capture ends first, the board keeps timing (up to 60 s). Any manual relay command cancels it |
| Open when capture completes | right after the last sample, before the data is sent |

Instant trip and over-current latch "TRIPPED" until the relay is next
commanded closed. The board reports each opening as `$SAFE,<reason>,<amps>`.

## Current ranges

| Range | Shunt | Full scale | Calibration |
|---|---|---|---|
| HIGH | 0.01 Ω | ±25 A | `IGAIN` / `IOFF` |
| LOW | 0.1 Ω | ±2.5 A | `IGAINLO` / `IOFFLO` |

**Wiring.** Put the shunts in series and let the relay **short out the 0.1 Ω
one** for the HIGH range, so the load circuit is never broken while the
relay moves. A changeover contact that picks one shunt or the other opens the
load for a few ms on every switch. Use the relay's **normally-closed** contact
for that short, so an unpowered or resetting board sits in HIGH. Read each
shunt with its own Kelvin taps, and use a second pole to move the isolation
amp's input between them. Then the bypass contact's resistance never appears
in the reading.

**Before you switch it on:**
- When the LOW range overloads, the 0.1 Ω shunt carries the full fault
  current until the relay opens, which takes a few ms. 25 A is 62 W, so for a
  few ms that's fine for a power resistor.
- The isolation amp's input sees 10× its rated input in that time. Check its
  absolute maximum rating, or clamp the input.
- A fuse is still what clears a real short.

**As built:** the shunts are in series, and D7's relay shorts out the 0.1 Ω
one through its NC contact. The board is wired as recommended, so an unpowered
board sits in HIGH.

`RNGPIN` stays 0 until the relay is wired. With it at 0 the board runs HIGH
only, exactly as before. Set `RNGPIN` and `RNGINV` in the Config tab, then
calibrate the LOW range. When the stored config upgrades to v3, `IGAINLO`
starts as `IGAIN`/10.

| `RNGMODE` | Behaviour |
|---|---|
| 0 auto (default) | Overloads go HIGH. After `RNGHOLD` (3 s) of window peaks below `RNGDNA` (1.5 A), it drops back to LOW. Closing the load relay goes HIGH first, so inrush never hits the 0.1 Ω shunt |
| 1 | HIGH only |
| 2 | LOW. An overload still goes HIGH, and it stays HIGH until you select the range again |
| 3 auto-delayed | Starts LOW and rides through short overloads. It goes HIGH only when the overload lasts more than `RNGCYC` (2) mains cycles in a row, then drops back like auto. Closing the load relay stays LOW, so a short inrush is recorded clipped at about 2.5 A. **While LOW, `TRIPPK` and safe start can't see current above ~2.5 A**, so a fault takes up to `RNGCYC` + 1 cycles to show |

**Overload** is any single scan above `RNGUPA` (2.2 A), or a clipped reading,
while in LOW. It's checked in every hot loop, in the same place as `TRIPPK`,
so the relay is told within µs. The reading is meaningless while the relay
moves, so for `RNGSETMS` (20 ms) after a switch:
- the current trips ignore it;
- a metering window that spans a switch is thrown away;
- a capture keeps recording. It reports the sample where the range changed,
  and the GUI scales the samples after it with the HIGH calibration and shades
  the settling time.

**In the GUI:**
- **Top bar:** range mode (Auto / HIGH only / LOW / Auto-delayed) and the range in use.
- **Capture tab:** *Current range* is Follow mode, HIGH or LOW for that one
  capture. V-only captures always run HIGH, because there's no current
  reading to catch an overload.
- **Calibration tab:** pick the range before the current sweep. The sweep pins
  the board to that range, checks it didn't switch range part-way through, and
  restores the mode afterwards. Default LOW setpoints stop at 2 A.
- **Zero offsets:** zeroes both ranges.

## Remote and status light

NEC remote, decoded in an edge interrupt, so it costs nothing while idle and
adds no jitter to captures. The keys default to the remote you have now:
address `IRADDR` 0x00, OK `IROK` 0x1C, * `IRCONF` 0x16.

| Relay | Button | Result |
|---|---|---|
| open | OK | primes a close. The OLED shows "PRESS * TO CLOSE" for `IRARMMS` (5 s) |
| open, primed | * | closes the relay (and clears a latched trip, like the LOAD button) |
| open, primed | OK | cancels the prime |
| closed | OK | **opens at once**. The press is checked inside the metering and capture loops, so it acts within microseconds of the frame. During a capture the opening is recorded like a safe-start one |

Held buttons send NEC repeat frames, and those are ignored, so a held OK
can't prime and then cancel. Every frame decoded from any remote is reported
as `$IR,<addr>,<cmd>` in the Console tab. That's the quick way to learn a
different remote, then set `IRADDR` / `IROK` / `IRCONF` in the Config tab.

The built-in NeoPixel shows **red at `LIVEVAC` (30) Vrms or above** and
**green below** (2 V of hysteresis). `PIXEL` sets the brightness, 0 = off.

## Calibration (Calibration tab)

The bench supply's **CH2** is wired to the 120 V input, **+ to hot**. The
isolation amps are DC-coupled, so a DC sweep fixes gain, offset *and sign*:
positive means hot above neutral and current flowing hot→load, so real power
reads positive on mains.

1. **Connect supply.** *Auto* uses a running Virtual Bench (it holds the
   supply's only SCPI session) and falls back to direct SCPI at
   192.168.10.6:5025. Only the chosen channel is touched. It refuses to run in
   series/parallel tracking mode. Afterwards the output is switched off and the
   setpoints restored.
2. **Voltage sweep.** Mains disconnected. The relay opens and the supply runs
   in CV 0…30 V with a 50 mA limit. Truth is the supply's `MEAS:VOLT?`, read
   either side of a 500 ms `!DC`. If the input shows AC, the sweep is refused.
3. **Current sweep**, once per range. Mains disconnected, with a **short or
   low-ohm power resistor in the outlet**. The relay closes and the supply runs in CC,
   0.25…2.5 A under a 5 V limit. Truth is `MEAS:CURR?`. It warns if the supply
   is not actually in CC.
4. **Phase.** Mains on, supply removed, a purely resistive load. It measures
   the delay between the V and I fundamentals in a raw capture, and that delay
   becomes `PHASEUS`.
5. **Apply**, then **Save to EEPROM**. The residual plots show linearity.

## Protocol

See the header of `120VMonitor.ino`. In short: `!CMD,args` in; `$TYPE,fields`
out. Config uses the BlinkyHawk pattern (`!CFG` / `!SET,K,V` / `!SAVE`), with a
CRC over `offsetof(Config, crc)`. Captures come back as `$CAP` header,
`$CD,<hex int16 ×32>` lines, then `$CEND`. Range changes arrive as
`$RNG,<0 HIGH|1 LOW>,<BOOT|CMD|AUTO|OVER|CLOSE|CAP>,<mode>[,<amps>]`, and
`!RANGE,<0|1|2>` sets the mode.

## Config snapshots

Every key belongs to one group, so a snapshot can be copied a kind at a time:

| Group  | Keys | Copy to a new board? |
|--------|------|----------------------|
| CAL    | VGAIN VOFF IGAIN IOFF IGAINLO IOFFLO PHASEUS ADCVREF | Gains yes, if the isolation-amp front end moves with them; zero the offsets again and check PHASEUS |
| FUSE   | TRIPA TRIPPK | Yes |
| RANGE  | RNGMODE RNGUPA RNGDNA RNGHOLD RNGSETMS RNGCYC | Yes |
| METER  | MAINS SPC CYCLES | Yes |
| WIRING | RLYPIN RLYINV RNGPIN RNGINV IRPIN | Only between boards of the same type |
| UI     | OLED OLEDADDR OLEDROT IRADDR IROK IRCONF IRARMMS PIXEL LIVEVAC | Usually |

`!DUMP` prints the running values and `!DUMP,SAVED` the flash/EEPROM block:
`$DUMPHDR,fw=,board=,uid=,cfgver=,src=,dirty=`, then
`$DUMP,<group>,<key>,<value>` per key, then `$DUMPEND`. `uid` is the chip's
factory ID, so a file always names the unit it came from.

The GUI logs the saved block to `config_log/<board>_<uid>_<time>.csv` on every
connect and every save. It skips the write when nothing has changed since that
board's last file. The Config tab also has **Export snapshot…** and
**Apply snapshot…**. Apply lists the keys that differ, with one checkbox per
group; only CAL is ticked by default. It then sends `!SET` for the chosen keys,
reads them back, and saves if you ask it to. Firmware older than 1.4 has no
`!DUMP`; the GUI falls back to `!CFG` there, which gives the running values
without a uid.

To move a calibration from the XIAO to the Feather:

1. Connect the XIAO; its snapshot lands in `config_log/`.
2. Connect the Feather and choose **Apply snapshot…** with that file.
3. Keep CAL ticked, plus FUSE, RANGE and METER if you want them. Leave WIRING
   unticked.
4. Run **Zero offsets** with no mains and no load, then save.

## Android app

`android_app/` is a phone version of the GUI (Kotlin + Jetpack Compose, the
same build setup as the other Micro-DMM apps). Open the folder in Android
Studio and run it, or build `app-debug.apk` with `gradle assembleDebug`. Plug
the board into the phone with a USB-OTG adapter. The app offers to open when the
board is attached (Feather, XIAO or UNO R4), and **Connect** finds it otherwise.

| Tab | What it does |
|---|---|
| Meter | live readings, V/I and W/PF trends, Reset Wh, CSV log to a file you choose |
| Capture | every trigger, timing mode, average-N and safe-start option of the desktop GUI. Includes the same inrush confirmation and the 1 s open before a close-relay trigger. Plot is zero-centred: pinch to zoom, double-tap to reset. Time / Heat map toggle, harmonics, CSV export. The Meter tab also has Live persistence |
| Config | every key, tap to edit; Save / Reload / Defaults / Zero offsets; Export snapshot, Apply from log or from a file |
| Console | the raw protocol and a command line |

The top bar has the load relay, Stream, and the range mode. Disconnecting
opens the relay, as the desktop GUI does. Snapshots use the same CSV format as
`config_log/`, so files move both ways. The app logs its own to
`Android/data/com.microdmm.monitor120v/files/config_log/` on every connect
and save. To apply a PC snapshot on the phone, copy the file over and use
**Apply from file…**.

Calibration is desktop-only: it drives the bench supply over the LAN.

**Network…** connects over TCP instead of USB. `py -3 sim_server.py` serves
the simulator on port 8120, so the app can be tried in the Android emulator
with no board attached (connect to `10.0.2.2:8120`).

## Safety

`TRIPA` opens the relay above an Irms; it is checked once per window
(~200 ms). `TRIPPK` opens it on any single scan above |i|, in microseconds,
and stays active during captures. Both latch until the relay is commanded
closed. Neither one replaces a fuse. **Never have mains and the bench supply
on the input at the same time.**
