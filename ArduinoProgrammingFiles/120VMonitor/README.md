# 120V Monitor

Bench mains monitor: RA4M1 (XIAO; UNO R4 builds too), isolation-amp front end,
SSD1306 OLED, load relay, Python GUI, bench-supply calibration.

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
| OLED SSD1306 128x64 | D4 SDA / D5 SCL, 0x3C (`OLEDADDR`) |
| Relay driver | **D6 by default**, `RLYPIN` key; `RLYINV 1` if active-low |
| IR receiver (TSOP-style, active-low out) | **D9** by default, `IRPIN` key, 0 = off. It needs a pin with an edge interrupt; **D10 (P109) has none on the XIAO RA4M1**. D7, D8 and D9 all work |
| Status NeoPixel | the XIAO's built-in RGB (P112, enabled by P500) |

The relay is driven open in `setup()` before anything else runs, and the GUI
opens it on disconnect.

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

Triggers: immediate · |I| ≥ level · V rising zero crossing · **close the
relay** (inrush, optionally *n* µs after a V zero crossing, e.g. 4167 µs for
the 60 Hz peak, though the relay's own mechanical delay adds to that) ·
|V| ≥ level. Pretrigger is a percentage of the buffer.

**Averaging** (`Average N scans/sample`, default 7). This is oversampling,
done on the board: every stored sample is the mean of N consecutive ADC
scans, spaced evenly across the sample period (back to back at rate 0). The
Rate field is still the stored sample rate, so the ADC actually runs at
rate × N. If that's more than it can do, `LATE` is set and the real rate is
reported. Random noise drops by about √N. Triggers and the over-current check
work on the averaged samples. `TRIPPK` and the instant trip still check every
single scan. `$CAP` reports N as its last field, and the CSV header records
`scans_per_sample`.

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
3. **Current sweep.** Mains disconnected, with a **short or low-ohm power
   resistor in the outlet**. The relay closes and the supply runs in CC,
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
`$CD,<hex int16 ×32>` lines, then `$CEND`.

## Safety

`TRIPA` opens the relay above an Irms; it is checked once per window
(~200 ms). `TRIPPK` opens it on any single scan above |i|, in microseconds,
and stays active during captures. Both latch until the relay is commanded
closed. Neither one replaces a fuse. **Never have mains and the bench supply
on the input at the same time.**
