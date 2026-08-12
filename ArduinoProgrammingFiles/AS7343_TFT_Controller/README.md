# AS7343_TFT_Controller

Bench controller for an **Adafruit AS7343** 14-channel spectral sensor on an
**Adafruit Feather RP2040** carrying a **2.4" TFT FeatherWing v2 (#3315)**.

Two pieces:

| File | Role |
|---|---|
| `AS7343_TFT_Controller/AS7343_TFT_Controller.ino` | Firmware — sensor control, touchscreen UI, serial protocol, burst capture |
| `as7343_gui.py` | Host GUI — every lever, live spectrum, timed captures, parameter sweeps |

The touchscreen carries a deliberately reduced control set (gain / ATIME / ASTEP /
SMUX / LED / capture window + a live spectrum). The Python GUI is the full
control surface.

---

## Wiring

Everything is either FeatherWing-hardwired or I²C — nothing to solder except the
AS7343's four STEMMA QT wires.

| Signal | Pin | Note |
|---|---|---|
| TFT CS | 9 | FeatherWing hardwired |
| TFT DC | 10 | FeatherWing hardwired |
| Touch IRQ | 6 | TSC2007, FeatherWing hardwired |
| SD CS | 5 | FeatherWing slot, unused by this sketch |
| NeoPixel | 16 | Feather RP2040 onboard |
| I²C | SDA=GPIO2, SCL=GPIO3 | AS7343 @ `0x39`, TSC2007 @ `0x48` |

**No I²C address conflict here.** `DataLogger_TFT` on this same FeatherWing needs
the TSC2007 moved off `0x48` because its ADS1015s collide; the AS7343 sits at
`0x39`, so the touch controller stays at its default and the FeatherWing needs no
address jumpers.

## Libraries

`Adafruit AS7343` (v1.1.0), `Adafruit ILI9341`, `Adafruit TSC2007`,
`Adafruit GFX`, `Adafruit NeoPixel`, plus `EEPROM` from the rp2040 core.

Board: **Adafruit Feather RP2040** — FQBN `rp2040:rp2040:adafruit_feather`
(Earle Philhower core). Compiles to ~99 KB flash / 71 KB RAM; the 60 KB of that
is the capture buffer.

Host side: `pip install pyserial matplotlib numpy`.

---

## Channel map

The AS7343's auto-SMUX reads out in a fixed order, three cycles of six:

| idx | ch | nm | idx | ch | nm | idx | ch | nm |
|---|---|---|---|---|---|---|---|---|
| 0 | FZ | 450 | 6 | F2 | 425 | 12 | F1 | 405 |
| 1 | FY | 555 | 7 | F3 | 475 | 13 | F7 | 690 |
| 2 | FXL | 600 | 8 | F4 | 515 | 14 | F8 | 745 |
| 3 | NIR | 855 | 9 | F6 | 640 | 15 | F5 | 550 |
| 4 | VTL0 | clear | 10 | VTL1 | clear | 16 | VTL2 | clear |
| 5 | VBR0 | clear | 11 | VBR1 | clear | 17 | VBR2 | clear |

### About "turning channels on and off"

Worth being explicit, because it constrains what the GUI can honestly offer:
the AS7343's auto-SMUX presents **fixed** channel sets, and the Adafruit library
does not expose the manual SMUX programming that would let you pick arbitrary
photodiode-to-ADC routings. So channel selection is split into two controls that
do genuinely different things:

* **`smux` (6 / 12 / 18)** — a real *hardware* lever. It sets how many SMUX
  cycles run, so it changes the measurement time directly: 6-channel mode reads
  indices 0–5 in one cycle, 12-channel reads 0–11 in two, 18-channel reads all
  in three. **This is the main lever on throughput.**
* **`chmask` (18-bit)** — a *software* mask over which of the measured channels
  get stored, streamed and plotted. It does not change sensor timing, but it does
  cut serial traffic and multiplies the number of samples that fit in the capture
  buffer (which is sized in words, not samples).

If you need arbitrary per-channel routing, that requires manual SMUX register
programming and would have to be added to the firmware directly.

---

## Serial protocol

Commands start with `!`, replies with `$`, and human-readable debug with `#`.
115200 baud (ignored — RP2040 native USB CDC).

### Host → device

| Command | Effect |
|---|---|
| `!PING` | → `$HELLO,AS7343_TFT,<fw>` |
| `!ID` | → `$ID,<part>,<rev>,<aux>` |
| `!CFG` | Dump full config, one key per line, then `$CFGEND` |
| `!GET,<key>` | → `$VAL,<key>,<val>` |
| `!SET,<key>,<val>` | → `$OK,<key>,<val>` or `$ERR,<key>,range\|nokey` |
| `!READ` | One measurement → `$R,…` |
| `!STREAM,0\|1` | Continuous `$R` lines every `streamms` |
| `!CAP[,<ms>]` | Timed burst capture (below) |
| `!LED,0\|1` | Shorthand for `!SET,led` |
| `!FLICKER` | One-shot flicker detect → `$FLK,<status>,<freq_hz>` |
| `!SAVE` / `!LOAD` / `!DEFAULTS` | Flash-backed EEPROM config |

### Config keys

`gain` (0–12, = 0.5× … 2048×), `atime` (0–255), `astep` (0–65534),
`wtime` (0–255), `waiten`, `smux` (6/12/18), `led`, `ledma` (4–258),
`az` (auto-zero frequency), `chmask` (18-bit), `streamms`, `capms`,
`capmode` (0/1), `thlow`, `thhigh`, `thch`, `pers`, `spint`.

`!CFG` reports each as `$CFG,<key>,<val>,<min>,<max>` — the GUI reads the
ranges from the device rather than hardcoding them — plus two read-only
pseudo-keys `tint_ms` and `nchan`, then a `$CH,<name>,…` header naming the
currently enabled channels in transmit order.

### Capture reply

```
$CAPB,<n>,<nchan>,<mask_hex>,<elapsed_us>,<rate_hz>,<mode>,<gain>,<atime>,<astep>,<smux>,<tint_ms>
$CH,<name0>,<name1>,…
$S,<t_us>,<v0>,<v1>,…          ← one line per sample, t_us relative to start
$CAPE,<n>
```

Buffer limits: 24000 sample-words and 3000 samples, whichever binds first. With
all 18 channels enabled that's 1333 samples; with six it's 3000.

---

## Speed testing

Two capture modes, selectable as `capmode`, because they measure different things:

* **`0` restart-per-sample** — stop, clear status, start, wait for `AVALID`, read.
  Every sample is a fully independent integration with no carry-over. Costs
  restart dead-time between samples. This is the honest "what rate can I get
  clean, independent samples at" number.
* **`1` free-run** — `SP_EN` stays asserted so the sensor integrates back to
  back. A floor of 90 % of the per-cycle integration time keeps the loop from
  re-reading the same conversion, then `AVALID` gates the actual read.

Both timestamp with `micros()` at the moment `AVALID` was observed, and the
reported rate is `n / elapsed`, measured — not computed from the register
settings.

**On integration time:** the sketch reports `tint_ms = (ATIME+1)(ASTEP+1)×2.78 µs`,
which is the **per-cycle** figure the Adafruit library computes. A full
18-channel measurement runs three SMUX cycles, so the wall-clock period per
sample is longer than `tint_ms` — how much longer is exactly what the capture
measures. Trust the measured rate, not the formula. This is also why the sweep
feature exists: point it at `smux` and you can see the cycle-count cost directly.

### GUI tabs

* **Live** — spectrum bar chart (12 spectral channels, wavelength-ordered and
  wavelength-coloured), linear or log, single read or continuous stream, plus a
  one-shot flicker detect.
  **Lock Y** freezes the y-axis so bars stay comparable between readings instead
  of the axis rescaling under you every sample. Ticking it with the box empty
  pins the axis where it currently sits; type a number to set an explicit top, or
  hit **Full scale** to lock to the ADC's actual full scale. Untick to go back to
  autoscaling. Under log scale the locked axis starts at 1 rather than 0.

  Full scale is **not** a fixed 65535 — the AS7343's ADC full scale is
  `(ATIME+1) x (ASTEP+1)`, capped at the 16-bit register limit. The stock
  ATIME=29 / ASTEP=599 tops out at **18000**, and a fast capture config like
  ATIME=0 / ASTEP=99 at just **100**, so pinning the axis to 65535 would squash
  the data into the bottom of the plot. The button label shows the current value,
  and once locked to full scale the axis *tracks* it — change ATIME or ASTEP, or
  run a sweep that changes them, and the axis follows. Typing your own number
  stops the tracking.
  **Timed CSV log** takes one reading every *n* seconds (default 10) for *x*
  seconds or minutes (default 600 s), appending each to a single CSV. Pick a file
  with **File...** or let it auto-name one; re-running against the same file
  appends without repeating the header, and a column-set mismatch prompts before
  it makes the file ragged. The status readout shows rows written and time
  remaining, and it stops on its own at the end of the window, on **Stop**, or if
  the port drops.

  Rows are written when the `$R` reply lands, not on the timer tick, so every row
  is a real measurement rather than a stale value carried over while the board was
  busy. If a reply has not returned by the next tick — a long integration time
  relative to the interval — that tick is counted as `late` in the status rather
  than duplicating the previous row. Each row is `timestamp, elapsed_s, asat,
  dsat, gain, atime, astep, smux` followed by one column per enabled channel, so
  the acquisition settings travel with the data.

* **Controls** — gain, ATIME, ASTEP with live integration-time readout, SMUX,
  WTIME + wait enable, auto-zero, LED enable/current, spectral threshold
  (low/high/channel/persistence/interrupt), the 18-channel mask with
  All / None / Spectral-only / Clear-only presets, and EEPROM save/load/defaults.
  A tree view mirrors the device's own reported config with its min/max limits.
* **Speed Test** — single capture with window and mode, showing achieved rate
  plus inter-sample interval mean/min/max/σ, and a per-channel time-series plot
  with a channel picker. Save to CSV. Below it, a **sweep**: walk `astep`,
  `atime`, `gain` or `smux` across a list of values, run a capture at each, and
  get a table plus a rate-vs-parameter plot. Also saveable as CSV.

The board dumps captures unsolicited when you hit CAPTURE on the touchscreen
too, so a capture started on the device shows up in the GUI plot if it happens
to be connected.

---

## Touchscreen

Two pages via the bottom tab bar.

**SPECTRUM** — live 12-channel bar chart, colour-coded by wavelength, autoscaled
with the peak value shown. Channels not covered by the current SMUX mode, or
masked out, are drawn as empty outlines. Buttons: `HOLD` (pause background
reads), `LED`, `CAPTURE`. Last measured capture rate is shown bottom-right.

**CONTROL** — six ± rows: gain, ATIME, ASTEP (steps of 50), SMUX, LED mA
(steps of 2), capture window (steps of 250 ms). Buttons: `CAPTURE`, `SAVE`,
`LOAD`, `DEFAULT`. Per-cycle integration time and the last capture result are
shown along the bottom.

The header carries sensor status, gain/integration/SMUX summary, an
ASAT/DSAT saturation flag, and a `USB` badge when a host has talked to the
board in the last three seconds.

Touch calibration constants (`TS_RAW_*`) are shared with `DataLogger_TFT` — if
your panel registers off, uncomment `TOUCH_DEBUG` and re-probe the corners.
