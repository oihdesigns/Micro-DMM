# MR60BHA2_Controller

Bench controller for the **Seeed XIAO 60GHz mmWave Human Breathing and Heartbeat
Sensor — MR60BHA2**, driven by the **XIAO ESP32-C6** that ships plugged into it.

Two pieces:

| File | Role |
|---|---|
| `MR60BHA2_Controller/MR60BHA2_Controller.ino` | Firmware — frame parsing, levers, capture, protocol tools |
| `mr60bha2_gui.py` | Host GUI — every lever, live traces, capture + FFT, protocol workbench |

No screen on this build; the Python GUI is the whole control surface.

---

## What the levers actually are

Worth being blunt up front, because it shapes everything below. Unlike the
AS7343 rig — where every knob is a register on the sensor — the MR60BHA2 is a
sealed module that streams decoded reports over UART, and **Seeed's library
exposes no configuration commands for it at all**. Nothing published maps its
command space either. So the levers here fall into three honest groups:

1. **Acquisition** — how the C6 pumps the module's UART. Real, measurable, and
   the one place a bad default costs you latency (see `pumpms` below).
2. **Interpretation** — validity gating, median + EMA filtering, presence hold,
   and a capture path that lets you re-derive breathing and heart rate from the
   raw phase signal instead of trusting the module's own numbers.
3. **Protocol** — raw frame monitoring, arbitrary frame transmission and a type
   range probe, so undocumented module commands can be hunted for from the GUI.

If you were expecting integration-time and gain equivalents, they do not exist
on this part. What you *can* do is see exactly what it emits, how often, and
whether its reported rate agrees with the waveform it derived it from.

---

## Wiring / board

Everything is hardwired by the carrier; nothing to solder.

| Signal | Pin | Note |
|---|---|---|
| Radar UART | D6 TX / D7 RX (UART0) | 115200 8N1, carrier-hardwired |
| RGB LED | D1 | single WS2812 on the carrier |
| Host | native USB | USB CDC |

Board: **XIAO_ESP32C6** — FQBN `esp32:esp32:XIAO_ESP32C6` (esp32 core 3.3.5
here). Compiles to ~304 KB flash / 82 KB RAM, of which 67 KB is the capture
buffer.

> **Tools ▸ USB CDC On Boot must be Enabled.** Otherwise `Serial` *is* UART0 and
> fights the radar for the same two pins — you get garbage in the monitor and no
> frames at all.

## Libraries

[`Seeed Arduino mmWave`](https://github.com/Love4yzp/Seeed-mmWave-library) (not
in the Library Manager index — install the GitHub zip), `Adafruit NeoPixel`,
plus `Preferences` from the esp32 core.

Host side: `pip install pyserial matplotlib numpy`.

---

## Frames are parsed by this sketch, not by the library

The sketch subclasses `SEEED_MR60BHA2`, overrides `handleType()`, and parses
every payload itself — the base implementation is deliberately never called.
The library is used only for framing and I/O. Two reasons, both of which matter
on the bench:

* **Its getters are one-shot.** `getBreathRate()` clears the valid flag and
  returns `false` from then on, so "no new frame" and "no reading" are
  indistinguishable to the caller.
* **They hide the interesting bit.** `getDistance()` returns `false` whenever the
  module's range flag is 0 — which is exactly the case worth seeing, "I have a
  range, but I do not trust it". `isHumanDetected()` likewise returns `false`
  both for "no human" and for "no frame since you last asked".

Parsing the payloads directly hands the host the flag *and* the value, and lets
every report type be counted for rate statistics even when `repmask` filters it
out of interpretation. As a side effect it also fixes an over-read: the
library's point-cloud handler trusts the target count in the payload without
checking it against the frame length, and walks off the end of a short frame.
This sketch bounds it and counts the frame as `bad` instead.

## The `pumpms` gotcha

`SeeedmmWave::fetch(timeout)` is `do { drain UART } while (millis() < expire)`.
It burns the **whole timeout every call** whether or not data arrives, so the
`mmWave.update(100)` that appears in every Seeed example costs 100 ms of dead
loop per pass. `fetch(0)` still runs the body once — draining everything
currently buffered — and returns immediately.

So `pumpms` defaults to **0**, and the loop is genuinely non-blocking. The lever
is left exposed because winding it up is the clearest way to *see* what the
blocking form costs: set `pumpms` to 100, watch the frame ages in the Protocol
tab stretch, and set it back.

---

## Serial protocol

Commands start with `!`, replies with `$`, and human-readable debug with `#`.
USB CDC, so the baud rate is ignored.

### Host → device

| Command | Effect |
|---|---|
| `!PING` | → `$HELLO,MR60BHA2,<fw>` |
| `!ID` | Module firmware → `$ID,<proj>,<major>,<sub>,<mod>` |
| `!CFG` | Dump config, one key per line, then `$CFGEND` |
| `!GET,<key>` | → `$VAL,<key>,<val>` |
| `!SET,<key>,<val>` | → `$OK,<key>,<val>` or `$ERR,<key>,range\|nokey` |
| `!READ` | One snapshot → `$R,…` |
| `!TGT` | Current target list → `$T,…` then `$TEND,<n>` |
| `!STREAM,0\|1` | Continuous `$R` lines |
| `!CAP[,<ms>]` | Timed capture of the phase waveform |
| `!ABORT` | End a running capture early and dump what it has |
| `!STATS[,0]` | Per-type frame counts and rates (`,0` clears them) |
| `!RAW,0\|1\|2` | Raw frame echo: off / unknown only / everything |
| `!TX,<type>[,<hex>]` | Send one frame to the module |
| `!PROBE,<t0>,<t1>[,<gap>]` | Sweep a type range looking for replies |
| `!RESET` | Pulse the module reset pin, if `rstpin` is set |
| `!ZERO` | Reset filters, presence state and statistics |
| `!SAVE` / `!LOAD` / `!DEFAULTS` | NVS-backed config |

`!ID` reports the **last version the module announced**, which it does
unprompted at power-up. There is no known way to ask for it on demand, so a
board that has been running since before you connected answers `$ERR,id,notseen`
until it is power-cycled.

### Config keys

| Group | Keys |
|---|---|
| Acquisition | `pumpms` (0–200), `strmode` (0 timer / 1 per phase frame), `streamms` |
| Reports | `repmask` (8-bit, one bit per report type), `rawmode` (0/1/2) |
| Gating | `dminmm`, `dmaxmm`, `brmin`, `brmax`, `hrmin`, `hrmax`, `holdms` |
| Filtering | `medn` (1–9, odd), `emabr`, `emahr`, `emad` (alpha × 1000) |
| Indicator | `ledmode` (0–4), `ledbri` |
| Capture | `capms`, `capmode` (0/1), `capdtms` |
| Protocol | `txck`, `unsafe`, `rstpin` (255 = none) |

`!CFG` reports each as `$CFG,<key>,<val>,<min>,<max>` — the GUI reads the ranges
from the device rather than hardcoding them — plus three read-only pseudo-keys
`fwmod`, `ntypes` and `capmax`.

Filtering defaults to passthrough (`medn 1`, all alphas 1000) on purpose: the
module smooths its own rate output hard already, and stacking another filter on
top mostly buys you lag. The levers are there so you can measure that, and so
you can deliberately make it worse when you want to see how a downstream
consumer behaves.

### Report types and `repmask`

| bit | type | name | payload |
|---|---|---|---|
| 0 | `0x0A13` | phase | total / breath / heart phase, 3 floats |
| 1 | `0x0A14` | breath | breath rate, 1 float |
| 2 | `0x0A15` | heart | heart rate, 1 float |
| 3 | `0x0A16` | dist | range flag u32 + range float |
| 4 | `0x0F09` | human | presence byte |
| 5 | `0x0A08` | pcdet | point-cloud detections |
| 6 | `0x0A04` | pctgt | tracked targets |
| 7 | `0xFFFF` | fwver | firmware version u32 |

Clearing a bit stops that report being *interpreted*; it is still counted in
`!STATS`, so "filtered" and "absent" stay distinguishable. Which of these the
module actually emits depends on its own firmware version — the point cloud and
human-presence reports appear on 1.6.x and later.

### `$R` snapshot

```
$R,<t_ms>,<present>,<rflag>,<dist>,<br>,<hr>,<total>,<breath>,<heart>,
   <br_raw>,<hr_raw>,<dist_raw>,<human>,<ntgt>,<age_ms>
```

`present` is derived from `holdms`; `rflag` is the module's own range-valid flag;
`*_raw` are ungated values straight off the wire, so a reading rejected by the
gating window is still visible. `nan` means never received, `human` is `-1` until
a presence frame arrives, and `age_ms` counts from the last *accepted* frame.

**Distance units are assumed to be metres** (`dminmm`/`dmaxmm` are that × 1000).
The module's own documentation does not state it and the library just passes a
float through. The GUI shows `dist_raw`, so check it against a tape measure on
first use before trusting the gate.

### Capture reply

```
$CAPB,<n>,<elapsed_us>,<rate_hz>,<mode>,<dt_ms>,<dropped>
$CH,t_us,total,breath,heart,dist,br,hr
$S,<t_us>,<total>,<breath>,<heart>,<dist>,<br>,<hr>     ← one line per sample
$CAPE,<n>
```

Buffer limit: 2400 samples — about two minutes if the module reports phase at
20 Hz, but check the measured rate in the Protocol tab rather than assuming it.
`dropped` counts anything the buffer had no room for.

---

## Capture modes

Two, because they measure different things:

* **`0` per frame** — one sample each time a phase report arrives, i.e. the
  module's native cadence, timestamped when it landed. Non-uniform if a frame is
  ever late or lost, but it is the honest record of what arrived and when. The
  reported rate is `n / elapsed`, measured.
* **`1` grid** — a snapshot of the latest values every `capdtms`. Uniformly
  sampled, so it goes straight into an FFT, at the cost of repeating values
  whenever the grid is faster than the module reports.

Capture is **non-blocking** — samples accumulate while the command parser stays
live, so `!ABORT` works and the GUI is not frozen for the window. The whole
buffer is dumped as text when the window closes.

---

## GUI tabs

* **Live** — big breathing / heart / distance readouts, presence and range-flag
  state, and two rolling strip charts (phase traces on top, rates below) over a
  configurable window. Stream on a timer or one line per phase frame. **CSV log**
  writes one row per `$R` as it arrives — no separate timer, because the device
  already sets the cadence — with the full field set including the raw ungated
  values, so the acquisition settings and the rejected readings both travel with
  the data.

* **Controls** — every lever grouped: acquisition, validity gating, filtering,
  indicator LED, capture, protocol switches, and the 8-bit report mask with
  All/None presets. A tree view mirrors the device's own reported config with its
  min/max limits, so firmware key additions show up without a GUI change.

* **Capture** — run a capture, plot any of the six recorded columns, and below it
  a **spectrum that re-derives the rate from the waveform**: pick a signal
  (breath / heart / total phase) and a band (breath 6–40 /min, heart 40–150
  /min), and it resamples onto a uniform grid, detrends, windows, transforms, and
  reports the interpolated peak next to what the module said. That comparison is
  the main reason this rig exists — the module's rate is a black box, and this is
  how you find out whether it is tracking the signal it is looking at. Save to
  CSV.

* **Protocol** — per-type frame counts, measured Hz and age (auto-refreshing if
  you want it), a raw frame monitor, the tracked-target list, manual frame
  transmission, and a type-range probe.

---

## Sending frames to the module

Nothing published documents this module's command space. `!TX` and `!PROBE`
exist so you can go looking, and both are gated:

* By default only types in `0x0A00–0x0AFF` and `0x0F00–0x0FFF` — the ranges its
  own reports use — may be sent.
* `unsafe 1` unlocks everything. The GUI puts a confirmation dialog in front of
  that switch, and `!PROBE` refuses to run without it.

The block is **precautionary, not authoritative**: it is not based on a published
map of what is dangerous, only on staying near the ranges known to be in use.
If firmware-update frames exist, they live somewhere in that space too. Probing
a range you have not thought about could land the module in a state only a power
cycle clears — write down what you send.

`txck` exists because the library's own `send()` omits the trailing data checksum
entirely when the payload is empty, which does not match the frame layout its own
receiver expects. This sketch builds frames itself and appends `0xFF` for empty
payloads by default; clear `txck` to reproduce the library's behaviour exactly.

`!PROBE` reports a type as responsive only when the module replies with a type
*outside* the known report set — spontaneous reports keep arriving throughout the
sweep and would otherwise look like a reply to everything. The board ignores host
commands while probing, and the range is capped at 256 types per run.

---

## Indicator LED

`ledmode`: `0` off, `1` presence (green present / dim red absent), `2` breath
phase as brightness, `3` one flash per heart report, `4` a flash per received
frame — the last being the fastest way to tell "the module is dead" from "the
module is talking and nothing is being decoded".

Mode 2 normalises against a slowly decaying peak rather than a fixed full scale,
because the phase floats have no documented range.
