# ADS122C04 AC Power Monitor / Logger

An AC-mode fork of `ads122c04_tests`: the same serial-command style and the same
GUI shape, with the thermocouple maths replaced by RMS / real-power maths and a
load relay added.

| | |
|---|---|
| `ads122c04_ac_power.ino` | firmware (Feather M4 / RP2040 / RP2350) |
| `ac_types.h` | the two types the .ino cannot declare itself — see the note in the file |
| `ads122c04_ac_gui.py` | the GUI — `pip install pyserial matplotlib` |
| `ac_power_profile.json` | written by **Save profile**, reloaded at startup |

## Wiring

| Signal | Pin |
|---|---|
| Mains voltage, differential | AIN0 (+) / AIN1 (−) |
| Load current, differential | AIN2 (+) / AIN3 (−) |
| Relay driver | D5 (open at boot; `!RLYINV,1` if the driver is active-low) |

Both inputs are differential and both are AC, so bias each pair near mid-supply
and leave the PGA bypassed — that is the default, and it lets the inputs swing
rail to rail. The board sweeps 0x40–0x4F at startup and reports which address
answered, so A0/A1 strapping does not matter.

## The one thing to know about the measurement

There is one ADC and one multiplexer, so voltage and current are **sampled
alternately**, not simultaneously — roughly 550–750 V/I pairs per second, about
9–12 pairs per 60 Hz cycle. Two consequences:

- Real power pairs each current sample with a voltage **interpolated** to the
  instant that current sample was taken (a cubic through `v[k-1 … k+2]`
  evaluated at `PHASECAL`, with the interpolator's own amplitude loss divided
  back out at the measured frequency). Against synthetic 50/60 Hz this returns
  P and PF exact to better than 0.01 % at any phase angle.
- **Anything above ~300 Hz aliases.** Vrms, Irms and P are right for linear
  loads and approximate for choppy ones. When the shape is what matters, take a
  **single-channel** waveform capture: one mux, continuous mode, the full
  2000 SPS — 33 points per 60 Hz cycle, good to the 16th harmonic.

Raising the I2C clock to 1 MHz (ADC tab) buys roughly a third more pairs per
second, because I2C time is most of the per-sample cost.

## Calibration, in order

Scaling is expressed as "this many ADC volts differential equals this much real
world", defaulting to **1.5 V = 170 V** and **1.5 V = 25 A**. Note that 170 V is
the *peak* of 120 Vrms; the readings on screen are RMS.

1. **Zero.** Front end powered, no mains, no load → *Scaling* tab → **Zero both
   channels**. Whatever is present at that moment becomes the new zero.
2. **Voltage.** Apply mains, measure it with a meter you trust. If the GUI reads
   `V_shown` and the meter reads `V_true`, set the voltage "equals" box to
   `old × V_true / V_shown`.
3. **Current.** Same trick against a clamp meter, with a load drawing a decent
   fraction of full scale.
4. **Phase.** With a *purely resistive* load (an incandescent bulb or a heater —
   not a switching supply), trim **PHASECAL** until PF reads 1.000. At 0.5 it is
   already correct for the interleave itself, so this step is only cancelling
   the phase error of a CT or of an input filter. Anything far from 0.5 means
   something else is wrong.
5. **Save profile**, so the numbers come back next launch.

## Reading the screen

**Trends** plots what the board computes, one point per measurement window
(~5/s): Vrms and Irms on top, real power and power factor below. **Waveform**
plots the raw conversions — voltage in volts on the left axis, current in amps
on the right — with RMS, P, PF, frequency and crest factors recomputed in the
GUI from the samples themselves. Tick **Repeat** to keep it refreshing.

The red text beside the LOAD button is the flag field from each window:

| Flag | Meaning |
|---|---|
| `NO ZERO X` | no voltage zero crossings — DC, no mains, or too short a window |
| `V CLIP` / `I CLIP` | that channel reached ~95 % of full scale |
| `TRIPPED` | over-current trip is latched; switch LOAD on to clear it |
| `TRUNCATED` | the window filled the buffer before it filled its time |
| `ADC ERR` | a conversion read failed |

**CSV log** writes one row per window with a wall-clock timestamp, every
computed quantity, the relay state and the flags.

## Safety

The relay is driven open in `setup()` before anything else runs, and the GUI
opens it again on disconnect. Set **Trip at** to a current above which the board
opens the relay by itself and latches; it is checked once per window (~200 ms),
so it is a convenience, not a substitute for a fuse or breaker.

A measurement window holds the serial port for ~200 ms, which is a long time to
sit on a relay command, so the acquisition loop drops out as soon as a byte
arrives and discards that window — relay latency is one sample, not one window.
