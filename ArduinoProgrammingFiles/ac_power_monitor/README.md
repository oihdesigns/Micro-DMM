# AC Power Monitor / Logger

An AC-mode fork of `ads122c04_tests`: the same serial-command style and the same
GUI shape, with the thermocouple maths replaced by RMS / real-power maths and a
load relay added.

| | |
|---|---|
| `ac_power_monitor.ino` | firmware, both front ends |
| `ac_types.h` | the two types the .ino cannot declare itself — see the note in the file |
| `ac_power_gui.py` | the GUI — `pip install pyserial matplotlib` |
| `ac_power_profile.json` | written by **Save profile**, reloaded at startup |

## Pick a front end

One line near the top of the sketch:

```c
#define ADC_BACKEND  BACKEND_RA4M1        // or BACKEND_ADS122C04
```

| | `BACKEND_ADS122C04` | `BACKEND_RA4M1` |
|---|---|---|
| Converter | external 24-bit delta-sigma, I2C | the RA4M1's own ADC |
| Inputs | truly differential | pseudo-differential (two conversions subtracted) |
| Resolution | 24 bit | 14 bit default (`!BITS`) |
| Sample grid | ~550–750 V/I pairs/s, whatever the ADC gives | 32 pairs per mains cycle, fixed (`!SPC`) |
| Boards built | Feather M4, Feather RP2040 | UNO R4 Minima, XIAO RA4M1 |

Everything downstream of acquisition — the maths, the windowing, the relay, the
energy accumulator, the waveform capture, the protocol — is shared, so the two
cannot drift apart in the part that is hard to get right. The GUI reads the
backend from `$CFG` and shows the matching ADC panel; nothing to set there.

## Wiring

| Signal | ADS122C04 | RA4M1 |
|---|---|---|
| Mains voltage | AIN0 (+) / AIN1 (−) | A0 (+) / A1 (−), see `!VPIN` |
| Load current | AIN2 (+) / AIN3 (−) | A2 (+) / A3 (−), see `!IPIN` |
| Relay driver | D5 | D5 |

The relay is opened at boot; add `!RLYINV,1` if the driver is active-low.

**ADS122C04.** Both pairs are truly differential, so bias each near mid-supply
and leave the PGA bypassed — the default, and it lets the inputs swing rail to
rail. The board sweeps 0x40–0x4F at startup and reports which address answered,
so A0/A1 strapping does not matter.

**RA4M1.** The ADC is single-ended only, so a "pair" is two conversions
subtracted: the **+** pin carries the signal biased at mid-supply, the **−** pin
watches the bias node itself, and subtracting cancels bias and supply drift.
The two conversions are not simultaneous, so the **−** pin wants to be a quiet
reference, not the other half of an anti-phase drive. Set it to *none* for a
plain single-ended read — per-window DC removal then does the same job, just
without rejecting supply noise. `VREF` should be the board's analog reference
(5.0 for an UNO R4, 3.3 for a 3.3 V board), though calibration absorbs any
error in it.

## The one thing to know about the measurement

Either way there is one converter, so voltage and current are **sampled
alternately**, not simultaneously. Real power therefore pairs each current
sample with a voltage **interpolated** to the instant that current sample was
taken — a cubic through `v[k-1 … k+2]` evaluated at `PHASECAL`, with the
interpolator's own amplitude loss divided back out at the measured frequency.
Against synthetic 50/60 Hz that returns P and PF exact to better than 0.01 % at
any phase angle.

The two front ends then differ in how much room they have:

- **ADS122C04** manages only ~9–12 pairs per 60 Hz cycle, so **anything above
  ~300 Hz aliases**. Vrms, Irms and P are right for linear loads and
  approximate for choppy ones. When the shape is what matters, take a
  **single-channel** waveform capture: one mux, continuous mode, the full
  2000 SPS — 33 points per 60 Hz cycle, good to the 16th harmonic. Raising the
  I2C clock to 1 MHz (ADC tab) buys roughly a third more pairs per second,
  because I2C time is most of the per-sample cost.
- **RA4M1** is quicker than the maths needs, so rather than racing it,
  acquisition is cut into equal time buckets and every conversion inside a
  bucket is averaged into one stored sample. That pins the sample grid to
  `SPC` per cycle whatever `analogRead` actually costs, spends the surplus
  speed on noise instead of discarding it, and the boxcar is a real anti-alias
  filter. Lowering `SPC` buys deeper averaging rather than losing data; the
  Device config panel shows how many conversions are landing in each sample.

## Calibration, in order

Scaling is expressed as "this many ADC volts differential equals this much real
world", defaulting to **1.5 V = 170 V** and **1.5 V = 25 A**. It works the same
way on either front end. Note that 170 V is
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
