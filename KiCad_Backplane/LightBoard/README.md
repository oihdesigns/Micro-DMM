# LightBoard rev A

A 4-channel bright white LED board, 56 x 36 mm, 4 layers, built almost entirely from parts in
`AvaliableParts/Combined_parts_DigiKey_and_Mouser.xlsx`.

- **LEDs:** 4 x Samsung LH351D 4000 K (SPHWHTL3DA0GF4TPQ6) in a 2 x 2 cluster with 8 mm pitch.
- **Drivers:** 4 x MIC4801YM, a 600 mA linear current sink, one per LED.
- **Supply:** +5 V on J1 (**5.5 V max**; the MIC4801 is a 6 V part). A low-side N-FET
  (Q1, AOSS32334C) gives reverse-polarity protection.
- **Control:** SW1 **ON** runs all four LEDs at full set current. SW1 **EXT** takes enable/PWM
  from J2 (3.3 V or 5 V logic, 200 Hz to 10 kHz for linear dimming). With nothing connected
  to J2, the LEDs stay off. The first PWM pulse after more than 40 ms low must be at least
  60 µs long to wake the MIC4801.

## Current setting (R5-R8, one per channel)

ILED = 4920 / (RSET[kΩ] - 0.14)

| RSET | Current per LED | Board input | Notes |
|---|---|---|---|
| 22k | 225 mA | 0.9 A | |
| **15k (default)** | **331 mA** | **1.35 A** | about 6.5 W of heat in total |
| 10k | 499 mA | 2.0 A | about 10 W of heat; needs a real heatsink |

## Why the 2.7 Ω resistors (R1-R4) are there

They are the only parts that are **not in stock**: Yageo RC2512FK-7W2R7L, 2512, 2 W (LCSC C5843102).

With a 5 V supply, each LED leaves about 2.2 V of headroom. Without a dropper, the SOIC-8 would
have to burn all of it (about 0.75 W per chip at 331 mA, with θJA ≈ 99 °C/W). The resistor takes
0.9 V of that, which leaves the MIC4801 about 0.45 W. If you run the board from a 1-cell Li-ion
(3.0-4.2 V), fit 0 Ω links instead.

## Heat: read this before powering up

This is a linear driver, so all the power drawn from the 5 V input ends up as heat on this small
board: about 6.5 W at the default current.

- **Bolt it to a heatsink.** Use an aluminium plate of roughly 100 cm² or more, with a thermal pad
  between it and the board, and the four M3 holes.
- The back copper (B.Cu) is GND and sits under solder mask.
- Each LED's thermal pad (electrically isolated) is tied to GND copper, with via arrays above,
  between and below the LED rows.
- Bare on the bench, use EXT with a low PWM duty cycle.

## Assembly notes

- The LH351D thermal pad sits under the part, so it needs a hot plate or hot air, not an iron.
- Check SW1's ON/EXT direction with a meter. The silk assumes that the actuator on the pin-1
  side connects common (pin 2) to pin 1.

## Files

- `tools/design.py` is the single source of truth for the circuit.
- `tools/make_lib.py` builds the symbols and the LH351D footprint. The footprint follows the
  datasheet Rev 9.1, p.16 recommended pattern.
- `tools/gen_sch.py` and `tools/gen_pcb.py` regenerate the schematic and the **fully routed**
  board. The PCB generator does the routing itself: explicit tracks plus an automatic plane-via
  placer.
- After running `gen_pcb.py`, refill the zones with
  `kicad-cli pcb drc --refill-zones --save-board LightBoard.kicad_pcb`.

Status (2026-11-22): ERC 0. DRC 0 errors, 0 unconnected, schematic parity 0; 4 silk-at-edge
warnings remain from the flush terminal blocks. Not fabricated.
