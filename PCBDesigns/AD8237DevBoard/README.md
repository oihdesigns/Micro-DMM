# AD8237 Sensor Development Board - Rev C

KiCad 10, 60 x 50 mm, two layers. Eight I2C-selected gains, buffered half-supply reference, 3.3 V or 5 V analog supply.

Open `AD8237DevBoard.kicad_pro`. Nick's edited PCB is the layout source. Rev C replaces U2 and adds nine resistors. Existing placements are retained except C10, moved slightly to clear the larger mux. The pre-change design is in `archive/pre-eight-gain/`.

**Prototype:** not fabricated or measured. Electrical/layout checks and ideal DC calculations do not establish measured stability, noise, settling or accuracy.

## Eight gains

U2: **TMUX1108PWR**, PW TSSOP-16, 4.4 x 5 mm body, 0.65 mm pitch. U3: **TCA9536DGKR**, I2C address **0x41**. P0/P1/P2 drive A0/A1/A2; P3 is unused. No range-bank jumper is needed.

| Code / A2 A1 A0 | Nominal gain | Upper resistor | Lower resistor | U2 channel / pin |
|---|---:|---|---|---|
| 0 / 000 | 1 | R13 2k sense resistor | None | S1 / 4 |
| 1 / 001 | 2.5 | R21 30k | R22 20k | S2 / 5 |
| 2 / 010 | 5.02 | R23 40.2k | R24 10k | S3 / 6 |
| 3 / 011 | 10.09 | R4 90.9k | R3 10k | S4 / 7 |
| 4 / 100 | 25.048096 | R25 120k | R26 4.99k | S5 / 12 |
| 5 / 101 | 49.780488 | R27 100k | R28 2.05k | S6 / 11 |
| 6 / 110 | 101 | R6 100k | R5 1k | S7 / 10 |
| 7 / 111 | 1001 | R12 1M | R11 1k | S8 / 9 |

Upper resistors run from OUT_RAW to the selected tap; lower resistors run from the tap to REF. Gain resistors are 0.1% thin film, <=25 ppm/degree C, 0805. The original 10.09/101/1001 resistor pairs are retained. The mux selects a high-impedance feedback node, keeping switch on-resistance out of the gain-setting resistor ratio.

Initialize U3 in this order:

1. Write register `0x01 = 0x00`: preload unity into the output latch.
2. Write register `0x03 = 0xF0`: P0-P3 outputs; unused P3 stays low, avoiding a floating input when pull-ups are disabled.
3. Write register `0x50 = 0x40`: disable internal port pull-ups.

Then write code 0-7 to register `0x01`, updating all selection bits together. **The code mapping changed from Rev B:** code 1 is now 2.5, not 10.09.

R17/R18/R29 are 4.7k pull-downs: the maximum specified 100 uA internal startup pull-up current produces <=0.475 V including resistor tolerance. EN is tied high; VSS is grounded. After valid supplies establish, the default is unity before software initialization. Reinitialize after VIO power cycling; a host reset alone does not reset a separately powered U3.

The Arduino Wire example is `firmware/AD8237GainExample/AD8237GainExample.ino`. It starts at unity and accepts a serial digit 0-7. Its 50 ms startup and 20 ms gain-change delays are initial bench-test allowances, not guaranteed settling times. Discard ADC samples during switching and settling. The example has not been compiled for a particular controller or tested on hardware.

## Connections

| Connector | Pins in order |
|---|---|
| J1 | 1 external regulated analog supply, 2 GND |
| J2 | 1 IN+, 2 IN-, 3 GND |
| J3 | 1 OUT after R8, 2 GND, 3 external reference |
| J4 | 1 GND, 2 host VIO supply, 3 SDA, 4 SCL |

| Jumper | Default | Alternative |
|---|---|---|
| JP1 | 2-3: analog supply from VIO | 1-2: analog supply from J1 |
| JP2 | **1-2: LOW bandwidth** | 2-3: HIGH only while gain >=10 |
| JP3 | 3-4: buffered VS/2 | 1-2: GND; 5-6: external J3 reference |
| JP4 | Fitted: 4.7k I2C pull-ups to VIO | Remove if host has suitable pull-ups |

Fit one shunt on each JP1/JP2/JP3, plus JP4 as appropriate. Change supply/reference shunts with power off. **Keep JP2 LOW for normal eight-gain operation and startup.** HIGH is unsuitable at gains 1, 2.5 and 5.02. With JP2 empty, R7 pulls BW low.

For normal 3.3 V use, power J4 VIO at 3.3 V and select JP1 2-3. For 5 V analog with a 3.3 V controller, power J1 from regulated 5 V, J4 VIO from 3.3 V, and select JP1 1-2. The mux accepts 3.3 V controls with a 5 V supply. Always match VIO to the host logic voltage. A 5 V analog output can exceed a 3.3 V ADC's input range.

## Analog behavior

`VOUT_RAW = VREF + G * (VIN+ - VIN-)`, before output resistor R8.

OPA333 U4 buffers a filtered equal-10k divider, giving about 1.65 V at VS=3.3 V or 2.5 V at VS=5 V. Selected REF drives all seven divider returns; R14 connects it to the AD8237 REF pin. External references must be low impedance and able to source and sink divider current.

The seven dividers present about 12.7k from OUT_RAW to REF. Prefer an output load of **100k or greater**: effective load is then about 11.3k, above the AD8237's recommended 10k. At 1.65 V output-reference difference, divider current is about 130 uA. R8 adds about 0.1% attenuation into 100k; TP5 senses before R8. Individual feedback divider Thevenin resistances are below 30k.

At gain 1001 and REF=1.65 V, +/-1 mV differential input ideally gives 0.649-2.651 V output. Both physical inputs must remain within supply limits. Allow output headroom and calibrate for resistor ratios, temperature and amplifier/load errors.

C7 stays fitted at 470 pF C0G, following AD8237 Figure 76. The TMUX1108 has higher feedback-node capacitance than the former mux. LOW bandwidth is the baseline; bench-check overshoot, oscillation and settling at every gain. No device-level transient simulation or measured stability margin is claimed.

C3-C6 and R9/R10 are DNP by default. They provide optional filters and input bias-current returns. Floating sensors need an appropriate DC return. C7 and the remaining resistors/capacitors are fitted.

## Deliverables and checks

- `docs/AD8237DevBoard-schematic.pdf`: current schematic.
- `docs/ERC.json`, `docs/DRC.json`, `docs/design-verification.json`: electrical, routing/parity and independent pin/gain verification.
- `manufacturing/BOM.csv`, `manufacturing/placement.csv`: current assembly data.
- `manufacturing/AD8237DevBoard-RevC-Gerbers.zip`: current fabrication package.
- `manufacturing/stencil/`: paste with DNP parts omitted.

General routing uses 0.20 mm tracks/clearance. A local 0.15 mm pad-clearance override on U3 accommodates that standard package's pad gaps. Other footprints retain the project clearance rule.

`scripts/export.ps1` checks and exports the edited board. `scripts/design.py` regenerates the schematic from component data, preserving existing project settings; it does not ingest manual schematic changes. The old full-board builder is disabled to protect the edited PCB. The one-time migration writes only a draft under `tmp/eight-gain/`.

## Datasheet references

- [AD8237](https://www.analog.com/media/en/technical-documentation/data-sheets/ad8237.pdf): pinout p8, gain/BW p20, programmable gain Figure 76 p25.
- [TMUX1108](https://www.ti.com/lit/ds/symlink/tmux1108.pdf): PW pinout p2-3, supply and logic ratings, truth table p23.
- [TCA9536](https://www.ti.com/lit/ds/symlink/tca9536.pdf): DGK pins, startup pull-ups, address and registers p19-20.
- [OPA333](https://www.ti.com/lit/ds/symlink/opa333.pdf): SOT-23 pinout and buffer operation.
