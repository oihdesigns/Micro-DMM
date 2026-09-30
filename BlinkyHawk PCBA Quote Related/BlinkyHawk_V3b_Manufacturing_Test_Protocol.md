# Blinky Hawk V3b — Manufacturing Test Protocol

**Document:** BH-V3b-MTP rev A
**Date:** 2026-08-24
**Owner:** Nicholas Mann, OIH Designs — Nick@OIHDesigns.com
**Purpose:** Define the test and acceptance work to be **quoted and performed** for Blinky Hawk V3b, at both the bare-PCBA stage and the finished-unit stage.

> **To the quoting manufacturer:** please price Gate 1 and Gate 2 as separate line items, and call out fixture NRE separately again. Section 11 lists open items on our side; a quote may be issued against the assumptions stated there, and we will confirm each before a purchase order.

---

## 1. Product summary

Blinky Hawk is a pocket detector that clips onto a standard digital multimeter and piggybacks on the meter's V/COM inputs through stacking banana plugs. It continuously answers one question the meter cannot: when the display reads 0 V, is that a **real zero** (leads closed on a genuine circuit) or **no connection at all** (open leads)?

It reports three states on an RGB LED and a piezo buzzer:

| State | Meaning | LED | Buzzer |
|---|---|---|---|
| **Floating** | Open leads — no low-impedance path between the probes | Dim blue flash | Silent |
| **Closed** | Low-impedance path between the probes | Green flash | Single beep |
| **Voltage** | Voltage present above approx. 0.6 V absolute | Red flash | Double beep |

The measurement principle: a bridge MOSFET (Q1) briefly breaks a biased divider and the firmware watches how the sense node settles. The recovery time maps to the impedance between the probes, which is how the unit answers without ever becoming an ohmmeter or leaving voltmeter mode.

**Core electrical facts relevant to test:**

| Parameter | Value |
|---|---|
| MCU | Seeed Studio XIAO RA4M1 (module, soldered to the main PCB) |
| Battery | LiPo 402535, 3.7 V, 320 mAh, JST to J4 |
| Charging | USB-C on the XIAO module |
| Probe-to-probe impedance, bridge resting (Q1 on) | approx. 1.1 MΩ |
| Probe-to-probe impedance, bridge open (Q1 off) | approx. 2.0 MΩ |
| Current into a device under test | approx. 6 µA worst case near a short, falling to nano-amps at high impedance |
| Ghost voltage on host meter, open leads | approx. −5 mV DC |
| Current draw, awake | approx. 11 mA |
| Current draw, low-power polling | approx. 2 mA |
| Serial console | USB CDC, 115200 baud, line based |

---

## 2. Scope and responsibility split

| Activity | CM | OIH Designs |
|---|---|---|
| SMT assembly of main PCB, including XIAO module | ● | |
| Gate 1 test (Section 6) | ● | |
| Firmware programming | ● | |
| Serial number assignment | | ● |
| Final assembly: switch, battery, banana block, shell (Section 7) | ○ quote optional | ● if not quoted |
| Gate 2 test (Section 7) | ○ quote optional | ● if not quoted |
| Test fixture design and build | ● quote as NRE | |
| Host test script | ○ see 5.2 | ● supplies protocol + golden config |
| Failure analysis beyond Section 10 | | ● |

● = responsible ○ = optional, price separately

---

## 3. Definitions

- **DUT** — device under test: a populated PCBA at Gate 1, a finished unit at Gate 2.
- **Gate** — a pass/fail checkpoint. Product may not advance to the next stage until it passes.
- **Golden config** — the reference `!CFG` dump supplied by OIH, against which each unit is compared.
- **Lot** — one build against one purchase order, identified `BH3b-<YYYYMMDD>-<nn>`.
- **Probe nets** — `V_Input_REF` at J1 and `V_Input_Float` at J2. These are the two connections that reach the outside world through the banana plugs.

---

## 4. Safety

1. **Lithium cells.** The 402535 cell is shipped and stored charged. Do not crush, puncture, short or heat. Store below 30 °C in a fire-resistant container. Cells showing swelling, corrosion or physical damage must be quarantined and not fitted. Confirm you hold current UN38.3 documentation for the cell you supply or receive, and state in your quote how you intend to ship assembled units containing cells.
2. **ESD.** The XIAO module, Q1, LED1 and U3 are ESD-sensitive. Standard EPA controls apply throughout.
3. **High voltage is out of production scope.** The product is verified by design from 0 V to ±1 kV. **Do not apply more than 12 V to the probe nets on the production line.** High-voltage verification is a design-validation activity performed by OIH, not a production test.
4. **No mains.** Nothing in this protocol requires connection to mains wiring.

---

## 5. Test equipment and fixture

### 5.1 Fixture (quote as NRE)

A simple jig is sufficient; a full bed-of-nails ICT fixture is **not** required.

Required features:

- Spring-pin or clip contact to **J1** and **J2** (probe nets).
- Spring-pin or clip contact to **J4** pin 1 (Gnd) and pin 2 (BatteryRail) for the bench supply and current measurement, in place of the battery. If the J4 header is not fitted — see Section 11, item 2 — contact the J4 **pads** directly.
- USB-C lead to the XIAO for programming, console and charge-detect testing.
- A switchable load bank across J1–J2 offering, as a minimum: **short (0 Ω)**, **10 kΩ ±1%**, and **open**.
- A switchable DC source across J1–J2 offering **0 V** and **+2.0 V ±5%**, current-limited, referenced so that neither probe net is tied to earth.
- Means of confirming the LED colour and that the buzzer sounds. An operator may do this by eye and ear; an optional colour sensor and microphone threshold are welcome if you wish to price full automation.

The fixture must not tie either probe net to earth ground during detection tests. The measurement is differential and floating; an earth reference on one side will produce false results.

### 5.2 Host test script

OIH supplies the serial protocol (Appendix A) and the golden config. The CM supplies a host script that drives the sequence, evaluates limits and writes one record per unit (Section 9). Any language is acceptable. If you would rather OIH supply the script, say so in the quote and we will provide a Python reference implementation.

### 5.3 Instruments

- DMM with a low-voltage ohms range (test voltage below 0.3 V) and 4½-digit resolution or better, calibrated.
- Bench supply, 3.7 V nominal, with current readout to 0.1 mA, or a series microammeter.
- Standard AOI capability.

> **Ohms-range note:** the sense node carries Schottky clamps D1/D2 to the 3.3 V rail and ground. A meter whose ohms range applies more than roughly 0.3 V can forward-bias a clamp and give a misleading probe-to-probe reading. Use a low-voltage ohms range.

---

## 6. Gate 1 — Bare PCBA acceptance

Performed on the populated board with the XIAO module fitted, **before** any wiring, battery, switch or shell. **100% of units.**

### 6.1 Visual and AOI

| ID | Check | Accept |
|---|---|---|
| 1.1.1 | AOI: presence, orientation, polarity of all placements | No defects |
| 1.1.2 | Q1, U3, D1, D2, LED1 orientation specifically | Correct per CPL |
| 1.1.3 | XIAO module seating and solder fillets on all pads | IPC-A-610 Class 2 |
| 1.1.4 | Solder bridging at LED1 and the XIAO castellations | None |
| 1.1.5 | Board cleanliness, no flux residue in the J1/J2/J5 wire-pad area | Clean |

Workmanship standard: **IPC-A-610 Class 2** unless otherwise agreed.

### 6.2 Pre-power resistance checks

Board unpowered, nothing connected to J4. Q1 is held off by its gate pulldown R4, so the only DC path between the probe nets is through the two 500 kΩ ladders and R22.

| ID | Measurement | Nominal | Limits | Catches |
|---|---|---|---|---|
| 1.2.1 | J1 to J2 | 2.00 MΩ | **1.90–2.10 MΩ** | Missing, wrong-value or unsoldered ladder resistor; missing R22 |
| 1.2.2 | J4 pin 2 (BatteryRail) to J4 pin 1 (Gnd) | — | **> 100 kΩ** | Assembly short on the battery rail |
| 1.2.3 | J1 to J4 pin 1 (Gnd) | — | **> 1 MΩ** | Probe net shorted to ground |
| 1.2.4 | J2 to J4 pin 1 (Gnd) | — | **> 1 MΩ** | Probe net shorted to ground |

Test 1.2.1 is the single most valuable pre-power check: it exercises all ten ladder resistors and R22 in one measurement.

> Limits assume 1% resistors. If the parts actually fitted are 5%, the 1.2.1 window must be widened to 1.80–2.20 MΩ — see Section 11, item 3.

### 6.3 Power-up

Supply 3.70 V ±0.05 V to J4 with the current meter in series. USB **disconnected**.

| ID | Check | Accept |
|---|---|---|
| 1.3.1 | Inrush does not trip the supply's current limit at 100 mA | Pass |
| 1.3.2 | Board draws current consistent with a running MCU | **3–20 mA** |
| 1.3.3 | LED1 produces its boot indication (1–4 green blinks) | Visible |
| 1.3.4 | No component detectably warm after 60 s | Pass |

A board that draws under 1 mA or over 50 mA at 1.3.2 is a hard fail — stop and quarantine rather than continuing to programming.

### 6.4 Firmware programming

| ID | Step | Accept |
|---|---|---|
| 1.4.1 | Load the OIH-supplied firmware image over USB | Programmer reports success |
| 1.4.2 | Power-cycle; open the console at 115200 baud | Board enumerates as USB CDC |
| 1.4.3 | Send `!STATUS` | A single `$STATUS,...` line returns |

The RA4M1 stores its configuration in data flash, which a sketch upload does not erase. On a factory-blank board the firmware loads and saves defaults automatically on first boot.

### 6.5 Configuration verification

| ID | Step | Accept |
|---|---|---|
| 1.5.1 | Send `!CFG`, capture all `$CFG` rows to `$CFGEND` | Complete dump received |
| 1.5.2 | Compare against the golden config | Every key matches |
| 1.5.3 | Confirm `hwrev=3` in `$STATUS` | **Must be 3** |
| 1.5.4 | Confirm `dip=3` (THRESHSEL) in `$STATUS` | Must be 3 |
| 1.5.5 | Confirm `sn=` is empty | Empty at Gate 1 |

> **1.5.3 is safety-relevant, not cosmetic.** On hardware revision 2, D8 is a DIP-switch input. If a V3b board ever came up believing it were a V2, the firmware would drive D8 as a push-pull output into what it thinks is a switch. Verify `hwrev=3` on every unit and fail any board that reports otherwise.

### 6.6 Bridge and detection functional test

Board powered from J4 at 3.70 V. Allow 2 s after each load change before reading state.

**6.6a — Bridge switching (verifies Q1 and R5 directly)**

| ID | Command | Measure J1–J2 | Limits |
|---|---|---|---|
| 1.6.1 | `!MOSFET,1` (hold on) | approx. 1.09 MΩ | **1.02–1.16 MΩ** |
| 1.6.2 | `!MOSFET,0` (hold off) | approx. 2.00 MΩ | **1.90–2.10 MΩ** |
| 1.6.3 | `!MOSFET,-1` | Returns to automatic detection | `$OK` |

A board that passes 1.2.1 but fails 1.6.1 has a faulty Q1, R5, or D7 net.

**6.6b — Detection states**

| ID | Load across J1–J2 | Expected state | LED | Buzzer |
|---|---|---|---|---|
| 1.6.4 | Open | Floating | Dim blue flash | Silent |
| 1.6.5 | Short, 0 Ω | Closed | Green flash | Single beep |
| 1.6.6 | 10 kΩ ±1% | Closed | Green flash | Single beep |
| 1.6.7 | Open again | Floating | Dim blue flash | Silent |

State is read from the `$STATUS` line and confirmed visually or by the fixture's sensors.

> **Deliberately excluded:** intermediate resistances between 10 kΩ and open. The closed/open boundary is a tuned design parameter, not a production limit, and units are not screened against it. If tighter coverage is wanted, OIH will supply a "must read closed" maximum and a "must read open" minimum — see Section 11, item 4.

**6.6c — Voltage detection**

| ID | Applied across J1–J2 | Expected state | LED | Buzzer |
|---|---|---|---|---|
| 1.6.8 | +2.0 V DC, floating source | Voltage | Red flash | Double beep |
| 1.6.9 | Source removed, leads open | Returns to Floating within 3 s | Dim blue | Silent |

**Do not exceed 12 V on the production line.**

### 6.7 Indicators

| ID | Check | Accept |
|---|---|---|
| 1.7.1 | LED1 produces all three colours across 6.6 | Blue, green and red all seen |
| 1.7.2 | LED1 colours are correct and not swapped or washed out | Correct |
| 1.7.3 | Buzzer audible at 30 cm in normal room noise during 1.6.5 | Audible |
| 1.7.4 | Buzzer double-beep distinguishable from single beep at 1.6.8 | Distinguishable |

The buzzer is a 4 kHz resonator driven anti-phase between D8 and D9. Both states use the same pitch and are told apart by pulse count, so 1.7.4 confirms both drive legs are working — a single-leg failure still makes noise but at reduced volume.

### 6.8 Charge detect

| ID | Step | Accept |
|---|---|---|
| 1.8.1 | Connect USB; send `!STATUS` | `charge=1` |
| 1.8.2 | Observe LED | Slow dim-red 25% blink |
| 1.8.3 | Disconnect USB; send `!STATUS` | `charge=0` |

This verifies the R20/R21 divider into A3.

### 6.9 Battery sense

With the supply at J4 set to each value, read `battv` from `$STATUS`:

| ID | Supply at J4 | `battv` accept |
|---|---|---|
| 1.9.1 | 3.70 V | **3.55–3.85 V** |
| 1.9.2 | 4.10 V | **3.95–4.25 V** |

### 6.10 Current draw

Measured at J4, USB disconnected, leads open.

| ID | Condition | Nominal | Limits |
|---|---|---|---|
| 1.10.1 | Awake, running detection | approx. 11 mA | **7–16 mA** |
| 1.10.2 | `!FLOOR,1` — firmware-reachable floor | TBD — see Section 11 item 5 | Record only for the first lot |

`!FLOOR` parks the board in a fixed state so a series ammeter reads something stable. Send the command over USB, then unplug USB; the board keeps running on the bench supply in the parked state.

For the first production lot, **record** 1.10.2 on every unit and return the data without applying a limit. OIH will set the limit from that distribution for subsequent lots.

### 6.11 Gate 1 acceptance

A board passes Gate 1 only if **every** test above passes. Boards that fail move to Section 10.

---

## 7. Gate 2 — Final assembly and finished-unit test

Applies to the box-build stage: fitting the 3-position toggle switch, the LiPo cell, the banana-plug block with its 240 mm (9.5 in) lead, and the printed shell and lid. Quote separately; OIH will perform this stage in-house if it is not quoted.

### 7.1 Incoming inspection

| ID | Item | Check |
|---|---|---|
| 2.1.1 | LiPo 402535 cells | No swelling, damage or corrosion; open-circuit voltage 3.6–3.9 V; JST polarity correct |
| 2.1.2 | Toggle switch | 3-position, correct part, contacts clean |
| 2.1.3 | Banana plug block, lead, shell, lid | Correct parts, no print defects, plugs seat firmly |

> **Check JST polarity on every cell.** J4 pin 1 is ground, pin 2 is BatteryRail. Cell vendors are not consistent about JST wire order, and a reversed cell will damage the board.

### 7.2 Assembly workmanship

| ID | Check | Accept |
|---|---|---|
| 2.2.1 | Wire terminations at J1, J2, J5 and the switch | Full fillets, no cold joints, no strand escape |
| 2.2.2 | Strain relief on the probe lead where it leaves the shell | Lead does not transmit force to the solder joint |
| 2.2.3 | Cell secured, cannot move inside the shell, wires clear of the lid | Pass |
| 2.2.4 | No wire pinched by the lid at closure | Pass |
| 2.2.5 | Shell screws present and seated, hook-and-loop pad applied to the back | Pass |

### 7.3 Serial number assignment — OIH

Performed by OIH, listed here so the CM's records can be reconciled.

| ID | Step |
|---|---|
| 2.3.1 | `!SN,<value>` writes the serial number to its own EEPROM block |
| 2.3.2 | `!SN` reads it back; must match |
| 2.3.3 | Serial number recorded against the CM's lot and unit index |

No `!SAVE` is needed — the serial number lives in a separate EEPROM block from the config and survives `!DEFAULTS`.

### 7.4 Finished-unit functional test

Repeat, on the assembled unit, through the banana plugs rather than the bare pads:

| ID | Test | Reference |
|---|---|---|
| 2.4.1 | Probe-to-probe resistance, unit switched on, bridge resting | approx. 1.1 MΩ |
| 2.4.2 | Detection states: open, short, 10 kΩ | As 6.6b |
| 2.4.3 | Voltage detection at +2.0 V | As 6.6c |
| 2.4.4 | LED and buzzer | As 6.7 |
| 2.4.5 | USB charge detect | As 6.8 |

### 7.5 Switch function

The 3-position toggle selects off, normal operation, and a 100 kΩ test load used to tell a real supply from a phantom voltage.

| ID | Position | Check |
|---|---|---|
| 2.5.1 | Off | Unit fully off. Probe-to-probe reads open, and the host meter reads as it would with nothing attached. |
| 2.5.2 | Normal | Detection functions per 2.4.2 |
| 2.5.3 | Test load | 100 kΩ ±5% appears across the probes |

> **2.5.1 is a published product claim** — "off means off," the switch disconnects the battery *and* both leads. Verify it on every unit rather than sampling.

### 7.6 Charge and runtime — sample

| ID | Test | Sample | Accept |
|---|---|---|---|
| 2.6.1 | Full charge from USB, cell reaches 4.15 V ±0.1 V | 1 in 20 | Pass |
| 2.6.2 | Awake current on battery | 100% | **7–16 mA** |
| 2.6.3 | 4-hour soak, unit left in floating state, then confirm still functional | 1 in 20 | Pass |

Full runtime is a design-validation claim of roughly 100 hours per charge and is **not** production-tested. Current draw at 2.6.2 is the production proxy.

### 7.7 Cosmetic and packaging

| ID | Check |
|---|---|
| 2.7.1 | Shell free of scuffs, cracks and stringing; lettering legible |
| 2.7.2 | Correct colourway (currently purple shell, green lettering) |
| 2.7.3 | Plug cover fitted; quick-start sheet included |
| 2.7.4 | Cell at partial charge for shipping, not full |

---

## 8. Sampling

| Stage | Plan |
|---|---|
| Gate 1, all tests | 100% |
| Gate 2, 7.1–7.5 | 100% |
| Gate 2, 7.6 charge and soak | Per table |
| Gate 2, 7.7 cosmetic | 100% |

The Gate 1 sequence is scriptable and should run in well under two minutes per board once the fixture exists. If your quote assumes sampling rather than 100% on any electrical test, state which tests and at what AQL.

---

## 9. Records and traceability

Return with every shipment:

1. **Per-unit test record** — lot ID, unit index, date, operator or station, firmware version, and pass/fail plus measured value for every numbered test that produces a number.
2. **The raw `!CFG` dump** captured at 1.5.1, one file per unit or one concatenated file keyed by unit index.
3. **Summary** — units built, units passed, units failed, failure Pareto by test ID.
4. **Deviations** — anything done differently from this document, and why.

CSV or JSON both acceptable. Units are not yet serialised at Gate 1, so the CM's unit index is the traceability key until OIH assigns serial numbers at 2.3.1. Mark each board's index physically or keep boards in indexed trays.

---

## 10. Failure handling

1. **Quarantine.** Failed units are segregated and clearly marked with the failing test ID.
2. **Rework.** Solder defects, bridges and misplacements found at 6.1 or 6.2 may be reworked and retested from the start of Gate 1. Record the rework.
3. **No rework without notice on:** Q1, U3, LED1 or the XIAO module. Notify OIH before reworking these; they are more likely to indicate a process problem worth diagnosing than a one-off.
4. **Stop-and-call.** Notify OIH immediately, and hold the lot, if any of these occur:
   - More than 5% of a lot fails any single test.
   - Any unit fails 1.5.3 (`hwrev`).
   - Any unit shows a battery-rail short at 1.2.2.
   - Any cell shows swelling or damage.
5. **Retest after rework is full retest**, not just the failing step.

---

## 11. Open items for OIH to close before a firm quote

These came out of preparing this protocol. Each is a real gap between the documents in this folder; a quote may assume the stated default.

1. **The BOM does not list the XIAO RA4M1 module.** `BOM-BlinkyHawk_V3b.csv` covers only the discrete components. The MCU module is the single most expensive item on the board. *Assume for quoting:* the module is consigned by OIH unless quoted otherwise — please price both ways.

2. **J4 is in the CPL but has been removed from the BOM.** `CPL-BlinkyHawk_V3b.csv` still places J4 as a `PinHeader_1x02_P1.27mm_Vertical`, but the matching BOM line has been deleted. If that deletion is intentional — battery wires soldered directly, or the header consigned — then **the CPL line should be deleted or marked DNP to match**, or the CM will quote it, query it, or fit it. *Assume for quoting:* J4 is **not** fitted and battery wires are soldered to the J4 pads. This affects the fixture at 5.1, which must then contact the pads rather than a header.

3. **Resistor tolerance is not stated in the BOM.** All limits in 6.2 and 6.6a assume 1%. If the fitted parts are 5%, those windows widen as noted.

4. **Detection boundary is not specified as a production limit.** Section 6.6b tests only short, 10 kΩ and open. If screening against the closed/open boundary is wanted, OIH must supply the two limit resistances.

5. **`!FLOOR,1` current is not yet characterised.** First lot records data without a limit; OIH sets the limit afterwards.

6. **Voltage reference part is ambiguous.** The V3b BOM specifies `TL431DBZ` for U3, while the published design description refers to a 1.25 V reference (LM4060). These are different parts with different reference voltages. This does not change any test in this document — no test measures the reference node directly — but it must be resolved before the BOM is released for purchasing. *Action: OIH to confirm which part the V3b board is designed around.*

7. **R16 and R17 are described in firmware but absent from the BOM.** The firmware header documents hardware revision 3 as driving the buzzer through 100 Ω series resistors R16 and R17 on D8 and D9. Neither appears in the V3b BOM or on the V3 schematic, where BZ1 connects directly. This affects buzzer drive current and the 6.7 limits. *Action: OIH to confirm whether V3b intends the series resistors.*

8. **J5's net is not shown on the V3 schematic sheet** used to prepare this document, though it appears in both the BOM and CPL. Fixture design at 5.1 does not depend on it, but assembly instructions at 2.2.1 do. *Action: OIH to confirm J5's function and mating wire.*

---

## Appendix A — Serial command reference

USB CDC, **115200 baud**, line based, each command terminated with a newline.

### Commands used by this protocol

| Command | Purpose | Used at |
|---|---|---|
| `!STATUS` or `!?` | Print the full status line | Throughout |
| `!CFG` | Dump every config key as `$CFG` rows, ending `$CFGEND` | 1.5.1 |
| `!GET,<key>` | Report one config value | As needed |
| `!MOSFET,<-1\|0\|1>` | Bridge: −1 automatic, 0 hold off, 1 hold on | 6.6a |
| `!FLOOR,<0-3>` | Park the board in a fixed state for current measurement | 1.10.2 |
| `!SN[,<value>]` | Read, or write, the unit serial number | 2.3.1 |

### Other commands, for reference and failure analysis

| Command | Purpose |
|---|---|
| `!SET,<key>,<value>` | Set a config value in RAM, effective immediately |
| `!SAVE` | Persist the RAM config to EEPROM |
| `!LOAD` | Discard RAM changes, reload from EEPROM |
| `!DEFAULTS` | Load factory defaults into RAM |
| `!DIAG[,0\|1]` | Enter or exit diagnostic mode |
| `!STREAM[,0\|1]` | Continuous raw streaming, diagnostic mode only |
| `!RATE,<ms>` | Stream interval |
| `!VMODE,<0\|1\|2>` | Voltage mode: 0 auto, 1 lock on, 2 disable |
| `!ALERTS[,0\|1]` | Re-enable normal alerts while charging |
| `!CAP[,<ms>]` | Capture the ADC across a bridge toggle, then dump it |
| `!SLEEP[,0]` | Arm or disarm the low-power timeout |
| `!SLEEPTEST` | Run one sleeping-mode probe now and report its decision |
| `!SLEEPLOG[,0]` | Dump, or clear, probes made while asleep |

`!CAP` is the most useful failure-analysis tool on this list: it returns the raw ADC samples across a bridge toggle, which is exactly the waveform the detection decision is made from.

### Response formats

| Response | Meaning |
|---|---|
| `$STATUS,...` | Status summary — see below |
| `$CFG,<key>,<value>` | One config value |
| `$CFGEND` | End of a `!CFG` dump |
| `$SN,<value>` | Serial number, empty if unassigned |
| `$OK,<what>` | Command acknowledged |
| `$ERR,<what>[,detail]` | Command failed |
| `$DIAG,<ms>,<rawPos>,<rawNeg>,<posV>,<negV>,<diffV>` | Streaming sample |
| `$CAPSTART,...` / `$CAP,...` / `$CAPEND` | Capture header, rows, terminator |

### `$STATUS` fields used by this protocol

| Field | Meaning | Test |
|---|---|---|
| `hwrev` | Hardware revision the firmware believes it is on | 1.5.3 |
| `dip` | Active threshold slot; equals THRESHSEL on hardware revision 3 | 1.5.4 |
| `sn` | Unit serial number, empty until assigned | 1.5.5, 2.3.2 |
| `charge` | 1 when USB power is detected | 1.8.1 |
| `battv` | Measured battery voltage | 1.9.1 |
| `battpct` | Battery percentage | Recorded |
| `mosfet` | Current bridge hold state | 6.6a |
| `metric`, `retms` | Last detection metric and return time | Recorded |
| `floor` | Active `!FLOOR` mode | 1.10.2 |

> `$DIAG` reports `negV` from A1. On hardware revision 3 **A1 is not connected**, so `negV` carries no information and must not be used as a limit. Use `diffV`, `metric` and `retms`.

---

## Appendix B — Net and test-point map

| Net | Access | Notes |
|---|---|---|
| `V_Input_REF` | J1 | Probe input. Feeds the R10–R6 ladder, 500 kΩ total, into the reference node. |
| `V_Input_Float` | J2 | Probe input. Feeds the R15–R11 ladder, 500 kΩ total, into the sense node. |
| `ADC_Signal` | A2 on the XIAO | Sense node. Clamped by D1 to ground and D2 to +3V3. |
| `Bridge_CTRL` | D7 on the XIAO | Q1 gate, pulled down by R4 100 kΩ. High equals bridge resting. |
| `BatteryRail` | J4 pin 2 | Also feeds LED1 VDD directly — LED1 cannot be gated off in firmware on this revision. |
| `Gnd` | J4 pin 1 | |
| VBus sense | A3 on the XIAO | R20/R21 divider, Vbus/2. |
| LED data | D6 on the XIAO | SK6812 side-view, LED1. |
| Buzzer | D8, D9 on the XIAO | BZ1 across both pins, driven anti-phase. |
| — | J5 | See Section 11, item 8. |

**Signal path, probe to probe:**

```
J1 ──[R10 R9 R8 R7 R6]── reference node ──┬──[R5 100k]──[Q1]──┐
     500k total                           │                    ├── ADC_Signal (A2)
                                          └──────[R22 1M]──────┘        │
                                                                        │
J2 ──[R15 R14 R13 R12 R11]──────────────────────────────────────────────┘
     500k total
```

- Q1 **on**: R5 parallels R22 → 90.9 kΩ, so J1 to J2 is 500k + 90.9k + 500k ≈ **1.09 MΩ**.
- Q1 **off**: only R22 bridges, so J1 to J2 is 500k + 1M + 500k = **2.00 MΩ**.

That difference is what tests 1.6.1 and 1.6.2 measure, and it isolates Q1 and R5 from the rest of the front end.

---

## Appendix C — Revision history

| Rev | Date | Change |
|---|---|---|
| A | 2026-08-24 | First issue, for quotation |
