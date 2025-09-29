# Micro-DMM Giga Hardware Control Audit

The table below captures how each hardware control defined in `microDMM_Giga.ino` is used, the electrical behavior that the firmware expects, and what measurements are taken from the auxiliary ADC channels. Line numbers refer to the original Arduino sketch for traceability.

| Identifier | Direction | Purpose in firmware | Expected electrical behavior |
| --- | --- | --- | --- |
| `SETRANGE_PIN` (D7) | Digital output | Selects the high/low ohms range before sampling resistance. | High level engages the high-resistance divider (±22 kΩ path) and becomes the default in power-save; low level enables the constant-current low range. The sketch toggles the pin when thresholds are crossed (`digitalWrite(SETRANGE_PIN, HIGH/LOW)`), forcing high in power-save (`powerSave`), and samples the initial state with `digitalRead` to seed the auto-range logic.【F:ArduinoProgrammingFiles/microDMM_Giga/microDMM_Giga.ino†L1343-L1377】 |
| `OHMPWMPIN` (D2) | PWM output | Drives the constant-current source MOSFET used for low-ohms measurements and power saving. | The sketch writes 8‑bit PWM levels: `0` keeps the current source active during measurement and zeroing, while `254` (~99.6 % duty at 3.3 V) parks the MOSFET off during charging or power-save. Auto-zero also forces `0` before sampling.【F:ArduinoProgrammingFiles/microDMM_Giga/microDMM_Giga.ino†L824-L833】【F:ArduinoProgrammingFiles/microDMM_Giga/microDMM_Giga.ino†L1421-L1433】 |
| `CONTINUITY_PIN` (D6) | PWM output | Buzzer/LED drive for alerts, flashlight dimming, and logging cues. | PWM values span 0–255 for brightness: alerts pulse around 50–200, flashlight off uses `0`. Duty changes rapidly for beeps, so the driver must support kHz-class PWM updates without audible artifacts.【F:ArduinoProgrammingFiles/microDMM_Giga/microDMM_Giga.ino†L745-L764】【F:ArduinoProgrammingFiles/microDMM_Giga/microDMM_Giga.ino†L2076-L2106】 |
| `VbridgePin` (D5) | Digital output | Controls the voltage-bridge MOSFET that checks whether the probes are floating. | Pulled high only during the `ClosedOrFloat()` check to energize the bridge, then returned low immediately after sampling the differential ADS1115 channel.【F:ArduinoProgrammingFiles/microDMM_Giga/microDMM_Giga.ino†L2463-L2483】 |
| `cycleTrack` (D52) | Digital output | Logic analyzer heartbeat to time the main loop. | Toggles high/low every loop iteration, yielding a ~50 % duty diagnostic square wave so external tools can measure firmware cycle time.【F:ArduinoProgrammingFiles/microDMM_Giga/microDMM_Giga.ino†L594-L603】 |
| `TYPE_PIN` (D3) | Digital input w/ pull-up | Detects the physical mode button (held at boot for flashlight mode). | Read with `INPUT_PULLUP`; LOW at boot latches flashlight mode, and LOW pulses later trigger mode cycling together with microphone gestures.【F:ArduinoProgrammingFiles/microDMM_Giga/microDMM_Giga.ino†L474-L483】【F:ArduinoProgrammingFiles/microDMM_Giga/microDMM_Giga.ino†L2230-L2249】 |
| `BUTTON_PIN` (D4) | Digital input w/ pull-up | Front panel mode button. | Sampled with debounce timing derived from `millis()`; a LOW transition within 300 ms of stability advances the mode state machine.【F:ArduinoProgrammingFiles/microDMM_Giga/microDMM_Giga.ino†L2230-L2249】 |
| `logButton` (A1) | Digital input w/ pull-up | External “log” trigger. | LOW latches a logging session; requires a pulled-up idle state to avoid phantom sampling.【F:ArduinoProgrammingFiles/microDMM_Giga/microDMM_Giga.ino†L474-L483】【F:ArduinoProgrammingFiles/microDMM_Giga/microDMM_Giga.ino†L745-L753】 |
| `kbButton` (D8) | Digital input | Selects USB keyboard vs. serial streaming. | Interpreted as a simple HIGH/LOW toggle per the firmware’s inline documentation; hardware must present a defined logic level (pull-up recommended).【F:ArduinoProgrammingFiles/microDMM_Giga/microDMM_Giga.ino†L82-L90】 |
| `micPin` (A7) | Analog input | Reads the exposed capacitive microphone pad that doubles as a gesture key. | ADC counts of 0–60 request delta/typing features, 100–275 toggles min/max display, and 300–450 acts like the physical mode button. Requires a fast ADC capable of resolving ~12-bit levels around the 0–500 count span without heavy filtering.【F:ArduinoProgrammingFiles/microDMM_Giga/microDMM_Giga.ino†L667-L700】【F:ArduinoProgrammingFiles/microDMM_Giga/microDMM_Giga.ino†L2230-L2249】 |

## ADC behaviour

* **ADS1115 channels:** The sketch samples channel 2 single-ended for ohms voltage, adjusts gain dynamically, and uses the differential 0–1 pair during `ClosedOrFloat()` to sense the bridge voltage. Gains span from ±6.144 V down to ±0.256 V; switching thresholds are 10 k/30 k counts to avoid saturation.【F:ArduinoProgrammingFiles/microDMM_Giga/microDMM_Giga.ino†L1310-L1398】【F:ArduinoProgrammingFiles/microDMM_Giga/microDMM_Giga.ino†L2463-L2478】
* **Microphone pad:** The raw counts are read with `analogRead(micPin)` and compared directly against the gesture thresholds listed above, implying the default STM32 12-bit ADC scale. No averaging is performed, so downstream hardware must deliver similar instantaneous readings.【F:ArduinoProgrammingFiles/microDMM_Giga/microDMM_Giga.ino†L667-L700】

## Timing expectations

The firmware’s debounce and scheduler loops rely on millisecond-resolution monotonic timing. Key intervals include:

* ADC loop: 1 ms cadence (`ADC_INTERVAL`).
* Debounce: 300 ms guard (`DEBOUNCE_DELAY`).
* Power-save watchdog: 5 s of sustained high ohms voltage before parking the constant-current PWM high.【F:ArduinoProgrammingFiles/microDMM_Giga/microDMM_Giga.ino†L800-L833】【F:ArduinoProgrammingFiles/microDMM_Giga/microDMM_Giga.ino†L2230-L2249】

Replicating these behaviors on Linux requires monotonic millisecond counters and PWM/ADC primitives that match the duty-cycle and sampling expectations above.
