# Micro-DMM FT232H Desktop Port

This directory now contains a standalone desktop implementation of the
Micro-DMM firmware so you can operate the instrument from a Windows computer
through an FT232H breakout.  The application reproduces the measurement logic
from the original Arduino Giga sketch and exposes a Tkinter GUI with virtual
controls and status indicators.

## Requirements

Install the Adafruit Blinka stack (which provides the ``board``/``busio``/``digitalio``
APIs) together with the ADS1x15 driver on your PC:

```bash
pip install adafruit-blinka adafruit-circuitpython-ads1x15
```

The FT232H must be configured with Blinka.  Follow Adafruit's
[`FT232H` guide](https://learn.adafruit.com/circuitpython-on-any-computer-with-ft232h)
if you have not already done so.

## Running the application

Simply double-click or execute ``microDMM_FT232H.py``.  No command-line
arguments are needed; the GUI starts immediately and shows live voltage,
resistance and current readings.  The buttons in the interface replace the
hardware buttons and touch targets from the Arduino build.

The program maps the original Arduino pins to the FT232H ``C`` pins as follows:

| Arduino pin | Function                | FT232H pin |
|-------------|------------------------|------------|
| D2          | Constant-current PWM   | C2         |
| D5          | Bridge MOSFET control  | C5         |
| D6          | Continuity/buzzer      | C6         |
| D7          | Range select relay     | C3 *(adjust if wired differently)* |
| D52         | Cycle tracking output  | C7         |

If your wiring differs, edit the ``OUTPUT_MAPPING`` dictionary in
``microDMM_FT232H.py``.

Toggle **Show Advanced Controls** in the GUI to manually pick the ADS1115
sample rate or gain.  Leaving either dropdown on *Automatic* preserves the
original firmware's adaptive behaviour; selecting a specific value locks the
converter to that setting for all measurements.

When the ADS1115 or FT232H hardware is not present the GUI still opens and
shows an error banner so you can diagnose connection problems without the
application crashing.

