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

For the optional plotting and CSV logging tools install Matplotlib:

```bash
pip install matplotlib
```

## Running the application

Simply double-click or execute ``microDMM_FT232H.py``.  No command-line
arguments are needed; the GUI starts immediately and shows live voltage,
resistance and (optionally) current readings.  The buttons in the interface
replace the hardware buttons and touch targets from the Arduino build.  Current
measurement is disabled at startup to keep the shunt powered down unless you
explicitly enable it from the controls.

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

The **Advanced Controls** panel (visible by default, or hidden via the *Show
Advanced Controls* checkbox) lets you manually pick the ADS1115 sample rate or
gain.  Leaving either dropdown on *Automatic* preserves the original firmware's
adaptive behaviour; selecting a specific value locks the converter to that
setting for all measurements.

The advanced panel also exposes an optional *Manual Pin Control* mode so you can
force the FT232H outputs without modifying the firmware logic.  Enable the
checkbox to unlock toggles for:

* **R circuit (C2)** – the constant-current source drive (Arduino D2)
* **V Bridge (C5)** – the bridge MOSFET gate (Arduino D5)
* **R Range (6)** – the range-select relay (Arduino D7)

When the manual mode is disabled the firmware takes over again immediately and
re-applies its preferred states to the pins.  Adjust the labels in
``MANUAL_PIN_LABELS`` if your FT232H wiring calls for different names.

Use the **Show Bridge Voltage Readout** checkbox (also in the advanced panel) to
reveal the bridge voltage field and its min/max entry on the meter tab, together
with the corresponding logging channel toggle.  Leaving the option off keeps the
main readouts focused on voltage, VAC, resistance and current only.

### Min/max tracking and logging

The main *Meter* tab now keeps running minimum and maximum values for the DC
voltage, RMS voltage, resistance/temperature, current and bridge voltage.  Use
the **Reset Min/Max** button to clear the statistics at any time – the next
reading seeds a fresh set of extrema.

Switch to the *Logging* tab to record and visualise incoming data:

* Enable **Record measurements** to start accumulating samples.  Logging uses
  the same units that are shown in the meter (for example, resistance switches
  to °F when the alternate units toggle is active).
* The Matplotlib chart updates live and lets you toggle individual channels to
  focus on specific signals.  The Y axis rescales automatically to the visible
  traces.
* Click **Clear Log** to discard the captured samples and reset the plot.
* Click **Export CSV** to save the log (timestamp, elapsed time and all channel
  values) for offline analysis.

When the ADS1115 or FT232H hardware is not present the GUI still opens and
shows an error banner so you can diagnose connection problems without the
application crashing.

