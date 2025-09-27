# Micro-DMM Raspberry Pi Runtime

This folder contains the Python port of the Micro-DMM firmware that runs on a Raspberry Pi paired with an ADS1256 measurement HAT.  The code is split into three pieces:

* `ads1256_backend.py` – SPI driver that reproduces the Arduino auto-ranging logic using the 24‑bit ADS1256.【F:rpi_port/ads1256_backend.py†L1-L401】
* `app.py` – Tkinter-based desktop UI that mirrors the display from the original Giga sketch and provides logging actions.【F:rpi_port/app.py†L1-L308】
* `run_pi.py` – glue script that instantiates the backend, UI, and optional GPIO helpers so the system can be launched directly on the Pi.【F:rpi_port/run_pi.py†L1-L236】

## 1. Prepare the Raspberry Pi

1. Enable SPI via `sudo raspi-config` → *Interface Options* → *SPI* → *Enable*.
2. Make sure the ADS1256 HAT is wired for SPI bus 0 (SCLK/MISO/MOSI/CS) and expose the **DRDY** pin on a free GPIO (recommended BCM 22).
3. Connect the digital range-select MOSFET, optional cycle-tracking pin, and any physical buttons to free GPIOs.  BCM numbering is assumed throughout.【F:rpi_port/HARDWARE_BEHAVIOR.md†L7-L38】
4. Install the OS packages required for GUI support: `sudo apt install python3-tk python3-gpiozero`.

## 2. Install the Python environment

```bash
python3 -m venv ~/microdmm-pi
source ~/microdmm-pi/bin/activate
pip install --upgrade pip
pip install -r requirements.txt  # see below for the minimal list
```

If you prefer manual installation, the code depends on:

```bash
pip install gpiozero spidev adafruit-circuitpython-busdevice adafruit-circuitpython-ads1x15
```

Tkinter ships with the `python3-tk` package mentioned earlier.

## 3. Launch the application

From the repository root (or after adding `/Micro-DMM` to `PYTHONPATH`):

```bash
python3 -m rpi_port.run_pi \
    --drdy-pin 22 \
    --range-pin 23 \
    --cycle-pin 24 \
    --gpio-button mode=5 --gpio-button manual_log=6
```

If you prefer a single-file style invocation, the launcher now also works when
executed directly:

```bash
python3 rpi_port/run_pi.py --drdy-pin 22 --range-pin 23
```

Both commands expect the `rpi_port` folder to stay intact, since the Tk UI,
ADS1256 backend, and logging helpers live alongside `run_pi.py`.

The options are all optional:

* `--drdy-pin` hooks the ADS1256 **DRDY** line so conversions are synchronised.【F:rpi_port/run_pi.py†L107-L133】
* `--range-pin` toggles the high/low resistance MOSFET by calling back into the backend whenever auto-ranging switches state.【F:rpi_port/run_pi.py†L35-L58】【F:rpi_port/run_pi.py†L180-L189】
* `--cycle-pin` pulses a GPIO on every sample so you can watch the main loop on a scope.【F:rpi_port/run_pi.py†L60-L88】【F:rpi_port/run_pi.py†L155-L166】
* `--gpio-button` attaches physical buttons to UI actions (`mode`, `manual_log`, `auto_log`, `minmax`, or `type`).【F:rpi_port/app.py†L84-L124】【F:rpi_port/run_pi.py†L205-L214】
* `--no-current` disables current sampling if you do not have the hall/low-range front-ends wired.【F:rpi_port/run_pi.py†L215-L238】
* `--show-resistance` flips the primary readout to ohms instead of volts.【F:rpi_port/run_pi.py†L215-L238】【F:rpi_port/app.py†L120-L161】

The script spawns a background thread that continuously polls the ADS1256 backend and feeds the Tk UI with calibrated voltage, resistance, and current readings.【F:rpi_port/run_pi.py†L90-L170】  Logging controls mirror the original firmware: pressing the UI buttons (or their GPIO equivalents) writes CSV files under `~/micro_dmm_logs` unless you override the directory via `--log-dir`.【F:rpi_port/logging.py†L1-L33】【F:rpi_port/app.py†L129-L188】

## 4. Shutting down

The program installs SIGINT/SIGTERM handlers so `Ctrl+C` or a systemd stop cleanly closes the ADS1256 SPI device, releases GPIO resources, and exits the Tk main loop.【F:rpi_port/run_pi.py†L190-L238】

## 5. Troubleshooting tips

* Run with `--log-level DEBUG` to see per-sample telemetry and any auto-ranging decisions written to stderr.【F:rpi_port/run_pi.py†L200-L238】
* If the UI stays on “Waiting for data…”, double-check SPI wiring and that the ADS1256 is powered; the backend logs a stack trace whenever sampling fails.【F:rpi_port/run_pi.py†L140-L170】
* For headless deployments you can forward the Tk display over VNC or X11, or replace the UI by importing `ADS1256Backend` in your own script and bypassing `MicroDmmApp` entirely.【F:rpi_port/ads1256_backend.py†L113-L401】【F:rpi_port/app.py†L1-L308】
