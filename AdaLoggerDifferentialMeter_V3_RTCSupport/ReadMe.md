# Adalogger-Based Datalogger

A differential-voltage datalogger built around an Adalogger platform, supporting autoranging, dual-channel acquisition, SD card logging, and optional serial output. Configuration is handled via onboard DIP switches and is largely fixed at boot.

---

## Features

- Differential voltage measurement
- Autoranging for improved low-voltage accuracy
- Single- or dual-channel logging
- microSD card data logging (CSV)
- Serial output (optional)
- Battery voltage monitoring
- Temperature sensing (relative)
- Isolated analog input stage
- External trigger threshold adjustment
- Python-based data plotting (beta)

---

## Configuration

### DIP Switch Settings

> ⚠️ **Important:** Switches marked *boot-only* are read only at startup. Power-cycle the board after changing them.

---

### **DIP Switch 1 — Autorange** *(boot-only)*

- **UP:** Autorange **Disabled**
- **DOWN:** Autorange **Enabled**

Autorange improves measurement accuracy for voltages below approximately **5 VDC**, at the cost of reduced logging speed.

| Mode | Minimum Sample Interval |
|-----|--------------------------|
| Autorange Disabled | ~0.5 ms |
| Autorange Enabled | ~2.2 ms |

**Recommendation:**  
Leave autorange disabled unless improved accuracy below ~5 VDC is required.

---

### **DIP Switch 2 — Channel 2 Enable (CH2)** *(boot-only)*

- **UP:** CH2 **Disabled**
- **DOWN:** CH2 **Enabled**

Enables the second differential input pair.

- **CH1 only:** up to ~0.5 ms between samples
- **CH1 + CH2:** sample interval increases to several milliseconds
- When both channels are enabled, the input range is **fixed** (autorange disabled)

---

### **DIP Switch 3 — Serial Output**

- **UP:** Serial **Disabled**
- **DOWN:** Serial **Enabled**

Enables or suppresses serial status updates.  
Useful for reducing serial traffic or running headless.

---

### **DIP Switch 4 — SD Card Usage** *(boot-only)*

- **UP:** SD **Disabled**
- **DOWN:** SD **Enabled**

Disables microSD usage entirely.

Useful for:
- Streaming data only
- Debugging
- Verifying acquisition timing without SD write overhead

---

## Operating Notes

### Voltage Range

- Approximate input range: **–40 V to +40 V** (as of current implementation)

---

### Mark Button

- Pressing the onboard button writes the string `"mark"` into the current row of the CSV file
- Useful for correlating external events with recorded data

---

### Temperature Sensor

- Intended for **relative temperature change**, not absolute accuracy
- Currently mounted near the PCB
- Planned improvement: remote placement via a short cable to measure a point of interest

---

### Battery Monitoring

- Battery voltage is logged
- Approximate thresholds:
  - **Full:** ~4.2 V
  - **Low:** ~3.6 V

---

### Log File Creation

- A new CSV file is created on the SD card at boot
- Filename is based on the **date and time at power-on**

---

### Trigger Threshold Adjustment

- Controlled via a potentiometer connected to **A0**
- Implemented as an **exponential response**
- Maximum threshold: ~8 VDC
- Typical useful setting: ~0.25 VDC

---

## Python Plotting Script

- A Python plotting script is included on the SD card
- Edit the file paths at the **bottom of the script** to select which CSV files to view
- Run the script to generate plots

⚠️ **Status:**  
The plotting script is **very much in beta** and subject to change.

---

## SD Card Status Indicator

- The display indication for SD card status (OK / error) is **not fully validated**
- Do not rely on this indicator alone for data integrity verification

---

## Isolation & Powering Notes

- The analog input stage is **isolated from the rest of the board**
- This allows:
  - Powering the logger from vehicle power
  - Measuring vehicle power with minimal noise coupling
- For best results and a shared timebase, powering both the logger and measured system from a **battery** is recommended

---

## Known Limitations

- SD card status display not fully proven
- Python plotting script is experimental
- Temperature sensor not intended for precision measurements
- Autorange and channel configuration require power cycling

---

## License

[Specify license here]

---

## Future Improvements (Planned)

- Remote temperature probe
- Improved SD card fault detection
- Expanded plotting features
- Documentation of fixed-range behavior when dual-channel mode is enabled
