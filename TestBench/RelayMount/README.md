# Relay and Arduino mounting plate

Two relay boards side by side across the top; the UNO centered below, USB and barrel power facing left, matching the supplied photograph. All model units are millimetres.

## Files

- `Relay_Arduino_Mount.stl` - printable, watertight single-part mesh. Import into the slicer in millimetres at 100% scale.
- `Relay_Arduino_Mount.step` - exact solid CAD for Fusion, FreeCAD, SolidWorks, and other CAD programs.
- `Relay_Arduino_Mount.scad` - editable parametric OpenSCAD source. Optional board envelopes are preview-only.
- `Mount_preview.png` - actual model render and annotated placement drawing.
- `hole_coordinates_mm.csv` - all twelve hole centers measured from the base's lower-left corner, viewed from above.
- `validation.json` - exact-solid and exported-mesh validation results.
- `source/build_mount.py` - parametric CadQuery source that regenerates STEP, STL, OpenSCAD, coordinates, and validation results.

## Dimensions

| Feature | Dimension |
| --- | --- |
| Base outline | 165.1 x 139.7 mm (6.5 x 5.5 inches) |
| Base thickness | 3 mm |
| Standoffs | 12, each 2 mm tall and 6 mm diameter |
| Overall printed height | 5 mm |
| Screw holes | 2 mm diameter, through the standoffs and base |
| Outer corner radius | 3 mm |
| Relay hole spacing | 66.7 mm horizontal x 45 mm vertical, each board |
| Relay lower-left holes | (9.85, 80.00) and (88.55, 80.00) mm |
| UNO PCB reference origin | (48.26, 14.00) mm |

## Arduino mounting pattern and sources

The photographed board appears to be a Freenove Control Board V5 WiFi (an UNO R4 equivalent). The model uses the **standard Arduino UNO four-hole pattern**, with USB/power connectors on the left. The manufacturer describes the Freenove V5 as UNO R4 compatible; a separate dimensioned Freenove hole drawing was not found. The photo is consistent with the standard pattern, but is not a metrology reference. Confirm the four holes on your board before a full print.

Coordinates relative to the lower-left corner of the standard UNO PCB outline:

| Hole | X (mm) | Y (mm) |
| --- | ---: | ---: |
| Lower left | 13.97 | 2.54 |
| Lower right | 66.04 | 7.62 |
| Upper right | 66.04 | 35.56 |
| Upper left | 15.24 | 50.80 |

The pattern is asymmetric. The right holes are 27.94 mm apart; the left holes differ by 1.27 mm horizontally and 48.26 mm vertically. Do not substitute a rectangular pattern.

Verified against the official Arduino UNO R4 WiFi datasheet, section 13 (page 20 of the downloaded edition), and the official CAD archive's `PCB-RoundHoles.TXT`, tool T08. The drill coordinates are (0.55,0.10), (2.60,0.30), (2.60,1.40), and (0.60,2.00) inches.

- Arduino board page and CAD downloads: https://docs.arduino.cc/hardware/uno-r4-wifi/
- Arduino mechanical drawing: https://docs.arduino.cc/resources/datasheets/ABX00087-datasheet.pdf
- Freenove manufacturer listing: https://store.freenove.com/products/fnk0096

Arduino's PCB holes are 3.2 mm diameter; this printed base intentionally has **2 mm holes**, as requested. Relay spacing is the user-specified center-to-center spacing. Board outlines in the preview are approximate placement guides, not printed geometry.

## Printing and fit

Print flat underside down, standoffs upward. No supports are needed. Keep 100% scale; do not resize to fit the bed. The 2 mm holes are nominal CAD diameters without printer compensation and may need clearing after printing. The underside clearance is exactly 2 mm as requested: check protruding solder leads on the actual boards before tightening screws.

The model includes no additional chassis holes, countersinks, or printed threads. Screw length depends on the board thickness and chosen fastening method.
