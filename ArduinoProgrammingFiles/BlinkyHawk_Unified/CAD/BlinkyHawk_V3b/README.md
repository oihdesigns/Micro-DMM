# BlinkyHawk V3b enclosure — 29 September 2026

Use **BlinkyHawk_V3b_Shell.3mf** and **BlinkyHawk_V3b_Lid.3mf** for the first fit print. Each imports as one manifold part in PrusaSlicer. Equivalent STL files are included. Import in millimeters, at 100% scale, with the flat underside on the bed.

**BlinkyHawk_V3b_Lid_Multipart.3mf** retains the original 15 bodies for separate lettering/panel colors. Import its bodies together as parts of one object, preserving their relative positions. It has no assigned filament colors or printer profile. Use the ordinary lid 3MF for a single-material print.

## Configuration

- PCB: `PCBDesigns/BlinkyHawk_V3b/BlinkyHawk_V3b.kicad_pcb`, as supplied.
- XIAO soldered directly to the PCB, confirmed by Nick.
- Existing battery and external toggle switch retained, confirmed by Nick.
- Based on the two supplied V3 3MF files. The older PCB and assembly STL exports establish the PCB position in the existing shell.
- These are revised printable meshes; the original files and native SolidWorks feature models have not been modified.

## Changes

| Feature | Revision |
|---|---|
| PCB length | 51.943 → 53.340 mm; width remains 25.400 mm |
| Shell and lid length | 76.000 → 77.397 mm, added only at the USB end |
| Width and height | Shell 31 × 77.397 × 16 mm; lid 31 × 77.397 × 2.5 mm including lettering |
| USB-end lid screws/posts | Both move 1.397 mm with the end wall; diameters preserved |
| USB aperture | Existing profile moves +0.2386 mm across the width and +0.186 mm upward, matching the modeled connector |
| J4 battery solder relief | Rounded 3.5 × 4.0 mm recess, 0.8 mm deep, below the relocated connection |
| J5 underside wire route | 2.4 mm wide, 0.8 mm deep, from the backside J5 pad to the switch compartment; 1.2 mm of floor remains |
| Existing features | Switch end, LED opening, lead exit, interior stop, front screws, and lettering geometry retained |

The J5 route provides about **1.305 mm** from the recessed floor to the nominal PCB bottom copper. It is intended for a small insulated wire around **1.0 mm outside diameter or less**, laid flat. It is not a verified model of the actual harness.

## Fit checks completed

- Original shell versus V3b PCB: approximately **43.39 mm³** of solid overlap at the USB end. Revised shell versus PCB: **zero** modeled overlap.
- Imported PCB/component geometry: **72 groups** checked against the revised shell. Conservative bounding boxes clear except the USB housing, which was checked against its actual closed mesh. All checks pass.
- USB aperture minimum radial clearance at the inner wall: approximately **0.196 mm**. The existing close-fit allowance is preserved.
- PCB USB-end clearance: **0.200 mm**. PCB side clearance: **0.500 mm per side**. The pre-existing clearance at the lower stop is retained.
- Modeled J4 pin tip to relief floor: **0.645 mm**.
- Four lid/shell screw centers agree within **0.001 mm**; shell and seated lid have zero solid overlap.
- Main shell and lid STL/3MF meshes are watertight, consistently oriented, positive-volume, connected solids. PrusaSlicer confirms one manifold part for each main 3MF.
- Geometry forward of the USB-end extension, above the floor relief, matches the original shell.

These are **model-based checks, not a physical fit test**. The PCB model has some approximate component models and does not include the real battery, loose wires, solder fillets, or USB cable overmold. Check those with the first printed pair before printing a batch. The narrow USB clearance is inherited from the original design; use the same printer compensation that worked for V3.

`Validation.json` contains the measured checks. `Preview.png` shows the revised parts with the PCB; preview colors are illustrative. Measurements and reproducible mesh-edit scripts remain in the workspace's `CAD/BlinkyHawk_V3b/work` folder.
