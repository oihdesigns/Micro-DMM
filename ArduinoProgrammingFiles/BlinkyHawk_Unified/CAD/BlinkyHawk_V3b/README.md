# BlinkyHawk V3b enclosure — 29 September 2026

Use **BlinkyHawk_V3b_Shell.3mf** and **BlinkyHawk_V3b_Lid.3mf** for the first fit print. Each imports as one manifold part in PrusaSlicer. Equivalent STL files are included. Import in millimeters, at 100% scale, with the flat underside on the bed.

**BlinkyHawk_V3b_Lid_Multipart.3mf** retains the original 15 bodies for separate lettering/panel colors. Import its bodies together as parts of one object, preserving their relative positions. It has no assigned filament colors or printer profile. Use the ordinary lid 3MF for a single-material print.

## Slide-switch variant — 2 October 2026

Use **BlinkyHawk_V3b_Shell_SlideSwitch.3mf** (or the equivalent STL) with the existing V3b lid. The original shell files remain available.

- Centered on the long wall opposite the LED (X=0 wall; Y=38.6985 mm).
- Opening: 10.66 mm along the wall × 5.8 mm vertically, with its top edge flush with the Z=16 mm enclosure lip. This forms an open-top notch; the existing lid closes its top.
- Two through-holes: 1.8 mm diameter, 15.0 mm center spacing along the horizontal opening axis, at Z=13.1 mm.
- Exact nominal dimensions, with no added print-fit allowance. Switch depth, flange thickness, terminals, and screw-head clearance were not supplied and have not been checked.
- STL and 3MF verified as one watertight positive-volume solid, with matching volume. Changes are confined to the intended side-wall cuts. Other enclosure features and the lid are preserved.

See `SlideSwitch_Preview.png` and `SlideSwitch_Validation.json`. Reproduce with `work/add_slide_switch.py`. These are revised printable meshes; native SolidWorks feature files are not present in this folder and were not edited.

## Switch above XIAO variant — 3 October 2026

Use **BlinkyHawk_V3b_Shell_SlideSwitch_AboveXIAO.3mf** (or its equivalent STL) with the existing V3b lid. This variant starts from the uncut V3b shell, so it has only the new switch opening. The earlier centered switch version remains available.

The switch stays on the long wall opposite the LED, with its top edge at the enclosure lip. Its center is now Y=64.7552 mm, aligned with the longitudinal center of the imported XIAO board (Y=54.2777–75.2327 mm). This moves the switch 26.0567 mm toward the USB end. The opening remains 10.66 × 5.8 mm; the two mounting holes remain Ø1.8 mm, 15 mm apart, at Z=13.1 mm.

The new shell and 3MF round-trip are one watertight solid. Dimensional section checks confirm both holes, and boolean checks confirm that only the intended cuts changed the shell. Switch depth and actual hardware clearance remain unverified. See `SlideSwitch_AboveXIAO_Preview.png` and `SlideSwitch_AboveXIAO_Validation.json`. Reproduce with `work/add_slide_switch.py --above-xiao`.

## Switch above XIAO, moved inward 5 mm — 3 October 2026

Use **BlinkyHawk_V3b_Shell_SlideSwitch_AboveXIAO_Inward5mm.3mf** (or its equivalent STL) with the existing lid. The switch center is Y=59.7552 mm: 5 mm toward the enclosure middle from the previous above-XIAO position, away from the USB-end lid screw mount. Opening dimensions, top-lip alignment, and mounting holes are unchanged. Earlier versions remain available; this shell has only the relocated opening and holes.

Both exports pass watertight single-solid and dimensional checks. The shift increases longitudinal separation from the end screw mount by 5 mm; actual switch flange, screw-head, and body clearance still require a fit check. See `SlideSwitch_AboveXIAO_Inward5mm_Preview.png` and `SlideSwitch_AboveXIAO_Inward5mm_Validation.json`. Reproduce with `work/add_slide_switch.py --above-xiao --shift-inward-5mm`.

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
