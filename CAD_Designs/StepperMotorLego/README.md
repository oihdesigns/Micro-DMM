# Horizontal stepper motor to LEGO stud mount — prototype 1

All dimensions are millimeters. The motor mounts behind the upright face; the
shaft points horizontally out through the front. Attach using four screws
through the bracket into the motor. The rotating part is not clamped.

## Geometry

| Feature | Dimension |
|---|---:|
| Motor body supplied | 28 x 28 |
| Screw centers | 23.5 x 23.5 |
| Screw clearance holes | 2.4 diameter |
| Rotating part supplied | 18.5 diameter |
| Through-opening | 22.5 diameter (enlarged to clear motor lip) |
| Base footprint | 47.8 x 47.8 (6 x 6 studs) |
| Base thickness | 5 |
| Upright thickness | 4 |
| Overall height | 41 |
| Shaft center above LEGO mating surface | 24 |
| Underside socket grid | 8 pitch, 36 sockets |
| Nominal socket diameter / depth | 4.90 / 2.10 |

Two 3 mm side braces sit outside the 28 mm motor body. Motor bottom is 10 mm
above the LEGO mating surface, leaving 5 mm above the base. The rear is open
for longer motors and wires. Check connector clearance against your motor.

## First print

Print one Fit_Test STL first, flat face on the build plate and socket openings
facing UP. Coupons have 4.70 through 5.20 mm socket diameters in 0.10 mm steps;
each is a 2 x 2 pattern. The user confirmed Fit_Test_4p90 fits best; the mount now uses that same 4.90 mm socket diameter. Fit is printer/material dependent.
Use a larger diameter if tight, smaller if loose. Do not force a tight coupon
onto LEGO. The sockets are individual round recesses rather than LEGO-style
underside tubes; this is an experimental connection intended for iteration.

For the full mount, place the FRONT of the upright plate on the print bed
(the upright's outward normal is -Y in the CAD files). The base then stands
vertically, with sockets facing sideways. This avoids supports inside blind
sockets and inside the large circular opening. Use 0.16–0.20 mm layers and
4–5 walls. Inspect the slicer preview for the side-facing socket roofs; localized
support may be useful there. PETG is a reasonable starting material. Check the
motor's operating temperature against the chosen filament's heat resistance.

The supplied 2 mm motor holes do not establish their thread specification.
2.4 mm bracket holes assume M2 screws; confirm the motor actually takes M2.
Select screw length as 4 mm bracket thickness plus the motor manufacturer's
allowed thread engagement (plus any washer). Do not bottom screws in the motor.
No counterbores are used; screw heads remain accessible on the front.

## Files and changes

- Stepper_LEGO_Mount.stl: printable full bracket.
- Stepper_LEGO_Mount.step: CAD exchange solid.
- Stepper_LEGO_Mount.FCStd: FreeCAD solid (not a sketch-based feature tree).
- Fit_Test_*.stl: small socket-fit coupons; STEP/FCStd copies also included.
- build_mount.py: editable dimension parameters and generation source; run with
  FreeCAD's bundled Python to regenerate all files.
- validation.json: solid and mesh checks; these do not prove physical fit.

After a coupon test, change SOCKET_D in build_mount.py and regenerate the mount.
Dimensions are based on your supplied motor measurements. This prototype has
not been physically printed or load tested. Keep external shaft loads modest;
stud grip and the motor shaft bearings will limit the assembly.
