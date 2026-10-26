# Three-piece board enclosure

Current printable parts (millimeters):

1. `base_platform.stl`: existing 3 mm base with eight 4 mm M2 board standoffs.
2. `wall_frame.stl`: separate 25.4 mm tall wall frame, 2 mm walls, with screw posts at both ends.
3. `top_plate.stl`: separate 2 mm top, with four 3.2 mm M3 clearance holes.

All three share a 134.9 x 108 mm outside footprint and 1 mm rounded vertical corners. Straight top and bottom perimeter edges are square. Assembled height is 30.4 mm, excluding screw heads. Interior height above the base is 25.4 mm, including the board standoffs.

## Assembly and screws

The top and base screw independently into the wall frame. Four 6 mm diameter corner posts have 2.5 mm diameter, 10 mm deep blind pilots from each end. A 5.4 mm solid section separates opposing pilots. Tap M3 before assembly; adjust the pilot diameter for your printer as needed. These are pilot holes, not modeled threads or insert pockets.

Use four M3 x 10 mm screws through the 3 mm base (7 mm engagement), and four M3 x 8 mm screws through the 2 mm top (6 mm engagement). The clearance holes are not countersunk; heads project outside the plates. Screw-center pattern is 126.9 x 100 mm, with centers 4 mm from adjacent outer edges. The joints are flat with no locating lip or seal.

## Wire openings

Four rectangular edge notches open toward the base. Attaching the base closes that edge, forming rectangular wire openings. Every notch is 12.7 mm high, half the 25.4 mm wall height. Widths and approximate locations match the previous version and the reference photo:

- Photo bottom-left: 7 mm wide, center X=18 mm on the bottom wall.
- Photo bottom-right: 32 mm wide, center X=108.5 mm on the bottom wall.
- Both upper sides: 15 mm wide, centers Y=85 mm on the left and right walls.

Coordinates above use the assembled base layout, with origin at the lower-left corner of the photo. The frame STL is flipped for printing: top mating face on bed, base mating face up. The notches therefore open upward during printing, avoiding bridging across them. Print both plates flat and the base standoffs up.

## Board layout

Sensor hole centers: 31.5 mm horizontally x 61 mm vertically. Relay hole centers: 66.7 mm horizontally x 45 mm vertically. Standoffs are 5.5 mm diameter, 4 mm tall, with 1.7 mm M2 pilot holes, 5 mm deep. A 2 mm solid floor remains under them.

Actual PCB outside dimensions were not supplied. The model assumes edges 3 mm beyond mounting-hole centers: sensor 37.5 x 67 mm; relay 72.7 x 51 mm. Both lower edges align, with a 12.7 mm gap between assumed outlines, 45 mm free area above the relay and 6 mm outer layout margin. Confirm actual PCB outlines and component heights for final fit.

## Sources and validation

- `three_piece_enclosure.scad`: wall, top or assembled view via the `part` parameter.
- `wall_frame.scad`, `top_plate.scad`: individual part entry points.
- `base_platform.scad`: editable existing base.
- `three_piece_enclosure.blend`: exploded editable assembly.
- `build_three_piece.py`: regenerate wall/top STLs and previews in Blender 3.6. `build_shell.py` now calls it.
- `cover_shell.scad`: compatibility entry point to the three-piece source.

The former `cover_shell.stl`, `cover_shell.blend` and `cover_shell_preview.png` represent the obsolete two-piece design; do not print that shell for the current assembly. Copies with `_legacy` suffix preserve the previous source and mesh.

Wall and top pass closed-manifold, positive-volume and single-connected-solid checks. Exported heights are verified. Corner posts clear the existing board standoffs by more than 1 mm; screw engagement stays within each pilot. Actual printed fit and component clearance still require checking.

This remains a mechanical fit draft, not a verified mains-voltage enclosure. Material, electrical clearances and wire retention need to be addressed in the completed enclosure.

## Feather Doubler lid extension

The current top_plate.stl includes a flat trapezoid extension, centered on the long edge at assembled Y=108 mm. It is coplanar with the lid: 2 mm thick, projects 56 mm, is 100 mm wide at its root and 62 mm wide at its flat outer end. Overall lid footprint is 134.9 x 164 mm. Print flat with the eight standoffs upward.

The Adafruit FeatherWing Doubler (#2890) mounts parallel to the lid, entirely beyond the enclosure edge. PCB origin is X=42.05, Y=112 mm, giving a 4 mm board-to-enclosure edge gap. Eight 5.5 mm diameter standoffs are 4 mm above the lid, with 1.7 mm diameter M2 blind pilots, 4 mm deep; the 2 mm lid floor remains intact. M2 x 5 mm screws give about 3.4 mm engagement through the 1.6 mm PCB. Tap or use appropriate plastic screws and verify printed pilot fit.

Mounting centers follow Adafruit's Eagle CAD, not photo scaling: PCB-relative X=2.54 and 48.26 mm, Y=2.54, 20.32, 26.67 and 44.45 mm. Outer pattern is 45.72 x 41.91 mm, with intermediate rows for both Feather footprints. CAD outline is 50.8 x 46.99 mm. The board's long dimension runs along the lid edge, as in the supplied photos.

Source: https://github.com/adafruit/Adafruit-FeatherWing-Proto-Doubler-Tripler-and-Quad/blob/master/DoubleWing.brd
Product: https://www.adafruit.com/product/2890

The lid's original four M3 attachment holes, base and wall geometry are retained. Lid mesh passes manifold, positive volume, single-connected-solid and 6 mm total-height checks. top_plate_before_feather.stl and three_piece_before_feather.scad preserve the preceding lid design.
