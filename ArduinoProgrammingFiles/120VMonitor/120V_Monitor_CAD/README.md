# Three-piece board enclosure

Current printable parts (millimeters):

1. `base_platform.stl`: 3 mm base with twelve 4 mm M2 standoffs, including four Mega/GIGA mounts on its extension.
2. `wall_frame.stl`: separate 25.4 mm tall wall frame, 2 mm walls, with screw posts at both ends.
3. `top_plate.stl`: separate 2 mm top, with four 3.2 mm M3 clearance holes.

The wall frame and plain top retain the 134.9 x 108 mm footprint and 1 mm rounded vertical corners. The base has a coplanar trapezoid extension, giving it a 134.9 x 169.34 mm overall footprint. Straight top and bottom perimeter edges are square. Assembled height is 30.4 mm, excluding screw heads. Interior height above the base is 25.4 mm, including the board standoffs.

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

Actual PCB outside dimensions were not supplied. The model assumes edges 3 mm beyond mounting-hole centers: sensor 37.5 x 67 mm; relay 72.7 x 51 mm. In the original layout both lower edges align; the latest layout rotates both boards together by 180 degrees, so their upper edges align, with a 12.7 mm gap between assumed outlines, 45 mm free area above the relay and 6 mm outer layout margin. Confirm actual PCB outlines and component heights for final fit.

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

## Current revision: Arduino Mega / GIGA on the base

All three boards now mount on the same base surface. All twelve standoffs point in +Z, with their mounting faces 4 mm above the 3 mm base. There are no board mounts or extensions on the lid. The three-piece construction, screw pattern and rectangular wire notches are retained.

The flat base extension projects 61.34 mm from the long edge at Y=108 mm, with a 130 mm root and 114 mm flat end. It is 3 mm thick and supports a standard 101.6 x 53.34 mm Arduino Mega 2560 Rev3 or GIGA R1 WiFi, long axis parallel to the enclosure edge. Board envelope remains X=16.65..118.25, Y=112..165.34 mm, with a 4 mm gap outside the wall frame. The board is rotated 180 degrees in that envelope, placing USB/power toward the right as in the latest photo. The Arduino remains exposed outside the wall frame, as with the previous Feather extension.

Four common outer mounting centers are used (board-relative X,Y in mm): (13.97,2.54), (15.24,50.8), (90.17,50.8), (96.52,2.54). These mounts use the same 5.5 mm diameter, 4 mm tall standoffs and 1.7 mm M2 pilot bores as the other boards. Pilot depth is 5 mm, leaving 2 mm of solid base below. Use M2 screws with suitable engagement, e.g. M2 x 6 through a nominal 1.6 mm PCB (4.4 mm engagement); tune pilot fit to your printer. The internal mounting positions of the Arduino are intentionally unused.

Mechanical references: Arduino GIGA R1 WiFi datasheet, section 15, and Mega 2560 Rev3 datasheet, section 6.2. Local drawing renders: giga_drawing.png and mega_drawing.png.
- https://docs.arduino.cc/resources/datasheets/ABX00063-datasheet.pdf
- https://docs.arduino.cc/resources/datasheets/A000067-datasheet.pdf

Print the base flat with every standoff upward; print the plain top flat. Current source files are base_platform.scad and three_piece_enclosure.scad, with wall_frame.scad and top_plate.scad as entry points. Regenerate the base first with build_base.py, then frame/top/assembly with build_three_piece.py. The preceding Feather revision is backed up in revisions/before_mega_giga.zip.

Verification: revised base is a single connected closed manifold with positive volume; twelve standoffs, all facing +Z; total height 7 mm. Wall and plain top also pass manifold, connected-solid and height checks. Actual board and printed fit still require a physical check.

## Latest layout: GIGA above lower boards

Matches the latest reference photo when viewed from the standoff side: GIGA/MEGA across the top, relay below-left, sensor below-right. Both lower board patterns are rotated together 180 degrees relative to the preceding base, bringing their flush edges next to the GIGA instead of the free-space edge. All twelve 4 mm standoffs still point upward. GIGA connectors face right. Board footprints are shown in base_layout_preview.png; the lower-board outlines retain the earlier assumed edge offsets.

The wall frame is rotated with the lower-board layout, preserving wire-opening alignment. Reprint the revised base; the existing wall can be reused by rotating it 180 degrees in its own plane (not flipping it). The plain lid fits either orientation. The frame STL and editable source have been regenerated in the new orientation. Back-up: revisions/before_giga_above.zip.

Latest assembled wire opening centers: 7 mm opening at X=116.9 on the top wall (Y=108); 32 mm opening at X=26.4 on that wall; two 15 mm side openings centered Y=23. All remain 12.7 mm tall. Earlier photo-oriented coordinates above describe the historical layout.
