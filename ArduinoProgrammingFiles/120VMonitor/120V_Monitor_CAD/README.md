# Board platform and cover shell — fit draft

Print `base_platform.stl` flat-side down, in millimeters. Editable parameters are in `base_platform.scad`; the Blender scene is also included.

- Platform: 134.9 × 108 mm, 3 mm thick.
- Four platform corners: 1 mm radius in plan view. Top and bottom perimeter edges remain square.
- Four cover attachment through-holes: 3.2 mm diameter for M3 screws, centers 4 mm from adjacent platform edges. Hole pattern: 126.9 × 100 mm. Matching shell bosses accept screws from underneath the platform. Holes are not countersunk.
- Eight standoffs: 4 mm above the platform, 5.5 mm diameter.
- M2 pilot bores: 1.7 mm diameter, 5 mm deep, with 2 mm of material below. Tap for M2 or use appropriate plastic-threading screws. Printer calibration may require changing bore diameter. Screw engagement should not exceed 5 mm.
- Sensor hole centers: 31.5 mm horizontally × 61 mm vertically.
- Relay hole centers: 66.7 mm horizontally × 45 mm vertically.
- Both boards align along their lower edges, as in the supplied photo.
- Assumed PCB edges extend 3 mm beyond mounting-hole centers on all sides: sensor 37.5 × 67 mm, relay 72.7 × 51 mm. **Actual board outside dimensions were not supplied.** Exact edge alignment, the 12.7 mm edge gap and 45 mm free space depend on this assumption; confirm board outlines before printing a final fit.
- Board gap: 12.7 mm between assumed outlines. Free area above relay: 45 mm, plus 6 mm perimeter margin.
- Shell interior height: 25.4 mm above the platform surface, including the 4 mm standoffs. Check actual component heights before final printing.

## Cover shell

Print `cover_shell.stl` as supplied, flat roof on the bed and open side upward. Editable source: `cover_shell.scad`; editable scene: `cover_shell.blend`. Use millimeters.

- Outside footprint: 134.9 × 108 mm, matching platform; shell height 27.4 mm. Assembled height: 30.4 mm, excluding screw heads.
- Walls and roof: 2 mm. Four outside vertical corners have 1 mm radius; straight top and bottom edges are square.
- Four 6 mm diameter posts join walls and roof. Their ends seat on the platform, with matching 126.9 × 100 mm screw spacing.
- Blind pilot bores: 2.5 mm diameter × 10 mm deep, intended for M3 tapping. These are not modeled threads or insert pockets. Adjust pilots to suit print calibration.
- Use four M3 × 10 mm screws from underneath: approximately 7 mm engagement after the 3 mm base. Screw heads project underneath. Tap the shell pilots before assembly.
- Flat butt joint; no locating lip or seal. Screws locate the shell. Board outlines retain the provisional 3 mm edge-offset assumption above.
- Four U-shaped wire slots open to the rim in roof-down printing orientation. Widths (reference photo orientation): bottom left 7 mm, bottom right 32 mm, upper left side 15 mm, upper right side 15 mm. Each has a semicircular end with radius half its width; the lowest point is 5 mm above the inside roof surface (7 mm above the print bed). Slot centers approximate the photo marks: bottom edge X=18 and 108.5 mm; both side edges Y=85 mm, measured in assembled platform coordinates from the lower left corner. Exact locations can be adjusted in the source.
- Validation: closed manifold mesh, positive volume, matching screw pattern and height; more than 1 mm between modeled shell posts and board standoffs. Physical component and wire clearance still require a fit check.

This is a mechanical layout draft, not a verified mains-voltage enclosure; material, electrical clearances and wire retention still need to be addressed in the completed enclosure.
