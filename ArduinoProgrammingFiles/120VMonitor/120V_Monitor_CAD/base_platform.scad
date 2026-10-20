// All dimensions in mm. Hole spacing is confirmed; PCB edge offsets are provisional.
$fn = 64;
corner_radius = 1;
cover_hole_diameter = 3.2; // M3 clearance; mates to future shell bosses.
cover_hole_inset = 4;
base_thickness = 3;
standoff_height = 4;
standoff_diameter = 5.5;
m2_pilot_diameter = 1.7; // Tap M2 or use suitable plastic-threading screws; tune for printer.
pcb_edge_offset = 3; // Assumed distance from mounting-hole centers to each board edge.
board_gap = 12.7; // Edge-to-edge gap, based on assumed PCB outlines.
edge_margin = 6;
relay_holes = [66.7, 45];
sensor_holes = [31.5, 61];
free_space = 45;
future_interior_height = 25.4; // From top of platform, including standoffs. Shell not modeled.
relay_size = relay_holes + [2*pcb_edge_offset, 2*pcb_edge_offset];
sensor_size = sensor_holes + [2*pcb_edge_offset, 2*pcb_edge_offset];
sensor_origin = [edge_margin, edge_margin];
relay_origin = [edge_margin + sensor_size.x + board_gap, edge_margin];
platform_size = [relay_origin.x + relay_size.x + edge_margin,
                 edge_margin + max(relay_size.y + free_space, sensor_size.y) + edge_margin];
module mounting_points() {
    for (board = [0:1]) {
        origin = board == 0 ? sensor_origin : relay_origin;
        spacing = board == 0 ? sensor_holes : relay_holes;
        for (x = [0, spacing.x], y = [0, spacing.y])
            translate([origin.x + pcb_edge_offset + x, origin.y + pcb_edge_offset + y, 0]) children();
    }
}
difference() {
    union() {
        linear_extrude(height=base_thickness)
            translate([corner_radius, corner_radius])
                offset(r=corner_radius)
                    square([platform_size.x-2*corner_radius,
                            platform_size.y-2*corner_radius]);
        mounting_points() translate([0,0,base_thickness-0.1])
            cylinder(d=standoff_diameter, h=standoff_height+0.1);
    }
    // Blind pilot holes leave 2 mm of solid material below each screw.
    mounting_points() translate([0,0,2])
        cylinder(d=m2_pilot_diameter, h=base_thickness+standoff_height);
    for (x = [cover_hole_inset, platform_size.x-cover_hole_inset],
         y = [cover_hole_inset, platform_size.y-cover_hole_inset])
        translate([x,y,-1]) cylinder(d=cover_hole_diameter, h=base_thickness+2);
}
