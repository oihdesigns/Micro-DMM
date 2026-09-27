// Relay / Arduino UNO mounting plate. All dimensions are millimetres.
// Top view matches the photo: relay outputs toward +Y; UNO USB/power toward -X.
// UNO pattern: official Arduino UNO R4 WiFi manufacturing drill, T08.
// https://docs.arduino.cc/hardware/uno-r4-wifi/
$fn = 128;
width = 165.1;
height = 139.7;
base_thickness = 3;
standoff_height = 2;
standoff_diameter = 6;
hole_diameter = 2;
corner_radius = 3;
relay_pitch = [66.7, 45];
relay_inner_hole_gap = 12;
relay_bottom_y = 80;
relay_left_x = (width - (2*relay_pitch[0] + relay_inner_hole_gap))/2;
relay_origins = [[relay_left_x, relay_bottom_y],
                 [relay_left_x + relay_pitch[0] + relay_inner_hole_gap, relay_bottom_y]];
uno_origin = [(width - 68.58)/2, 14];
uno_holes_local = [[13.97,2.54], [66.04,7.62], [66.04,35.56], [15.24,50.80]];
relay_holes = [for(o=relay_origins, dx=[0,relay_pitch[0]], dy=[0,relay_pitch[1]])
    [o[0]+dx, o[1]+dy]];
uno_holes = [for(p=uno_holes_local) [uno_origin[0]+p[0],uno_origin[1]+p[1]]];
holes = concat(relay_holes,uno_holes);

module rounded_base() {
    linear_extrude(base_thickness)
        hull() for(x=[corner_radius,width-corner_radius],y=[corner_radius,height-corner_radius])
            translate([x,y]) circle(r=corner_radius);
}
module mount() {
    difference() {
        union() {
            rounded_base();
            for(p=holes) translate([p[0],p[1],base_thickness-0.01])
                cylinder(d=standoff_diameter,h=standoff_height+0.01);
        }
        for(p=holes) translate([p[0],p[1],-0.1])
            cylinder(d=hole_diameter,h=base_thickness+standoff_height+0.2);
    }
}
mount();

// Optional approximate board envelopes for F5 preview only; excluded from STL.
show_boards = false;
if(show_boards) {
    for(o=relay_origins) %color("firebrick",0.35)
        translate([o[0]-3.15,o[1]-3,5]) cube([73,51,1.6]);
    %color("teal",0.35) translate([uno_origin[0],uno_origin[1],5]) cube([68.58,53.34,1.6]);
}
