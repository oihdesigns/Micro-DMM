// Millimeters. Default orientation is roof-down for support-free printing.
$fn=64;
width=134.9;
depth=108;
interior_height=25.4; // From platform top to inside roof, including standoffs.
wall=2;
roof=2;
corner_radius=1;
post_diameter=6;
m3_pilot=2.5; // Tap M3; tune for printer. Not a heat-set-insert pocket.
pilot_depth=10;
hole_inset=4;
print_orientation=true;
module outer_profile() {
    translate([corner_radius,corner_radius]) offset(r=corner_radius)
        square([width-2*corner_radius,depth-2*corner_radius]);
}
module shell() {
    difference() {
        union() {
            difference() {
                linear_extrude(height=interior_height+roof) outer_profile();
                translate([wall,wall,-0.1])
                    cube([width-2*wall,depth-2*wall,interior_height+0.1]);
            }
            for(x=[hole_inset,width-hole_inset],y=[hole_inset,depth-hole_inset])
                translate([x,y,0]) cylinder(d=post_diameter,h=interior_height+0.1);
        }
        for(x=[hole_inset,width-hole_inset],y=[hole_inset,depth-hole_inset])
            translate([x,y,-0.1]) cylinder(d=m3_pilot,h=pilot_depth+0.1);
    }
}
// Slot centers are approximate locations taken from the reference photo.
// Printing coordinates reverse Y relative to the board layout.
module u_slot(slot_width, cut_depth) {
    r=slot_width/2;
    curve_bottom=roof+5;
    translate([0,0,curve_bottom+r]) rotate([90,0,0])
        cylinder(r=r,h=cut_depth,center=true);
    translate([-r,-cut_depth/2,curve_bottom+r])
        cube([slot_width,cut_depth,interior_height+roof+1-(curve_bottom+r)]);
}
module finished_print_shell() {
    difference() {
        translate([0,depth,interior_height+roof]) rotate([180,0,0]) shell();
        translate([18,depth-wall/2,0]) u_slot(7,wall+2);
        translate([108.5,depth-wall/2,0]) u_slot(32,wall+2);
        translate([wall/2,depth-85,0]) rotate([0,0,90]) u_slot(15,wall+2);
        translate([width-wall/2,depth-85,0]) rotate([0,0,90]) u_slot(15,wall+2);
    }
}
if(print_orientation) finished_print_shell();
else translate([0,depth,3+interior_height+roof]) rotate([180,0,0]) finished_print_shell();
