// All units mm. Select wall, top, or assembly.
$fn=64;
part="wall";
width=134.9; depth=108; wall_height=25.4;
wall_thickness=2; top_thickness=2; corner_radius=1;
hole_inset=4; post_diameter=6; m3_pilot=2.5; pilot_depth=10;
notch_height=wall_height/2;
module outer_profile() {
    translate([corner_radius,corner_radius]) offset(r=corner_radius)
        square([width-2*corner_radius,depth-2*corner_radius]);
}
module corners() {
    for(x=[hole_inset,width-hole_inset],y=[hole_inset,depth-hole_inset])
        translate([x,y,0]) children();
}
// Print top mating face down; notches open upward toward base mating face.
module wall_frame() {
    difference() {
        union() {
            difference() {
                linear_extrude(height=wall_height) outer_profile();
                translate([wall_thickness,wall_thickness,-1])
                    cube([width-2*wall_thickness,depth-2*wall_thickness,wall_height+2]);
            }
            corners() cylinder(d=post_diameter,h=wall_height);
        }
        corners() {
            translate([0,0,-0.1]) cylinder(d=m3_pilot,h=pilot_depth+0.1);
            translate([0,0,wall_height-pilot_depth]) cylinder(d=m3_pilot,h=pilot_depth+0.1);
        }
        // Photo bottom edge; print orientation reverses Y.
        translate([18-7/2,depth-wall_thickness-1,wall_height-notch_height])
            cube([7,wall_thickness+2,notch_height+1]);
        translate([108.5-32/2,depth-wall_thickness-1,wall_height-notch_height])
            cube([32,wall_thickness+2,notch_height+1]);
        for(x=[-1,width-wall_thickness-1])
            translate([x,depth-85-15/2,wall_height-notch_height])
                cube([wall_thickness+2,15,notch_height+1]);
    }
}
module top_plate() {
    difference() {
        linear_extrude(height=top_thickness) outer_profile();
        corners() translate([0,0,-1]) cylinder(d=3.2,h=top_thickness+2);
    }
}
if(part=="wall") wall_frame();
else if(part=="top") top_plate();
else if(part=="assembly") {
    color("steelblue") import("base_platform.stl");
    color("cadetblue") translate([0,depth,3+wall_height]) rotate([180,0,0]) wall_frame();
    color("lightblue") translate([0,0,3+wall_height]) top_plate();
}
