// All units mm. Select wall, top, or assembly.
$fn=64;
part="wall";
width=134.9; depth=108; wall_height=25.4;
wall_thickness=2; top_thickness=2; corner_radius=1;
hole_inset=4; post_diameter=6; m3_pilot=2.5; pilot_depth=10;
notch_height=wall_height/2;
// Adafruit #2890 DoubleWing.brd: board X=0..50.8, Y=0..46.99.
// Eight mounts from FEATHERWING_DIM at Y=0 and Y=24.13.
feather_origin=[width/2-25.4,depth+4];
feather_standoff_height=4;
feather_pilot=1.7; // M2 tapping/plastic screw pilot.
extension_projection=56;
extension_root_width=100;
extension_tip_width=62;
module feather_points() {
    for(x=[2.54,48.26], y=[2.54,20.32,26.67,44.45])
        translate([feather_origin.x+x,feather_origin.y+y,0]) children();
}
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
        union() {
            linear_extrude(height=top_thickness) union() {
                outer_profile();
                polygon([[width/2-extension_root_width/2,depth-0.5],
                         [width/2+extension_root_width/2,depth-0.5],
                         [width/2+extension_tip_width/2,depth+extension_projection],
                         [width/2-extension_tip_width/2,depth+extension_projection]]);
            }
            feather_points() translate([0,0,top_thickness-0.1])
                cylinder(d=5.5,h=feather_standoff_height+0.1);
        }
        corners() translate([0,0,-1]) cylinder(d=3.2,h=top_thickness+2);
        // Blind holes leave the lid's 2 mm thickness intact.
        feather_points() translate([0,0,top_thickness])
            cylinder(d=feather_pilot,h=feather_standoff_height+1);
    }
}
if(part=="wall") wall_frame();
else if(part=="top") top_plate();
else if(part=="assembly") {
    color("steelblue") import("base_platform.stl");
    color("cadetblue") translate([0,depth,3+wall_height]) rotate([180,0,0]) wall_frame();
    color("lightblue") translate([0,0,3+wall_height]) top_plate();
}
