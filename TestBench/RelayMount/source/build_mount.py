"""Parametric relay/UNO mounting plate. Dimensions in millimetres.

Requires cadquery and trimesh. Run from any directory; outputs go beside source/.
Official UNO R4 WiFi NC drill: T08, (0.55,0.10), (2.60,0.30),
(2.60,1.40), (0.60,2.00) inches, USB/power facing left.
"""
from pathlib import Path
import csv
import json
import math

import cadquery as cq
import trimesh

ROOT = Path(__file__).resolve().parent.parent
WIDTH, HEIGHT = 165.1, 139.7
BASE_T = 3.0
STANDOFF_H, STANDOFF_D = 2.0, 6.0
HOLE_D, CORNER_R = 2.0, 3.0
RELAY_DX, RELAY_DY = 66.7, 45.0
RELAY_INNER_HOLE_GAP = 12.0
RELAY_BOTTOM_Y = 80.0
UNO_SIZE = (68.58, 53.34)
UNO_ORIGIN = ((WIDTH - UNO_SIZE[0]) / 2, 14.0)
UNO_LOCAL = [(13.97, 2.54), (66.04, 7.62), (66.04, 35.56), (15.24, 50.80)]
RELAY_LEFT_X = (WIDTH - (2 * RELAY_DX + RELAY_INNER_HOLE_GAP)) / 2
RELAY_ORIGINS = [(RELAY_LEFT_X, RELAY_BOTTOM_Y),
                 (RELAY_LEFT_X + RELAY_DX + RELAY_INNER_HOLE_GAP, RELAY_BOTTOM_Y)]
holes = []
for board, (ox, oy) in enumerate(RELAY_ORIGINS, 1):
    for name, dx, dy in [('LL', 0, 0), ('LR', RELAY_DX, 0),
                         ('UR', RELAY_DX, RELAY_DY), ('UL', 0, RELAY_DY)]:
        holes.append((f'Relay_{board}_{name}', round(ox + dx, 5), round(oy + dy, 5)))
for name, (x, y) in zip(['LL', 'LR', 'UR', 'UL'], UNO_LOCAL):
    holes.append((f'UNO_{name}', round(UNO_ORIGIN[0] + x, 5), round(UNO_ORIGIN[1] + y, 5)))
points = [(x, y) for _, x, y in holes]

base = cq.Workplane('XY').box(WIDTH, HEIGHT, BASE_T, centered=(False, False, False))
base = base.edges('|Z').fillet(CORNER_R)
bosses = (cq.Workplane('XY').workplane(offset=BASE_T)
          .pushPoints(points).circle(STANDOFF_D / 2).extrude(STANDOFF_H))
cutters = (cq.Workplane('XY').workplane(offset=-1)
           .pushPoints(points).circle(HOLE_D / 2).extrude(BASE_T + STANDOFF_H + 2))
mount = base.union(bosses).cut(cutters).clean()
solid = mount.val()
assert solid.isValid(), 'Invalid CAD solid'
assert len(mount.solids().vals()) == 1, 'Plate must be one connected solid'
bb = solid.BoundingBox()
assert all(abs(a - b) < 1e-6 for a, b in zip([bb.xlen, bb.ylen, bb.zlen], [WIDTH, HEIGHT, BASE_T + STANDOFF_H]))
expected_volume = ((WIDTH * HEIGHT - (4 - math.pi) * CORNER_R**2) * BASE_T
                   + len(points) * math.pi * (STANDOFF_D / 2)**2 * STANDOFF_H
                   - len(points) * math.pi * (HOLE_D / 2)**2 * (BASE_T + STANDOFF_H))
assert abs(solid.Volume() - expected_volume) < 1e-5

# Confirm all twelve continuous cylindrical bores in the exact solid.
bores = [f for f in solid.Faces() if f.geomType() == 'CYLINDER'
         and abs(f._geomAdaptor().Cylinder().Radius() - HOLE_D / 2) < 1e-7]
assert len(bores) == 12
for f in bores:
    cylinder = f._geomAdaptor().Cylinder()
    p = cylinder.Location()
    assert any(math.hypot(p.X() - x, p.Y() - y) < 1e-6 for x, y in points)
    assert abs(f.BoundingBox().zlen - 5.0) < 1e-6

step_path, stl_path = ROOT / 'Relay_Arduino_Mount.step', ROOT / 'Relay_Arduino_Mount.stl'
cq.exporters.export(mount, str(step_path))
cq.exporters.export(mount, str(stl_path), tolerance=0.005, angularTolerance=0.05)
reimported = cq.importers.importStep(str(step_path))
assert reimported.val().isValid() and len(reimported.solids().vals()) == 1
assert abs(reimported.val().Volume() - expected_volume) < 1e-4
mesh = trimesh.load_mesh(stl_path)
assert mesh.is_watertight and mesh.is_winding_consistent and mesh.is_volume
assert mesh.body_count == 1 and mesh.euler_number == 2 - 2 * 12
assert max(abs(mesh.extents - [WIDTH, HEIGHT, 5.0])) < 1e-4
assert abs(mesh.volume - expected_volume) / expected_volume < 0.0001

with (ROOT / 'hole_coordinates_mm.csv').open('w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['hole', 'x_from_left_mm', 'y_from_bottom_mm', 'bore_diameter_mm', 'standoff_diameter_mm'])
    writer.writerows((name, x, y, HOLE_D, STANDOFF_D) for name, x, y in holes)

report = dict(dimensions_mm=[WIDTH, HEIGHT, BASE_T + STANDOFF_H], base_thickness_mm=BASE_T,
              standoff_height_mm=STANDOFF_H, standoff_diameter_mm=STANDOFF_D,
              holes=len(holes), hole_diameter_mm=HOLE_D, exact_solid_valid=True,
              step_reimport_valid=True, connected_solids=1, stl_watertight=bool(mesh.is_watertight),
              stl_winding_consistent=bool(mesh.is_winding_consistent), mesh_euler_number=int(mesh.euler_number),
              triangles=len(mesh.faces), exact_volume_mm3=solid.Volume(), mesh_volume_mm3=float(mesh.volume),
              origin='Lower-left corner of base, underside; +X right, +Y up, +Z toward boards',
              hole_coordinates=[dict(name=n, x=x, y=y) for n,x,y in holes])
(ROOT / 'validation.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps({k:v for k,v in report.items() if k != 'hole_coordinates'}, indent=2))

# STEP is exact analytic geometry; this OpenSCAD source is a second editable format.
scad = '''// Relay / Arduino UNO mounting plate. All dimensions are millimetres.
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
'''
(ROOT / 'Relay_Arduino_Mount.scad').write_text(scad)
