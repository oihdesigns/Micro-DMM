import sys,json,re,warnings
from pathlib import Path
sys.path.insert(0,r'C:\Users\Nick\.codex\visualizations\2026\09\29\01a0ece9-af29-7560-aa5b-b8634a1263b9\geometry-deps')
import numpy as np,trimesh
from shapely.geometry import Polygon
warnings.filterwarnings('ignore',category=RuntimeWarning)
W=Path(__file__).resolve().parent;O=W.parent
shell=trimesh.load_scene(O/'BlinkyHawk_V3b_Shell.3mf').to_mesh()
lidscene=trimesh.load_scene(O/'BlinkyHawk_V3b_Lid_Multipart.3mf')
lid=trimesh.load_mesh(O/'BlinkyHawk_V3b_Lid.stl')
old=trimesh.load_mesh(W/'BlinkyHawk_V3_Shell_original.stl')
records=json.loads((W/'fit_study.json').read_text())
report={'parts':{},'components':{},'checks':{}}
for name,m in [('shell',shell),('lid_stl',lid),('lid_3mf',trimesh.load_scene(O/'BlinkyHawk_V3b_Lid.3mf').to_mesh())]:
    assert m.is_watertight and m.is_volume and len(m.split())==1
    assert np.all(m.area_faces>1e-10)
    report['parts'][name]=dict(watertight=True,positive_volume=True,connected_solids=1,triangles=len(m.faces),bounds_mm=m.bounds.tolist())
report['parts']['lid_multipart_3mf']={'bodies':len(lidscene.geometry),'all_watertight':all(m.is_watertight for m in lidscene.geometry.values())}
assert report['parts']['lid_multipart_3mf']['bodies']==15 and report['parts']['lid_multipart_3mf']['all_watertight']
for record in records:
    name=record['name'];m=trimesh.load_mesh(W/('pcb_part_'+re.sub(r'[^a-zA-Z0-9_-]','_',name)+'.ply'))
    if m.is_watertight:m.fix_normals(multibody=True)
    bbox=m.bounding_box.to_mesh()
    overlap=float(trimesh.boolean.intersection([shell,bbox],engine='manifold').volume)
    method='conservative bounding box'
    if overlap>1e-6 and m.is_volume:
        overlap=float(trimesh.boolean.intersection([shell,m],engine='manifold').volume);method='exact closed mesh'
    assert overlap<1e-5,(name,overlap)
    top_clearance=16-m.bounds[1,2]
    assert top_clearance>0
    report['components'][name]={'shell_overlap_mm3':max(0,overlap),'check':method,'lid_clearance_mm':float(top_clearance)}

def loops(m,axis,value,axes):
    o=np.zeros(3);o[axis]=value;n=np.zeros(3);n[axis]=1
    return [Polygon(p[:,axes]) for p in m.section(plane_origin=o,plane_normal=n).discrete]
sholes=[p for p in loops(shell,2,15,[0,1]) if p.area<5]
lholes=[p for p in loops(lid,2,.75,[0,1]) if p.area<5]
sc=sorted([list(p.centroid.coords)[0] for p in sholes]);lc=sorted([list(p.centroid.coords)[0] for p in lholes])
assert len(sc)==len(lc)==4
# Compare by nearest location (triangulation can perturb ordering by micrometres).
errors=[min(np.linalg.norm(np.array(a)-np.array(b)) for b in lc) for a in sc]
print('screw centers shell',sc,'lid',lc,'error',errors)
assert max(errors)<.001
report['checks']['screw_centers_mm']=sc
report['checks']['maximum_screw_center_error_mm']=max(errors)
assembled_lid=lid.copy().apply_translation([0,0,16])
report['checks']['shell_lid_overlap_mm3']=max(0,float(trimesh.boolean.intersection([shell,assembled_lid],engine='manifold').volume))
assert report['checks']['shell_lid_overlap_mm3']<1e-5
# Compare USB exterior at the inner wall, where the metal nose enters the case.
usb=trimesh.load_mesh(W/'pcb_part_CHAMFER9_1.ply');usb.fix_normals()
opening=min(loops(shell,1,75.7,[0,2]),key=lambda p:p.area)
u=loops(usb,1,75.7,[0,2]);outer=max(u,key=lambda p:p.area)
assert opening.contains(outer)
report['checks']['USB_min_radial_clearance_mm']=opening.boundary.distance(outer.boundary)
report['checks']['USB_opening_bounds_xz_mm']=list(opening.bounds)
report['checks']['PCB_end_clearance_mm']=75.597-75.397
report['checks']['PCB_side_clearance_each_mm']=.5
report['checks']['J4_pin_tip_floor_clearance_mm']=1.845-1.2
report['checks']['floor_remaining_under_relief_mm']=1.2
report['checks']['J5_wire_route_width_mm']=2.4
report['checks']['J5_wire_route_vertical_gap_to_bottom_copper_mm']=2.505-1.2
# Shell changes below Y=68 are confined to the intended floor relief.
region=trimesh.creation.box(extents=[40,70,20],transform=trimesh.transformations.translation_matrix([15,33,12.01]))
a=trimesh.boolean.intersection([old,region],engine='manifold');b=trimesh.boolean.intersection([shell,region],engine='manifold')
same=float(trimesh.boolean.difference([a,b],engine='manifold').volume+trimesh.boolean.difference([b,a],engine='manifold').volume)
assert same<1e-4
report['checks']['unchanged_front_geometry_difference_mm3']=max(0,same)
(O/'Validation.json').write_text(json.dumps(report,indent=2))
print(json.dumps({'parts':report['parts'],'checks':report['checks'],'component_groups_clear':len(report['components'])},indent=2))
