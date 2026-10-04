"""Create the slide-switch shell variant from the existing V3b shell (mm)."""
import sys
from pathlib import Path
import json
import zipfile
import hashlib
import xml.etree.ElementTree as ET

sys.path.insert(0, r'C:\Users\Nick\.codex\visualizations\2026\09\29\01a0ece9-af29-7560-aa5b-b8634a1263b9\geometry-deps')
import numpy as np
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from shapely.geometry import Polygon

OUT = Path(__file__).resolve().parent.parent
SOURCE = OUT / 'BlinkyHawk_V3b_Shell.stl'
source = trimesh.load_mesh(SOURCE)
# Snap sub-nanometer export noise at the coordinate planes before booleans.
vertices = source.vertices.copy()
vertices[np.abs(vertices) < 1e-9] = 0
source.vertices = vertices
above_xiao = '--above-xiao' in sys.argv
shifted = '--shift-inward-5mm' in sys.argv
assert not shifted or above_xiao, 'Use --above-xiao with --shift-inward-5mm'
xiao = trimesh.load_mesh(OUT / 'work' / 'pcb_part____0_1_1_39_.ply') if above_xiao else None
center_y = float(xiao.bounds[:, 1].mean() if above_xiao else source.bounds[:, 1].mean())
if shifted:
    center_y -= 5.0
label = 'centered above the XIAO board' if above_xiao else 'centered along the long side'
prefix = 'SlideSwitch_AboveXIAO' if above_xiao else 'SlideSwitch'
if shifted:
    label = 'above XIAO, moved 5 mm toward enclosure middle'
    prefix += '_Inward5mm'
top = float(source.bounds[1, 2])
width, height, diameter, pitch = 10.66, 5.8, 1.8, 15.0
center_z = top - height / 2

# LED is on the X=31 side. Cut only the opposite wall, at X=0.
opening = trimesh.creation.box(extents=[4, width, height + 1])
opening.apply_translation([1, center_y, top - height / 2 + .5])
cutters = [opening]
for y in [center_y - pitch / 2, center_y + pitch / 2]:
    hole = trimesh.creation.cylinder(radius=diameter / 2, height=4, sections=128)
    hole.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    hole.apply_translation([1, y, center_z])
    cutters.append(hole)
result = trimesh.boolean.difference([source, *cutters], engine='manifold')
assert result.is_watertight and result.is_volume and len(result.split()) == 1
assert np.all(result.area_faces > 1e-10)
assert np.allclose(result.bounds, source.bounds, atol=1e-5)
# Confirm the notch and both through-holes at the middle of the wall.
section = result.section(plane_origin=[1, 0, 0], plane_normal=[1, 0, 0])
polygons = [Polygon(p[:, [1, 2]]) for p in section.discrete]
holes = sorted([p for p in polygons if p.area < 3], key=lambda p: p.centroid.x)
assert len(holes) == 2
for p, y in zip(holes, [center_y - pitch / 2, center_y + pitch / 2]):
    assert np.allclose(p.bounds, [y - .9, center_z - .9, y + .9, center_z + .9], atol=1e-5)
outer = max(polygons, key=lambda p: p.area)
assert not outer.contains(Polygon([(center_y-width/2+.01, top-height+.01),
                                 (center_y+width/2-.01, top-height+.01),
                                 (center_y+width/2-.01, top-.01),
                                 (center_y-width/2+.01, top-.01)]))
added = trimesh.boolean.difference([result, source], engine='manifold')
assert abs(added.volume) < 1e-5
removed = trimesh.boolean.difference([source, result], engine='manifold')
assert removed.bounds[1, 0] <= 3.00001
outside_cutters = trimesh.boolean.difference([removed, *cutters], engine='manifold')
# Boolean roundoff can leave zero-thickness faces at old wall boundaries.
assert abs(outside_cutters.volume) < 1e-5

stem = 'BlinkyHawk_V3b_Shell_SlideSwitch' + ('_AboveXIAO' if above_xiao else '')
if shifted:
    stem += '_Inward5mm'
result.export(OUT / (stem + '.stl'))
ns = 'http://schemas.microsoft.com/3dmanufacturing/core/2015/02'
ET.register_namespace('', ns)
q = lambda s: '{' + ns + '}' + s
mf_source = OUT / 'BlinkyHawk_V3b_Shell.3mf'
with zipfile.ZipFile(mf_source) as z:
    root = ET.fromstring(z.read('3D/3dmodel.model'))
    obj = next(o for o in root.findall('.//' + q('object')) if o.find(q('mesh')) is not None)
    obj.set('name', stem)
    obj.remove(obj.find(q('mesh')))
    mesh = ET.SubElement(obj, q('mesh'))
    vertices = ET.SubElement(mesh, q('vertices'))
    triangles = ET.SubElement(mesh, q('triangles'))
    for v in result.vertices:
        ET.SubElement(vertices, q('vertex'), **dict(zip('xyz', [format(float(x), '.8f') for x in v])))
    for f in result.faces:
        ET.SubElement(triangles, q('triangle'), **dict(zip(['v1', 'v2', 'v3'], [str(int(i)) for i in f])))
    for item in root.findall(q('metadata')):
        if item.get('name') == 'ModificationDate': item.text = '2026-10-03' if above_xiao else '2026-10-02'
        if item.get('name') == 'Description': item.text = f'Slide switch opposite LED, {label}: 10.66 x 5.8 mm top-flush notch; diameter 1.8 mm holes, pitch 15 mm.'
    with zipfile.ZipFile(OUT / (stem + '.3mf'), 'w', zipfile.ZIP_DEFLATED) as dest:
        for item in z.infolist():
            # Remove the old thumbnail rather than showing obsolete geometry.
            if item.filename == 'Metadata/thumbnail.png': continue
            data = ET.tostring(root, encoding='utf-8', xml_declaration=True) if item.filename == '3D/3dmodel.model' else z.read(item.filename)
            if item.filename.endswith('.rels'):
                rel = ET.fromstring(data)
                for child in list(rel):
                    if 'thumbnail' in child.get('Type', ''): rel.remove(child)
                data = ET.tostring(rel, encoding='utf-8', xml_declaration=True)
            dest.writestr(item.filename, data)
roundtrip = trimesh.load_scene(OUT / (stem + '.3mf')).to_mesh()
assert roundtrip.is_volume and len(roundtrip.split()) == 1
assert abs(roundtrip.volume - result.volume) < 1e-4

fig = plt.figure(figsize=(12, 7))
ax = fig.add_subplot(211)
for pts in section.discrete: ax.plot(pts[:, 1], pts[:, 2], color='#29485c', lw=1.5)
ax.set_aspect('equal'); ax.set_xlim(center_y-15, center_y+15); ax.set_ylim(7, 19)
ax.annotate('', (center_y-width/2, 17.3), (center_y+width/2, 17.3), arrowprops={'arrowstyle':'<->'})
ax.text(center_y, 17.6, '10.66 mm', ha='center')
ax.annotate('', (center_y-7.5, 8.6), (center_y+7.5, 8.6), arrowprops={'arrowstyle':'<->'})
ax.text(center_y, 7.8, '15.0 mm hole spacing; diameter 1.8 mm', ha='center')
ax.text(center_y+6, 14.6, '5.8 mm deep\nTop edge at enclosure lip', fontsize=10)
ax.set_xlabel('Position along enclosure (mm)'); ax.set_ylabel('Height (mm)')
ax.set_title('Slide-switch mounting: opposite the LED, ' + label)
if above_xiao:
    board_center = float(xiao.bounds[:, 1].mean())
    ax.axvline(board_center, color='#b47625', linestyle=':', alpha=.6)
    ax.text(board_center, 7.05, 'XIAO board center', ha='center', color='#b47625', fontsize=9)
ax = fig.add_subplot(212, projection='3d')
light = np.array([-1, -.3, 1]); light /= np.linalg.norm(light)
shade = .5 + .5*np.clip(result.face_normals @ light, 0, 1)
ax.add_collection3d(Poly3DCollection(result.triangles, facecolors=shade[:,None]*np.array([.45,.62,.72]), edgecolors='none'))
ax.set_xlim(0,31); ax.set_ylim(0,77.397); ax.set_zlim(0,16)
ax.set_box_aspect([31,77.397,16]); ax.view_init(30, 195); ax.set_axis_off()
fig.tight_layout(); fig.savefig(OUT / (prefix + '_Preview.png'), dpi=160); plt.close(fig)
report = dict(source=SOURCE.name, source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
              opening_mm=[width,height], top_mm=top, center_y_mm=center_y, center_z_mm=center_z,
              mounting_hole_diameter_mm=diameter, mounting_hole_pitch_mm=pitch,
              hole_centers_yz_mm=[[center_y-pitch/2,center_z],[center_y+pitch/2,center_z]],
              watertight=True, connected_solids=1, stl_3mf_volume_match=True,
              removed_material_mm3=float(removed.volume), modification_confined_to_opposite_side_wall=True,
              tolerance_allowance_mm=0, switch_body_depth_not_supplied=True)
if above_xiao:
    report['xiao_board_bounds_mm'] = xiao.bounds.tolist()
    report['shift_from_enclosure_center_mm'] = center_y - float(source.bounds[:, 1].mean())
    report['position'] = label
    report['shift_from_xiao_center_mm'] = center_y - float(xiao.bounds[:, 1].mean())
(OUT / (prefix + '_Validation.json')).write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
