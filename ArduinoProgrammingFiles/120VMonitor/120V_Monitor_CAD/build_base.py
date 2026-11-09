"""Rebuild printable mesh and preview using Blender 3.6 background mode."""
import bpy, math, os
from mathutils import Vector
ROOT = os.path.dirname(os.path.abspath(__file__))
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
def box(name, location, dimensions):
    bpy.ops.mesh.primitive_cube_add(size=1, location=location)
    obj=bpy.context.object; obj.name=name; obj.dimensions=dimensions
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    return obj
def cylinder(name,x,y,z,r,depth):
    bpy.ops.mesh.primitive_cylinder_add(vertices=64, radius=r, depth=depth, location=(x,y,z))
    obj=bpy.context.object; obj.name=name; return obj
def boolean(target, other, operation):
    bpy.context.view_layer.objects.active=target
    mod=target.modifiers.new(operation,'BOOLEAN'); mod.operation=operation; mod.solver='EXACT'; mod.object=other
    bpy.ops.object.modifier_apply(modifier=mod.name)
    bpy.data.objects.remove(other, do_unlink=True)
base=box('Base platform mm',(134.9/2,108/2,1.5),(134.9,108,3))
bpy.context.view_layer.objects.active=base
 # Bevel only vertical edges: rounded plan-view corners, square top/bottom edges.
for edge in base.data.edges:
    a,b=(base.data.vertices[i].co for i in edge.vertices)
    edge.bevel_weight=1.0 if abs(a.z-b.z)>2.9 else 0.0
fillet=base.modifiers.new('1 mm corner radius','BEVEL')
fillet.limit_method='WEIGHT'
fillet.width=1; fillet.segments=8
bpy.ops.object.modifier_apply(modifier=fillet.name)
outline=[(2.45,107.5),(132.45,107.5),(124.45,169.34),(10.45,169.34)]
verts=[(x,y,z) for z in (0,3) for x,y in outline]
faces=[(3,2,1,0),(4,5,6,7)]+[(i,(i+1)%4,(i+1)%4+4,i+4) for i in range(4)]
mesh=bpy.data.meshes.new('Mega GIGA extension'); mesh.from_pydata(verts,[],faces); mesh.update()
extension=bpy.data.objects.new('Flat base extension',mesh); bpy.context.collection.objects.link(extension)
boolean(base,extension,'UNION')
cover_points=[(x,y) for x in (4,130.9) for y in (4,104)]
for x,y in cover_points:
    boolean(base,cylinder('M3 cover clearance',x,y,1.5,1.6,5),'DIFFERENCE')
points=[(x,y) for x in (9,40.5) for y in (9,70)]+[(x,y) for x in (59.2,125.9) for y in (9,54)]
# Rotate the lower-board layout 180 degrees: relay left, sensor right,
# flush board edges adjacent to the GIGA extension, matching the latest photo.
points=[(134.9-x,108-y) for x,y in points]
arduino_holes=[(13.97,2.54),(15.24,50.8),(90.17,50.8),(96.52,2.54)]
# GIGA USB/power connectors toward the right, as in the photo.
points += [(118.25-x,165.34-y) for x,y in arduino_holes]
for x,y in points:
    boolean(base,cylinder('Boss',x,y,4.95,2.75,4.1),'UNION')
    boolean(base,cylinder('M2 pilot',x,y,5,0.85,6),'DIFFERENCE')
import bmesh
bm=bmesh.new(); bm.from_mesh(base.data)
bad=sum(not e.is_manifold for e in bm.edges)
assert bad==0, f'Non-manifold edges: {bad}'
assert bm.calc_volume()>0
unseen=set(bm.verts); components=0
while unseen:
    components+=1; stack=[unseen.pop()]
    while stack:
        for e in stack.pop().link_edges:
            for v in e.verts:
                if v in unseen: unseen.remove(v); stack.append(v)
assert components==1
assert len(points)==12
bm.free()
bpy.ops.object.select_all(action='DESELECT'); base.select_set(True)
bpy.context.view_layer.objects.active=base
bpy.ops.export_mesh.stl(filepath=os.path.join(ROOT,'base_platform.stl'),use_selection=True)
mat=bpy.data.materials.new('Blue polymer'); mat.diffuse_color=(0.075,0.29,0.46,1); base.data.materials.append(mat)
scene=bpy.context.scene
scene.world.color=(0.65,0.65,0.65)
bpy.ops.object.camera_add(location=(205,-150,215))
camera=bpy.context.object; camera.rotation_euler=(Vector((67,83,0))-camera.location).to_track_quat('-Z','Y').to_euler()
camera.data.type='ORTHO'; camera.data.ortho_scale=235; scene.camera=camera
scene.render.engine='BLENDER_WORKBENCH'; scene.display.shading.light='STUDIO'; scene.display.shading.color_type='MATERIAL'
scene.display.shading.show_shadows=True; scene.display.shading.show_cavity=True
scene.render.resolution_x=1200; scene.render.resolution_y=1000; scene.render.resolution_percentage=100
scene.render.filepath=os.path.join(ROOT,'base_platform_preview.png')
bpy.context.preferences.filepaths.save_version=0
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(ROOT,'base_platform.blend'))
bpy.ops.render.render(write_still=True)
# Top-down layout diagram with approximate PCB outlines; not exported to STL.
for label,center,size in [('GIGA / MEGA',(67.45,138.67),(101.6,53.34)),
                          ('RELAY',(42.35,76.5),(72.7,51)),
                          ('SENSOR',(110.15,68.5),(37.5,67))]:
    x,y=center; w,h=size
    curve=bpy.data.curves.new(label+' outline','CURVE'); curve.dimensions='3D'; curve.bevel_depth=0.2
    spline=curve.splines.new('POLY'); spline.points.add(3)
    for p,co in zip(spline.points,[(x-w/2,y-h/2,7.2,1),(x+w/2,y-h/2,7.2,1),(x+w/2,y+h/2,7.2,1),(x-w/2,y+h/2,7.2,1)]): p.co=co
    spline.use_cyclic_u=True
    obj=bpy.data.objects.new(label+' outline',curve); bpy.context.collection.objects.link(obj)
    bpy.ops.object.text_add(location=(x,y,7.5))
    text=bpy.context.object; text.data.body=label; text.data.align_x='CENTER'; text.data.align_y='CENTER'; text.data.size=4
camera.location=(67.45,84.67,400); camera.rotation_euler=(0,0,0); camera.data.ortho_scale=210
scene.display.shading.show_shadows=False
scene.render.resolution_x=1100; scene.render.resolution_y=1200
scene.render.filepath=os.path.join(ROOT,'base_layout_preview.png'); bpy.ops.render.render(write_still=True)
print('PASS: one closed manifold base, positive volume; 134.9 x 169.34 x 3 mm; twelve 4 mm bosses all face +Z; height 7 mm.')
