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
cover_points=[(x,y) for x in (4,130.9) for y in (4,104)]
for x,y in cover_points:
    boolean(base,cylinder('M3 cover clearance',x,y,1.5,1.6,5),'DIFFERENCE')
points=[(x,y) for x in (9,40.5) for y in (9,70)]+[(x,y) for x in (59.2,125.9) for y in (9,54)]
for x,y in points:
    boolean(base,cylinder('Boss',x,y,4.95,2.75,4.1),'UNION')
    boolean(base,cylinder('M2 pilot',x,y,5,0.85,6),'DIFFERENCE')
import bmesh
bm=bmesh.new(); bm.from_mesh(base.data)
bad=sum(not e.is_manifold for e in bm.edges)
assert bad==0, f'Non-manifold edges: {bad}'
bm.free()
bpy.ops.object.select_all(action='DESELECT'); base.select_set(True)
bpy.context.view_layer.objects.active=base
bpy.ops.export_mesh.stl(filepath=os.path.join(ROOT,'base_platform.stl'),use_selection=True)
mat=bpy.data.materials.new('Blue polymer'); mat.diffuse_color=(0.075,0.29,0.46,1); base.data.materials.append(mat)
scene=bpy.context.scene
scene.world.color=(0.65,0.65,0.65)
bpy.ops.object.camera_add(location=(205,-150,215))
camera=bpy.context.object; camera.rotation_euler=(Vector((67,54,0))-camera.location).to_track_quat('-Z','Y').to_euler()
camera.data.type='ORTHO'; camera.data.ortho_scale=190; scene.camera=camera
scene.render.engine='BLENDER_WORKBENCH'; scene.display.shading.light='STUDIO'; scene.display.shading.color_type='MATERIAL'
scene.display.shading.show_shadows=True; scene.display.shading.show_cavity=True
scene.render.resolution_x=1200; scene.render.resolution_y=1000; scene.render.resolution_percentage=100
scene.render.filepath=os.path.join(ROOT,'base_platform_preview.png')
bpy.context.preferences.filepaths.save_version=0
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(ROOT,'base_platform.blend'))
bpy.ops.render.render(write_still=True)
print('Verified closed manifold; platform 134.9 x 108 x 3 mm; eight 4 mm bosses; total height 7 mm.')
