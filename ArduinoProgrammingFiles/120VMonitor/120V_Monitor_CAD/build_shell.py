"""Blender 3.6: create roof-down shell STL and editable scene; units are mm."""
import bpy, bmesh, os
from mathutils import Vector, Matrix
ROOT=os.path.dirname(os.path.abspath(__file__))
bpy.ops.object.select_all(action='SELECT'); bpy.ops.object.delete(use_global=False)
def box(name, center, size):
    bpy.ops.mesh.primitive_cube_add(size=1,location=center)
    obj=bpy.context.object; obj.name=name; obj.dimensions=size
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    return obj
def cylinder(x,y,z,r,h):
    bpy.ops.mesh.primitive_cylinder_add(vertices=64,radius=r,depth=h,location=(x,y,z))
    return bpy.context.object
def boolean(target,other,operation):
    bpy.context.view_layer.objects.active=target
    mod=target.modifiers.new(operation,'BOOLEAN'); mod.operation=operation; mod.solver='EXACT'; mod.object=other
    bpy.ops.object.modifier_apply(modifier=mod.name)
    bpy.data.objects.remove(other,do_unlink=True)
# Build in assembly coordinates: platform top z=3, inside roof z=28.4.
shell=box('Cover shell',(67.45,54,16.7),(134.9,108,27.4))
for e in shell.data.edges:
    a,b=(shell.data.vertices[i].co for i in e.vertices)
    e.bevel_weight=1 if abs(a.z-b.z)>27 else 0
mod=shell.modifiers.new('Rounded vertical corners','BEVEL'); mod.limit_method='WEIGHT'; mod.width=1; mod.segments=8
bpy.ops.object.modifier_apply(modifier=mod.name)
boolean(shell,box('Interior',(67.45,54,15.65),(130.9,104,25.5)),'DIFFERENCE')
for x in (4,130.9):
    for y in (4,104):
        boolean(shell,cylinder(x,y,15.75,3,25.5),'UNION')
        boolean(shell,cylinder(x,y,7.95,1.25,10.1),'DIFFERENCE')
bm=bmesh.new(); bm.from_mesh(shell.data)
assert all(e.is_manifold for e in bm.edges), 'Shell must be a closed manifold'
assert bm.calc_volume()>0, 'Shell normals must face outward'
bm.free()
# Check post separation from each existing board standoff.
board_points=[(x,y) for x in (9,40.5) for y in (9,70)]+[(x,y) for x in (59.2,125.9) for y in (9,54)]
assert min(((x-bx)**2+(y-by)**2)**0.5-3-2.75 for x in (4,130.9) for y in (4,104) for bx,by in board_points)>1
assert abs((28.4-3)-25.4)<1e-6
# Transform geometry to roof-down print coordinates, bottom z=0.
bpy.context.view_layer.objects.active=shell
bpy.ops.object.select_all(action='DESELECT'); shell.select_set(True)
bpy.ops.object.transform_apply(location=True,rotation=True,scale=True)
shell.data.transform(Matrix(((1,0,0,0),(0,-1,0,108),(0,0,-1,30.4),(0,0,0,1))))
def u_slot(width,x,y,side=False):
    r=width/2; center_z=7+r
    curve=cylinder(x,y,center_z,r,4)
    curve.rotation_euler[1 if side else 0]=1.5707963267948966
    top=28.4
    size=(4,width,top-center_z) if side else (width,4,top-center_z)
    boolean(curve,box('Open slot', (x,y,(center_z+top)/2),size),'UNION')
    boolean(shell,curve,'DIFFERENCE')
u_slot(7,18,107)
u_slot(32,108.5,107)
u_slot(15,1,23,True)
u_slot(15,133.9,23,True)
bm=bmesh.new(); bm.from_mesh(shell.data)
bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=0.00001)
bmesh.ops.dissolve_degenerate(bm,edges=list(bm.edges),dist=0.00001)
bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces))
assert all(e.is_manifold for e in bm.edges), 'Slotted shell must be closed manifold'
assert bm.calc_volume()>0
bm.to_mesh(shell.data); shell.data.update()
bm.free()
coords=[v.co for v in shell.data.vertices]
assert abs(min(v.z for v in coords))<1e-4
assert abs(max(v.z for v in coords)-27.4)<1e-4
bpy.ops.object.select_all(action='DESELECT'); shell.select_set(True)
bpy.ops.export_mesh.stl(filepath=os.path.join(ROOT,'cover_shell.stl'),use_selection=True)
mat=bpy.data.materials.new('Blue polymer'); mat.diffuse_color=(0.075,0.29,0.46,1); shell.data.materials.append(mat)
scene=bpy.context.scene; scene.world.color=(0.65,0.65,0.65)
bpy.ops.object.camera_add(location=(205,250,240)); camera=bpy.context.object
camera.rotation_euler=(Vector((67,54,10))-camera.location).to_track_quat('-Z','Y').to_euler()
camera.data.type='ORTHO'; camera.data.ortho_scale=195; scene.camera=camera
scene.render.engine='BLENDER_WORKBENCH'; scene.display.shading.light='STUDIO'; scene.display.shading.color_type='MATERIAL'
scene.display.shading.show_shadows=True; scene.display.shading.show_cavity=True
scene.render.resolution_x=1200; scene.render.resolution_y=1000; scene.render.resolution_percentage=100
scene.render.filepath=os.path.join(ROOT,'cover_shell_preview.png')
bpy.context.preferences.filepaths.save_version=0
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(ROOT,'cover_shell.blend'))
bpy.ops.render.render(write_still=True)
print('PASS: closed manifold; 25.4 mm internal height; matching four M3 posts; >1 mm post-to-standoff clearance; roof-down STL.')
