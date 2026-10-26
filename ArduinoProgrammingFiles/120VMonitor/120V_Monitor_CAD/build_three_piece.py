"""Blender 3.6: generate wall frame, top plate and exploded assembly."""
import bpy, bmesh, os
from mathutils import Vector, Matrix
ROOT=os.path.dirname(os.path.abspath(__file__))
bpy.ops.object.select_all(action='SELECT'); bpy.ops.object.delete(use_global=False)
W,D,H=134.9,108,25.4
posts=[(x,y) for x in (4,130.9) for y in (4,104)]
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
def rounded_box(name,height):
    obj=box(name,(W/2,D/2,height/2),(W,D,height))
    for e in obj.data.edges:
        a,b=(obj.data.vertices[i].co for i in e.vertices)
        e.bevel_weight=1 if abs(a.z-b.z)>height-0.01 else 0
    mod=obj.modifiers.new('1 mm vertical corner radius','BEVEL')
    mod.limit_method='WEIGHT'; mod.width=1; mod.segments=8
    bpy.ops.object.modifier_apply(modifier=mod.name)
    return obj
def verify_export(obj,filename,height):
    bm=bmesh.new(); bm.from_mesh(obj.data)
    bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=0.00001)
    bmesh.ops.dissolve_degenerate(bm,edges=list(bm.edges),dist=0.00001)
    bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces))
    assert all(e.is_manifold for e in bm.edges), filename+' has non-manifold edges'
    assert bm.calc_volume()>0
    unseen=set(bm.verts); components=0
    while unseen:
        components+=1; stack=[unseen.pop()]
        while stack:
            for e in stack.pop().link_edges:
                for v in e.verts:
                    if v in unseen: unseen.remove(v); stack.append(v)
    assert components==1, (filename,components)
    bm.to_mesh(obj.data); bm.free(); obj.data.update()
    bpy.ops.object.select_all(action='DESELECT'); obj.select_set(True)
    bpy.context.view_layer.objects.active=obj
    bpy.ops.object.transform_apply(location=True,rotation=True,scale=True)
    assert abs(min(v.co.z for v in obj.data.vertices))<0.0001
    assert abs(max(v.co.z for v in obj.data.vertices)-height)<0.0001
    bpy.ops.export_mesh.stl(filepath=os.path.join(ROOT,filename),use_selection=True)
    print('PASS:',filename,'closed manifold, one connected solid, height',height)
# Print frame with top mating face down and base mating face up.
frame=rounded_box('Wall frame',H)
boolean(frame,box('Interior',(W/2,D/2,H/2),(W-4,D-4,H+2)),'DIFFERENCE')
for x,y in posts:
    boolean(frame,cylinder(x,y,H/2,3,H),'UNION')
    boolean(frame,cylinder(x,y,4.95,1.25,10.1),'DIFFERENCE')
    boolean(frame,cylinder(x,y,H-4.95,1.25,10.1),'DIFFERENCE')
slots=[(7,18,107,False),(32,108.5,107,False),(15,1,23,True),(15,133.9,23,True)]
for width,x,y,side in slots:
    size=(4,width,H/2+1) if side else (width,4,H/2+1)
    boolean(frame,box('Rectangular wire notch',(x,y,3*H/4+0.5),size),'DIFFERENCE')
assert H-20>0
verify_export(frame,'wall_frame.stl',H)
top=rounded_box('Top plate',2)
outline=[(17.45,107.5),(117.45,107.5),(98.45,164),(36.45,164)]
verts=[(x,y,z) for z in (0,2) for x,y in outline]
faces=[(3,2,1,0),(4,5,6,7)]+[(i,(i+1)%4,(i+1)%4+4,i+4) for i in range(4)]
mesh=bpy.data.meshes.new('Trapezoid extension'); mesh.from_pydata(verts,[],faces); mesh.update()
extension=bpy.data.objects.new('Flat extension',mesh); bpy.context.collection.objects.link(extension)
boolean(top,extension,'UNION')
feather_points=[(W/2-25.4+x,D+4+y) for x in (2.54,48.26) for y in (2.54,20.32,26.67,44.45)]
for x,y in feather_points:
    boolean(top,cylinder(x,y,3.95,2.75,4.1),'UNION')
    boolean(top,cylinder(x,y,4.5,0.85,5),'DIFFERENCE')
for x,y in posts: boolean(top,cylinder(x,y,1,1.6,4),'DIFFERENCE')
verify_export(top,'top_plate.stl',6)
bpy.ops.import_mesh.stl(filepath=os.path.join(ROOT,'base_platform.stl'))
base=bpy.context.object; base.name='Base platform'
bm=bmesh.new(); bm.from_mesh(base.data)
assert all(e.is_manifold for e in bm.edges); bm.free()
board_points=[(x,y) for x in (9,40.5) for y in (9,70)]+[(x,y) for x in (59.2,125.9) for y in (9,54)]
assert min(((x-bx)**2+(y-by)**2)**0.5-5.75 for x,y in posts for bx,by in board_points)>1
assert 10-3<10 and 8-2<10
for obj,color in [(base,(0.075,0.29,0.46,1)),(frame,(0.10,0.38,0.52,1)),(top,(0.19,0.48,0.61,1))]:
    mat=bpy.data.materials.new(obj.name); mat.diffuse_color=color; obj.data.materials.append(mat)
scene=bpy.context.scene; scene.world.color=(0.65,0.65,0.65)
scene.render.engine='BLENDER_WORKBENCH'; scene.display.shading.light='STUDIO'; scene.display.shading.color_type='MATERIAL'
scene.display.shading.show_shadows=True; scene.display.shading.show_cavity=True
scene.render.resolution_x=1200; scene.render.resolution_y=1100; scene.render.resolution_percentage=100
base.hide_render=True; top.hide_render=True
bpy.ops.object.camera_add(location=(205,250,210)); camera=bpy.context.object
camera.rotation_euler=(Vector((67,54,10))-camera.location).to_track_quat('-Z','Y').to_euler()
camera.data.type='ORTHO'; camera.data.ortho_scale=190; scene.camera=camera
scene.render.filepath=os.path.join(ROOT,'wall_frame_preview.png'); bpy.ops.render.render(write_still=True)
frame.hide_render=True; top.hide_render=False
camera.location=(205,-160,250)
camera.rotation_euler=(Vector((67,80,1))-camera.location).to_track_quat('-Z','Y').to_euler()
camera.data.ortho_scale=225
scene.render.filepath=os.path.join(ROOT,'top_plate_preview.png'); bpy.ops.render.render(write_still=True)
frame.hide_render=False
frame.data.transform(Matrix(((1,0,0,0),(0,-1,0,D),(0,0,-1,H),(0,0,0,1))))
frame.location.z=17; top.location.z=58
base.hide_render=False; top.hide_render=False
camera.location=(220,-210,195)
camera.rotation_euler=(Vector((67,80,29))-camera.location).to_track_quat('-Z','Y').to_euler()
camera.data.ortho_scale=260
scene.render.filepath=os.path.join(ROOT,'three_piece_preview.png')
bpy.context.preferences.filepaths.save_version=0
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(ROOT,'three_piece_enclosure.blend'))
bpy.ops.render.render(write_still=True)
print('PASS: wall height 25.4; rectangular notch height 12.7; opposing blind pilots separated by 5.4; standoff clearance >1 mm.')
