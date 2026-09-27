"""Render actual exported STL with Blender 3.6, in background mode."""
import bpy
from pathlib import Path
from mathutils import Vector

ROOT = Path(__file__).resolve().parent.parent
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
bpy.ops.import_mesh.stl(filepath=str(ROOT / 'Relay_Arduino_Mount.stl'))
part = bpy.context.selected_objects[0]
part.name = 'Printed mounting plate - actual STL'
mat = bpy.data.materials.new('Slate blue polymer')
mat.diffuse_color = (0.27, 0.45, 0.55, 1)
mat.use_nodes = True
shader = mat.node_tree.nodes.get('Principled BSDF')
shader.inputs['Base Color'].default_value = (0.27, 0.45, 0.55, 1)
shader.inputs['Roughness'].default_value = 0.72
part.data.materials.append(mat)
# Weighted normals smooth cylindrical walls without changing the mesh geometry.
for p in part.data.polygons:
    p.use_smooth = abs(p.normal.z) < 0.5
part.data.use_auto_smooth = True

bpy.ops.mesh.primitive_plane_add(size=2000, location=(82, 70, -0.04))
floor = bpy.context.object
floor_mat = bpy.data.materials.new('Warm white background')
floor_mat.diffuse_color = (0.91, 0.93, 0.95, 1)
floor.data.materials.append(floor_mat)
target = Vector((82.55, 69.85, 0))
bpy.ops.object.camera_add(location=(215, -195, 330))
camera = bpy.context.object
camera.rotation_euler = (target - camera.location).to_track_quat('-Z', 'Y').to_euler()
camera.data.type = 'ORTHO'
camera.data.ortho_scale = 226
bpy.context.scene.camera = camera
for pos, energy, size in [((-80,-40,270), 1400000, 170), ((220,160,200), 800000, 160)]:
    bpy.ops.object.light_add(type='AREA', location=pos)
    light = bpy.context.object
    light.data.energy, light.data.size = energy, size
    light.rotation_euler = (target - light.location).to_track_quat('-Z', 'Y').to_euler()
scene = bpy.context.scene
scene.render.engine = 'CYCLES'
scene.cycles.device = 'CPU'
scene.cycles.samples = 32
scene.cycles.use_denoising = True
scene.world.color = (0.6, 0.6, 0.6)
scene.view_settings.view_transform = 'Standard'
scene.view_settings.look = 'Medium High Contrast'
scene.view_settings.exposure = 0
scene.render.resolution_x, scene.render.resolution_y = 1000, 880
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = 'PNG'
scene.render.filepath = str(ROOT / 'reference' / 'plate_render.png')
bpy.ops.render.render(write_still=True)
