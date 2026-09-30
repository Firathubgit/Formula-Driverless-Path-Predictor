"""Render a loaded Urus studio from reference-facing detail cameras."""
import bpy
from pathlib import Path
from mathutils import Vector

scene = bpy.context.scene
assert scene.get('appearance_id') == 'urus'
scene.frame_set(73)
root = bpy.data.objects['MOTION_urus']
root.animation_data_clear()
root.location = (0, 0, 0)
root.rotation_euler = (0, 0, 0)
camera = scene.camera
camera.animation_data_clear()
scene.render.resolution_x, scene.render.resolution_y = 1280, 720
scene.eevee.taa_render_samples = 48
# Neutral inspection fill makes the lamp internals readable against the user's
# daylight photo. This review-only light is never saved into the animation set.
data = bpy.data.lights.new('Headlamp inspection fill', 'AREA')
data.energy, data.shape, data.size = 500, 'DISK', 3
data.specular_factor = 0.2
fill = bpy.data.objects.new(data.name, data)
scene.collection.objects.link(fill)
fill.location = (5, 0, 2.5)
fill.rotation_euler = (Vector((2.1, 0, 0.9)) - fill.location).to_track_quat('-Z', 'Y').to_euler()
out = Path(bpy.data.filepath).parent.parent / 'lamp-review'
out.mkdir(exist_ok=True)
for name, position, target, lens in [
    ('urus-front', (6.4, 0, 1.55), (1.7, 0, 1.0), 65),
    ('urus-headlight', (4.5, -1.6, 1.25), (2.12, -0.69, 0.86), 90),
]:
    camera.location = position
    camera.rotation_euler = (Vector(target) - camera.location).to_track_quat('-Z', 'Y').to_euler()
    camera.data.lens = lens
    scene.render.filepath = str(out / (name + '.png'))
    bpy.ops.render.render(write_still=True)
