"""Render neutral diagnostic angles of the four local, packed showroom assets.

This is an inspection tool, separate from the cinematic stage. It does not
modify the supplied models or the active media pack.
"""
import argparse
import json
import sys
from collections import Counter
from pathlib import Path

import bpy
from mathutils import Vector


ROOT = Path(__file__).resolve().parents[2]


def point_at(obj, target):
    obj.rotation_euler = (Vector(target) - obj.location).to_track_quat('-Z', 'Y').to_euler()


def area(name, location, target, energy, size, size_y, color):
    light = bpy.data.lights.new(name, 'AREA')
    light.shape = 'RECTANGLE'
    light.energy = energy
    light.size = size
    light.size_y = size_y
    light.color = color
    obj = bpy.data.objects.new(name, light)
    bpy.context.scene.collection.objects.link(obj)
    obj.location = location
    point_at(obj, target)


def main(car, out, samples):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    source = ROOT / 'artifacts/showroom/cars' / f'{car}.blend'
    with bpy.data.libraries.load(str(source), link=False) as (available, imported):
        imported.collections = [f'CAR_{car}']
    if len(imported.collections) != 1:
        raise RuntimeError(f'Missing prepared car: {car}')
    scene.collection.children.link(imported.collections[0])
    meshes = [obj for obj in imported.collections[0].all_objects if obj.type == 'MESH']
    if len(meshes) != 1:
        raise RuntimeError(f'Expected one normalized mesh, found {len(meshes)}')
    car_mesh = meshes[0]
    scene.render.engine = 'BLENDER_EEVEE_NEXT'
    scene.eevee.taa_render_samples = samples
    scene.eevee.use_raytracing = True
    scene.eevee.shadow_ray_count = 4
    scene.eevee.shadow_step_count = 12
    scene.render.resolution_x = 960
    scene.render.resolution_y = 600
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = 'PNG'
    scene.render.image_settings.color_mode = 'RGB'
    scene.render.dither_intensity = 0
    scene.world = bpy.data.worlds.new('Neutral reference world')
    scene.world.use_nodes = True
    world_node = next(node for node in scene.world.node_tree.nodes if node.type == 'BACKGROUND')
    world_node.inputs['Color'].default_value = (0.14, 0.15, 0.17, 1)
    world_node.inputs['Strength'].default_value = 0.35
    floor = bpy.data.materials.new('Neutral floor')
    floor.use_nodes = True
    shader = next(node for node in floor.node_tree.nodes if node.type == 'BSDF_PRINCIPLED')
    shader.inputs['Base Color'].default_value = (0.075, 0.077, 0.08, 1)
    shader.inputs['Metallic'].default_value = 0.0
    shader.inputs['Roughness'].default_value = 0.45
    bpy.ops.mesh.primitive_plane_add(size=200, location=(0, 0, -0.015))
    bpy.context.object.name = 'Neutral backdrop'
    bpy.context.object.data.materials.append(floor)
    area('Long ceiling reflection', (0, 0, 7), (0, 0, 0), 900, 8, 1.8, (1, 1, 1))
    area('Camera-side softbox', (1, -5, 4), (0, 0, 0.7), 700, 6, 3, (0.95, 0.97, 1))
    area('Rear contour', (-4, 2.5, 3), (0, 0, 0.8), 1100, 5, 0.6, (0.75, 0.84, 1))
    area('Front contour', (4, 2, 4), (0, 0, 0.8), 750, 5, 0.7, (1, 0.96, 0.88))
    camera_data = bpy.data.cameras.new('Diagnostic camera')
    camera = bpy.data.objects.new('Diagnostic camera', camera_data)
    scene.collection.objects.link(camera)
    scene.camera = camera
    camera_data.type = 'ORTHO'
    height = car_mesh.dimensions.z
    views = {
        'front': ((8, -0.2, max(1.25, height * 0.58)), (0, 0, height * 0.48), 3.6),
        'rear': ((-8, 0.2, max(1.25, height * 0.58)), (0, 0, height * 0.48), 3.6),
        'side': ((0, -9, max(1.5, height * 0.7)), (0, 0, height * 0.48), 7.2),
        'front-quarter': ((7, -7, max(2.3, height * 1.1)), (0, 0, height * 0.45), 6.7),
        'rear-quarter': ((-7, -7, max(2.3, height * 1.1)), (0, 0, height * 0.45), 6.7),
        'front-wheel': ((3.1, -3.2, 1.15), (1.35, -0.85, 0.45), 2.8),
    }
    out.mkdir(parents=True, exist_ok=True)
    for name, (location, target, scale) in views.items():
        camera.location = location
        point_at(camera, target)
        camera_data.ortho_scale = scale
        scene.render.filepath = str(out / f'{car}-{name}.png')
        bpy.ops.render.render(write_still=True)
    counts = Counter(poly.material_index for poly in car_mesh.data.polygons)
    report = {
        'car': car,
        'source': str(source),
        'mesh_dimensions_m': list(car_mesh.dimensions),
        'polygon_count': len(car_mesh.data.polygons),
        'materials': [
            {'slot': slot, 'name': material.name if material else None,
             'faces': counts[slot],
             'images': sorted({node.image.name for node in material.node_tree.nodes
                               if node.type == 'TEX_IMAGE' and node.image}) if material and material.use_nodes else []}
            for slot, material in enumerate(car_mesh.data.materials)
        ],
    }
    (out / f'{car}-materials.json').write_text(json.dumps(report, indent=2), encoding='utf-8')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--car', choices=('amr23', 'jesko', 'urus', 'rb19'), required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=24)
    options = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
    main(options.car, (ROOT / options.out).resolve(), options.samples)
