"""Build the reproducible black studio and keyed clip graph in Blender 4.5.

Run through Blender --background --python this_file -- --car amr23 ...
Inputs are local prepared assets; outputs and renders stay outside Git.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import bpy
from mathutils import Vector

FPS = 24
CLIPS = {"enter": (1, 72), "idle": (73, 120), "exit": (121, 156), "select": (157, 228)}
LOOKS = {
    "amr23": (-0.12, -0.15, -10),
    "jesko": (0.08, 0.12, -28),
    "urus": (0.0, 0.0, 24),
    "rb19": (-0.12, 0.15, 160),
}

# Coordinates are relative to each car: nose +X, left/right Y, Z up.
# Each entrance has two authored details followed by its established hero.
# Selection reserves a new composition, reached by a cut, never a flying orbit.
SHOT_DESIGNS = {
    'amr23': {
        'details': [
            ('Front wing / low sculpture', (4.5, -2.6, .78), (4.25, -2.45, .72), (1.65, -.25, .28), 58),
            ('Halo / cockpit crown', (-.7, -3.1, 2.8), (-.5, -2.95, 2.7), (-.5, 0, .72), 60),
        ],
        'select': ('Low rear three-quarter', (-7.7, -4.8, 1.1), (-7.35, -4.6, 1.06), (-.35, 0, .53), 52),
        'idle': (.05, 0, .012), 'exit': (.16, -.10, .025), 'scan': 180,
    },
    'rb19': {
        'details': [
            ('Exhaust / rear aero', (-4.5, -3.1, 1.0), (-4.3, -2.95, .94), (-1.4, 0, .6), 58),
            ('Nose / overhead diagonal', (4.5, -3.2, 3.2), (4.25, -3, 3.0), (.9, 0, .35), 55),
        ],
        'select': ('Head-on low symmetry', (8.5, 0, 1.05), (8.15, 0, .98), (.35, 0, .40), 58),
        'idle': (0, -.05, .01), 'exit': (.12, -.17, -.02), 'scan': 200,
    },
    'jesko': {
        'details': [
            ('Silver alloy / black rubber', (1.65, -2.8, .68), (1.85, -2.7, .64), (1.15, -.7, .55), 58),
            ('Carbon wing / rear lamps', (-4.7, -3.3, 2.15), (-4.5, -3.15, 2.05), (-1.1, 0, .94), 58),
        ],
        'select': ('Rear symmetry / wing silhouette', (-8.8, 0, 1.12), (-8.4, 0, 1.08), (-.8, 0, .66), 65),
        'idle': (-.035, .025, .008), 'exit': (.17, -.11, .015), 'scan': 75,
    },
    'urus': {
        'details': [
            ('White Y lamp / front shoulder', (4.7, -2.3, 1.4), (4.5, -2.2, 1.35), (1.85, -.6, .93), 68),
            ('High rear quarter / roofline', (-5.4, -4.4, 3.35), (-5.2, -4.25, 3.22), (-.45, 0, .85), 55),
        ],
        'select': ('Low opposite front quarter', (7.6, 5.4, 1.45), (7.3, 5.15, 1.39), (.5, 0, .82), 52),
        'idle': (.03, .035, .015), 'exit': (.15, -.12, .025), 'scan': 90,
    },
}


def enum_set(owner, prop, value):
    values = {x.identifier for x in owner.bl_rna.properties[prop].enum_items}
    if value not in values:
        raise RuntimeError(f"{prop}: {value} unavailable; found {values}")
    setattr(owner, prop, value)


def smooth(t):
    return t * t * t * (t * (t * 6 - 15) + 10)


def mix(a, b, t):
    return Vector(a).lerp(Vector(b), t)


def principled(name, color, roughness, metallic=0.0):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    node = next(n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    node.inputs["Base Color"].default_value = (*color, 1)
    node.inputs["Roughness"].default_value = roughness
    node.inputs["Metallic"].default_value = metallic
    return mat


def aim(obj, target):
    obj.rotation_euler = (Vector(target) - obj.location).to_track_quat('-Z', 'Y').to_euler()


def area(name, position, target, energy, size, size_y, color):
    data = bpy.data.lights.new(name, 'AREA')
    enum_set(data, 'shape', 'RECTANGLE')
    data.energy, data.size, data.size_y, data.color = energy, size, size_y, color
    data.use_shadow_jitter = True
    obj = bpy.data.objects.new(name, data)
    bpy.context.scene.collection.objects.link(obj)
    obj.location = position
    aim(obj, target)
    return obj


def line(name, points, width, material):
    data = bpy.data.curves.new(name, 'CURVE')
    data.dimensions = '3D'
    data.bevel_depth = width
    data.bevel_resolution = 3
    spline = data.splines.new('POLY')
    spline.points.add(len(points) - 1)
    for p, xyz in zip(spline.points, points):
        p.co = (*xyz, 1)
    obj = bpy.data.objects.new(name, data)
    bpy.context.scene.collection.objects.link(obj)
    data.materials.append(material)
    return obj


def emission(name, color, strength):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    nodes.clear()
    em = nodes.new('ShaderNodeEmission')
    em.inputs[0].default_value = (*color, 1)
    em.inputs[1].default_value = strength
    out = nodes.new('ShaderNodeOutputMaterial')
    mat.node_tree.links.new(em.outputs[0], out.inputs[0])
    return mat


def block(name, position, dimensions, material, rotation=(0, 0, 0)):
    bpy.ops.mesh.primitive_cube_add(size=1, location=position)
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = dimensions
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    obj.rotation_euler = rotation
    obj.data.materials.append(material)
    bevel = obj.modifiers.new('Machined edges', 'BEVEL')
    bevel.width, bevel.segments = 0.055, 3
    obj.modifiers.new('Weighted normals', 'WEIGHTED_NORMAL')
    return obj


def local_world(point, origin, yaw):
    x, y, z = point
    return origin + Vector((x * math.cos(yaw) - y * math.sin(yaw),
                            x * math.sin(yaw) + y * math.cos(yaw), z))


def horizon_studio(car, floor_material):
    """A continuous 360-degree sweep; hue lives above the neutral floor.

    World-space height keeps the horizon treatment coherent through every
    camera cut and orbit. No colored lamp changes the car's authored livery.
    """
    profile = [(10 + 4 * math.sin(i * math.pi / 64),
                4 * (1 - math.cos(i * math.pi / 64))) for i in range(33)]
    profile += [(14, 8), (14, 16)]
    count = 192
    verts = [(r * math.cos(a * 2 * math.pi / count),
              r * math.sin(a * 2 * math.pi / count), z)
             for r, z in profile for a in range(count)]
    faces = [(j * count + a, (j + 1) * count + a,
              (j + 1) * count + (a + 1) % count, j * count + (a + 1) % count)
             for j in range(len(profile) - 1) for a in range(count)]
    mesh = bpy.data.meshes.new('Continuous studio sweep')
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    obj = bpy.data.objects.new('Horizon cyclorama | neutral ground', mesh)
    bpy.context.scene.collection.objects.link(obj)
    for polygon in mesh.polygons:
        polygon.use_smooth = True
    mat = floor_material.copy()
    mat.name = f'Horizon | {car} | height controlled'
    obj.data.materials.append(mat)
    nodes, links = mat.node_tree.nodes, mat.node_tree.links
    surface = next(n for n in nodes if n.type == 'BSDF_PRINCIPLED')
    output = next(n for n in nodes if n.type == 'OUTPUT_MATERIAL')
    position = nodes.new('ShaderNodeNewGeometry')
    xyz = nodes.new('ShaderNodeSeparateXYZ')
    links.new(position.outputs['Position'], xyz.inputs[0])
    height = nodes.new('ShaderNodeMapRange')
    height.inputs['From Max'].default_value = 5.0
    links.new(xyz.outputs['Z'], height.inputs['Value'])
    ramp = nodes.new('ShaderNodeValToRGB')
    ramp.name = 'Low horizon glow fading upward'
    ramp.color_ramp.interpolation = 'B_SPLINE'
    levels = [(0, .65), (.05, 1), (.13, .65), (.30, .12), (.55, 0), (1, 0)]
    for element in list(ramp.color_ramp.elements)[2:]:
        ramp.color_ramp.elements.remove(element)
    for i, (at, value) in enumerate(levels):
        element = ramp.color_ramp.elements[i] if i < 2 else ramp.color_ramp.elements.new(at)
        element.position = at
        element.color = (value, value, value, 1)
    links.new(height.outputs[0], ramp.inputs[0])
    palette = {
        'amr23': ((.006, .045, .024), (.006, .045, .024)),
        'rb19': ((.023, .012, .015), (.011, .016, .029)),
        'jesko': ((.035, .036, .038), (.035, .036, .038)),
        'urus': ((.035, .036, .038), (.035, .036, .038)),
    }
    horizontal = nodes.new('ShaderNodeVectorMath')
    horizontal.operation = 'DOT_PRODUCT'
    horizontal.inputs[1].default_value = (.76, .65, 0)
    links.new(position.outputs['Position'], horizontal.inputs[0])
    across = nodes.new('ShaderNodeMapRange')
    across.inputs['From Min'].default_value = -9
    across.inputs['From Max'].default_value = 9
    links.new(horizontal.outputs['Value'], across.inputs['Value'])
    hue = nodes.new('ShaderNodeMixRGB')
    hue.inputs[1].default_value = (*palette[car][0], 1)
    hue.inputs[2].default_value = (*palette[car][1], 1)
    links.new(across.outputs[0], hue.inputs[0])
    glow = nodes.new('ShaderNodeMixRGB')
    glow.inputs[1].default_value = (.0007, .0008, .001, 1)
    links.new(ramp.outputs[0], glow.inputs[0])
    links.new(hue.outputs[0], glow.inputs[2])
    em = nodes.new('ShaderNodeEmission')
    links.new(glow.outputs[0], em.inputs[0])
    blend = nodes.new('ShaderNodeMapRange')
    # A metres-wide blend across the curved sweep avoids a visible horizon
    # edge. The previous 8 cm shader transition read like a nearby black lip.
    blend.inputs['From Max'].default_value = .65
    blend.interpolation_type = 'SMOOTHSTEP'
    links.new(xyz.outputs['Z'], blend.inputs['Value'])
    shader = nodes.new('ShaderNodeMixShader')
    links.new(blend.outputs[0], shader.inputs[0])
    links.new(surface.outputs[0], shader.inputs[1])
    links.new(em.outputs[0], shader.inputs[2])
    links.new(shader.outputs[0], output.inputs['Surface'])


def orbit_between(start, end, origin, t):
    a, b = Vector(start) - origin, Vector(end) - origin
    angle_a, angle_b = math.atan2(a.y, a.x), math.atan2(b.y, b.x)
    turn = math.atan2(math.sin(angle_b - angle_a), math.cos(angle_b - angle_a))
    radius = math.hypot(a.x, a.y) * (1 - t) + math.hypot(b.x, b.y) * t
    angle = angle_a + turn * t
    return origin + Vector((radius * math.cos(angle), radius * math.sin(angle), a.z * (1 - t) + b.z * t))


def build(args):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.name = f"SHOWROOM_{args.car}"
    enum_set(scene.unit_settings, 'system', 'METRIC')
    scene.unit_settings.scale_length = 1.0
    with bpy.data.libraries.load(str(args.source), link=False) as (src, dst):
        dst.collections = [n for n in src.collections if n == f"CAR_{args.car}"]
    if len(dst.collections) != 1:
        raise RuntimeError("Prepared CAR collection missing")
    collection = dst.collections[0]
    scene.collection.children.link(collection)
    root = bpy.data.objects.new(f"MOTION_{args.car}", None)
    scene.collection.objects.link(root)
    for obj in collection.all_objects:
        if obj.parent is None:
            obj.parent = root

    scene.render.engine = 'BLENDER_EEVEE_NEXT'
    scene.eevee.taa_render_samples = args.samples
    scene.eevee.use_raytracing = True
    scene.eevee.shadow_ray_count = 4
    scene.eevee.shadow_step_count = 12
    scene.eevee.ray_tracing_options.screen_trace_quality = 1.0
    scene.render.resolution_x = args.width
    scene.render.resolution_y = args.width * 9 // 16
    scene.render.resolution_percentage = 100
    scene.render.fps = FPS
    scene.render.film_transparent = False
    # Blender's default output dither can turn mathematical black into RGB 1.
    scene.render.dither_intensity = 0.0
    enum_set(scene.render.image_settings, 'file_format', 'PNG')
    enum_set(scene.render.image_settings, 'color_mode', 'RGB')
    scene.render.image_settings.color_depth = '8'
    scene.render.image_settings.compression = 20
    scene.world = bpy.data.worlds.new('Black infinity')
    scene.world.use_nodes = True
    background = next(n for n in scene.world.node_tree.nodes if n.type == 'BACKGROUND')
    background.inputs['Color'].default_value = (0.015, 0.018, 0.022, 1)
    background.inputs['Strength'].default_value = 0.10
    # Retain the factory scene's AgX view transform (the dynamic RNA enum under-reports it).
    scene.view_settings.exposure = 0.0

    gt_studio = args.look == 'gt-studio'
    floor_mat = principled('Graphite reflection floor | studio' if gt_studio else 'Obsidian satin | studio',
                           (0.008, 0.009, 0.011) if gt_studio else (0.002, 0.0025, 0.003),
                           0.29 if gt_studio else 0.85, 0.0)
    floor_shader = next(n for n in floor_mat.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
    floor_shader.inputs['Specular IOR Level'].default_value = 0.35 if gt_studio else 0.0
    # The large seamless floor and broad rectangular reflections echo the supplied pink studio.
    bpy.ops.mesh.primitive_plane_add(size=2000, location=(0, 0, 0))
    floor = bpy.context.object
    floor.name = 'Infinite graphite floor'
    floor.data.materials.append(floor_mat)
    ceiling = area('Ceiling softbox', (0, 0, 6.5), (0, 0, 0), 300, 7.5, 0.4, (0.96, 0.98, 1.0))
    front = area('Front sculpting strip', (4, -5, 3.8), (0, 0, 0.5), 450, 6.5, 0.35, (1.0, 0.98, 0.96))
    rim = area('Far shoulder strip', (-1.5, 4, 3.4), (0, 0, 0.6), 900, 7.0, 0.25, (0.92, 0.96, 1.0))
    nose = area('Nose edge', (5, 2.0, 1.7), (0, 0, 0.7), 180, 3.2, 0.3, (1.0, 1.0, 1.0))
    fill = area('Camera fill', (1, -7, 3), (0, 0, 0.7), 260, 5.5, 4.0, (0.98, 0.99, 1.0))
    if hasattr(fill.data, 'specular_factor'):
        fill.data.specular_factor = 0.0
    scan = area('Moving detail reveal', (1, -3, 1.8), (0, 0, 0.6), 0, 0.16, 3.6, (0.92, 0.97, 1.0))
    if args.car in ('jesko', 'urus'):
        # Broad carbon panels must remain black. Control the studio's reflected
        # radiance independently of the diffuse fill that exposes fine detail.
        for light, share in ((ceiling, .015), (front, .3), (rim, .35), (nose, .25), (scan, .04)):
            light.data.specular_factor = share
    if gt_studio:
        # The Porsche references use a small number of long, sharply bounded
        # reflection cards. Broad pale windows and silver-looking black paint
        # came from the old ceiling and large camera-facing softbox.
        ceiling.data.energy = 145
        ceiling.data.size = 7.5
        ceiling.data.size_y = 0.12
        front.data.energy = 510
        front.data.size = 5.2
        front.data.size_y = 0.16
        rim.data.energy = 720
        rim.data.size = 6.5
        rim.data.size_y = 0.14
        nose.data.energy = 210
        nose.data.size = 2.8
        nose.data.size_y = 0.12
        fill.data.energy = 390
        fill.data.size = 6.0
        fill.data.size_y = 4.0
        for light in (ceiling, front, rim, nose, scan):
            light.data.specular_factor = 0.24 if args.car in ('jesko', 'urus') else 0.65
        area('Low white wheel card', (1.6, -4.6, 0.7), (0, 0, 0.45),
             105, 5.5, 0.12, (0.91, 0.95, 1.0)).data.specular_factor = 0.2
        area('Rear lamp edge card', (-5, -1.2, 1.9), (-1, 0, 0.8),
             155, 2.5, 0.15, (1.0, 0.95, 0.92)).data.specular_factor = 0.2

    # Original dark architectural set: staggered basalt fins and fractured light seams.
    wall_mat = principled('Basalt | charcoal architecture', (0.009, 0.012, 0.016), 0.48, 0.38)
    wall_nodes = wall_mat.node_tree.nodes
    wall_shader = next(n for n in wall_nodes if n.type == 'BSDF_PRINCIPLED')
    noise = wall_nodes.new('ShaderNodeTexNoise')
    noise.inputs['Scale'].default_value = 34
    bump = wall_nodes.new('ShaderNodeBump')
    bump.inputs['Strength'].default_value = 0.13
    bump.inputs['Distance'].default_value = 0.035
    wall_mat.node_tree.links.new(noise.outputs['Fac'], bump.inputs['Height'])
    wall_mat.node_tree.links.new(bump.outputs['Normal'], wall_shader.inputs['Normal'])
    seam_mat = emission('Architectural seam | cold pearl', (0.36, 0.47, 0.57), 1.6)
    for i, (x, y, height, tilt) in enumerate([(-8.5, 6.3, 8, -14), (-4.6, 8.0, 10, 12),
                                            (0.2, 9.0, 8.8, -18), (5.2, 10.5, 11, 14), (9, 9, 8, -12)]):
        panel = block(f'Obsidian fin {i + 1}', (x, y, height / 2), (1.8, 0.75, height), wall_mat,
                      (0, math.radians(tilt), math.radians(-14)))
        edge = line(f'Light fracture {i + 1}', [(-0.94, -0.40, -height * 0.35),
                    (-0.94, -0.40, height * 0.18), (-0.3, -0.40, height * 0.46)], 0.016, seam_mat)
        edge.parent = panel
    # Low angular walls anchor the scale while leaving the car's entire silhouette clear.
    block('Left folded wall', (-8, 2.5, 1.0), (5.5, 0.5, 2.0), wall_mat, (0, math.radians(-12), math.radians(25)))
    block('Far floating beam', (0, 8, 5.8), (13.5, 0.65, 0.55), wall_mat, (0, 0, math.radians(-8)))
    architecture_wash = area('Architecture edge wash', (-4, 1.5, 7.5), (0, 8, 3), 1300, 7, 0.4, (0.55, 0.70, 0.86))
    if args.car in ('jesko', 'urus'):
        architecture_wash.data.specular_factor = 0.35
    # Two tightly controlled shafts in a very thin local atmosphere, rather than a bright ambient world.
    for i, (position, target) in enumerate([((-5, 3.2, 8.5), (-3, 0, 0)), ((6, 6, 9), (3, 2, 0))]):
        data = bpy.data.lights.new(f'Vault shaft {i + 1}', 'SPOT')
        data.energy, data.spot_size, data.spot_blend = 2200, math.radians(20), 0.55
        data.color, data.shadow_soft_size = (0.67, 0.80, 1.0), 0.12
        if args.car in ('jesko', 'urus'):
            data.specular_factor = .03
        obj = bpy.data.objects.new(data.name, data)
        scene.collection.objects.link(obj)
        obj.location = position
        aim(obj, target)
    atmosphere = bpy.data.materials.new('Thin vault atmosphere')
    atmosphere.use_nodes = True
    atmosphere.node_tree.nodes.clear()
    volume = atmosphere.node_tree.nodes.new('ShaderNodeVolumePrincipled')
    volume.inputs['Density'].default_value = 0.002
    volume.inputs['Anisotropy'].default_value = 0.3
    output = atmosphere.node_tree.nodes.new('ShaderNodeOutputMaterial')
    atmosphere.node_tree.links.new(volume.outputs['Volume'], output.inputs['Volume'])
    bpy.ops.mesh.primitive_cube_add(size=1, location=(0, 9, 4.5))
    fog = bpy.context.object
    fog.name = 'Local atmosphere | light shafts'
    fog.scale = (22, 7, 9)
    fog.data.materials.append(atmosphere)

    # Fractured floor inlays echo the architecture without encircling the car.
    arc_mat = emission('Floor inlay | neutral silver', (0.25, 0.29, 0.33), 0.35)
    line('Left floor fracture', [(-9, 6, .002), (-5, 3, .002), (-5, -1, .002)], .007, arc_mat)
    line('Right floor fracture', [(8, 5, .002), (6, 1, .002), (6, -4, .002)], .007, arc_mat)
    if gt_studio:
        # Keep the connected black transitions, but remove the old vault and
        # floor graphics so the actual car panels, wheels and lamps lead each cut.
        for obj in scene.objects:
            if obj.name.startswith(('Obsidian fin', 'Light fracture', 'Left folded wall', 'Far floating beam',
                                    'Left floor fracture', 'Right floor fracture',
                                    'Local atmosphere', 'Vault shaft', 'Architecture edge wash')):
                obj.hide_render = True
        architecture_wash.data.energy = 0
        background.inputs['Strength'].default_value = 0.035
        horizon_studio(args.car, floor_mat)

    camera_data = bpy.data.cameras.new('Showroom camera')
    camera = bpy.data.objects.new('Showroom camera', camera_data)
    scene.collection.objects.link(camera)
    scene.camera = camera
    camera_data.lens = 58
    camera_data.clip_end = 300
    hx, hy, heading = LOOKS[args.car]
    hero_pos = Vector((hx, hy, 0))
    hero_yaw = math.radians(heading)
    hero_cam = Vector((5.9, -6.9, 2.65 if args.car == 'urus' else 2.15)) if gt_studio else Vector((7.0, -8.8, 3.4 if args.car == 'urus' else 2.65))
    hero_target = Vector((0, 0, 0.65 if args.car == 'urus' else 0.38))
    design = SHOT_DESIGNS[args.car]

    def detail_camera(shot, progress):
        _, start, end, focus, lens = shot
        camera.location = local_world(mix(start, end, smooth(progress)), hero_pos, hero_yaw)
        camera.data.lens = lens
        return local_world(focus, hero_pos, hero_yaw)

    # Composite fade is keyed, so every canonical dark endpoint is literally RGB zero.
    scene.use_nodes = True
    nodes = scene.node_tree.nodes
    nodes.clear()
    render = nodes.new('CompositorNodeRLayers')
    fade = nodes.new('CompositorNodeMixRGB')
    fade.name = 'CANONICAL_BLACK_GATE'
    fade.inputs[1].default_value = (0, 0, 0, 1)
    scene.node_tree.links.new(render.outputs['Image'], fade.inputs[2])
    out = nodes.new('CompositorNodeComposite')
    scene.node_tree.links.new(fade.outputs[0], out.inputs[0])

    for clip, (start, end) in CLIPS.items():
        scene.timeline_markers.new(f'{clip.upper()} | {args.car}', frame=start)
        for frame in range(start, end + 1):
            t = (frame - start) / (end - start)
            u = smooth(t)
            root.location = hero_pos
            root.rotation_euler = (0, 0, hero_yaw)
            camera.location = hero_cam
            camera.data.lens = 58
            target = hero_target.copy()
            gate = 1.0
            rim.data.energy = 720 if gt_studio else 900
            scan.data.energy = 0
            if clip == 'enter':
                if gt_studio:
                    if frame <= 24:
                        target = detail_camera(design['details'][0], (frame - 1) / 23)
                    elif frame <= 48:
                        target = detail_camera(design['details'][1], (frame - 25) / 23)
                    else:
                        # A short independent push settles exactly on the hero,
                        # rather than interpolating from the preceding detail.
                        q = smooth((frame - 49) / 23)
                        camera.location = hero_cam + (hero_cam - hero_target).normalized() * .22 * (1 - q)
                    gate = smooth(min(1, t / 0.11))
                else:
                    # Original continuous tyre/sidepod reveal.
                    macro_end = local_world((0.2, -2.9, 1.0), hero_pos, hero_yaw)
                    focus_end = local_world((0.0, -0.65, 0.62), hero_pos, hero_yaw)
                    if t <= 0.3:
                        macro_t = smooth(t / 0.3)
                        camera.location = mix(local_world((1.7, -2.25, 0.62), hero_pos, hero_yaw), macro_end, macro_t)
                        target = mix(local_world((1.35, -0.65, 0.56), hero_pos, hero_yaw), focus_end, macro_t)
                    else:
                        reveal = smooth((t - 0.3) / 0.7)
                        camera.location = orbit_between(macro_end, hero_cam, hero_pos, reveal)
                        target = mix(focus_end, hero_target, reveal)
                    gate = smooth(min(1, t / 0.18))
                scan.location = local_world((-2.4 + 5.5 * t, -3, 1.8), hero_pos, hero_yaw)
                aim(scan, local_world((-2.4 + 5.5 * t, 0, 0.6), hero_pos, hero_yaw))
                scan.data.energy = (design['scan'] if gt_studio else 1600) * math.sin(math.pi * t) ** 2
            elif clip == 'idle':
                # Closed orbit and light breathe: first/last transforms match hero exactly.
                pulse = math.sin(math.pi * t) ** 2
                camera.location += Vector(design['idle']) * pulse
                rim.data.energy = (720 if gt_studio else 900) + 25 * pulse
            elif clip == 'exit':
                camera.location += Vector(design['exit']) * u
                gate = 1 - smooth(max(0, (t - 0.15) / 0.85))
            elif clip == 'select':
                # Six hero frames preserve the queued idle -> select seam.
                # Then an editorial cut reveals this car's reserved composition.
                if frame >= 163:
                    target = detail_camera(design['select'], (frame - 163) / 65)
                gate = 1 - smooth(max(0, (t - 0.78) / 0.22))
            aim(camera, target)
            for obj in (root, camera):
                obj.keyframe_insert('location', frame=frame)
                obj.keyframe_insert('rotation_euler', frame=frame)
            camera.data.keyframe_insert('lens', frame=frame)
            fade.inputs[0].default_value = gate
            fade.inputs[0].keyframe_insert('default_value', frame=frame)
            rim.data.keyframe_insert('energy', frame=frame)
            scan.keyframe_insert('location', frame=frame)
            scan.keyframe_insert('rotation_euler', frame=frame)
            scan.data.keyframe_insert('energy', frame=frame)

    scene.frame_start, scene.frame_end = 1, 228
    scene.frame_preview_start, scene.frame_preview_end = CLIPS['idle']
    scene.use_preview_range = True
    scene['clip_contract'] = json.dumps(CLIPS)
    scene['appearance_id'] = args.car
    scene['handoff_note'] = 'Reserved car angle, then fade to black. The car stays parked; no studio racing line.'
    scene['shot_design'] = json.dumps(design)
    # Baked frame-by-frame camera keys: linear small moves and explicit hard cuts
    # cannot acquire Bezier overshoot when scrubbing or rendering subframes.
    for owner in (camera, camera.data):
        for curve in owner.animation_data.action.fcurves:
            for key in curve.keyframe_points:
                key.interpolation = 'CONSTANT' if int(key.co.x) in (24, 48, 162) else 'LINEAR'
    scene.frame_set(CLIPS['idle'][0])
    # Save an immediately usable camera view in the Blender workspace.
    for screen in bpy.data.screens:
        for view in screen.areas:
            if view.type == 'VIEW_3D':
                view.spaces.active.region_3d.view_perspective = 'CAMERA'
    args.out.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(args.out / f'{args.car}-studio.blend'), compress=True)
    (args.out / f'{args.car}-timeline.json').write_text(json.dumps({
        'car': args.car, 'fps': FPS, 'width': args.width, 'height': scene.render.resolution_y,
        'samples': args.samples, 'clips': CLIPS, 'look': args.look,
        'shot_design': design,
    }, indent=2), encoding='utf-8')
    if args.render:
        render_media(args)


def render_media(args):
    scene = bpy.context.scene
    scene.use_preview_range = False
    scene.render.dither_intensity = 0.0
    scene.render.resolution_x = args.width
    scene.render.resolution_y = args.width * 9 // 16
    scene.eevee.taa_render_samples = args.samples
    frames = args.out / 'frames' / args.car
    frames.mkdir(parents=True, exist_ok=True)
    if args.frames:
        for frame in args.frames:
            scene.frame_set(frame)
            scene.render.filepath = str(frames / f'{frame:04d}.png')
            bpy.ops.render.render(write_still=True)
    elif args.poster_only:
        scene.frame_set(CLIPS['idle'][0])
        scene.render.filepath = str(frames / f"{CLIPS['idle'][0]:04d}.png")
        bpy.ops.render.render(write_still=True)
    else:
        if args.preview_first:
            scene.frame_set(CLIPS['idle'][0])
            scene.render.filepath = str(frames / f"{CLIPS['idle'][0]:04d}.png")
            bpy.ops.render.render(write_still=True)
        scene.render.filepath = str(frames / '') + '/'
        scene.frame_start = args.start or 1
        scene.frame_end = args.end or 228
        bpy.ops.render.render(animation=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--car', choices=list(LOOKS), required=True)
    parser.add_argument('--source', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--samples', type=int, default=32)
    parser.add_argument('--look', choices=('black-vault', 'gt-studio'), default='gt-studio')
    parser.add_argument('--render', action='store_true')
    parser.add_argument('--render-existing', action='store_true')
    parser.add_argument('--poster-only', action='store_true')
    parser.add_argument('--preview-first', action='store_true')
    parser.add_argument('--start', type=int)
    parser.add_argument('--end', type=int)
    parser.add_argument('--frames', type=int, nargs='+')
    options = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
    repository = Path(__file__).resolve().parents[2]
    options.out = (repository / options.out).resolve()
    if options.source:
        options.source = (repository / options.source).resolve()
    if options.render_existing:
        render_media(options)
    else:
        build(options)
