"""Prepare local, privately supplied showroom meshes with Blender 4.5.

Run using Blender --background --python tools/showroom/prepare_cars.py -- --car ID.
Sources remain untouched. Derived assets and audits live under ignored artifacts/.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import bpy
from mathutils import Matrix, Vector


ROOT = Path(__file__).resolve().parents[2]
DOWNLOADS = Path.home() / "Downloads"
SOURCES = {
    "amr23": ("aston-martin-f1-amr23-2023/source/Sketchfab_2024_04_05_23_03_10.blend", 5.6),
    "jesko": ("2024-koenigsegg-jesko-attack-7378/source/2024 Koenigsegg Jesko Attack.glb", 4.61),
    "urus": ("2023-lamborghini-urus/source/2023_lamborghini_urus_performante.glb", 5.14),
    "rb19": ("redbull-rb19-oracle-wwwvecarzcom/source/oracle_redbull_rb19.glb", 5.6),
}


def bounds(objects):
    points = [o.matrix_world @ Vector(v) for o in objects if o.type == "MESH" for v in o.bound_box]
    return [min(v[i] for v in points) for i in range(3)], [max(v[i] for v in points) for i in range(3)]


def inspect(car, source):
    meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    lo, hi = bounds(meshes)
    return {
        "id": car, "source": str(source), "bounds_min": lo, "bounds_max": hi,
        "dimensions": [hi[i] - lo[i] for i in range(3)],
        "objects": [{"name": o.name, "type": o.type,
                     "location": list(o.matrix_world.translation),
                     "dimensions": list(o.dimensions),
                     "materials": [m.name if m else None for m in o.data.materials],
                     "vertices": len(o.data.vertices), "hidden": o.hide_render} for o in meshes],
        "materials": [{"name": m.name, "nodes": [{"type": n.type, "name": n.name,
                       "image": n.image.name if n.type == "TEX_IMAGE" and n.image else None,
                       "base": list(n.inputs["Base Color"].default_value) if n.type == "BSDF_PRINCIPLED" else None,
                       "roughness": n.inputs["Roughness"].default_value if n.type == "BSDF_PRINCIPLED" else None,
                       "alpha": n.inputs["Alpha"].default_value if n.type == "BSDF_PRINCIPLED" else None}
                      for n in m.node_tree.nodes] if m.use_nodes else []} for m in bpy.data.materials],
        "images": [{"name": i.name, "filepath": i.filepath, "packed": bool(i.packed_file),
                    "size": list(i.size)} for i in bpy.data.images],
    }


def darken_texture(material, principled, preserve_neutral=True):
    """Desaturate paint, preserving grayscale decals and existing PBR channels."""
    tree = material.node_tree
    color = principled.inputs["Base Color"]
    if color.is_linked:
        source = color.links[0].from_socket
        hue = tree.nodes.new("ShaderNodeHueSaturation")
        hue.name = "Showroom dark paint / retained texture"
        hue.inputs["Saturation"].default_value = 0.035
        tree.links.new(source, hue.inputs["Color"])
        multiply = tree.nodes.new("ShaderNodeMixRGB")
        multiply.blend_type = "MULTIPLY"
        multiply.inputs[0].default_value = 1
        multiply.inputs[2].default_value = (0.08, 0.092, 0.11, 1)
        tree.links.new(hue.outputs[0], multiply.inputs[1])
        if preserve_neutral:
            hsv = tree.nodes.new("ShaderNodeSeparateColor")
            hsv.mode = "HSV"
            tree.links.new(source, hsv.inputs[0])
            ramp = tree.nodes.new("ShaderNodeMapRange")
            ramp.inputs["From Min"].default_value = 0.08
            ramp.inputs["From Max"].default_value = 0.42
            ramp.inputs["To Min"].default_value = 0.0
            ramp.inputs["To Max"].default_value = 1.0
            tree.links.new(hsv.outputs[1], ramp.inputs["Value"])
            tree.links.new(ramp.outputs[0], multiply.inputs[0])
        tree.links.new(multiply.outputs[0], color)
    else:
        color.default_value = (0.012, 0.016, 0.021, 1)


def black_lacquer(material, principled, metallic_finish=False):
    """Deep black road-car lacquer; the metallic substrate stays restrained."""
    tree = material.node_tree
    color = principled.inputs["Base Color"]
    if color.is_linked:
        source = color.links[0].from_socket
        luminance = tree.nodes.new("ShaderNodeRGBToBW")
        tree.links.new(source, luminance.inputs[0])
        level = tree.nodes.new("ShaderNodeMapRange")
        level.name = "Deep black lacquer / original paint variation"
        level.inputs["To Min"].default_value = 0.0025
        level.inputs["To Max"].default_value = 0.006
        tree.links.new(luminance.outputs[0], level.inputs["Value"])
        tint = tree.nodes.new("ShaderNodeMixRGB")
        tint.blend_type = "MULTIPLY"
        tint.inputs[0].default_value = 1
        tint.inputs[2].default_value = (0.75, 0.9, 1, 1)
        tree.links.new(level.outputs[0], tint.inputs[1])
        tree.links.new(tint.outputs[0], color)
    else:
        color.default_value = (0.003, 0.004, 0.005, 1)
    metallic = principled.inputs["Metallic"]
    if metallic.is_linked:
        source = metallic.links[0].from_socket
        reduction = tree.nodes.new("ShaderNodeMath")
        reduction.operation = "MULTIPLY"
        reduction.inputs[1].default_value = 0.2
        tree.links.new(source, reduction.inputs[0])
        tree.links.new(reduction.outputs[0], metallic)
    else:
        metallic.default_value = 0.15
    principled.inputs["Coat Weight"].default_value = 0.3
    principled.inputs["Coat Roughness"].default_value = 0.12
    if not principled.inputs["Roughness"].is_linked:
        principled.inputs["Roughness"].default_value = 0.24
    if metallic_finish:
        material["showroom_role"] = "black_metallic_lacquer"
        # A metallic base beneath a smooth dielectric clearcoat. Keep the
        # existing paint variation and normals, with fine flake variation rather
        # than making the entire body a polished mirror.
        for link in list(metallic.links):
            tree.links.remove(link)
        metallic.default_value = 0.72
        principled.inputs["Coat Weight"].default_value = 0.8
        principled.inputs["Coat Roughness"].default_value = 0.095
        noise = tree.nodes.new("ShaderNodeTexNoise")
        noise.name = "Metallic lacquer / fine flake distribution"
        noise.inputs["Scale"].default_value = 950
        noise.inputs["Detail"].default_value = 2
        rough = tree.nodes.new("ShaderNodeMapRange")
        rough.name = "Metallic lacquer / satin metal under clearcoat"
        rough.inputs["To Min"].default_value = 0.20
        rough.inputs["To Max"].default_value = 0.29
        tree.links.new(noise.outputs["Fac"], rough.inputs["Value"])
        tree.links.new(rough.outputs[0], principled.inputs["Roughness"])
        bump = tree.nodes.new("ShaderNodeBump")
        bump.name = "Metallic lacquer / microscopic flake orientation"
        bump.inputs["Distance"].default_value = 0.000015
        bump.inputs["Strength"].default_value = 0.08
        if principled.inputs["Normal"].is_linked:
            tree.links.new(principled.inputs["Normal"].links[0].from_socket, bump.inputs["Normal"])
        tree.links.new(noise.outputs["Fac"], bump.inputs["Height"])
        tree.links.new(bump.outputs[0], principled.inputs["Normal"])


def wheel_surface(material, principled, role):
    """Black molded rubber and silver alloy rims retain separate responses."""
    tree = material.node_tree
    rubber = role == "black_rubber"
    material["showroom_role"] = role
    color = principled.inputs["Base Color"]
    source = color.links[0].from_socket if color.is_linked else None
    if source:
        gray = tree.nodes.new("ShaderNodeRGBToBW")
        gray.name = role + " / source markings"
        tree.links.new(source, gray.inputs[0])
        level = tree.nodes.new("ShaderNodeMapRange")
        level.name = role + " / bounded albedo"
        level.inputs["To Min"].default_value = 0.0015 if rubber else 0.30
        level.inputs["To Max"].default_value = 0.0045 if rubber else 0.65
        tree.links.new(gray.outputs[0], level.inputs["Value"])
        tree.links.new(level.outputs[0], color)
        if rubber and not principled.inputs["Normal"].is_linked:
            bump = tree.nodes.new("ShaderNodeBump")
            bump.name = "Rubber / molded sidewall lettering"
            # This sidewall atlas draws the molded letters dark; invert their
            # relief so the letters rise above the rubber instead of engraving it.
            bump.invert = material.name == "Material.107"
            bump.inputs["Distance"].default_value = 0.0004
            bump.inputs["Strength"].default_value = 0.5
            tree.links.new(gray.outputs[0], bump.inputs["Height"])
            tree.links.new(bump.outputs[0], principled.inputs["Normal"])
    else:
        color.default_value = (0.003, 0.003, 0.003, 1) if rubber else (0.55, 0.55, 0.55, 1)
    for name, value in {
        "Metallic": 0.0 if rubber else 0.95,
        "Roughness": 0.82 if rubber else 0.20,
        "Specular IOR Level": 0.08 if rubber else 0.5,
        "Coat Weight": 0.0 if rubber else 0.16,
        "Coat Roughness": 0.16,
    }.items():
        socket = principled.inputs[name]
        for link in list(socket.links):
            tree.links.remove(link)
        socket.default_value = value
    if rubber:
        coords = tree.nodes.new("ShaderNodeTexCoord")
        grain = tree.nodes.new("ShaderNodeTexNoise")
        grain.name = "Rubber / fine molded grain in meters"
        grain.inputs["Scale"].default_value = 1200
        grain.inputs["Detail"].default_value = 2
        tree.links.new(coords.outputs["Object"], grain.inputs["Vector"])
        bump = tree.nodes.new("ShaderNodeBump")
        bump.name = "Rubber / grain over sidewall relief"
        bump.inputs["Distance"].default_value = 0.00007
        bump.inputs["Strength"].default_value = 0.18
        if principled.inputs["Normal"].is_linked:
            tree.links.new(principled.inputs["Normal"].links[0].from_socket, bump.inputs["Normal"])
        tree.links.new(grain.outputs["Fac"], bump.inputs["Height"])
        tree.links.new(bump.outputs[0], principled.inputs["Normal"])
        rough = tree.nodes.new("ShaderNodeMapRange")
        rough.inputs["To Min"].default_value = 0.76
        rough.inputs["To Max"].default_value = 0.9
        tree.links.new(grain.outputs["Fac"], rough.inputs["Value"])
        tree.links.new(rough.outputs[0], principled.inputs["Roughness"])


def exposed_carbon(material, principled, albedo):
    """Dark exposed weave beneath a controlled, visible resin reflection."""
    tree = material.node_tree
    gray = tree.nodes.new("ShaderNodeRGBToBW")
    gray.name = "Exposed carbon / original weave luminance"
    tree.links.new(albedo.outputs["Color"], gray.inputs[0])
    level = tree.nodes.new("ShaderNodeMapRange")
    level.name = "Carbon resin / bounded dark weave"
    level.inputs["From Max"].default_value = 0.20
    level.inputs["To Min"].default_value = 0.0015
    level.inputs["To Max"].default_value = 0.012
    tree.links.new(gray.outputs[0], level.inputs["Value"])
    tree.links.new(level.outputs[0], principled.inputs["Base Color"])
    principled.inputs["Metallic"].default_value = 0.02
    roughness = principled.inputs["Roughness"]
    if roughness.is_linked:
        source = roughness.links[0].from_socket
        rough_range = tree.nodes.new("ShaderNodeMapRange")
        rough_range.name = "Dry carbon / retained roughness variation"
        rough_range.inputs["To Min"].default_value = 0.3
        rough_range.inputs["To Max"].default_value = 0.55
        tree.links.new(source, rough_range.inputs["Value"])
        tree.links.new(rough_range.outputs[0], roughness)
    else:
        roughness.default_value = max(0.3, roughness.default_value)
    principled.inputs["Specular IOR Level"].default_value = 0.06
    principled.inputs["Coat Weight"].default_value = 0.08
    principled.inputs["Coat Roughness"].default_value = 0.16
    if not principled.inputs["Normal"].is_linked:
        bump = tree.nodes.new("ShaderNodeBump")
        bump.name = "Carbon weave / subtle resin microsurface"
        bump.inputs["Distance"].default_value = 0.0003
        bump.inputs["Strength"].default_value = 0.12
        tree.links.new(gray.outputs[0], bump.inputs["Height"])
        tree.links.new(bump.outputs[0], principled.inputs["Normal"])
    # A dry-carbon art direction: even a low-IOR glossy lobe approaches white
    # at grazing angles. Bound its entire contribution while retaining its PBR
    # normal/coat response and the original weave in both shader components.
    diffuse = tree.nodes.new("ShaderNodeBsdfDiffuse")
    diffuse.name = "Dry carbon / diffuse weave"
    diffuse.inputs["Roughness"].default_value = 0.4
    tree.links.new(level.outputs[0], diffuse.inputs["Color"])
    tree.links.new(principled.inputs["Normal"].links[0].from_socket, diffuse.inputs["Normal"])
    mix = tree.nodes.new("ShaderNodeMixShader")
    mix.name = "Exposed carbon / 25 percent resin reflection"
    mix.inputs[0].default_value = 0.25
    tree.links.new(diffuse.outputs[0], mix.inputs[1])
    tree.links.new(principled.outputs[0], mix.inputs[2])
    for output in [node for node in tree.nodes if node.type == "OUTPUT_MATERIAL" and node.is_active_output]:
        tree.links.new(mix.outputs[0], output.inputs["Surface"])


def urus_white_signature(material):
    """Light only the Y-shaped DRL region of this source's housing atlas."""
    tree = material.node_tree
    shader = next(n for n in tree.nodes if n.type == 'BSDF_PRINCIPLED')
    atlas = next(n for n in tree.nodes if n.type == 'TEX_IMAGE')
    uv = tree.nodes.new('ShaderNodeTexCoord')
    xy = tree.nodes.new('ShaderNodeSeparateXYZ')
    tree.links.new(uv.outputs['UV'], xy.inputs[0])
    def math_node(operation, first, second):
        node = tree.nodes.new('ShaderNodeMath')
        node.operation = operation
        for socket, value in zip(node.inputs, (first, second)):
            if isinstance(value, (float, int)):
                socket.default_value = value
            else:
                tree.links.new(value, socket)
        return node.outputs[0]
    # Image_24: upper horizontal strip and upper-right diagonal branches.
    region = math_node('MAXIMUM', math_node('GREATER_THAN', xy.outputs['X'], .58),
                       math_node('GREATER_THAN', xy.outputs['Y'], .90))
    region = math_node('MULTIPLY', region, math_node('GREATER_THAN', xy.outputs['Y'], .64))
    region = math_node('MULTIPLY', region, math_node('LESS_THAN', xy.outputs['Y'], .96))
    gray = tree.nodes.new('ShaderNodeRGBToBW')
    tree.links.new(atlas.outputs['Color'], gray.inputs[0])
    mask = tree.nodes.new('ShaderNodeMapRange')
    mask.name = 'White Y signature | source atlas mask'
    mask.inputs['From Min'].default_value = .18
    mask.inputs['From Max'].default_value = .65
    tree.links.new(gray.outputs[0], mask.inputs['Value'])
    strength = math_node('MULTIPLY', math_node('MULTIPLY', region, mask.outputs[0]), 2.5)
    shader.inputs['Emission Color'].default_value = (1, 1, 1, 1)
    tree.links.new(strength, shader.inputs['Emission Strength'])


def thin_glazing(original, name, tint, cabin=False):
    """Thin automotive glazing with Fresnel reflection, without alpha dithering.

    The original game asset has zero-thickness glass surfaces. A transparent/
    reflection mix avoids pretending these meshes have a refractive solid volume.
    Original texture nodes remain packed for provenance, but their plain black
    placeholder is not treated as an opaque paint map.
    """
    material = original.copy()
    material.name = name
    material["showroom_role"] = "thin_glazing"
    material.surface_render_method = "BLENDED"
    material.use_transparency_overlap = False
    tree = material.node_tree
    output = next(n for n in tree.nodes if n.type == "OUTPUT_MATERIAL")
    clear = tree.nodes.new("ShaderNodeBsdfTransparent")
    clear.inputs[0].default_value = (*tint, 1)
    reflection = tree.nodes.new("ShaderNodeBsdfGlossy")
    reflection.inputs["Color"].default_value = ((0.14, 0.18, 0.22, 1) if cabin else (0.8, 0.86, 0.92, 1))
    reflection.inputs["Roughness"].default_value = 0.11 if cabin else 0.055
    fresnel = tree.nodes.new("ShaderNodeFresnel")
    fresnel.inputs["IOR"].default_value = 1.25 if cabin else 1.45
    mix = tree.nodes.new("ShaderNodeMixShader")
    tree.links.new(fresnel.outputs[0], mix.inputs[0])
    tree.links.new(clear.outputs[0], mix.inputs[1])
    tree.links.new(reflection.outputs[0], mix.inputs[2])
    tree.links.new(mix.outputs[0], output.inputs["Surface"])
    return material


def component_roots(mesh):
    parents = list(range(len(mesh.vertices)))
    def find(i):
        while parents[i] != i:
            parents[i] = parents[parents[i]]
            i = parents[i]
        return i
    for edge in mesh.edges:
        a, b = map(find, edge.vertices)
        parents[a] = b
    return [find(i) for i in range(len(mesh.vertices))]


def detail_materials(car, meshes):
    """Split source draw-call materials into their actual visible roles."""
    if car == "jesko":
        rotor = bpy.data.materials.new("Jesko | bare brake rotor / missing source assignment")
        rotor.use_nodes = True
        p = rotor.node_tree.nodes.get("Principled BSDF")
        p.inputs["Base Color"].default_value = (0.10, 0.115, 0.13, 1)
        p.inputs["Metallic"].default_value = 0.85
        p.inputs["Roughness"].default_value = 0.34
        noise = rotor.node_tree.nodes.new("ShaderNodeTexNoise")
        noise.inputs["Scale"].default_value = 420
        bump = rotor.node_tree.nodes.new("ShaderNodeBump")
        bump.inputs["Strength"].default_value = 0.06
        bump.inputs["Distance"].default_value = 0.001
        rotor.node_tree.links.new(noise.outputs["Fac"], bump.inputs["Height"])
        rotor.node_tree.links.new(bump.outputs[0], p.inputs["Normal"])
        for obj in meshes:
            if len(obj.data.materials) == 0:
                if obj.name.startswith("brake_"):
                    obj.data.materials.append(rotor)
                elif obj.name == "SK_Door_FL_101.116":
                    obj.data.materials.append(bpy.data.materials["Material.038"])
                elif obj.name == "keyfob.003":
                    obj.data.materials.append(bpy.data.materials["Material.004"])
                else:
                    raise RuntimeError("Unreviewed source object without material: " + obj.name)
        original = bpy.data.materials["Material.005"]
        cabin = thin_glazing(original, "Jesko | smoked cabin glazing", (0.40, 0.46, 0.50), cabin=True)
        lens = thin_glazing(original, "Jesko | clear lamp lenses", (0.97, 0.98, 0.99))
        for obj in meshes:
            for slot in obj.material_slots:
                if slot.material and slot.material.name in {"Material.005", "Material.010"}:
                    slot.material = lens if obj.name.startswith("SM_Light_") else cabin
        # Existing lamp-strip geometry; no invented badges, lenses, or lamps.
        led = bpy.data.materials["MI_Light.007"]
        p = next(n for n in led.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
        p.inputs["Base Color"].default_value = (0.65, 0.72, 0.8, 1)
        p.inputs["Emission Color"].default_value = (0.8, 0.9, 1.0, 1)
        p.inputs["Emission Strength"].default_value = 2.2
        p.inputs["Roughness"].default_value = 0.18
        tail = bpy.data.materials["Material.015"]
        p = next(n for n in tail.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
        tex = next(n for n in tail.node_tree.nodes if n.type == "TEX_IMAGE" and n.image.name == "TailLight")
        tail.node_tree.links.new(tex.outputs["Color"], p.inputs["Emission Color"])
        p.inputs["Emission Strength"].default_value = 3.2
        p.inputs["Roughness"].default_value = 0.2
        gold = bpy.data.materials["Material.011"]
        p = next(n for n in gold.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
        tex = next(n for n in gold.node_tree.nodes if n.type == "TEX_IMAGE" and n.image.name == "gold")
        gold.node_tree.links.new(tex.outputs["Color"], p.inputs["Base Color"])
        p.inputs["Metallic"].default_value = 0.72
        p.inputs["Roughness"].default_value = 0.24
    elif car == "urus":
        original = next(m for m in bpy.data.materials if "Window_Material" in m.name)
        cabin = thin_glazing(original, "Urus | smoked cabin glazing", (0.56, 0.60, 0.63), cabin=True)
        lens = thin_glazing(original, "Urus | clear lamp lenses", (0.97, 0.98, 0.99))
        for obj in meshes:
            if original not in list(obj.data.materials):
                continue
            original_indices = {i for i, material in enumerate(obj.data.materials) if material == original}
            roots = component_roots(obj.data)
            tops = {}
            for vertex in obj.data.vertices:
                key = roots[vertex.index]
                tops[key] = max(tops.get(key, -math.inf), vertex.co.z)
            obj.data.materials.append(cabin)
            cabin_index = len(obj.data.materials) - 1
            obj.data.materials.append(lens)
            lens_index = len(obj.data.materials) - 1
            for polygon in obj.data.polygons:
                if polygon.material_index not in original_indices:
                    continue
                # Source components above 1.2m are cabin windows. The lower
                # components are headlamps/rear lenses, despite a shared material.
                polygon.material_index = cabin_index if tops[roots[polygon.vertices[0]]] > 1.2 else lens_index
        interior = next(m for m in bpy.data.materials if "InteriorA_Materia" in m.name)
        upholstery = interior.copy()
        upholstery.name = "Urus | red leather upholstery / original normal map"
        p = next(n for n in upholstery.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
        source = p.inputs["Base Color"].links[0].from_socket
        tint = upholstery.node_tree.nodes.new("ShaderNodeMixRGB")
        tint.blend_type = "MULTIPLY"
        tint.inputs[0].default_value = 1
        tint.inputs[2].default_value = (0.66, 0.013, 0.009, 1)
        luminance = upholstery.node_tree.nodes.new("ShaderNodeRGBToBW")
        upholstery.node_tree.links.new(source, luminance.inputs[0])
        level = upholstery.node_tree.nodes.new("ShaderNodeMapRange")
        level.inputs["From Max"].default_value = 0.5
        level.inputs["To Min"].default_value = 0.2
        level.inputs["To Max"].default_value = 0.8
        upholstery.node_tree.links.new(luminance.outputs[0], level.inputs["Value"])
        upholstery.node_tree.links.new(level.outputs[0], tint.inputs[1])
        upholstery.node_tree.links.new(tint.outputs[0], p.inputs["Base Color"])
        p.inputs["Roughness"].default_value = 0.36
        for obj in meshes:
            if interior not in list(obj.data.materials):
                continue
            interior_indices = {i for i, material in enumerate(obj.data.materials) if material == interior}
            obj.data.materials.append(upholstery)
            index = len(obj.data.materials) - 1
            for polygon in obj.data.polygons:
                if polygon.material_index not in interior_indices:
                    continue
                center = sum((obj.data.vertices[i].co for i in polygon.vertices), Vector()) / len(polygon.vertices)
                if -1.45 < center.x < 0.4 and 0.13 < abs(center.y) < 0.65 and 0.5 < center.z < 1.43:
                    polygon.material_index = index
        lamps = next(m for m in bpy.data.materials if "LightEmissiveA_Ma" in m.name)
        p = next(n for n in lamps.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
        for link in list(p.inputs["Alpha"].links):
            lamps.node_tree.links.remove(link)
        p.inputs["Alpha"].default_value = 1.0
        p.inputs["Emission Strength"].default_value = 4.0
        # The source emissive atlas contains the actual red/white lamp pattern;
        # its other base-color image is plain white.
        atlas = next(n for n in lamps.node_tree.nodes if n.type == "TEX_IMAGE" and n.image.name == "Image_23")
        lamps.node_tree.links.new(atlas.outputs["Color"], p.inputs["Base Color"])
        # The front lamp's reflector, emitting strip and clear cover are
        # distinct surfaces. A full-metal mirror under a glossy cover was
        # turning the entire headlamp silver instead of exposing a dark housing.
        housing = next(m for m in bpy.data.materials if "LightA_Material" in m.name)
        front_housing = housing.copy()
        front_housing.name = "Urus | dark front lamp housing"
        tree = front_housing.node_tree
        shader = next(n for n in tree.nodes if n.type == "BSDF_PRINCIPLED")
        source = shader.inputs["Base Color"].links[0].from_socket
        tint = tree.nodes.new("ShaderNodeMixRGB")
        tint.blend_type = "MULTIPLY"
        tint.inputs[0].default_value = 1
        tint.inputs[2].default_value = (0.035, 0.043, 0.055, 1)
        tree.links.new(source, tint.inputs[1])
        tree.links.new(tint.outputs[0], shader.inputs["Base Color"])
        shader.inputs["Metallic"].default_value = 0.55
        shader.inputs["Roughness"].default_value = 0.3
        urus_white_signature(front_housing)
        front_led = lamps.copy()
        front_led.name = "Urus | front lamp optical elements"
        shader = next(n for n in front_led.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
        shader.inputs["Metallic"].default_value = 0
        shader.inputs["Roughness"].default_value = 0.3
        shader.inputs["Coat Weight"].default_value = 0
        shader.inputs["Specular IOR Level"].default_value = 0.15
        # The front signature in the atlas is dim blue-gray, unlike the bright
        # red rear lights. Normalize its hue and expose it through the retained
        # source opacity; do not make the full optical card opaque again.
        shader.inputs["Emission Strength"].default_value = 3.0
        tree = front_led.node_tree
        white_led = tree.nodes.new("ShaderNodeRGBToBW")
        white_led.name = "White front LEDs | retained atlas pattern"
        tree.links.new(shader.inputs["Emission Color"].links[0].from_socket, white_led.inputs[0])
        tree.links.new(white_led.outputs[0], shader.inputs["Emission Color"])
        optical_tint = tree.nodes.new("ShaderNodeMixRGB")
        optical_tint.name = "Front optics / dark backing with retained lamp atlas"
        optical_tint.blend_type = "MULTIPLY"
        optical_tint.inputs[0].default_value = 1
        optical_tint.inputs[2].default_value = (0.04, 0.05, 0.065, 1)
        tree.links.new(shader.inputs["Base Color"].links[0].from_socket, optical_tint.inputs[1])
        tree.links.new(optical_tint.outputs[0], shader.inputs["Base Color"])
        # The source optical card covers the complete lamp. Preserve its source
        # opacity so the dark reflector geometry is visible behind the card.
        tree.links.new(tree.nodes["Math"].outputs[0], shader.inputs["Alpha"])
        front_led.surface_render_method = "BLENDED"
        front_led.use_transparency_overlap = False
        front_lens = lens.copy()
        front_lens.name = "Urus | clear front lamp cover"
        reflection = front_lens.node_tree.nodes["Glossy BSDF"]
        reflection.inputs["Color"].default_value = (0.16, 0.20, 0.25, 1)
        next(n for n in front_lens.node_tree.nodes if n.type == "FRESNEL").inputs["IOR"].default_value = 1.35
        replacements = {housing: front_housing, lamps: front_led, lens: front_lens}
        for obj in meshes:
            slots = {i: replacements[m] for i, m in enumerate(obj.data.materials) if m in replacements}
            if not slots:
                continue
            new_slots = {}
            for index, material in slots.items():
                obj.data.materials.append(material)
                new_slots[index] = len(obj.data.materials) - 1
            for polygon in obj.data.polygons:
                if polygon.material_index in new_slots:
                    center = sum((obj.data.vertices[i].co for i in polygon.vertices), Vector()) / len(polygon.vertices)
                    if center.x > 1.5 and center.z > 0.65:
                        polygon.material_index = new_slots[polygon.material_index]


def prepare(car, source, output):
    scene = bpy.context.scene
    scene.frame_set(1)
    meshes = [o for o in scene.objects if o.type == "MESH" and not o.hide_render]
    depsgraph = bpy.context.evaluated_depsgraph_get()
    # Freeze source rigs/modifiers exactly once; showroom motion belongs to the stage.
    for obj in meshes:
        world = obj.matrix_world.copy()
        evaluated = obj.evaluated_get(depsgraph)
        mesh = bpy.data.meshes.new_from_object(evaluated, preserve_all_data_layers=True, depsgraph=depsgraph)
        obj.modifiers.clear()
        obj.constraints.clear()
        obj.animation_data_clear()
        obj.parent = None
        obj.data = mesh
        obj.matrix_world = world
    for obj in list(bpy.data.objects):
        if obj not in meshes:
            bpy.data.objects.remove(obj, do_unlink=True)
    bpy.context.view_layer.update()
    lo, hi = bounds(meshes)
    scale = SOURCES[car][1] / (hi[1] - lo[1])
    # All supplied models use Z up and -Y nose, verified against rendered geometry.
    rotation = Matrix.Rotation(math.pi / 2, 4, "Z")
    center = Matrix.Translation((-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]))
    transform = Matrix.Scale(scale, 4) @ rotation @ center
    for obj in meshes:
        obj.data.transform(transform @ obj.matrix_world)
        obj.matrix_world = Matrix.Identity(4)
        obj.hide_set(False)
        obj.hide_viewport = False
        obj.hide_render = False
    bpy.context.view_layer.update()
    collection = bpy.data.collections.new("CAR_" + car)
    scene.collection.children.link(collection)
    for obj in meshes:
        for old in list(obj.users_collection):
            old.objects.unlink(obj)
        collection.objects.link(obj)
    for old in list(bpy.data.collections):
        if old != collection:
            bpy.data.collections.remove(old)

    detail_materials(car, meshes)
    changed = []
    used_materials = {m for obj in meshes for m in obj.data.materials if m}
    for material in used_materials:
        if not material.use_nodes:
            continue
        image_names = {n.image.name for n in material.node_tree.nodes if n.type == "TEX_IMAGE" and n.image}
        carbon = (car == "urus" and "Carbon1M_Material" in material.name) or (car == "jesko" and bool(image_names & {"tc-carbon-02", "common_carbon05_black_diff"}))
        # F1 cars retain their entire authored liveries, including sponsor colors.
        paint = (car == "urus" and "Paint_Material" in material.name) or (car == "jesko" and "5" in image_names)
        jesko_rim = car == "jesko" and material.name in {"Material.058", "Material.027", "Material.031", "AttackMI_ChangeA_Rim1.001", "DefaultMaterial.003", "DefaultMaterial.004", "DefaultMaterial.005", "DefaultMaterial.006", "DefaultMaterial.007"}
        rubber = car == "jesko" and material.name in {"Material.107", "Material.108"}
        if material.get("showroom_role") == "thin_glazing":
            continue
        for node in list(material.node_tree.nodes):
            if node.type != "BSDF_PRINCIPLED":
                continue
            if carbon:
                albedo = next(n for n in material.node_tree.nodes if n.type == "TEX_IMAGE" and n.image
                              and (n.image.name == "Image_2" if car == "urus" else n.image.name in {"tc-carbon-02", "common_carbon05_black_diff"}))
                exposed_carbon(material, node, albedo)
                changed.append(material.name)
            elif paint:
                black_lacquer(material, node, metallic_finish=car == "urus")
                changed.append(material.name)
            elif jesko_rim:
                wheel_surface(material, node, "silver_alloy_rim")
                changed.append(material.name)
            elif rubber:
                wheel_surface(material, node, "black_rubber")
                changed.append(material.name)
    # Joining retains distinct material slots, UV maps, and imported split normals.
    bpy.ops.object.select_all(action="DESELECT")
    for obj in meshes:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = meshes[0]
    if len(meshes) > 1:
        bpy.ops.object.join()
    car_object = bpy.context.object
    car_object.name = "Car_" + car
    # Source object bounds can overestimate rotated submesh extents. Ground and
    # center once more from the joined geometry rather than those source boxes.
    bpy.context.view_layer.update()
    final_lo, final_hi = bounds([car_object])
    final_scale = SOURCES[car][1] / (final_hi[0] - final_lo[0])
    final_center = Matrix.Translation((-(final_lo[0] + final_hi[0]) / 2,
                                       -(final_lo[1] + final_hi[1]) / 2, -final_lo[2]))
    car_object.data.transform(Matrix.Scale(final_scale, 4) @ final_center)
    bpy.context.view_layer.update()
    car_object["appearance_id"] = car
    car_object["forward_axis"] = "+X"
    car_object["source_is_visual_only"] = True
    empty_material_faces = sum(1 for polygon in car_object.data.polygons
                               if polygon.material_index >= len(car_object.data.materials)
                               or car_object.data.materials[polygon.material_index] is None)
    if empty_material_faces:
        raise RuntimeError(f"Prepared car has {empty_material_faces} faces without a material")
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0
    # The .blend contains abandoned source-material experiments with missing files.
    # Only keep datablocks used by the baked visible geometry.
    for material in list(bpy.data.materials):
        if material not in used_materials:
            bpy.data.materials.remove(material, do_unlink=True)
    used_images = {n.image for material in used_materials if material.use_nodes
                   for n in material.node_tree.nodes if n.type == "TEX_IMAGE" and n.image}
    for img in list(bpy.data.images):
        if img not in used_images:
            bpy.data.images.remove(img, do_unlink=True)
    bpy.ops.outliner.orphans_purge(do_recursive=True)
    missing_images = []
    for img in bpy.data.images:
        if img.source in {"FILE", "GENERATED"} and img.size[0] > 0:
            if not img.packed_file:
                img.pack()
        elif img.users:
            missing_images.append(img.name)
    if missing_images:
        raise RuntimeError("Used source textures are missing: " + ", ".join(missing_images))
    scene.world = None
    scene.camera = None
    report = inspect(car, source)
    report.update({"collection": collection.name, "object": car_object.name, "units": "meters", "forward_axis": "+X", "scale_applied": scale * final_scale, "rotation_z_radians": math.pi / 2, "darkened_materials": changed, "missing_images": missing_images, "empty_material_faces": empty_material_faces, "livery": "authored source colors and PBR" if car in {"amr23", "rb19"} else "deep black lacquer, authored colored trim"})
    (output / f"{car}.json").write_text(json.dumps(report, indent=2))
    bpy.ops.wm.save_as_mainfile(filepath=str(output / f"{car}.blend"), compress=True)
    print("CAR_PREPARED " + json.dumps({"id": car, "dimensions": report["dimensions"], "paint": changed, "images": len(report["images"])}))


def render_audit(car, output, detail=False):
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.samples = 12
    scene.cycles.use_denoising = True
    scene.render.resolution_x = 800
    scene.render.resolution_y = 600
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    world = bpy.data.worlds.new("Audit world")
    scene.world = world
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (0.09, 0.09, 0.09, 1)
    world.node_tree.nodes["Background"].inputs[1].default_value = 0.3
    bpy.ops.mesh.primitive_plane_add(size=200, location=(0, 0, -0.01))
    floor = bpy.data.materials.new("Audit neutral floor")
    floor.diffuse_color = (0.055, 0.055, 0.055, 1)
    bpy.context.object.data.materials.append(floor)
    for name, loc, power, size in [("Key", (0, -3, 7), 1600, 6), ("Rim", (-3, 4, 5), 2200, 5), ("Fill", (5, 2, 4), 1200, 4)]:
        data = bpy.data.lights.new(name, "AREA")
        data.energy = power
        data.shape = "DISK"
        data.size = size
        obj = bpy.data.objects.new(name, data)
        scene.collection.objects.link(obj)
        obj.location = loc
        obj.rotation_euler = (Vector((0, 0, 0.5)) - obj.location).to_track_quat("-Z", "Y").to_euler()
    data = bpy.data.cameras.new("Audit camera")
    camera = bpy.data.objects.new("Audit camera", data)
    scene.collection.objects.link(camera)
    camera.location = (7, -7, 3.8)
    camera.rotation_euler = (Vector((0, 0, 0.5)) - camera.location).to_track_quat("-Z", "Y").to_euler()
    data.lens = 47
    scene.camera = camera
    scene.render.filepath = str(output / f"{car}.audit.png")
    bpy.ops.render.render(write_still=True)
    if detail:
        for view, position, target in [
            ("front", (8.0, -2.7, 2.0), (0.7, 0, 0.8)),
            ("rear", (-7.0, -3.2, 2.2), (-0.5, 0, 0.7)),
            ("close", (4.5, -3.5, 2.2), (0.85, -0.3, 0.85)),
        ]:
            camera.location = position
            camera.rotation_euler = (Vector(target) - camera.location).to_track_quat("-Z", "Y").to_euler()
            data.lens = 58
            scene.cycles.samples = 24
            scene.render.resolution_x = 1000
            scene.render.resolution_y = 750
            scene.render.filepath = str(output / f"{car}.{view}.png")
            bpy.ops.render.render(write_still=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--car", choices=SOURCES, required=True)
    parser.add_argument("--audit-only", action="store_true")
    parser.add_argument("--render-audit", action="store_true")
    parser.add_argument("--render-details", action="store_true")
    parser.add_argument("--out", default="artifacts/showroom/cars")
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
    output = (ROOT / args.out).resolve()
    output.mkdir(parents=True, exist_ok=True)
    source = DOWNLOADS / SOURCES[args.car][0]
    bpy.ops.wm.read_factory_settings(use_empty=True)
    if source.suffix == ".blend":
        bpy.ops.wm.open_mainfile(filepath=str(source))
    else:
        bpy.ops.import_scene.gltf(filepath=str(source))
    report = inspect(args.car, source)
    (output / f"{args.car}.source.json").write_text(json.dumps(report, indent=2))
    print("CAR_AUDIT " + json.dumps({k: report[k] for k in ("id", "dimensions", "bounds_min", "bounds_max")}))
    if args.audit_only:
        return
    prepare(args.car, source, output)
    if args.render_audit or args.render_details:
        render_audit(args.car, output, args.render_details)


if __name__ == "__main__":
    main()
