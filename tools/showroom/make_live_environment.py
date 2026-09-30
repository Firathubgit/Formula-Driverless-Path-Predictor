"""Render the live driving scene's image-based lighting: a dark studio sky with soft overhead strips and a low horizon glow,
as an equirectangular Radiance HDR. It lights and reflects in the live car's paint; it is never drawn as a background.

    blender -b --factory-startup --python tools/showroom/make_live_environment.py -- --out artifacts/live-cars/studio.hdr

Everything here is generated: no downloaded image enters it.
"""
import argparse
import math
import sys
from pathlib import Path

import bpy


def emissive_plane(name, location, size, rotation, strength, color=(1.0, 0.97, 0.93)):
    bpy.ops.mesh.primitive_plane_add(size=1, location=location, rotation=rotation)
    plane = bpy.context.active_object
    plane.name = name
    plane.scale = (size[0], size[1], 1)
    material = bpy.data.materials.new(name)
    material.use_nodes = True
    nodes = material.node_tree.nodes
    nodes.clear()
    emission = nodes.new("ShaderNodeEmission")
    emission.inputs["Color"].default_value = (*color, 1)
    emission.inputs["Strength"].default_value = strength
    output = nodes.new("ShaderNodeOutputMaterial")
    material.node_tree.links.new(emission.outputs["Emission"], output.inputs["Surface"])
    plane.data.materials.append(material)
    return plane


def main():
    args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--width", type=int, default=2048)
    parser.add_argument("--samples", type=int, default=128)
    options = parser.parse_args(args)
    scene = bpy.context.scene
    for obj in list(bpy.data.objects):
        bpy.data.objects.remove(obj, do_unlink=True)

    # Sky: near black overhead, a cool grey glow at the horizon, a dark floor below it.
    world = bpy.data.worlds.new("LiveStudio")
    scene.world = world
    world.use_nodes = True
    nodes, links = world.node_tree.nodes, world.node_tree.links
    nodes.clear()
    coords = nodes.new("ShaderNodeTexCoord")
    separate = nodes.new("ShaderNodeSeparateXYZ")
    # The direction's height runs from -1 below to 1 above; the ramp takes it as 0 to 1, the horizon at one half.
    height = nodes.new("ShaderNodeMapRange")
    height.inputs["From Min"].default_value = -1.0
    height.inputs["From Max"].default_value = 1.0
    ramp = nodes.new("ShaderNodeValToRGB")
    background = nodes.new("ShaderNodeBackground")
    output = nodes.new("ShaderNodeOutputWorld")
    links.new(coords.outputs["Generated"], separate.inputs["Vector"])
    links.new(separate.outputs["Z"], height.inputs["Value"])
    links.new(height.outputs["Result"], ramp.inputs["Fac"])
    elements = ramp.color_ramp.elements
    elements[0].position, elements[0].color = 0.0, (0.004, 0.004, 0.005, 1)
    elements[1].position, elements[1].color = 1.0, (0.006, 0.007, 0.009, 1)
    for position, color in ((0.47, (0.012, 0.013, 0.015, 1)), (0.5, (0.09, 0.1, 0.115, 1)), (0.56, (0.02, 0.022, 0.026, 1))):
        element = elements.new(position)
        element.color = color
    links.new(ramp.outputs["Color"], background.inputs["Color"])
    background.inputs["Strength"].default_value = 1.0
    links.new(background.outputs["Background"], output.inputs["Surface"])

    # Studio strips overhead, running along the car, and two broad softboxes either side: the long reflections a car's
    # paint carries in a photographer's studio.
    for offset in (-3.0, 0.0, 3.0):
        emissive_plane(f"Strip{offset}", (offset, 0, 7.5), (0.9, 14), (0, 0, 0), 9.0)
    emissive_plane("SoftboxLeft", (-9, 0, 3.2), (5, 10), (0, math.radians(90), 0), 2.2, (0.9, 0.95, 1.0))
    emissive_plane("SoftboxRight", (9, 0, 3.2), (5, 10), (0, math.radians(-90), 0), 1.6, (1.0, 0.95, 0.9))
    emissive_plane("Front", (0, -12, 2.5), (8, 3), (math.radians(90), 0, 0), 1.2)

    bpy.ops.object.camera_add(location=(0, 0, 1.0), rotation=(math.radians(90), 0, 0))
    camera = bpy.context.active_object
    camera.data.type = "PANO"
    camera.data.panorama_type = "EQUIRECTANGULAR"
    scene.camera = camera
    scene.render.engine = "CYCLES"
    scene.cycles.samples = options.samples
    scene.cycles.use_denoising = True
    scene.render.resolution_x = options.width
    scene.render.resolution_y = options.width // 2
    scene.render.resolution_percentage = 100
    scene.view_settings.view_transform = "Standard"
    scene.render.image_settings.file_format = "HDR"
    options.out.parent.mkdir(parents=True, exist_ok=True)
    scene.render.filepath = str(options.out)
    bpy.ops.render.render(write_still=True)
    print("ENVIRONMENT", options.out)


main()
