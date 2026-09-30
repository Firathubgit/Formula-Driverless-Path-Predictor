"""Export a prepared showroom car for the live driving scene: its body and four wheels as GLB files, a contact shadow and
a manifest the desktop reads.

Run inside Blender on a prepared car, which stands nose +X, Z up, ground at Z0, in metres:

    blender -b artifacts/showroom/cars/rb19.blend --python tools/showroom/export_live_car.py -- --car rb19 --out artifacts/live-cars

The prepared car is one joined mesh whose original parts remain separate connected islands. Each tire is the island of
the lowest point in its corner; every island inside that tire's cylinder turns with it (tire, rim, spokes, nuts, disc).
The live car's origin is the rear axle's centre on the ground, nose along -Z and right along +X in Qt, so the exported
nodes carry that transform; each wheel's centre is its own origin, so the desktop spins it about X and steers it about Y.
Materials are exported as they are; textures larger than the limits are scaled down, since Qt refuses images whose
decoded size exceeds its allocation limit and the live scene does not need them. Nothing is written back to the source.
"""
import argparse
import hashlib
import json
import math
import sys
import time
from pathlib import Path

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

CORNERS = (("fl", True, True), ("fr", True, False), ("rl", False, True), ("rr", False, False))


def island_labels(mesh):
    """Connected-component label of every vertex, by label propagation with pointer jumping."""
    edges = np.empty(len(mesh.edges) * 2, dtype=np.int64)
    mesh.edges.foreach_get("vertices", edges)
    edges = edges.reshape(-1, 2)
    labels = np.arange(len(mesh.vertices), dtype=np.int64)
    while True:
        low = np.minimum(labels[edges[:, 0]], labels[edges[:, 1]])
        new = labels.copy()
        np.minimum.at(new, edges[:, 0], low)
        np.minimum.at(new, edges[:, 1], low)
        new = new[new]
        if np.array_equal(new, labels):
            return labels
        labels = new


def per_island(values, island, count, reduce):
    out = np.full(count, np.inf if reduce is np.minimum else -np.inf)
    reduce.at(out, island, values)
    return out


def find_wheels(co, island, count):
    """The four tires from their ground contact, and the islands that turn with each.

    A tire may be one island (the F1 cars) or a ring of tread segments (the road cars). Either way the islands touching
    the ground in a corner are the bottom of its tread: they are symmetric about the contact, which gives the centre's
    position along the car, and every tread point implies the radius of a circle standing on the ground through it,
    ((x - xc)^2 + z^2) / 2z. Points on the outer tread give the tire's radius; grooves and sidewalls lie inside it and
    give less, so the radius is taken from the top of that distribution, and the tread must also be found at the top.
    """
    x, y, z = co[:, 0], co[:, 1], co[:, 2]
    ground = float(z.min())
    lo = {axis: per_island(co[:, i], island, count, np.minimum) for i, axis in enumerate("xyz")}
    hi = {axis: per_island(co[:, i], island, count, np.maximum) for i, axis in enumerate("xyz")}
    mid_x, mid_y = (lo["x"] + hi["x"]) / 2, (lo["y"] + hi["y"]) / 2
    wheels = []
    for name, front, left in CORNERS:
        # Tires stand outboard of the floor and plank, so only the outer part of each corner is searched.
        touching = np.flatnonzero((lo["z"] < ground + 0.02) & ((mid_x > 0) == front) & ((mid_y > 0) == left) &
                                  (np.abs(mid_y) > 0.45))
        if not len(touching):
            raise RuntimeError(f"Nothing touches the ground in the {name} corner")
        tread = np.isin(island, touching)
        xs, ys, zs = x[tread], y[tread], z[tread] - ground
        xc = float((xs.min() + xs.max()) / 2)
        high = zs > 0.01
        radius = float(np.percentile(((xs[high] - xc) ** 2 + zs[high] ** 2) / (2 * zs[high]), 98))
        crown = np.count_nonzero((np.abs(x - xc) < 0.06) & (np.abs(z - ground - 2 * radius) < 0.025) &
                                 (y >= ys.min() - 0.02) & (y <= ys.max() + 0.02))
        if not 0.2 < radius < 0.55 or crown == 0:
            raise RuntimeError(f"No round tire in the {name} corner: radius {radius:.3f}, {crown} points at its crown")
        centre = np.array([xc, float((ys.min() + ys.max()) / 2), ground + radius])
        width = float(ys.max() - ys.min())
        # Inboard is toward the centreline: allow the disc and hub a little inside the tire, nothing outside it.
        inner, outer = (ys.min() - 0.06, ys.max() + 0.02) if left else (ys.min() - 0.02, ys.max() + 0.06)
        radial = np.sqrt((x - centre[0]) ** 2 + (z - centre[2]) ** 2)
        reach = per_island(radial, island, count, np.maximum)
        members = np.flatnonzero((reach <= radius * 1.03 + 0.01) & (lo["y"] >= inner) & (hi["y"] <= outer))
        wheels.append({"name": name, "front": front, "left": left, "centre": centre, "radius": radius,
                       "width": width, "islands": members})
    return wheels


def extract(source, keep, name):
    """A copy of the source object holding only the faces kept."""
    mesh = source.data.copy()
    mesh.name = name
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bm.faces.ensure_lookup_table()
    doomed = [face for face, kept in zip(bm.faces, keep) if not kept]
    bmesh.ops.delete(bm, geom=doomed, context="FACES")
    bm.to_mesh(mesh)
    bm.free()
    return obj


def contact_shadow(co, frame, path, cell=0.02, height=0.35):
    """A soft shadow of the car on the ground in the live frame, from its lowest geometry: rows run nose to tail."""
    p = (frame @ np.c_[co, np.ones(len(co))].T).T[:, :3]  # live Blender frame: right +X, forward +Y, up +Z
    weight = np.exp(-np.clip(p[:, 2], 0, None) / height)
    margin = 0.35
    x0, x1 = p[:, 0].min() - margin, p[:, 0].max() + margin
    y0, y1 = p[:, 1].min() - margin, p[:, 1].max() + margin
    nx, ny = int(math.ceil((x1 - x0) / cell)), int(math.ceil((y1 - y0) / cell))
    grid, _, _ = np.histogram2d(p[:, 1], p[:, 0], bins=(ny, nx), range=((y0, y1), (x0, x1)), weights=weight)
    occupancy = np.clip(grid / max(np.percentile(grid[grid > 0], 60), 1e-9), 0, 1)
    kernel = np.exp(-0.5 * (np.arange(-12, 13) / 5.0) ** 2)
    kernel /= kernel.sum()
    for axis in (0, 1):
        occupancy = np.apply_along_axis(lambda row: np.convolve(row, kernel, mode="same"), axis, occupancy)
    alpha = np.clip(occupancy * 1.4, 0, 1) ** 1.2
    image = bpy.data.images.new("contact_shadow", nx, ny, alpha=True)
    rgba = np.zeros((ny, nx, 4), dtype=np.float32)
    rgba[..., 3] = alpha
    image.pixels.foreach_set(rgba.ravel())  # Blender's rows run bottom-up: row 0 is the tail
    image.filepath_raw = str(path)
    image.file_format = "PNG"
    image.save()
    # In Qt: x across, z = -forward.
    return {"file": path.name, "x": [float(x0), float(x1)], "z": [float(-y1), float(-y0)]}


# Materials for glTF ---------------------------------------------------------------------------------------------------
# The showroom's approved materials (tools/showroom/prepare_cars.py) reach their looks through node chains a real-time
# PBR material cannot run: black lacquer from a luminance remapped into a narrow dark range, rubber and alloy remapped
# the same way, dry carbon as a darkened weave mixed with a diffuse lobe, glazing as a Fresnel mix of transparent and
# glossy, procedural flake, grain and bumps. Each input is evaluated to what glTF carries, a constant or an image times
# a factor, and the material rebuilt as one Principled BSDF: multiply tints stay exact, a remap into a range narrower
# than 5 thousandths becomes its mean, a wider remap keeps its image scaled to the same mean, procedural detail is
# dropped for the normal map beneath it, and glazing becomes a tinted, blended, glossy surface.
MEAN_LUMINANCE = {}


def mean_luminance(image, samples=None):
    """The image's mean luminance, or its luminance samples on a sparse grid when asked for them."""
    if image.name not in MEAN_LUMINANCE:
        w, h = image.size
        pixels = np.empty(w * h * image.channels, dtype=np.float32)
        image.pixels.foreach_get(pixels)
        grid = pixels.reshape(h, w, image.channels)[::8, ::8, :3]
        lum = grid @ np.array([0.2126, 0.7152, 0.0722], dtype=np.float32)
        MEAN_LUMINANCE[image.name] = lum.ravel()
    lum = MEAN_LUMINANCE[image.name]
    return lum if samples else float(lum.mean())


def const(value):
    return ("const", value)


def scale(value, factor):
    """A value times a scalar or colour factor."""
    def times(a, b):
        if isinstance(a, tuple) and isinstance(b, tuple):
            return tuple(x * y for x, y in zip(a, b))
        if isinstance(a, tuple):
            return tuple(x * b for x in a)
        if isinstance(b, tuple):
            return tuple(a * y for y in b)
        return a * b
    if value[0] == "const":
        return const(times(value[1], factor))
    return ("tex", value[1], times(value[2], factor))


def mean_of(value):
    """A value's mean as a scalar."""
    if value[0] == "const":
        v = value[1]
        return sum(v) / 3 if isinstance(v, tuple) else v
    f = value[2]
    return mean_luminance(value[1].image) * (sum(f) / 3 if isinstance(f, tuple) else f)


def evaluate(socket, depth=0):
    """What a shader input evaluates to: ("const", v) or ("tex", image node, factor)."""
    if not socket.is_linked:
        v = socket.default_value
        return const(tuple(v)[:3] if hasattr(v, "__len__") else float(v))
    if depth > 12:
        return const(0.5)
    node = socket.links[0].from_node
    kind = node.bl_idname
    if kind == "ShaderNodeTexImage" and node.image:
        return ("tex", node, 1.0)
    if kind in ("ShaderNodeMixRGB", "ShaderNodeMix"):
        mixing = kind == "ShaderNodeMixRGB" or node.data_type == "RGBA"
        if kind == "ShaderNodeMixRGB":
            fac, a, b = node.inputs[0], node.inputs[1], node.inputs[2]
        else:
            fac, a, b = node.inputs["Factor"], node.inputs[6 if mixing else 2], node.inputs[7 if mixing else 3]
        blend = node.blend_type
        va, vb = evaluate(a, depth + 1), evaluate(b, depth + 1)
        # A factor fed by a chain (a mask) counts as fully applied: the intended look is the blend's.
        weight = fac.default_value if not fac.is_linked else 1.0
        if blend == "MULTIPLY":
            if vb[0] == "const":
                return scale(va, tuple(1 - weight + weight * c for c in vb[1]) if isinstance(vb[1], tuple) else 1 - weight + weight * vb[1])
            if va[0] == "const":
                return scale(vb, va[1])
            return va
        return vb if weight >= 0.5 else va
    if kind in ("ShaderNodeHueSaturation", "ShaderNodeRGBToBW", "ShaderNodeSeparateColor", "ShaderNodeGamma",
                "ShaderNodeBrightContrast", "ShaderNodeInvert"):
        return evaluate(node.inputs["Color"] if "Color" in node.inputs else node.inputs[0], depth + 1)
    if kind == "ShaderNodeMapRange":
        v = evaluate(node.inputs["Value"], depth + 1)
        lo_in, hi_in = node.inputs["From Min"].default_value, node.inputs["From Max"].default_value
        lo_out, hi_out = node.inputs["To Min"].default_value, node.inputs["To Max"].default_value
        def remap(x):
            t = (np.asarray(x) - lo_in) / max(hi_in - lo_in, 1e-9)
            return lo_out + np.clip(t, 0, 1) * (hi_out - lo_out)
        if v[0] == "const":
            return const(float(remap(mean_of(v))))
        f = v[2]
        samples = mean_luminance(v[1].image, samples=True) * (sum(f) / 3 if isinstance(f, tuple) else f)
        mapped = float(remap(samples).mean())
        base = mean_luminance(v[1].image)
        if abs(hi_out - lo_out) < 0.005 or base < 1e-4:
            return const(mapped)
        return ("tex", v[1], mapped / base)
    if kind == "ShaderNodeMath":
        a, b = evaluate(node.inputs[0], depth + 1), evaluate(node.inputs[1], depth + 1)
        if node.operation == "MULTIPLY":
            if b[0] == "const" and not isinstance(b[1], tuple):
                return scale(a, b[1])
            if a[0] == "const" and not isinstance(a[1], tuple):
                return scale(b, a[1])
        if a[0] == "const" and b[0] == "const" and not isinstance(a[1], tuple) and not isinstance(b[1], tuple):
            x, y = a[1], b[1]
            ops = {"ADD": x + y, "SUBTRACT": x - y, "MULTIPLY": x * y, "DIVIDE": x / y if y else 0.0,
                   "MAXIMUM": max(x, y), "MINIMUM": min(x, y), "GREATER_THAN": float(x > y), "LESS_THAN": float(x < y)}
            return const(ops.get(node.operation, x))
        return a if a[0] == "tex" else const(0.0)
    if kind == "ShaderNodeTexNoise":
        return const(0.5)
    if kind == "ShaderNodeFresnel":
        return const(0.05)
    return const(0.5)


def principled_of(material):
    """The material's Principled BSDF and the node its surface came from."""
    tree = material.node_tree
    output = next((n for n in tree.nodes if n.bl_idname == "ShaderNodeOutputMaterial" and n.is_active_output), None) \
        or next((n for n in tree.nodes if n.bl_idname == "ShaderNodeOutputMaterial"), None)
    surface = output.inputs["Surface"].links[0].from_node if output and output.inputs["Surface"].is_linked else None
    stack, seen = [surface], set()
    while stack:
        node = stack.pop(0)
        if node is None or node.name in seen:
            continue
        seen.add(node.name)
        if node.bl_idname == "ShaderNodeBsdfPrincipled":
            return output, surface, node
        stack += [link.from_node for s in node.inputs for link in s.links if s.type == "SHADER"]
    return output, surface, None


def flatten_material(material):
    """Rebuild one material as a single glTF-compatible Principled BSDF. Returns what was done."""
    if not material.use_nodes:
        return "no nodes"
    tree = material.node_tree
    output, surface, principled = principled_of(material)
    if output is None:
        return "no output"
    if material.get("showroom_role") == "thin_glazing":
        # Fresnel(transparent tint, glossy): a dark or clear tint, mostly see-through, with a sharp reflection.
        glossy = next((n for n in tree.nodes if n.bl_idname == "ShaderNodeBsdfGlossy"), None)
        clear = next((n for n in tree.nodes if n.bl_idname == "ShaderNodeBsdfTransparent"), None)
        tint = tuple(clear.inputs["Color"].default_value)[:3] if clear else (0.5, 0.5, 0.5)
        cabin = sum(tint) / 3 < 0.8
        principled = tree.nodes.new("ShaderNodeBsdfPrincipled")
        principled.inputs["Base Color"].default_value = (*(c * (0.08 if cabin else 0.9) for c in tint), 1)
        principled.inputs["Metallic"].default_value = 0.0
        principled.inputs["Roughness"].default_value = glossy.inputs["Roughness"].default_value if glossy else 0.06
        principled.inputs["Alpha"].default_value = 0.62 if cabin else 0.12
        tree.links.new(principled.outputs["BSDF"], output.inputs["Surface"])
        material.surface_render_method = "BLENDED"
        return "glazing"
    if principled is None:
        # No Principled anywhere: take the colour of the first BSDF there is.
        bsdf = next((n for n in tree.nodes if n.bl_idname.startswith("ShaderNodeBsdf") and "Color" in n.inputs), None)
        principled = tree.nodes.new("ShaderNodeBsdfPrincipled")
        if bsdf:
            value = evaluate(bsdf.inputs["Color"])
            if value[0] == "const":
                principled.inputs["Base Color"].default_value = (*value[1], 1) if isinstance(value[1], tuple) else (value[1],) * 3 + (1,)
    done = []
    for name in ("Base Color", "Metallic", "Roughness", "Alpha", "Coat Weight", "Coat Roughness", "Emission Color",
                 "Emission Strength"):
        socket = principled.inputs.get(name)
        if socket is None or not socket.is_linked:
            continue
        source = socket.links[0].from_node
        # Chains the exporter already reads: an image, its separated channels, its vertex colours.
        if source.bl_idname == "ShaderNodeTexImage" or (source.bl_idname == "ShaderNodeSeparateColor" and name in ("Metallic", "Roughness")
                                                        and source.inputs[0].is_linked and source.inputs[0].links[0].from_node.bl_idname == "ShaderNodeTexImage") \
                or source.bl_idname in ("ShaderNodeVertexColor", "ShaderNodeAttribute"):
            continue
        value = evaluate(socket)
        for link in list(socket.links):
            tree.links.remove(link)
        if value[0] == "const" or name not in ("Base Color", "Emission Color"):
            v = value[1] if value[0] == "const" else mean_of(value)
            if socket.type == "RGBA":
                socket.default_value = (*v, 1) if isinstance(v, tuple) else (v, v, v, 1)
            else:
                socket.default_value = sum(v) / 3 if isinstance(v, tuple) else v
            done.append(f"{name}=const")
            continue
        image, factor = value[1], value[2]
        colour = factor if isinstance(factor, tuple) else (factor,) * 3
        if all(abs(c - 1) < 1e-6 for c in colour):
            tree.links.new(image.outputs["Color"], socket)
        else:
            multiply = tree.nodes.new("ShaderNodeMixRGB")
            multiply.blend_type = "MULTIPLY"
            multiply.inputs[0].default_value = 1.0
            multiply.inputs[2].default_value = (*(min(c, 1.0) for c in colour), 1)
            tree.links.new(image.outputs["Color"], multiply.inputs[1])
            tree.links.new(multiply.outputs[0], socket)
        done.append(f"{name}=image*{tuple(round(c, 4) for c in colour)}")
    # Procedural bumps give way to the normal map beneath them.
    normal = principled.inputs["Normal"]
    guard = 0
    while normal.is_linked and normal.links[0].from_node.bl_idname == "ShaderNodeBump" and guard < 8:
        bump = normal.links[0].from_node
        beneath = bump.inputs["Normal"].links[0].from_socket if bump.inputs["Normal"].is_linked else None
        tree.links.remove(normal.links[0])
        if beneath is not None:
            tree.links.new(beneath, normal)
        done.append("bump dropped")
        guard += 1
    if surface is not principled:
        tree.links.new(principled.outputs["BSDF"], output.inputs["Surface"])
        done.append("surface is its Principled")
    return ", ".join(done) or "as it was"


def limit_textures(color_limit, data_limit):
    scaled = []
    for image in bpy.data.images:
        if image.users == 0 or image.size[0] == 0:
            continue
        limit = color_limit if image.colorspace_settings.name == "sRGB" else data_limit
        w, h = image.size
        if max(w, h) > limit:
            factor = limit / max(w, h)
            image.scale(max(1, round(w * factor)), max(1, round(h * factor)))
            scaled.append({"image": image.name, "from": [w, h], "to": list(image.size)})
    return scaled


def export(obj, path):
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.export_scene.gltf(filepath=str(path), export_format="GLB", use_selection=True, export_yup=True,
                              export_apply=False, export_texcoords=True, export_normals=True, export_tangents=True,
                              export_materials="EXPORT", export_image_format="AUTO", export_cameras=False,
                              export_lights=False, export_animations=False)


def main():
    args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--car", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--color-limit", type=int, default=4096)
    parser.add_argument("--data-limit", type=int, default=2048)
    options = parser.parse_args(args)
    started = time.time()
    out = (options.out / options.car).resolve()
    out.mkdir(parents=True, exist_ok=True)
    car = next(o for o in bpy.data.objects if o.type == "MESH")
    if car.matrix_world != Matrix.Identity(4):
        raise RuntimeError("The prepared car must carry no object transform")
    mesh = car.data
    co = np.empty(len(mesh.vertices) * 3)
    mesh.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    labels = island_labels(mesh)
    roots, island = np.unique(labels, return_inverse=True)
    wheels = find_wheels(co, island, len(roots))

    # Faces by the island of their first vertex; an island is connected, so all its vertices share it.
    loops = np.empty(len(mesh.loops), dtype=np.int64)
    mesh.loops.foreach_get("vertex_index", loops)
    starts = np.empty(len(mesh.polygons), dtype=np.int64)
    mesh.polygons.foreach_get("loop_start", starts)
    face_island = island[loops[starts]]
    taken = np.zeros(len(roots), dtype=bool)
    for wheel in wheels:
        if taken[wheel["islands"]].any():
            raise RuntimeError(f"The {wheel['name']} wheel shares islands with another wheel")
        taken[wheel["islands"]] = True

    # The live frame: the rear axle's centre on the ground at the origin, nose along +Y, which glTF's Y-up conversion
    # turns into -Z, and right along +X.
    rear = [w for w in wheels if not w["front"]]
    axle = np.mean([w["centre"] for w in rear], axis=0)
    centre_y = float(np.mean([w["centre"][1] for w in wheels]))
    frame = Matrix.Rotation(math.pi / 2, 4, "Z") @ Matrix.Translation((-float(axle[0]), -centre_y, 0.0))

    scaled = limit_textures(options.color_limit, options.data_limit)
    flattened = {m.name: flatten_material(m) for m in {s.material for s in car.material_slots if s.material}}
    body = extract(car, ~taken[face_island], f"{options.car}_body")
    body.matrix_world = frame
    export(body, out / "body.glb")
    manifest_wheels = []
    for wheel in wheels:
        keep = np.isin(face_island, wheel["islands"])
        obj = extract(car, keep, f"{options.car}_wheel_{wheel['name']}")
        at = frame @ Vector(wheel["centre"])
        obj.matrix_world = Matrix.Translation(-at) @ frame
        export(obj, out / f"wheel_{wheel['name']}.glb")
        manifest_wheels.append({"name": wheel["name"], "file": f"wheel_{wheel['name']}.glb", "front": wheel["front"],
                                "left": wheel["left"],
                                # Qt: x right, y up, z backward.
                                "position": [round(at.x, 4), round(at.z, 4), round(-at.y, 4)],
                                "radius": round(wheel["radius"], 4), "width": round(wheel["width"], 4),
                                "islands": int(len(wheel["islands"])), "faces": int(keep.sum())})
    shadow = contact_shadow(co, np.array(frame), out / "shadow.png")
    front = [w for w in manifest_wheels if w["front"]]
    back = [w for w in manifest_wheels if not w["front"]]
    manifest = {
        "schema_version": 1, "car": options.car, "source": bpy.data.filepath,
        "source_sha256": hashlib.sha256(Path(bpy.data.filepath).read_bytes()).hexdigest(),
        "frame": "origin at the rear axle's centre on the ground; Qt x right, y up, -z forward; metres",
        "body": "body.glb", "wheels": manifest_wheels, "shadow": shadow,
        "wheelbase_m": round(-front[0]["position"][2], 4),
        "track_front_m": round(abs(front[0]["position"][0] - front[1]["position"][0]), 4),
        "track_rear_m": round(abs(back[0]["position"][0] - back[1]["position"][0]), 4),
        "faces": {"total": len(mesh.polygons), "body": int((~taken[face_island]).sum())},
        "textures_scaled": scaled, "color_limit": options.color_limit, "data_limit": options.data_limit,
        "materials_flattened": flattened,
        "blender": bpy.app.version_string, "seconds": round(time.time() - started, 1),
    }
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print("EXPORTED", options.car, json.dumps({k: manifest[k] for k in ("wheelbase_m", "track_front_m", "track_rear_m", "faces", "seconds")}))


main()
