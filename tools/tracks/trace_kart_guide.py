"""Extract the poster's coaching stroke, registered to the existing kart track.

Run with .tools/tum-python/Scripts/python.exe trace_kart_guide.py POSTER OUT_DIR.
Requires numpy, scipy, Pillow and matplotlib. The private poster stays outside Git.
Writes a display-only CSV in OUT_DIR; copy it to tracks after inspecting guide.png.
Blue means the poster's coast advice, not the simulation plan's yellow hold-speed.
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import interpolate, ndimage
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection, PolyCollection

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("poster", type=Path)
parser.add_argument("out", type=Path)
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
args.out.mkdir(parents=True, exist_ok=True)
# The centreline's tracing coordinates are in this normalized poster frame.
poster = Image.open(args.poster).convert("RGB").resize((1500, 2000), Image.Resampling.LANCZOS)
normalized = args.out / "poster-normalized.png"
poster.save(normalized)
registration = args.out / "registration"
subprocess.run([sys.executable, str(root / "tools/tracks/trace_poster.py"), str(normalized),
                str(root / "tools/tracks/gokartcentralen-goteborg.points.json"), str(registration)], check=True)
pixel_track = np.loadtxt(registration / "centreline-px.csv", delimiter=",", skiprows=1)[:, :2]
registered = np.loadtxt(registration / "centreline.csv", delimiter=",", skiprows=1)
existing = np.loadtxt(root / "tracks/gokartcentralen-goteborg.csv", delimiter=",", skiprows=5)
retrace_difference = float(np.max(np.linalg.norm(registered - existing, axis=1)))
if retrace_difference > 1.1:
    raise ValueError(f"Poster differs from the installed track: {retrace_difference:.3f} m")
# Resizing the supplied higher-resolution poster changes the barrier pixels and thus
# some of the old tracer's snaps. Preserve the installed trace's documented scale;
# use only the re-found start-line origin, never replace the installed track.
evidence = json.loads((root / "docs/evidence/gokart-track-verification.json").read_text(encoding="utf-8"))
scale = evidence["rounds"][-1]["trace"]["metres_per_px"]
pixel_track = existing * [1 / scale, -1 / scale] + pixel_track[0]
count = len(existing)
at = np.arange(0, count, 0.25)
spline = interpolate.CubicSpline(np.arange(count + 1), np.vstack([pixel_track, pixel_track[:1]]), bc_type="periodic")
centre = spline(at)
tangent = spline(at, 1)
normals = np.c_[-tangent[:, 1], tangent[:, 0]] / np.linalg.norm(tangent, axis=1)[:, None]
rgb = np.asarray(poster).astype(float)
r, g, b = rgb.transpose(2, 0, 1)
# Dark green line, blue coast, red brake and their purple/teal transitions. The bright
# fan zone, pit lane, white grid and text do not qualify as the coaching stroke.
stroke = ((rgb.max(axis=2) - rgb.min(axis=2)) > 45) & (
    ((g > 65) & (g < 155) & (r < 65) & (b < 145)) |
    ((b > 80) & (g < 160) & (r < 165)) |
    ((r > 110) & (g < 65) & (b < 160)))
offsets = np.full(len(at), np.nan)
scan = np.arange(-4.0 / scale, 4.0 / scale, 0.25)
for k, (p, n) in enumerate(zip(centre, normals)):
    points = np.rint(p + scan[:, None] * n).astype(int)
    hits = stroke[points[:, 1], points[:, 0]]
    groups, size = ndimage.label(hits)
    candidates = [np.mean(scan[groups == j]) for j in range(1, size + 1)
                  if 4 <= np.sum(groups == j) * 0.25 <= 32]
    if candidates:
        offsets[k] = min(candidates, key=abs)
valid = np.isfinite(offsets)
coverage = float(np.mean(valid))
if np.mean(valid) < 0.95:
    raise ValueError(f"Only {np.mean(valid):.1%} of the coaching stroke found")
# The dashed green footbridge crosses the stroke; bridge lettering also occludes it.
valid &= ~((centre[:, 0] > 750) & (centre[:, 0] < 860) & (centre[:, 1] > 1150) & (centre[:, 1] < 1230))
offsets = np.interp(at, at[valid], offsets[valid], period=count)
offsets = ndimage.gaussian_filter1d(offsets, 2.0, mode="wrap")
source = centre + offsets[:, None] * normals
colours = ndimage.map_coordinates(rgb, [np.repeat(source[:, 1], 3), np.repeat(source[:, 0], 3),
                                       np.tile(np.arange(3), len(source))], order=1).reshape(-1, 3)
# During a gradient, use the dominant channel: red brake, blue coast, green gas.
phases = np.argmax(colours, axis=1)
phases = ndimage.median_filter(np.tile(phases, 3), size=9)[len(at):2 * len(at)]
# The poster's variable-width asphalt differs from the installed constant 6 m corridor.
# Keep the complete 0.4 m stroke inside that corridor, with 0.3 m clear to the edge.
limited = np.clip(offsets * scale, -2.5, 2.5) / scale
guide_px = centre + limited[:, None] * normals
guide = (guide_px - pixel_track[0]) * [scale, -scale]
names = np.array(["brake", "accelerate", "coast"])
target = args.out / "gokartcentralen-goteborg-guide.csv"
with target.open("w", encoding="utf-8", newline="\n") as f:
    f.write("# Poster coaching advice, display only; generated by tools/tracks/trace_kart_guide.py.\n")
    f.write("# Registered to gokartcentralen-goteborg.csv; 0.4 m stroke, inset at least 0.3 m from the 6 m corridor.\n")
    f.write("x_m,y_m,advice\n")
    for p, phase in zip(guide, phases):
        f.write(f"{p[0]:.4f},{p[1]:.4f},{names[phase]}\n")
palette = np.array(["#f47770", "#89eab5", "#36adf1"])
fig, axes = plt.subplots(1, 2, figsize=(12, 10))
axes[0].imshow(poster)
axes[0].plot(guide_px[:, 0], guide_px[:, 1], color="yellow", lw=0.8)
axes[0].set(xlim=(40, 1370), ylim=(1970, 240), title="Extracted line over supplied poster")
closed = np.vstack([guide, guide[:1]])
map_centre = (centre - pixel_track[0]) * [scale, -scale]
map_normals = normals * [1, -1]
left, right = map_centre + 3 * map_normals, map_centre - 3 * map_normals
road = np.stack([left, right, np.roll(right, -1, axis=0), np.roll(left, -1, axis=0)], axis=1)
axes[1].add_collection(PolyCollection(road, facecolors="#45484d", edgecolors="none"))
axes[1].add_collection(LineCollection(np.stack([closed[:-1], closed[1:]], axis=1), colors=palette[phases], linewidths=2, zorder=3))
axes[1].set(aspect="equal", title="Ground coaching line (green / blue / red)")
axes[1].autoscale_view()
axes[1].set_facecolor("#202329")
fig.tight_layout()
fig.savefig(args.out / "guide.png", dpi=150)
report = {"retrace_max_difference_m": retrace_difference, "metres_per_px": scale,
          "poster_origin_px": pixel_track[0].tolist(), "samples": len(guide),
          "source_coverage": coverage, "max_inset_adjustment_m": float(np.max(np.abs(offsets-limited))*scale),
          "phase_samples": {str(n): int(np.sum(phases == i)) for i, n in enumerate(names)}}
(args.out / "guide-summary.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
