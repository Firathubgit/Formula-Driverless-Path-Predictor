"""Traces a track poster into a closed centreline, scaled to the published lap length (decision 0038).

python trace_poster.py POSTER POINTS.json OUT_DIR [--track-out FILE]

Made for Gokartcentralen Göteborg's published track poster (1500 x 2000 px; not in the repository, the user supplies it):
POINTS.json holds control points in the poster's pixels, in lap order from the start line. Each is snapped,
perpendicular to the lap, to the middle of the drawn asphalt (between the poster's white lines) unless marked "fixed"; a
periodic smoothing spline through them is resampled every metre and scaled so the lap is exactly 400 m. OUT_DIR gets the
evidence the verification judged: overlay.png (the trace and a 6 m corridor on the poster), side-by-side.png,
differences.png, summary.json (scale, direction, the drawn width about the line, overlap) and centreline CSVs.
--track-out writes the track file the simulator reads (x_m,y_m from the start, map y up).

Run it with the Python environment that has numpy, scipy and Pillow (.tools/tum-python/Scripts/python.exe).
"""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy import interpolate, ndimage

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("poster", type=Path)
parser.add_argument("points", type=Path)
parser.add_argument("out", type=Path)
parser.add_argument("--track-out", type=Path)
options = parser.parse_args()
out = options.out
out.mkdir(parents=True, exist_ok=True)
spec = json.loads(options.points.read_text(encoding="utf-8"))
poster = Image.open(options.poster).convert("RGB")
rgb = np.asarray(poster).astype(np.float32)
lum = rgb @ np.array([0.2126, 0.7152, 0.0722], dtype=np.float32)
sat = rgb.max(axis=2)-rgb.min(axis=2)
H, W = lum.shape
yy, xx = np.mgrid[0:H, 0:W]

# Barriers: the white lines and islands. Within the footbridge's grey band, what was white beneath it stays a barrier.
barrier = (lum > 185) & (sat < 45)
bridge = (yy >= 1160) & (yy <= 1220) & (xx >= 705) & (xx <= 1195)
barrier |= bridge & (lum > 100) & (sat < 45)
# The pit lane's blue box is not track.
barrier |= (rgb[..., 2] > 150) & (rgb[..., 0] < 120) & (sat > 60)
# Markings painted on the asphalt (grid boxes, the start line's checks, dotted lines, arrows) are not barriers:
# only white regions of real extent are.
labels, count = ndimage.label(barrier)
sizes = ndimage.sum(np.ones_like(lum), labels, index=np.arange(1, count+1))
small = np.zeros(count+1, dtype=bool)
small[1:] = sizes < 2500
barrier[small[labels]] = False
# The velodrome's shading at T9 is banked asphalt, not a barrier, however light.
for x0, x1, y0, y1 in spec.get("asphalt_boxes", []):
    barrier[y0:y1, x0:x1] = False
for (x0, y0), (x1, y1) in spec.get("walls", []):
    n = int(max(abs(x1-x0), abs(y1-y0)))+1
    for t in np.linspace(0, 1, n):
        x, y = int(round(x0+t*(x1-x0))), int(round(y0+t*(y1-y0)))
        barrier[max(0, y-2):y+3, max(0, x-2):x+3] = True


def ray(x, y, dx, dy, limit=260):
    """Distance along (dx, dy) from (x, y) to the first barrier pixel, or None within the limit."""
    for step in np.arange(0.5, limit, 0.5):
        px, py = int(round(x+dx*step)), int(round(y+dy*step))
        if px < 0 or py < 0 or px >= W or py >= H or barrier[py, px]:
            return step
    return None


def closed_spline(points, smoothing, count):
    pts = np.asarray(points, dtype=float)
    pts = np.vstack([pts, pts[:1]])
    tck, _ = interpolate.splprep([pts[:, 0], pts[:, 1]], s=smoothing, per=1, k=3)
    u = np.linspace(0, 1, count, endpoint=False)
    x, y = interpolate.splev(u, tck)
    dx, dy = interpolate.splev(u, tck, der=1)
    return np.c_[x, y], np.c_[dx, dy]


control = [p["at"] for p in spec["points"]]
fixed = [bool(p.get("fixed")) for p in spec["points"]]
names = [p.get("name", "") for p in spec["points"]]
snapped = [list(p) for p in control]
report = []
for sweep in range(3):
    dense, tangent = closed_spline(snapped, len(snapped)*4.0, 4000)
    for i, p in enumerate(snapped):
        if fixed[i]:
            continue
        j = int(np.argmin(np.hypot(dense[:, 0]-p[0], dense[:, 1]-p[1])))
        tx, ty = tangent[j]/np.hypot(*tangent[j])
        nx, ny = -ty, tx
        a, b = ray(p[0], p[1], nx, ny), ray(p[0], p[1], -nx, -ny)
        if a is None or b is None or a+b > spec.get("max_band_px", 330):
            continue
        shift = (a-b)/2
        # Never move a point further than a band's half width from where it was placed.
        if abs(shift) > 90:
            continue
        snapped[i] = [p[0]+nx*shift, p[1]+ny*shift]
        if sweep == 2:
            report.append((i, names[i], round(shift, 1), round(a+b, 1)))

dense, tangent = closed_spline(snapped, len(snapped)*spec.get("smoothing", 6.0), 20000)
seg = np.hypot(*np.diff(np.vstack([dense, dense[:1]]), axis=0).T)
length_px = seg.sum()
scale = 400.0/length_px
# Resample every metre of the scaled lap.
cum = np.r_[0, np.cumsum(seg)]
targets = np.arange(0, 400.0, 1.0)/scale
xs = np.interp(targets, cum, np.r_[dense[:, 0], dense[0, 0]])
ys = np.interp(targets, cum, np.r_[dense[:, 1], dense[0, 1]])
samples = np.c_[xs, ys]
# The drawn corridor at each sample: the distance to the white on either side, perpendicular to the lap.
widths = []
for k in range(len(samples)):
    a, b = samples[(k+1) % len(samples)], samples[k-1]
    t = (a-b)/np.hypot(*(a-b))
    left = ray(samples[k][0], samples[k][1], t[1], -t[0])   # left of travel on screen (x right, y down)
    right = ray(samples[k][0], samples[k][1], -t[1], t[0])
    widths.append((left, right))
left_m = np.array([w[0]*scale if w[0] else np.nan for w in widths])
right_m = np.array([w[1]*scale if w[1] else np.nan for w in widths])
total = left_m+right_m
# Signed area in map coordinates (y up): negative is clockwise, a right-hand circuit.
mx, my = samples[:, 0]*scale, -samples[:, 1]*scale
area = 0.5*np.sum(mx*np.roll(my, -1)-np.roll(mx, -1)*my)
summary = {"length_px": float(length_px), "metres_per_px": float(scale), "control_points": len(control),
           "signed_area_m2": float(area), "direction": "clockwise" if area < 0 else "counterclockwise",
           "width_m": {"median": float(np.nanmedian(total)), "p10": float(np.nanpercentile(total, 10)),
                       "p90": float(np.nanpercentile(total, 90)), "missing": int(np.isnan(total).sum())},
           "footprint_m": [float(np.ptp(mx)), float(np.ptp(my))], "snaps": report}
(out/"summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
with open(out/"centreline-px.csv", "w", encoding="utf-8", newline="\n") as f:
    f.write("x_px,y_px,left_m,right_m\n")
    for (x, y), l, r in zip(samples, left_m, right_m):
        f.write(f"{x:.2f},{y:.2f},{l:.3f},{r:.3f}\n")
x0, y0 = samples[0]
metres = "".join(f"{(x-x0)*scale:.3f},{-(y-y0)*scale:.3f}\n" for x, y in samples)
with open(out/"centreline.csv", "w", encoding="utf-8", newline="\n") as f:
    f.write("x_m,y_m\n"+metres)
if options.track_out:
    with open(options.track_out, "w", encoding="utf-8", newline="\n") as f:
        f.write("# Gokartcentralen Göteborg (Bergslagsgatan 6), traced from the operator's published track poster (decision 0038):\n"
                "# the middle of the drawn asphalt, clockwise from the start line toward T1, resampled every metre and scaled so the lap\n"
                "# is exactly 400 m (published: 400 m long, 6 m wide). Made by tools/tracks/trace_poster.py from\n"
                "# tools/tracks/gokartcentralen-goteborg.points.json.\n"
                "x_m,y_m\n"+metres)

# Overlay on the poster: the traced centreline, the published 6 m corridor about it, the control points and the start.
over = poster.copy().convert("RGBA")
layer = Image.new("RGBA", over.size, (0, 0, 0, 0))
d = ImageDraw.Draw(layer)
half = 3.0/scale
normals = []
for k in range(len(samples)):
    a, b = samples[(k+1) % len(samples)], samples[k-1]
    t = (a-b)/np.hypot(*(a-b))
    normals.append((t[1], -t[0]))
normals = np.asarray(normals)
for sign in (1, -1):
    edge = samples+sign*half*normals
    d.line([tuple(p) for p in np.vstack([edge, edge[:1]])], fill=(255, 200, 0, 255), width=3)
d.line([tuple(p) for p in np.vstack([samples, samples[:1]])], fill=(255, 0, 60, 255), width=4)
for i, p in enumerate(snapped):
    d.ellipse([p[0]-5, p[1]-5, p[0]+5, p[1]+5], fill=(0, 255, 255, 255) if not fixed[i] else (255, 0, 255, 255))
d.ellipse([x0-10, y0-10, x0+10, y0+10], outline=(255, 255, 255, 255), width=4)
k = 12
d.line([tuple(samples[0]), tuple(samples[k])], fill=(255, 255, 255, 255), width=6)
over = Image.alpha_composite(over, layer).convert("RGB")
over.save(out/"overlay.png")
# The traced track alone, as the simulator will lay it, 6 m wide, with sample 0 marked.
S = 10.0
margin = 30
ox, oy = mx.min(), my.max()
img = Image.new("RGB", (int(np.ptp(mx)*S)+2*margin, int(np.ptp(my)*S)+2*margin), (8, 9, 11))
g = ImageDraw.Draw(img)
pts = [((x-ox)*S+margin, (oy-y)*S+margin) for x, y in zip(mx, my)]
g.line(pts+pts[:1], fill=(60, 64, 72), width=int(6*S))
g.line(pts+pts[:1], fill=(230, 230, 230), width=2)
g.ellipse([pts[0][0]-8, pts[0][1]-8, pts[0][0]+8, pts[0][1]+8], fill=(255, 60, 60))
img.save(out/"track-alone.png")
# The drawn asphalt: everything not a barrier that is connected to the start line, the pit lane closed off at its mouths.
drawn = ~barrier
for (x0w, y0w), (x1w, y1w) in spec.get("pit_walls", []):
    n = int(max(abs(x1w-x0w), abs(y1w-y0w)))+1
    for t in np.linspace(0, 1, n):
        x, y = int(round(x0w+t*(x1w-x0w))), int(round(y0w+t*(y1w-y0w)))
        drawn[max(0, y-3):y+4, max(0, x-3):x+4] = False
parts, _ = ndimage.label(drawn)
drawn = parts == parts[int(round(y0)), int(round(x0))]
# The traced 6 m corridor rasterised in the poster's frame.
corridor_img = Image.new("L", poster.size, 0)
ImageDraw.Draw(corridor_img).line([tuple(p) for p in np.vstack([samples, samples[:1]])], fill=255, width=int(round(6.0/scale)), joint="curve")
corridor = np.asarray(corridor_img) > 0
inter = (corridor & drawn).sum()
overlap = {"corridor_on_drawn_asphalt": float(inter/corridor.sum()), "drawn_asphalt_covered": float(inter/drawn.sum()),
           "iou": float(inter/(corridor | drawn).sum()), "drawn_asphalt_px": int(drawn.sum())}
summary["overlap"] = overlap
(out/"summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
# Side by side in the same frame: the poster's track area, and the trace alone as the simulator lays it.
x_lo, x_hi, y_lo, y_hi = 40, 1370, 240, 1970
left_img = poster.crop((x_lo, y_lo, x_hi, y_hi))
right_img = Image.new("RGB", poster.size, (18, 18, 18))
r = ImageDraw.Draw(right_img)
ring = [tuple(p) for p in np.vstack([samples, samples[:1]])]
r.line(ring, fill=(235, 235, 235), width=int(round(6.0/scale))+16, joint="curve")
r.line(ring, fill=(30, 30, 30), width=int(round(6.0/scale)), joint="curve")
r.line(ring, fill=(255, 60, 60), width=3)
for p, name in zip(snapped, names):
    if name:
        r.text((p[0]+12, p[1]-8), name, fill=(120, 220, 255))
r.ellipse([x0-9, y0-9, x0+9, y0+9], fill=(255, 255, 255))
right_img = right_img.crop((x_lo, y_lo, x_hi, y_hi))
pair = Image.new("RGB", (left_img.width*2+20, left_img.height), (0, 0, 0))
pair.paste(left_img, (0, 0))
pair.paste(right_img, (left_img.width+20, 0))
pair.save(out/"side-by-side.png")
drawn_img = poster.copy()
tint = np.asarray(drawn_img).copy()
tint[drawn & ~corridor] = (0, 120, 255)
tint[corridor & ~drawn] = (255, 40, 40)
Image.fromarray(tint).save(out/"differences.png")
print(json.dumps({k: v for k, v in summary.items() if k != "snaps"}, indent=1))
