"""Generate web/data/track.json for the browser gokart from the repo's own data.

Sources (read-only, unchanged):
  tracks/gokartcentralen-goteborg.csv            the traced centreline, 400 m, 6 m wide
  tracks/gokartcentralen-goteborg-poster-line.csv the operator's poster racing line (the coach line)
  tracks/gokartcentralen-goteborg-guide.csv       poster coaching colours (accelerate/coast/brake)
  core/src/vehicle_profiles.cpp                   the fitted rental-kart parameters
  core/src/track_relief.cpp                       the banked T9 bend (292-336 m, 8 m ramp, 6 deg)
  apps/desktop/kart_hall.cpp                      the hall's dimensions

Scene coordinates follow the desktop: x is the map's x, z the map's -y, y up.
"""
import json
import math
import os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT = os.path.join(ROOT, "web", "data", "track.json")


def read_xy(path):
    rows = []
    for line in open(path):
        line = line.strip()
        if not line or line.startswith("#") or line.startswith("x_m"):
            continue
        a, b = line.split(",")[:2]
        rows.append((float(a), float(b)))
    return rows


def read_guide(path):
    rows = []
    for line in open(path):
        line = line.strip()
        if not line or line.startswith("#") or line.startswith("x_m"):
            continue
        a, b, c = line.split(",")[:3]
        rows.append((float(a), float(b), c))
    return rows


track = read_xy(os.path.join(ROOT, "tracks", "gokartcentralen-goteborg.csv"))
poster = read_xy(os.path.join(ROOT, "tracks", "gokartcentralen-goteborg-poster-line.csv"))
guide = read_guide(os.path.join(ROOT, "tracks", "gokartcentralen-goteborg-guide.csv"))
n = len(track)

# Cumulative station along the closed loop.
s = [0.0]
for i in range(1, n):
    dx = track[i][0] - track[i - 1][0]
    dy = track[i][1] - track[i - 1][1]
    s.append(s[-1] + math.hypot(dx, dy))
length = s[-1] + math.hypot(track[0][0] - track[-1][0], track[0][1] - track[-1][1])

# Tangent and signed curvature at each sample (central differences, wrapped).
def wrap(i):
    return i % n

tangents, curv = [], []
for i in range(n):
    a = track[wrap(i - 1)]
    b = track[wrap(i + 1)]
    dx, dy = b[0] - a[0], b[1] - a[1]
    d = math.hypot(dx, dy) or 1.0
    tangents.append((dx / d, dy / d))
for i in range(n):
    t0 = tangents[wrap(i - 1)]
    t1 = tangents[wrap(i + 1)]
    ds = (s[wrap(i + 1)] - s[wrap(i - 1)]) if i > 0 else 2.0
    if ds <= 0:
        ds = 2.0
    cross = t0[0] * t1[1] - t0[1] * t1[0]
    curv.append(cross / ds)

# Corners: contiguous stretches where the radius drops below 40 m, with a 1.5 m gap join.
CORNER_RADIUS = 40.0
tight = [1.0 / abs(k) < CORNER_RADIUS if abs(k) > 1e-9 else False for k in curv]
corners = []
i = 0
while i < n:
    if tight[i]:
        j = i
        while j + 1 < n and (tight[j + 1] or (j + 2 < n and tight[j + 2])):
            j += 1
        mid = wrap((i + j) // 2)
        direction = 1 if curv[mid] > 0 else -1
        corners.append({
            "from_s": round(s[i] - 2.0, 2),
            "to_s": round(s[j] + 2.0, 2),
            "direction": direction,
            "apex_s": round(s[mid], 2),
            "min_radius_m": round(1.0 / abs(curv[mid]), 2) if abs(curv[mid]) > 1e-9 else None,
        })
        i = j + 1
    else:
        i += 1

half = 3.0  # width 6 m
xs = [p[0] for p in track]
ys = [p[1] for p in track]
hall = {
    "min_x": round(min(xs) - half - 1.6, 2),
    "max_x": round(max(xs) + half + 7.0, 2),
    "min_z": round(-(max(ys) + half + 1.6), 2),
    "max_z": round(-(min(ys) - half - 5.0), 2),
    "eave_m": 6.0, "ridge_m": 9.0, "lamp_height_m": 5.6,
    "lamp_dx": 9.0, "lamp_dz": 3.6,
}
hall["lamp_columns"] = max(1, int((hall["max_x"] - hall["min_x"] - 5.0) / hall["lamp_dx"]) + 1)
hall["lamp_rows"] = max(1, int((hall["max_z"] - hall["min_z"] - 4.0) / hall["lamp_dz"]) + 1)
hall["lamp_x0"] = round((hall["min_x"] + hall["max_x"]) / 2 - (hall["lamp_columns"] - 1) * hall["lamp_dx"] / 2, 2)
hall["lamp_z0"] = round((hall["min_z"] + hall["max_z"]) / 2 - (hall["lamp_rows"] - 1) * hall["lamp_dz"] / 2, 2)

# The fitted rental kart (gokartcentralen-realistic) with the desktop's 40% tire-friction bump (decision 0055).
kart = {
    "mass_kg": 261.0,
    "wheelbase_m": 1.045,
    "cg_to_front_m": 0.645,
    "cg_to_rear_m": 0.4,
    "cg_height_m": 0.25,
    "track_front_m": 1.055,
    "track_rear_m": 1.2,
    "wheel_radius_m": 0.139,
    "yaw_inertia_kgm2": 25.0 * 261.0 / 165.0,
    "roll_balance_front": 17.7 / (17.7 + 60.0),
    "max_steering_rad": 0.55,
    "max_steering_rate_radps": 3.0,
    "max_drive_power_w": 14 * 735.49875,
    "max_drive_torque_nm": 115.0,
    "max_drive_speed_mps": 60.0 / 3.6,
    "brake_bias_front": 0.4,
    "max_brake_torque_nm": 150.0,
    "drag_area_m2": 0.78,
    "grip_lateral": 1.35 * 1.4,
    "grip_longitudinal": 1.0 * 1.4,
    "peak_slip_angle_rad": 0.091,
    "peak_slip_ratio": 0.088,
    "sliding_fraction_lateral": 0.61,
    "sliding_fraction_longitudinal": 0.71,
    "rolling_resistance": 0.015,
}

data = {
    "source": "Formula-Driverless-Path-Predictor (Gokartcentralen Göteborg)",
    "track": {
        "name": "Gokartcentralen Göteborg",
        "width_m": 6.0,
        "length_m": round(length, 2),
        "points": [[round(x, 4), round(y, 4)] for x, y in track],
        "stations": [round(v, 3) for v in s],
        "curvature": [round(k, 6) for k in curv],
    },
    "posterLine": [[round(x, 4), round(y, 4)] for x, y in poster],
    "guide": [{"x": round(x, 4), "y": round(y, 4), "advice": a} for x, y, a in guide],
    "corners": corners,
    "banking": {"from_s_m": 292.0, "to_s_m": 336.0, "ramp_m": 8.0, "tilt_rad": 6.0 * math.pi / 180},
    "hall": hall,
    "kart": {k: round(v, 6) for k, v in kart.items()},
}

os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "w") as f:
    json.dump(data, f, separators=(",", ":"))

print("wrote", OUT)
print("track points:", n, "length %.2f m" % length, "corners:", len(corners))
print("poster points:", len(poster), "guide points:", len(guide))
print("hall x[%.1f,%.1f] z[%.1f,%.1f] lamps %dx%d" % (hall["min_x"], hall["max_x"], hall["min_z"], hall["max_z"], hall["lamp_columns"], hall["lamp_rows"]))
for c in corners:
    print("  corner s=%.1f..%.1f dir=%+d rmin=%s" % (c["from_s"], c["to_s"], c["direction"], c["min_radius_m"]))
print("json bytes:", os.path.getsize(OUT))
