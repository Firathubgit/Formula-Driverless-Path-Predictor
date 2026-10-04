"""The poster's coloured line, and what a kart can do on it (decision 0046).

The operator's poster paints one racing line round Gokartcentralen Göteborg in three colours, gas, coast and brake
(tracks/gokartcentralen-goteborg-guide.csv, decision 0039). This module smooths that line into a path with honest
curvature, sets a kart's grip, power and brakes against it in a quasi-steady-state lap (the line as a closed path, speed
limited by cornering grip, power and braking, forwards and backwards), runs the project's own plant round it through
fd_headless, and compares the plant's gas, coast and brake with the poster's colours at the same places.

Needs numpy and scipy (Python 3.8 in .tools/tum-python is enough).
"""

import csv
import math
import os
import re
import shutil
import subprocess
import tempfile

import numpy as np
from scipy.interpolate import splev, splprep
from scipy.spatial import cKDTree

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
GUIDE = os.path.join(ROOT, "tracks", "gokartcentralen-goteborg-guide.csv")
POSTER_LINE = os.path.join(ROOT, "tracks", "gokartcentralen-goteborg-poster-line.csv")
HEADLESS = os.path.join(ROOT, "build", "core", "apps", "headless", "Release", "fd_headless.exe")
G = 9.80665
GAS, COAST, BRAKE = 0, 1, 2
NAMES = ("gas", "coast", "brake")
# What counts as braking and as coasting when a plant's motion is turned back into the poster's three colours: braking is
# deceleration beyond 0.4 g, coasting the speed held or nearly so. Fitted once, on the model's own lap against the poster
# (fit_kart.py --thresholds), then fixed for every car.
BRAKE_THRESHOLD_MPS2 = -4.0
COAST_BAND_MPS2 = 0.4


class PosterLine:
    """The poster's line smoothed by a periodic spline, sampled every `step` metres, with its curvature and colours."""

    def __init__(self, sigma_m=0.25, step_m=0.25, guide=GUIDE):
        rows = list(csv.DictReader(l for l in open(guide) if not l.startswith("#")))
        x = np.array([float(r["x_m"]) for r in rows])
        y = np.array([float(r["y_m"]) for r in rows])
        code = np.array([{"accelerate": GAS, "coast": COAST, "brake": BRAKE}[r["advice"]] for r in rows])
        n = len(x)
        # The traced stroke is noisy at the scale of a kart (radii under a metre are tracing noise): smooth within sigma.
        tck, _ = splprep([np.append(x, x[0]), np.append(y, y[0])], s=n * sigma_m**2, per=1)
        fine = np.linspace(0, 1, 20000, endpoint=False)
        px, py = splev(fine, tck)
        seg = np.hypot(np.diff(np.append(px, px[0])), np.diff(np.append(py, py[0])))
        self.length_m = float(seg.sum())
        s_fine = np.concatenate(([0], np.cumsum(seg)[:-1]))
        m = int(round(self.length_m / step_m))
        self.s = np.linspace(0, self.length_m, m, endpoint=False)
        u = np.interp(self.s, np.append(s_fine, self.length_m), np.append(fine, 1.0))
        self.x, self.y = splev(u, tck)
        dx, dy = splev(u, tck, der=1)
        ddx, ddy = splev(u, tck, der=2)
        self.kappa = (dx * ddy - dy * ddx) / (dx**2 + dy**2) ** 1.5
        d2 = (self.x[:, None] - x[None, :]) ** 2 + (self.y[:, None] - y[None, :]) ** 2
        self.colour = code[np.argmin(d2, axis=1)]
        self.tree = cKDTree(np.column_stack((self.x, self.y)))

    @property
    def ds(self):
        return self.length_m / len(self.s)

    def colour_metres(self):
        return [float((self.colour == c).sum() * self.ds) for c in (GAS, COAST, BRAKE)]


def qss_lap(line, mu, mass_kg, power_w, force_limit_n, drag_area_m2, top_speed_mps, brake_g, rolling=0.015):
    """The lap a kart with this much grip, power and braking can drive on the line: its speed at each point, the along-track
    acceleration, and the time. Grip is one coefficient for both directions and shared between cornering and
    accelerating or braking by the friction circle; the drive controller fades the force out over the last 5% of top speed."""
    n = len(line.s)
    ds = line.ds
    k = np.abs(line.kappa)
    v = np.minimum(np.sqrt(mu * G / np.maximum(k, 1e-6)), top_speed_mps)
    resist = lambda s: (0.5 * 1.225 * drag_area_m2 * s**2 + rolling * mass_kg * G) / mass_kg

    def circle(speed, kk):
        return np.sqrt(np.maximum((mu * G) ** 2 - (speed**2 * kk) ** 2, 0))

    for _ in range(3):
        for _ in range(2):
            for i in range(n - 1, -n, -1):
                j, nxt = i % n, (i + 1) % n
                dec = min(circle(v[nxt], k[nxt]), brake_g * G) + resist(v[nxt])
                v[j] = min(v[j], math.sqrt(v[nxt] ** 2 + 2 * dec * ds))
        for _ in range(2):
            for i in range(2 * n):
                j, nxt = i % n, (i + 1) % n
                fade = np.clip((top_speed_mps - v[j]) / (0.05 * top_speed_mps), 0, 1)
                drive = min(force_limit_n, power_w / max(v[j], 0.5)) * fade / mass_kg
                acc = min(circle(v[j], k[j]), drive) - resist(v[j])
                v[nxt] = min(v[nxt], math.sqrt(max(v[j] ** 2 + 2 * acc * ds, 0)))
    a = (np.roll(v, -1) ** 2 - v**2) / (2 * ds)
    return v, a, float(np.sum(ds / np.maximum(0.5 * (v + np.roll(v, -1)), 0.1)))


def colours_of(accel_mps2):
    """Gas, coast or brake from the along-track acceleration."""
    code = np.full(len(accel_mps2), GAS, dtype=int)
    code[accel_mps2 < COAST_BAND_MPS2] = COAST
    code[accel_mps2 < BRAKE_THRESHOLD_MPS2] = BRAKE
    return code


def agreement(line, code, tolerance_m=3.0):
    """Share of the lap on which the model's colour matches the poster's exactly, and within `tolerance_m` either way."""
    k = int(tolerance_m / line.ds)
    near = np.zeros(len(code), dtype=bool)
    for shift in range(-k, k + 1):
        near |= np.roll(code, shift) == line.colour
    return float((code == line.colour).mean()), float(near.mean())


# ---------------------------------------------------------------- the project's own plant, through fd_headless

def envelope_table(profile, sets=None, headless=HEADLESS):
    """The plant's performance envelope for a profile (and tuned values) as {speed: array of lateral share, lateral,
    forward, braking}, the left side."""
    out = tempfile.mkdtemp(prefix="kart-envelope-")
    shutil.rmtree(out)
    args = [headless, "--profile", profile]
    for name, value in (sets or {}).items():
        args += ["--car-set", "%s=%r" % (name, value)]
    run = subprocess.run(args + ["--write-envelope", out], capture_output=True, text=True)
    if run.returncode != 0:
        raise RuntimeError(run.stderr.strip() or run.stdout.strip())
    rows = [r for r in csv.reader(l for l in open(os.path.join(out, "envelope.csv")) if not l.startswith("#"))][1:]
    shutil.rmtree(out, ignore_errors=True)
    table = {}
    for r in rows:
        if r[1] == "left":
            table.setdefault(float(r[0]), []).append((float(r[2]), float(r[3]), float(r[4]), float(r[5])))
    return {v: np.array(sorted(rows)) for v, rows in table.items()}


def envelope_lap(line, table, share=1.0):
    """The lap the plant's own envelope allows on the line at a share of it: the plan the project's speed plan makes."""
    speeds = sorted(table)

    def one(speed, ay):
        t = table[speed]
        ay_max = t[-1, 1] * share
        frac = min(abs(ay) / max(ay_max, 1e-6), 1.0)
        return np.array([ay_max, np.interp(frac, t[:, 0], t[:, 2]) * share, np.interp(frac, t[:, 0], t[:, 3]) * share])

    def limits(v, ay):
        v = min(max(v, speeds[0]), speeds[-1])
        i = int(np.searchsorted(speeds, v))
        if i == 0 or speeds[i] == v:
            return one(speeds[i], ay)
        a, b = speeds[i - 1], speeds[i]
        f = (v - a) / (b - a)
        return one(a, ay) * (1 - f) + one(b, ay) * f

    n, ds, k = len(line.s), line.ds, np.abs(line.kappa)
    vs = np.linspace(speeds[0], speeds[-1], 300)
    ay_of_v = np.array([limits(v, 0)[0] for v in vs])
    v = np.array([vs[(vs**2 * k[i]) <= ay_of_v].max() if ((vs**2 * k[i]) <= ay_of_v).any() else vs[0] for i in range(n)])
    for _ in range(3):
        for i in range(n - 1, -n, -1):
            j, nxt = i % n, (i + 1) % n
            brk = limits(v[nxt], v[nxt] ** 2 * k[nxt])[2]
            v[j] = min(v[j], math.sqrt(v[nxt] ** 2 - 2 * brk * ds))
        for i in range(2 * n):
            j, nxt = i % n, (i + 1) % n
            fwd = limits(v[j], v[j] ** 2 * k[j])[1]
            v[nxt] = min(v[nxt], math.sqrt(max(v[j] ** 2 + 2 * max(fwd, 0) * ds, 0)))
    a = (np.roll(v, -1) ** 2 - v**2) / (2 * ds)
    return v, a, float(np.sum(ds / np.maximum(0.5 * (v + np.roll(v, -1)), 0.1)))


def drive_plant(profile, share, out, laps=3, sets=None, headless=HEADLESS):
    """Drives the plant round the poster's line on its own envelope plan at a share of it, as the desktop's kart is driven
    (Pure Pursuit aiming 2 m ahead), through fd_headless. Returns the lap times the judge saw and the run's verdicts."""
    if os.path.exists(out):
        shutil.rmtree(out)
    config = os.path.join(tempfile.gettempdir(), "fd-kart-config.cfg")
    with open(config, "w") as f:
        f.write("schema_version=1\nlookahead_base_m=2.0\n")
    args = [headless, "--config", config, "--profile", profile, "--track", POSTER_LINE, "--track-width", "3",
            "--track-smoothing", "0.05", "--speed-plan", "envelope", "--envelope-fraction", str(share), "--laps", str(laps),
            "--out", out]
    for name, value in (sets or {}).items():
        args += ["--car-set", "%s=%r" % (name, value)]
    run = subprocess.run(args, capture_output=True, text=True)
    text = run.stdout + run.stderr
    result = {"ok": run.returncode == 0}
    m = re.search(r"laps timed \(([^)]*)\)", text)
    result["laps_s"] = [float(x) for x in re.findall(r"\d+\.\d+", m.group(1).replace("best", ""))][:laps] if m else []
    m = re.search(r"(\d+) cone\(s\) down or out, (\d+) time\(s\) off course", text)
    result["cones"], result["off_course"] = (int(m.group(1)), int(m.group(2))) if m else (-1, -1)
    m = re.search(r"max rear sideslip: ([\d.]+) rad", text)
    result["rear_sideslip_rad"] = float(m.group(1)) if m else float("nan")
    m = re.search(r"max tracking error: ([\d.]+) m", text)
    result["tracking_error_m"] = float(m.group(1)) if m else float("nan")
    return result


def plant_colours(line, out, lap_index=1):
    """The plant's gas, coast and brake on one flying lap of a run, from its achieved acceleration, against the poster's
    colours at the same place: exact and within 3 m, and the metres of each colour."""
    rows = list(csv.DictReader(open(os.path.join(out, "telemetry.csv"))))
    laps = np.array([int(float(r["laps"])) for r in rows])
    keep = laps == lap_index
    if keep.sum() < 100:
        return None
    t = np.array([float(r["time_s"]) for r in rows])[keep]
    x = np.array([float(r["x_m"]) for r in rows])[keep]
    y = np.array([float(r["y_m"]) for r in rows])[keep]
    v = np.array([float(r["speed_mps"]) for r in rows])[keep]
    a = np.gradient(v, t)
    k = 30  # 0.15 s
    a = np.convolve(np.pad(a, k, mode="edge"), np.ones(2 * k + 1) / (2 * k + 1), mode="valid")
    code = colours_of(a)
    idx = line.tree.query(np.column_stack((x, y)))[1]
    w = np.maximum(v, 0.1) * np.gradient(t)  # a metre of travel counts once
    poster = line.colour[idx]
    near = np.zeros(len(code), dtype=bool)
    for d in range(-3, 4):
        j = np.argmin(np.abs(((line.s[None, :] - (line.s[idx][:, None] + d)) + line.length_m / 2) % line.length_m - line.length_m / 2), axis=1)
        near |= line.colour[j] == code
    return {"exact": float(np.sum(w * (code == poster)) / np.sum(w)), "within_3m": float(np.sum(w * near) / np.sum(w)),
            "metres": [float(np.sum(w * (code == c))) for c in (GAS, COAST, BRAKE)]}
