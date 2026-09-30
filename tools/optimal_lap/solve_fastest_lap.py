"""Solve a lap problem's minimum-time lap with fastest-lap and write a reference-optimal artefact.

TrackWayFastPlan Phase 4.1, decision 0034. The problem is written by

    fd_headless --profile formula-one-style --write-lap-problem PROBLEM_DIR

and this script puts its car into fastest-lap's 3-DOF car model (f1-3dof) and its corridor into fastest-lap's discrete
track format, runs fastest-lap's optimal lap time (Ipopt) in a child process whose output it keeps as solver.log, and
writes, into a fresh directory, optimal_lap.json (the solver, its version and digests, its exit status, iterations,
tolerance, mesh, the lap time, how the car was mapped, each tire's dissipated energy and the setup sensitivities),
lap.csv (every mesh point in this project's frame) and the problem's own problem.json and corridor.csv. The artefact is
validated by fd_headless --check-optimal-lap, with the flags the problem was written with.

fastest-lap is not part of this repository and is not linked into it: it is the prebuilt Windows release, downloaded
once with the user's approval and checked against the digests pinned below. It needs Python 3.8 with numpy; see
README.md beside this script.

fastest-lap's frame has z down: its y is this project's -y, and every lateral quantity, yaw, yaw rate, steering and slip
angle changes sign across the boundary. Nothing else is converted except the reference point, which stays fastest-lap's
centre of gravity here; the reader converts it to the rear axle.
"""

import argparse
import hashlib
import json
import math
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile

PINNED = {
    "tool": "fastest-lap",
    "version": "0.5",
    "release": "fastest-lap-w10-x64-msvc2022-v0.5.zip",
    # Computed here on download (2026-09-25); the release publishes no digest of its own.
    "release_sha256": "81947ae517c7029278ed7cef0c06e74a2664cec3130a573d66647483bd08831c",
    "library": "libfastestlapc-0.5.dll",
    "library_sha256": "28a22902b2bf8cd0b4c930033f092ea34dbd7ec69c29d1de5cc6678a85a8533c",
    "model": "f1-3dof",
    "nlp_solver": "Ipopt",
}
WHEELS = [("fl", "front-axle.left-tire"), ("fr", "front-axle.right-tire"),
          ("rl", "rear-axle.left-tire"), ("rr", "rear-axle.right-tire")]
LABEL = ("Offline optimum for fastest-lap's 3-DOF model of this car, at the stated mesh, tolerance and solver status: "
         "not a run of this project's plant")
LABEL_FLOORED = ("Offline optimum for fastest-lap's 3-DOF model of this car with its peak friction floored at 1.0, more "
                 "than this car has, at the stated mesh, tolerance and solver status: faster than this car can be, and not "
                 "a run of this project's plant")


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def read_problem(directory):
    problem = json.loads((directory / "problem.json").read_text(encoding="utf-8"))
    rows = (directory / "corridor.csv").read_text(encoding="utf-8").splitlines()
    header = rows[0].split(",")
    corridor = {name: [] for name in header}
    for row in rows[1:]:
        for name, value in zip(header, row.split(",")):
            corridor[name].append(float(value))
    return problem, corridor


# ---------------------------------------------------------------- the track

def periodic_derivative(values, spacing):
    n = len(values)
    return [(values[(i + 1) % n] - values[i - 1]) / (2 * spacing) for i in range(n)]


def unwrap(angles):
    out = [angles[0]]
    for a in angles[1:]:
        out.append(out[-1] + math.remainder(a - out[-1], 2 * math.pi))
    return out


def track_xml(problem, corridor, half_width):
    """fastest-lap's discrete closed track, in its frame: y mirrored, yaw and curvature negated. Its boundaries are the
    corridor's edges less the car's half width, so the centre of gravity it places keeps the whole car inside."""
    length = problem["corridor"]["length_m"]
    s = corridor["s_m"]
    n = len(s)
    spacing = length / n
    x = corridor["x_m"]
    y = [-v for v in corridor["y_m"]]
    yaw = unwrap([-h for h in corridor["heading_rad"]])
    yaw_dot = [-k for k in corridor["curvature_1pm"]]
    nl = [e - half_width for e in corridor["left_m"]]
    nr = [e - half_width for e in corridor["right_m"]]
    if min(nl) <= 0 or min(nr) <= 0:
        raise SystemExit("The corridor is narrower than the car at some sample")
    # fastest-lap's left, in its frame, lies along (sin yaw, -cos yaw): this project's left, mirrored.
    left_x = [x[i] + nl[i] * math.sin(yaw[i]) for i in range(n)]
    left_y = [y[i] - nl[i] * math.cos(yaw[i]) for i in range(n)]
    right_x = [x[i] - nr[i] * math.sin(yaw[i]) for i in range(n)]
    right_y = [y[i] + nr[i] * math.cos(yaw[i]) for i in range(n)]

    def row(values):
        return ", ".join(repr(float(v)) for v in values)

    def pair(tag, xs, ys):
        return "        <%s>\n            <x>%s</x>\n            <y>%s</y>\n        </%s>\n" % (tag, row(xs), row(ys), tag)

    return ("<circuit format=\"discrete\" type=\"closed\" dimensions=\"2\">\n"
            "    <header>\n        <track_length units=\"m\">%r</track_length>\n"
            # Its preprocessor's fit against measured boundaries; here the boundaries are the corridor's own.
            "        <L2_error_left>0.0</L2_error_left>\n        <L2_error_right>0.0</L2_error_right>\n"
            "        <max_error_left>0.0</max_error_left>\n        <max_error_right>0.0</max_error_right>\n    </header>\n"
            # Its preprocessor's weights, which the reader requires and nothing here uses: the corridor was conditioned
            # by this project (decision 0019), not by fastest-lap's preprocessor.
            "    <optimization>\n        <cost_curvature>0.0</cost_curvature>\n"
            "        <cost_track_limits_smoothness>0.0</cost_track_limits_smoothness>\n"
            "        <cost_track_limits_errors>0.0</cost_track_limits_errors>\n        <cost_centerline>0.0</cost_centerline>\n"
            "        <maximum_yaw_dot>0.0</maximum_yaw_dot>\n        <maximum_dyaw_dot>0.0</maximum_dyaw_dot>\n    </optimization>\n"
            # Where its map would lie on the Earth, also required and unused: the corridor is a map in metres already.
            "    <GPS_parameters>\n        <origin_longitude units=\"deg\">0.0</origin_longitude>\n"
            "        <origin_latitude units=\"deg\">0.0</origin_latitude>\n        <earth_radius units=\"m\">6378388</earth_radius>\n"
            "        <reference_latitude units=\"deg\">0.0</reference_latitude>\n    </GPS_parameters>\n"
            "    <data number_of_points=\"%d\">\n" % (float(length), n)
            + "        <arclength units=\"m\">%s</arclength>\n" % row(s)
            + pair("centerline", x, y) + pair("left_boundary", left_x, left_y) + pair("right_boundary", right_x, right_y)
            + pair("left_measured_boundary", left_x, left_y) + pair("right_measured_boundary", right_x, right_y)
            + "        <yaw>%s</yaw>\n" % row(yaw)
            + "        <yaw_dot>%s</yaw_dot>\n" % row(yaw_dot)
            + "        <nl>%s</nl>\n" % row(nl)
            + "        <nr>%s</nr>\n" % row(nr)
            + "        <dyaw_dot>%s</dyaw_dot>\n" % row(periodic_derivative(yaw_dot, spacing))
            + "        <dnl>%s</dnl>\n" % row(periodic_derivative(nl, spacing))
            + "        <dnr>%s</dnr>\n" % row(periodic_derivative(nr, spacing))
            + "    </data>\n</circuit>\n")


# ---------------------------------------------------------------- the car

def shape_for_sliding(fraction):
    """This project's Magic Formula shape factor Q for a sliding residual (core/tire.cpp)."""
    return 2 - 2 * math.asin(fraction) / math.pi


def peak_scale(shape):
    """fastest-lap's curve sin(Q atan(S rho)) uses S = pi/(2 atan Q), whose peak is not at rho = 1; this project's uses
    S = tan(pi/(2Q)), whose peak is. Scaling fastest-lap's peak slip by the ratio of the two makes the curves identical
    (TrackWayFastPlan 4.1(a))."""
    return (math.pi / (2 * math.atan(shape))) / math.tan(math.pi / (2 * shape))


FRICTION_FLOOR = 1.0  # fastest-lap 0.5's own, hard-coded; see friction_floor.py beside this script


def friction_below_floor(problem):
    """The peak frictions of the car's tires, times grip, that fall below fastest-lap's floor at the reference loads."""
    grip = problem["config"]["grip_mu"]
    found = []
    for axle in ("front_tire", "rear_tire"):
        t = problem["car"][axle]
        for key in ("peak_friction_longitudinal_low", "peak_friction_longitudinal_high", "peak_friction_lateral_low",
                    "peak_friction_lateral_high"):
            if grip * t[key] < FRICTION_FLOOR:
                found.append("%s %s %.4g" % (axle, key, grip * t[key]))
    return found


def vehicle_xml(problem, accept_floor=False):
    car, config = problem["car"], problem["config"]
    mapping = []
    if car["drive_front_fraction"] != 0:
        raise SystemExit("fastest-lap's f1-3dof drives the rear axle alone; this car drives its front axle too")
    below = friction_below_floor(problem)
    if below and not accept_floor:
        raise SystemExit("fastest-lap 0.5 floors every tire's peak friction at %.1f, and this car's falls below it (%s): it would "
                         "solve a car with more grip than this one. Give --accept-friction-floor to solve it anyway, stated "
                         "in the artefact" % (FRICTION_FLOOR, "; ".join(below)))
    if below:
        mapping.append("tires, found: fastest-lap 0.5 floors every tire's peak friction at %.1f, a floor fastest-lap sets "
                       "itself, and this car's falls below it (%s), so the solved car grips more than this one and its "
                       "optimum is faster than this car can be" % (FRICTION_FLOOR, "; ".join(below)))
    wheelbase = config["wheelbase_m"]
    rear = car["cg_to_rear_m"]
    front = wheelbase - rear
    height = car["cg_height_m"]
    radius = car["wheel_radius_m"]
    pressure = car["aero_balance_front"] * wheelbase - rear
    grip = config["grip_mu"]

    def tire(t, tag):
        qx, qy = shape_for_sliding(t["sliding_fraction_longitudinal"]), shape_for_sliding(t["sliding_fraction_lateral"])
        kx, ky = peak_scale(qx), peak_scale(qy)
        if abs(kx - ky) > 1e-12:
            mapping.append("%s: the longitudinal and lateral curves differ in shape, so their shared normalised slip is "
                           "scaled by %.4f and %.4f; combined slip then differs from this project's" % (tag, kx, ky))
        return ("    <%s model=\"tire-pacejka-simple\" type=\"normal\">\n" % tag
                + "        <radius units=\"m\">%r</radius>\n" % radius
                + "        <radial-stiffness>0.0</radial-stiffness>\n        <radial-damping>0.0</radial-damping>\n"
                + "        <Fz-max-ref2> 1.0 </Fz-max-ref2>\n"
                + "        <reference-load-1 units=\"N\">%r</reference-load-1>\n" % t["reference_load_low_n"]
                + "        <reference-load-2 units=\"N\">%r</reference-load-2>\n" % t["reference_load_high_n"]
                + "        <mu-x-max-1>%r</mu-x-max-1>\n" % (grip * t["peak_friction_longitudinal_low"])
                + "        <mu-x-max-2>%r</mu-x-max-2>\n" % (grip * t["peak_friction_longitudinal_high"])
                + "        <kappa-max-1>%r</kappa-max-1>\n" % (t["peak_slip_ratio_low"] * kx)
                + "        <kappa-max-2>%r</kappa-max-2>\n" % (t["peak_slip_ratio_high"] * kx)
                + "        <mu-y-max-1>%r</mu-y-max-1>\n" % (grip * t["peak_friction_lateral_low"])
                + "        <mu-y-max-2>%r</mu-y-max-2>\n" % (grip * t["peak_friction_lateral_high"])
                + "        <lambda-max-1 units=\"deg\">%r</lambda-max-1>\n" % math.degrees(t["peak_slip_angle_low_rad"] * ky)
                + "        <lambda-max-2 units=\"deg\">%r</lambda-max-2>\n" % math.degrees(t["peak_slip_angle_high_rad"] * ky)
                + "        <Qx>%r</Qx>\n        <Qy>%r</Qy>\n    </%s>\n" % (qx, qy, tag))

    bias = car["brake_bias_front"]
    # fastest-lap names one brake torque per axle; its split between the wheels is its own. The cap is set so that the
    # front wheels' share at this bias is this car's per-wheel cap, and the solution is checked not to reach it.
    brake_front = car["max_brake_torque_nm"] / max(bias, 1e-9)
    brake_rear = car["max_brake_torque_nm"] / max(1 - bias, 1e-9)
    text = ("<vehicle type=\"f1-3dof\">\n"
            "    <front-axle model=\"axle-car\">\n"
            "        <track units=\"m\">%r</track>\n" % car["track_front_m"]
            + "        <inertia units=\"kg.m2\">%r</inertia>\n" % car["wheel_inertia_kgm2"]
            + "        <smooth_throttle_coeff> 1.0e-5 </smooth_throttle_coeff>\n"
            "        <brakes>\n            <max_torque units=\"N.m\">%r</max_torque>\n        </brakes>\n" % brake_front
            + "    </front-axle>\n    <rear-axle>\n"
            "        <track units=\"m\">%r</track>\n" % car["track_rear_m"]
            + "        <inertia units=\"kg.m2\">%r</inertia>\n" % car["wheel_inertia_kgm2"]
            + "        <smooth_throttle_coeff> 1.0e-5 </smooth_throttle_coeff>\n"
            "        <differential_stiffness units=\"N.m.s/rad\">%r</differential_stiffness>\n" % car["viscous_coupling_nms"]
            + "        <brakes>\n            <max_torque units=\"N.m\">%r</max_torque>\n        </brakes>\n" % brake_rear
            + "        <engine>\n            <maximum-power units=\"kW\">%r</maximum-power>\n        </engine>\n" % (car["max_drive_power_w"] / 1000)
            + "        <boost>\n            <maximum-power units=\"kW\"> 0.0 </maximum-power>\n        </boost>\n"
            "    </rear-axle>\n    <chassis>\n"
            "        <mass units=\"kg\">%r</mass>\n" % car["mass_kg"]
            + "        <inertia>\n            <Ixx> 0.0 </Ixx> <Ixy> 0.0 </Ixy> <Ixz> 0.0 </Ixz>\n"
            "            <Iyx> 0.0 </Iyx> <Iyy> 0.0 </Iyy> <Iyz> 0.0 </Iyz>\n"
            "            <Izx> 0.0 </Izx> <Izy> 0.0 </Izy> <Izz>%r</Izz>\n        </inertia>\n" % car["yaw_inertia_kgm2"]
            + "        <aerodynamics>\n            <rho units=\"kg/m3\"> 1.225 </rho>\n            <area units=\"m2\"> 1.0 </area>\n"
            "            <cd>%r</cd>\n            <cl>%r</cl>\n        </aerodynamics>\n" % (car["drag_area_m2"], car["downforce_area_m2"])
            + "        <com units=\"m\"><x> 0.0 </x><y> 0.0 </y><z>%r</z></com>\n" % (-height)
            + "        <front_axle units=\"m\"><x>%r</x><y> 0.0 </y><z>%r</z></front_axle>\n" % (front, -radius)
            + "        <rear_axle units=\"m\"><x>%r</x><y> 0.0 </y><z>%r</z></rear_axle>\n" % (-rear, -radius)
            + "        <pressure_center units=\"m\"><x>%r</x><y> 0.0 </y><z>%r</z></pressure_center>\n" % (pressure, -height)
            + "        <brake_bias>%r</brake_bias>\n" % bias
            + "        <roll_balance_coefficient>%r</roll_balance_coefficient>\n" % car["roll_balance_front"]
            + "        <Fz_max_ref2> 1.0 </Fz_max_ref2>\n    </chassis>\n"
            + tire(car["front_tire"], "front-tire") + tire(car["rear_tire"], "rear-tire") + "</vehicle>\n")
    mapping[:0] = [
        "model: fastest-lap's f1-3dof, a 3-DOF chassis (u, v, yaw rate) with four wheel speeds and algebraic wheel loads "
        "from force and moment balance with a roll-balance closure; this project's four-wheel plant has the same degrees of "
        "freedom and quasi-static loads",
        "geometry: centre of gravity %.4g m ahead of the rear axle and %.4g m up; axles %.4g m and %.4g m wide; wheel radius "
        "%.4g m, each wheel's inertia %.4g kg m^2" % (rear, height, car["track_front_m"], car["track_rear_m"], radius,
                                                      car["wheel_inertia_kgm2"]),
        "tires: fastest-lap's Pacejka_simple_model with this car's loads, frictions (times grip %.4g) and peak slips, the "
        "peak slips scaled so the peak sits where this project puts it, and Q from the sliding fractions, which makes "
        "each pure-slip curve identical; fastest-lap extrapolates peak slip beyond the reference loads where this project "
        "holds it" % grip,
        "aerodynamics: air at 1.225 kg/m^3 over 1 m^2, drag coefficient %.4g and lift coefficient %.4g, the car's drag and "
        "downforce areas; the pressure centre %.4g m %s the centre of gravity, at its height, puts %.4g of downforce "
        "on the front axle" % (car["drag_area_m2"], car["downforce_area_m2"], abs(pressure),
                               "ahead of" if pressure >= 0 else "behind", car["aero_balance_front"]),
        "powertrain: %.6g kW at the rear axle through a viscous coupling of %.4g N m s/rad; fastest-lap limits power "
        "alone, this car also %.4g N m per driven wheel; boost 0" % (car["max_drive_power_w"] / 1000,
                                                                       car["viscous_coupling_nms"], car["max_drive_torque_nm"]),
        "brakes: bias %.4g to the front; axle torque caps %.6g and %.6g N m, so the front wheels' share is this car's %.6g N m "
        "per wheel" % (bias, brake_front, brake_rear, car["max_brake_torque_nm"]),
        "controls: steering within this car's lock of %.4g rad and forward speed within its cap of %.4g m/s, as bounds of "
        "the optimal control problem; throttle and brake one control, as the project's acceleration command is"
        % (config["max_steering_rad"], config["max_speed_mps"]),
        "friction floor: fastest-lap floors peak friction at %.1f at every load, this project at the car's own %.4g and only "
        "beyond the reference loads" % (FRICTION_FLOOR, car["front_tire"]["minimum_friction"]),
        "not modelled by fastest-lap here: the slip speed floor, the kinematic blend below %.4g m/s and gravity's last "
        "digits (9.81 against 9.80665 m/s^2)" % car["dynamic_above_mps"],
    ]
    return text, mapping


SENSITIVITIES = [
    # parameter, unit, the step quoted, the finite-difference half step, fastest-lap's parameter path and alias (None
    # where fastest-lap's own sensitivity analysis does not reach it), its value from the problem's car, and
    # d(fastest-lap value)/d(parameter).
    dict(parameter="mass_kg", unit="kg", step=-10.0, half_step=5.0, path="vehicle/chassis/mass", alias="mass",
         value=lambda c: c["mass_kg"], scale=1.0),
    dict(parameter="cg_height_m", unit="m", step=-0.01, half_step=0.01, path="vehicle/chassis/com/z", alias="com_z",
         value=lambda c: -c["cg_height_m"], scale=-1.0),
    # fastest-lap carries the brake bias as a control held at the parameter's value, which its sensitivity analysis
    # does not differentiate; the bias is always a central difference of two re-solves.
    dict(parameter="brake_bias_front", unit="fraction", step=0.02, half_step=0.02, path=None, alias=None,
         value=lambda c: c["brake_bias_front"], scale=1.0),
    dict(parameter="downforce_area_m2", unit="m^2", step=0.1, half_step=0.1, path="vehicle/chassis/aerodynamics/cl",
         alias="cl", value=lambda c: c["downforce_area_m2"], scale=1.0),
    dict(parameter="max_drive_power_w", unit="W", step=10000.0, half_step=10000.0,
         path="vehicle/rear-axle/engine/maximum-power", alias="power", value=lambda c: c["max_drive_power_w"] / 1000,
         scale=1e-3),
]
ANALYTIC = "fastest-lap's sensitivity analysis at the solution"
DIFFERENCE = "central difference of two re-solves"


# ---------------------------------------------------------------- the child: fastest-lap itself

def child(work, fastest_lap):
    sys.path.insert(0, str(fastest_lap / "include"))
    import fastest_lap as fl  # noqa: E402  (the release's own wrapper)

    settings = json.loads((work / "settings.json").read_text(encoding="utf-8"))
    fl.create_vehicle_from_xml("car", str(work / "vehicle.xml"))
    fl.create_track_from_xml("track", str(work / "track.xml"))
    if settings["sensitivities"]:
        for item in SENSITIVITIES:
            if item["path"]:
                fl.vehicle_declare_new_constant_parameter("car", item["path"], item["alias"], settings["values"][item["alias"]])
    length = fl.track_download_length("track")
    mesh = settings["mesh"]
    s = [length * i / mesh for i in range(mesh)]
    options = ("<options>\n    <output_variables>\n        <prefix>run/</prefix>\n    </output_variables>\n"
               "    <print_level> 5 </print_level>\n    <max_iter> %d </max_iter>\n"
               "    <error_tolerance> %r </error_tolerance>\n" % (settings["max_iterations"], settings["tolerance"])
               + "    <variable_bounds>\n"
               "        <chassis.velocity.x>\n            <lower> 0.1 </lower>\n            <upper> %r </upper>\n"
               "        </chassis.velocity.x>\n" % settings["speed_cap"]
               + "        <front-axle.steering-angle>\n            <lower> %r </lower>\n            <upper> %r </upper>\n"
               "        </front-axle.steering-angle>\n" % (-settings["steering"], settings["steering"])
               + "    </variable_bounds>\n"
               + ("    <compute_sensitivity> true </compute_sensitivity>\n" if settings["sensitivities"] else "")
               + "</options>\n")
    (work / "options.xml").write_text(options, encoding="utf-8")
    sys.stdout.flush()
    prefix, names = fl.optimal_laptime("car", "track", s, options)
    sys.stdout.flush()
    result = {}
    for name in names:
        if fl.variable_type(prefix + name) == "vector":
            result[name] = fl.download_vector(prefix + name)
    for name in ["laptime"] + ["integral_quantities.tire-%s-energy" % w for w, _ in WHEELS]:
        if fl.variable_type(prefix + name) == "scalar":
            result[name] = fl.download_scalar(prefix + name)
    result["derivatives"] = {}
    for item in SENSITIVITIES:
        if settings["sensitivities"] and item["alias"]:
            name = prefix + "derivatives/laptime/" + item["alias"]
            if fl.variable_type(name) == "scalar":
                result["derivatives"][item["alias"]] = fl.download_scalar(name)
    (work / "result.json").write_text(json.dumps(result), encoding="utf-8")


# ---------------------------------------------------------------- the parent

def solver_exit(log):
    """Ipopt's own exit message, iteration count and unscaled objective, as it printed them."""
    status = re.findall(r"EXIT: (.+?)\s*$", log, re.M)
    iterations = re.findall(r"Number of Iterations\.*:\s*(\d+)", log)
    objective = re.findall(r"Objective\.*:\s*(\S+)\s+(\S+)", log)
    return (status[-1].rstrip(".") if status else "No exit reported", int(iterations[-1]) if iterations else 0,
            float(objective[-1][1]) if objective else float("nan"))


def solve(args, problem, track_text, work, name, sensitivities):
    """One solve in its own child process and work directory; returns fastest-lap's results and the solver's log."""
    directory = work / name
    directory.mkdir()
    (directory / "track.xml").write_text(track_text, encoding="utf-8")
    vehicle_text, _ = vehicle_xml(problem, args.accept_friction_floor)
    (directory / "vehicle.xml").write_text(vehicle_text, encoding="utf-8")
    config = problem["config"]
    settings = {"mesh": args.mesh, "tolerance": args.tolerance, "max_iterations": args.max_iterations,
                "speed_cap": config["max_speed_mps"], "steering": config["max_steering_rad"],
                "sensitivities": sensitivities,
                "values": {item["alias"]: item["value"](problem["car"]) for item in SENSITIVITIES if item["alias"]}}
    (directory / "settings.json").write_text(json.dumps(settings), encoding="utf-8")
    completed = subprocess.run([sys.executable, str(pathlib.Path(__file__).resolve()), "--fastest-lap", str(args.fastest_lap),
                                "--problem", str(args.problem), "--out", str(args.out), "--child", str(directory)],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    log = completed.stdout.decode("utf-8", "replace")
    if completed.returncode != 0 or not (directory / "result.json").exists():
        (args.out / ("solver-%s.log" % name)).write_text(log, encoding="utf-8")
        raise SystemExit("fastest-lap failed (exit %d); see %s" % (completed.returncode, args.out / ("solver-%s.log" % name)))
    return json.loads((directory / "result.json").read_text(encoding="utf-8")), log


def varied(problem, item, sign):
    changed = json.loads(json.dumps(problem))
    car = changed["car"]
    key = item["parameter"]
    car[key] = car[key] + sign * item["half_step"]
    return changed


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--fastest-lap", required=True, type=pathlib.Path,
                        help="the unpacked release: the directory holding bin and include")
    parser.add_argument("--release-zip", type=pathlib.Path, help="the downloaded release archive, checked against its pinned digest")
    parser.add_argument("--problem", required=True, type=pathlib.Path, help="a directory written by fd_headless --write-lap-problem")
    parser.add_argument("--out", required=True, type=pathlib.Path, help="a fresh directory for the artefact")
    parser.add_argument("--mesh", type=int, default=500, help="mesh points around the lap (default 500)")
    parser.add_argument("--tolerance", type=float, default=1e-8, help="Ipopt's error tolerance (default 1e-8)")
    parser.add_argument("--max-iterations", type=int, default=3000, help="Ipopt's iteration limit (default 3000)")
    parser.add_argument("--half-width", type=float, default=0.9,
                        help="the car's half width kept inside the corridor (default 0.9 m, the planners' own)")
    parser.add_argument("--no-sensitivities", action="store_true", help="skip the setup sensitivities")
    parser.add_argument("--accept-friction-floor", action="store_true",
                        help="solve a car whose peak friction falls below fastest-lap's floor of 1.0 anyway, stated in the artefact")
    parser.add_argument("--check-sensitivities", action="store_true",
                        help="also re-solve each parameter a half step either side and record the central difference beside "
                             "fastest-lap's own sensitivity, which the reader then requires to agree")
    parser.add_argument("--keep-work", type=pathlib.Path, help="also keep fastest-lap's raw results in this directory")
    parser.add_argument("--child", type=pathlib.Path, help=argparse.SUPPRESS)
    args = parser.parse_args()

    if args.child:
        child(args.child, args.fastest_lap)
        return

    library = args.fastest_lap / "bin" / PINNED["library"]
    if not library.exists():
        raise SystemExit("No %s under %s" % (PINNED["library"], args.fastest_lap))
    if sha256(library) != PINNED["library_sha256"]:
        raise SystemExit("%s is not the pinned fastest-lap %s library" % (library, PINNED["version"]))
    if args.release_zip and sha256(args.release_zip) != PINNED["release_sha256"]:
        raise SystemExit("%s is not the pinned fastest-lap %s release" % (args.release_zip, PINNED["version"]))
    if args.out.exists() and any(args.out.iterdir()):
        raise SystemExit("The artefact directory is not empty; choose a fresh --out")
    if not 10 <= args.mesh <= 5000:
        raise SystemExit("--mesh must lie in 10..5000")
    if args.no_sensitivities and args.check_sensitivities:
        raise SystemExit("--check-sensitivities checks the sensitivities --no-sensitivities skips; choose one")

    problem, corridor = read_problem(args.problem)
    track_text = track_xml(problem, corridor, args.half_width)
    _, mapping = vehicle_xml(problem, args.accept_friction_floor)
    mapping.append("track: the problem's corridor, %d samples over %.6g m, mirrored into fastest-lap's frame with its "
                   "curvature, and its edges less the car's half width of %.4g m, within which fastest-lap keeps the "
                   "centre of gravity" % (len(corridor["s_m"]), problem["corridor"]["length_m"], args.half_width))

    args.out.mkdir(parents=True, exist_ok=True)
    work = pathlib.Path(tempfile.mkdtemp(prefix="fd-optimal-lap-"))
    try:
        result, log = solve(args, problem, track_text, work, "optimum", not args.no_sensitivities)
        (args.out / "solver.log").write_text(log, encoding="utf-8")
        for name in ("track.xml", "vehicle.xml", "options.xml"):
            shutil.copyfile(work / "optimum" / name, args.out / name)
        status, iterations, objective = solver_exit(log)
        # Differences: the brake bias always, every other parameter when asked to check fastest-lap's own.
        differences = {}
        for item in SENSITIVITIES if not args.no_sensitivities else []:
            if item["alias"] and not args.check_sensitivities:
                continue
            times = []
            for sign, tag in ((1, "plus"), (-1, "minus")):
                name = "%s-%s" % (item["parameter"], tag)
                varied_result, varied_log = solve(args, varied(problem, item, sign), track_text, work, name, False)
                varied_status, _, _ = solver_exit(varied_log)
                (args.out / ("solver-%s.log" % name)).write_text(varied_log, encoding="utf-8")
                if varied_status not in ACCEPTED:
                    raise SystemExit("The re-solve with %s %s did not converge: %s" % (item["parameter"], tag, varied_status))
                times.append(varied_result["laptime"])
            differences[item["parameter"]] = (times[0] - times[1]) / (2 * item["half_step"])
        write_artefact(args, problem, result, mapping, status, iterations, objective, differences)
        for name in ("problem.json", "corridor.csv"):
            shutil.copyfile(args.problem / name, args.out / name)
        if args.keep_work:
            shutil.copytree(work, args.keep_work)
    finally:
        shutil.rmtree(work, ignore_errors=True)


ACCEPTED = ("Optimal Solution Found", "Solved To Acceptable Level")


def write_artefact(args, problem, result, mapping, status, iterations, objective, differences):
    config, car = problem["config"], problem["car"]
    s = result["road.arclength"]
    n = len(s)
    time = result["time"]
    lap_time = result.get("laptime")
    if lap_time is None:
        raise SystemExit("fastest-lap returned no lap time")
    header = ["point", "s_m", "time_s", "cog_x_m", "cog_y_m", "yaw_rad", "offset_m", "forward_mps", "lateral_mps",
              "yaw_rate_radps", "steering_rad", "throttle"]
    for w, _ in WHEELS:
        header += ["slip_ratio_" + w, "slip_angle_%s_rad" % w, "force_x_%s_n" % w, "force_y_%s_n" % w, "load_%s_n" % w,
                   "dissipation_%s_w" % w]
    lines = [",".join(header)]
    brake_torque = drive_torque = 0.0
    radius = car["wheel_radius_m"]
    for i in range(n):
        # fastest-lap's y, yaw, lateral velocity, yaw rate, steering and slip angle change sign in this frame; its
        # "kappa" is slip over peak slip, "true_kappa" the slip ratio; its vertical force and dissipation point down.
        row = [i, s[i], time[i], result["chassis.position.x"][i], -result["chassis.position.y"][i],
               -result["chassis.attitude.yaw"][i], -result["road.lateral-displacement"][i], result["chassis.velocity.x"][i],
               -result["chassis.velocity.y"][i], -result["chassis.omega.z"][i], -result["front-axle.steering-angle"][i],
               result["chassis.throttle"][i]]
        for w, name in WHEELS:
            fx = result[name + ".force.x"][i]
            row += [result[name + ".true_kappa"][i], -result[name + ".lambda"][i], fx, -result[name + ".force.y"][i],
                    -result[name + ".force.z"][i], -result[name + ".dissipation"][i]]
            if fx < 0:
                brake_torque = max(brake_torque, -fx * radius)
            elif name.startswith("rear"):
                drive_torque = max(drive_torque, fx * radius)
        lines.append(",".join(str(v) if isinstance(v, int) else repr(float(v)) for v in row))
    (args.out / "lap.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")
    mapping = list(mapping)
    mapping.append("checked: the largest braking torque at a wheel, from its tire's force, is %.0f N m against this car's "
                   "%.0f N m per wheel; the largest driving torque %.0f N m against %.0f N m, so neither cap binds"
                   % (brake_torque, car["max_brake_torque_nm"], drive_torque, car["max_drive_torque_nm"]))
    if brake_torque >= car["max_brake_torque_nm"] or drive_torque >= car["max_drive_torque_nm"]:
        mapping[-1] = mapping[-1].replace("so neither cap binds", "so a cap fastest-lap does not model is reached")
    sensitivities = []
    for item in SENSITIVITIES if not args.no_sensitivities else []:
        car_value = item["value"](car) / item["scale"]
        analytic = result["derivatives"].get(item["alias"]) if item["alias"] else None
        entry = {"parameter": item["parameter"], "unit": item["unit"], "value": car_value, "step": item["step"]}
        if analytic is not None:
            # d(lap)/d(parameter) = d(lap)/d(fastest-lap's value) times d(fastest-lap's value)/d(parameter)
            entry.update(seconds_per_unit=analytic * item["scale"], method=ANALYTIC,
                         cross_check_seconds_per_unit=differences.get(item["parameter"]))
        elif item["parameter"] in differences:
            entry.update(seconds_per_unit=differences[item["parameter"]], method=DIFFERENCE, cross_check_seconds_per_unit=None)
        else:
            raise SystemExit("fastest-lap returned no sensitivity for " + item["parameter"])
        entry["half_step"] = item["half_step"]
        sensitivities.append(entry)
    bias = differences.get("brake_bias_front")
    if bias is not None and abs(bias) < 1e-9:
        mapping.append("brakes, found: fastest-lap's solution does not change with the brake bias (central difference %.3g s "
                       "per unit over %.2g either side): its f1-3dof model does "
                       "not constrain this lap's braking by the bias, so the zero sensitivity is the model's, not the car's"
                       % (bias, SENSITIVITIES[2]["half_step"]))
    # fastest-lap integrates each tire's dissipation in megajoules.
    energy = [result["integral_quantities.tire-%s-energy" % w] * 1e6 for w, _ in WHEELS]
    fingerprint = fnv1a((args.problem / "problem.json").read_bytes().replace(b"\r\n", b"\n").decode("utf-8"))
    document = {
        "schema_version": 1,
        "kind": "reference-optimal",
        "label": LABEL if not friction_below_floor(problem) else LABEL_FLOORED,
        "solver": dict(tool=PINNED["tool"], version=PINNED["version"], release=PINNED["release"],
                       release_sha256=PINNED["release_sha256"], library_sha256=PINNED["library_sha256"],
                       model=PINNED["model"], nlp_solver=PINNED["nlp_solver"], status=status, converged=status in ACCEPTED,
                       iterations=iterations, max_iterations=args.max_iterations, tolerance=args.tolerance),
        "mesh_points": n,
        "lap_time_s": lap_time,
        # The objective Ipopt minimised: the lap time and fastest-lap's small penalties on how fast the controls change.
        "objective": objective if math.isfinite(objective) else None,
        "speed_cap_mps": config["max_speed_mps"],
        "vehicle_half_width_m": args.half_width,
        "problem_fingerprint": fingerprint,
        "mapping": mapping,
        "tire_energy_j": energy,
        "sensitivities": sensitivities,
    }
    (args.out / "optimal_lap.json").write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
    print("%s: %s after %d iterations; lap %.3f s over %d points" % (PINNED["tool"], status, iterations, lap_time, n))
    for entry in sensitivities:
        check = entry.get("cross_check_seconds_per_unit")
        print("  %s: %.6g s per %s (%s)%s" % (entry["parameter"], entry["seconds_per_unit"], entry["unit"], entry["method"],
                                              "" if check is None else "; central difference %.6g" % check))


def fnv1a(text):
    """The project's fingerprint (fingerprint_hex in core/src/model_identity.cpp): 64-bit FNV-1a as 16 hexadecimal digits."""
    h = 0xcbf29ce484222325
    for byte in text.encode("utf-8"):
        h ^= byte
        h = (h * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
    return "%016x" % h


if __name__ == "__main__":
    main()
