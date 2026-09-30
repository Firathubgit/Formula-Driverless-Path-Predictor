"""Cross-check a fastest-lap optimum with TUM's independent minimum-time formulation (TrackWayFastPlan Phase 4.2).

    python cross_check_tum.py --tum TUM_DIR --optimal ARTEFACT_DIR --out DIR

ARTEFACT_DIR is a reference-optimal artefact written by solve_fastest_lap.py for the electric-race-car profile, whose
car is TUM's own example car (params/racecar.ini, decision 0033). This script gives the same car, in TUM's double-track
parameter set, and the same corridor to TUM's opt_mintime, run in place from TUM_DIR with TUM's own track preparation,
and compares the two optima: lap time, line and speed along the corridor. TUM's car grips less than fastest-lap's
floor on peak friction (1.0, see friction_floor.py), so TUM is solved twice: as published, and at that floor, which is
the car fastest-lap solved and so the like-for-like comparison. Each run writes tum_lap_NAME.csv (TUM's line and speed
in this project's frame beside fastest-lap's at the same stations), tum_NAME.log and TUM's own export; cross_check.json
holds the numbers and how the two formulations differ. The directory must be fresh.

TUM's code is LGPL-3.0 and stays where it was downloaded: it is imported from TUM_DIR, never copied into this
repository, and nothing is written into TUM_DIR. It needs TUM's pinned Python environment (Python 3.8, casadi 3.5.1,
numpy 1.18.1, trajectory_planning_helpers 0.76); see README.md beside this script.
"""

import argparse
import configparser
import copy
import csv
import io
import json
import math
import pathlib
import sys
import time
from contextlib import redirect_stdout

import numpy as np

import solve_fastest_lap as solver

AIR_DENSITY = 1.225  # this project's (core/vehicle.hpp)


def read_csv(path):
    with open(path, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def tum_parameters(tum, problem, half_width):
    """TUM's racecar.ini with the problem's car written into it, member by member; TUM's tire parameters stay its own,
    because the profile's tires are a fit of them."""
    car, config = problem["car"], problem["config"]
    if problem["profile"] != "electric-race-car":
        raise SystemExit("The cross-check is defined for the electric-race-car profile, TUM's own example car, whose "
                         "tires were fitted from TUM's; this artefact's car is " + (problem["profile"] or "the configured one"))
    parser = configparser.ConfigParser()
    if not parser.read(str(tum / "params" / "racecar.ini")):
        raise SystemExit("No params/racecar.ini under " + str(tum))

    def section(group, name):
        return json.loads(parser.get(group, name))

    pars = {name: section("GENERAL_OPTIONS", name) for name in ("stepsize_opts", "reg_smooth_opts", "veh_params",
                                                                 "vel_calc_opts", "curv_calc_opts")}
    pars["optim_opts"] = section("OPTIMIZATION_OPTIONS", "optim_opts_mintime")
    pars["vehicle_params_mintime"] = section("OPTIMIZATION_OPTIONS", "vehicle_params_mintime")
    pars["tire_params_mintime"] = section("OPTIMIZATION_OPTIONS", "tire_params_mintime")
    pars["pwr_params_mintime"] = section("OPTIMIZATION_OPTIONS", "pwr_params_mintime")
    veh, mintime, optim = pars["veh_params"], pars["vehicle_params_mintime"], pars["optim_opts"]
    wheelbase = config["wheelbase_m"]
    q = 0.5 * AIR_DENSITY
    driven = 2 if car["drive_front_fraction"] in (0.0, 1.0) else 4
    stated = {
        "veh_params.mass": (veh, "mass", car["mass_kg"]),
        "veh_params.dragcoeff": (veh, "dragcoeff", q * car["drag_area_m2"]),
        "veh_params.v_max": (veh, "v_max", config["max_speed_mps"]),
        "veh_params.width": (veh, "width", 2 * half_width),
        "wheelbase_front": (mintime, "wheelbase_front", wheelbase - car["cg_to_rear_m"]),
        "wheelbase_rear": (mintime, "wheelbase_rear", car["cg_to_rear_m"]),
        "track_width_front": (mintime, "track_width_front", car["track_front_m"]),
        "track_width_rear": (mintime, "track_width_rear", car["track_rear_m"]),
        "cog_z": (mintime, "cog_z", car["cg_height_m"]),
        "I_z": (mintime, "I_z", car["yaw_inertia_kgm2"]),
        "liftcoeff_front": (mintime, "liftcoeff_front", q * car["downforce_area_m2"] * car["aero_balance_front"]),
        "liftcoeff_rear": (mintime, "liftcoeff_rear", q * car["downforce_area_m2"] * (1 - car["aero_balance_front"])),
        "k_brake_front": (mintime, "k_brake_front", car["brake_bias_front"]),
        "k_drive_front": (mintime, "k_drive_front", car["drive_front_fraction"]),
        "k_roll": (mintime, "k_roll", car["roll_balance_front"]),
        "power_max": (mintime, "power_max", car["max_drive_power_w"]),
        "f_drive_max": (mintime, "f_drive_max", driven * car["max_drive_torque_nm"] / car["wheel_radius_m"]),
        "f_brake_max": (mintime, "f_brake_max", 4 * car["max_brake_torque_nm"] / car["wheel_radius_m"]),
        "delta_max": (mintime, "delta_max", config["max_steering_rad"]),
        "width_opt": (optim, "width_opt", 2 * half_width),
    }
    changes = []
    for label, (target, key, value) in stated.items():
        if abs(target[key] - value) > 1e-9 * max(1.0, abs(value)):
            changes.append("%s %g -> %g" % (label, target[key], value))
        target[key] = value
    mintime["wheelbase"] = mintime["wheelbase_front"] + mintime["wheelbase_rear"]
    optim.update(var_friction=None, warm_start=False, safe_traj=False, limit_energy=False)
    pars["pwr_params_mintime"]["pwr_behavior"] = False
    return pars, changes


def project(points, cx, cy, s, length):
    """Each point's station on the corridor: the nearest point of the closed polyline through its samples."""
    stations = []
    n = len(cx)
    for px, py in points:
        best, where = float("inf"), 0.0
        d = (cx - px) ** 2 + (cy - py) ** 2
        i = int(np.argmin(d))
        for a in ((i - 1) % n, i):
            b = (a + 1) % n
            ex, ey = cx[b] - cx[a], cy[b] - cy[a]
            seg = ex * ex + ey * ey
            f = min(1.0, max(0.0, ((px - cx[a]) * ex + (py - cy[a]) * ey) / seg))
            qx, qy = cx[a] + f * ex, cy[a] + f * ey
            dist = (qx - px) ** 2 + (qy - py) ** 2
            if dist < best:
                span = (s[b] if b else length) - s[a]
                best, where = dist, (s[a] + f * span) % length
        stations.append(where)
    return np.array(stations)


def periodic_interp(x, xp, fp, period):
    order = np.argsort(xp)
    xp, fp = np.asarray(xp)[order], np.asarray(fp)[order]
    return np.interp(x, np.concatenate((xp - period, xp, xp + period)), np.concatenate((fp, fp, fp)))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--tum", required=True, type=pathlib.Path, help="TUM's global_racetrajectory_optimization directory")
    parser.add_argument("--optimal", required=True, type=pathlib.Path, help="fastest-lap's reference-optimal artefact")
    parser.add_argument("--out", required=True, type=pathlib.Path, help="a fresh directory for the comparison")
    args = parser.parse_args()
    if args.out.exists() and any(args.out.iterdir()):
        raise SystemExit("The comparison directory is not empty; choose a fresh --out")

    optimum = json.loads((args.optimal / "optimal_lap.json").read_text(encoding="utf-8"))
    problem = json.loads((args.optimal / "problem.json").read_text(encoding="utf-8"))
    corridor = read_csv(args.optimal / "corridor.csv")
    lap = read_csv(args.optimal / "lap.csv")
    half_width = optimum["vehicle_half_width_m"]
    pars, changes = tum_parameters(args.tum, problem, half_width)

    sys.path.insert(0, str(args.tum))
    import opt_mintime_traj  # noqa: E402  (TUM's, from where it lies)
    import helper_funcs_glob  # noqa: E402

    x = np.array([float(r["x_m"]) for r in corridor])
    y = np.array([float(r["y_m"]) for r in corridor])
    s = np.array([float(r["s_m"]) for r in corridor])
    left = np.array([float(r["left_m"]) for r in corridor])
    right = np.array([float(r["right_m"]) for r in corridor])
    length = problem["corridor"]["length_m"]
    reftrack = np.column_stack((x, y, right, left))

    args.out.mkdir(parents=True, exist_ok=True)
    fl_speed = np.array([math.hypot(float(r["forward_mps"]), float(r["lateral_mps"])) for r in lap])
    fl_offset = np.array([float(r["offset_m"]) for r in lap])
    fl_station = np.array([float(r["s_m"]) for r in lap])

    def solve(name, parameters):
        """One TUM solve in its own export directory, compared with fastest-lap along the corridor."""
        export = args.out / ("tum_export_" + name)
        export.mkdir()
        log = io.StringIO()
        began = time.perf_counter()
        with redirect_stdout(log):
            reftrack_interp, normvec_interp, _, coeffs_x, coeffs_y = helper_funcs_glob.src.prep_track.prep_track(
                reftrack_imp=reftrack, reg_smooth_opts=parameters["reg_smooth_opts"],
                stepsize_opts=parameters["stepsize_opts"], debug=True, min_width=None)
            alpha, v, reftrack_opt, _, normvec_opt = opt_mintime_traj.src.opt_mintime.opt_mintime(
                reftrack=reftrack_interp, coeffs_x=coeffs_x, coeffs_y=coeffs_y, normvectors=normvec_interp,
                pars=parameters, tpamap_path="", tpadata_path="", export_path=str(export), print_debug=True, plot_debug=False)
        seconds = time.perf_counter() - began
        text = log.getvalue()
        (args.out / ("tum_%s.log" % name)).write_text(text, encoding="utf-8")
        status = [line.split("EXIT:", 1)[1].strip().rstrip(".") for line in text.splitlines() if "EXIT:" in line]
        status = status[-1] if status else "No exit reported"
        states = np.loadtxt(export / "states.csv", delimiter=";")
        lap_time = float(states[-1, 1])
        line = reftrack_opt[:, :2] + np.expand_dims(alpha, 1) * normvec_opt
        points = list(map(tuple, line))
        speed = np.asarray(v)
        station = project(points, x, y, s, length)
        # TUM's line and speed along the corridor, against fastest-lap's at the same stations.
        offset = []
        for (px, py), st in zip(points, station):
            i = int(np.searchsorted(s, st, side="right") - 1) % len(s)
            nx, ny = float(corridor[i]["left_normal_x"]), float(corridor[i]["left_normal_y"])
            cxs = np.interp(st, np.append(s, length), np.append(x, x[0]))
            cys = np.interp(st, np.append(s, length), np.append(y, y[0]))
            offset.append((px - cxs) * nx + (py - cys) * ny)
        offset = np.array(offset)
        fl_offset_at = periodic_interp(station, fl_station, fl_offset, length)
        fl_speed_at = periodic_interp(station, fl_station, fl_speed, length)
        line_gap = np.abs(offset - fl_offset_at)
        speed_gap = speed - fl_speed_at
        with open(args.out / ("tum_lap_%s.csv" % name), "w", encoding="utf-8", newline="") as f:
            w = csv.writer(f)
            w.writerow(["point", "station_m", "x_m", "y_m", "offset_m", "speed_mps", "fastest_lap_offset_m",
                        "fastest_lap_speed_mps"])
            for k, ((px, py), st) in enumerate(zip(points, station)):
                w.writerow([k, repr(float(st)), repr(float(px)), repr(float(py)), repr(float(offset[k])),
                            repr(float(speed[k])), repr(float(fl_offset_at[k])), repr(float(fl_speed_at[k]))])
        tire = parameters["tire_params_mintime"]
        return {"lap_time_s": lap_time, "status": status, "mesh_points": int(len(points)),
                "stepsize_m": parameters["stepsize_opts"]["stepsize_reg"], "seconds": seconds,
                "friction": {"mue": parameters["optim_opts"]["mue"], "eps_front": tire["eps_front"], "eps_rear": tire["eps_rear"],
                             "f_z0": tire["f_z0"]},
                "top_speed_mps": float(speed.max()), "lowest_speed_mps": float(speed.min()),
                "lap_time_difference_s": lap_time - optimum["lap_time_s"],
                "line": {"median_gap_m": float(np.median(line_gap)), "p95_gap_m": float(np.percentile(line_gap, 95)),
                         "worst_gap_m": float(line_gap.max())},
                "speed": {"median_difference_mps": float(np.median(speed_gap)),
                          "p95_abs_difference_mps": float(np.percentile(np.abs(speed_gap), 95)),
                          "worst_abs_difference_mps": float(np.abs(speed_gap).max())}}

    runs = {"as_published": solve("as_published", pars)}
    below = solver.friction_below_floor(problem)
    if below:
        # fastest-lap solved this car with every peak friction raised to its floor (decision 0034). TUM's car is below
        # it at every load, so at the floor it is TUM's friction circle of that value with no load sensitivity.
        floored = copy.deepcopy(pars)
        floored["optim_opts"]["mue"] = solver.FRICTION_FLOOR
        floored["tire_params_mintime"]["eps_front"] = floored["tire_params_mintime"]["eps_rear"] = 0.0
        runs["at_fastest_lap_floor"] = solve("at_fastest_lap_floor", floored)

    summary = {
        "schema_version": 1,
        "kind": "minimum-time cross-check",
        "problem_fingerprint": optimum["problem_fingerprint"],
        "fastest_lap": {"lap_time_s": optimum["lap_time_s"], "status": optimum["solver"]["status"],
                        "mesh_points": optimum["mesh_points"], "tolerance": optimum["solver"]["tolerance"],
                        "model": optimum["solver"]["model"], "label": optimum["label"],
                        "top_speed_mps": float(fl_speed.max()), "lowest_speed_mps": float(fl_speed.min())},
        "tum_car_changes_from_racecar_ini": changes,
        "tum": runs,
        "fastest_lap_friction_floor": ("fastest-lap floors peak friction at %.1f and this car's falls below it (%s): "
                                       "tum.at_fastest_lap_floor solves TUM's formulation at that floor, the like-for-like "
                                       "comparison" % (solver.FRICTION_FLOOR, "; ".join(below))) if below else None,
        "differences_in_formulation": [
            "tires: TUM's lateral force is a Magic Formula of slip angle with load-dependent peak, and each wheel's "
            "longitudinal force is a control bounded by a friction circle; fastest-lap's forces come from slip ratio and "
            "slip angle through one normalised combined slip, and this profile's tires are a fit of TUM's",
            "wheels: fastest-lap integrates four wheel speeds and drives through them; TUM has no wheel dynamics and "
            "splits drive and brake force between the axles by fixed shares",
            "load transfer: both are quasi-steady with a roll-moment split, TUM's from the accelerations and fastest-lap's "
            "from force and moment balance with its roll-balance closure",
            "actuators: TUM delays steering by %.3g s and drive and brake by %.3g and %.3g s; fastest-lap penalises how "
            "fast its controls change" % (pars["vehicle_params_mintime"]["t_delta"], pars["vehicle_params_mintime"]["t_drive"],
                                           pars["vehicle_params_mintime"]["t_brake"]),
            "resistance: TUM adds rolling resistance (c_roll %.3g) and fastest-lap does not"
            % pars["tire_params_mintime"]["c_roll"],
            "track: TUM smooths the corridor again by its own spline regression and solves on %.3g m steps; fastest-lap "
            "takes the corridor as conditioned here, on its own mesh" % pars["stepsize_opts"]["stepsize_reg"],
        ],
    }
    (args.out / "cross_check.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print("fastest-lap %.3f s (%s)" % (optimum["lap_time_s"], optimum["solver"]["status"]))
    for name, run in runs.items():
        print("TUM %s: %.3f s (%s), %+.3f s; line apart by median %.2f m, 95th percentile %.2f m; speed apart by median "
              "%+.2f m/s, 95th percentile %.2f m/s" % (name.replace("_", " "), run["lap_time_s"], run["status"],
                                                       run["lap_time_difference_s"], run["line"]["median_gap_m"],
                                                       run["line"]["p95_gap_m"], run["speed"]["median_difference_mps"],
                                                       run["speed"]["p95_abs_difference_mps"]))


if __name__ == "__main__":
    main()
