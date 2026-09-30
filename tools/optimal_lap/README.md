# The theoretical best lap

TrackWayFastPlan Phase 4, decision 0034. An offline minimum-time solver, fastest-lap, answers "what is the fastest this
car could drive this corridor" for its own 3-DOF model; TUM's independent formulation cross-checks it; the desktop shows
the answer beside the live car. Nothing here is linked into the build or drives the car.

## What is needed, and where it lives

Both tools were downloaded once, with the user's approval (2026-09-25), into `.tools/`, which Git ignores. Neither is
redistributed here.

- **fastest-lap 0.5**, the prebuilt Windows release `fastest-lap-w10-x64-msvc2022-v0.5.zip` from the project's GitHub
  releases (Juan Manzanero, MIT), unpacked to `.tools/fastest-lap/v0.5/v0.5`. The release publishes no digest; the ones
  computed here on download are pinned in `solve_fastest_lap.py` and checked on every run:
  archive `81947ae517c7029278ed7cef0c06e74a2664cec3130a573d66647483bd08831c`, library `libfastestlapc-0.5.dll`
  `28a22902b2bf8cd0b4c930033f092ea34dbd7ec69c29d1de5cc6678a85a8533c`. It carries Ipopt 3.14.10 with MUMPS 5.5.1 and
  Intel runtime libraries, each under its own licence (see `THIRD_PARTY_NOTICES.md`).
- **Python 3.8.20** (installed with uv into `.tools/python`) and a virtual environment `.tools/tum-python` with TUM's pinned
  requirements: numpy 1.18.1, scipy 1.3.3, casadi 3.5.1, matplotlib 3.3.1, scikit-learn 0.23.1 and
  trajectory_planning_helpers 0.76 (installed without its dependencies, beside quadprog 0.1.12 because the pinned 0.1.7
  does not build here). fastest-lap's own Python wrapper runs in the same environment.
- **TUM's global_racetrajectory_optimization** (LGPL-3.0) stays where it was downloaded; `cross_check_tum.py` imports it
  from there and writes nothing into it.

## Commands

From the repository root, with the core build in place:

```bash
build/core/apps/headless/Release/fd_headless.exe --profile formula-one-style --write-lap-problem out/optimal/f1-problem
```

```bash
.tools/tum-python/Scripts/python.exe tools/optimal_lap/solve_fastest_lap.py --fastest-lap .tools/fastest-lap/v0.5/v0.5 --release-zip .tools/fastest-lap/fastest-lap-w10-x64-msvc2022-v0.5.zip --problem out/optimal/f1-problem --out out/optimal/f1-optimum --check-sensitivities
```

```bash
build/core/apps/headless/Release/fd_headless.exe --profile formula-one-style --check-optimal-lap out/optimal/f1-optimum
```

```bash
./scripts/run.ps1 --profile formula-one-style --optimal-lap out/optimal/f1-optimum
```

The problem is whatever the flags pose: `--profile ID` or `--plant four-wheel` with its setup flags, on the preset, a
`--track` file or a `--course`. The check requires the same flags, so an artefact can only be read as the optimum of the
problem it was solved for. `--mesh` (default 500 points), `--tolerance` (Ipopt's, default 1e-8) and `--max-iterations`
(default 3000) are recorded in the artefact; `--half-width` (default 0.9 m, the planners' own) is kept inside the corridor.

`--check-sensitivities` re-solves every parameter a half step either side and records the central difference beside
fastest-lap's own sensitivity; the reader then requires the two to agree within 5% (or a millisecond over the quoted
step). The brake bias is always a central difference, because fastest-lap's sensitivity analysis does not reach it.

The cross-check, for the electric-race-car profile (TUM's own example car):

```bash
.tools/tum-python/Scripts/python.exe tools/optimal_lap/cross_check_tum.py --tum PATH_TO_TUM --optimal out/optimal/erc-optimum --out out/optimal/erc-tum
```

`friction_floor.py --fastest-lap RELEASE --problem PROBLEM` shows fastest-lap's floor on peak friction with its own G-G
diagram.

## What the artefact holds

`optimal_lap.json`: the solver (tool, version, release, both digests, model `f1-3dof`, Ipopt), its own exit status,
iterations, iteration limit and tolerance, whether that status is a solution Ipopt accepts, the mesh, the lap time, the
objective Ipopt minimised (the lap time and fastest-lap's penalties on how fast controls change), the speed cap kept, the
half width, the problem's fingerprint, every statement of how the car was put into fastest-lap's model, each tire's
dissipated energy and the setup sensitivities. `lap.csv`: every mesh point in this project's frame (map x/y, z up, yaw
counterclockwise; fastest-lap's frame has z down, so y, yaw, lateral velocity, yaw rate, steering and slip angle change
sign), the centre of gravity's position, speed, controls and each tire's slip ratio, slip angle, forces, load and
dissipated power. `problem.json` and `corridor.csv` are the problem as written. `solver.log` and `solver-*.log` are
Ipopt's own output; `track.xml`, `vehicle.xml` and `options.xml` are what fastest-lap was given.

## Two findings

- **fastest-lap floors every tire's peak friction at 1.0.** A car whose friction is 0.5, 0.8 or 1.0 reaches the same
  lateral limit in fastest-lap's G-G diagram; 1.2 and 1.5 scale it. No car here with friction above 1 is affected (the
  Formula One style and Formula Student cars); TUM's car, at 0.80 to 0.93, is. The solver refuses such a car unless
  `--accept-friction-floor` is given, and then says so in the label and the mapping.
- **fastest-lap's lap time does not depend on the brake bias** on this problem (the same lap to machine precision at
  bias 0.3 and 0.6): its f1-3dof braking is not constrained by it, so that sensitivity is the model's zero.
