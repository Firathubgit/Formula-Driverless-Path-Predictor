# Formula-Driverless-Path-Predictor

Product: make autonomous racing decisions visible. Read `docs/PROJECT_BRIEF.md` and
`docs/STATUS.md` before choosing work. Work through the active milestone; the roadmap
is context, not an instruction to build everything in one session.

## Engineering invariants

- C++20 algorithms use ordinary data and have no Qt, ROS, renderer, or hardware imports.
- One simulation plant advances the vehicle. Display its actual pose; never snap to a path.
- SI internally, right-handed map x/y with z up, yaw counterclockwise, rear-axle reference.
- A track's arc length between samples is never shorter than its chord; projection relies on it.
- Fixed simulation ticks schedule control; render timing must not alter the integrated model.
- Green/yellow/red mean planned acceleration/constant speed/braking, with a documented deadband.
- The kinematic baseline has no mass sensitivity; the dynamic plants do, through their equations. Expose only
  implemented, effective controls: every exposed vehicle parameter has a test proving its effect.
- Every plant moves only through the vehicle seam (`core/vehicle.hpp`); the kinematic adapter is exact.
- Validate changes before mutation, apply at tick boundaries, record revision and simulation time.
- A change at tick T governs motion after T: the sample at T keeps the old revision.
- A recording is one cross-checked contract. Replay seeks a cursor and never advances the plant.
- Blocked regions are scenario ground truth. Nothing here detects, classifies or tracks anything.
- The car's own instruments read its own state, late and noisy, from a recorded seed. On the known track what they
  measure drives nothing; driving on cones the driver decides from it. The plant always advances on its true state, and a
  run records both.
- The course's cones are ground truth laid along the corridor. Perception samples them into simulated detections, labelled
  as such, whose frames never say which cone each was; that truth is the evaluation's alone.
- Driving on cones, the driver is given the pose the car was placed at, its readings (measured, or the true state as the
  stated ideal-state assumption) and its perception frames, never the track, the cones or a frame's truth. It follows the
  path it believes, planned to be at rest where that path ends; the known track only judges the run. Every recording of
  such a run is checked by a driver run again on its readings and frames alone. The driver remembers the cones it placed,
  not a map estimated with its pose.
- Every run is judged on its course, after PacSim's competition logic: laps from the start line, cones down, off course, an
  unsafe stop, a DNF. The judge reads the true pose and the course's cones and is the evaluation's alone; nothing that
  drives reads it, and every recording is judged again when loaded.
- Around a stated blockage the lattice planner takes, of each action's cheapest path through the offline lattice (layers
  every 6 m, 4 m in curves; nodes every 0.5 m) within 60 m or just past the blockage, the clear one of least estimated
  time to the farthest such path's end, each timed by a speed profile along its own path from the actual speed within the
  grip fractions or the envelope share; it does not search for the fastest path, and without a blockage in the way it
  follows the reference plan. The five-offset planner selects among feasible offsets. The racing line is a reference
  solved before a run for minimum curvature, not minimum time.
- The planner chooses the action; the policy or a predictive controller drives it. On cones the one action is to follow
  the believed path, and the policy drives it. MPCC's plan drives only while it is
  solved, inside its own envelope and the corridor, and clear of every blockage by the planners' own check, and never a
  hold; otherwise the policy drives, and every decision records who drove it and why. A plan that drives states its
  command at each point and drives the first; its model error is measured against the plant's response to those
  commands, and its prediction model is a documented reduction of the plant, the dynamic plant's own equations.
- A person may take the car (decision 0037): their controls, sampled at control decisions, replace the command through the
  vehicle seam and nothing else; the planner still decides, shown and not driven, each decision says a person drove it,
  and the same judge judges. Nothing caps their speed but the plant unless they explicitly enable the kart's
  Forgiveness override (decision 0040): Overall 1/off preserve ordinary physics, 100 gives deliberately no-slip arcade handling,
  through that same vehicle seam, with changes at tick boundaries. Manual mode independently adjusts acceleration,
  braking, steering, grip assistance and speed (decision 0042). A person's drive is never recorded.
- `fd_core` has no external dependency; code ported into it keeps its licence notice in `THIRD_PARTY_NOTICES.md`.
  Solvers live in their own targets, built from pinned, checksum-verified sources.
- Known track and ideal state are explicit assumptions. Replay is not closed-loop policy evaluation.
- The theoretical best lap is an offline solver's optimum of a stated lap problem, for its own model at the mesh, tolerance
  and exit status stated with it, read as one checked contract. It never drives and is never a second plant; it is shown
  only while its problem is the car and corridor on screen, and an unconverged or friction-floored solve says so.
- Tests exercise physics, constraints, lifecycle and shared contracts, including failure cases.
- Keep build outputs, downloaded tools, private references and generated media out of Git.
- Report only builds, tests, renders and integrations that actually ran. Update status with evidence.
- No external outreach or publication without the user's instruction. Routine local work may proceed.

## Commands

Windows: `./scripts/build.ps1` (core) or `./scripts/build.ps1 -Desktop` (Qt + UI checks).
Run native UI: `./scripts/run.ps1`. First-time Qt setup: `./scripts/bootstrap-qt.ps1`.
Put the showroom's cars on the track with `python tools/showroom/build_live_cars.py` (Blender 4.5, decision 0036); the
desktop reads `artifacts/live-cars/`, or `--live-cars DIR`, and draws the procedural cars without them.
Drive the car yourself with an Xbox controller from the desktop's Settings (Driver: You, decision 0037): right trigger
accelerates, left trigger brakes, left stick steers, Menu starts and pauses. Under You, Gokart (or `fd_desktop --kart`) drives
Gokartcentralen Göteborg (`tracks/gokartcentralen-goteborg.csv`, traced from its poster by `tools/tracks/trace_poster.py`)
with the `gokartcentralen-rsx2` rental kart profile (decision 0038).
Headless CLI: `build/core/apps/headless/Release/fd_headless.exe --help`.
Each recorded experiment requires a fresh `--out` directory. See GUIDE.md for comparisons.
Validate a run with `--check-recording DIR`; replay one with `./scripts/run.ps1 --replay DIR`.
Derive the four-wheel car's G-G-V envelope, without a lap, with `--plant four-wheel --write-envelope DIR`.
Plan its speed from that envelope with `--plant four-wheel --speed-plan envelope [--envelope-fraction 0.5..1]`, or the
desktop's Settings (Speed plan); the grip fractions stay the default plan for every model.
Drive a closed centreline file instead of the preset with `--track FILE [--track-width M] [--track-smoothing M]`.
Drive the minimum-curvature racing line of either with `--line racing [--line-margin M]`, or the desktop's Settings (Line); it
needs OSQP's sources, fetched once with `./scripts/bootstrap-osqp.ps1` (without them the rest still builds).
Write the offline lattice along the line a run would drive, without a lap, with `--write-lattice DIR` (with `--track` or
`--line racing` as for a run); the desktop draws it ahead of the car. Domain terms are in `CONTEXT.md`.
State a blocked region with `--obstruct FROM_S:TO_S:FROM_OFFSET:TO_OFFSET[:NAME]` in either app. The lattice planner
passes it along the lattice by default; `--local-planner five-offsets`, or the desktop's Settings (Planner), restores the
five-offset planner for comparison. Fit the car with its own instruments with `--sensors SEED`, or the desktop's
Settings (Sensors, Instruments): pose and speed at 20 Hz behind 50 ms, wheel speeds, inertial unit and steering at
200 Hz behind 5 ms, each with its own Gaussian error drawn from SEED (decision 0029). `--perception SEED`, or the same
section's Cone perception, lays the course's cones and gives the car simulated cone perception of them after PacSim's model (decision
0030). `--drive cones`, or its Drive on row, drives on the path the car believes from those detections instead of the
known track (decision 0031): the port of FaSTTUBe's cone path planner, checked against the published package on
`tests/fixtures/cone_path` and cross-checked by a Delaunay planner written from FS-FEUP's description alone. It needs
`--perception`; the preset needs `--track-width 5` (the desktop lays it to 5 m itself). Every run is judged
(decision 0032); `--discipline trackdrive|autocross` sets the rules (ten laps or one). `--course FILE`, in either app,
drives a Formula Student layout in PacSim's format from where it lies, never copied here; its hairpins need
`--config configs/formula-student.cfg`, a Formula Student car's wheelbase, lock and aim. `--profile ID` builds the
four-wheel car from a documented physics profile, every value marked published, derived, fitted or chosen (decision
0033); appearance is not part of it, and the showroom is not touched by it (the desktop takes `--profile ID` at start).
Write a car's minimum-time problem with `--write-lap-problem DIR` (with `--profile` or `--plant four-wheel`), solve it with
`tools/optimal_lap/solve_fastest_lap.py` (fastest-lap 0.5 and Python 3.8 in `.tools/`, decision 0034; see its README),
check the artefact with `--check-optimal-lap DIR` and the same flags, cross-check it with TUM's formulation with
`tools/optimal_lap/cross_check_tum.py`, and show it in the desktop with `--profile ID --optimal-lap DIR`. Recordings are schema 18 (a car's drive controller top speed, decision 0038): what
each channel delivered at every tick in `measurements.csv`, the cones and every perception frame in `cones.csv`,
`perception_frames.csv`, `detections.csv` and `missed.csv`, what the driver believed at each decision on cones in
`beliefs.csv` and every path it believed in `believed_paths.csv`, everything the judge saw in `timing.csv`, and the
instruments, the perception, cone driving, the rules and their seeds in metadata; each
action's lattice path is in `paths.csv`, its speed profile in `profiles.csv`, who drove each decision in
`decisions.csv`, and the planned commands of every plan MPCC drove in `commands.csv`; a run with alternatives prints its
planning time. `--controller mpcc`, or the desktop's Settings (Control), drives the chosen action with model predictive
contouring control (needs OSQP's sources, as the racing line does); the run prints MPCC's solve times, and
`--check-recording` its plan error and model error ahead.
`--prediction-error DIR` reports both every 0.25 s over the horizon for an MPCC run, the flying laps alone, and the
plans its prediction model got most wrong. While such a plan drives the four-wheel car, live or replayed, the desktop's
tire card in its telemetry draws what each wheel has left of its friction circle along that plan (decision 0027).
Select the plant with `--plant kinematic|dynamic|four-wheel` (headless) or the desktop's Settings (Vehicle model).
`--steering pure-pursuit|map` chooses the steering law for either car with tires, MAP from a steady-state table
generated for that car (decision 0026); for the dynamic single-track plant `--car default|soft-front` chooses the
understeering setup; `--brake-bias-front`, `--cg-height`, `--roll-balance-front`,
`--drag-area`, `--downforce-area`, `--aero-balance-front`, `--viscous-coupling`, `--mass` and `--max-power-kw` apply to
the four-wheel car only; the desktop offers its setup controls from `core/setup.hpp` while paused.
Do not delete `build/`, `out/` or `artifacts/`; configure a new binary directory instead.
Keep one shared instruction file; CLAUDE.md imports this file. Use the installed workflow skills
when their actual task applies; pasted skill descriptions are not commands to run all skills.
