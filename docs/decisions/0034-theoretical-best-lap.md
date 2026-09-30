# 0034: The theoretical best lap

Date: 2026-09-25. Status: accepted for TrackWayFastPlan Phase 4 (4.1 offline minimum-time solution with fastest-lap, 4.2
independent cross-check with TUM's formulation, 4.3 in the application) and Phase 8's row for it (ghost car, delta bar,
setup-sensitivity panel, energy bars). Amends decision 0033.

Phase 4 asks how a Formula One team calculates the theoretical maximum, with the same method, for this project's car and
track, and for the gap between theory and what the controller achieves to be visible. Its honesty rules: the theoretical
lap is optimal for the model, mesh, tolerance and solver status stated with it; reports always show solver status; a
failed or non-converged solve is shown as such. The plan requires the user's approval to download fastest-lap and TUM's
pinned Python environment; the user approved both on 2026-09-25 ("Download both"). Neither is linked into the build.

## Decision

**The problem is written by this build, as files.** `fd_headless --write-lap-problem DIR` writes `problem.json` and
`corridor.csv` (`adapters/recording/optimal_lap.*`): the corridor the car drives, the preset or a `--track` file
conditioned as the racing line's reference is (decision 0019) or a `--course` as traced, with each sample's station,
position, heading, curvature, left normal and edges; and the four-wheel car of `--profile` or `--plant four-wheel` and
its setup flags, with its configuration (steering lock, speed cap, grip). The text is deterministic at full precision and
fingerprinted; two problems are the same exactly when their fingerprints are. The car's scenario, instruments,
perception, planner, controller and judging are refused beside it, and so is `--line racing`: the optimum chooses its own
line within the corridor.

**fastest-lap solves it offline, in its own process.** `tools/optimal_lap/solve_fastest_lap.py` checks the pinned
release's digests, puts the car into fastest-lap's `f1-3dof` model and the corridor into its discrete track format, and
runs its optimal lap time (Ipopt, MUMPS) in a child process whose output it keeps, reading Ipopt's own exit message,
iteration count and objective from it. fastest-lap's frame has z down, so the map is mirrored across x: y, yaw,
curvature, lateral velocity, yaw rate, steering and slip angle change sign, and its left is the car's left. The tire
maps exactly: fastest-lap's `Pacejka_simple_model` shares this project's structure (decision 0006) but uses
S = π/(2 atan Q), which puts the peak at ρ = tan(π/2Q)·2 atan(Q)/π rather than 1 (TrackWayFastPlan 4.1(a)); scaling its
peak slips by the ratio of the two S makes each pure-slip curve this project's, with Q from the sliding fractions. The
corridor's edges less the car's 0.9 m half width bound the centre of gravity; the car's steering lock and speed cap bound
its controls and states. Every statement of the mapping is written into the artefact, including what fastest-lap does not
model (the slip speed floor, the kinematic blend, a per-wheel drive torque cap) and a check, from the solution's tire
forces, that neither torque cap it lacks is reached.

**The artefact is a contract.** The solver writes `optimal_lap.json` (the solver, its version, release and library
digests, model, Ipopt's status and whether it is a solution Ipopt accepts, iterations and limit, tolerance, mesh, lap
time, objective, speed cap, half width, the problem's fingerprint, the mapping, each tire's dissipated energy and the
sensitivities) and `lap.csv` (every mesh point in this project's frame), beside the problem it solved. `fd::load_optimal_lap`
reads it as one contract, as a recording is read: the problem must read back as itself and be the fingerprint solved;
the mesh starts on the line and increases in station and time; every point lies within 5 cm of where its station and
offset put it on the corridor, keeps the car's half width inside, stays within the speed cap and the lock; distance and
time agree with speed span by span, the closing span included, so the lap time is accounted; each tire's energy is its
dissipation integrated here within 2% of the solver's own; every sensitivity checked by re-solving agrees with its central
difference within 5% or a millisecond over its step. A status and a `converged` flag that disagree are refused; an
unconverged solve reads back, marked. The solver places the centre of gravity; the reader converts it to the rear axle,
the pose this project publishes. `fd_headless --check-optimal-lap DIR` prints it all and requires the artefact to solve
the problem the same flags pose in this build.

**Setup sensitivities are the solver's, stated with their method.** d(lap time)/d(parameter) for mass, centre of gravity
height, downforce area and power come from fastest-lap's own sensitivity analysis at the solution; `--check-sensitivities`
re-solves each a half step either side and records the central difference beside it. fastest-lap carries the brake bias
as a control held at the parameter's value, which its analysis does not differentiate, so the bias is always a central
difference.

**Two findings about fastest-lap, stated where they apply.** Its lap does not depend on the brake bias on this problem:
the same lap to machine precision at 0.3 and at 0.6, the hardest braking split 0.509 to the front either way, so the zero
sensitivity is the model's and is said to be. And it floors every tire's peak friction at 1.0 at every load:
`tools/optimal_lap/friction_floor.py` gives the same lateral limit in fastest-lap's own G-G diagram for friction 0.5, 0.8
and 1.0, and a proportionally larger one for 1.2 and 1.5, on two cars. A car whose friction falls below 1 is refused unless
`--accept-friction-floor` is given; then the label says the optimum grips more than the car and is faster than it can be.

**TUM cross-checks it, and found this project's own error.** `tools/optimal_lap/cross_check_tum.py` gives TUM's example
car, the electric-race-car profile, in TUM's double-track parameter set, and the same corridor, to TUM's `opt_mintime`,
imported from where it lies (LGPL-3.0, nothing copied or written there), and compares lap time, line and speed along the
corridor. The first comparison found TUM 0.97 s slower. Its own tire forces showed why: TUM's solver scales the lateral peak
by 1 + eps Fz / f_z0, and decision 0033 had read the usual 1 + eps (Fz − f_z0) / f_z0, giving TUM's car 0.1 more lateral
friction than TUM's. The profile is corrected: lateral friction 0.933 at 2000 N and 0.800 at 6000 N, longitudinal 1.0,
TUM's friction circle, floor 0.6. That left TUM still slower, because at those frictions fastest-lap's floor gives the car
1.0; TUM is therefore also solved at the floor, the car fastest-lap solves, which is the like-for-like comparison.

**The desktop shows it only while it is the car on screen.** `fd_desktop --profile ID` builds the live four-wheel car
from a profile, as the headless runner does, leaving the showroom and appearances alone; `--optimal-lap DIR` loads an
artefact. On every change of car, setup or corridor the bridge poses the live problem and compares fingerprints. While they
match, a translucent car labelled OFFLINE OPTIMUM is drawn where the optimum is at the time since the live lap began,
timed from its rear axle crossing the start line as the judge times the live car, and ringed on the circuit map; the
theoretical lap card shows the optimum's lap, Ipopt's status in words and colour, the solver, model, mesh and tolerance,
the live lap's running delta (noting a first lap from rest against a flying optimum), each sector's delta against the
optimum's sector through the judge's own gates, each tire's energy dissipated so far on the optimum's lap against its
whole lap, and each setup step's worth. When they do not match, the car and the bars are hidden and the card says whether
the car or the corridor differs. Replay shows the same from the recorded car and timing; nothing new is recorded, because
the optimum depends only on the artefact and the recorded timing.

## Verification

`tests/optimal_lap_tests.cpp` (`fd_optimal_lap`, three groups): the Formula One style car's problem on the preset has
845 corridor samples over 506.843 m, unit normals left of travel, a deterministic text and a fingerprint that changes with
the car's mass and the corridor's width, and a used directory is refused. A steady lap written as a solver would reads back
with every point, the rear axle converted from the centre of gravity, energies integrated from the line, and helpers
that interpolate and wrap; the rear axle crosses the start line 0.08 s into the lap and the judge's sector gates when it
reaches them. Eighteen violations are refused by name (a status and a converged flag that disagree either way, a
converged solve without an objective, an objective below its lap time, a malformed digest, a wrong mesh count, a
sensitivity that disagrees with its re-solve, an energy that is not its dissipation's integral, a point 0.2 m off its
station, the car's half width leaving the corridor, speed and time that disagree, speed above the cap, steering beyond
the lock, a mesh not starting on the line, time not increasing, the problem changed, the corridor changed, the problem
missing), and an unconverged solve reads back marked as one.

`tests/vehicle_profile_tests.cpp` passes with TUM's corrected tire: its lateral limit grows 5% from 15 to 30 m/s and its
envelope plan laps the preset in 29.75 s (28.70 s before).

The solves, 500 mesh points at tolerance 1e-8 (`out/optimal/p4-final-*`), each read back by `--check-optimal-lap` with
the flags it was written with:

- Formula One style: 14.143 s, "Optimal Solution Found" after 53 iterations, 27.6 to 40.0 m/s (the cap). Each tire
  dissipated 34.5, 144.5, 87.7 and 186.7 kJ (front left, front right, rear left, rear right). Mass 3.155 ms per kg
  (re-solving 3.153), so 10 kg less is worth 0.032 s; centre of gravity 2.970 s per m (2.896), 1 cm lower 0.030 s;
  downforce area −0.323 s per m² (−0.323), 0.1 m² more 0.032 s; power −5.6e-10 s per W (−5.2e-10), 10 kW more 6 µs,
  because the lap spends its straights at the cap; brake bias 4e-14 s per unit.
- Electric race car, TUM's, corrected and at fastest-lap's floor as stated: 19.822 s after 71 iterations, 18.3 to
  35.5 m/s; mass 0.447 ms per kg (0.447), centre of gravity 1.801 s per m (1.668), downforce −0.283 s per m² (−0.283),
  power −6.9e-7 s per W (−7.1e-7), 10 kW worth 6.9 ms. Without `--accept-friction-floor` it is refused, naming the four
  frictions below the floor, before anything is written.
- Formula Student electric: 16.360 s after 34 iterations, 29.97 to 30.03 m/s: its downforce holds it at its 30 m/s cap
  the whole lap, so its optimum is the shortest line the corridor allows at the cap.

Frame and mapping: on a probe solve every position rebuilt from its station and offset on the corridor lies within
0.5 mm of fastest-lap's; the reader allows 5 cm. The largest braking torque any wheel's tire force implies is 1176 N m
and driving torque 1388 N m against the F1 car's 3000 N m caps fastest-lap does not model.

fastest-lap's friction floor, by `friction_floor.py`, largest lateral acceleration at 30 m/s in its G-G diagram with every
peak friction set to 0.5, 0.8, 1.0, 1.2 and 1.5: TUM's car 1.085, 1.085, 1.087, 1.303 and 1.628 g; the Formula One car
1.353, 1.353, 1.355, 1.623 and 2.029 g. Its brake bias: the Formula One solution at bias 0.3 and 0.6 differs in speed by
7e-15 m/s at most, braking split 0.509 to the front at the hardest point either way.

The cross-check (`out/optimal/p4-final-erc-tum`), TUM's `opt_mintime` on 169 points of 3 m after its own track
preparation, both "Optimal Solution Found": TUM's car as published laps in 20.683 s, 0.861 s slower than fastest-lap,
lines a median 0.67 m apart (95th percentile 1.18 m), TUM a median 1.12 m/s slower; at fastest-lap's floor it laps in
20.023 s, 0.201 s (1.0%) slower, lines a median 0.65 m apart (1.17 m), TUM a median 0.45 m/s slower (95th percentile of
the difference 1.69 m/s), faster through the slowest corner (18.9 against 18.3 m/s) and slower at the top (35.3 against
35.5 m/s), as its rolling resistance and actuator lags would make it. Before the profile's correction the first
comparison stood at 19.711 against 20.683 s; the correction moved fastest-lap only to 19.822 s because the floor already
gave the car 1.0, which is how the floor was found.

The gap between theory and the controller, the Formula One style car on the preset (`out/runs/p4-gap-*`), the judge's
flying second lap against the optimum's 14.143 s, none leaving the course or touching a cone: the grip fraction plan
31.340 s (2.2 times), the envelope plan at 80% 20.920 s (+48%) and at 100% 19.160 s (+35%), MPCC on the 80% envelope
15.115 s (+6.9%), MPCC's line up to 3.9 m from the centreline. The optimum keeps no reserve, knows the whole lap and
chooses its line over the whole corridor; MPCC, which chooses its own line over three seconds, comes within 7% of it.
TUM's corrected car on its envelope plan laps the preset from rest in 31.93 s (30.98 s before).

The desktop's UI verification loads the Formula One style profile, writes a steady lap for exactly the live car and
corridor as a solver would, and checks that it loads as this car's optimum; that the card and the translucent car show
with the solver and status; that after 12 s the optimum has driven on from the line, the first sector is timed against
the optimum's, each tire's energy so far is part of its lap and the sensitivities are listed; that the card sits beneath
the G-G card, beside the tire card and above the bottom bar in the compact window; that a changed mass hides the car and
says the setup differs; and that resetting the setup makes it the optimum's car again. Capture:
`build/desktop/ui-evidence/compact-optimal.png`.

Builds (source fingerprint `acf7dab6372560aa09d627d0aa7aac92e7a97fc7520b01fac4d06f543ed28245`): `scripts/build.ps1`
passed 28 of 28 suites in 688.3 s and `scripts/build.ps1 -Desktop` 30 of 30 in 723.9 s, the UI suite 615 checks in
115.8 s with no QML warning, and no compiler warning. An earlier desktop run lost the UI suite and the recording suite
to their timeouts while offline solves ran beside them; alone they took 162 s and 76 s, so the UI suite's limit is now
300 s.

Two mutations were run and reverted. Switching off the reader's check of a point against its station and offset let the
moved point through to the speed check, so the named refusal failed; timing gates by the centre of gravity instead of
the rear axle put the start crossing at zero instead of 0.08 s. Both failed `fd_optimal_lap`, and the restored source
passes it.

## What this is not

Not a run of this project's plant, and not what the controller could do: the optimum knows the whole lap, keeps no
reserve and is solved for fastest-lap's 3-DOF model, which shares this project's degrees of freedom and tire structure
but not its integration, its slip floors or its kinematic blend. Not the fastest this car can go on this track in any
absolute sense: it is optimal at the stated mesh and tolerance, within the corridor less 0.9 m, the lock and the speed
cap. Not a Formula One team's number: the Formula One style car is Limebeer's published academic car at a 40 m/s cap. The
cross-check compares two optimisers of two models; their agreement bounds the method, not the car.

## Consequences

- AGENTS.md states the theoretical best lap as an invariant: an offline optimum of a stated problem, never driving, shown
  only while its problem is on screen, a floored or unconverged solve saying so.
- The recording adapter's JSON and CSV readers are shared through `contract_io.inc`; the recording contract is unchanged.
- Decision 0033's TUM car is corrected, and its envelope lap on the preset is 29.75 s, not 28.70 s.
- A car with front drive is refused by the solve: fastest-lap's `f1-3dof` drives the rear axle alone, so the project's
  roadster (half front drive) has no optimum yet.
