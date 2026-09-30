# TrackWayFastPlan

**How Formula-Driverless-Path-Predictor gets from a kinematic demonstrator to a realistic, fast,
explainable driverless race car, using algorithms that seven open-source projects have already
solved.**

Prepared 2026-09-14 from a source-code audit, for
[Formula-Driverless-Path-Predictor](../README.md). This is a plan, not a record of work done.
Nothing in the audited projects was built, run or copied into this repository while writing it.
Every claim about the current codebase refers to its state on this date; every claim about an
audited project refers to the downloaded copy listed in [Source map](#1-source-map). Every local
path cited below was checked to exist, and quoted parameters and equations were re-read from the
named files, on the same date. The audit was made in two independent passes; the second re-derived
the tire, load-transfer and wheel-dynamics claims from the source and corrected the first where it
had reasoned loosely, see [§13](#13-audit-method-and-limits).

---

## Contents

- [0. Executive summary](#0-executive-summary)
- [1. Source map](#1-source-map)
- [2. Where the project stands today](#2-where-the-project-stands-today)
- [3. The goal, stated precisely](#3-the-goal-stated-precisely)
- [4. Audit of each project](#4-audit-of-each-project)
  - [4.1 fastest-lap](#41-fastest-lap)
  - [4.2 TUM global race trajectory optimization](#42-tum-global-race-trajectory-optimization)
  - [4.3 MPCC](#43-mpcc)
  - [4.4 PacSim](#44-pacsim)
  - [4.5 ForzaETH race_stack](#45-forzaeth-race_stack)
  - [4.6 FaSTTUBe ft-fsd-path-planning](#46-fasttube-ft-fsd-path-planning)
  - [4.7 FS-FEUP autonomous-systems](#47-fs-feup-autonomous-systems)
- [5. Licensing and dependency rules](#5-licensing-and-dependency-rules)
- [6. The combination map](#6-the-combination-map)
- [7. The plan, phase by phase](#7-the-plan-phase-by-phase)
  - [Phase 0: Make room for realism](#phase-0-make-room-for-realism)
  - [Phase 1: Tires and a dynamic single-track plant](#phase-1-tires-and-a-dynamic-single-track-plant)
  - [Phase 2: Wheels, brakes, load transfer and aero](#phase-2-wheels-brakes-load-transfer-and-aero)
  - [Phase 3: A vehicle-derived envelope and a real racing line](#phase-3-a-vehicle-derived-envelope-and-a-real-racing-line)
  - [Phase 4: The theoretical best lap](#phase-4-the-theoretical-best-lap)
  - [Phase 5: A local planner that looks a few metres ahead](#phase-5-a-local-planner-that-looks-a-few-metres-ahead)
  - [Phase 6: MPCC, the flagship controller](#phase-6-mpcc-the-flagship-controller)
  - [Phase 7: Sensors, uncertainty and unknown tracks](#phase-7-sensors-uncertainty-and-unknown-tracks)
  - [Phase 8: Making every new decision visible](#phase-8-making-every-new-decision-visible)
- [8. Selectable vehicles with honest physics](#8-selectable-vehicles-with-honest-physics)
- [9. Invariants that change, and when](#9-invariants-that-change-and-when)
- [10. Verification strategy](#10-verification-strategy)
- [11. What not to take, and why](#11-what-not-to-take-and-why)
- [12. Effort, order and risk](#12-effort-order-and-risk)
- [13. Audit method and limits](#13-audit-method-and-limits)
- [14. Glossary](#14-glossary)

---

## 0. Executive summary

**Goal.** Make the car drive like a real race car and decide like a driverless race car: fast,
at the limit of its tires, without crashing, reacting to its situation a few metres at a time,
and able to explain every decision on screen.

**The honest gap today.** The car is a kinematic bicycle. It has no tires, no mass effect, no
wheel speeds, no load transfer and no way to lock a wheel. Its speed plan uses fixed grip
fractions instead of a real performance envelope. Its local planner chooses among five fixed
lateral offsets and cannot find a genuinely faster line. None of that is a flaw in the work so
far; it is the correct place to have started. It is also exactly what the seven downloaded
projects have already solved.

**The one-sentence plan.** Replace the plant first, then derive the performance envelope from
that plant, then plan a racing line against the envelope, then compute the theoretical best lap
offline, then plan locally and control with optimisation, and only then add sensors, making each
step visible as it lands.

**What comes from where.**

| Capability | Primary source | Cross-check |
| --- | --- | --- |
| Tire forces with load sensitivity and combined slip | fastest-lap structure, Magic Formula shape as in TUM and MPCC | FS-FEUP |
| Dynamic single-track plant | MPCC | PacSim, race_stack |
| Wheel spin, brake bias, locking, power limit, differential | fastest-lap | FS-FEUP |
| Quasi-static load transfer and aero | fastest-lap and TUM, the same model written two ways | FS-FEUP |
| Steering at the limit (understeer-aware) | race_stack MAP | MPCC |
| Reactive fallback when no planned line is feasible | race_stack follow-the-gap | none needed |
| G-G-V performance envelope from the vehicle | fastest-lap | TUM, FS-FEUP |
| Minimum-curvature racing line | TUM | FS-FEUP, fastest-lap |
| Theoretical minimum lap time | fastest-lap | TUM |
| Local lattice planner with action sets | TUM graph planner in race_stack | MPCC corridor search |
| Model predictive contouring control | MPCC | race_stack, FS-FEUP |
| Sensor rate, delay and noise; mock cone detections | PacSim | FS-FEUP |
| Unknown-track path from cones | FaSTTUBe | FS-FEUP |
| Competition rules, penalties and FS tracks | PacSim | FS-FEUP |

**What to do next session.** Phase 0: make nearest-point projection fast, introduce a
vehicle-model seam with the current kinematic bicycle as its first adapter, and write the tire
module from fastest-lap's structure with the peak and the sliding residual chosen deliberately,
with tests. That unblocks everything else and changes no existing behaviour.

**Scale.** Roughly 430 to 800 focused hours across all phases. Each phase ships something that
runs and something that is visible. This is a roadmap to choose slices from, not a sprint.

---

## 1. Source map

The downloads were extracted with an extra nested folder. The real roots are:

| Alias | Project | Local root | Upstream | License |
| --- | --- | --- | --- | --- |
| `<FL>` | fastest-lap | `C:\Users\Firat\Downloads\BunchOfFFormulaPathProjects\fastest-lap-main\fastest-lap-main` | github.com/juanmanzanero/fastest-lap | MIT |
| `<TUM>` | global_racetrajectory_optimization | `C:\Users\Firat\Downloads\BunchOfFFormulaPathProjects\global_racetrajectory_optimization-master\global_racetrajectory_optimization-master` | github.com/TUMFTM/global_racetrajectory_optimization | LGPL-3.0 |
| `<MPCC>` | MPCC (fullsize) | `C:\Users\Firat\Downloads\BunchOfFFormulaPathProjects\MPCC-fullsize\MPCC-fullsize` | github.com/alexliniger/MPCC | Apache-2.0 |
| `<PAC>` | PacSim | `C:\Users\Firat\Downloads\BunchOfFFormulaPathProjects\pacsim-master\pacsim-master` | github.com/PacSim/pacsim | MIT |
| `<RS>` | ForzaETH race_stack | `C:\Users\Firat\Downloads\BunchOfFFormulaPathProjects\race_stack-main\race_stack-main` | github.com/ForzaETH/race_stack | MIT root; vendored parts differ, see §5 |
| `<FT>` | FaSTTUBe ft-fsd-path-planning | `C:\Users\Firat\Downloads\BunchOfFFormulaPathProjects\ft-fsd-path-planning-main\ft-fsd-path-planning-main` | github.com/papalotis/ft-fsd-path-planning | MIT |
| `<FEUP>` | FS-FEUP autonomous-systems | `C:\Users\Firat\Downloads\BunchOfFFormulaPathProjects\autonomous-systems-main\autonomous-systems-main` | github.com/fs-feup/autonomous-systems | GPL-3.0 |
| `<PROJECT>` | This project | `C:\Users\Firat\Documents\Formula-Driverless-Path-Predictor` | local only, no remote | not yet decided, see ADR 0001 |

**Missing pieces.** GitHub zip downloads omit git submodules. These folders are empty locally
and were therefore not audited:

- `<RS>\planner\gb_optimizer\src\global_racetrajectory_optimization` (ForzaETH's modified copy of `<TUM>`)
- `<RS>\planner\predictive-spliner`, `<RS>\planner\multiopponent-pspliner`
- `<RS>\base_system\f110-simulator`, `<RS>\base_system\f1tenth_system`
- `<RS>\system_identification\on_track_sys_id`, `<RS>\f110_utils\libs\ccma`
- `<FEUP>\ext\acados`, `<FEUP>\ext\osqp\src`, `<FEUP>\ext\gtsam`, `<FEUP>\ext\kiss-icp`

TUM's minimum-curvature QP and velocity-profile code also lives outside `<TUM>`, in the separate
`trajectory_planning_helpers` package that its `requirements.txt` pins at version 0.76. That
package was not downloaded, so its implementation is described from how `<TUM>` calls it, not
from its source. fastest-lap's numerics layer `lion-cpp` is fetched at build time and was not
inspected either.

---

## 2. Where the project stands today

Read [`STATUS.md`](STATUS.md), [`ARCHITECTURE.md`](ARCHITECTURE.md) and the decision records for
the full evidence. The parts this plan builds on:

| Concern | Current implementation | File |
| --- | --- | --- |
| Vehicle plant | Kinematic bicycle at the rear axle, exact arc integration per 5 ms tick, steering angle and rate limits, no mass, no slip | `<PROJECT>\core\src\core.cpp`, `integrate_bicycle` |
| State | Rear-axle x, y, yaw, speed, front-wheel angle, time | `<PROJECT>\core\include\fd\core.hpp`, `State` |
| Configuration | Grip μ, lateral grip fraction 0.65, longitudinal 0.55, wheelbase, speed cap, controller gains | same file, `Config`; `<PROJECT>\configs\default.cfg` |
| Track | Foundry Circuit, 507.56 m, 10 m wide, lines and 18 to 38 m arcs, samples every 0.6 m | `make_preset_track`; `<PROJECT>\tracks\README.md` |
| Speed plan | `sqrt(0.65·μ·g/|κ|)` cap, periodic forward and backward passes at `0.55·μ·g` | `make_speed_plan` |
| Controller | Pure Pursuit plus speed feedback and planned-acceleration feedforward | `compute_control` |
| Prediction | Three-second rollout of the same policy from actual state | `<PROJECT>\core\src\trajectory.cpp` |
| Local planner | Five lateral offsets, reject blocked or out-of-envelope, pick greatest advance, else brake | `<PROJECT>\core\src\local_planner.cpp` |
| Simulation | One plant, fixed 5 ms tick, 20 ms control, tick-boundary parameter events | `<PROJECT>\adapters\simulation\src\simulation.cpp` |
| Recording | Schema 1 cross-checked run directory; replay seeks a cursor | `<PROJECT>\adapters\recording` |
| Presentation | Native Qt Quick 3D driving view, prediction ribbon, rejected lines, decision panel | `<PROJECT>\apps\desktop` |

Three measured facts shape the order of this plan:

- **Planning cost is already tight.** A blocked decision costs about 16 ms against a 20 ms
  control period, and `fd_core_behavior` takes about 70 s. The cause is `project()`, a linear
  scan over every track sample, called at every predicted control point. Any richer model or
  planner multiplies that cost, so it has to be fixed first.
- **The recording contract cannot yet hold richer state.** Schema 1's telemetry header is compared
  exactly. Wheel speeds, slip and loads need a schema 2 branch.
- **The plant has no mass.** Any control that pretends to change weight is decorative today, which
  AGENTS.md forbids. The plant must change before the "weight slider" can become honest.

---

## 3. The goal, stated precisely

The user's vision, translated into engineering requirements that can be tested:

| Vision in the user's words | Engineering requirement | Becomes true in |
| --- | --- | --- |
| "Realistic driving", "wheel variables and everything" | Four wheels with rotational dynamics, tire forces from slip ratio and slip angle, load transfer, aero, brakes and power limit | Phases 1 and 2 |
| "How much braking without locking up" | Wheel speed can fall to zero under excess brake torque; tire force drops past peak slip; a controller can hold slip near its peak | Phase 2, controller in Phase 6 |
| "Change the weights, change the car stats" | Mass, centre-of-gravity height, brake bias, aero and power change measured behaviour through implemented equations | Phase 2 |
| "Custom line that's perfect with the racing line" | A minimum-curvature line inside the track limits, with a speed profile from the car's own G-G-V envelope | Phase 3 |
| "How the F1 team theoretical max speed was calculated" | A minimum-lap-time optimal control solution for the same car and track, with setup sensitivities | Phase 4 |
| "Couple metres at a time, always adjusting" | A local planner that re-plans from actual state every control tick over a spatial lattice and returns an action set | Phase 5 |
| "Planned on being fast, breaking lap records, without crashing" | An optimising controller that maximises progress subject to track and tire-friction constraints | Phase 6 |
| "Tesla-like situational awareness", "pedestrian avoidance" | Observations with rate, delay, noise and missed detections; decisions made from observations, not ground truth | Phase 7 |
| "Make decisions visible" | Every new quantity above is drawn and explained, not only computed | Phase 8, continuously |

Two boundaries stay in force throughout, because they are what keep the portfolio credible:

- **Optimal means optimal for the model.** A lap time from an optimiser is the best lap for that
  model, mesh, tolerance and solver status. It is not a prediction of a real car.
- **Simulated observations are not perception.** Mock cone detections exercise planning under
  uncertainty. They do not evaluate a camera or LiDAR detector, and nothing here detects a
  pedestrian.

---

## 4. Audit of each project

Each audit answers the same questions: what the project really contains, which algorithms are
worth taking, the exact files, the equations in the form this project needs, and the traps.

### 4.1 fastest-lap

**What it is.** A C++17 vehicle dynamics and lap-time optimisation library by Juan Manzanero,
driven from Python or MATLAB through a C API. It computes numerical G-G diagrams and minimum-lap-time
optimal control for a 3-DOF F1 model and a 6-DOF kart model. Its README reports an optimal lap of
Circuit de Catalunya with 500 mesh points in about one minute. It solves the fully transient
equations rather than a quasi-steady-state approximation. Its core is about 11,600 lines of
C++ headers under `<FL>\src\core`.

**Why it is the best fit.** It is the only audited project whose central purpose is exactly the
user's question: what is the fastest a given car can drive a given track, and why. Its models
are published, parameterised in readable XML, MIT-licensed, and documented by engineering blog
posts that explain tire dynamics, understeer and lift-and-coast with simulations.

**Algorithms worth taking.**

**a) The simple Pacejka tire with load sensitivity and combined slip.**
Files: `<FL>\src\core\tire\tire_pacejka.h`, `<FL>\src\core\tire\tire_pacejka.hpp`
(`Pacejka_simple_model`), slip definitions in `<FL>\src\core\tire\tire.h`.

Slip is defined as

- longitudinal slip `κ = (ω·R0 − vx) / vx`
- lateral slip, the sideslip angle, `λ = −atan(vy / vx)`

Peak friction and the slip at which it occurs both vary linearly with vertical load between two
reference loads `Fz1` and `Fz2`. That is load sensitivity: a more heavily loaded tire gives more
force but a lower friction coefficient. Longitudinal and lateral slip share one normalised radius:

```
κn = κ / κmax(Fz)        λn = λ / λmax(Fz)        ρ = sqrt(κn² + λn²)
Sx = π / (2·atan(Qx))
Fx = μx_max(Fz) · Fz · sin(Qx · atan(Sx · ρ)) · κn / ρ
Fy = μy_max(Fz) · Fz · sin(Qy · atan(Sy · ρ)) · λn / ρ
```

Because both forces scale by the same `ρ`, braking hard while cornering automatically leaves less
lateral grip. Beyond the peak the force falls, which is what makes a locked wheel lose grip.

*Trap, verified numerically.* `tire_pacejka.h` documents `kappa-max` as the "slip coefficient for
the friction peak", but the shape factor `S = π/(2·atan(Q))` does not put the peak there. The peak
sits at `ρ* = tan(π/2Q) · 2·atan(Q)/π`; for the F1 value `Q = 1.9` that is `ρ* = 0.751`, and the
curve is at 0.965 of peak at `ρ = 1`. The expression that would put the peak at `ρ = 1` by
construction is `S = tan(π/2Q)`. At `Q = 1.9`, `tan(π/2Q)` and `atan(Q)` happen to agree to four
digits, 1.0863 both, which looks like where the slip came from; the code uses the reciprocal of one
where the other was meant.

**What matters more for this project.** The same shape sets how much grip a sliding tire keeps.
With `Q = 1.9` a locked wheel (`κ = −1`, so `ρ ≈ 9` with `κmax = 0.11`) keeps 0.30 of peak force,
and the large-slip limit `sin(Qπ/2)` is 0.16. Tires on dry asphalt keep roughly 0.7 to 0.85 of
peak when sliding, which is why anti-lock braking is worth tens of percent of stopping distance,
not a factor of three. Ported as-is, this shape would make every lock about three times more
dramatic than reality, and the "braking without locking" story this project wants to tell would
be exaggerated by the model rather than shown by it. MPCC's lateral shape, `C = 1.38`, keeps 0.83;
`Q = 1.5` keeps 0.78 at lock. Phase 1.1 therefore keeps fastest-lap's load-sensitive combined-slip
*structure*, places the peak at `κmax` by construction, and chooses the shape factor for a stated
sliding residual, with a test for each.

Two smaller notes for the port. Friction extrapolates linearly above `Fz2` with only a floor
`mu_min`, so a high-downforce profile must be tested above 6000 N per wheel. And sharing one
normalised radius between the longitudinal and lateral curves is the similarity approximation to
combined slip, not measured combined-slip data; it is the right first model, and it should be
labelled as such.

**b) The 3-DOF chassis with algebraic wheel loads.**
Files: `<FL>\src\core\chassis\chassis_car_3dof.h`, `<FL>\src\core\chassis\chassis_car_3dof.hpp`,
base in `<FL>\src\core\chassis\chassis.h`, `<FL>\src\core\chassis\chassis.hpp`.

States are longitudinal velocity `u`, lateral velocity `v` and yaw rate `ω`. The four vertical
tire loads are algebraic unknowns solved together with vertical force balance, roll and pitch
moment balance and a roll-balance closure

```
(Fz_fr − Fz_fl)·(1 − D) + (Fz_rl − Fz_rr)·D = 0
```

that resolves the statically indeterminate four-wheel distribution with a roll balance
coefficient `D`. Load transfer is therefore not assumed; it emerges from the force and moment
balance including aero. Aerodynamics in `chassis.hpp`, `get_aerodynamic_force`:

```
drag = ½ · ρ · cd · A · v_air · |v_air|        lift = ½ · ρ · cl · A · v_air,x²
```

applied at a pressure centre, with an optional wind vector.

*How this runs in a fixed-tick plant.* The four loads are algebraic unknowns, so inside fastest-lap
the optimiser solves them together with everything else. That does not make them unusable in an
explicit simulation. Given the tire horizontal forces, the four equations, vertical balance, pitch
and roll moment balance, and the roll-balance closure, are *linear* in the four loads: one 4×4
solve per substep, with the horizontal forces taken from the previous substep. The feedback from
loads back to forces through load sensitivity is weak, about 20% of friction across a 4000 N load
change, so the one-substep lag is benign. TUM's closed-form load transfer in §4.2 is the same
quasi-static assumption written out by hand; `roll_balance_coefficient` here is `k_roll` there.
The real differences are small terms: fastest-lap keeps the aerodynamic moment about the pressure
centre and the exact moment arms. Phase 2.2 implements one form and tests it against the other.

**c) Axles, brakes, engine power and the differential.**
Files: `<FL>\src\core\chassis\axle_car_3dof.h`, `<FL>\src\core\chassis\axle_car_3dof.hpp`,
`<FL>\src\core\actuators\engine.h`, `<FL>\src\core\actuators\engine.hpp`,
`<FL>\src\core\actuators\brake.h`.

- The front axle is steering and free-rolling; the rear axle is powered.
- Throttle is one control in `[−1, 1]`; negative means brake. Brake torque is
  `brake_percentage · max_torque · bias`, with `bias` at the front and `1 − bias` at the rear,
  opposing wheel rotation through a smooth sign.
- Engine torque comes from power at the mean axle wheel speed:
  `T_engine = P(throttle) / ½(ω_left + ω_right)`, where `P = throttle · P_max` in the
  maximum-power mode. An optional boost engine adds power the same way.
- A viscous differential moves `k_diff · (ω_left − ω_right)` of torque between the rear wheels.
  The source comment notes the axle's net power is engine power minus the differential's
  dissipation `k_diff · (ω_left − ω_right)²`.
- Each wheel's angular momentum obeys `dL/dt = T_brake_and_drive + T_tire`, where `T_tire` is
  the tire's longitudinal torque at the wheel centre.

In `axle_car_3dof.h` the algebraic inputs of an axle are the two slip ratios `κ`, and the states
are the two angular momenta. With the published F1 wheel inertia of zero the wheel equation
collapses to `0 = T_brake_and_drive + T_tire(κ)`, and the optimiser solves it for `κ` directly.
The model never integrates a wheel speed, which is why fastest-lap never meets the stiffness a
time-stepping plant meets. A test case with wheel inertia exists,
`<FL>\src\test\applications\data\f1_optimal_laptime_catalunya_discrete_with_wheel_inertia.xml`.
The live plant in this project must integrate wheel speed with non-zero inertia to show a lock
happening; Phase 2.1 gives the number that makes that stiff.

**d) The curvilinear road frame, which is what makes lap time optimisable.**
Files: `<FL>\src\core\vehicles\road_curvilinear.h`, `<FL>\src\core\vehicles\road_curvilinear.hpp`.

With lateral displacement `n` from the reference line, heading `α` relative to it, and reference
curvature `κ_ref`:

```
dt/ds = (1 − n·κ_ref) / (u·cos α − v·sin α)
dn/dt = u·sin α + v·cos α
dα/dt = ω − κ_ref / (dt/ds)
```

Lap time is `∫ (dt/ds) ds` over the lap. Integrating along distance rather than time is what lets
a fixed spatial mesh represent a lap whose duration is the unknown being minimised. This belongs
in offline optimisation only. The live plant stays in time, at fixed ticks, as AGENTS.md requires.

**e) Minimum-lap-time optimal control.**
Files: `<FL>\src\core\applications\optimal_laptime.h`, `<FL>\src\core\applications\optimal_laptime.hpp`,
F1 model wrapper `<FL>\src\core\vehicles\limebeer2014f1.h`.

- Objective: `∫ dt/ds ds` plus a dissipation penalty `λ_u · ∫ (du/ds)² ds` per control, which
  keeps controls smooth.
- Transcription: first-order collocation with a θ-method on the spatial mesh. The default
  `σ = 0.5` is Crank-Nicolson, the trapezoidal rule named in the README; algebraic equations are
  enforced at every node, with periodic closure for closed tracks.
- Extra constraints per node for the F1 model: the normalised slip angle `λ / λmax(Fz)` of all
  four tires, and real track limits, which the source comment in
  `<FL>\src\core\vehicles\limebeer2014f1.h` writes as `−wL < n + sign(n)·t·cos α < wR`.
- Integral constraints: engine energy, per-tire energy dissipated, boost time. These produce
  the lift-and-coast behaviour from its blog.
- Solver: Ipopt with CppAD automatic differentiation, sparse, warm-startable.
- **Setup sensitivities.** It computes `d(lap time)/d(parameter)` from the solution. That answers
  "which setup change buys the most lap time", which is exactly the kind of question an F1
  engineer asks and a strong visual feature.

**f) Numerical G-G diagram.**
Files: `<FL>\src\core\applications\steady_state.h`, `<FL>\src\core\applications\steady_state.hpp`.
For a speed `v`, first solve for maximum lateral acceleration, then for each lateral
acceleration find the maximum and minimum feasible longitudinal acceleration. Each is a
steady-state nonlinear problem of the full vehicle model, solved with Ipopt. Repeat for several speeds and the result is the
car's G-G-V envelope, including aero and load transfer effects.

**g) Minimum-curvature path and circuit preprocessing.**
Files: `<FL>\src\core\applications\minimum_curvature_path.h`,
`<FL>\src\core\applications\minimum_curvature_path.hpp`,
`<FL>\src\core\applications\circuit_preprocessor.h`,
`<FL>\src\core\applications\circuit_preprocessor.hpp`.
The circuit preprocessor reads left and right boundaries as KML files, for example traced in
Google Earth, and solves an Ipopt optimisation for a smooth centreline with curvature. Its
weights, such as `eps_k` and `eps_n`, are in `circuit_preprocessor.h`. That is the rigorous version of the original "draw a track and
something fine-tunes it into realistic corners" idea. Example walkthrough:
`<FL>\examples\python\circuit_preprocessor\README.md`.

**Data worth taking.**

- F1 vehicles: `<FL>\database\vehicles\f1\limebeer-2014-f1.xml`, `mercedes-2020-catalunya.xml`,
  `ferrari-2022-australia.xml`, `redbull-2022-imola-wet.xml`.
- Karts: `<FL>\database\vehicles\kart\roberto-lot-kart-2016.xml`, `rental-kart.xml`.
- Tracks: `<FL>\database\tracks\` includes Catalunya, Imola, Hungaroring, Melbourne, Mugello,
  Laguna Seca, Zolder and several karting circuits.

The Limebeer 2014 F1 parameters give a feel for scale: 660 kg, yaw inertia 450 kg·m², centre of
gravity 0.3 m high, axles at +1.8 m and −1.6 m, track 1.46 m, `cd = 0.9`, `cl = 3.0` on 1.5 m²,
735 kW engine plus 120 kW boost, 5000 N·m brake torque per wheel, brake bias 0.6, tire radius
0.33 m, friction 1.75 falling to 1.40 longitudinally and 1.80 falling to 1.45 laterally between
2000 N and 6000 N of load.

*Provenance trap.* The team-named files are community fits, not team data. The project must
keep original branding, as `PROJECT_BRIEF.md` requires, and label parameter provenance.

**Build reality on this machine.** fastest-lap's Windows CI builds under MSYS2 with GCC and
gfortran (`<FL>\.github\workflows\windows.yml`), because Ipopt needs a Fortran linear solver.
This project builds with MSVC. Linking fastest-lap into `fd_core` is therefore the wrong first
move. Its README lists prebuilt Windows releases v0.1 to v0.5 containing the DLL and Python
bindings, which is the pragmatic route for offline use in Phase 4. Downloading those needs the
user's approval.

**Further reading in the source.** `<FL>\docs\source\blogs\blog_car_tire_dynamics_part_1.rst`,
`blog_car_tire_dynamics_part_2.rst`, `blog_lift_and_coast.rst`, `blog_130R_Suzuka.rst`, and the
optimal lap walkthrough `<FL>\examples\python\f1\optimal-laptime\1-simple-lap\README.md`.

---

### 4.2 TUM global race trajectory optimization

**What it is.** Python tooling from TU Munich's Institute of Automotive Technology (TUMFTM). It
offers shortest path, minimum curvature, iterated minimum curvature, minimum time, and minimum
time with powertrain thermal and battery behaviour. Output is a CSV race trajectory with columns
`s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2`. Its own README notes that the
minimum-curvature line is close to a minimum-time line in corners but differs wherever the car's
acceleration limits are not exploited.

**Pipeline, from `<TUM>\main_globaltraj.py`.**

1. Import `[x, y, w_right, w_left]` from `<TUM>\inputs\tracks\*.csv`.
2. Prepare the track, `<TUM>\helper_funcs_glob\src\prep_track.py`: linear resampling at 1 m,
   B-spline regression with order 3 and smoothing factor 10, resampling at 3 m, normal vectors,
   and a check that neighbouring normals do not cross, with optional minimum-width inflation.
3. Optimise lateral shifts `α` along the normals.
4. Build the race line from splines and compute heading and curvature analytically.
5. Compute the velocity profile from a G-G-V table plus drag and machine acceleration limits.
6. Compute acceleration and time profiles, then check the result with
   `<TUM>\helper_funcs_glob\src\check_traj.py` against boundaries, curvature limit and G-G-V.
7. Export with `<TUM>\helper_funcs_glob\src\export_traj_race.py`.

**Algorithms worth taking.**

**a) Minimum-curvature line.** Implemented in the external `trajectory_planning_helpers` package
as `opt_min_curv` and `iqp_handler`. From the paper this repository cites, Heilmeier et al. 2019,
DOI 10.1080/00423114.2019.1631455: curvature of the shifted line is linearised in the shifts `α`,
the sum of squared curvature becomes a quadratic program, and shifts are bounded by track width
minus vehicle width. The iterated variant re-linearises around the previous solution until the
curvature error falls below a threshold. `<TUM>\params\racecar.ini` uses a 3.4 m optimisation
width and a 0.12 rad/m curvature limit for its 2 m wide car.

**b) G-G-V velocity profile.** External `calc_vel_profile`. Inputs are `ggv.csv` rows of speed,
maximum longitudinal and lateral acceleration, and `ax_max_machines.csv` rows of speed and
powertrain acceleration limit, both in `<TUM>\inputs\veh_dyn_info\`. A `dyn_model_exp` exponent
between 1 and 2 shapes how lateral demand reduces longitudinal capacity. Drag is included, so the
car decelerates on its own. This is the direct replacement for this project's fixed 0.65 and 0.55
fractions.

**c) Minimum-time optimal control with a double-track model.** Local and complete:
`<TUM>\opt_mintime_traj\src\opt_mintime.py`, explained in `<TUM>\opt_mintime_traj\Readme.md`.

- States: speed `v`, sideslip `β`, yaw rate `ωz`, lateral offset `n`, relative heading `ξ`.
- Controls: steering `δ`, drive force, brake force, and an auxiliary lateral load transfer `γy`.
- Transcription: direct orthogonal Gauss-Legendre collocation of degree 3, curvilinear abscissa,
  CasADi automatic differentiation, Ipopt.
- **Quasi-static load transfer in closed form.** This is the piece the live plant needs:

```
Fz_static,front = ½ · m · g · l_r / L            (per wheel)
Fz_aero,front   = ½ · c_lift,front · v²          (per wheel)
ΔFz_long        = ∓ ½ · (h / L) · (F_drive + F_brake − F_drag − F_roll)
ΔFz_lat,front   = ± k_roll · γy          ΔFz_lat,rear = ± (1 − k_roll) · γy
γy              = (h / t_mean) · ((Fy_fl + Fy_fr)·cos δ + Fy_rl + Fy_rr + (Fx_fl + Fx_fr)·sin δ)
```

  In the source, `γy` is a control variable tied to that expression by an equality constraint.
  A live plant can evaluate the same expression from the previous substep's forces.

  Because every load is an explicit function of forces already known, no algebraic loop is
  needed, unlike fastest-lap's formulation.
- Tires: the standard Magic Formula for lateral force,
  `Fy = D · sin(C · atan(B·α − E·(B·α − atan(B·α))))`, with a load-dependent peak
  `D = μ · Fz · (1 + ε·Fz/Fz0)`. Longitudinal force is not a slip curve at all: drive and brake
  forces are controls, split by `k_drive_front` and `k_brake_front`, and the per-wheel Kamm circle
  below is what couples them to lateral grip. That is a clean separation for an optimiser and a
  weak one for a plant, which is why the tire shape comes from here and the slip structure from
  fastest-lap.
- **Kamm circle per wheel:** `(Fx/(μ·Fz))² + (Fy/(μ·Fz))² ≤ 1`.
- Actuator dynamics as rate constraints with time constants for steering, drive and brake.
- No simultaneous throttle and brake: `F_drive · F_brake ≈ 0`.
- Power limit: `v · F_drive ≤ P_max`.
- Optional per-wheel friction coefficients from a friction map, as linear or Gaussian-basis
  functions of lateral position, from `<TUM>\inputs\frictionmaps\` and `<TUM>\frictionmap\src\`.
- Optional energy limit per lap, and optional "safe trajectory" acceleration ellipse.

**Traps.**

- **Frame conventions differ from this project.** TUM's normal vectors point right in the
  direction of travel, and heading zero points north along +y. This project uses left-positive
  offsets and yaw zero along +x. Every import must convert both, with a test on a known shape.
- LGPL-3.0. See §5: reimplement from the papers, do not paste code into `fd_core`.
- The package versions pinned in `<TUM>\requirements.txt` are old (NumPy 1.18.1, CasADi 3.5.1,
  Python 3.7 era). Running it unchanged on a current Python may fail.

---

### 4.3 MPCC

**What it is.** Alexander Liniger's Model Predictive Contouring Control from ETH Zurich,
in C++ and MATLAB. The C++ version follows the AMZ Driverless full autonomous system paper,
arXiv 1905.05150, with a dynamic single-track model sized like a Formula Student car.

**Algorithms worth taking.**

**a) Dynamic single-track model.** File: `<MPCC>\C++\Model\model.cpp`, function `getF`.

```
α_f = −atan2(vy + r·lf, vx) + δ         α_r = −atan2(vy − r·lr, vx)
F_fy = Df · sin(Cf · atan(Bf · α_f))    F_ry = Dr · sin(Cr · atan(Br · α_r))
F_rx = Cm1·D − Cm2·D·vx                 F_fric = −Cr0 − Cr2·vx²
v̇x = (F_rx + F_fric − F_fy·sin δ + m·vy·r) / m
v̇y = (F_ry + F_fy·cos δ − m·vx·r) / m
ṙ  = (F_fy·lf·cos δ − F_ry·lr) / Iz
```

Parameters in `<MPCC>\C++\Params\model.json`: 190 kg, `Iz = 110`, `lf = lr = 1.22 m`,
`B = 10`, `C = 1.38`, `D = 1500 N`. Analytic Jacobians follow in `getModelJacobian`, and
`discretizeModel` combines a matrix-exponential zero-order hold with an RK4 offset term.

**b) Contouring and lag error, with progress maximisation.** File: `<MPCC>\C++\Cost\cost.cpp`.
The reference line is an arc-length spline, `<MPCC>\C++\Spline\arc_length_spline.cpp`. With the
reference point and heading `θ_ref` at the progress variable `s`:

```
e_c = −sin θ_ref · (x_ref − X) + cos θ_ref · (y_ref − Y)     (contouring, lateral)
e_l =  cos θ_ref · (x_ref − X) + sin θ_ref · (y_ref − Y)     (lag, along-track)
J   = Σ q_c·e_c² + q_l·e_l² − q_v·v_s + q_β·β² + q_r·r² + input and input-rate costs
```

The negative weight on progress speed `v_s` is what makes the controller drive fast rather than
merely track. `<MPCC>\C++\Params\cost.json` weights lag error heavily, `q_l = 1000`, so the
progress variable stays honest.

**c) Friction and stability constraints.** File: `<MPCC>\C++\Constraints\constraints.cpp`.

- Track: linearised half-spaces at the progress point.
- Rear tire friction ellipse: `(E_long·F_rx/F_N)² + (F_ry/F_N)² ≤ (E_eps·D_r/F_N)²`, with
  `E_long = 1.25`, `E_eps = 0.95`.
- Front slip angle: `|α_f| ≤ 0.15 rad`. With `B = 10`, `C = 1.38` the front tire peaks at
  `α = tan(π/2C)/B = 0.217 rad`, so the constraint holds the front tire on the rising, stable side
  of its curve, about 70% of the way to the peak. That is a design choice worth copying: the
  controller never plans on the far side of the peak, where the linearisation points the wrong way.
- All softened with quadratic and linear slack penalties.

**d) Real-time iteration.** File: `<MPCC>\C++\MPC\mpc.cpp`, `runMPC`. Two SQP iterations per call,
solution mixing 0.9, warm start, reset after repeated solver failures. Horizon `N = 100` at
`Ts = 0.05 s`, a five-second preview, `<MPCC>\C++\config.h`. QP solver HPIPM with BLASFEO.

**e) Obstacle corridor by 1-D dynamic programming, MATLAB only.** File:
`<MPCC>\Matlab\getNewBorders.m`. The track width is split into a lateral grid at each horizon
stage, occupied cells are marked from other cars, dynamic programming finds the cheapest
corridor, and that corridor replaces the track constraint. This is a principled upgrade of this
project's five fixed offsets.

**Traps.** Dependencies are fetched by a bash script, `<MPCC>\C++\install.sh`; plotting links
Python 2.7 through matplotlib-cpp (`find_package(PythonLibs 2.7)` in `<MPCC>\C++\CMakeLists.txt`).
The cost weights contouring error lightly, `q_c = 0.01`, and progress mildly, `q_vs = 0.2`, so
tuning is sensitive to scale. The model has no longitudinal slip, no load transfer, and only
the rear friction ellipse, because the original cars had no front brakes. MPCC's own closed-loop
checks drive the same bicycle model the controller predicts with: `<MPCC>\Matlab\SimTimeStep.m`
integrates `fx_bicycle` with `ode45`, and `<MPCC>\C++\Tests\model_integrator_test.cpp` tests the
integrator against itself. The repository therefore never measures model mismatch; Phase 6.2 does.

---

### 4.4 PacSim

**What it is.** A ROS 2 Formula Student Driverless simulator from Elbflorace, now community
maintained. Its own getting-started guide calls the default vehicle model "rather simple and just
meant to be a starting point" (`<PAC>\doc\getting_started.md`). Its value is everything around the
model.

**Vehicle model, for context only.** `<PAC>\src\VehicleModel\VehicleModelBicycle.cpp`: dynamic
bicycle, Magic Formula lateral force scaled by grip-map friction, axle loads from static weight plus
half the downforce each with no load transfer. Longitudinal force is `gear_ratio · torque / R` with
no slip curve, zeroed below 0.3 m/s unless torque is positive, and wheel speeds are back-computed
from ground speed, so they can never differ from it. A low-speed "stillstand" branch zeroes the
slip angles. Do not take this as the realistic plant.

**Algorithms worth taking.**

**a) The vehicle-model seam.** `<PAC>\include\VehicleModel\VehicleModelInterface.hpp` defines an
abstract model with setters for torques, steering and powered ground, getters for pose,
velocities, wheel speeds and wheel positions, and `forwardIntegrate(dt, friction per wheel)`. Two
real implementations would make that a real seam in this project, the kinematic baseline and the
dynamic plant.

**b) Sensor timing.** `<PAC>\include\sensorModels\sensorBase.hpp` and
`<PAC>\include\VehicleModel\deadTime.hpp`. A sensor samples when `t ≥ t_last + 1/rate`, stores the
sample in a queue, and releases it when `t ≥ t_sample + dead_time`. Configured in
`<PAC>\config\sensors.yaml`: 200 Hz with 5 ms dead time for IMU, steering, wheel speeds and
torques; GNSS at 20 Hz with 50 ms delay and 3 cm horizontal position noise.

**c) Mock cone perception.** `<PAC>\src\sensorModels\perceptionSensor.cpp`, configured in
`<PAC>\config\perception.yaml`:

- Transform cones into the sensor frame; keep only those within range 1 to 45 m and ±60°.
- Drop cones already knocked down.
- Missed detections: keep a cone with probability
  `p(d) = max(0.1, 0.99 − 0.01·(d − d_min) − 0.00012·(d − d_min)²)`, where `d_min` is the
  sensor's minimum range.
- Misclassification: beyond 15 m the colour becomes unknown. Otherwise the true class gets a
  distance-dependent probability from the same form with its own coefficients (floor 0.2, which
  the config notes equals guessing among five classes), the other classes share the remainder,
  and the reported class is sampled from those weights.
- Noise in Cartesian, angular and range terms, with a 3×3 covariance built in spherical
  coordinates and propagated through the spherical-to-Cartesian Jacobian.
- 10 Hz with 100 to 200 ms delay per LiDAR.

**d) Grip map.** `<PAC>\src\track\gripMap.cpp`, `<PAC>\tracks\gripMap.yaml`: friction per wheel
from the nearest map point, times a global scale.

**e) Competition logic.** `<PAC>\src\competitionLogic.cpp`, `<PAC>\include\competitionLogic.hpp`,
`<PAC>\config\mainConfig.yaml`: timekeeping gates and laps; a hit cone costs 2 s (0.2 s in
skidpad), off course 10 s, and an unsafe stop 10 s in trackdrive; DNF for an unsafe stop, more
than 8 s off course, a wrong turn direction or a timeout; per-discipline timeouts such as 25 s
for acceleration and 300 s for autocross.

**f) Deterministic clock.** `<PAC>\src\pacsim_main.cpp`: a 1 kHz fixed step, a real-time ratio,
a published simulation clock, and services that run the clock to an absolute or relative stop
time. That matches this project's fixed-tick discipline and suggests 1 kHz plant substeps once
wheel dynamics exist.

**g) Real Formula Student layouts.** `<PAC>\tracks\` holds FSG 2019, 2021, 2023 and 2024, FSE 2022
to 2024, FSS 2019 and two 2022 variants, FSCZ 2024, FSI 2024, FSO 2020, skidpad and acceleration,
as coloured cone lists.
A Python track editor is in `<PAC>\track_editor\`, and an FSSIM converter in
`<PAC>\scripts\convert_fssim_sdf_to_yaml.py`.

**Traps.** ROS 2 Iron on Ubuntu 22.04 per its README; `RESEARCH_NOTES.md` already records that
Jazzy compatibility is unverified. The track files are competition layouts; confirm their
provenance before redistributing them in a public build.

---

### 4.5 ForzaETH race_stack

**What it is.** ETH Zurich's complete stack for 1/10-scale F1TENTH head-to-head racing, described in
the Journal of Field Robotics 2024 paper. Much of its planning is ROS-bound Python. Several key
pieces are empty submodules in this download, see §1.

**Algorithms worth taking.**

**a) Model- and Acceleration-based Pursuit (MAP).** `<RS>\controller\map\src\MAP_Controller.py`,
`<RS>\controller\map\README.md`, paper arXiv 2209.04346.

Pure Pursuit computes a steering angle from geometry alone, which assumes the tires do not slip.
MAP keeps the same geometry but asks a different question:

```
L1   = clip(q_l1 + m_l1 · v,  max(t_clip_min, √2 · |lateral error|),  t_clip_max)
η    = angle between car heading and the vector to the L1 point
a_lat,desired = 2 · v² · sin η / L1
δ    = LUT(a_lat,desired, v)
```

The lookup table comes from simulating a dynamic single-track model with Pacejka tires to steady
state over a grid of steering angles and speeds, and recording the lateral acceleration reached:
`<RS>\system_identification\id_analyser\generate_lookup_table.py`, model in
`<RS>\system_identification\id_analyser\helpers\vehicle_dynamics.py`, lookup in
`<RS>\system_identification\steering_lookup\src\steering_lookup\lookup_steer_angle.py`. The effect
is that the controller asks for more steering when the tires understeer at high lateral
acceleration, and cannot ask for lateral acceleration the car does not have. Additional details
in the source: a speed lookahead to compensate actuation delay, lookup speed reduced with lateral
error, steering reduced when accelerating harder than 1 m/s² and increased when braking harder
than 1 m/s², a speed-dependent gain `clip(1 + v/10, 1, 1.25)`, and a 0.4 rad clamp on the change
per update. Several of these are empirical tuning for a 1/10-scale car.

This is the smallest, most effective change that keeps this project's controller working once
the plant becomes dynamic.

**b) System identification of tire parameters.** `<RS>\system_identification\id_analyser\analyse_tires.py`,
fitted models in `<RS>\system_identification\id_analyser\models\NUC2\NUC2_pacejka.txt`. Later,
for calibration against a real vehicle.

**c) TUM Graph-Based Local Trajectory Planner, vendored.**
`<RS>\planner\graph_based_planner\src\GraphBasedPlanner\`, license LGPL-3.0 in that folder, paper
Stahl et al. ITSC 2019. Used by TUM at over 200 km/h in Roborace.

- Offline, `graph_ltpl\offline_graph\src\`: a lattice of layers along the race line, nodes spaced
  laterally on each layer's normal, spline edges between layers, pruned for curvature, with an
  offline cost of race-line deviation and average and peak curvature. ForzaETH's scaled weights in
  `params\ltpl_config_offline.ini` are `w_raceline = 10000`, `w_curv_avg = 7500`,
  `w_curv_peak = 2500`.
- Online, `graph_ltpl\online_graph\src\`: prune edges blocked by objects and outside bounds,
  search for the cheapest path per action, and return an action set such as straight, follow,
  pass left and pass right. Each action gets its own velocity profile by forward-backward passes,
  `VpForwardBackward.py`, or an SQP, `VpSQP.py`.

This is the intended successor of this project's five-offset planner.

**d) Frenet optimal trajectories.** `<RS>\planner\frenet-planner\`, license Apache-2.0, C++, after
Werling et al. 2010. Samples lateral quintic and longitudinal quartic polynomials in the Frenet
frame, discards infeasible or colliding candidates, and picks by cost on lateral deviation,
velocity, acceleration and jerk. Its README reports about 7 ms average and 20 ms maximum with ten
obstacles. It is packaged for ROS 1 catkin, so only its algorithm core is portable.

**e) Behaviour state machine.** `<RS>\state_machine\src\states.py`: global tracking, trailing,
overtaking, follow-the-gap only. A useful shape for Phase 5's action selection.

**f) Spliner overtakes.** `<RS>\planner\spliner\README.md`: fit a spline around an obstacle apex at
an evasion distance, bounded by track margins. Its README states "License: TODO".

**g) Follow-the-gap, a reactive fallback.** `<RS>\controller\ftg\ftg.py`, `<RS>\controller\ftg\README.md`,
MIT. Pick the widest free gap in a range scan and steer to it, with a safety radius against corner
cutting. ForzaETH uses it for mapping laps and as an optional sector mode. For this project it is
the cheapest possible last resort: when no planned action is feasible, steer to the widest gap in
the corridor and brake. It needs only the corridor and the blocked regions, not a sensor.

**h) Tuning and evaluation tooling.** `<RS>\f110_utils\nodes\sector_tuner` slices the racing line
into sectors and scales each sector's speed, which is how a team dials in confidence corner by
corner; `<RS>\f110_utils\nodes\lap_analyser` reports lap and tracking statistics;
`<RS>\f110_utils\nodes\param_optimizer` runs Bayesian optimisation over MAP parameters and sector
scalers. `<RS>\f110_utils\nodes\obstacle_publisher` publishes a dummy moving obstacle on a chosen
line at chosen speeds, the role `--obstruct` plays here, extended to motion.

**i) Opponent detection and tracking.** `<RS>\perception\abd_tracker\README.md`: segment a range
scan into objects, fit rectangles, classify static against dynamic by position variance, predict
dynamic ones with a Kalman filter. Not for now, since nothing here detects anything, but it is the
reference if moving obstacles ever get a simulated observation model.

**Traps.** 1/10-scale parameters (3.54 kg, 0.307 m wheelbase) do not transfer to a full-size car.
The MPC controllers in `<RS>\controller\mpc\src\` are Python with acados. Sub-package license
statements are inconsistent, so treat anything without a clear license as study material only.

---

### 4.6 FaSTTUBe ft-fsd-path-planning

**What it is.** TU Berlin's Formula Student Driverless path planner, a standalone MIT-licensed
Python package with NumPy, SciPy and Numba. Its README credits it with contributing to a 2nd place
in Trackdrive at FS Czech 2023 and reports about 10 ms on a Jetson Xavier AGX. Stateless per call
except for the skidpad mission.

**Why it matters here.** It is the honest bridge to unknown-track mode. It turns a set of
possibly uncoloured, possibly one-sided cone detections into a local centreline, and it is
specifically robust to the inside of a corner being invisible.

**Pipeline, `<FT>\fsd_path_planning\full_pipeline\full_pipeline.py`.**

1. **Cone sorting**, `<FT>\fsd_path_planning\sorting_cones\`. Build a neighbour graph of cones
   within 6.5 m, up to 5 neighbours. Enumerate candidate ordered traces from the car up to 12
   cones, with angle thresholds of 40° directional and 65° absolute. Score each with the terms in
   `<FT>\fsd_path_planning\sorting_cones\trace_sorter\cost_function.py`: angle cost, number of
   cones, initial direction relative to the car, change of direction, wrong direction, and cones
   on either side.
2. **Cone matching**, `<FT>\fsd_path_planning\cone_matching\functional_cone_matching.py`. For each
   sorted cone, search the opposite side within 5 m and 50° along the local normal. Where no match
   exists, create a virtual cone at a minimum track width of 3 m.
3. **Path calculation**, `<FT>\fsd_path_planning\calculate_path\core_calculate_path.py`. Midpoints
   of matched pairs, spline fit with smoothing 0.2, fall back to the previous path when too few
   matches exist, reparameterise to 20 m and 40 points for a controller.
4. **Relocalisation** for skidpad and acceleration to a known layout,
   `<FT>\fsd_path_planning\relocalization\`.

Defaults live in `<FT>\fsd_path_planning\config.py`. Output rows are spline parameter, x, y and
curvature.

**Traps.** Numba compiles on first run, 30 to 60 s. The skidpad logic has been stateful since its
December 2023 rework, so one `PathPlanner` instance must live for a whole run. A C++ port must
reproduce behaviour on recorded cone sets before it replaces anything.

---

### 4.7 FS-FEUP autonomous-systems

**What it is.** FEUP Porto's full Formula Student Driverless pipeline in ROS 2 Humble: perception,
EKF state estimation, SLAM, planning, control, their own simulator InvictaSim, and a modular
vehicle dynamics library. **Licensed GPL-3.0.** Copying its code into this project would bind this
project's distribution to GPL-3.0. Study it; do not paste it.

**What is worth studying.**

**a) A genuinely modular vehicle library.** `<FEUP>\src\motion_lib\include\motion_lib\` provides
swappable models, each behind a base class and a name-to-factory map: aero; Thevenin battery;
brakes; delayed inverter; rigid-body and vehicle-dynamics load transfer; map-based motor; Ackermann
and parallel steering; first-order and PID steering motor; Pacejka MF 6.2 and a simpler combined-slip
tire; Salisbury and viscous differentials. This is the architecture the user described as
"modular so parts are plug and play", applied to the vehicle.

**b) Load transfer decomposed the way a race engineer would.**
`<FEUP>\src\motion_lib\src\load_transfer_model\vd_load_transfer_model.cpp`. Lateral transfer per
axle is the sum of unsprung, geometric through the roll centre, and elastic through the roll axis
split by roll stiffness distribution. Longitudinal transfer uses the same three parts about the pitch
centre. Inputs are the previous step's accelerations, so it stays explicit.

**c) Wheel spin with a lock guard.** `<FEUP>\src\invictasim\src\vehicle_model\FSFEUP03.cpp`. Per
wheel:

```
brake_sign = (2/π) · atan(10 · ω)
ω_next     = ω + (T_drive − Fx·R − rolling − T_brake · brake_sign) / I · dt
if braking and ω · ω_next ≤ 0:  ω = 0      (the wheel locks instead of reversing)
```

Front inertia is tire plus rim; rear adds motor and transmission reflected through the gear ratio.

**d) Velocity planning with a friction ellipse.** `<FEUP>\src\planning\src\planning\velocity_planning.cpp`:
Menger curvature, curvature speed cap, then forward acceleration and backward braking passes where
available longitudinal acceleration is `a_x,max · sqrt(1 − (a_y/a_y,max)²)`. Minimum curvature is
solved with OSQP in `<FEUP>\src\planning\src\planning\smoothing.cpp`.

**e) Evaluation tooling.** InvictaSim statistics for collisions, lap timing and tracking, and a
tuning evaluator, `<FEUP>\src\invictasim\src\statistics\`, `<FEUP>\src\invictasim\src\tuning\`.

**f) A second cone-to-path method.** `<FEUP>\src\planning\README.md`: Delaunay triangulation of the
visible cones with CGAL, midpoints of triangle edges filtered by length, a graph search with
recursive lookahead, cone colouring by cross-product side, B-spline smoothing with GSL, then the
OSQP minimum-curvature pass. It is the classic alternative to FaSTTUBe's trace sorting and the
natural cross-check for Phase 7.3, since both consume the same cone sets.

**g) Controllers and a PacSim adapter.** `<FEUP>\src\control\`: PID longitudinal, Pure Pursuit
lateral, and two acados-based MPCs, `bombated_mpc` and `mpczinho`, behind adapters in
`<FEUP>\src\control\include\adapter\pacsim.hpp` and `invictasim.hpp`. That a third team's
controller already runs against PacSim's interface is evidence that PacSim's vehicle-model and
sensor seams are practical, which supports taking that seam shape in Phases 0.2 and 7.

---

## 5. Licensing and dependency rules

This project's core is meant to stay independently licensable. The Qt Quick 3D application is
already GPLv3-or-commercial per ADR 0001, but that does not make it acceptable to pull copyleft
code into `fd_core`.

| Source | License | Rule for this project |
| --- | --- | --- |
| fastest-lap | MIT | May port code into `fd_core` with the MIT notice preserved in a `THIRD_PARTY_NOTICES.md` and in file headers |
| MPCC | Apache-2.0 | May port with the license text and any NOTICE preserved; mark modified files |
| PacSim | MIT | May port with notice |
| FaSTTUBe | MIT | May port with notice |
| race_stack root | MIT | Port only parts whose license is clearly MIT |
| race_stack `graph_based_planner` | LGPL-3.0 | Reimplement from the Stahl et al. paper; do not copy code into `fd_core` |
| race_stack `frenet-planner` | Apache-2.0 | May port with license and notice |
| race_stack `spliner`, `gb_optimizer` | README says "License: TODO" | Study only |
| TUM optimizer and its helper package | LGPL-3.0 for the optimizer; verify the helper package | Reimplement from the Heilmeier 2019 and Christ 2019 papers, or run unmodified as an external offline tool |
| FS-FEUP | GPL-3.0 | Study only; never copy code |
| Ipopt, CppAD | EPL-2.0 | Offline tools only, not linked into `fd_core` |
| CasADi | LGPL-3.0 | Offline tools only |
| OSQP | Apache-2.0 | Acceptable in a separate planning library outside `fd_core` |
| HPIPM, BLASFEO | BSD-2-Clause | Acceptable in a separate controller library outside `fd_core` |
| Eigen | MPL-2.0 | Acceptable outside `fd_core` if a later phase needs it |
| CGAL, GSL, used by FS-FEUP's planner | GPL-3.0 and GPL | Not used; a Delaunay midpoint planner, if ever wanted, is written without them |

**Rules that follow.**

- Algorithms and equations are not copyrightable; code is. Porting from a paper with a citation is
  always safe. Porting code requires the license to allow it and the notice to travel with it.
- `fd_core` keeps zero external dependencies. Solvers live in separate targets such as
  `fd_raceline` or `fd_mpcc`, behind plain-data interfaces, so the core still builds without them.
- Offline optimisers produce data artefacts with provenance, never runtime dependencies of the
  driving loop.
- Record each adopted source in a new decision record and in `THIRD_PARTY_NOTICES.md` at the time
  code is ported, not before.
- Prefer one QP solver. OSQP serves both the minimum-curvature line and, in a sparse formulation,
  MPCC. Add HPIPM only if a measured solve-time gap justifies a second dependency.

---

## 6. The combination map

No single project does everything the user wants. The strongest result comes from combining them
at clear seams:

**The plant: realistic and explicit.**
fastest-lap's load-sensitive combined-slip tire *structure* with the peak placed at the named slip
and a Magic Formula shape chosen for a realistic sliding residual, plus quasi-static load transfer
as a 4×4 solve with lagged horizontal forces, which is fastest-lap's equations and TUM's explicit
evaluation of the same model, plus fastest-lap's brakes, power-limited engine and viscous
differential, plus FS-FEUP's wheel-spin lock guard as a studied pattern, plus MPCC's dynamic
single-track equations as the verified reduced model, integrated at PacSim's 1 kHz substep inside
this project's 5 ms tick with wheel speed handled implicitly.

**The envelope: derived, not assumed.**
fastest-lap's numerical G-G diagram run against this project's own plant parameters, exported in
TUM's `ggv.csv` format, and cross-checked against FS-FEUP's friction-ellipse velocity planner.

**The line: minimum curvature, then minimum time.**
TUM's track preparation and minimum-curvature QP reimplemented in C++ with OSQP, followed by a
G-G-V velocity profile, then fastest-lap's minimum-time optimal control run offline for the same
car as the theoretical reference, with TUM's minimum-time formulation as the independent
cross-check.

**The local decision: a lattice with an action set.**
The TUM graph planner's offline lattice and online action set, reimplemented; Werling-style
polynomial edges from the Apache-licensed Frenet planner; a per-action G-G-V velocity profile so a
wider line can be faster; MPCC's MATLAB dynamic-programming corridor as the alternative when the
lattice is too coarse; race_stack's state machine for behaviour.

**The control: from pursuit to optimisation.**
race_stack's MAP controller as the robust baseline on the dynamic plant, then MPCC's contouring
control with friction-ellipse constraints as the flagship, always compared against MAP on the same
scenario.

**The world: observations instead of ground truth.**
PacSim's sensor timing and mock cone detections feeding FaSTTUBe's cone-to-path pipeline, judged by
PacSim's competition logic on its Formula Student layouts.

---

## 7. The plan, phase by phase

Every phase uses the same template: goal, what changes in this codebase, sources, how, tests and
evidence, what becomes visible, effort and risks. Estimates are focused hours with a range.

---

### Phase 0: Make room for realism

**Goal.** Remove the three blockers from §2 without changing any existing behaviour.

**0.1 Fast projection.**

- Changes: `project()` in `<PROJECT>\core\src\core.cpp`, plus a stateful hint where callers can
  supply one, such as `make_local_trajectory` in `<PROJECT>\core\src\trajectory.cpp`.
- Source pattern: MPCC's `porjectOnSpline` in `<MPCC>\C++\Spline\arc_length_spline.cpp` searches
  near the previous progress value rather than the whole track.
- How: add a windowed projection around a station hint with an exact full-scan fallback when the
  windowed result is not provably the global minimum. Keep the exhaustive function and its
  overflow-safe distance as the reference.
- Tests: every existing analytic projection test in `<PROJECT>\tests\trajectory_tests.cpp` passes
  unchanged, including the 1e200 overflow case; a new property test compares windowed and
  exhaustive projection on thousands of random poses; recorded runs reproduce byte for byte.
- Evidence target: `fd_core_behavior` well under 20 s; a blocked planning decision under 5 ms.
- Effort: 8 to 16 h. Risk: subtle tie-breaking differences at the start/finish seam.

**0.2 A vehicle-model seam.**

- Changes: new `<PROJECT>\core\include\fd\vehicle_model.hpp`; the current `integrate_bicycle`
  becomes the first implementation; `Simulation` in `<PROJECT>\adapters\simulation\src\simulation.cpp`
  calls the seam.
- Source pattern: `<PAC>\include\VehicleModel\VehicleModelInterface.hpp`.
- How: follow the project's own `codebase-design` rule that one adapter is a hypothetical seam
  and two make a real one. Introduce the interface in the same phase as the second model, Phase 1,
  but design it now so Phase 1 is additive. Keep plain data in and out; no inheritance is required
  if a tagged variant is clearer.
- Tests: all six current suites pass with the kinematic model selected.
- Effort: 6 to 12 h.

**0.3 Recording schema 2.**

- Changes: `<PROJECT>\adapters\recording\`; this is already the next milestone in `STATUS.md`.
- How: a versioned reader branch; schema 1 keeps loading. Add the selected trajectory, rejected
  alternatives and their reasons, which ADR 0004 says are missing. Reserve columns for body
  velocities, yaw rate and per-wheel speed, slip ratio, slip angle, vertical load and forces, so
  Phases 1 and 2 only fill them.
- Tests: the existing tampered-recording rejection suite extended to schema 2; old recordings in
  `<PROJECT>\out\runs\` still validate.
- Effort: 12 to 24 h.

**0.4 The reference-point decision.**

- Dynamic models are written at the centre of gravity. AGENTS.md fixes the state at the rear axle.
- Recommended: keep the rear-axle pose in the published `State` contract so display, recording and
  controllers keep their meaning, and convert at the model boundary using the model's
  centre-of-gravity offset. Record it as ADR 0005.
- Effort: 2 to 4 h plus the conversion tests in Phase 1.

**Phase 0 total: 28 to 56 h.**

---

### Phase 1: Tires and a dynamic single-track plant

**Goal.** A car whose motion comes from tire forces, which can understeer, oversteer and slide, and
which still drives the circuit with an adapted controller.

**1.1 The tire module.**

- New files: `<PROJECT>\core\include\fd\tire.hpp`, `<PROJECT>\core\src\tire.cpp`.
- Source: fastest-lap `Pacejka_simple_model` in `<FL>\src\core\tire\tire_pacejka.hpp`, MIT, ported
  to plain `double` without CppAD templates.
- Parameters: load-dependent peak friction and peak slip between two reference loads, and one
  shape factor per direction. Place the peak at the named slip by construction, `S = tan(π/2Q)`,
  and choose `Q` for a stated sliding residual, about 0.75 of peak by default. Record the F1 XML's
  `Q = 1.9` as "reproduces fastest-lap, residual 0.30", not as the default.
- Tests, written before the code as the `tdd` skill prescribes:
  - zero slip gives zero force;
  - forces are odd in slip;
  - the numerical peak lies at the named `κmax` and `λmax` within 1%, and a locked wheel keeps the
    documented fraction of peak force, per the trap in §4.1;
  - friction above `Fz2` follows the documented extrapolation and never falls below `mu_min`;
  - a heavier load gives more force but a lower friction coefficient;
  - pure longitudinal slip at the peak reduces available lateral force when lateral slip is added;
  - beyond the peak the force decreases, which is what later makes locking lose grip;
  - non-finite inputs are rejected.
- Effort: 10 to 18 h.

**1.2 The dynamic single-track plant.**

- New files: `<PROJECT>\core\include\fd\vehicle_dynamic_single_track.hpp` and its source.
- Sources: MPCC `getF` in `<MPCC>\C++\Model\model.cpp` for the equations; fastest-lap tires from 1.1
  instead of MPCC's fixed `D`; PacSim's 1 kHz step for integration cadence.
- States: rear-axle pose published, internally centre-of-gravity `vx`, `vy`, yaw rate, steering.
- **Low-speed singularity.** Slip angles and slip ratios divide by `vx`, and this simulation starts
  from rest, which none of the optimisers ever do. Two standard measures, both needed: a floor on
  the denominator, `max(|vx|, v_eps)` with `v_eps` around 0.5 m/s, and the AMZ blend between a
  kinematic and the dynamic model over a stated speed band, for example 1 to 3 m/s. MPCC's
  `vx_zero = 0.3` and PacSim's stillstand branch are the same idea. Implement the blend explicitly
  and test that it is continuous in speed.
- Integration: RK4 or semi-implicit Euler at 1 ms substeps inside the existing 5 ms tick, so the
  tick contract and recorded times do not change.
- Tests:
  - below the blend speed, motion matches the current kinematic model within a stated tolerance;
  - steady-state circle: yaw rate and lateral acceleration match the linear understeer-gradient
    prediction at small lateral acceleration;
  - at large steering the lateral acceleration saturates near `μ·g` instead of growing without bound;
  - removing tire grip mid-corner makes sideslip grow, rather than the car following geometry;
  - the plant never advances from anything other than `Simulation::step`.
- Effort: 20 to 36 h.

**1.3 Keep it driving: MAP steering.**

- Changes: `compute_control` in `<PROJECT>\core\src\core.cpp` gains a steering mode.
- Source: race_stack MAP, `<RS>\controller\map\src\MAP_Controller.py`, with the lookup-table method
  from `<RS>\system_identification\id_analyser\generate_lookup_table.py`.
- How: generate the lateral-acceleration-to-steering table from 1.2's own plant by steady-state
  simulation over steering and speed, store it with the vehicle parameters and a fingerprint of the
  model that produced it, and look it up at control time. Keep Pure Pursuit geometry for the target.
- Tests: the car completes two laps on the dynamic plant with bounded tracking error; with Pure
  Pursuit instead, error is measurably worse at high lateral acceleration, which documents why MAP
  exists; the table is regenerated and rejected if its fingerprint does not match the plant.
- Effort: 10 to 20 h.

**Visible.** Sideslip angle and yaw rate in telemetry; body velocity vector drawn at the car; a
per-axle slip-angle gauge; "understeer" and "oversteer" labels derived from front versus rear slip,
with the definition shown.

**Phase 1 total: 40 to 74 h.** Risk: tuning the kinematic-dynamic blend without numerical chatter.

---

### Phase 2: Wheels, brakes, load transfer and aero

**Goal.** The "wheel variables" car. Mass, centre-of-gravity height, brake bias, aero and power
become effective controls. Wheels can lock and spin.

**2.1 Four wheels with rotational dynamics.**

- Sources: fastest-lap axle and actuators, `<FL>\src\core\chassis\axle_car_3dof.hpp`,
  `<FL>\src\core\actuators\engine.hpp`, `<FL>\src\core\actuators\brake.h`; FS-FEUP's lock guard in
  `<FEUP>\src\invictasim\src\vehicle_model\FSFEUP03.cpp` studied, not copied.
- Per wheel: inertia, brake torque from pedal times maximum torque times front or rear bias, drive
  torque from `P_max/ω` capped by maximum force, viscous differential on the driven axle, tire
  torque `−Fx·R`.
- Locking: when brake torque would reverse the wheel within a substep, clamp wheel speed to zero.
- **Stiffness, with the number.** Linearised about free rolling, wheel speed relaxes with time
  constant `τ = I·vx / (Cκ·R²)`, where `Cκ = μ·Fz/κmax` is the slip stiffness. For the F1 tire,
  `Cκ ≈ 1.75 · 4000 / 0.11 ≈ 64 kN`, `R = 0.33 m`, `I ≈ 1 kg·m²`, so `τ ≈ vx / 7000 s`: about 3 ms
  at 20 m/s and 0.3 ms at 2 m/s. An explicit 1 ms substep is unstable below roughly 7 m/s. Treat
  the tire torque implicitly in the wheel update, one scalar Newton step per wheel per substep is
  enough, or give the tire a relaxation length so force lags slip. Shrinking the step alone does
  not fix a standing start. Record the chosen scheme with the run.
- Tests:
  - threshold braking stops shorter than locked braking on the same car;
  - excessive brake torque locks wheels, slip ratio reaches −1, and deceleration falls toward the
    sliding level;
  - front-biased lock keeps the car straight but unable to steer; rear-biased lock under braking
    makes yaw diverge, which is the classic brake-bias experiment;
  - wheelspin under full power at low speed, limited by the power cap at high speed;
  - energy: brake dissipation equals kinetic energy lost minus drag work within tolerance.
- Effort: 24 to 40 h.

**2.2 Quasi-static load transfer.**

- Sources: fastest-lap's four vertical equations in `<FL>\src\core\chassis\chassis_car_3dof.hpp`,
  `update` and `_roll_balance_equation_N`, and TUM's hand-written equivalent in
  `<TUM>\opt_mintime_traj\src\opt_mintime.py`; FS-FEUP's roll-centre decomposition as a later
  refinement.
- How: per substep, take the tire horizontal forces and aero from the previous substep, solve the
  4×4 linear system for the four loads (vertical balance, pitch and roll moment balance, the
  roll-balance closure with one coefficient), and clamp a lifted wheel at zero as fastest-lap's
  `smooth_pos` does. A test compares the result with TUM's closed form; they must agree to the
  size of the terms TUM drops.
- Tests: loads sum to `m·g` plus downforce; braking shifts load forward by `m·a·h/L`; cornering
  shifts load outward by `m·a_y·h/track`; roll distribution splits lateral transfer as configured.
- Effort: 8 to 16 h.

**2.3 Aerodynamics.**

- Sources: fastest-lap `get_aerodynamic_force` in `<FL>\src\core\chassis\chassis.hpp`; TUM's front
  and rear lift coefficients.
- Tests: top speed where drag equals available drive force; corner speed rises with speed on a
  high-downforce setup and not on a zero-downforce setup.
- Effort: 6 to 10 h.

**2.4 Honest setup controls.**

- Changes: split `Config` into vehicle parameters and planning or control configuration; bump the
  configuration schema; validate every parameter range before mutation, as today.
- Now allowed: mass, centre-of-gravity height, brake bias, cd and cl, power, differential stiffness.
- AGENTS.md's "a kinematic bicycle has no mass sensitivity" becomes "the kinematic baseline has no
  mass sensitivity; the dynamic plant does, through these equations", updated in the same commit
  that ships the evidence.
- Tests: every exposed control changes a measured outcome in a documented direction on a fixed
  scenario, or it is not exposed.
- Effort: 8 to 14 h.

**Visible.** Four tire "friction circles" around the car showing `(Fx, Fy)` against `μ·Fz`; vertical
load bars per wheel; wheel-speed versus ground-speed indicators with a red lock flash; brake-bias and
weight controls that visibly move braking points and lock behaviour.

**Phase 2 total: 46 to 80 h.** Risk: stiffness and low-speed behaviour; keep the kinematic plant
selectable as a baseline throughout.

---

### Phase 3: A vehicle-derived envelope and a real racing line

**Goal.** Replace fixed grip fractions with the car's own performance envelope, and replace the
centreline with a minimum-curvature racing line.

**3.1 G-G-V envelope from the plant.**

- Source: fastest-lap steady-state and G-G diagram, `<FL>\src\core\applications\steady_state.hpp`.
- How: for each speed on a grid, find the steady-state plant's maximum lateral acceleration, then
  the maximum and minimum longitudinal acceleration at several lateral levels. A steady state needs
  two inputs solved together, steering and throttle or brake, so a single bisection is not enough.
  Either a two-variable Newton on the plant's steady-state residual, or the method race_stack uses
  for its steering table: simulate the plant with speed held until yaw rate settles and sweep
  steering. The second needs no solver and reuses the plant as it is. Export in TUM's `ggv.csv`
  format with a model fingerprint. Because the envelope comes from the four-wheel plant, brake
  bias, centre-of-gravity height and downforce all move it, which is what makes the setup controls
  of Phase 2.4 visible in planning as well as in driving.
- Cross-check: if the fastest-lap Windows release is available, compute the G-G diagram for the same
  parameters there and compare shapes; document differences caused by model reductions.
- Tests: symmetric in lateral acceleration; braking capacity exceeds drive capacity; lateral capacity
  grows with speed only when downforce is non-zero.
- Effort: 14 to 24 h.

**3.2 G-G-V velocity profile.**

- Changes: `make_speed_plan` in `<PROJECT>\core\src\core.cpp` gains an envelope input; the fixed
  fractions remain as the baseline mode.
- Sources: TUM's velocity-profile method and FS-FEUP's friction-ellipse passes,
  `<FEUP>\src\planning\src\planning\velocity_planning.cpp`, reimplemented.
- How: curvature cap from `a_y,max(v)`, forward pass with `a_x,max(v, a_y)` minus drag and power limit,
  backward pass with braking capacity, the project's existing periodic convergence at the seam kept.
- Tests: the existing periodic-constraint tests generalise to the envelope; lower grip still moves
  braking earlier on the same scenario; planned speed never exceeds the envelope.
- Effort: 12 to 22 h.

**3.3 Track conditioning.**

- Source: `<TUM>\helper_funcs_glob\src\prep_track.py`.
- How: spline regression, uniform resampling, normal vectors, normal-crossing rejection. This is
  also the first half of the future "draw a track" feature.
- Tests: a noisy version of Foundry Circuit recovers its curvature within tolerance; a
  self-intersecting input is rejected.
- Effort: 10 to 18 h.

**3.4 Minimum-curvature racing line.**

- New target: `fd_raceline`, outside `fd_core`, with OSQP.
- Source: Heilmeier et al. 2019, as used by `<TUM>\main_globaltraj.py` through `opt_min_curv` and
  `iqp_handler`; FS-FEUP's OSQP formulation in `<FEUP>\src\planning\src\planning\smoothing.cpp`
  studied for structure.
- How: lateral shift per sample, linearised curvature, quadratic cost on curvature, box constraints
  from track width minus vehicle width and margin, iterate re-linearisation until the curvature
  error is below threshold. Convert TUM's right-pointing normal convention to this project's
  left-positive offsets explicitly.
- Output: a `Track`-compatible reference line with curvature and width to each boundary, so the
  existing controller, prediction and planner consume it unchanged.
- Tests: stays inside the corridor; curvature within the steering limit; estimated lap time with
  3.2's profile is lower than on the centreline for the same car; a straight track yields a straight line.
- Effort: 20 to 36 h.

**Visible.** Centreline and racing line drawn together; the speed plan chart switches to the
envelope; an estimated lap time for centreline versus racing line; the current point on a live G-G
diagram with the envelope drawn around it.

**Phase 3 total: 56 to 100 h.** Risk: OSQP build on MSVC is usually straightforward, but verify
before committing to it.

---

### Phase 4: The theoretical best lap

**Goal.** Answer "how did the F1 team calculate the theoretical maximum" with the same method, for
this project's car and track, and show the gap between theory and what the controller achieves.

**4.1 Offline minimum-time solution with fastest-lap.**

- Tool path: fastest-lap prebuilt Windows release, or an MSYS2 or WSL build. Needs the user's
  approval to download. Not linked into the MSVC build.
- How: write a vehicle XML that matches Phase 2's parameters as closely as the 3-DOF model allows,
  and a track XML for Foundry Circuit from Phase 3's conditioned reference; run the optimal lap;
  export positions, speeds, controls, per-wheel slip, loads and dissipated energy.
- New tool: `<PROJECT>\tools\optimal_lap\` with a script that runs the solve and writes a
  `reference-optimal` artefact carrying the fastest-lap version, solver status, mesh, tolerance and
  objective value, validated by a C++ reader like the recording contract.
- Setup sensitivities: export `d(lap time)/d(parameter)` for mass, centre-of-gravity height, brake
  bias, cl and power.
- Effort: 18 to 36 h, with high uncertainty from the build path.

**4.2 Independent cross-check with TUM's minimum-time formulation.**

- Tool path: `<TUM>` in an isolated Python environment matching its pinned requirements.
- How: same car within the double-track model's parameter set, same track; compare lap time, line and
  speed trace; explain differences from load-transfer treatment and combined slip.
- Effort: 10 to 20 h.

**4.3 In the application.**

- A translucent "theoretical car" replayed from the artefact along its own line, labelled as an
  offline optimum for this model, never as a live plant.
- Delta-time bar against the theoretical lap, per sector.
- A setup-sensitivity panel: "mass −10 kg would be worth X s", from 4.1.
- Tire energy bars per wheel, after fastest-lap's Suzuka analysis.

**Honesty rules.** The theoretical lap is optimal for the model, mesh, tolerance and solver status
stated with it. Reports always show solver status. A failed or non-converged solve is shown as such.

**Phase 4 total: 28 to 56 h.**

---

### Phase 5: A local planner that looks a few metres ahead

**Goal.** Re-plan from actual state every control tick over a spatial lattice around the racing line,
return an action set, and give each action its own feasible speed profile so a different line can be
genuinely faster. This is what the user means by "Tesla-like, a few metres at a time".

**5.1 Offline lattice.**

- Source: Stahl et al. ITSC 2019, as implemented in
  `<RS>\planner\graph_based_planner\src\GraphBasedPlanner\graph_ltpl\offline_graph\src\`, reimplemented
  because that code is LGPL-3.0.
- How: layers along the Phase 3 racing line, spaced more densely in corners; nodes spread across each
  layer's normal within the track; edges as polynomial splines, using the Werling polynomials from
  `<RS>\planner\frenet-planner\` which are Apache-2.0; prune edges exceeding steering curvature;
  precompute offline cost from race-line deviation and average and peak curvature.
- Effort: 20 to 34 h.

**5.2 Online search and action set.**

- Source: `graph_ltpl\online_graph\src\main_online_path_gen.py` concept; MPCC's
  `<MPCC>\Matlab\getNewBorders.m` corridor search as an alternative.
- How: from the node nearest the actual state, prune edges that intersect obstructions or leave the
  corridor, search the cheapest path per action within a planning horizon, and return straight or
  follow, pass left, pass right, and brake. Behaviour selection follows race_stack's
  `<RS>\state_machine\src\states.py` shape.
- Fallback: when no action is feasible, race_stack's follow-the-gap idea applied to the corridor
  and the blocked regions, steer to the widest gap and brake, replaces today's "hold the reference
  and stop". It is a few dozen lines and always returns something.
- Obstructions remain scenario ground truth until Phase 7. Moving obstacles are out of scope for
  Phases 0 to 7; when they come, the references are race_stack's tracker in
  `<RS>\perception\abd_tracker` for how observations become predicted motion, and MPCC's
  `getNewBorders.m` for how predicted motion becomes a corridor over the horizon.
- Effort: 18 to 30 h.

**5.3 Per-action velocity profile.**

- Source: `graph_ltpl\online_graph\src\VpForwardBackward.py` concept with Phase 3's envelope.
- How: every candidate path gets curvature-derived speed limits from its own geometry and forward
  and backward passes from the actual current speed, so comparing candidates compares time.
- This retires the limitation in ADR 0004 that offsets do not re-derive curvature limits. AGENTS.md's
  "the planner selects among feasible lines; it does not solve for a fastest one" is updated to state
  exactly what is optimised, over what horizon and lattice resolution.
- Effort: 12 to 20 h.

**Tests and evidence.** The two closed-loop scenarios from `<PROJECT>\tests\planner_tests.cpp` still
avoid and stop; with a clear track the chosen path converges to the racing line; on the avoidance
scenario, peak grip utilisation is lower than the five-offset planner's measured 0.995 for the same
clearance; planning time is measured against the control period on the reference host.

**Visible.** The lattice drawn faintly ahead of the car; each action's path in its own colour with its
estimated time; the chosen action highlighted and its reason and cost breakdown shown in the decision
panel.

**Phase 5 total: 50 to 84 h.** Risk: lattice resolution versus planning time; keep Phase 0's
projection work honest by measuring.

---

### Phase 6: MPCC, the flagship controller

**Goal.** A controller that optimises: maximise progress along the chosen path subject to track and
tire-friction constraints over a multi-second horizon. The green, yellow and red ribbon becomes an
optimised plan rather than a rollout of a fixed policy.

**6.1 Port.**

- New target: `fd_mpcc`, outside `fd_core`.
- Source: `<MPCC>\C++\` Apache-2.0: model and Jacobians, contouring and lag cost, track, friction-ellipse
  and slip-angle constraints, SQP loop with warm start and failure reset.
- Changes from MPCC: front and rear friction ellipses because this car brakes on all wheels; tire
  forces from Phase 1's model; progress along Phase 5's selected path.
- Solver: OSQP first, since Phase 3.4 already depends on it and one QP solver is the simpler
  architecture; a sparse multiple-shooting QP with 50 to 100 stages is well within its range.
  HPIPM with BLASFEO, BSD-2-Clause, only if a measured solve-time gap justifies a second dependency
  and only after its MSVC build is verified, since its kernels target Linux toolchains.
- Keep the front slip-angle constraint below the tire's peak, as §4.3 explains, and add a rear one.
  The linearised SQP is only trustworthy on the rising side of the curve.
- Effort: 36 to 60 h.

**6.2 Model mismatch as an experiment, not an accident.** The MPCC prediction model is a reduced
single-track model; the plant is Phase 2's four-wheel model. Record both and report prediction error
over the horizon. MPCC's own repository never measures this, its simulations and tests drive the
same bicycle model the controller predicts with, so the result is new relative to the source, not
only to this project.

**6.3 Head-to-head comparison.** Same scenarios, same car, same seed: MAP with the envelope speed
profile versus MPCC. Report lap time, maximum and RMS tracking error, peak per-wheel grip utilisation,
lock events, and solve time distribution against the 20 ms period. Keep MAP selectable forever as the
baseline.

**6.4 Visible.** MPCC's horizon drawn as the primary ribbon coloured by planned longitudinal
acceleration with the existing deadband; its friction-ellipse margin per wheel along the horizon;
solver status and solve time on screen.

**Phase 6 total: 60 to 110 h.** Risk: real-time budget on Windows; SQP robustness from a standing
start, which none of the source projects handle because their cars are launched at speed.

---

### Phase 7: Sensors, uncertainty and unknown tracks

**Goal.** Decisions made from delayed, noisy, incomplete observations instead of ground truth, with
the known map kept away from the controller. This is the honest version of "situational awareness".

**7.1 Sensor adapters.** Source: PacSim, MIT, `<PAC>\include\sensorModels\sensorBase.hpp`,
`<PAC>\include\VehicleModel\deadTime.hpp`, `<PAC>\config\sensors.yaml`. Rate, dead-time queue and
Gaussian noise for pose, speed, wheel speeds, IMU and steering. New adapter directory
`<PROJECT>\adapters\sensors\`. Seeds recorded. Effort: 16 to 28 h.

**7.2 Mock cone perception.** Source: `<PAC>\src\sensorModels\perceptionSensor.cpp`,
`<PAC>\config\perception.yaml`. Field of view, distance-dependent detection and classification
probability, spherical noise with propagated covariance, per-sensor delay. Requires a cone layout per
track; derive Foundry Circuit's from its boundaries. Labelled everywhere as simulated detections.
Effort: 16 to 28 h.

**7.3 Cone-to-path planning.** Source: FaSTTUBe, MIT, `<FT>\fsd_path_planning\`. Port sorting,
matching with virtual cones and path calculation to C++; first reproduce the Python outputs on fixed
cone sets saved as test fixtures. The local path feeds Phase 5 instead of the known racing line. FS-FEUP's Delaunay method, §4.7(f),
is the cross-check: implement it from the description, never from the GPL code, and run both on the
same fixtures; where they disagree, the fixture is a good test.
Effort: 30 to 50 h.

**7.4 No map leakage.** An adapter boundary test proves that in unknown-track mode the planner and
controller never receive the ground-truth track or cone map, only observations. Evaluation tools may
still read ground truth, clearly separated. Effort: 6 to 10 h.

**7.5 Competition evaluation and tracks.** Source: `<PAC>\src\competitionLogic.cpp` and
`<PAC>\tracks\*.yaml`. Cone hits, off-course, unsafe stop, lap and sector timing; import Formula
Student layouts as additional presets after checking their provenance. Effort: 14 to 24 h.

**Visible.** Detected cones versus true cones with missed and misclassified detections drawn
differently; detection covariance ellipses; the path the car believes versus the true track; penalties
in the timing panel.

**Phase 7 total: 82 to 140 h.** Risk: scope; ship 7.1 and 7.2 before 7.3.

---

### Phase 8: Making every new decision visible

This is continuous, not a separate milestone. The product definition is "make autonomous racing
decisions visible", so no phase is complete until its new quantity is on screen with its meaning.

| Phase | New quantity | Visual |
| --- | --- | --- |
| 1 | Sideslip, yaw rate, axle slip angles | Velocity vector at the car, slip gauges, understeer and oversteer label with definition |
| 2 | Per-wheel force, load, wheel speed, lock | Friction circle per tire, load bars, lock flash, brake-bias and weight effect |
| 3 | Envelope, racing line, lap estimate | Live G-G dot inside the envelope, racing line versus centreline |
| 4 | Theoretical lap, sensitivities, tire energy | Ghost car, delta bar, setup-sensitivity panel, energy bars |
| 5 | Lattice and actions | Faint lattice, action paths with times, chosen-action cost breakdown |
| 6 | Optimised horizon | MPCC ribbon, per-wheel margin along the horizon, solve time |
| 7 | Observations and belief | Detected versus true cones, covariance, believed path, penalties |

The same data must also land in recordings so replay can show it, extending schema 2 rather than
recomputing anything at replay time.

---

## 8. Selectable vehicles with honest physics

The original vision has several selectable cars. After Phase 2 each appearance can carry a distinct,
documented physics profile instead of sharing one:

| Profile | Parameter source | Scale | Notes |
| --- | --- | --- | --- |
| Formula One style | `<FL>\database\vehicles\f1\limebeer-2014-f1.xml` | 660 kg, 735 kW + 120 kW | Published academic parameters; keep original branding; re-choose the tire shape factor per §4.1(a) |
| Formula Student electric | `<PAC>\config\vehicleModel.yaml`, `<MPCC>\C++\Params\model.json` | 178 to 190 kg | Author-labelled "made up" in PacSim; treat as representative, not measured |
| Electric race car | `<TUM>\params\racecar.ini` | 1200 kg, 230 kW | TUM's example parameters; includes powertrain thermal options for later |
| Racing kart | `<FL>\database\vehicles\kart\roberto-lot-kart-2016.xml` | small, no aero | Published academic parameters |
| Road car | none audited | | Needs its own sourced parameters before it is offered |

Rules: each profile states its source and whether values are measured, fitted or illustrative; a
team-named file is never presented as team data; appearance and physics stay separable exactly as the
current two procedural appearances are.

---

## 9. Invariants that change, and when

AGENTS.md is the shared authority. These lines change only in the commit that ships the evidence:

| Current invariant | Changes to | When |
| --- | --- | --- |
| Rear-axle reference | Rear-axle pose stays the published contract; models may compute at the centre of gravity and convert at the boundary | Phase 0.4, ADR 0005 |
| A kinematic bicycle has no mass sensitivity | The kinematic baseline has none; the dynamic plant does, through listed equations | Phase 2.4 |
| The planner selects among feasible lines; it does not solve for a fastest one | The local planner minimises estimated time over a stated lattice, horizon and envelope | Phase 5.3 |
| Blocked regions are scenario ground truth | Still true in known-track mode; unknown-track mode plans from simulated observations only | Phase 7.4 |
| Fixed simulation ticks | Unchanged; plant substeps occur inside a tick and never change recorded tick times | Phase 1.2 |
| Green, yellow, red mean planned acceleration | Unchanged; the ribbon's source moves from rollout to optimised horizon | Phase 6.4 |

New invariants to add as their phases land:

- Offline optimisers produce artefacts with solver status and provenance; they never advance the plant.
- Every exposed vehicle parameter has a test proving its effect.
- Unknown-track mode never passes ground truth to planning or control.
- Ported code carries its license notice; copyleft sources are reimplemented from papers.

---

## 10. Verification strategy

The project's existing standard applies: report only what ran, test physics and failure cases, keep
evidence with dates and fingerprints.

**Oracles.** Where possible, a test compares against something independent:

- analytic limits: straight-line braking distance, steady circle under linear tire assumptions,
  load sums, energy balance;
- the kinematic baseline at low speed;
- fastest-lap's G-G diagram and optimal lap for the same parameters;
- TUM's minimum-time result as a second, independently formulated optimum;
- FaSTTUBe's Python outputs on saved cone fixtures before and after the C++ port.

**Regression.** Every recorded run in `<PROJECT>\out\runs\` that is not intentionally invalidated
keeps validating with `--check-recording`. Kinematic mode keeps reproducing its recorded telemetry
byte for byte until deliberately retired.

**Performance.** Planning, envelope lookup and MPCC solve times are measured and recorded per run
against the 20 ms control period. A claim of real-time capability requires the distribution, not a
single number.

**Claims unlocked by evidence.**

| Claim | Requires |
| --- | --- |
| "Wheels can lock" | Phase 2.1 lock tests and a recorded run showing slip ratio −1 |
| "Weight changes behaviour" | Phase 2.4 parameter-effect tests |
| "Racing line" | Phase 3.4 corridor, curvature and lap-estimate tests |
| "Theoretical best lap" | Phase 4 artefact with converged solver status |
| "Chooses the fastest local line" | Phase 5.3 with stated lattice, horizon and envelope |
| "Optimising controller" | Phase 6 comparison against MAP with solve-time distribution |
| "Plans from observations" | Phase 7.4 no-leakage test |

---

## 11. What not to take, and why

- **PacSim's vehicle model as the realistic plant.** No load transfer and no longitudinal tire slip;
  its own documentation calls it "rather simple and just meant to be a starting point".
- **FS-FEUP code of any kind.** GPL-3.0. Its architecture and equations are worth studying; its code
  would change this project's license obligations.
- **Linking Ipopt, CppAD or CasADi into `fd_core`.** Fortran and build-system cost on MSVC, and no
  need: optimal laps are offline references.
- **race_stack's Python acados MPCs directly.** Python in the control loop and a heavy toolchain; the
  C++ MPCC gives the same idea in this project's language.
- **1/10-scale parameters for a full-size car.** Tire and inertia values do not scale.
- **Neural perception from race_stack's TinyCenterSpeed** or any dataset-trained detector. Out of the
  current scope and against the brief's rule of keeping perception datasets off the initial path.
- **TUM's powertrain thermal and battery models now.** Real value for a Formula E-style profile, but
  only after the plant, envelope and line are solid.
- **A "mass slider" before Phase 2.** It would be decorative, which AGENTS.md forbids.
- **fastest-lap's F1 tire shape `Q = 1.9` as the default.** It keeps 0.30 of peak grip when
  sliding, so a lock would look about three times worse than on a real dry track. Use it only to
  reproduce fastest-lap's own results.
- **Treating the empty submodule folders as audited.** They were not.

---

## 12. Effort, order and risk

| Phase | Hours | Depends on | Ships |
| --- | ---: | --- | --- |
| 0 Make room | 28 to 56 | none | Fast projection, model seam design, schema 2, ADR 0005 |
| 1 Tires and dynamic plant | 40 to 74 | 0 | Sliding, understeering car that still drives |
| 2 Wheels, brakes, load, aero | 46 to 80 | 1 | Locking, brake bias, honest weight and aero controls |
| 3 Envelope and racing line | 56 to 100 | 2 | Car-derived G-G-V, minimum-curvature line |
| 4 Theoretical best lap | 28 to 56 | 3 | Offline optimum, ghost, setup sensitivities |
| 5 Local lattice planner | 50 to 84 | 3, 0.1 | Action sets with genuine time comparison |
| 6 MPCC | 60 to 110 | 1, 5 | Optimising controller with measured comparison |
| 7 Sensors and unknown tracks | 82 to 140 | 5 | Planning from observations, FS evaluation |
| 8 Visibility | included above | each phase | Every decision on screen and in replay |
| **Total** | **390 to 700** | | plus integration and documentation overhead, about 430 to 800 |

**Recommended order.** 0, 1, 2, 3, then 4 and 5 in either order, then 6, then 7. Phase 4 can move
earlier if a portfolio demonstration of "theoretical best lap" matters sooner, because it only needs
Phase 3's track and Phase 2's parameters.

**Option B, line first.** If a visibly faster car matters sooner than a realistic one: 0.1, then
3.3 and 3.4 with a hand-stated envelope on the current kinematic plant, then 3.2. A minimum-curvature
line and its speed profile are the single most visible racing improvement, they need no tires, and
the honest label is simply "kinematic plant, stated envelope". Phases 1 and 2 then replace the
stated envelope with a derived one without touching the line code. Roughly 50 to 90 h to the first
racing-line lap. The hours are estimates from two independent passes that broadly agreed; treat the
ranges as scale, not commitments.

**Top risks.**

- **Real-time budget.** Richer models and planners multiply cost. Phase 0.1 is not optional.
- **Stiff wheel dynamics.** Choose integration deliberately and record it; the numbers in Phase 2.1
  say an explicit 1 ms step fails below about 7 m/s.
- **Tire shape.** The sliding residual decides how a lock looks. Choose it per §4.1(a) and test it,
  or the model tells a story the physics does not.
- **Build friction on Windows.** OSQP, HPIPM and fastest-lap each need a verified build path before
  they are relied on.
- **Scope.** Each phase has a shippable subset; the product is served by finishing slices, not by
  starting all phases.
- **License drift.** Decide per source before porting, record in notices, prefer papers.

**Next session, concretely.**

1. Phase 0.1: windowed projection with exhaustive fallback and a property test; measure the core suite.
2. Phase 1.1: the tire module from fastest-lap's model, test-first, including the actual peak location.
3. Draft ADR 0005 on the reference point and the vehicle-model seam shape.

---

## 13. Audit method and limits

**Read.** For each project: README, license, build files, directory structure, and the source files
named in §4. Equations and parameters quoted in §4 were taken from the source and configuration
files named beside them. The cited papers themselves were not read for this audit.

**Two passes.** A first pass drafted the audit and plan on 2026-09-14 and re-checked every cited
path and number. An independent second pass the same day re-derived the physics claims from the
source and changed the plan where the first had reasoned loosely: fastest-lap's vertical loads are
a 4×4 linear solve given the tire forces, not a formulation a fixed-tick plant cannot use, and are
the same quasi-static model as TUM's; fastest-lap's tire shape factor sets both the peak location
and the sliding residual, 0.30 of peak at `Q = 1.9`, which decides how a lock looks; TUM's tire is
the standard Magic Formula with a Kamm-circle constraint, not a combined-slip model; wheel-speed
stiffness was quantified; MPCC's own tests never measure model mismatch; PacSim's track list and
FaSTTUBe's statefulness were corrected; race_stack's follow-the-gap, sector tuner, opponent tracker
and obstacle publisher, and FS-FEUP's Delaunay planner, MPCs and PacSim adapter were added. One QP
solver, OSQP, is now recommended across Phases 3 and 6, and a line-first ordering is offered.

**Not done.** None of the seven projects was built, installed or run. No numbers in §4 are
measurements by this project; they are the projects' own parameters and their authors' reported
figures. No code was copied into this repository.

**Not available.** Empty submodules listed in §1; TUM's `trajectory_planning_helpers` source;
fastest-lap's `lion-cpp`; FS-FEUP's acados, OSQP and GTSAM submodules.

**Checked by calculation.** fastest-lap's simple tire shape: the peak lies at normalised slip
`tan(π/2Q) · 2·atan(Q)/π`, which is 0.751 for `Q = 1.9`, 1.002 for `Q = 1.565` and 1.084 for
`Q = 1.5`; the value at normalised slip 1 is 0.965, 1.000 and 0.999; a locked wheel at `κ = −1`
with `κmax = 0.11` keeps 0.297, 0.712 and 0.776 of peak; the large-slip limit `sin(Qπ/2)` is 0.156,
0.631 and 0.707. MPCC's lateral shape, `C = 1.38` and `B = 10`, peaks at 0.217 rad and keeps 0.827.
The wheel time constant `I·vx/(Cκ·R²)` with the F1 numbers in Phase 2.1 is 2.9 ms at 20 m/s.

**Freshness.** The local copies are zip downloads without git metadata, so their upstream commit is
unknown. Upstream projects may have changed. Pin a commit when any code is actually ported.

---

## 14. Glossary

- **Slip ratio `κ`.** Difference between wheel surface speed and ground speed, divided by ground
  speed. Zero when rolling freely, −1 when the wheel is locked and the car still moving.
- **Slip angle `α` or `λ`.** Angle between where a tire points and where its contact patch moves.
- **Combined slip.** A tire asked to brake and corner at once can deliver less of each than of either
  alone.
- **Friction circle, friction ellipse, Kamm circle.** The limit on the vector sum of a tire's
  longitudinal and lateral force, proportional to its vertical load and friction.
- **Load sensitivity.** Tire friction coefficient falls as vertical load rises, so load transfer costs
  total grip.
- **Load transfer.** Shift of vertical load between wheels caused by acceleration acting at the centre
  of gravity above the ground.
- **G-G-V diagram.** The set of longitudinal and lateral accelerations a car can sustain, as a function
  of speed.
- **Minimum-curvature line.** The path through the track corridor with the least total squared
  curvature; close to fastest in corners, not identical to minimum time.
- **Minimum-time optimal control.** Choosing steering, throttle and brake over the whole lap to
  minimise lap time subject to the vehicle model and track limits.
- **Curvilinear abscissa, Frenet frame.** Describing position by distance along a reference line and
  lateral offset from it.
- **Collocation.** Turning a continuous optimal control problem into a finite nonlinear program by
  enforcing the dynamics at chosen points.
- **Contouring error, lag error.** MPCC's lateral and along-track errors from the reference point at the
  controller's progress variable.
- **SQP, QP.** Sequential quadratic programming solves a nonlinear problem as a series of quadratic
  programs.
- **Lattice planner.** A planner that precomputes a graph of possible paths through space and searches
  it online.
- **Action set.** The alternatives a local planner returns, such as follow, pass left, pass right,
  brake, each with its own feasibility and cost.
- **Dead time.** Delay between when a sensor measures and when its value becomes available.
- **Known-track and unknown-track mode.** Whether planning may read the true map or only what
  simulated observations reveal.
