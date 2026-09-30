# 0024: Model predictive contouring control drives the chosen action

Date: 2026-09-19. Status: accepted for TrackWayFastPlan Phase 6.1, with a first measurement of Phase 6.2's plan error and
the solver status and solve time of Phase 6.4. Adds a second controller beside the policy of decisions 0003 and 0010;
the local planner of decisions 0022 and 0023 is unchanged and still chooses the action. Recording schema 12.

Phase 6.1 of the plan: a new target `fd_mpcc`, outside `fd_core`, porting MPCC's model, contouring and lag cost, track,
friction-ellipse and slip-angle constraints, and its SQP loop with warm start and failure reset; front and rear friction
ellipses, because this car brakes on all wheels; tire forces from Phase 1's model; progress along Phase 5's selected
path; OSQP first, since the racing line already depends on it. The plan's section 4.3 read MPCC in detail.

Liniger's MPCC (`<MPCC>\C++`, Apache-2.0) was read for structure: `Model\model.cpp` (`getF`, the Jacobians,
`discretizeModel`), `Cost\cost.cpp` (contouring and lag error, progress reward, heading and sideslip costs),
`Constraints\constraints.cpp` (linearised track half-spaces, the rear friction ellipse, the front slip angle),
`MPC\mpc.cpp` (`runMPC`: two SQP iterations, mixing 0.9, warm start by shifting the last solution, reset after repeated
failures) and `Params\*.json`. Nothing was copied; the implementation here is written against this project's plant and
seams.

## Decision

**A controller seam in `fd_core`** (`core/predictive_control.hpp`): `PredictiveController` plans the car's motion for a
`ControlRequest` (track, reference plan, configuration, plant, state, the planner's decision, blockages, the command
being held) and returns a `ControllerPlan`: a prediction on the fixed-tick grid ending `prediction_horizon_s` ahead
whose `first_control` is the command, a status, the optimisation steps taken, whether it started again, and the solve
time. `reset()` forgets what it carried between decisions. `fd_core` stays free of solvers; `fd_mpcc` implements the
seam with OSQP, and tests implement it with scripted controllers.

**The prediction model is the plant's own equations.** `SingleTrackModel` (`core/vehicle.hpp`) prepares the dynamic
single-track plant's derivative, axle demand and frame conversions once per decision; RK4 of its derivative at the
plant's substep is `advance()` exactly. `reduced_single_track()` gives MPCC a single-track model of any car with tires:
the dynamic car itself; the four-wheel car's mass, yaw inertia, centre of gravity, tires, drive split and blend speeds on
one track, without load transfer, aerodynamics, brake bias or wheel speeds. The kinematic bicycle has no tires to reduce
and is refused, so MPCC drives the dynamic and four-wheel cars only; `PredictiveController::check` lets the simulation
refuse a car the controller cannot drive before a run.

**MPCC** (`mpcc/mpcc.hpp`, `fd::Mpcc`):

1. **State and inputs.** The centre of gravity's pose and body-frame velocity, the progress point's reference station,
   the acceleration and steering being applied, and the progress point's rate; its inputs are the rates of the last
   three, bounded by 40 m/s^3, the configuration's steering rate and 40 m/s^2. Sixty stages of 0.05 s, the prediction
   horizon, each integrated with two RK4 steps.
2. **The path and its corridor.** The chosen action's path as offsets from the reference, its first offset before it and
   its last beyond it; the reference itself for a straight action. The centre of gravity is kept inside the corridor less
   the half width and 0.3 m, a linearised half-space across the reference at its projection; beside a blockage, only on
   the side the chosen path takes. Holding for a blockage is the policy's to drive, and MPCC refuses to be asked.
3. **Cost, per stage.** Contouring error 0.1 and lag error 100 per square metre against the path point at the progress
   station, the contouring weight ten times at the last stage; progress rewarded at 1 per metre per second; heading off
   the path's, sideslip beyond the kinematic bicycle's, yaw rate, the values and rates of the inputs, and slack.
4. **Constraints, softened by slack.** Each axle's slip angle within 0.7 of the angle of its peak grip, on the rising side
   of the tire curve as MPCC keeps its front tire; each axle's requested longitudinal and pure lateral force within 0.95
   of its friction ellipse, front and rear; and speed within the configuration's top speed, or for a car above it, a
   bound falling from its speed at half a g. Steering stays within its limit and speed above zero as hard bounds.
5. **Solving.** Two sequential quadratic programming steps per decision, each linearising the model by forward
   differences of the stage's integration, only in the six columns and two inputs the motion depends on, and taking 0.9
   of the step OSQP returns. Each step is penalised by its size, 1 per squared unit, a proximal term: it keeps every
   direction strictly convex, which OSQP needs to converge in few iterations on a problem this close to a linear
   programme. OSQP starts from the last solved programme's multipliers and stops at a tolerance of 1e-3.
6. **Where it starts.** From its last plan moved on by the time since, the plan being the prediction model driven by the
   planned inputs from the actual state; at a run's start, and after two decisions in a row without a solved step, from
   the policy's prediction of the chosen action.
7. **Its own checks.** A plan whose predicted slip angles pass a tire's peak above walking pace, or whose rear axle leaves
   the corridor less the half width, is flagged.

**The simulation** takes a predictive controller at construction. At every decision it asks the controller to drive the
planner's choice, and the plan drives only if it is solved, not flagged and clear of every blockage by the planners' own
check (`first_blockage_entered`), and the decision does not hold for a blockage. Then the plan replaces the chosen
option's prediction and its first command is held; otherwise the policy drives, exactly as without a controller, and
`controller_outcome()` says why. While MPCC drives, the simulation's target speed is the speed its plan reaches at its
next stage, and a speed above the reference plan's is not a validity violation, since MPCC does not drive to that plan;
the corridor and the tires still judge every sample.

**Recording, schema 12.** Metadata gains `predictive_controller`: `{"mode": "policy"}`, or `{"mode": "mpcc",
"settings": {...}}` with every option. `decisions.csv` gains `driven_by`, `controller_status` ("none" when no
predictive controller was asked), `controller_iterations`, `controller_restarted` and `controller_note`. Loading checks
that a policy run reports no controller; that a decision MPCC drove had a solved plan, was not holding, gave no reason
for the policy, and recorded a chosen prediction of exactly the controller's stages, one stage apart; that a decision the
policy drove under MPCC says why; and that a hold was never asked of it. Older runs read as driven by the policy.
`controller_plan_errors()` measures, from a recording alone, how far each plan's rear axle was from where the car went at
a given time ahead.

**Headless.** `--controller policy|mpcc`; MPCC needs OSQP's sources and is refused without them, for the kinematic
bicycle, and for the offline lattice. A run prints who drove and MPCC's solve-time distribution against the control period; `--check-recording`
prints the controller, who drove, and the plan error half a second, one, two and three seconds ahead.

**Desktop.** The steering row becomes CONTROL: Pursuit, MAP or MPCC, a fresh run each, refused while running and in
replay; like MAP, MPCC is offered for the cars with tires, and choosing the kinematic bicycle returns to the policy. Under MPCC the caption beneath the prediction gives who drives, the solver's status and steps, and the solve time
on this host; replay gives the recorded ones, without a time. MPCC's plan is the prediction, so it is the ribbon.

## Found by building it

- **A first-order solver on a problem this close to a linear programme is slow.** Progress is rewarded linearly and most
  variables are weighted lightly, as in MPCC, which HPIPM's interior point handles well. With OSQP and no more than a
  1e-6 regularisation, 444 of 1307 decisions had no solved step, the car left the corridor, and a decision took 267 ms at
  the median. Penalising each step's size by 1 per squared unit solved all but 5 and kept the car inside, at about 800
  OSQP iterations and 57 ms; starting OSQP from the last solved programme's multipliers brought that to 575 iterations
  and 48 ms, and a tolerance of 1e-3 rather than 1e-4 to 175 iterations and 16 ms, the lap unchanged. A penalty of 10
  damped each step so far that the car covered 212 m in 60 s.
- **The policy's prediction holds a point per control tick, not per plant tick.** The first guesses read it by index as
  though every plant tick were there, so they ran four times too fast, and every programme was infeasible against the
  progress point's trust region. The guess is now read at each stage's time.
- **The top speed must not follow the car.** Bounded by the configuration's top speed or the car's own speed, whichever
  was higher, plus 0.5 m/s, the bound rose each decision with the speed it allowed: the car reached 28 m/s against a
  20 m/s top speed and the lap looked 1.6 to 2.5 s faster than it was. The bound now falls from the car's speed toward the top
  speed at half a g when the car is above it, and a test holds the car to the top speed.
- **MPCC's advantage is carried from one decision to the next.** Started every decision from the policy's prediction,
  two damped steps stay near it: the dynamic car lapped in 33.7 s, the policy's 33.8 s, against 26.7 s when each decision
  starts from the last plan. The start from the policy is for the first decision and after failures only.
- **A model with tire slip cannot drive a car without it.** At first the kinematic bicycle was predicted as the default
  dynamic car. Its two laps under MPCC (`out/runs/mpcc-kinematic-laps-mpcc`, made before the refusal) took 26.68 and
  25.37 s, but 1291 of 10409 samples were invalid, asking up to 1.70 times the grip: MPCC steers further than geometry to
  build the slip angles its model needs, and a car without slip turns tighter than planned by exactly that. Rather than
  invent a tire model for a car that has none, MPCC refuses it, as MAP does.
- **The reduced model does not know load transfer, and the four-wheel car's tires show it.** In the test lap MPCC drove
  the four-wheel car inside the corridor, but 473 samples had a tire past its peak: the plan keeps each axle within 0.95
  of a friction ellipse at static load, and the wheels unloaded by braking and cornering saturate first. That is Phase
  6.2's model mismatch, now measured by the plan error.
- **On this track the friction ellipse binds before the slip angle.** Removing the slip-angle constraints left the
  dynamic car's lap unchanged; the lane pass is where their removal shows, as a plan passing a tire's peak that the
  simulation refuses.
- **A hard top speed made MPCC give up exactly when it was needed.** In the first sweep MPCC failed to solve 141 of
  1402 decisions around the corner bollard, 139 of them with the car above 20 m/s, and the policy, taking over, let the
  car reach 20.52 m/s. A car still accelerating above the top speed cannot shed the excess within one stage, since its
  acceleration changes at no more than 40 m/s^3, so as a hard bound the top speed made every programme infeasible until the
  car had slowed by other means. Softened like the other constraints, the same run has 4 decisions the policy drove for
  a refused plan, and a car at 22 m/s still accelerating gets a solved plan back at 20 m/s within a second.
- **A faster car meets a blockage with fewer ways past.** MPCC reaches the corner bollard faster than the policy would,
  and the planner, which rolls the policy forward from that speed to compare its actions, then found no clear pass for 18
  decisions and held; the policy braked, faster than its revised plan allowed, for 84 samples before MPCC drove again.
  The planner does not yet know which controller will drive its choice.
- **"Faster than the revised plan" is the policy's validity test.** MPCC corners faster than the reference plan by design,
  so under the old rule every corner it drove would have been an invalid sample; the rule now applies while the policy
  drives, and MPCC's samples are judged by the corridor and the tires.
- **The compact window has no row to spare.** MPCC joined the steering row, renamed CONTROL, rather than adding a row
  that would push the decision panel into the run metrics.

## Verification

`tests/vehicle_tests.cpp` adds two groups: integrating `SingleTrackModel`'s derivative with the plant's own RK4 substeps
and steering ramp reproduces `advance()` to 1e-12 for both dynamic cars and two commands, and its axle demand is
`demand()`'s slip angles, peaks and grip use; `reduced_single_track` keeps the four-wheel car's mass, inertia, centre of
gravity, tires, drive split and blend, and refuses the kinematic bicycle.

`tests/mpcc_tests.cpp` (`fd_mpcc`, 9 groups, about 150 s): on the straight a plan is solved, spans the prediction
horizon from the actual state and speeds up without passing the top speed; each decision starts from the last plan and
`reset()` restarts from the policy; options out of range, a stage off the plant's grid and the kinematic bicycle are
refused; a car at 22 m/s and still accelerating gets a solved plan back at the top speed within a second; in closed loop
the dynamic car laps without leaving the corridor, passing a tire's peak or the top speed; the four-wheel car laps
inside the corridor; with the lane blocked MPCC drives every decision and passes on the planner's side clear of the
blockage; through the simulation a lap has no invalid sample and MPCC's plan is the decision's prediction; and a
scripted controller shows the simulation refusing an unsolved plan, a flagged one, one entering a blockage and a hold,
with the car moving exactly as without a controller. Five mutations were run and reverted: dropping the simulation's
blockage check, dropping the blockage narrowing of the corridor, dropping the slip-angle constraints (caught by the lane
pass; the dynamic lap was unchanged), letting the top speed follow the car (28 m/s), and dropping the warm start (33.7 s
laps); each failed a group.

`fd_recording_playback` (17 groups) round-trips schema 12 with a scripted controller, decision for decision including
who drove, and refuses a policy run claiming a controller, an unsolved plan that drove, a policy decision under MPCC
without its reason, a hold the controller was asked about, and a plan of another shape than the recorded settings; a
schema 11 run reads as driven by the policy and schema 11 metadata naming a controller is refused; plan errors start at
zero and end at the horizon. `fd_recording_contract` records an MPCC run with the headless runner, validates it, and
refuses MPCC for the kinematic bicycle, for the offline lattice and an unknown controller. The UI verification checks
the CONTROL row, the refusals, the plan as the ribbon, the caption's status and solve time, a replayed MPCC fixture, and
captures the corner shown in `docs/assets/mpcc-corner.png`.

`scripts/build.ps1`: 20 of 20 CTest suites pass in 327.3 s, `fd_mpcc` taking 125.9 s. `scripts/build.ps1 -Desktop`: 21
of 21 in 418.7 s, the QML controls suite with 482 checks and no QML warning; neither build reports a compiler warning.

Runs with `fd_headless` into fresh `out/runs/mpcc3-*` directories on the preset; solving is median / 95th percentile /
longest, wall clock on this host:

| Scenario | Policy (Pure Pursuit) | MPCC | Other baseline |
| --- | --- | --- | --- |
| Dynamic car, two laps | 33.80 / 32.13 s, max 0.35 m off, peak grip 0.750, 0 invalid | 26.66 / 25.26 s, max 3.74 m off, peak grip 0.950, 0 invalid; drove 2596 of 2596, solving 18.4 / 63.2 / 236 ms | MAP 33.78 / 32.10 s, max 0.35 m off, peak grip 0.751, 0 invalid |
| Soft-front car, two laps | 33.94 / 32.26 s, max 0.71 m off, peak grip 0.761, 0 invalid | 26.68 / 25.29 s, max 3.74 m off, peak grip 0.950, 0 invalid; drove 2598 of 2598, solving 17.9 / 69.1 / 188 ms | MAP 33.85 / 32.18 s, max 0.33 m off, peak grip 0.747, 0 invalid |
| Four-wheel car, two laps | 34.05 / 32.34 s, max 0.34 m off, peak grip 0.826, 0 invalid | 27.07 / 25.35 s, max 3.70 m off, peak grip 1.000, 471 invalid; drove 2621 of 2621, solving 17.5 / 68.8 / 125 ms | 80% envelope plan 31.95 / 30.70 s, max 0.51 m off, peak grip 0.977, 0 invalid |
| Lane blocked, one lap | 33.74 s, max 2.80 m off, peak grip 0.755, 0 invalid | 26.71 s, max 3.74 m off, peak grip 0.950, 0 invalid; drove 1336 of 1336, solving 20.0 / 61.8 / 224 ms |  |
| Bollard entering the first corner, one lap | 33.73 s, max 3.26 m off, peak grip 0.848, 0 invalid | 27.99 s, max 3.81 m off, peak grip 0.951, 84 invalid; drove 1378 of 1400, solving 19.7 / 55.2 / 187 ms |  |
| Whole corridor blocked, 20 s | stops at 61.78 m | stops at 61.79 m; drove 75 of 1000 before the hold | |
| Grip 0.7 from 10 s, two laps | 37.83 / 37.12 s, max 0.40 m off, peak grip 0.926, 86 invalid | 28.27 / 27.25 s, max 3.83 m off, peak grip 0.953, 56 invalid; drove 2762 of 2776, solving 20.5 / 148.0 / 259 ms |  |

Every run passes `--check-recording`. Plan error one second ahead, median and 95th percentile: 0.008 and 0.29 m for the
dynamic car, 0.007 and 0.27 m for the soft-front car, 0.059 and 0.68 m for the four-wheel car; three seconds ahead 0.45
and 3.62, 0.46 and 3.70, 0.57 and 3.43 m. The four-wheel car's 471 invalid samples are all tires past their peak in its
first three seconds, pulling away from rest: its load moves rearward and the front wheels, which the reduced model loads
statically, spin. At the corner bollard the policy drove 18 holds and 4 refused plans, and its braking in the hold is
every invalid sample; after the grip change it drove 14 decisions MPCC's plan could not. The kinematic bicycle's run
before the refusal is `out/runs/mpcc-kinematic-laps-mpcc`. With this build 255 of the 256 recordings in `out/runs` pass
`--check-recording` (schemas 1 to 12, 40 of them schema 12); the other, `mpcc-dynamic-laps-mpcc` from the first sweep,
was cut off while it was written and is refused for its missing `events.csv`. Source fingerprint
`9968a3ef3572882a58d3db22d7810016e2d98b8af170d81ef78048910fabfa12`. Machine-readable evidence is in
`docs/evidence/mpcc.json`.

## What this is not

Not a real-time controller: a decision's solving is measured as a distribution on one host and its median sits near the
20 ms control period with a long tail; nothing bounds it, and the simulation waits for it. Not a guarantee: the plan is
checked at its stage points like every prediction, the constraints are softened, and a plan that fails a check simply
leaves the policy to drive. Not a time-optimal lap: the horizon is three seconds, the progress weight trades against
contouring, and the chosen action's path and the corridor it leaves open bound the search. Not the plant: the four-wheel
car is predicted without load transfer, aerodynamics or wheel speeds, which is what the plan error measures. Holding for
a blockage stays with the policy.

## Consequences

- Phase 6.2 has its measurement: `controller_plan_errors()` over recordings of each car.
- Phase 6.3's head-to-head can run the same scenarios with `--controller mpcc` against the policy baselines.
- Phase 6.4's per-wheel friction-ellipse margin along the horizon remains; the ribbon and the solver status are done.
- A persistent OSQP workspace, updating values instead of setting up each step, would cut the setup time from each step.
