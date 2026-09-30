# 0014: Quasi-static load transfer

Date: 2026-09-15. Status: accepted for TrackWayFastPlan Phase 2.2, with the first of Phase 2's visuals.
Decision 0015 adds downforce to the vertical balance and records the car as schema 7.

Decision 0012's car kept every wheel at its static load, so braking could not move weight onto the front
wheels, and brake bias, centre of gravity height and roll balance could have no effect. Phase 2.2 asks
for quasi-static load transfer: per substep, take the tire forces from the previous substep, solve the
four linear equations for the four wheel loads (vertical balance, pitch and roll moment balance, and a
roll-balance closure with one coefficient), and clamp a lifted wheel at zero as fastest-lap's
`smooth_pos` does. Its tests: loads sum to weight, braking moves `m·a·h/L` forward, cornering moves
`m·a_y·h/track` outward, roll distribution splits lateral transfer as configured, and the result agrees
with TUM's closed form to the size of the terms TUM drops. fastest-lap's `chassis_car_3dof` and TUM's
`opt_mintime` were studied through the plan's description; nothing was copied.

## Decision

**Two new `FourWheelCar` parameters** (`core/vehicle.hpp`):
- `cg_height_m`, default 0.35 m, from zero to half the narrower track. Zero keeps every wheel at its
  static load, which is exactly decision 0012's car. Above half the track a car would tip over on the
  reference surface before its tires slide, and nothing here models a rollover, so it is rejected.
- `roll_balance_front`, default 0.5, from 0 to 1: the front axle's share of the lateral load difference,
  fastest-lap's roll balance coefficient and TUM's `k_roll`. The default keeps the car neutral.

**The load equations** (`quasi_static_wheel_loads`). For a horizontal force `(Fx, Fy)` at the ground in the
body frame at the centre of gravity, the four loads satisfy weight, the pitch moment `Σ x·Fz = −h·Fx`, the
roll moment `Σ y·Fz = −h·Fy` about the centre of gravity, and the closure
`(Fz_fr − Fz_fl)(1 − D) + (Fz_rl − Fz_rr) D = 0`. The system is solved in closed form: each axle gets
`(W·l_other ∓ h·Fx)/L`, and the right-minus-left difference `2·h·Fy/(D·t_f + (1 − D)·t_r)` is split `D`
to the front and `1 − D` to the rear. A negative load is clamped at zero; the loads then exceed weight by
the amount clamped. The function is public so tests can check the equations directly.

**Loads are plant state.** `PlantState::wheel_loads_n` holds each wheel's load. A substep prepares every
tire at the load the state carries, holds it through the wheel update and the chassis RK4, and stores the
loads that balance the substep's horizontal tire force, RK4-weighted. That is the plan's
previous-substep scheme, and it makes the loads part of the state, like wheel speeds: a copied state
predicts identically, recordings carry them, and replay shows them without advancing anything.
Solving the loads to consistency within a substep instead was considered. It would keep them a function
of pose and velocities, but it needs a fixed-point iteration per substep, and a blocked decision already
costs about the control period.

**The force that moves load is the tires' force.** In the dynamic range it is the sum of the four tire
forces. Through the 1 to 3 m/s blend band it is blended toward the force the kinematic bicycle's rolling
motion needs, mass times its longitudinal acceleration and times speed times its own yaw rate, which is
also what the kinematic range below 1 m/s uses. The blend's pull back toward kinematic motion is a
numerical device, not a force, so it moves no load.

**Tires at any load.** `PreparedTire` validates a tire once and gives `TireAtLoad` at any load without
validating again; `TireAtLoad(tire, load)` is now built with it and gives bit-identical results. A lifted
wheel transmits no force; its tire is prepared at the lower reference load only for its peak slips, which
the tire law holds constant below that load. A lifted wheel puts the car outside its envelope.

**Demand and naming.** `Demand` gains each tire's longitudinal and lateral friction use, force over peak
force at its present load, which stays inside the unit circle. Axle peak slip angles are the mean of the
axle's two wheels at their loads. A car with load transfer names itself "four-wheel car with rotating
wheels and load transfer, rear axle reference"; at zero height it keeps decision 0012's name.

**Recording schema 6.** The `four_wheel_car` metadata gains `cg_height_m` and `roll_balance_front`, required
from schema 6 and refused in older metadata. Telemetry appends `front_left_wheel_load_n` to
`rear_right_wheel_load_n`. The reader requires loads that are never negative, zero for a car without
rotating wheels, the static loads for a car at zero height, and a sum equal to the car's weight unless a
wheel reads exactly zero, when the sum may only be larger. A schema 5 four-wheel car reads back at zero
height with its static loads; schema 1 to 4 have no loads.

**Headless.** `--cg-height` and `--roll-balance-front`, refused unless the plant is the four-wheel car and
validated before any output. `--check-recording` names both and counts samples with a wheel lifted, and
reports the largest front axle load above static.

**Desktop.** Each wheel tile in the TIRE SLIP card now shows the tire's force as a point in its friction
circle (forward force up, leftward force left, the rim at peak grip), the wheel's load in kN, and a load
bar whose full width is twice the load at rest, with a tick at the load at rest. A wheel carrying no load
reads LIFT in red. Replay shows the recorded loads. The plan places friction circles "around the car"; they
sit in the tiles, which are laid out as the wheels are on the car, so they stay readable at the minimum
window size. No setup control is added to the desktop yet: Phase 2.4 exposes each with a test proving its
effect, and the tests below are those proofs for the core.

## Found by building it

- **Brake bias became delicate, as it should.** With load transfer, braking at 0.95 g with the static bias
  of 0.5 locks the unloaded rear wheels and stops in 25.59 m. The bias matched to the braking load and the
  wheels' inertia, 0.617, locks nothing and stops in 21.98 m, the distance the car without load transfer
  managed at 0.5. Two Phase 2.1 tests assumed the static bias reaches threshold braking; they now use the
  matched bias.
- **10% front bias now locks the rear wheels under the planner's braking.** Decision 0013 found that it did
  not without load transfer. With the centre of gravity 0.35 m high, the planner's 5.39 m/s² of braking
  leaves each rear wheel 1751 N instead of 1962 N; the rear wheels locked at 7.99 s and the car spun to
  1.139 rad of sideslip.
- **The blend's relaxation is not a force.** The first version took the load force from the centre of
  gravity's acceleration. When the rear-locked recording fixture spun into the blend band sliding sideways
  at 13 m/s, the blend's pull back toward kinematic motion produced hundreds of m/s² of "acceleration",
  lifted both left wheels and put up to 8.6 kN on a right-hand one. Taking the tires' force, blended toward the
  rolling motion's, brought the largest front axle change in that run to 840 N.
- **Loads lagging a substep make hard transients first order.** Full-power wheelspin from 4 m/s against a
  0.05 ms reference differed by 1.35e-4 m/s at 0.1 ms, 4.0e-4 at 0.2 ms, 1.14e-3 at 0.5 ms, 2.27e-3 at
  1 ms and 5.43e-3 at the 2.5 ms default after one second. The implicit wheel update was already first
  order (about 1e-3 m/s at 1 ms without load transfer); the lag adds a larger constant in this, the
  harshest case. Against a 0.2 ms reference the other cases stay within 1e-3 m/s, except locked braking in
  a turn at 1.8e-3 m/s and 5.5 mm, whose lock onset falls on a substep boundary as before; the wheelspin
  case's tolerance is now 1e-2 m/s.
- **A spinning car does not measure convergence.** Hard braking into a turn at the static bias spun the car
  to 1.25 rad/s of yaw rate, where 3.3e-3 rad/s between substeps is the spin's sensitivity, not the
  integrator's error. The convergence case brakes at the matched bias instead.
- **Roll balance moves balance only near the limit.** The roadster tire loses friction with load, but its
  peak slip angle also falls with load, so in the linear range an axle's cornering stiffness barely changes
  when load moves across it. Near 7 m/s² the understeer angle went from −0.00065 rad at a roll balance of
  0.2 to +0.00053 at 0.5 and +0.00153 at 0.8.
- **An edited mass is now refused outright.** The recording test that edited the four-wheel car's mass
  expected plant reproduction to expose it; the recorded loads no longer sum to the new weight, so the
  reader refuses it. Reproduction is now shown with an edited yaw inertia.

## Verification

`tests/load_transfer_tests.cpp` (`fd_load_transfer`, 10 groups):
- over 5000 random cars and forces, 3865 unlifted load sets satisfy weight, pitch, roll and the closure
  within 1e-9 relative; 1135 with a lifted wheel never go negative or below weight;
- against TUM's closed form: identical within 9.1e-13 N with equal tracks and straight wheels; with the
  front wheels steered, differences up to 53.3 N, never more than the steered forces' pitch term TUM drops;
  with unequal tracks, up to 95.9 N against a bound of 187.5 N for the mean-track approximation;
- braking at a measured 4.699 m/s² put 4428.67 N on the front axle against 3922.66 + 506.01 N; driving at
  2.393 m/s² left 3664.99 N against 3922.66 − 257.67 N, both within 1% of the transfer;
- in a steady 3.64 m/s² turn the load moved outward was within 0.01 N of `m·a_y·h/track` at roll balances
  of 0.3, 0.5 and 0.8, and the front axle's share of the lateral difference equalled the roll balance
  within 1e-9;
- roll balance moved the understeer angle as above;
- the brake bias results above;
- a 0.75 m high car with all roll transfer on the front lifted its inner front wheel 0.295 s into a turn at
  12 m/s: no force from it, outside the envelope, loads above weight only then, the rear wheels equal;
- against a 0.2 ms substep: braking into a turn and threshold braking at the matched bias, turning in until
  a wheel lifts, and rear-wheel drive out of a corner, all within 1.3 mm, 8e-4 m/s, 6e-6 rad/s and 0.33 N;
- at zero height every wheel keeps exactly its static load braking, cornering, locked and at walking pace;
- seven invalid parameters, a negative or non-finite load in the state and a non-finite force are refused.

The Phase 2.1 suite now runs the car with load transfer: a locked car decelerates at 7.669 m/s², the tire
law at slip ratio −1 at the loads the state carries; wheelspin accelerates at 3.95 m/s² against the rear
tires' peak of 5.29 at their raised loads; braking energy balances to 0.03%; the unchanged controller laps
in 33.81 s with no tire past its peak. A prediction measured 2.9 to 3.2 ms and a blocked decision 18.0 to
19.0 ms on this host, within or below the range measured for decision 0012 (measurement only; this host
varies by up to 1.5× between runs).

`fd_recording_playback` round-trips both parameters and every wheel load, reproduces a rear-locked run to
9.77e-10 m, shows its front axle loaded up to 509 N above static, reads a schema 5 copy at zero height with
static loads (where reproduction exposes the missing transfer at 9.0e-6 m), and refuses a load on a
kinematic run, a negative load, loads that do not sum to weight, load transfer keys in schema 5 metadata,
a missing roll balance and an edited mass. `fd_recording_contract` records a run with
`--cg-height 0.42 --roll-balance-front 0.65`, checks both keys and the load columns, and refuses a height
for the single-track car and a height above half the track before any output. `fd_desktop_controls`
(301 checks) checks each tile's load, load bar and friction point against the plant, the four loads against
weight, a replay's recorded loads with the front axle loaded under rear braking, and no loads for the
kinematic bicycle; capture `docs/assets/four-wheel-loads.png` was inspected.

Headless runs on the preset, each validated with `--check-recording`, plant reproduction between 9.6e-10
and 9.95e-10 m for all:

| Run | Result | Samples with a wheel locked | With a wheel lifted | Invalid samples |
| --- | --- | ---: | ---: | ---: |
| `loads-flat-lap`, height 0 | 33.805 s; identical to `wheels-lap` | 0 | 0 | 0 of 6762 |
| `loads-lap`, default | 33.810 s; front axle up to 578 N above static | 0 | 0 | 0 of 6763 |
| `loads-bias-front-0.9`, 20 s | no lock | 0 | 0 | 19 of 4001 |
| `loads-bias-front-0.6`, 20 s | no lock | 0 | 0 | 0 of 4001 |
| `loads-bias-front-0.5`, 20 s | no lock | 0 | 0 | 0 of 4001 |
| `loads-bias-front-0.1`, 20 s | rear lock at 7.99 s, spun to 1.139 rad | 362 | 0 | 1062 of 4001 |
| `loads-bias-front-0`, 20 s | rear lock at 7.86 s, spun to 1.341 rad | 404 | 0 | 911 of 4001 |
| `loads-tall-roll-front`, 0.75 m, roll balance 1 | 33.805 s; inner front wheel lifted in corners | 0 | 2155 | 3219 of 6762 |

`loads-flat-lap` against `wheels-lap` (schema 5): every file byte-identical apart from metadata identity,
the two new vehicle keys, and the four appended load columns. A kinematic scenario, a default dynamic run
and a MAP run of the soft-front car (`loads-scenario-avoid`, `loads-pursuit-default`, `loads-map-soft`)
are byte-identical to their schema 4 recordings apart from metadata identity and the eight appended wheel
columns, all zero.

## What this is not

Quasi-static: no suspension, springs, dampers, pitch or roll inertia, roll centres or anti-dive geometry,
so load moves instantly (one substep late) rather than over the tenths of a second a real car takes.
No aerodynamic load (Phase 2.3). A lifted wheel is clamped, not solved as a three-wheeled car. The
parameters are illustrative. No setup control is exposed in the desktop, and the differential is still
open on both axles.

## Consequences

- A viscous differential now has an effect to prove: the inner wheel of a driven axle is unloaded in a
  corner.
- Phase 2.3's downforce adds to the vertical balance in the same function.
- Phase 2.4 can expose centre of gravity height, roll balance and brake bias with the tests above as their
  effect proofs.
