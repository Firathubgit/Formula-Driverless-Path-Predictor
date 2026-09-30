# 0012: A car with four rotating wheels

Date: 2026-09-15. Status: accepted for TrackWayFastPlan Phase 2.1. As first built it was available
through the vehicle seam only, with recordings refusing it; decision 0013 records it as schema 5 and
offers it in both applications. MAP steering is still not available for it. Decision 0014 gives it
quasi-static load transfer; a centre of gravity height of zero keeps the car described here exactly.

Phase 2 is the "wheel variables" car. Its first step asks for four wheels with rotational dynamics:
per-wheel inertia, brake torque shared by a bias, power-limited drive torque, locking when the brake
would reverse a wheel, and an implicit wheel-speed update because an explicit one is unstable at low
speed. Its tests: threshold braking stops shorter than locked braking; excessive brake torque locks
the wheels at slip ratio -1 and the deceleration falls toward sliding; front-biased locking leaves
the car unable to steer and rear-biased locking makes its yaw diverge; wheelspin at low speed and the
power limit at high speed; and an energy balance. fastest-lap's axle, brake and engine models were
studied; nothing was copied.

## Decision

**A third `VehicleModel` alternative, `FourWheelCar`** (`core/vehicle.hpp`), with `PlantState` gaining
`wheel_speeds_radps` (front left, front right, rear left, rear right; never negative; zero for the
other models) and `Demand` gaining each wheel's slip ratio and combined normalised slip.

- **Wheels and tires.** Four wheels at their static loads, 1.5 m apart on each axle. Each gets its
  longitudinal and lateral force from the tire law's combined slip, `TireAtLoad::force(slip ratio,
  slip angle)`, scaled by road grip. Slip ratio is `(ωR - u)/max(|u|, 0.5 m/s)` with `u` the wheel's
  speed along its own heading; slip angle has the single-track form. There is no load transfer
  (Phase 2.2), aerodynamics or drag, and the default parameters are illustrative.
- **Actuation, an ideal pedal map.** The controller still commands an acceleration. Positive, it
  becomes wheel torque `m·a·R` shared by `drive_front_fraction` and equally across each open
  differential, capped at `max_drive_torque_nm` per wheel and then scaled so the power at the wheels'
  present speed stays within `max_drive_power_w`. Negative, it becomes brake torque shared by
  `brake_bias_front`, capped at `max_brake_torque_nm` per wheel. A wheel therefore locks or spins only
  when its torque exceeds what its tire can transmit. Because the torque also has to spin the wheels
  up or down, the car reaches only `m/(m + 4I/R²)` of the commanded acceleration while its tires grip.
- **Wheel update, implicit.** Each substep solves `I (ω' - ω) = h (drive - brake - Fx(ω') R)` for every
  wheel with the chassis held at the start of the substep. The brake opposes forward rotation and can
  hold a stopped wheel, so when even a stopped wheel would not turn forward the wheel is locked at zero;
  that is the plan's lock guard. Otherwise Newton's method with secant slopes finds the root, with a
  bracketed search as the fallback.
- **Chassis.** RK4 at the centre of gravity, as for the single-track model, with the same derivative
  blend to the kinematic bicycle between 1 and 3 m/s and exactly the kinematic bicycle below 1 m/s,
  where the wheels roll at their own forward speeds. During each substep every wheel's slip ratio at
  its new speed is held, so the chassis receives exactly the longitudinal force that turned the wheel.
- **Substep 2.5 ms.** A 5 ms step, one per plant tick, would halve the cost, but it made a second of
  wheelspin a hundred times less accurate against a 0.2 ms reference (5.5e-3 m/s against 5.7e-5).
- **An envelope query.** `envelope(model, state, config, acceleration)` answers only the envelope
  question, from slip angles, slip ratios or accelerations without evaluating a tire force, and is
  always exactly `demand`'s answer. The prediction asks it at both ends of every tick instead of
  computing a full demand; recorded decisions of the existing plants did not change.
- **Balance and the envelope.** Axle slip angles for the balance label (decision 0011) are measured
  at the axle centres; the car is inside its envelope while every wheel's combined slip is at most one.
- **Not offered at first.** `RecordingWriter` refused the car, because schema 4 had nowhere to put
  wheel speeds or its parameters, until decision 0013 added schema 5. `SteeringTable` refuses it,
  because the steady-turn search would also have to solve for the four wheel speeds.
- **Shared code.** The chassis conversion and the low-speed blend became helpers used by both
  dynamic models, and the tire law evaluates its curve once when the longitudinal and lateral curves
  have the same shape. Recordings of a kinematic scenario, a default dynamic run and a MAP run of the
  soft-front car made before and after are byte-identical apart from the source fingerprint.

The plan also lists a viscous differential. With static loads and one road grip the left and right
wheels of an axle are always loaded alike, so a coupling between them has no measurable effect yet,
and the plan's rule that every exposed parameter has a test proving its effect would fail. Each axle
has an open differential until load transfer arrives in Phase 2.2.

## Found by testing it

- **Splitting the wheels from the chassis cost momentum.** The first version held wheel speeds
  during the chassis step but let the chassis re-evaluate slip as its own speed changed, so wheel and
  chassis saw different forces. Acceleration came out 1.9% below Newton's law with wheel inertia, and
  a pull-away from rest differed from a 0.2 ms reference by 20 cm. Holding each wheel's slip ratio for
  the substep brought the acceleration within 1% and the pull-away within 0.07 mm.
- **Cost.** With a 1 ms substep a prediction took 10.3 ms and a blocked decision among six lines
  66 ms. Secant slopes in the wheel solve, the 2.5 ms substep, one curve evaluation per tire, one force
  evaluation per wheel in `demand` and the envelope query brought them to 3.4 to 4.6 ms and 19 to
  26 ms across runs on a loaded host: about the 20 ms control period for a blocked decision, and over
  it in some runs. One 5 ms tick of the car costs about 3.1 µs against the single-track model's 1.0.
- **Two expectations were wrong.** Undriven wheels need a small force to spin up with the car, so
  their slip ratio is small rather than zero. And 0.85 g of commanded braking is well short of the
  tires' threshold once wheel inertia absorbs part of the brake torque; the comparison uses 0.95 g.

## Verification

`tests/wheel_tests.cpp` (`fd_wheel_model`, 11 groups):
- a rolling start spins every wheel at speed over radius, a coasting car keeps its speed with its
  wheels rolling, and a steady turn follows the linear understeer gradient within 3% for the default
  and a soft-front car, which are labelled neutral and understeer;
- acceleration from 5 m/s is the wheel torque over mass plus wheel inertia within 1%;
- from 20 to 3 m/s threshold braking at 0.95 g took 21.98 m and never locked a wheel; locked
  braking at 2.5 g took 25.32 m;
- locked wheels have slip ratio exactly -1 and the car decelerates at 7.697 m/s², the tire law's
  force at slip ratio -1, against a peak of 9.625 m/s²;
- braking at 1.6 g in a turn at 16 m/s: 90% front bias locked only the front wheels, the yaw rate fell
  from 0.244 to -0.004 rad/s and the car ran straight; 10% front bias locked only the rear wheels and
  the car spun to 1.18 rad of sideslip;
- rear-wheel drive at full torque from 4 m/s spun the rear wheels to slip ratio 4.17 and accelerated
  at 3.58 m/s² against a rear peak of 4.81; with 30 kW at 25.6 m/s the acceleration was 1.394 m/s²
  against power over speed and equivalent mass of 1.401, with no wheel past its peak;
- braking energy: 121,722 J of chassis and wheel kinetic energy lost against 121,652 J of brake and
  tire slip work computed from the tire law at each state;
- against a 0.2 ms substep: a pull-away from rest, threshold braking, wheelspin and cornering within
  1.2 mm and 1e-3 m/s; locked braking in a turn within 5.0 mm and 3.9e-3 m/s, because lock onset falls
  on a substep boundary;
- a standing start and a hard stop keep speeds finite and wheels never backward, and a stopped car
  does not creep;
- fifteen invalid parameters, a backward or non-finite wheel speed, and a MAP table are rejected;
- the unchanged controller laps the car in 33.805 s with a maximum tracking error of 0.358 m and no
  tire past its peak.

`fd_vehicle_model` adds a check that the envelope query agrees with `demand` over 20,000 random
states of five cars, inside and outside the envelope.

## What this is not

It has no load transfer, so no weight shift under braking or cornering, no aerodynamics, drag, rolling
resistance, suspension, camber or tire relaxation, and no engine map, gearbox or clutch. The pedal map
is ideal, so there is no ABS or traction control: a wheel locks or spins exactly when asked to exceed
its tire. Parameters describe no real car.

## Consequences

- A blocked decision on this car costs about the control period. The simulation is on fixed ticks, so
  a slow decision delays the picture, not the physics, but further cost work is still worthwhile.
- Recording it needed schema 5 with the car's parameters and wheel speeds; decision 0013.
- MAP for it needs the wheel speeds in the steady-turn search.
- Phase 2.2 (load transfer) now has per-wheel loads to change, and gives a viscous differential and
  brake bias against weight shift something to show.
