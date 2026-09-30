# 0008: A dynamic single-track plant behind the vehicle seam

Date: 2026-09-15. Status: accepted for TrackWayFastPlan Phase 1.2, amended the same day by decision
0009: tires prepared once per advance, 2.5 ms substeps, a blended derivative in place of the state
blend described below, recording schema 3 and selection in both applications.

Decision 0005 designed the vehicle seam and deferred building it until a second model existed.
Decision 0006 provided a tested tire force law with nothing to use it. This decision adds the
second model and builds the seam in the same change.

## Decision

`core/vehicle.*` holds the seam as designed in decision 0005: `VehicleModel` is a
`std::variant<KinematicBicycle, DynamicSingleTrack>`, `PlantState` is the published rear-axle
`State` plus lateral velocity and yaw rate, and the pure functions `advance`, `demand`,
`plant_state_from` and `rear_sideslip_rad` are the only way callers move or question a plant.
`Simulation`, `make_local_trajectory` and `choose_local_action` now go through it. Their previous
signatures remain and select the kinematic bicycle, so no caller changed. Two refinements of the
0005 sketch: `demand` takes the longitudinal acceleration the caller attributes to the plant
rather than a command, because the prediction measures it over a step while diagnostics use the
command; and the published pose is the whole `pose` field of `PlantState`, not a separate function.

**The kinematic adapter is `integrate_bicycle`, unchanged.** Its demand keeps the previous
expressions exactly. A test compares 20,000 random steps and demands bit for bit, and four
scenarios rerun through the seam reproduce every file of their previous recordings byte for byte,
including all decisions and predicted trajectories.

**The dynamic model** integrates the centre of gravity in its body frame:

```
m (dvx/dt - vy r) = Fx_rear + Fx_front cos(d) - Fy_front sin(d)
m (dvy/dt + vx r) = Fy_rear + Fx_front sin(d) + Fy_front cos(d)
Iz dr/dt         = lf (Fx_front sin(d) + Fy_front cos(d)) - lr Fy_rear
```

- Slip angles are `d - atan2(vy + lf r, vx)` at the front and `-atan2(vy - lr r, vx)` at the rear,
  with `vx` floored at 0.5 m/s in the denominator. Each axle's lateral force is twice
  `tire_force(0, slip angle, static axle load / 2)`.
- Longitudinal force is the commanded acceleration times mass. Driving force is split by
  `drive_front_fraction`, braking in proportion to static load. Each axle's longitudinal force is
  capped at its peak friction, and its lateral force is scaled by `sqrt(1 - (Fx/Fx_max)^2)` so the
  pair stays inside a friction ellipse. This is a quasi-steady stand-in for combined slip until
  wheel speeds exist (Phase 2); `tire_force`'s own combined-slip law needs a slip ratio this model
  does not have.
- `Config::grip_mu` scales tire friction, not the slip at which it peaks, so the grip control
  changes what the dynamic car can do, not only its plan.
- Integration is classical RK4 on substeps inside the 5 ms plant tick (1 ms as built, 2.5 ms since
  decision 0009 with a convergence test), with the steering
  actuator ramped across each substep. At the default car's cornering stiffness the fastest
  lateral mode is well inside RK4's stability region at that step.
- Below 1 m/s the model is exactly the kinematic bicycle, integrated per substep, which also
  handles stopping. Between 1 and 3 m/s the model blends toward the kinematic bicycle. As built, the
  lateral state was pulled toward the kinematic values after each substep; decision 0009 replaced
  that with a blended derivative after a convergence test showed the pull depended on the step. A
  test sweeps the band in 1 mm/s steps for jumps.
- `demand` reports the body lateral acceleration from tire forces, grip use as the larger axle's
  friction-ellipse use, and "within envelope" as both slip angles within their peak. The validity
  message changes accordingly: a dynamic car that leaves the envelope is sliding, not unmodeled.

Default parameters are illustrative: 800 kg, yaw inertia 1352 kg m^2, centre of gravity midway on
the 2.6 m wheelbase, and a "roadster" tire with peak friction about 1.0 at 2000 N falling with load
and a peak slip angle of 0.122 rad, so `grip_mu` keeps its meaning on the reference surface.

## Found by running it

The first closed-loop lap with rear-wheel drive spun out. Leaving the first corner at 12.16 s, the
unchanged controller requested its full 0.55·μg acceleration while still turning. With all driving
force on a rear axle carrying half the weight and no load transfer, that axle can deliver at most
μ·Fz_rear/m ≈ 4.8 m/s², less than the 5.39 m/s² requested. Its friction ellipse went entirely to
traction, lateral grip at the rear vanished, and sideslip grew from 0.05 to 0.84 rad within a
second. That is power-on oversteer, correctly modeled. The kinematic planner's longitudinal budget
assumes the whole car's friction, which holds only when driving force is shared in proportion to
load.

The default is therefore `drive_front_fraction = 0.5`, equal to the default car's static front
load. Rear-wheel drive remains selectable, and a test keeps the finding: settled in a corner at
11 m/s and then given the full budget, the rear-wheel-drive car reaches -1.39 rad of rear sideslip
while the load-shared car stays at -0.05 rad. A vehicle-derived performance envelope (Phase 3)
is the right way to let a rear-wheel-drive car be driven at its own limit.

## Verification

`tests/vehicle_tests.cpp` (`fd_vehicle_model`, 13 groups) checks, against oracles independent of
the integrator:
- the kinematic adapter bit for bit;
- Newton's law in a straight line, with each axle's grip use;
- equality with the kinematic bicycle below the blend speed;
- steady yaw rate within 2% of the linear understeer-gradient prediction built from cornering
  stiffness measured on the tire law, for an understeering and an oversteering car;
- lateral acceleration capped at peak friction while the kinematic formula claims more than twice
  as much;
- lower road grip capping lateral acceleration and increasing sideslip;
- the rear-wheel-drive spin above;
- braking to rest without reversing;
- continuity across the blend band;
- published rear-axle motion consistent with the published speed, lateral velocity and yaw rate;
- rejection of invalid vehicles, states and commands;
- the prediction and the planner rolling the same plant forward;
- a closed-loop lap.

On the preset with the default configuration and unchanged controller, the dynamic car completed
the lap in 33.795 s against the kinematic 33.520 s. Its maximum tracking error was 0.350 m against
0.267 m, maximum rear sideslip 0.042 rad and maximum axle grip use 0.75, and no tick passed a slip
peak.

## What this is not

It has no load transfer, aerodynamics, drag, rolling resistance, wheel-speed dynamics, locking,
drivetrain or brake model, and its parameters describe no real car. Its combined slip is a friction
ellipse, not the tire's own law. It says nothing about weight effects beyond mass and yaw inertia
in these equations, and neither is an exposed control.

## Consequences

- Predicting with it was expensive (resolved in decision 0009: 1.35 ms per prediction). The
  closed-loop lap test took 9.0 s of wall time on the reference host against 0.7 s for the
  kinematic lap, about 5 ms per control decision with a clear track. Five candidates would exceed
  the 20 ms control period. Before either application offers the plant, the per-call tire
  validation in the substep loop and the substep length should be measured and reduced.
- `RecordingWriter` refused a dynamic-plant simulation until schema 3 (decision 0009): schema 2
  has nowhere to put sideslip, yaw rate or tire use, and a recording that silently omits them
  would misdescribe the run. Schema 3 adds them.
- AGENTS.md's invariant about mass sensitivity now names the kinematic bicycle explicitly, since a
  second plant exists that has mass.
