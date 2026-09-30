# 0010: MAP steering from the plant's own steady-state table

Date: 2026-09-15. Status: accepted for TrackWayFastPlan Phase 1.3.

The plan's Phase 1.3 gives `compute_control` a steering mode: keep Pure Pursuit's target, convert
the demanded turn into steering with a table generated from the dynamic plant by steady-state
simulation, store the table with the parameters and a fingerprint of the plant that produced it,
and reject a table whose fingerprint does not match. Its title, "keep it driving", assumed Pure
Pursuit would struggle on the dynamic plant. Decision 0008 already showed the default dynamic car
lapping under Pure Pursuit, so this decision measures where MAP actually changes anything.

## Decision

**`core/steering.hpp` holds the steering table.** `SteeringTable(model, config)` generates it,
`steering_for(curvature, speed)` looks it up, and `generated_for(model, config)` and
`fingerprint()` identify it. It depends only on the vehicle seam.

- **A row per speed.** Rows start where the plant is exactly the kinematic bicycle (the dynamic
  car's lower blend speed, 1 m/s by default; 1 m/s for the kinematic bicycle), then every 2 m/s up
  to 2 m/s above the configured top speed. Each row sweeps steering upward in 0.01 rad steps.
- **A steady turn is a fixed point of the simulated plant.** At a held speed and steering angle,
  the generator solves with Newton's method for the lateral velocity, yaw rate and longitudinal
  command that one control period of `advance()` returns unchanged. The residual is the real
  integrator, so the turn is the one the simulated car holds, including its substeps.
- **Only turns the car can hold are kept.** A turn is accepted when the period map of lateral
  velocity and yaw rate has both eigenvalues inside the unit circle, the plant's own `demand()`
  says both tires are inside their envelope, and the turn is tighter than the previous one. The
  row ends at the first angle that fails, with the boundary refined by seven halvings. Asking for
  a sharper turn than the last one returns that turn's steering.
- **Lookup.** The table is indexed by path curvature, with lateral acceleration recorded beside it
  (`a = v²κ`). Curvature keeps the lookup defined down to rest. Rows are interpolated linearly in
  speed squared, which is exact for a car whose steering grows linearly with lateral acceleration.
  Below the lowest row the geometric law `atan(Lκ)` applies, which is exact there; above the
  highest row the highest row is used.
- **Fingerprint.** 64-bit FNV-1a over the generator version and every configuration and vehicle
  parameter in exact round-trip text, printed as 16 hexadecimal digits. `generated_for` compares
  that text exactly. It is an identity check against mismatch, not a cryptographic digest.

**The controller.** `compute_control` takes an optional table after the intent. Without one it is
Pure Pursuit, bit for bit as before. With one it computes the same target and the same arc,
curvature `2y/d²`, and requests `steering_for(curvature, speed)`. `ControlResult` now also reports
`geometric_steering_rad`, Pure Pursuit's steering for that target, so the difference is visible.
`make_local_trajectory` and `choose_local_action` pass the table to every predicted control and
reject one generated for a different model or configuration. `Simulation` has a four-argument
constructor taking a `SteeringMode`; under MAP it generates the table and regenerates it with the
plan when a grip change commits, before anything is replaced.

**What was taken from race_stack's MAP, and what was not.** Taken: the idea of converting the
pursuit arc into a lateral-acceleration demand and looking up the steering a model needs for it.
Not taken: its lookahead formula (this project's lookahead is unchanged, as the plan says), and
its empirical extras for a 1/10-scale car, namely the speed lookahead for actuation delay, the
lookup speed reduced with lateral error, the steering changes when accelerating or braking harder
than 1 m/s², the gain `clip(1 + v/10, 1, 1.25)` and the 0.4 rad change clamp. Its table is
simulated to steady state; this one solves for the same fixed points directly (below).

**A named understeering car.** `DynamicSingleTrack::soft_front()` is the default car with front
tires whose grip peaks at 0.2 rad of slip instead of 0.122. The default car has equal axle loads
and equal tires, so it is nearly neutral and its steady steering is almost geometry; MAP needs a
car that is not to show what it does.

**Recording schema 4.** Metadata gains `"steering": {"law": "pure_pursuit"}` or
`{"law": "map", "table_fingerprint": "…"}`, the fingerprint of the table generated for the initial
configuration. Telemetry appends `geometric_steering_rad`. The reader requires a known law, a
fingerprint exactly for MAP, 16 lowercase hexadecimal digits, and under Pure Pursuit a geometric
steering equal to the requested steering. Schema 3, 2 and 1 load as Pure Pursuit, with the
requested steering as geometric; a steering line in older metadata is refused.
`recorded_steering_table_matches` reports whether this build derives the recorded fingerprint from
the recorded plant and initial configuration. Like the planner and plant reproductions it is
reported, not enforced.

**Selection.** Headless: `--steering pure-pursuit|map` and `--car default|soft-front`, both
refused before any output unless the plant is dynamic. Desktop: a Soft front vehicle model beside
Kinematic and Dynamic, a Pure Pursuit / MAP choice under the local prediction offered only for a
model with tires and only while paused and live, and a MAP CORRECTION readout: MAP's steering minus
Pure Pursuit's, both within the steering limit. Choosing the kinematic bicycle returns to Pure
Pursuit. Replay states the recorded law and shows the recorded correction.

## Found by building it

- **Simulating to steady state was slow and stopped early.** The first generator held each probe
  until it stopped changing. A table took 1.0 s; with warm starts, looser tolerances and rows
  every 2 m/s it took 0.05 s. The grip-limit test then failed: at μ 0.5 the fastest row ended at
  76% of peak friction, because near the peak the tire's slope approaches zero and the turn could
  not settle within the 3 s budget. Solving for the fixed point instead reaches 96% of peak
  friction at both μ 0.5 and μ 1.0, and a table takes about 0.03 s.
- **The default car does not need MAP.** Over two laps at the planner's most aggressive cornering,
  MAP and Pure Pursuit tracked the default car alike. On the soft-front car Pure Pursuit's error
  doubled and MAP's stayed at or below the default car's.
- **MAP caps a turn the car cannot hold.** While swerving around a cone cluster at 19.7 m/s, the
  pursuit arc asked for 26 m/s² of lateral acceleration and MAP steered 0.062 rad instead of
  geometry's 0.173 (`out/runs/map-blockage-grip-drop`). In a transient that cap can cost tracking:
  on the default car at aggressive cornering MAP's largest error was 0.530 m against Pure Pursuit's
  0.501 m. The same swerve run then met a grip drop at 7 s 0.19 m/s faster than the Pure Pursuit
  run did, so 579 of its 2401 samples carry the speed envelope's braking-transient flag, against
  none for Pure Pursuit; no sample left the tire envelope in either.

## Verification

- `tests/steering_tests.cpp` (`fd_steering_law`, 6 groups):
  - the kinematic bicycle's table equals `atan(Lκ)` within 5e-5 rad inside its envelope and
    saturates at the steering limit and at μg;
  - the soft-front and soft-rear tables follow the linear understeer gradient built from cornering
    stiffness measured on the tire law, within 2%, and their correction to geometry is `K a`
    within 10%;
  - the largest steady turn lies between 85% and 100% of peak friction, doubles with grip, and
    lookups never decrease as the demanded turn tightens;
  - a table for a heavier car, more grip or another model is rejected, and a MAP simulation
    regenerates its table when grip commits;
  - MAP keeps Pure Pursuit's target, speed control and lookahead in every sampled state;
  - two laps per law, per car, at lateral grip fraction 0.75.
- `fd_recording_playback` (10 groups) round-trips a MAP run's law, fingerprint and every
  geometric and MAP steering value, loads schema 3 as Pure Pursuit, and refuses five tampered
  steering records. `fd_recording_contract` records and checks a MAP run through the headless
  binary and refuses MAP on the kinematic bicycle.
- Fresh kinematic runs of four scenarios kept every decision, trajectory, plan, event and track
  file byte-identical to their schema 3 recordings, every schema 3 telemetry column identical, and
  the summary identical.

## What this is not

MAP here is a steady-state correction. It does nothing about yaw inertia or tire lag in
transients, where it can track slightly worse than geometry. It does not change longitudinal
allocation: a rear-wheel-drive car still spins leaving the first corner under MAP. The table
describes the plant as modeled, with the limits of decision 0008, and the soft-front car is an
illustrative setup, not a measured one. Nothing is fitted to a real vehicle.

## Consequences

- A MAP run regenerates its table at every committed grip change, which cost about 0.03 s per table
  in the test suite on the reference host. That tick takes that much longer; it was not measured in
  the desktop, and it is not a real-time guarantee.
- Every new recording is schema 4. Tools reading telemetry by position must account for one more
  trailing column.
- Phase 3's vehicle-derived performance envelope can reuse the same fixed-point search to find
  the largest steady turn at each speed from the plant rather than from `lateral_grip_fraction`.
