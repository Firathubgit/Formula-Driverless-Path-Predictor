# 0005: Where a dynamic plant plugs in, and what it publishes

Date: 2026-09-14. Status: accepted and built on 2026-09-15 together with the dynamic plant,
as decision 0008 records, including two small refinements of the sketch below.

TrackWayFastPlan Phase 1.2 adds a dynamic single-track plant with tire forces. Today the
kinematic bicycle is the only plant, and knowledge of it is spread across three places:
`Simulation::step` calls `integrate_bicycle`, `make_local_trajectory` calls it for every
predicted tick, and both `Simulation::update_diagnostics` and the prediction's envelope
check recompute lateral acceleration as `v^2*tan(steering)/wheelbase`. That formula is only
true for a kinematic bicycle. A second model would have to be threaded through all three.

## Decision

**One module owns "the vehicle", with a small interface of pure functions over copyable
plant state.** The shape, in terms of the codebase-design vocabulary:

- `PlantState` is plain data: the published rear-axle `State` plus whatever the model needs
  internally (for the dynamic model: body lateral velocity, yaw rate, later wheel speeds).
  It is a value, because predictions copy the live state and roll the copy forward; a plant
  that owned hidden mutable state could not be predicted without disturbing it.
- `advance(model, plant_state, command, dt) -> PlantState` is the only way a plant moves.
- `pose(plant_state) -> State` gives controllers, recordings and the view the rear-axle
  state they already consume.
- `demand(model, plant_state, command) -> {longitudinal, lateral acceleration, grip use}`
  replaces the three copies of the kinematic formula, so the diagnostics and the envelope
  check ask the model instead of assuming one.
- `VehicleModel` is a closed `std::variant` of the two parameter sets, kinematic and
  dynamic, dispatched with `std::visit`. The set of plants is small and known, both adapters
  are value types, and a variant needs no heap, no inheritance and no virtual calls in the
  per-tick path.

`Command` stays requested longitudinal acceleration and front-wheel angle. Controllers keep
their meaning. The dynamic plant owns the allocation from requested acceleration to drive
and brake torque, bounded by what its actuators and tires can deliver; the kinematic plant
keeps applying acceleration directly. This moves "the caller owns longitudinal allocation"
from the plant's interface into the kinematic adapter's implementation.

**The rear axle stays the published reference.** A dynamic model integrates at the centre
of gravity and converts at the interface: the rear-axle position is the centre of gravity
minus `l_r` along the heading, and body longitudinal velocity is the same at both points,
because for two points on the body x axis yaw rate adds only a lateral velocity difference,
`-r*l_r`. `State::speed_mps` is
therefore body longitudinal velocity at the rear axle, clamped nonnegative as today. Sideslip
and yaw rate are plant state, reported through diagnostics, not new `State` fields.

## Why not now

One adapter makes a hypothetical seam. Introducing `VehicleModel` with only the kinematic
bicycle behind it would add an interface without anything varying across it, and its shape
would be guessed rather than tested. The seam is built in the same change that adds the
dynamic plant, and that change's acceptance test is that every existing suite and every
preserved recording is byte-identical with the kinematic adapter selected.

## Consequences

- `integrate_bicycle` becomes the kinematic adapter's implementation. Its tests stay.
- The envelope message "tire slip is not modeled" becomes model-specific. With the dynamic
  plant the car can slide, so exceeding the envelope is simulated rather than only flagged.
- Recordings need schema 2 before a dynamic run is recorded: sideslip, yaw rate and later
  wheel speeds and loads have no place in schema 1's exactly compared header.
- AGENTS.md's "a kinematic bicycle has no mass sensitivity" remains true of the kinematic
  adapter and stops describing the whole simulation once the dynamic adapter is selectable.
  It is reworded in the commit that ships evidence for the new plant, not before.
- The low-speed start from rest needs the kinematic-to-dynamic blend described in
  TrackWayFastPlan Phase 1.2. The blend lives inside the dynamic adapter, behind `advance`.
