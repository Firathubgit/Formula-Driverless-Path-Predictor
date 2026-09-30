# 0011: Showing tire slip, handling balance and the velocity vector

Date: 2026-09-15. Status: accepted, completing TrackWayFastPlan Phase 1's "Visible" list.

Phase 1 asks for sideslip and yaw rate in telemetry, a body velocity vector at the car, a per-axle
slip-angle gauge, and understeer and oversteer labels derived from front against rear slip with the
definition shown. Sideslip and yaw rate were recorded by schema 3 (decision 0009). This decision
settles what the labels mean and where the numbers come from.

## The terms

- **Axle slip angle.** The angle between an axle's wheel heading and its velocity, positive when
  the tires push the car to its left. Front: steering minus `atan2(vy + lf r, vx)`; rear:
  `-atan2(vy - lr r, vx)`, at the centre of gravity's lateral velocity `vy`, with `vx` floored at
  the slip-speed floor. These are exactly the angles the dynamic plant feeds its tire law.
- **Peak slip angle.** The slip angle at which that axle's tires reach peak lateral grip at their
  static load, from `TireAtLoad`. Past it the tire is sliding, which is the existing envelope.
- **Understeer angle.** The front slip angle minus the rear, measured toward the turn (multiplied
  by the sign of lateral acceleration). In a steady turn it equals the steering beyond geometry,
  `δ - L r / v` to first order, which is the correction MAP steering adds (decision 0010).
- **Balance.** *Understeer* above +0.25°, *oversteer* below -0.25°, *neutral* between. Below 1 m/s²
  of lateral acceleration there is no turn to balance: *not cornering*. The kinematic bicycle, and
  the dynamic car below its lower blend speed where it is the kinematic bicycle, have no tire slip:
  *not modeled*. The label is instantaneous, so it also shows transient understeer on turn-in and
  oversteer on a power exit, not only the steady-state gradient.

The 0.25° deadband is a quarter of a degree of road-wheel steering, below which the correction is
too small to act on; it is a display threshold, not a property of the car.

## Decision

- `Demand` in `core/vehicle.hpp` gains `front_slip_angle_rad`, `rear_slip_angle_rad`,
  `front_peak_slip_angle_rad` and `rear_peak_slip_angle_rad`, zero when tire slip is not modeled.
  `handling_balance(demand)` returns the balance and understeer angle; `balance_name` gives stable
  text. `balance_deadband_rad` and `cornering_lateral_acceleration_mps2` are public constants so
  the interface shows the definition it applies.
- `Simulation`'s diagnostics carry those values from the same `demand()` call they already made.
- **No new recording columns.** Slip angles depend only on the plant state, steering and vehicle
  parameters, all of which schema 3 and 4 record. `Playback::demand()` derives them at the cursor
  through the vehicle seam with the recorded model and the configuration in force. Nothing is
  advanced and no controller runs, so replay still never recomputes a decision.
- **Desktop.** For a model with tires, a TIRE SLIP card over the driving view shows each axle's
  slip angle as a bar against 125% of its peak with a tick at the peak, red past it; the balance
  label, coloured; and the definition with the current understeer angle. An arrow at the rear axle
  points along its velocity beside a faint heading line, 0.6 s of travel long, so the angle
  between them is the rear sideslip the REAR SIDESLIP readout reports. The kinematic bicycle shows
  neither. Replay shows both from the recorded plant state.

## Verification

- `fd_vehicle_model` (16 groups):
  - over 5000 random dynamic states, the reported slip angles equal the formula from the state
    alone within 1e-12 rad, the peaks equal `tire_peak` at static axle load, and the envelope is
    exactly both slip angles within their peaks;
  - settled turns left and right at 11 m/s: the soft-front car is *understeer* with an understeer
    angle of 0.005994 rad against 0.005980 rad of steering beyond geometry from the state; a
    soft-rear car is *oversteer*, -0.007842 against -0.007863; the default car is *neutral*,
    7.9e-6 rad; driving straight is *not cornering*; the kinematic bicycle is *not modeled*;
  - the simulation's diagnostics equal the seam's values for its own state, and the soft-front car
    understeers in the first corner.
- `fd_recording_playback`: at every one of the dynamic fixture's 2401 samples, replay derives the
  live slip angles and understeer angle within 1e-9 rad and the same label (447 understeering and
  425 oversteering samples), without moving the cursor; a schema 1 run reports *not modeled*.
- `fd_desktop_controls` (256 checks): the card and arrow appear for the soft-front car, show the
  simulation's slip angles and the UNDERSTEER label with its definition, the arrow departs from the
  heading line by the rear sideslip, replay derives the card from the recorded state, and the
  kinematic bicycle shows none of it. Inspected capture: `docs/assets/map-steering.png`.

## What this is not

The slip angles are those of a single-track model at static axle loads: there are no left and
right wheels, no load transfer, and no slip ratio yet (Phase 2). The balance label is a reading of
two slip angles, not a handling assessment of a real car.
