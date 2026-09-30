# 0015: Aerodynamics and a viscous coupling

Date: 2026-09-16. Status: accepted for TrackWayFastPlan Phase 2.3, with the viscous differential Phase 2.1
listed and decision 0012 deferred.

Phase 2.3 asks for aerodynamics from fastest-lap's `get_aerodynamic_force` (drag ½·ρ·cd·A·v·|v| and lift
½·ρ·cl·A·v_x², applied at a pressure centre) and TUM's front and rear lift coefficients, tested by a top speed
where drag equals the available drive force and by corner speed that rises with speed on a high-downforce
setup and not on a zero-downforce one. Phase 2.2's test list adds that loads sum to weight plus downforce,
and the plan warns that a high-downforce car must be tested above the tire's 6000 N reference load. Phase
2.1 lists a viscous differential that moves `k·(ω_left − ω_right)` of torque between a driven axle's wheels;
decision 0012 deferred it because with static loads it had nothing measurable to do. Load transfer (decision
0014) now unloads the inner driven wheel in a corner. Nothing was copied from either source.

## Decision

**Aerodynamics** (`FourWheelCar`, `aerodynamic_force`):
- `drag_area_m2`, the drag coefficient times frontal area, default 0.6 m², a road car's. `downforce_area_m2`,
  the lift coefficient times area with positive pressing down, default 0: this illustrative car has no wings.
  `aero_balance_front`, the front axle's share of downforce, default 0.5. Each area is validated within
  0..10 m² and the balance within 0..1.
- Air is still and at the standard sea-level density `air_density_kgpm3` = 1.225 kg/m³. Drag is
  `½·ρ·drag_area·|v|·v` against the centre of gravity's velocity in the body frame; downforce is
  `½·ρ·downforce_area·v_x²`.
- **Drag acts through the centre of gravity.** It slows the car and neither turns it nor moves load by
  itself; load moves only through the tire force that overcomes it, whose pitch arm is the centre of gravity
  height (decision 0014). fastest-lap applies aero at a pressure centre with its own moment; that height would
  be a parameter with no separate test to prove it, so it is left out and named here.
- **Downforce acts at the axles** as the balance splits it: `quasi_static_wheel_loads` gains a downforce
  argument, and each axle's two wheels share its part equally. Loads in the state include the downforce at
  the state's own forward speed, so a recorded sample's loads sum to weight plus a downforce computed from
  its recorded speed.
- Through the 1 to 3 m/s blend band the aero force blends out with the rest of the dynamic derivative, and
  the kinematic range below 1 m/s has none; at 3 m/s default drag is 3.3 N.

**Viscous coupling** (`FourWheelCar::viscous_coupling_nms`, default 0, an open differential, validated
0..200 N·m·s/rad). On each driven axle, the front when `drive_front_fraction > 0` and the rear when it is
below 1, torque `k·(ω_left − ω_right)` moves from the faster wheel to the slower. The implicit wheel update
becomes `I(ω' − ω) = h(drive − brake − Fx(ω')R − k(ω' − ω_other'))`; the two wheels of a coupled axle are
solved alternately, each holding the other's latest speed, until neither changes by more than 1e-10
relative. Each solve also damps its own wheel by `h·k`, which makes the alternation a contraction. With
`k = 0` the update is exactly decision 0012's.

**Naming.** A car with drag or downforce names itself "…with rotating wheels, load transfer and
aerodynamics…", or "…with rotating wheels and aerodynamics…" at zero height; without either it keeps
decision 0014's or 0012's name. The coupling is a setup detail and does not change the name.

**Recording schema 7.** The `four_wheel_car` metadata gains `viscous_coupling_nms`, `drag_area_m2`,
`downforce_area_m2` and `aero_balance_front`, required from schema 7 and refused in older metadata. No
telemetry column is added: drag and downforce follow from the recorded state. The reader now requires wheel
loads to sum to weight plus the downforce at each sample's speed, and a car at zero height to carry its
static loads plus its share of that downforce. A schema 6 or older four-wheel car reads back with no
aerodynamics and no coupling. The metadata's assumptions name "aerodynamic drag and downforce in still
air" and "no rolling resistance or actuator delay" for a car with aerodynamics.

**Headless.** `--drag-area`, `--downforce-area`, `--aero-balance-front` and `--viscous-coupling`, refused
unless the plant is the four-wheel car. `--aero-balance-front` is also refused when the downforce area is
zero, because it would have no effect. `--check-recording` names all four.

**Desktop.** The TIRE SLIP card shows DRAG / DOWNFORCE in kN for a car with aerodynamics, live or from the
recorded plant state. No setup control is added: that is Phase 2.4.

## Found by building it

- **Drag changed every oracle that assumed none.** A coasting test, the acceleration, locked-sliding and
  power-limit oracles, the energy balance, the braking load transfer (the tire force is mass times
  acceleration plus drag) and three names. Each oracle now adds drag from its definition; the plan's
  own energy test already names drag work.
- **A coupling helps traction and can make oversteer worse.** Out of a left turn at 10 m/s on rear-wheel
  drive at 0.45 g, an open differential spun the unloaded inner rear wheel to slip ratio 2.06 and gained
  2.014 m/s in 0.6 s with 0.151 rad of rear sideslip; a 40 N·m·s/rad coupling held it at 0.33, gained
  2.248 m/s and slid 0.130 rad. At 0.6 g the outer wheel could not take the moved torque either, so the
  coupling spun both rear wheels: 0.964 m/s gained against 1.504 and 0.572 rad of sideslip against 0.368.
- **Downforce does not yet reach the planner.** On a lap the 3 m² car keeps the same lap time as the car
  without wings, 34.045 s, because the speed plan's fixed grip fractions know nothing of downforce; it uses
  less of its grip (largest use 0.794 against 0.826) and tracks slightly closer. Phase 3.1's envelope from
  the plant is what turns downforce into speed.
- **My high-downforce test first steered into a load transfer.** Steering 0.003 rad at 85 m/s is 8.3 m/s² of
  lateral acceleration, which moved the inner wheels below 6000 N; the test now steers 0.0005 rad and checks
  friction at the loads it starts with, before drag slows the car.

## Verification

`tests/aerodynamics_tests.cpp` (`fd_aerodynamics`, 7 groups):
- drag and downforce equal their definitions within 1e-9 N at four velocities, doubling speed quadruples
  downforce, a car at rest feels none, and downforce reaches the wheels as its balance says;
- coasting at 29.82 m/s decelerated at 0.391182 m/s² against drag over mass plus wheel inertia of 0.391190;
  without drag the car kept 30 m/s within 1e-6;
- at full throttle for 90 s from 45 m/s the car reached 64.631 m/s against the 64.801 m/s where drag times
  speed equals 100 kW, and 51.271 m/s against 51.433 with twice the drag area, both from below and still
  gaining less than 1e-3 m/s per second;
- a car with 3 m² of downforce area carried weight plus the downforce at its speed on every tick, up to
  2939 N at 40 m/s;
- the largest settled turn inside the tire envelope fell from 9.24 to 9.18 to 8.86 m/s² at 10, 20 and
  30 m/s without downforce and rose from 9.40 to 9.95 to 10.77 m/s² with 3 m², 21.5% above the plain car at
  30 m/s;
- at 85 m/s with 4 m² of downforce area every wheel carried more than the 6000 N reference load, the
  heaviest 6516 N with peak lateral friction 0.831 below the upper reference's 0.85 and above the floor, and a
  second of coasting kept loads, speed and the envelope physical;
- eight invalid parameters are refused, the limits accepted, and the three names checked.

`fd_wheel_model` (12 groups) adds the coupling group above, including that in a straight line a coupling
changes position and wheel speeds by less than 1e-9. With drag in every oracle: threshold braking took
21.76 m and locked braking 25.11 m; a locked car decelerated at 7.782 m/s² against 7.782 from the tire law and
drag; the power-limited car accelerated at 1.114 m/s² against 1.122 from power less drag; braking energy
balanced to 0.03%; the unchanged controller lapped in 34.045 s. `fd_load_transfer` (10 groups) passes
with drag in its oracle; a prediction measured 2.8 to 3.1 ms and a blocked decision 18.3 to 18.5 ms
(measurement only).

`fd_recording_playback` round-trips all four parameters on a rear-locked fixture that also has load
transfer, reproducing it to 9.73e-10 m; refuses a schema 6 copy whose loads carry downforce, aero keys in
schema 6 metadata, a missing drag area and an edited downforce area; and still reads a schema 6 kinematic
copy. `fd_recording_contract` records `--drag-area 0.8 --downforce-area 1.2 --aero-balance-front 0.45
--viscous-coupling 30`, checks the keys and the report, and refuses an aero balance without downforce and a
coupling for the single-track car. `fd_desktop_controls` (303 checks) checks the drag and downforce readout
against the plant and its absence for the kinematic bicycle; capture `docs/assets/four-wheel-aero.png`
was inspected.

Headless runs on the preset, each validated with `--check-recording`, plant reproduction between 9.84e-10 and
9.99e-10 m:

| Run | Lap | Max tracking error | Largest grip use | Max rear sideslip | Invalid samples |
| --- | ---: | ---: | ---: | ---: | ---: |
| `aero-still-air-lap`, `--drag-area 0` | 33.810 s | 0.353 m | 0.844 | 0.043 rad | 0 of 6763 |
| `aero-lap`, default | 34.045 s | 0.344 m | 0.826 | 0.041 rad | 0 of 6810 |
| `aero-winged-lap`, 3 m² at 0.45 front | 34.045 s | 0.331 m | 0.794 | 0.037 rad | 0 of 6810 |
| `aero-rwd-open-lap`, rear-wheel drive | 34.250 s | 0.365 m | 1.000 | 0.069 rad | 926 of 6851 |
| `aero-rwd-coupled-lap`, 40 N·m·s/rad | 34.215 s | 0.728 m | 1.000 | 0.107 rad | 771 of 6844 |

On the rear-wheel-drive laps the rear wheels were up to 94.2 rad/s apart with the open differential and
2.5 rad/s with the coupling. `aero-still-air-lap` is byte-identical to decision 0014's `loads-lap` apart
from metadata identity and the four new vehicle keys, and `aero-scenario-avoid`, `aero-pursuit-default` and
`aero-map-soft` are byte-identical to their schema 6 recordings apart from metadata identity.

## What this is not

No wind, air density variation, pressure centre height, aerodynamic pitch or yaw moment, ride-height
sensitivity, drag reduction or ground effect; no rolling resistance. The coupling is a linear viscous
coupling on driven axles, not a clutch-pack or Salisbury differential, and nothing limits its heat. The
planner's speed plan still ignores drag and downforce. Parameters are illustrative.

## Consequences

- Phase 2's plant is complete for the plan's list: wheels, brakes, load transfer, aerodynamics and a
  differential. Phase 2.4 can expose drag and downforce areas, the coupling, brake bias, centre of gravity
  height, roll balance, mass and power, with the tests above and in decisions 0012 and 0014 as the effect
  proofs for the core; each still needs its own on a fixed scenario through the application.
- Phase 3.1's envelope from the plant is where downforce first changes planned speed.
