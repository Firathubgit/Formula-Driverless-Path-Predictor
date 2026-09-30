# 0013: Recording and showing the four-wheel car

Date: 2026-09-15. Status: accepted.

Decision 0012 built a car with four rotating wheels behind the vehicle seam, but recordings refused
it and neither application could select it. Phase 2's "Visible" list asks for wheel-speed against
ground-speed indicators with a red lock flash. This decision records the car, offers it, and shows
what its wheels are doing.

## Decision

**Recording schema 5.**
- `metadata.json`'s `vehicle_model` gains the kind `four_wheel_car`, one line with every parameter and
  both tires. The reader rejects unknown keys, runs `validate_vehicle`, requires schema 5 for this
  kind, and rejects a MAP steering law for it, since no table is generated for it.
- `telemetry.csv` appends `front_left_wheel_speed_radps`, `front_right_wheel_speed_radps`,
  `rear_left_wheel_speed_radps` and `rear_right_wheel_speed_radps`. They must never be negative, and
  must be zero for a car without rotating wheels.
- `RecordedSample::plant_state()` now includes the wheel speeds, so plant reproduction re-integrates
  the four-wheel car from its recorded wheels, and replay derives its slip ratios and wheel states
  through the seam without advancing anything.
- Schema 4, 3, 2 and 1 load with zero wheel speeds; none of them had a car with rotating wheels.

**Headless.** `--plant four-wheel`; `--brake-bias-front 0..1`, refused unless the plant is the
four-wheel car; `--drive-front-fraction` now applies to it too; `--steering map` and `--car soft-front`
are refused for it before any output. `--check-recording` names the car with its mass, brake bias,
drive split and power, and counts samples with a wheel locked while moving.

**Desktop.**
- A fourth vehicle model button, **4 wheels**. MAP is not offered for it and is refused with the
  reason; choosing it returns to Pure Pursuit.
- The TIRE SLIP card gains a tile per wheel, laid out as on the car: slip ratio, and a state of GRIP,
  SPIN (driven past the tire's peak), SLIDE (past the peak otherwise) or LOCK (stopped while the car
  moves), with LOCK in red. The car in the scene draws a locked tire red. The state is a steady colour
  rather than a flash, so captures and checks are deterministic.
- REAR SIDESLIP and MAP CORRECTION moved from the metrics row into the TIRE SLIP card. At the minimum
  window size the replay scrubber sat over them; the metrics row is back to the three readouts it was
  laid out for. The status line now elides instead of running under the buttons beside it.

No brake-bias or drive-split control is added to the desktop. Those are Phase 2.4's honest setup
controls, which must each come with a test proving their effect.

## Found by building it

- **10% front bias does not lock the rear wheels under the planner's braking.** At the planner's
  0.55 g the rear brake torque comes out about 2% short of the rear tires' capacity once wheel inertia
  absorbs part of it. Only all braking on the rear axle locked them. A 20 s headless run at 10% front
  bias locked no wheel; with no front braking the rear wheels locked at 7.97 s and the car spun.
- **The replay scrubber covered two readouts** added in decisions 0009 and 0010, visible for the first
  time in a capture of a replayed dynamic run.

## Verification

- `fd_recording_playback` (11 groups): a four-wheel run with all braking on the rear axle round-trips
  every parameter and wheel speed against the live plant, records 264 samples with the rear left wheel
  locked while moving, and reproduces its motion to 9.90e-10 m; editing its mass exposes itself. A
  schema 4 copy loads with zero wheel speeds. Refused by name: a wheel speed on a kinematic run, a
  negative wheel speed, MAP on the four-wheel car, the four-wheel car in schema 4 metadata, and an
  unknown four-wheel key.
- `fd_recording_contract` records a four-wheel run through the headless binary with a brake bias of
  0.4, checks the recorded kind, bias and wheel columns, validates it, and refuses MAP for the car.
- `fd_desktop_controls` (280 checks): the 4 wheels button selects the car under Pure Pursuit, MAP is
  disabled and refused with the reason, the wheel tiles show each wheel's state and slip ratio from
  the plant in the first corner with no wheel locked, a replay of the rear-locking run shows the rear
  wheels LOCK and the front ones turning, the car's rear tires are drawn locked and its front tires not,
  and the kinematic bicycle shows no wheel states. Inspected captures: `docs/assets/four-wheel-lock.png`
  and a refreshed `docs/assets/map-steering.png`.
- Headless runs on the preset, each validated with `--check-recording`:

  | Run | Result | Samples with a wheel locked | Invalid samples |
  | --- | --- | ---: | ---: |
  | default, one lap | 33.805 s, max tracking error 0.358 m | 0 | 0 of 6762 |
  | 90% front bias, 20 s | max tracking error 0.371 m | 0 | 86 of 4001 |
  | 10% front bias, 20 s | max tracking error 0.345 m | 0 | 21 of 4001 |
  | no front braking, 20 s | spun: 1.347 rad of sideslip, 14.1 m off the line | 264 | 2475 of 4001 |

  Plant reproduction was between 9.7e-10 and 9.9e-10 m for all four.

## What this is not

The wheel states read the plant; they are not a traction-control or ABS signal, and nothing acts on
them. The four-wheel car still has no load transfer, so it cannot show weight moving onto the front
wheels under braking, which is what makes real brake bias delicate.
