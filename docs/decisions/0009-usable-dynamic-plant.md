# 0009: Making the dynamic plant usable: cost, recording schema 3 and selection

Date: 2026-09-15. Status: accepted.

Decision 0008 built the dynamic single-track plant but left it unusable outside the core. A 3 s
prediction cost 5.5 ms and a decision among six lines 33.5 ms, beyond the 20 ms control period.
Recordings refused it, because schema 2 had nowhere to put its state. Neither application could
select it. This decision removes those three obstacles, in that order.

## 1. Prediction cost

Measured first with a scratch benchmark: default car entering the first corner, Release build,
reference host.

| Step | One 5 ms tick | 3 s prediction | Decision among 6 lines |
| --- | ---: | ---: | ---: |
| Decision 0008 as built | 7.59 µs | 5.52 ms | 33.5 ms |
| Tire prepared once per advance | 2.24 µs | 2.17 ms | 13.1 ms |
| Substep 2.5 ms instead of 1 ms | 1.05 µs | 1.35 ms | 8.3 ms |

- **Prepared tires.** Every axle force evaluation had called `tire_peak` and `tire_force`, and both
  validated the tire parameters: 40 validations per tick. `TireAtLoad` (in `core/tire.hpp`)
  validates once and holds the peak values and curve constants at one load. `tire_force` and
  `tire_peak` are now implemented with it, so the force law exists in one place. A test compares
  2000 loads and 40,000 slips bit for bit. The plant's results did not change.
- **Substep 2.5 ms.** A new test compares the default step with a 0.5 ms reference over three
  seconds of a corner entry, a slide at μ 0.4 and a pull-away through the blend band; positions
  agree within 1 cm and velocities within 1e-3.

That test failed at first for the pull-away, by 2.7 cm. **The cause was a design flaw, not the step
size:** decision 0008's blend pulled the lateral state toward the kinematic bicycle's after every
substep, so a finer step pulled more often, and the blended model was not an ordinary differential
equation that could converge. The blend now weights the state derivative instead: the dynamic
derivative, and the kinematic bicycle's own derivative (rear lateral velocity zero, yaw rate speed
times curvature, both differentiated along the steering ramp), with a 20 s⁻¹ relaxation toward that
motion. With this change the convergence test passes, and a new check brakes a turning car through
the band and finds tick-to-tick changes below 6e-4 rad/s and 6e-4 m/s, so switching to the exact
kinematic bicycle below 1 m/s does not jump. Lap results changed only in the fourth significant
digit of the largest tracking error.

## 2. Recording schema 3

New recordings are schema 3 for both models.

- `metadata.json` gains a one-line `vehicle_model`: `{"kind": "kinematic_bicycle"}`, or
  `dynamic_single_track` with every parameter including both tires. `model` names it and
  `assumptions` states whether tire slip is modeled. The reader rejects unknown kinds and keys,
  validates the model with the recorded configuration, and requires the name to match.
- `telemetry.csv` appends `lateral_velocity_mps`, `yaw_rate_radps` and `within_grip_envelope`, which
  with the existing pose complete the plant state. For the kinematic bicycle the reader requires zero
  lateral velocity and a yaw rate equal to speed times curvature. For either model a sample outside
  the grip envelope cannot be marked valid.
- `summary.json` gains `max_rear_sideslip_rad`, recomputed from telemetry and compared.
- Schema 2 and schema 1 still load. Their samples derive the plant state the kinematic bicycle
  defines, which is what produced them; schema 2 metadata or summaries carrying schema 3 fields
  are refused as altered.

**Plant reproduction.** `plant_reproduction_error_m` re-integrates every recorded step with this
build's plant: from the recorded plant state, under the recorded acceleration and the governing
decision's steering command, with that step's configuration. It returns the largest one-step
position difference. Like planner reproduction it is reported, not enforced, because a recording
from a different build is still a valid recording. Measured: about 1e-9 m for fresh kinematic,
dynamic and spinning rear-wheel-drive runs, and 2.4e-5 m after editing a recorded 800 kg mass to
1100 kg, so an edited or foreign plant shows as a number four orders of magnitude above rounding.
Schema 1 recorded no steering command and reports that reproduction is unavailable.

Fresh kinematic runs of four scenarios kept every decision, trajectory, plan, event and track file
byte-identical, every schema 2 telemetry column identical on every row, and the summary identical
apart from the new line.

## 3. Selecting the plant

- Headless: `--plant kinematic|dynamic` and `--drive-front-fraction 0..1`, the second refused unless
  the plant is dynamic. `--check-recording` prints the vehicle and the plant reproduction.
- Desktop: a Kinematic / Dynamic choice in the header beside the simulation mode, enabled only when
  paused and live. Changing it starts a fresh run with the applied grip and the stated scenario. A
  rear sideslip readout appears for the dynamic plant only, since the kinematic bicycle cannot
  slide. Replay shows the recorded model and sideslip and refuses a change, saying why. The drive
  split is not exposed in the desktop yet.

## What this is not

Nothing here makes the plant more realistic. It has the limits decision 0008 lists: no load transfer,
aerodynamics, drag, wheel speeds or locking, and illustrative parameters. Plant reproduction checks
one step at a time; it is not a closed-loop replay and does not evaluate a different controller.

## Consequences

- The dynamic car's decisions now cost about 1.35 ms per prediction on the reference host, so a
  blocked decision fits the control period with margin. That is a measurement, not a real-time
  guarantee.
- Every new recording is schema 3, including kinematic ones, so tools that read telemetry by column
  position must account for three trailing columns.
- Phase 1.3 of TrackWayFastPlan, a steering lookup table generated from this plant, can now be
  developed, recorded and compared in both applications.
