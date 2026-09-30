# 0017: The car's performance envelope, derived from the plant

Date: 2026-09-16. Status: accepted for TrackWayFastPlan Phase 3.1.

Phase 3.1 asks for the car's G-G-V envelope from the plant: for each speed on a grid, the steady-state maximum
lateral acceleration, then the maximum and minimum longitudinal acceleration at several lateral levels. It names
two ways to reach a steady state, a two-variable Newton solve on the plant's residual or race_stack's method of
simulating the plant with speed held until yaw rate settles while steering is swept, and asks for an export in
TUM's `ggv.csv` format with a model fingerprint. Its tests: symmetric in lateral acceleration, braking capacity
exceeds drive capacity, lateral capacity grows with speed only when downforce is non-zero. Phase 3's visible list
includes the current point on a live G-G diagram with the envelope drawn around it.

## Decision

**Simulate to settle, through the vehicle seam.** `PerformanceEnvelope` (`core/performance_envelope.hpp`) needs no
solver and never touches the plant's equations: it calls `advance` and `envelope` as the simulation does. A turn is
held at its speed by a proportional-integral throttle loop at a fixed steering angle, half a second at a time for up
to six seconds, and judged on the last quarter second of each half second:

- *steady* when yaw rate varies by less than 0.2% of itself (at least of 0.05 rad/s), speed is within 1% of the
  target, and every tire stayed within its peak;
- *tire* when, after the first half second, a whole quarter second was spent with some tire beyond its peak, or a
  steady quarter second was not entirely within;
- *settling* when six seconds pass without either.

"Within its peak" is the four-wheel plant's own envelope answer: no wheel lifted and every tire's combined slip at
most one.

**The lateral limit** at a speed comes from sweeping steering in 24 steps up to its limit, each turn settled from the
last, until a turn fails; ten halvings between the last steady and the first failed steering refine the boundary.
The limit is the lateral acceleration, speed times yaw rate, of the last steady turn. It is labelled *tire* or
*settling* from the failure beyond it, or *steering* if the steering limit was reached with the tires within their
peak. If the car cannot hold a speed straight ahead, the grid ends below it.

**Lateral levels** are settled turns at 0, 25, 50, 75, 90 and 100% of that limit. The 0% level is the straight run
and the 100% level the limiting turn itself; each other level is steered onto its target by up to twelve halvings
of steering between the settled turns either side of it, stopping within 1%.

**Longitudinal limits** at each level are found from its settled turn: the largest forward and braking commands,
by fourteen halvings between zero and 2.5 g, whose next 0.1 s keeps every tire within its peak. Each is reported as
the acceleration the plant achieved over the second half of that 0.1 s, so drag, wheel inertia, slip losses, the
power and torque limits and the brake bias are all in it. If the 2.5 g command itself stays within, the limit is
labelled *actuator*: power, torque or brake capacity bind before the tires.

**Left and right are derived separately**, so symmetry is measured, not assumed.

**The grid** runs from 4 m/s, above the 1 to 3 m/s band where the plant blends into the kinematic bicycle, to 4 m/s
above the configured top speed, every 2 m/s; 4 to 24 m/s for the default configuration. Queries interpolate
linearly in speed between rows, clamped to the grid, and in lateral acceleration between levels; beyond the lateral
limit there is no longitudinal capacity. `side_at` interpolates one side's levels level by level, which is the
boundary to draw at any speed.

**Only the four-wheel car has an envelope.** The kinematic bicycle's limits are its stated grip fractions, and the
single-track car clamps each axle's longitudinal force at its capacity, so a probe would measure that allocation
rather than tires. Both are refused with that reason.

**The fingerprint** is FNV-1a over the generator version, the validated vehicle and every configuration value the
plant reads. The model and configuration identity that the steering tables already hashed moved, unchanged, into
`core/src/model_identity.*`, shared by both; this build still derives the recorded table of all ten recorded MAP
runs, seven distinct fingerprints. `generated_for` refuses an envelope for another car, setup or road grip.

**Export in TUM's format.** `write_performance_envelope` writes three files, each headed by comment lines naming the
fingerprint, the model and the grip:

- `ggv.csv` (`v_mps, ax_max_mps2, ay_max_mps2`);
- `ax_max_machines.csv` (`v_mps, ax_max_machines_mps2`);
- `envelope.csv`, every speed, side and level with the limit labels.

TUM's velocity profile adds drag itself and takes the tires' longitudinal limit and the powertrain's separately.
So `ax_max` is the straight-line braking limit less the drag deceleration at that speed, `ay_max` the smaller
side's lateral limit, and `ax_max_machines` the straight-line forward limit plus drag deceleration.
`fd_headless --plant four-wheel [setup flags] --write-envelope DIR` derives and writes one without running a lap,
refusing a non-empty directory and any other plant before doing any work.

**Desktop.** For the four-wheel car a G-G ENVELOPE card beside the TIRE SLIP card draws the envelope's boundary at
the current speed, a one-g circle for scale, and a dot at the car's lateral acceleration (from the tire forces,
left positive, drawn to the left) and the longitudinal acceleration it achieved over the last tick. The envelope is
derived on a worker thread, so the window never waits for it. It is derived again whenever the car, its setup or the
applied road grip changes, and a result is shown only while it was derived for the car shown. In replay it is derived
from the recorded car and configuration, the dot comes from consecutive recorded samples, and the card says so. The
setup panel opens in the card's place. A car the envelope cannot be derived for says so rather than waiting.

## Found by building it

- **Turns need time to settle.** With a fixed settling time many reachable turns were judged failures because yaw
  rate was still moving, so the limit was set by settling rather than by the tires. Settling in half-second chunks
  up to six seconds, judged on the last quarter second, fixed that. A few grid rows still end on a settling verdict
  near the limit (24 m/s for the default car and the 1200 kg car, 22 m/s at grip 0.7).
- **At low speed parallel steering, not grip, sets the limit.** The four-wheel car has no Ackermann geometry, so in a
  tight turn the outer front wheel is steered at the inner wheel's angle and scrubs. At 4 m/s the default car's
  lateral limit is 3.30 m/s², against 9.80 m/s² its tires could give and 3.77 m/s² full steering's geometry allows. On
  full steering the outer front tire's combined slip reaches 1.002 while the front axle's slip angle is 0.063 of its
  0.121 rad peak. The test states this directly on the plant. With a 0.6 m centre of gravity height the 4 m/s limit
  is *steering* instead (3.34 m/s²): that car holds full steering with every tire within its peak.
- **Wheel spin-up is not acceleration.** Measured over the whole 0.1 s probe, forward capacity read low, because the
  first ticks spend about a seventh of the power spinning the wheels up to their new slip; the second half measures
  the car.
- **Power less drag is an upper bound, not the answer.** Where power binds, forward capacity is 88 to 100% of
  (P/v − drag) over the car's mass plus its wheels' equivalent mass, the rest going into tire slip: 7.64 against 8.46 m/s² at 14 m/s,
  4.57 against 4.73 m/s² at 24 m/s.
- **At the static brake bias braking only matches driving.** The default car brakes 50:50 but under braking its load
  moves forward, so its unloaded rear wheels reach their peak first. Straight-line tire-limited braking is 7.72 m/s² at
  4 m/s against 7.73 m/s² all-wheel-drive forward capacity. Braking exceeds driving wherever power limits driving
  (7.87 against 7.64 m/s² at 14 m/s, 8.15 against 4.57 at 24 m/s), and at every speed by more than 0.5 m/s² with a bias
  matched to the braking load. The test states both.
- **At the lateral limit there is no forward capacity left.** Holding speed in the limiting turn already needs
  throttle, so the 100% level's forward capacity is about zero or slightly negative: the car can only brake there.
- **Without downforce the lateral limit is flat but not smooth.** It rises from 8.75 m/s² at 8 m/s to 9.46 at 14 m/s,
  then stays between 9.30 and 9.46 m/s² to 24 m/s, moving by up to 1.4% between neighbouring speeds. With 3 m² of
  downforce area it grows from 9.66 at 12 m/s to 10.46 at 24 m/s.
- **Inside the boundary is not within the tires' peak.** The diagram draws accelerations. In replay of the
  desktop's rear-braked fixture (all braking on the rear axle, envelope `a85f5f629f7c5a8a`, the same fingerprint
  `fd_headless --plant four-wheel --brake-bias-front 0 --write-envelope` derives), 8.0 s into the run the car
  decelerates at 3.59 m/s² at 18.39 m/s with both rear wheels locked, inside a braking limit of 4.33 m/s² there: a
  sliding tire gives less than its peak. The tire slip card, not the diagram, says whether the tires are past their peak.
- **Levels need steering onto.** Taken straight from the sweep's settled turns, levels sat off their fractions
  (9.16 against a target of 8.48 m/s² at one), because lateral acceleration is far from linear in steering near the
  limit; bisecting steering onto each level fixed it.

## Verification

`tests/performance_envelope_tests.cpp` (`fd_performance_envelope`, 12 groups) derives the default car, a winged car
(3 m² downforce area), a car with brake bias matched to its braking load and a car with a 0.6 m centre of gravity
height, and checks:

- the grid from 4 m/s to 4 m/s above the top speed every 2 m/s; for each side a level per fraction, ascending, the
  first straight ahead, the last the lateral limit, each within 3% plus 0.05 m/s² of its fraction; forward capacity
  above braking at every level, and both smaller at the 90% level than straight ahead;
- **symmetry**: left and right lateral limits within 1%, and forward and braking capacity within 3% plus 0.05 m/s² at
  every level below the limit and 0.3 m/s² at it, where capacity falls steeply with lateral use; the largest measured
  difference is 0.002 m/s² in lateral limit and 7.1e-14 m/s² in longitudinal capacity below it;
- the lateral limit never exceeds what the four tires can give at their loads plus downforce, and from 8 m/s reaches
  at least 85% of it (16 m/s: 9.36 of 9.68 m/s², winged 10.02 of 10.20);
- the low-speed parallel-steering limit, stated on the plant as above;
- forward capacity where power binds lies within 88 to 100.1% of (P/v − drag) over the car's equivalent mass;
- **braking capacity exceeds drive capacity** where power limits driving, and everywhere with the matched bias;
- **lateral capacity grows with speed only with downforce**: 12 to 24 m/s, 9.377 → 9.446 m/s² without (within 1%),
  9.660 → 10.461 with 3 m² (at least 5%);
- setup moves the envelope: a matched bias brakes more than 0.5 m/s² harder at 12 m/s (9.54 against 7.77 m/s² at
  8 m/s); the 0.6 m height lowers the 16 m/s lateral limit from 9.36 to 9.00 m/s²;
- queries interpolate between rows and levels, clamp to the grid, give nothing beyond the limit, and `side_at` agrees
  with them;
- the fingerprint is the one computed without deriving, changes with the car's mass and with road grip, and
  `generated_for` refuses both;
- the export writes TUM's columns with the drag conversion and the fingerprint in each file, and refuses a model it
  was not derived for;
- the kinematic bicycle, the single-track car and an invalid configuration are refused.

`fd_recording_contract` derives `--plant four-wheel --downforce-area 3 --write-envelope`, checks the reported
fingerprint is named in all three files and `ggv.csv` has eleven rows, and checks `--plant dynamic --write-envelope`
is refused before creating anything. `fd_desktop_controls` checks the card for the live car and its fingerprint,
the boundary through `side_at`'s levels at the car's speed, the dot, the status line, a new envelope after a setup
change, the envelope from the recorded car in replay, the card giving way to the setup panel, and nothing for the
kinematic bicycle.

Headless derivations with the desktop build's `fd_headless`, each 11 speeds from 4 to 24 m/s, into fresh `out/`
directories (`envelope-bias-0`, `--brake-bias-front 0`, `a85f5f629f7c5a8a`, brakes 4.2 to 4.4 m/s² across the grid
and is otherwise the default car):

| Run | Fingerprint | Derived in | Lateral at 16 m/s | Forward at 8 / 16 m/s | Braking at 8 / 24 m/s |
| --- | --- | ---: | ---: | ---: | ---: |
| `envelope-default` | `033cc3dd344a93cd` | 1.333 s | 9.362 | 7.729 / 6.891 (actuator) | −7.765 / −8.149 |
| `envelope-downforce-3`, `--downforce-area 3` | `c1327d99bbd97278` | 1.236 s | 10.024 | 7.854 / 6.922 (actuator) | −7.860 / −9.068 |
| `envelope-bias-0.7`, `--brake-bias-front 0.7` | `00e8da03707e59f7` | 1.349 s | 9.362 | 7.729 / 6.891 (actuator) | −8.210 / −8.493 |
| `envelope-cg-0.6`, `--cg-height 0.6` | `44994f66f01a7921` | 1.246 s | 8.997 | 6.748 / 6.672 (actuator) | −6.788 / −7.163 |
| `envelope-mass-1200`, `--mass 1200` | `f0ead0588fa16c3f` | 1.106 s | 9.056 | 7.536 / 4.776 (actuator) | −7.560 / −7.800 |
| `envelope-grip-0.7`, `--grip 0.7` | `65e0ab051a7c359d` | 1.228 s | 6.580 | 5.769 / 5.762 | −5.810 / −6.222 |

Accelerations in m/s². Brake bias moves only braking; downforce raises lateral and braking capacity with speed;
height lowers all three; mass lowers where power binds most.

**Not done: the fastest-lap cross-check.** The plan suggests comparing shapes with fastest-lap's G-G diagram if its
Windows release is available. It is not on this machine, and downloading it waits for the user's approval.

## What this is not

Not yet used by the planner: the speed plan still uses its fixed grip fractions, so braking points do not move with
the car until Phase 3.2 feeds this envelope into `make_speed_plan`. Not a steady-state solver: limits are resolved to
the settling tolerances and bisection depths above, and the grid rows are not smooth to better than about 2%. Levels
are held turns, and a longitudinal probe lets the turn drift for 0.1 s rather than holding lateral acceleration
exactly. The envelope is for the applied road grip on a flat road, and parameters remain illustrative.

## Consequences

- Phase 3.2 can build the G-G-V velocity profile on `lateral_limit`, `forward_limit` and `braking_limit`, which is
  where brake bias, height, mass and downforce start moving planned speed and braking points.
- A change to how envelopes are derived changes `generator_version` and with it every fingerprint.
- Deriving costs about 1.1 to 1.4 s per car on this machine, so it belongs on a worker thread or ahead of a run, not in
  a control tick.
