# 0018: A speed plan from the car's own envelope

Date: 2026-09-16. Status: accepted for TrackWayFastPlan Phase 3.2, amended 2026-09-17 by decision 0020: with an
envelope, driving is bounded by the forward limit at the lateral acceleration the reference asks at the car's speed;
braking keeps the straight-line capacity.

Phase 3.2 asks `make_speed_plan` to gain an envelope input, with the fixed grip fractions kept as the baseline mode.
Following TUM's velocity profile and FS-FEUP's friction-ellipse passes: a curvature cap from `a_y,max(v)`, a forward
pass with `a_x,max(v, a_y)` minus drag and the power limit, a backward pass with braking capacity, and the existing
periodic convergence at the start/finish seam kept. Its tests: the periodic-constraint tests generalise to the
envelope; lower grip still moves braking earlier on the same scenario; planned speed never exceeds the envelope. Its
visible item: the speed plan chart switches to the envelope.

## Decision

**One planner, two sources of capacity.** `make_speed_plan(track, config, envelope)` plans with the grip fractions when
the envelope is null, exactly as before, and with the four-wheel car's performance envelope (decision 0017) otherwise.
An envelope derived under another configuration is refused.

- **Curvature cap.** Each sample's speed is the first speed from rest at which the sample's curvature asks more lateral
  acceleration than the envelope allows on that side. The lateral limit is linear in speed between the envelope's grid
  speeds, so on each piece speed squared times curvature less the limit is convex; the first piece that ends beyond it
  holds the crossing, found by bisection. Every slower speed is then within the envelope too, which the passes rely on,
  because they only lower speeds and the lateral limit falls at low speed.
- **Forward and backward passes**, FS-FEUP's order: accelerating from a sample uses the forward limit at that sample's
  speed and the lateral acceleration its curvature asks there; braking into a sample uses the braking limit at that
  sample's speed and lateral acceleration. The monotone periodic relaxation over every edge, the seam included, is
  unchanged.
- **Nothing is subtracted again.** The plan's wording subtracts drag and the power limit from the tires' longitudinal
  capacity, as TUM's does from its `ggv.csv`. This project's envelope reports accelerations the plant achieved, with
  drag, power, torque, wheel inertia and slip losses already in them, so the planner uses them as they are.
- **Holding speed within the lateral limit is always allowed.** At the lateral limit the envelope's forward capacity
  is about zero or slightly negative (decision 0017), yet the limit is by definition a turn the plant held steady, so
  forward capacity within it is never taken below zero. Without that, a constant-radius corner planned at its limit
  would have to slow down sample after sample.

**A share of the envelope, not all of it.** `Config::envelope_fraction` (0.5 to 1, default 0.8) shrinks the envelope
toward its origin before planning: a point (lateral, longitudinal) is allowed when that point divided by the share is
within the envelope. The share was chosen by driving candidate plans closed loop on the preset, below. It is a planning
configuration value, recorded, loadable from a configuration file and settable in the headless runner, and like every
exposed control it has a test proving its effect. It is not part of any fingerprint: it chooses how much of an envelope
a plan uses, not what the plant does.

**The controller can follow the plan's braking.** `compute_control` bounded the applied acceleration by the
longitudinal grip fraction of mu g, 5.39 m/s² by default, which cannot follow an envelope plan braking at 6 m/s² and
more. Given the envelope, `compute_control`, `make_local_trajectory` and `choose_local_action` bound it instead by the
car's whole straight-line forward and braking capacity at its present speed, leaving the feedback its full authority
beyond the plan's share. The envelope travels beside the steering table, is checked against the model and
configuration in the same way, and a hold for a blocked corridor brakes toward the clearance at its fraction of the
car's braking capacity at the grid's lowest speed, the least it has on the way down.

**The simulation owns the envelope.** `Simulation(track, config, model, steering, SpeedPlanMode)` with
`SpeedPlanMode::grip_fractions` (the default) or `performance_envelope` (four-wheel car only) derives the envelope at
construction, plans with it, passes it to every decision, and derives it again with the plan whenever a grip change
commits, so envelope, plan and revision change together at a tick boundary. To make that affordable the envelope's
speeds and sides are now derived in parallel: the default envelope takes 0.27 s instead of 1.3 s on this machine and
is byte-identical to the one recorded in decision 0017.

**Recording schema 8.** `metadata.speed_plan` records the mode and, for the envelope, the fingerprint of the envelope
derived for the initial configuration; `initial_config.envelope_fraction` records the share. The reader requires both
from schema 8, rejects them before it, reads older runs as planned with the grip fractions, and refuses an envelope plan
for any car but the four-wheel one. `plan_reproduction_error_mps` derives the envelope again for every plan revision
and reproduces the plan; `recorded_performance_envelope_matches` reports whether this build derives the recorded
fingerprint.

**Headless.** `--speed-plan grip-fractions|envelope` and `--envelope-fraction 0.5..1`, the latter refused without the
envelope plan, the former for any plant but the four-wheel car, both before any output. The run summary names the plan;
`--check-recording` reports "Speed plan: 0.80 of performance envelope <fingerprint> (this build derives it from the
recorded car)" or the grip fractions.

**Desktop.** A PLAN row under STEERING offers Fractions and Envelope 80%, the latter for the four-wheel car while
paused; a change is a fresh run keeping grip, setup and scenario, a setup change keeps the plan, and any other model
returns to the grip fractions. In replay the row is hidden and the recorded plan follows the recorded steering law in
the caption. WHY THIS SPEED names the car's lateral, braking or acceleration envelope as the constraint, and its braking
bound becomes the plan's share of the car's straight-line braking capacity at the present speed. The G-G diagram shows
the envelope the plan was made from instead of deriving it again, with the plan's share drawn inside it. Under the envelope plan the grip slider applies on
release, because each change derives the envelope again.

## Found by building it

- **Planning with the whole envelope makes the car slide.** Driven closed loop on the preset for two laps, the
  exact envelope plan laps in 29.02 s against 32.35 s for the grip fractions, but a tire is past its peak in about 9% of
  samples, rear sideslip reaches 0.097 rad and tracking error 1.30 m. Located by station: braking into corners with
  the rear inner tire past its peak while the car already carries 2.4 to 3.7 m/s² of lateral acceleration over
  track curvature that is still zero, because Pure Pursuit turns in ahead of the corner; driving out of corners with
  the front inner tire past its peak while 0.7 to 2.8 m/s² of lateral acceleration remains after the curvature has
  ended; and mid-corner from 350 to 370 m, curvature about 0.03 per metre, 2.7% over the lateral limit while holding
  speed, because the car drives a tighter line than the reference.
- **A curvature window does not fix it; a share does.** Candidate plans driven for two laps (second lap):

  | Plan | Plan estimate | Second lap | Samples past a tire's peak | Tracking error | Rear sideslip |
  | --- | ---: | ---: | ---: | ---: | ---: |
  | Whole envelope | 28.26 s | 29.02 s | 1173 of 13200 | 1.31 m | 0.097 rad |
  | Lateral use from the largest curvature within 5 / 10 / 15 m | 28.89 / 29.45 / 29.99 s | 29.69 / 30.23 / 30.75 s | 1242 / 1005 / 705 | 1.33 m | 0.098 rad |
  | 0.9 of the envelope | 29.15 s | 29.67 s | 136 | 0.76 m | 0.078 rad |
  | **0.8 of the envelope** | 30.29 s | 30.70 s | **0** | 0.51 m | 0.057 rad |
  | 10 m window and 0.95 | 29.99 s | 30.61 s | 0 | 1.00 m | 0.092 rad |
  | 0.7 of the envelope | 31.77 s | 32.12 s | 0 | 0.37 m | 0.044 rad |
  | Grip fractions (baseline) | 31.89 s | 32.35 s | 0 | 0.34 m | 0.041 rad |

  Of the shares tried, 0.8 is the largest that kept every tire within its peak (0.9 did not; values between were not
  tried), with half the tracking error of the window combination for 0.09 s more. The reserve covers this controller's
  turn-in and tracking, not an error in the envelope: a controller that anticipated them could use more of it.
- **Braking points move with the car.** On the preset at the default share the first corner is planned at 11.61 m/s
  against 10.71 under the grip fractions, and braking for it begins at 116.6 m against 111.2 m. A brake bias matched
  to the braking load moves it to 120.2 m, a 0.6 m centre of gravity height to 113.0 m, 1200 kg to 115.4 m, 3 m² of
  downforce area to 117.8 m, and grip 0.7 to 105.9 m.

## Verification

`tests/envelope_speed_plan_tests.cpp` (`fd_envelope_speed_plan`, 10 groups) checks, through the envelope's public
queries:

- **planned speed never exceeds the envelope**: on the preset, every sample within the share of the lateral limit at
  its speed and every edge, the seam included, within the share of the forward or braking capacity at the end it is
  planned from; the planned acceleration is each edge's;
- **the periodic constraints generalise**: a 30 m circle is planned at one speed, the one at which its lateral
  acceleration meets the share of the lateral limit (15.03 m/s at 0.8, 16.75 m/s at 1);
- **lower grip still moves braking earlier**: braking for the first corner begins at 105.9 m at grip 0.7 against
  116.6 m at grip 1, for 9.79 against 11.61 m/s;
- the envelope plan takes the first corner faster and brakes later than the grip fractions, and its estimated lap is
  shorter (30.29 against 31.89 s); the whole envelope's is shorter still (28.26 s);
- brake bias, height, mass and downforce move the braking point in their documented directions;
- an envelope derived at another grip, and a share outside 0.5 to 1, are refused;
- the controller's bound with the envelope is its braking limit at the speed (-7.80 m/s² at 10 m/s) and forward limit
  (7.73 m/s² at 5 m/s), against the grip fraction's 5.39;
- the simulation derives the envelope, plans with it, pulls away at its forward limit, and derives both again as
  revision 2 when grip changes; the grip fractions stay the default and the kinematic bicycle is refused;
- closed loop for two laps: the default share laps faster than the grip fractions and keeps every tire within its
  peak, and the whole envelope is faster but does not.

`fd_recording_playback` (12 groups) records the four-wheel car planning with its envelope through a grip change: the
mode, fingerprint and share round-trip, this build derives the fingerprint, both revisions' plans are reproduced within
3.5e-10 m/s, an edited fingerprint loads but is reported as another envelope, an edited share is exposed by plan
reproduction, an envelope plan for the kinematic bicycle is rejected, and a schema 7 run reads as planned with the grip
fractions. `fd_recording_contract` records `--speed-plan envelope --envelope-fraction 0.9` through a grip event,
checks the metadata and the `--check-recording` report, and refuses the envelope plan for the single-track car and a
share without it before any output. `fd_desktop_controls` checks the PLAN row while paused and running, the fresh run
and its plan, the G-G diagram showing the plan's own envelope and the inner share, braking beyond the grip fraction's
bound into the first corner with the braking envelope named and the braking bound stated, the return to the grip
fractions, the recorded plan in replay, and nothing to plan with for the kinematic bicycle.

Headless runs on the preset, two laps each into fresh `out/runs` directories (standing lap, then flying lap):

| Run | Laps | First corner planned | Braking from | Tracking error | Rear sideslip | Invalid samples |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `plan-fractions`, grip fractions | 34.045 / 32.345 s | 10.71 m/s | 111.2 m | 0.344 m | 0.041 rad | 0 of 13279 |
| `plan-envelope`, `--speed-plan envelope` | 31.950 / 30.695 s | 11.61 m/s | 116.6 m | 0.514 m | 0.057 rad | 0 of 12530 |
| `plan-envelope-whole`, `--envelope-fraction 1` | 30.330 / 29.025 s | 13.02 m/s | 122.6 m | 1.305 m | 0.097 rad | 1091 of 11872, tires past peak |
| `plan-envelope-bias-0.62`, `--brake-bias-front 0.62` | 31.830 / 30.605 s | 11.61 m/s | 120.2 m | 0.516 m | 0.058 rad | 0 of 12488 |
| `plan-envelope-cg-0.6`, `--cg-height 0.6` | 32.655 / 31.260 s | 11.34 m/s | 113.0 m | 0.474 m | 0.053 rad | 97 of 12784, tires past peak |
| `plan-envelope-mass-1200`, `--mass 1200` | 32.550 / 31.295 s | 11.36 m/s | 115.4 m | 0.515 m | 0.058 rad | 0 of 12770 |
| `plan-envelope-downforce-3`, `--downforce-area 3` | 31.490 / 30.245 s | 11.78 m/s | 117.8 m | 0.528 m | 0.058 rad | 0 of 12348 |
| `plan-fractions-grip-0.7`, grip fractions, `--grip 0.7` | 39.580 / 37.390 s | 8.96 m/s | 95.7 m | 0.324 m | 0.042 rad | 0 of 15395 |
| `plan-envelope-grip-0.7`, `--grip 0.7` | 36.175 / 34.710 s | 9.79 m/s | 105.9 m | 0.510 m | 0.063 rad | 0 of 14178 |
| `plan-envelope-grip-drop`, `--grip-event 20:0.7` | 33.940 / 34.705 s | 11.61 m/s | 116.6 m | 0.525 m | 0.065 rad | 34 of 13730, braking transient |

The envelope plan laps the default car 1.65 s faster than the grip fractions on the flying lap, and 2.68 s faster at
grip 0.7. The share does not keep every car within its peak: the 0.6 m high car passes a tire's peak in 97 samples, all
braking into corners with about 3.5 m/s² of lateral acceleration at turn-in, the mechanism the share was chosen for;
0.8 was chosen on the default car, and a smaller share for the tall one was not tried. The grip drop's 34 invalid samples are the existing "current speed
exceeds revised plan; braking transient" for 0.17 s after the drop commits at 20.005 s, with no tire past its peak.
Every run validates with `--check-recording` as schema 8 from the same source as this build: plans reproduced within
3.1e-10 to 4.2e-10 m/s, the grip drop's two revisions included, plant motion within 9.94e-10 m, and every envelope
run reports that this build derives its envelope. All 95 recordings in `out/runs`, schemas 1 to 8, still load.
`scripts/build.ps1 -Desktop`: 15/15 suites passed, 180.17 s of tests, with 360 QML checks, up from 339, and no runtime warnings; `scripts/build.ps1`: 14/14 suites passed, 146.41 s of tests. Captures inspected:
`docs/assets/envelope-plan.png`, and the blockage scene with the new PLAN row, whose decision text stays clear of the
lap counters.

## What this is not

Not a racing line: the plan still follows the centreline, whose curvature steps at the joins of lines and arcs, and
Phase 3.4 is where a minimum-curvature line arrives. Not minimum-time optimisation of the speed profile beyond the two
passes. The share is a reserve for this controller on this track, measured, not derived. Only the four-wheel car has an
envelope; the kinematic bicycle and the single-track car keep the grip fractions. The controller is unchanged apart
from its bound, and it still does not anticipate its own tracking error.

## Consequences

- Brake bias, centre of gravity height, mass, downforce and grip now move planned speeds and braking points, which is
  what Phase 2.4's setup controls could not show.
- A changed envelope generator or planner changes plans, and recordings expose that through plan reproduction.
- Phase 3.3 conditions the track; a smoother reference would narrow the gap between planned and driven lateral
  acceleration that the share now covers.
