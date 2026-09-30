# 0030: Simulated cone perception

Date: 2026-09-23. Status: accepted for TrackWayFastPlan Phase 7.2, which it completes. Recording schema 14 becomes 15.

Phase 7.2 asks for mock cone perception after PacSim's perception sensor: a field of view, a detection probability and a
classification probability that fall with distance, spherical noise whose covariance is propagated, a delay per sensor,
a cone layout for the track derived from its boundaries, and everything labelled as simulated detections. PacSim (MIT)
is on this machine; its `src/sensorModels/perceptionSensor.cpp` and `config/perception.yaml` were read for structure and
for the model's form. Nothing was copied and nothing was downloaded.

## Decision

**The course's cones are ground truth in the core.** `make_cone_layout()` (`core/cones.hpp`) lays blue cones along the
corridor's left boundary and yellow along its right, in the direction of travel, each boundary being the reference
offset along its normal by the corridor's edge distance, so a racing-line track cones the corridor it records. A cone
stands every 5 m on a straight, the Formula Student handbook's maximum, and sooner wherever the boundary has turned
through 0.1 rad since the last, never closer than 1.5 m; a big orange cone stands on each boundary at the start line.
They are the scenario's ground truth, as a stated blockage is: nothing in the core detects them.

**A perception sensor observes them as PacSim's does, in the plane.** `Perception` (`adapters/sensors/perception.hpp`)
holds one or more sensors, each mounted on the car at a position and yaw from the rear axle, sampling at its own rate
and delivering each frame a dead time later. The default is PacSim's front LiDAR: 10 Hz behind 200 ms, 1 to 45 m and
60° either side. Of the cones in range and view, each is detected with probability
`max(0.1, 0.99 − 0.01 (d − 1) − 0.00012 (d − 1)²)` at distance `d`, and the rest are listed missed. A detected cone's
colour is right with a probability of the same form (floor 0.2, guessing among five), the other four colours, unknown
among them, sharing the rest, and beyond 15 m it is unknown for certain; as PacSim does, the sensor gives the colour it
reports the probability of the true one. It is placed with noise in bearing, range, range in proportion and each axis,
and reports the covariance of that noise at the cone, carried from polar to Cartesian through the Jacobian and turned
into the car's frame with the mount. Cones stand on the ground plane, so the planar model drops PacSim's vertical angle.

**What the planner may see is kept apart from what the evaluation knows.** A `PerceptionFrame` holds detections in the
car's frame at the moment of sampling, with no word of which cone each was. That lives in a `PerceptionTruth` beside it:
the true pose the frame was sampled from, the cone of each detection, and the cones missed. Phase 7.4's boundary test
can therefore hand a planner frames alone.

**One seed, one stream per sensor.** Every draw, detection, colour and noise, comes from a stream derived from the seed
and the sensor, numbered apart from the instruments'. PacSim seeds its generator with the frame count, so a standing car
sees the same noise every run and every sensor the same draws; here a recorded seed makes a run reproducible without
tying sensors together. The fingerprint covers the settings, the seed and the cones.

**Nothing drives on it, and a run records it all.** The simulation observes the perception at every fixed tick from the
car's true pose and never reads it. Schema 15 records the settings and seed in `metadata.perception`, every cone at full
precision in `cones.csv`, every frame delivered in `perception_frames.csv` with the tick that delivered it, every
detection in `detections.csv` with its cone for evaluation, and every missed cone in `missed.csv`. The loader runs the
perception again from the recorded seed and cones over the recorded poses and requires the same frames.

**The desktop draws the evaluation's view.** With CONES fitted in the sensors card the scene shows the course's cones in
their colours and the newest frame's detections placed from where the car truly was: a diamond in the colour reported,
white when unknown, a red cross over a wrong colour, a grey ring on a cone in view that was missed, and the ellipse of
twice the standard deviation. The card counts what the frame saw, missed and mis-coloured, and how old it is. The
card's readouts were cut to a line each so that, fitted with the instruments beneath the four-wheel car's tires and
MPCC's margin plot, it still ends above the bottom bar in the compact window; the instruments' four lines became two.

## Verification

`tests/cone_tests.cpp` (`fd_cones`, four groups): on the preset, 119 blue and 119 yellow cones and the big orange pair,
each within 5 cm of its boundary, the big orange ones at the start line; neighbours never more than 5 m apart nor closer
than the minimum; 0.32 cones per metre of boundary in corners against 0.20 on straights; a track with recorded edges
coned at those edges; bad options and an empty track refused.

`tests/perception_tests.cpp` (`fd_perception`, nine groups): a sensor reports exactly the cones in range and view,
standing where they are in the car's frame, turned and moved by the pose onto their cones whatever the mount; over 3999
frames a cone at 5 m is detected 0.952 of the time against the model's 0.948, and one at 35 m 0.517 against 0.511; the
colour is right 0.970 of the time at 3 m against 0.9695 and 0.844 at 14 m against 0.839, the wrong colours sharing the
rest evenly, and unknown for certain beyond 15 m; over 9999 frames of a cone 20 m away the scatter of its detections
matches the covariance it reports within 2%, and a sensor turned on its mount reports the same ellipse; a frame is
sampled a period in and arrives its dead time later; one seed perceives alike and another differently, the fingerprint
changing with the seed, a sensor or a cone; bad settings refused. On a running car, ten seconds of the dynamic car with
perception drive exactly as without it; 98 frames arrive, each sampled from the car's true pose, with about 14
detections each, 637 cones missed and 38 of 468 coloured detections the wrong colour.

`tests/recording_tests.cpp` adds `perceived_run_round_trips` and `rejects_perception_violations` (22 groups): eight
seconds perceived with seed 31 record 240 cones exactly as laid and 78 frames, 923 detections and 493 misses; a run
without perception records the files empty. Seven tamperings are refused by name: a detection moved by a millimetre,
recoloured, or credited to another cone; a missed cone changed; a cone moved; the seed changed; a frame dropped.
`tests/recording_contract.cmake` records a second of perception through the headless runner, checks its seed, rate and
eight frames, validates it, and refuses a seed that is not whole.

A one-lap run of the dynamic car with `--perception 4`: 335 frames, 4681 simulated detections, about 14 a frame, 2195
cones in view missed, 128 of 1618 coloured detections the wrong colour and 3063 uncoloured beyond 15 m; the check runs
the perception again over the recorded lap and matches every frame.

The desktop's UI verification fits the cones on the four-wheel car, checks the cones drawn are the course's own, that a
frame has arrived between 200 and 305 ms old, that its detections are drawn and each stands within half a metre of a
cone, that the card counts what the bridge does and ends above the bottom bar, the same beneath MPCC's margin plot with
the instruments fitted too, and that taking the cones off hides them.

Builds: `scripts/build.ps1` passed 23 of 23 suites in 523.1 s. The first `scripts/build.ps1 -Desktop` ran its UI
verification to 563 checks in 111.4 s with no QML warning, but its recording contract suite failed because CMake files
were edited while it ran, not from the change; the next desktop build, with Phase 7.3 and 7.4 on top, passed all 27
suites and 584 UI checks (decision 0031).

Five mutations were run and reverted. A detection probability that does not fall with distance, a covariance left in
the sensor's frame, a sensor facing the car's heading whatever its mount, and corners coned as sparsely as straights
each failed a group. Dropping the loader's comparison of missed cones would have passed: no tampering touched
`missed.csv` until one was added while the mutations were being written; with it, the mutation fails.

## What this is not

Not perception of anything real, and not a detector: it samples ground truth through a statistical model and says so on
screen. Not a LiDAR simulation: there is no ray casting, occlusion, cone height or vertical field of view, and no false
positive, which PacSim's model does not produce either. Not yet in the loop: the car drives on its true state and the
known track. Not a knocked-down cone: cones cannot be hit until Phase 7.5's competition logic.

## Consequences

- Phase 7.2 is complete. Phase 7.3 turns these frames into a local path; 7.4 proves the planner receives frames and
  measurements only; 7.5 counts cones hit.
- Recordings are schema 15. Every older schema still loads.
- `fingerprint_hex` moved from the core's private identity header to `fd/fingerprint.hpp`, so the steering tables, the
  envelopes, the instruments and the perception share one implementation.
