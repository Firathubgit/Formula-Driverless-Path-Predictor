# 0025: The model mismatch is measured, and the four-wheel car is predicted with its pitch and air

Date: 2026-09-19. Status: accepted for TrackWayFastPlan Phase 6.2. Recording schema 13. Measures the prediction model of
decision 0024 apart from its replanning; adds longitudinal load transfer and aerodynamics, off by default, to the
dynamic single-track plant of decision 0008; and gives the four-wheel car's single-track reduction that car's centre of
gravity height, drag and downforce.

Phase 6.2 of the plan asks for model mismatch as an experiment, not an accident: MPCC's prediction model is a reduced
single-track model and the plant is the four-wheel car, so record both and report the prediction error over the horizon.
MPCC's own repository never measures it; its simulations drive the model the controller predicts with.

## What the plan error could not say

Decision 0024 measured each recorded plan against where the car went. The car follows no plan for longer than one
control period before the next replaces it, so that plan error mixes two things: what the prediction model gets wrong,
and how far each new plan departs from the last. For the dynamic car, whose prediction model is its own plant, the plan
error three seconds ahead was 0.45 m at the median and 3.63 m at the 95th percentile: nearly all
replanning. It could not tell a four-wheel car the model misjudges from one it merely replans.

## Decision

**A plan states its commands.** `LocalTrajectory::commands` holds the acceleration and steering a predictive
controller's plan commands at each point, changing linearly between them; the policy's predictions leave it empty.
`planned_command()` reads them at any time. A plan that drives must state one at every point, and its first command must
be where they are at the end of the control interval; the simulation throws `std::logic_error` for a controller that
breaks either, rather than record a plan the car did not follow. MPCC's first command is now read from its plan this
way; it is the value it computed from its first input before, except where the steering limit clipped the first stage.

**The plant response.** `plant_response()` (`core/predictive_control.hpp`) copies the plant from the state a plan was
made in and drives the copy by the plan's commands exactly as the simulation holds a command: over each control
interval, where the commands are at its end, the first interval ending at the next regular control tick. Nothing is
replanned. How far the plan is from it a given time ahead is the **model error**, the prediction model's own; the
**plan error** stays what decision 0024 measured.

**Recording, schema 13.** `commands.csv` holds, in decisions.csv order, one row per point of every plan a predictive
controller drove: `decision,option,point,time_s,acceleration_mps2,steering_rad`. Loading checks one command at every
point of each such plan and none anywhere else, each at its point's time, steering within the steering limit, and that
the decision's applied command is where the plan's commands are at the end of its control interval. The dynamic car's
metadata gains `cg_height_m`, `drag_area_m2`, `downforce_area_m2` and `aero_balance_front`. Older runs load without
commands, and their model error is not measured.

**Measuring.** `controller_plan_deviations()` and `controller_model_deviations()` give each plan's deviation a time
ahead: distance, the component along the plan's direction of travel, positive ahead of it, the size across it, and the
difference in speed. The model deviation drives this build's plant from the recorded state under the configuration the
plan was made with, like the plant reproduction check; it is a measurement, and replay still never advances the plant.
`controller_plan_errors()` and `controller_model_errors()` summarise them; `prediction_error()` summarises any subset.
`--check-recording` prints both a half, one, two and three seconds ahead; `--prediction-error DIR` prints both every
0.25 s to the horizon, the flying laps alone, and the eight plans the model got most wrong a second ahead.

## Found by measuring

With the reduction of decision 0024, two laps under MPCC each, from the exploratory runs `out/runs/p62x-*` (source
fingerprint `76500131fe86711bd7de1a5fadc7c381516c9a4d0775f07c3eae449b486525d3`; their solve times were measured beside
each other and are not reported):

| MPCC run | Model error 1 s, median / p95 (m) | 3 s, median / p95 (m) | Along at 3 s (m) | Across at 3 s (m) | Speed at 3 s (m/s) | Flying laps, 3 s (m) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Dynamic car, the prediction model's own plant (unchanged by this decision; its final run) | 0.004 / 0.036 | 0.07 / 0.19 | -0.00 | 0.04 | -0.00 | 0.07 / 0.17 |
| Four-wheel car | 0.091 / 0.259 | 1.26 / 4.86 | -0.78 | 0.76 | -0.44 | 1.13 / 4.20 |
| Four-wheel car, centre of gravity on the ground | 0.089 / 0.123 | 0.77 / 0.95 | -0.74 | 0.15 | -0.49 | 0.76 / 0.80 |
| Four-wheel car, no drag | 0.037 / 0.241 | 0.94 / 4.71 | -0.00 | 0.72 | +0.05 | 0.82 / 3.97 |
| Four-wheel car, neither | 0.008 / 0.058 | 0.07 / 0.30 | -0.00 | 0.04 | -0.00 | 0.06 / 0.23 |

1. **The control.** For the dynamic car the plant under a plan's commands stays within 4 mm of it a second on
   at the median and 7 cm three seconds on: what remains is the plant's hold of each command over a 20 ms control period
   and its 2.5 ms integration step against the model's 25 ms.
2. **Drag is a steady bias.** The four-wheel car fell 0.78 m behind each plan in three seconds at the median,
   0.44 m/s slower; without drag the bias is gone.
3. **Load transfer is the tail.** Without it the 95th percentile three seconds ahead fell from 4.9 to 0.95 m and the
   median distance across the plan from 0.76 to 0.15 m. The worst plans a second ahead were all pulling away: MPCC
   commanded 9.14 m/s^2 at 7 m/s, the load moving rearward left the front wheels, which drive half the car, too little
   to carry their share, they spun, and the car fell 1.54 m behind in a second. Those were all 471 invalid samples of
   decision 0024's four-wheel run; with the centre of gravity on the ground there were none.
4. **The wheels themselves barely matter.** With neither, the four-wheel car's model error is close to the dynamic
   car's.
5. **Lateral load transfer would add little.** At 9 m/s^2 of lateral acceleration its 840 N per axle costs these tires
   about 1.3% of the axle's grip, since their friction falls only from 1.0 to 0.85 between 2000 and 6000 N.

**So the prediction model carries the four-wheel car's pitch and air.** `DynamicSingleTrack` gains `cg_height_m`,
`drag_area_m2`, `downforce_area_m2` and `aero_balance_front`, all off by default so the dynamic car and every recording
of it are unchanged: the requested tire force moves its height over the wheelbase of load between the axles, downforce
adds to them by the balance, each axle's tires are evaluated at the load they carry, and drag acts through the centre of
gravity. They are the plant's equations, so `SingleTrackModel` stays the dynamic plant's own, and a dynamic car given
them is still a plant its prediction model matches exactly. `reduced_single_track()` copies them from the four-wheel
car. Steering tables and envelopes of a car without them keep their fingerprints.

## Result

Final runs `out/runs/p62-*`, one at a time, two laps each:

| MPCC run | Model error 1 s, median / p95 (m) | 3 s, median / p95 (m) | Along at 3 s (m) | Across at 3 s (m) | Speed at 3 s (m/s) | Flying laps, 3 s (m) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Dynamic car | 0.004 / 0.036 | 0.07 / 0.19 | -0.00 | 0.04 | -0.00 | 0.07 / 0.17 |
| Soft-front car | 0.004 / 0.034 | 0.06 / 0.18 | -0.00 | 0.04 | -0.00 | 0.06 / 0.16 |
| Four-wheel car | 0.021 / 0.076 | 0.10 / 0.46 | -0.01 | 0.06 | -0.01 | 0.08 / 0.24 |
| Four-wheel car, centre of gravity on the ground | 0.007 / 0.058 | 0.07 / 0.29 | +0.00 | 0.04 | -0.00 | 0.06 / 0.23 |
| Four-wheel car, no drag | 0.020 / 0.075 | 0.09 / 0.46 | -0.01 | 0.05 | -0.02 | 0.07 / 0.23 |
| Four-wheel car, neither | 0.008 / 0.058 | 0.07 / 0.30 | -0.00 | 0.04 | -0.00 | 0.06 / 0.23 |
| Four-wheel car with downforce, 1.5 m^2 at 0.4 front | 0.019 / 0.074 | 0.10 / 0.45 | -0.01 | 0.06 | -0.01 | 0.09 / 0.25 |

The four-wheel car under MPCC laps in 26.97 / 25.33 s with 0 invalid samples and a peak grip use of
0.958, against 27.07 / 25.35 s with 471 invalid samples and 1.000 before. Its model error is now close to the
dynamic car's: the largest a second ahead is 0.16 m, at 0.96 s and 5.4 m/s. Its plan error a
second ahead fell from 0.060 m at the median and 0.68 m at the 95th percentile to 0.026 and
0.28 m; three seconds ahead it is still 0.48 m at the median, as the dynamic car's is, because
there replanning, not the model, is most of it. Solving the four-wheel car's decisions, with its tires evaluated at the
load each axle carries, took 16.7 ms at the median against 15.5 ms for the dynamic car in the same sequence of
runs.

## Verification

`tests/trajectory_tests.cpp` adds two groups: a plan's commands change linearly between its points and hold beyond them,
with bad input refused; and the plant response to a plan is what the simulation does when every decision keeps to that
plan, the four-wheel car over three seconds within 1e-9 m, a shortened first interval taking the command at its own end,
and times off the grid, a plan not starting at the state and a first interval beyond a control period refused.
`tests/vehicle_tests.cpp` adds `pitch_and_air_act_as_on_the_four_wheel_car`: drag decelerates a coasting car by the
four-wheel car's drag force to 1e-12, the front axle's friction use braking at 6 m/s^2 is the hand-computed one at the
transferred load, a tall car braking into a turn turns more, downforce loads each axle by its balance, and each
parameter out of range is refused; the plant's-equations identity and the envelope agreement now include a car with
pitch and air, and the reduction test the four copied parameters. `tests/mpcc_tests.cpp` checks MPCC's commands (one per
point, its steering the plan's, its first command the plan's at the end of the interval), bounds the dynamic car's model
error over a closed-loop lap (under 0.01 m a second ahead and 0.15 m three seconds ahead at the median), requires the
four-wheel car's lap to pass no tire's peak with its model error under 0.05 and 0.3 m, and makes the simulation refuse a
plan without commands and one whose first command is not its plan's. `fd_recording_playback` (18 groups) round-trips the
commands of every plan that drove, measures model error from the recording, refuses five tamperings of `commands.csv`
and a schema 12 run beside it, keeps a schema 12 run's plan error without a model error, round-trips and reproduces a
dynamic car with pitch and air and refuses its fields in schema 12, and records a four-wheel car kept to its plans
across an off-grid grip change: model error under 1e-6 m everywhere, plan error at most 1e-13 m wherever the grip did
not change under the plan and up to 0.046 m where it did. `fd_recording_contract` checks schema 13, an empty
`commands.csv` for a policy run, the model error line and `--prediction-error`'s table for an MPCC run, and its refusal
for a policy run.

Six mutations were run and reverted, each failing a group: the reduction without pitch and air
(`each_model_reduces_to_a_single_track`, and `the_four_wheel_car_laps_the_preset` passing a tire's peak); load moved the
wrong way and no drag (`pitch_and_air_act_as_on_the_four_wheel_car`); the plant response taking each command at its
interval's start (`the_plant_response_is_the_simulation_keeping_to_a_plan` by 1.2e-5 m at 0.05 s, and
`a_plan_kept_to_has_no_error`); the loader not checking a plan's first command (`command-not-driven`); and the
simulation not checking a plan's commands, which first survived because `planned_command` refused the plan anyway, until
the test required the simulation's own message.

`scripts/build.ps1`: 20 of 20 CTest suites pass in 392.6 s, `fd_mpcc` taking 165.6 s. `scripts/build.ps1 -Desktop`: 21
of 21 in 484.6 s, the QML controls suite with 482 checks and no QML warning; neither build reports a compiler warning.

Every final run into `out/runs/p62-*` passes `--check-recording`. With this build 288 of the 290 recordings in
`out/runs` pass `--check-recording` (schemas 1 to 13, 33 of them schema 13). The two refused are
`mpcc-dynamic-laps-mpcc`, cut off while it was written in the previous session and refused for its missing `events.csv`,
and the exploratory `p62x-dynamic-1789837332`, written by this session before the dynamic car's schema 13 metadata was
settled and refused for lacking `cg_height_m`. Source fingerprint
`4876ae6ce0d03902e010997689d76883ba2b264b5d6d4dd5b2ba260a4fd89a4c`. Machine-readable evidence is in
`docs/evidence/model-mismatch.json`.

## What this is not

Not lateral load transfer, brake bias or wheel speeds in the prediction model: the four-wheel car's brake bias is by
torque, its wheels spin and lock, and one track has no inner and outer wheel; what those leave is the model error that
remains. Not a model of a parameter change: the model error uses the configuration the plan was made with, so a grip
change under a plan is plan error. Not real time: predicting load-dependent tires costs MPCC more per decision. The
plant response drives a copy of the plant as a measurement; replay still never advances the recorded plant.

## Consequences

- Phase 6.3's head-to-head on the four-wheel car can now compare a valid MPCC run; its baseline, MAP with the envelope
  speed plan on the same car, still needs a steering table for the four-wheel car, which only the dynamic car has.
- Phase 6.4's per-wheel friction margin along the horizon can use `plant_response()` for the four-wheel car's own loads
  along a plan, since the single-track model knows axle loads, not wheel loads.
- Any future predictive controller must state its planned commands; the simulation refuses one that does not.
