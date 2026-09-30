# 0027: Each tire's friction margin along the horizon

Date: 2026-09-20. Status: accepted for TrackWayFastPlan Phase 6.4, which it completes. Builds on the plant response and
recorded plan commands of decision 0025 and the controller of decision 0024; no recording schema change.

Phase 6.4 asks for MPCC's horizon drawn as the primary ribbon coloured by planned longitudinal acceleration with the
existing deadband, its friction-ellipse margin per wheel along that horizon, and the solver's status and solve time on
screen. The ribbon, the status and the solve time arrived with decision 0024. The margin is what was left: the
prediction model knows axle loads on one track, so it cannot say what a wheel has left, and nothing drew it.

## Decision

**The margin comes from the plant, not from the controller's model.** `plan_tire_use()` (`core/predictive_control.hpp`)
drives the plant along the plan by the plan's own commands, exactly as `plant_response()` does, and at each of the
plan's points asks the vehicle seam what each wheel is using of its own friction circle. The margin a tire has left
there is one minus that use. Only a car with four wheels has four tires of its own: every other model returns nothing,
its axles being what the slip card already shows.

**What each wheel uses is the plant's own answer.** For the four-wheel car `demand()` derives each wheel's longitudinal
and lateral use from the state it is in — its wheel speeds, its load and the slips they imply — so the margin along a
plan is the plant's, not a reconstruction. The acceleration `demand()` is asked for names the achieved longitudinal
acceleration and no wheel's use depends on it; an earlier version of this function looked up the command the plant was
holding at each point to pass there, and a mutation showed nothing depended on it, so that lookup is gone.

**The desktop draws it where the wheels are.** Under the tire slip card's four wheel tiles, a panel gives FRICTION LEFT
AHEAD as a percentage and the time it falls at, and plots each wheel's use against the three-second horizon, with the
tires' own limit marked at the top and a legend naming FL, FR, RL and RR. It appears only when a plan that states its
commands is driving a car with four wheels, live or replayed; under the policy, whose predictions state no commands,
there is nothing to drive along and the panel stays hidden.

**It is recomputed when the plan is, not every frame.** Driving the four-wheel plant three seconds along a plan costs
milliseconds, so the bridge recomputes only when a new decision's plan arrives, from the state that decision was made
in, and at most a few times a second. In replay it recomputes when the recorded decision under the cursor changes,
driving the recorded plant from the recorded state under the recorded commands.

## Verification

`tests/trajectory_tests.cpp` adds `each_tire_s_use_along_a_plan_is_the_plant_s_own` to `fd_local_trajectory`, which now
has 13 groups. Along a three-second plan of one braking command it checks that there is an entry at every one of the
plan's points and at its times; that the first entry is exactly what `demand()` says of the state the plan was made in,
to 1e-12; that braking asks more of the rear tires' circles than the front's, 0.48 against 0.38 two seconds in, because
the car brakes with half its torque at each axle while its load moves forward; and that held at the cornering limit all
four tires sit between 0.9 and 1.02 of their circles and within 0.02 of each other. A plan whose commands change,
driving for a second and a half and then braking, reverses that order along its own length: the front tires are the
busier pair while it drives (0.31 against 0.27) and the rear ones once it brakes (0.68 against 0.49). A single-track car
and the kinematic bicycle report none, and times off the fixed-tick grid are refused.

The desktop's UI verification checks the panel where a viewer sees it: under the policy there is no plan of commands, so
`planTireUse` is empty and the panel hidden; under MPCC on the four-wheel car it holds a row at each of the plan's 61
points, every row a time on the plan's own stages and four real shares of a circle; the readout names the busiest tire
and when it is busiest, agreeing with those rows; the plot is drawn; and returning to Pursuit empties it again.
Replaying a recorded four-wheel MPCC run, the same panel is measured from the recorded plant under the recorded
commands. The capture `docs/assets/tire-margin.png` is that panel six seconds into such a run: 7% of a circle left,
three seconds ahead.

Four mutations were run and reverted. Reading each tire at the state the plan was made in rather than along the plan,
and reporting four wheels for a car that has none, each failed `each_tire_s_use_along_a_plan_is_the_plant_s_own`.
Drawing the margin for a plan that states no commands failed the desktop's check that the policy has nothing to draw.
The fourth, taking each point's command from the plan's first instead of the one the plant holds there, changed nothing:
for the four-wheel car every wheel's use comes from the state it is in, and the acceleration `demand()` is asked for
does not enter it. That lookup was removed rather than left as machinery no test could hold to account.

`scripts/build.ps1`: 20 of 20 CTest suites pass in 483.2 s. `scripts/build.ps1 -Desktop`: 21 of 21 in 669.6 s, the QML
controls suite running 517 checks in 109.4 s with no QML warning; neither build reports a compiler warning.

With this build 303 of the 305 recordings in `out/runs` pass `--check-recording` (schemas 1 to 13). The two refused are
the same as in the last two sessions: `mpcc-dynamic-laps-mpcc`, cut off while it was written, and the exploratory
`p62x-dynamic-1789837332`, written before the dynamic car's schema 13 metadata was settled. Source fingerprint
`63e663bc93f8bc95fdbbbbbf243bf6158aede23992e725534f29c6bb3bd4291e`. Machine-readable evidence is in
`docs/evidence/tire-margin.json`.

## What this is not

Not a prediction the controller uses: MPCC still plans with a single-track model that knows axle loads only, and this
panel measures what its plan would ask of the four wheels, after the fact of the plan. Not a guarantee: it drives the
plant open loop along one plan, with nothing replanned, so it says what that plan implies, not what the car will do once
the next decision replaces it. Not available for the cars without four wheels, whose axles the slip bars already show.
Not a per-wheel margin inside the optimiser's constraints, which remain per axle.

## Consequences

- Phase 6.4 is complete: ribbon, solver status, solve time and the per-wheel margin are all on screen.
- The same function gives any future work a per-wheel reading along a plan, including the friction reserve a four-wheel
  car might be planned with.
- Phase 6 is finished. What the phase leaves behind is the planner's yardstick (decision 0026) and the solve-time tail.
