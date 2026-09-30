# 0023: Each action's own speed profile, compared by time

Date: 2026-09-17. Status: accepted for TrackWayFastPlan Phase 5.3. Amends decision 0022: the lattice planner compares
its clear actions by estimated time instead of path cost, and its search allows a turn the car can brake for. Retires
decision 0004's limitation that alternatives do not re-derive curvature limits, for the lattice planner; the five-offset
planner keeps it. Changes AGENTS.md's planner invariant, as the plan's section 9 says it must at this phase.

Phase 5.3 of the plan: every candidate path gets curvature-derived speed limits from its own geometry and forward and
backward passes from the actual current speed, so comparing candidates compares time; the source is TUM's
`VpForwardBackward.py` concept with Phase 3's envelope. AGENTS.md's invariant is updated to state exactly what is
optimised, over what horizon and lattice resolution. Phase 5's last visible item belongs here: each action's path in its
own colour with its estimated time. The plan's section 10 asks for planning time as a distribution against the control
period before any real-time claim.

race_stack's copy of TUM's `graph_ltpl` was read for structure: `VpForwardBackward.py` (a forward-backward profile per
action path within a local G-G diagram, from the planned speed at the car to an end speed), and
`OnlineTrajectoryHandler.py`, which takes each action's end speed from the race line's speed at its end node, reduced by
how far that node lies off the race line. That code is LGPL-3.0; nothing was copied.

## Decision

**`make_speed_profile(track, config, request, envelope)`** in `fd_core` (`core/speed_profile.hpp`) returns the speed
profile along a path from the car: points at most 0.5 m apart along the reference, each with its station, its distance
along the path, the path's curvature and the planned speed, and the time the profile takes.

1. **Geometry.** The path is straight segments between its points, as a path intent pursues them. Its offset is
   interpolated between the points; its slope changes linearly between the middles of adjacent segments, so each change
   of slope is spread over the mean of the two segments, as the lattice search costs a turn. At the start the car's own
   slope turns into the first segment's over `start_turn_m`, at least that segment. The path's curvature follows from
   the reference's curvature and its rate and the offset's first and second derivatives, as the lattice's edges do.
2. **Limits.** Exactly the reference plan's: the grip fractions, or with the envelope the plan was made from, its share of
   the car's lateral, forward and braking limits; never above the top speed.
3. **Passes.** A backward pass from the end speed and a forward pass from the car's actual speed. A car already faster
   than the path allows brakes at the limit until within, and the profile says so.
4. **Time.** Each span covered at constant acceleration between its end speeds, along its own distance.

**`ControlIntent::speed_profile`** makes `compute_control` follow a profile while the car is within its stations: the
target speed is interpolated in speed squared, the feedforward is that span's acceleration; before the profile the target
is its first speed, beyond it the reference plan. Without a profile nothing changes, command for command.

**The lattice planner** (`choose_lattice_action`) gives every option it compares a profile:

- **Which options.** Both passes when the reference line enters a blockage, and when returning after one, the return
  along the lattice and rejoining the reference directly. The blocked reference line and the brake are not timed, and
  without a blockage in the way the car follows the reference plan with one prediction, as before.
- **The path as driven.** Pure Pursuit aims a pursuit distance ahead and cuts across whatever lies nearer, so the
  profile's path runs from the car straight to the lattice path's first point at least a pursuit distance ahead, then
  along its points; rejoining the reference directly runs to the reference a pursuit distance ahead. The car's own slope
  turns over that pursuit distance.
- **One horizon.** Every profile of a decision runs to the end of its farthest path, at least 60 m, and ends no faster
  than the reference plan there; beyond a path's own end it keeps its last offset.
- **Driven as timed.** Each option is predicted following its profile, so it is checked at the speed it would be driven.
- **The choice.** The clear option of least estimated time, the cheaper path on a tie within a nanosecond per second;
  but the previous decision's choice, the same action along a path or not, is kept while it is clear and no more than
  `lattice_switch_margin_s` (0.05 s) slower, about the error measured for an estimate over the planning horizon. The
  reason says by how much: "Passing left of bollard along the lattice; 0.078 s faster than passing right", or "kept within
  0.007 s of passing right". With nothing clear the car brakes toward the widest gap beside the blockage as before; on the
  way home, where there is no blockage left to brake for, it keeps the path it is on and says "no way home is clear".
- **The search.** A turn is refused only if it asks for more than the lateral budget even at the slowest speed the car
  can brake to by that station, since the profile will slow for it; its cost is still taken at the car's or the planned
  speed, whichever is higher, so the search keeps preferring paths drivable at speed. Within a pursuit distance of the car
  a blockage is widened by the half width only, since the pursuit lag that the wider margin allows for has not yet
  applied there.

**Recording, schema 11.** Each `decisions.csv` row gains `profile_points` and `estimated_time_s` ("none" without a
profile), and `profiles.csv` holds every profile's points. Loading checks that each profile starts at the car's station,
at distance zero and at its actual speed in the recorded sample the decision was made from; that stations and distances
increase and speeds are nonnegative; that one decision's profiles end at the same station; that each estimated time is
its profile's; that every lattice path has a profile and the five-offset planner makes none; and that the car took the
fastest clear action, or kept the previous decision's choice within the margin of it. A return decision may take either
the return or the direct rejoin. A line that is not clear may be taken only when no option is clear, and then only by a
decision that names no blockage in its way: with one named and nothing clear the car must be holding for it.

**Headless.** `--check-recording` counts the decisions that took the fastest of several clear actions by estimated time.
A run with alternatives prints the wall-clock time of each step that weighed them, median, 95th percentile and longest,
against the control period; it is a measurement on this host and is not recorded.

**Desktop.** Each timed option is labelled with its estimated time 15 m along its predicted line, in its action's
colour, framed when chosen and faded when rejected: "Left 3.01 s". Two paths that have barely separated would anchor
their labels on the same few pixels, so a label that would cover an earlier one sits above it instead. The decision panel
carries the margin in words ("Passing left of corner bollard along the lattice; 0.125 s faster than passing right"),
which takes a second line, so in the compact window the reason is set tighter and the path cost keeps one line. Replay
shows the recorded times. Capture: `docs/assets/action-times.png`.

## Found by building it

- **A path point just ahead of the car is not a turn the car makes.** The first profiles followed the lattice path from
  the car's own point, so a committed point 0.7 m ahead became a 0.068 1/m kink; the profile then demanded braking the car
  could not follow, the prediction rejected the pass as exceeding the reference envelope, and the car braked. It
  alternated between passing and braking four times on the lane scenario. Profiles now follow the path as Pure Pursuit
  drives it.
- **Where that pursued path starts matters.** Running it to an interpolated point exactly a pursuit distance ahead left a
  short segment whenever that point fell just short of a node: 1.2 m short, it read as a 0.023 1/m turn, the straight
  bollard's right pass looked 0.1 s slower than the left, and the car swapped sides and passed 3.5 m out at 0.759 grip.
  Cutting instead to the path's first point at least a pursuit distance ahead, it passes right throughout at 0.706 and
  reaches 100 m 0.05 s earlier than decision 0022's run.
- **Near ties swapped sides from one tick to the next** (right, left, right past the straight bollard). Measured against
  what the car then drove, an estimate is within about 1%, so a switch needs a margin; 0.05 s. Doubling it to 0.1 s only
  delayed the corner switch by two control ticks, because the gain grows quickly as the car nears the blockage, from
  0.064 s to 0.113 s in 40 ms, and changed no arrival time at 100, 150, 200 or 300 m.
- **Abandoning a pass the moment the reference line comes clear is dangerous.** Once the car is far enough across, the
  reference line no longer touches the blockage, and the decision becomes a return. With the return unable to continue
  the pass path, the four-wheel car on 80% of its envelope turned back at 17 m/s entering the corner, left the corridor
  for 495 samples and slid at 0.89 rad of rear sideslip. A return now continues whatever path the car was on: the same
  run stays 3.67 m off at worst, inside the corridor, at 0.064 rad. Rejoining the reference directly is also profiled
  now, so a car that must turn back sharply slows for that turn.
- **The search's own margins had to follow the profiles.** A turn is refused only if it is beyond the budget even at the
  slowest speed the car can brake to, since the profile slows for it; and within a pursuit distance of the car a blockage
  is widened by the half width alone, because the pursuit lag the wider margin allows for has already happened there.
  Without the second the car found no path for seven decisions in the middle of a pass, and braked.
- **Losing grip leaves no clear way home, and the reference is the wrong place to go.** The desktop's own replay fixture
  found it: a run with the lane blocked and grip dropped to 0.45 at 6 s stopped loading, because 216 decisions took a line
  that was not clear while the contract said a decision with alternatives takes a clear one. Out beside the blockage with
  grip gone, no way home is clear, and the planner was falling back to the first option, rejoining the reference, which is
  the sharper turn of the two and the one nothing had recommended. It now keeps the path the car is on, and the contract
  now says what it meant: a blocked line may not be taken while a clear one was offered, and with nothing clear the car
  must be holding for the blockage it named.
- **Profiles are cheap; rollouts are not.** One profile costs 0.010 ms with the grip fractions and 0.079 ms with the
  four-wheel car's envelope. Planning a blocked decision for that car takes 10.0 ms at the median (95th percentile
  12.4 ms, longest 21.8 ms) against 16.1 ms for the five-offset planner, because both are dominated by predicting each
  option.
- **Two labels at one place are one label.** Where the car is still far from the blockage, the two paths' first 15 m are
  the same line, so the labels landed on each other and the capture showed only the one drawn last. A label that would
  cover an earlier one now sits above it, and the check says neither covers the other. The same capture showed the
  margin's second line of text pushing the decision panel into the run metrics of the compact window, 4 px past them.
- **A label in a 3D scene needs the camera, not the car.** Bound to the car's pose, the estimated-time labels were mapped
  before the camera that follows it had moved, and stayed where the car used to be; anchored halfway along the prediction,
  30 m ahead, they sat above the top of the view. They are now bound to the camera's own scene transform and anchored
  15 m along each line.

## Verification

`tests/speed_profile_tests.cpp` (`fd_speed_profile`, 10 groups, 0.5 s), whose oracles are constant-acceleration
kinematics and the circle beside the reference:

- on the straight at the top speed the profile holds it, 60 m in 3 s, with samples every half metre;
- from rest it accelerates at the longitudinal grip fraction, and its time is that of constant acceleration;
- inside the first corner, an 18 m arc, the cap is the circle's: on the reference, and 2 m inside or outside, the
  curvature of a 16 m or 20 m circle, with the path's length scaled by the same ratio;
- braking into that corner follows the backward pass exactly over more than 40 samples, and the profile ends no faster
  than asked;
- a car already faster than the path allows brakes at the limit until within, and says so;
- a path stepping 2 m over 10 m turns at its segments' middles, curvature 0.02 over (1 + slope squared) to the 3/2, its
  length longer than its stations by its slope, and the turn caps the speed;
- along the reference the profile reproduces the reference plan's curvature limits, with the grip fractions and with the
  four-wheel car's envelope, at 34 points each;
- the controller follows a profile and the reference plan beyond it, command for command, and bad requests are refused.

`tests/lattice_planner_tests.cpp` (`fd_lattice_planning`, 16 groups, 8.6 s) adds: the faster side is taken when both are
clear, and before the first corner the inside path costs 2.16e6 against the outside's 1.18e6 yet is 0.078 s faster and is
taken; a near tie keeps the previous choice and a larger difference switches; every compared option has a profile from the
car's station and speed to the decision's common horizon, its time is that profile's, and its prediction targets it; the
first decision to pass estimated 3.210 s to its horizon and the car took 3.25 s; the four-wheel car finishes its pass
at the corner without leaving the corridor; and with grip dropped to 0.45 beside the lane blockage, where 220 decisions
find no clear way home, the car keeps the path it is on through every one of them and says so. Four mutations were run and
reverted: dropping the offset's second derivative fails the turn group, choosing by cost fails the corner comparison and
the grip comparison, a return that commits only to its own action leaves the corridor for 495 samples, and falling back to
the first option with nothing clear turns the car back to the reference and fails both the planner and the recording
group.

`fd_recording_playback` (16 groups) round-trips schema 11, including the run that loses every clear way home, and rejects
seven more tampered variants: a profile not starting at the recorded state, an estimated time disagreeing with its profile,
a lattice path without a profile, profiles of one decision ending at different stations, a choice that was neither the
fastest clear action nor the previous one within the margin, a blocked line taken while a clear one was offered, and a
decision that drove on past the blockage it named with nothing clear. `fd_recording_contract` checks a schema 11 run of
each planner, the reported times and the refusals.

`fd_desktop_controls` drives the real controls at the 1100x700 minimum: replay labels a recorded pass with its recorded
time; the five-offset planner estimates none; and with a bollard stated entering the first corner both ways past it are
timed and labelled by side, exactly one framed as the choice and no slower than the other by more than the margin,
neither label covering the other, and the decision panel, whose reason now carries the margin in seconds, still ends
above the run metrics.
`scripts/build.ps1`: 19 of 19 suites in 194.7 s. `scripts/build.ps1 -Desktop`: 20 of 20 in 264.7 s, with 447 checks of
actual QML controls, bindings and 1100x700 captures and no QML warnings.

Runs with `fd_headless` into fresh `out/runs/final2-*` directories, the kinematic bicycle with grip fractions unless
stated; offsets are the rear axle's beside the blockage; peak grip over the whole run, with its station:

| Scenario | Lattice planner | Five-offset planner |
| --- | --- | --- |
| Clear corridor, one lap | 33.52 s, peak 0.686 | the same telemetry, byte for byte, and the same as before speed profiles |
| Lane blocked, one lap | passes at 2.33-2.53 m, 33.50 s, peak 0.684 at 497 m, within 0.27 m from 60 m past; 22 decisions compared actions | passes at 3.07-3.08 m, 33.24 s, peak 0.995 at 161 m, still 3.32 m off at 142 m |
| Whole corridor blocked | stops at 61.78 m | the same telemetry, byte for byte |
| Bollard on the straight | passes right at 1.83-2.02 m, peak 0.706; 93 compared | passes right at 1.64 m, peak 0.953 |
| Bollard entering the first corner, one lap | passes right, then left on the inside at 3.01-3.09 m, 33.49 s, peak 0.850 at 160 m, within 0.18 m from 60 m past; 201 compared | passes left at 2.82-3.11 m, 33.26 s, peak 0.998 at 161 m |
| Four-wheel car, 80% envelope, lane blocked | passes at 2.55-2.81 m, peak 0.977 at 138 m, within 0.48 m; 60 compared | passes at 3.12-3.24 m, peak 1.000 at 20 m, still 3.19 m off |
| Four-wheel car, 80% envelope, corner bollard | passes right, then left at 3.47-3.60 m, peak 1.000 at 198 m, within 0.43 m; 120 compared | passes right at 1.55-1.75 m, peak 1.000 at 163 m |
| Dynamic car, soft front, MAP, lane blocked | passes at 2.52-2.76 m, peak 0.752 at 138 m; 73 compared | passes at 2.17-2.91 m, peak 0.946 at 81 m |

No run has an invalid sample, and each passes `--check-recording`. The lattice planner's peak grip is the lower on every
blocked scenario, and its peaks fall in corners the car would take anyway, not in the passes. Each first decision to pass
estimated its horizon within 2% of the time the car then took: 3.406 against 3.410 s, 3.405 against 3.410 s, 3.210 against
3.250 s, 3.341 against 3.370 s and 3.170 against 3.230 s. Planning a blocked decision takes 1.4 ms at the median for the
kinematic car (95th percentile 2.4 ms, longest 3.3 ms) against 2.3 ms for the five-offset planner, and 10.0 ms for the
four-wheel car at the corner (95th percentile 12.4 ms, longest 21.8 ms) against 16.1 ms; the control period is 20 ms and
these are wall clock on this host, not a real-time claim. All 215 recordings in `out/runs`, schemas 1 to 11, load under
this build. Source fingerprint
`030c34c3cf7248e186a6d35b369ce272d384c1c28e2db628dd1687651ea6915a`. Machine-readable evidence is in
`docs/evidence/speed-profiles.json`.

## What this is not

Not the fastest path through the lattice: each action's path is still the cheapest by geometric cost, and only those few
are timed; a slower-looking path that is faster is found only if it is some action's cheapest. Not a time-optimal line
either: the profile's limits are the reference plan's, applied to a curvature estimated from straight segments pursued
by Pure Pursuit, and the car's actual motion is checked by the prediction, not guaranteed. The horizon ends where the
farthest path does; beyond it every option is assumed to rejoin the reference plan. Not a real-time claim: planning is
measured as a distribution on one host, and nothing here is bounded. The five-offset planner is unchanged and still
compares station gain.

## Consequences

- Phase 6's MPCC can take a chosen action's path and profile as its reference and horizon.
- A future search could cost time directly, rather than timing each action's cheapest path.
- Moving obstacles, and blockages stated on the racing line, remain for later phases.
