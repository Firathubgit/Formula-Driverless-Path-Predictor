# 0022: A lattice planner with an action set

Date: 2026-09-17. Status: accepted for TrackWayFastPlan Phase 5.2, and amended the same day by decision 0023: each
action is now timed by its own speed profile and the car takes the clear one of least estimated time rather than least
cost, a return may continue the path the car is on or rejoin the reference directly, and the search's turn and near-car
margins follow the profiles. Amends decision 0021: the offline cost's reference deviation term saturates at 40000 per
metre, 4 m off the reference, instead of 10000. Replaces decision 0004's five-offset planner as the default; that planner
stays selectable for comparison.

Phase 5.2 of the plan: from the node nearest the actual state, prune edges that intersect obstructions or leave the
corridor, search the cheapest path per action within a planning horizon, and return straight or follow, pass left, pass
right, and brake, with behaviour selection after race_stack's state machine. When no action is feasible, follow the gap:
steer to the widest gap across the corridor and brake, instead of holding the reference and stopping. Obstructions stay
scenario ground truth. The plan's closed-loop tests: the two scenarios of `tests/planner_tests.cpp` still avoid and stop;
with a clear track the chosen path converges to the reference; on the avoidance scenario peak grip use is lower than the
five-offset planner's for the same clearance; planning time is measured against the control period. Phase 5's visible
items that belong here: each action's path in its own colour, the chosen action highlighted, and its reason and cost
breakdown in the decision panel. Each action's estimated time needs Phase 5.3's per-action speed profile.

race_stack's copy of TUM's `graph_ltpl` was read for structure: `Graph_LTPL.py`'s `const_path_seg` commitment, and
`GraphBase.py`'s virtual goal node with ForzaETH's `w_virt_goal=10000.0` in `ltpl_config_offline.ini` (TUM's default is
200). So were race_stack's `state_machine/src/states.py` (global tracking, trailing, overtaking, follow the gap only) and
its follow-the-gap controller's README. `graph_ltpl` is LGPL-3.0; nothing was copied.

## Decision

**`choose_lattice_action`** in `core/local_planner.*` makes each control decision under the new
`LocalPlannerMode::lattice`, the simulation's default. It predicts the reference line first.

1. **Straight.** Without stated blockages, or with the reference line clear and the car within 1 m of it, it follows
   the reference with that one prediction, command for command as the five-offset planner does.
2. **Pass left, pass right.** When the reference line enters a blockage it searches the lattice twice, for the cheapest
   path that passes the nearest blockage on its left and the cheapest on its right, predicts each as a path intent and
   takes the clear one of least cost.
3. **Return.** With the reference line clear of every blockage but the car more than 1 m off it, or the reference line
   failing only the model envelope, as after a pass, it returns along the cheapest lattice path that ends on the
   reference, if that is clear; otherwise it follows the reference.
4. **Brake, following the gap.** When no pass is clear it brakes for the blockage under decision 0004's speed cap while
   aiming at the centre of the widest gap across the corridor beside it, less the half width, or along the reference
   when there is none. It always returns something.

**The search.** It starts from the node nearest the car's offset on the first layer at least a metre ahead and runs over
the layers within 60 m, or just past the blockage passed. It is a dynamic programme over edges, so a path's cost can
depend on the edge before it.

- **Geometry.** A path is straight segments between its nodes, from the car's own station and offset, and the controller
  pursues it: `ControlIntent::path` gives the offset at the pursuit target's station, interpolated. The lattice's quintic
  edges supply only their offline cost.
- **Admissible edges.** A segment must stay inside the corridor less the 0.9 m half width, and clear of every blockage
  widened by the half width and, before it, by a pursuit distance, because Pure Pursuit reaches a path's offset about a
  pursuit distance after the path does. A pass must also keep to its side of the blockage it passes.
- **Cost.** Each edge's offline cost (decision 0021); plus, at every change of slope, `50000 × share² × span`, where
  share is the lateral acceleration the turn asks, at the larger of the car's speed and the planned speed there, over
  the lateral budget, and span the mean of the two segments; plus `10000 × |offset|` of the last node, TUM's goal cost with ForzaETH's weight. A turn asking more than
  the budget is not allowed, including the one back parallel to the reference at the end.
- **Lateral budget.** The envelope fraction of the performance envelope's lesser lateral limit when the plan is made from
  it, otherwise the lateral grip fraction of the grip.
- **Commitment.** Given the choice the previous decision made, a new path for the same action keeps that path's points
  within a pursuit distance of the car when they are still clear and continues from the node it had reached there, as
  TUM keeps `const_path_seg`. Without it the plan kept deferring the turn it had begun.

**Recording, schema 10.** `metadata.json` names the local planner. Each `decisions.csv` row gains the option's action,
its lattice path's point count and its six cost terms, and `paths.csv` holds every path's stations and offsets. Loading
checks that every action is one the recorded planner takes, that a pass follows a path and names its blockage, that only
the held choice brakes and every held lattice choice does, that path point counts match, and that each path starts at
the car's projection at the recorded state and runs forward. A return names no blockage, so alternatives without one are
accepted only when the choice is a return. Older runs load as planned by five offsets. Reasons are refused if they
contain a separator, so the gap's reason reads "toward the widest gap at 3.0 m left".

**Headless.** `--local-planner lattice|five-offsets` (default lattice), printed with each run and by
`--check-recording`, which counts passing, returning and braking decisions and path points. It is refused with
`--write-lattice`, which it could not affect.

**Desktop.** The simulation lays the lattice along its track when a blockage is stated. Under PLANNER beside the
scenario controls, Lattice or Five offsets starts a fresh run with the same scenario while paused. The decision panel's
title is the chosen action in its colour (straight grey, pass left violet, pass right pink, brake red), RETURNING for a
straight action along a path, with where a pass crosses the blockage and how many actions were rejected; below the reason
the chosen path's cost and its terms. Every other evaluated action's predicted line is drawn in its colour, darker when
it was rejected, and the legend names them. Replay shows the recorded actions and costs and names the recorded planner.

## Found by building it

- **A path the planner found clear was not clear once driven.** Pure Pursuit reaches a path's offset about a pursuit
  distance after the path does, so the first paths, clear by their own geometry, put the predicted car into the blockage.
  The search now keeps a pursuit distance clear before a blockage as well as the half width.
- **Driving the lattice's own edges took the car past its grip envelope.** Following the quintic edges, which leave and
  reach every node with its node slope, a pass bent at every layer; with a cost dominated by reference deviation, the
  cheapest path turned late and hard. Straight segments between nodes, a turn cost on the lateral acceleration each change
  of slope asks, and refusing turns beyond the lateral budget made the pass smooth.
- **Without commitment the plan kept deferring.** Each new plan, from a car a little further on, again found it cheaper
  to turn later, then braked when turning was no longer possible, then passed again: the car alternated between brake and
  pass and peaked at 0.986 grip. Keeping the previous path within a pursuit distance, as TUM does, ended that.
- **Turning straight back was as abrupt as the five-offset planner's.** After a pass, the reference line was clear, so
  the car turned straight back to it. Returning along the lattice when more than 1 m off fixed that, but the first
  version then passed a blockage far ahead whenever the reference line failed only the envelope; passes are now searched
  only when the reference line enters a blockage.
- **The deviation cost saturated at 1 m**, decision 0021's 10000 at 10000 per metre, so a return path ending 2.5 m off
  cost the same per metre as one ending 1 m off, and the car stayed 2.5 m off. The saturation is now 40000, 4 m.
- **The gap's reason held a comma**, which recordings refuse in a CSV cell, so it reads "at 3.0 m left".
- **The desktop's compact window had no room** for a planner row in the left column, the chosen path's cost and a longer
  legend: the decision panel ran into the run metrics, and in replay the legend ran under the scrubber. The planner row
  moved beside the scenario controls, the count moved onto the decision's title line, and the legend wraps; UI checks
  now measure all three, and the legend's check was shown to fail with a label long enough to need a third row.
- **Cost.** Planning a blocked decision took 1.15 ms with the lattice and 2.45 ms with five offsets on the reference
  host, against a 20 ms control period (measurement only; 1.29 and 2.05 ms in earlier runs of the same test).

## Verification

`tests/lattice_planner_tests.cpp` (`fd_lattice_planning`, 11 groups, 2.1 s):

- the pursuit target takes a path's offset at its own station, interpolated, and a path runs across the seam;
- a clear reference line is followed with exactly one prediction;
- a blocked reference line is passed on its clear side, with a cost equal to its edges' terms, its turns and its goal;
- past a bollard from -0.5 to 1.5 m both sides are clear, and the cheaper, right at 1.19e6 against left at 2.30e6, is
  taken;
- a full-width blockage brakes along the reference; a trailer reaching 2.0 m, too close to pass, brakes toward the
  centre of the gap left of it, (2.9 + 4.1) / 2 m; every reason is recordable;
- a plan a quarter second later keeps 2 points of the previous path within 8.1 m;
- a car 2.5 m off the reference with the blockage behind returns along a path ending on it, and one on it just follows;
- a lattice for another track, and no lattice with blockages, are refused;
- closed loop, the plan's tests: the car passes the lane blockage at 3.00 m and is within 0.267 m of the reference from
  60 m to 200 m past it, having returned along the lattice; it stops for a full-width blockage at 61.77 m at 0.02 m/s;
- on that pass the lattice planner's peak grip use is 0.577 against the five-offset planner's 0.780, at 1.15 ms against
  2.45 ms per blocked decision.

`fd_recording_playback` (14 groups) round-trips a lattice scenario, 700 decisions of which 147 pass, 30 return and 384
brake, with 1947 path points, and a five-offset scenario whose schema 9 downgrade still loads; it rejects fourteen
tampered or out-of-schema variants, each naming its rule. `fd_recording_contract` checks schema 10 runs of both planners,
their reports and the refusals. `scripts/build.ps1`: 18/18 suites passed, 230.16 s of tests. `scripts/build.ps1 -Desktop`: 19/19 suites passed, 316.24 s of tests, with 427 QML checks, up from 404, and no runtime warnings; the UI check took 61.7 s of its 120 and `fd_recording_playback` 49.3 s, so its limit is now 120 s.

Runs with `fd_headless` into fresh `out/runs/planner-*` directories, the kinematic bicycle with grip fractions unless
stated; offsets are the rear axle's beside the blockage; peak grip over the whole run, with its station:

| Scenario | Lattice planner | Five-offset planner |
| --- | --- | --- |
| Clear corridor, one lap | 33.52 s, peak 0.686 | the same telemetry, byte for byte |
| Lane blocked (62-68 m, -4.5 to 1.0 m), one lap | passes at 2.90-3.00 m, 33.56 s, peak 0.684 at 497 m (a corner), within 0.27 m from 60 m past | passes at 3.07-3.08 m, 33.24 s, still 3.32 m off at 142 m, peak 0.995 at 161 m turning back |
| Full width blocked | stops at 61.78 m | the same telemetry, byte for byte |
| Bollard (-0.5 to 1.5 m) | passes right at 2.40-2.52 m, peak 0.686 at 213 m (a corner) | passes right at 1.64 m, peak 0.953 at 93 m |
| Four-wheel car, 80% envelope plan, lane blocked | passes at 2.53-2.78 m, peak 0.973 at 138 m where its clear run peaks at 0.977; 0.777 from 20 to 130 m against 0.747 clear | passes at 3.12-3.24 m, peak 1.000 at 20 m turning out, 0.987 from 20 to 130 m, still 3.19 m off at 140 m |
| Dynamic car, soft front, MAP, lane blocked | passes at 2.61-2.85 m, peak 0.746 at 138 m where its clear run peaks at 0.747; 0.648 from 20 to 130 m against 0.560 clear | passes at 2.17-2.91 m, peak 0.946 at 81 m |

No run has an invalid sample, and each passes `--check-recording`. After the four-wheel car's five-offset pass, its
reference line's prediction passed a tire's peak on 790 decisions, so it kept an offset. All 149 recordings in `out/runs`,
135 of them schemas 1 to 9 from earlier sessions, load and validate with this build. Source fingerprint
`b0c9087f3d373e774b5630a9919e4cc53ddf7ade737fa5eefca427d0a356a742`. Machine-readable evidence is in
`docs/evidence/local-planner.json`.

## What this is not

Not a fastest line: every path is driven at the reference plan's speed, and paths are compared by a geometric cost, not
by time, until Phase 5.3 gives each its own speed profile; AGENTS.md's planner invariant stands until then. Not
collision checking: contact is still tested at predicted control points against a rectangle widened by the half width,
and the search's own clearance is a straight-segment test at 0.5 m steps. Not a behaviour for moving obstacles or for
more than the nearest blockage at a time: a pass keeps clear of every stated blockage but is chosen for the nearest one.
Obstructions are still stated across the centreline, so neither application combines them with the racing line, and
the lattice the planner searches is the centreline's. The weights are ForzaETH's scaled values and two chosen here (the
turn weight and the 1 m return threshold), not tuned across tracks or cars.

## Consequences

- Phase 5.3 can give each action's path its own speed profile, since every action already has a path and a prediction.
- The racing line needs obstructions in its own coordinates before blockages can be stated on it.
- Decision 0021's evidence costs were computed with the old deviation limit; its lattices' counts are unchanged.
