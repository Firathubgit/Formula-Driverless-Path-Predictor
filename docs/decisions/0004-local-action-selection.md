# 0004: Choose among local lines before optimising one

Date: 2026-09-14. Status: accepted for the current milestone; replaced as the default by decision 0022's lattice
planner on 2026-09-17, and kept selectable for comparison as the five-offset planner. Its limitation that alternatives do
not re-derive curvature limits is retired for the lattice planner by decision 0023, which gives every compared path its
own speed profile; the five-offset planner keeps the limitation.

Decision 0003 made the car's next three seconds visible, but the prediction only rolled
the one policy forward. Its future points never influenced the first action, so nothing
in the system actually chose anything. The user's direction is a car that reacts to its
situation a few metres at a time and is fast without crashing. That needs a real choice,
and a choice is only demonstrable when there is something to choose between.

## Decision

`choose_local_action` in `core/local_planner.*` evaluates candidate lines at every control
decision and returns what it picked together with everything it rejected. Candidates are
lateral offsets from the reference centerline. Each one is predicted with the existing
rollout, then accepted or rejected against the model envelope and against the scenario's
stated blocked regions. Among the clear candidates it takes the greatest predicted advance
along the reference within the horizon, preferring the line nearest the reference on ties.
When nothing is clear it holds the reference line under a speed cap derived from the
remaining room, which is the explicit slow-or-stop outcome.

An `Obstruction` is a blocked interval of station and lateral offset, stated by the
scenario. It is ground truth handed to the planner. Nothing detects, classifies or tracks
it, and describing one as a detected pedestrian would claim a capability that does not
exist. Contact is tested at the predicted control points against a sampled rectangular
footprint inflated by a vehicle half-width allowance: neither continuous nor swept-body
collision detection, so a clear result is not a safety guarantee.

`ControlIntent` carries the offset and the speed cap into `compute_control`. A
default-constructed intent reproduces the previous behaviour exactly, so every existing
caller and test is unchanged. Two details are derived rather than tuned. Moving sideways
by `y` within a pursuit distance `L` costs roughly `2*v^2*y/L^2` of lateral acceleration,
so an offset aims far enough ahead that the shift fits the lateral grip budget instead of
demanding an impossible snap onto the new line. And because a proportional speed controller
needs a standing error to command deceleration, it runs a predictable amount faster than
any cap it is given; that amount is subtracted from the commanded cap so the speed the car
actually holds is the curve that reaches zero at the clearance.

## What this is not

This selects among feasible lines. It is not minimum-time optimisation. Offsetting does not
re-derive curvature speed limits for the shifted line, so an alternative is never predicted
to be faster merely for being wider or tighter; without a blockage the reference line wins
and exactly one candidate is evaluated, leaving ordinary driving at its previous cost. A
racing line that is genuinely fastest needs per-offset curvature limits and a time-optimal
objective, which is the next step and is a different problem, as TUM's global racing
trajectory work and F1TENTH's planning stack both separate.

There is still no perception, no sensor model, no mass sensitivity, no tire slip, no wheel
locking and no ABS. Claims about driving at the adhesion limit need tire dynamics first.

## Consequences

Schema 1 records actual motion and reference plans, not the alternatives or the choice
between them. The run directory now carries `scenario_obstructions` so a recording states
the scenario it ran against, which older recordings simply lack and read back as empty.
Replay draws the recorded blockages but reports that the decision itself was not recorded,
rather than recomputing today's choice and presenting it as history. Persisting selected
trajectories and their identities is the next recording-contract change.

The driving view now draws the rejected alternatives beside the chosen line. That is the
point: a choice is only visible next to what it was chosen over.

Evaluating alternatives costs about five extra rollouts on the control tick where a
blockage is in range. Measured on the reference host that is roughly 3 ms for a clear
decision and 16 ms for a blocked one, against a 20 ms control period. The simulation is
not wall-clock bound, so this changes no result here, but it is too close to the period to
claim real-time headroom. The nearest-point projection is linear in track samples and sits
in that hot path; reducing it is the identified next performance step.
