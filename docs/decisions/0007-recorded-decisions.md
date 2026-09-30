# 0007: Record every decision, so replay shows the choice instead of guessing it

Date: 2026-09-15. Status: accepted. TrackWayFastPlan Phase 0.3.

Decision 0004 made the car choose among lines and drew the rejected ones beside the chosen
line, but schema 1 recorded only motion and reference plans. Replay therefore had to say "no
recorded decision" and hide the prediction ribbon, so the project's central claim, that its
decisions are visible, did not survive being saved. Recomputing the choice at replay time was
rejected in decisions 0002 and 0003: it would show what today's build would decide, presented as
what the recorded car decided.

## Decision

Schema 2 adds two files to the run directory and changes nothing else.

- `decisions.csv` has one row per evaluated option of every decision: time, option index,
  whether it was selected, lateral offset, speed limit (`none` when the reference governed),
  clear, within the model envelope, advance along the reference, the option's reason and
  validity reason, and the decision's holding flag, blocking identifier, reason and first
  requested and applied command. Decision-level columns repeat on each option row and must agree.
- `trajectories.csv` has every option's predicted points: time, position, speed and achieved
  acceleration. Yaw, steering and cumulative distance are not recorded, and the reader does not
  invent them.

`RecordingWriter` records a decision whenever `Simulation::decision_count()` advanced during a
step, so exactly the decisions that governed recorded motion are kept. The decision made on
reset or scenario setup is superseded by the first step's own decision before any motion and is
not recorded. A scenario changed after recording started, or a step recorded late enough to skip
a decision, is refused.

`load_recording` cross-checks the new files against the rest of the run, cheapest first:

1. Each decision selects exactly one option. A holding decision's choice has a speed limit, a
   non-holding choice among several lines is clear, a clear line is within the envelope, and a
   blocking identifier names a stated scenario obstruction.
2. Decision times are exactly the telemetry sample times on control ticks plus the times an
   accepted parameter change replanned between ticks, no more and no fewer.
3. Every option's trajectory starts at the recorded state the decision was made from, advances
   on the fixed-tick grid and ends at the three-second horizon, and every option has one.
4. Every telemetry sample's requested and applied command equals the first command of the
   decision that governed that step.

`Recording::decision_at(t)` returns the decision a sample at `t` shows, by the same rule as
parameter revisions: a decision made at `T` governs the motion after `T`, so the sample at `T`
still shows the previous one, and the sample at time zero shows the first. `Playback::decision()`
follows the cursor. In the desktop replay the decision panel, the rejected lines and the
prediction ribbon come from that recorded decision, labelled as recorded.

Schema 1 directories still load, report `Decisions: not recorded (schema 1)`, and replay exactly
as before. A schema 1 metadata file beside decision files is refused as altered.

## Deliberately not done

TrackWayFastPlan suggested reserving telemetry columns for body velocities, yaw rate and per-wheel
speed, slip, load and force. They are not reserved. The kinematic plant has none of these, so the
columns would hold invented zeros, which the recording contract has refused since schema 1
("quantities the simulation does not record are deliberately absent"). The dynamic plant will add
them with a schema 3 branch, which the versioned reader now makes a routine change.

## Consequences

Decisions are large. Measured on the reference host with this build:

| Run | Decisions | Predicted points | Run directory |
| --- | ---: | ---: | ---: |
| One clear lap, `schema2-scenario-baseline` | 1676 | 253,076 | 19 MB |
| Lane blockage, `schema2-scenario-avoid` | 1662 | 497,394 | 37 MB |
| Full-width blockage, 30 s, `schema2-scenario-stop` | 1500 | 1,284,255 | 84 MB |

Loading the one-lap test fixture took 0.63 s, and the recording test suite went from about 2 s to
about 13 s. The stop scenario is large because a stationary car re-evaluates six identical lines
every 20 ms. Recording fewer points or deduplicating repeated decisions would reduce this at the
cost of a more complex contract; neither is done until the size is a measured problem.

Replay still reconstructs a recorded run and does not evaluate a different policy. A recorded
decision is evidence of what the recorded build chose, including when a newer build would choose
differently.
