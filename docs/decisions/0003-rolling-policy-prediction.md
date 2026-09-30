# 0003: Predict from actual state before adding local optimization

Date: 2026-09-14. Status: accepted for the current milestone.

The driving view previously drew a cropped section of the track centerline, colored by
the full-lap speed envelope. It could not show the standing-start acceleration, tracking
correction or speed transient that the current controller would actually produce. The
user clarified that the main line should show the car's immediate intended motion and
change as its situation changes. This brings prediction forward in the milestone order.

`make_local_trajectory` is a pure C++ module returning a three-second prediction from
actual rear-axle pose, speed and steering. It rolls the existing Pure Pursuit and speed
feedback policy forward with the same fixed-step bicycle model. Simulation applies its
first command and discards the remaining future when it replans at the next control tick.
An accepted grip event also replans immediately; an event between regular control ticks
shortens the first hold so the prediction preserves the existing controller schedule.

The known-track periodic speed envelope remains the reference. It supplies corner and
braking context beyond the visible horizon: three seconds of displayed prediction does
not mean three seconds of sensor visibility. Horizon duration is fixed for this slice;
distance varies with actual speed and acceleration. Green/yellow/red encodes achieved
average longitudinal acceleration on each outgoing predicted segment, using the existing
0.15 m/s² deadband. The terminal point has no outgoing acceleration.

Prediction uses copied states and cannot advance the live plant. The primary ribbon
uses the predicted positions, with expired points removed and the current actual position
as its start. The track map retains the reference separately. Validity checks flag speed
above the reference envelope, sampled rear-axle track margin violations and per-tick
combined grip demand. They are diagnostics, not collision checking or a safety proof.

This is a prediction of the baseline policy, not a new optimizer. Its first action is
the same baseline action; future trajectory samples do not yet influence that choice.
There is no obstacle perception, path selection, mass sensitivity, wheel slip, locking,
ABS or maximum-performance tire model. Sudden infeasibility is exposed rather than
hidden by a fictitious feasible line. No new optimizer or robotics dependency is needed.

Schema 1 records actual motion and reference-plan revisions, not these rolling futures.
Replay therefore hides the live prediction and states that no prediction was recorded.
It continues to show recorded state and reference data; it never runs today's controller
to fabricate a recorded intent. A future recording revision must persist the selected
trajectory, generation time, configuration identity and validity together.

Next, add one controlled local-planning experiment: a scenario-supplied blocked section,
bounded candidate paths within the track, evaluation under the implemented model, and
an explicit slow/stop outcome when no candidate meets constraints. Verify changed first
commands and closed-loop outcomes before introducing minimum-time optimization. Treat
these objects as synthetic observations, not detected pedestrians. Add tire dynamics
before making claims about racing at the adhesion limit or wheel locking.

The separation between global minimum-curvature/minimum-time planning and local control
is also illustrated by [TUM's global racing trajectory project](https://github.com/TUMFTM/global_racetrajectory_optimization)
and [F1TENTH's planning/control stack](https://f1tenth-planning.readthedocs.io/en/latest/),
inspected on 2026-09-14. They are design references; their code was not imported.
