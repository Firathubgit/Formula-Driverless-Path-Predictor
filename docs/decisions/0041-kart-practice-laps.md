# 0041 — Personal kart lap clock

2026-09-28. At the user's request, You → Gokart shows a white current-lap
clock at the top right, with a smaller Previous line directly below. The circuit
card adds the session best and lap number. Times display minutes, seconds and
hundredths. Previous is empty until a complete lap, never a partial run time.

The existing conditioned track starts at the poster's checkered line and points
toward turn 1. Keep this start pose and the track geometry. A new two-row
checkered marking spans its 6 m corridor at that same start/finish gate. Restart
returns the rear axle to that position and releases the controls.

`fd::PracticeLaps` is a passive core timing helper, not a driver or another plant.
The desktop feeds it the true before/after poses at each fixed simulation tick
while a person drives the kart. The first throttle above 5% starts timing at that
tick's beginning, even if Play was pressed earlier. Accelerator input also starts
the paused run while the practice clock is waiting. Simulation time governs the
clock, so pausing freezes it and render timing never advances it separately.

A full lap passes eleven intermediate corridor-width gates in course order and
then crosses the start gate in the forward direction, between its ends. Crossing
time is linearly interpolated within the tick. That completed duration becomes
Previous; the new current lap begins at the same crossing instant. Reverse
crossings, start-line rocking and missing/out-of-order checkpoints do not count.
This is checkpoint-based practice timing, not shortcut detection or a new judge.
The existing competition judge and penalties still run independently. Practice
times are unpenalized and continue beyond its finish or DNF; best is a session
best across the selected assistance settings, not a certified track record.

Restart and setup rebuilds clear the unfinished lap and rearm the gas start while
keeping completed times and count. Changing driver rearms the current lap.
Leaving kart mode clears the session; nothing is persisted to disk. Autonomous
and replay displays retain their existing competition timing. Human drives
remain unrecordable.

Verification covers throttle start, no idle time, ordered/full laps, forward-only
crossings, missed/outside gates, interpolation, previous versus best, restart and
history clearing. Continuous synthetic pose fixtures traverse two full laps of
the actual conditioned kart track at its centre and 2 m either side. These
fixtures test timing, not vehicle physics. Desktop checks exercise gas start,
the initial spawn/marker, visible clocks, partial-lap handling, pause and restart.
Final build and render evidence is in `docs/STATUS.md`.
