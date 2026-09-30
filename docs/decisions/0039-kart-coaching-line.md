# 0039 — Poster coaching line and independent prediction visibility

2026-09-28. The user asked for Gokartcentralen's recommended line painted along the
whole track in the Xbox-driven You mode, following their supplied poster, and switches
for that line and the prediction ahead of the car.

The desktop bundles a display-only trace of the poster's coloured stroke in
`tracks/gokartcentralen-goteborg-guide.csv`. It uses the existing track's coordinate
frame and published scale (decision 0038). `tools/tracks/trace_kart_guide.py` reproduces
it from the privately supplied image, normalized to the original 1500 × 2000 tracing
frame. The old tracer locates the start; the existing track and its recorded scale are
kept. Re-tracing from the resized high-resolution image changes some barrier snaps, so
that new centreline is only a registration check and never replaces the installed one.

The coloured stroke is sampled along the track's normals at quarter-metre intervals;
short occlusions and the footbridge's markings are interpolated, offsets smoothed, and
the dominant colour gives gas, coast or brake. The 0.4 m ribbon is inset where the
poster's varying asphalt width puts it too near the existing constant 6 m corridor's
edge. Its 1600 samples form a closed loop, 0.04 m above the road, with continuous edge
normals across the start. Qt resources bundle it, so driving needs neither the poster
nor Python. It is static coaching advice, not a speed-dependent brake assistant or a
solved optimum. It never reaches the planner, driver, plant, recording or judge.

Green means gas, **blue means coast**, and red means brake, as on the poster. This is
labelled separately from the prediction's green/yellow/red planned acceleration,
constant speed and braking, whose existing core deadband is unchanged.

Settings places Coaching line, Prediction line and Motion arrow switches directly under Gokart.
Layers offers the same independent choices. All four coaching/prediction combinations are available while
paused or driving. Kart mode initially shows coaching alone; the lattice also defaults
off there, as does the separate white motion arrow (actual velocity and heading, not
a prediction). Both can be enabled independently. The computed comparison line is hidden
to avoid a second recommended route. Kart visibility choices last for the app session independently of the ordinary
track's preferences. Leaving kart mode removes the guide and restores ordinary
prediction/lattice visibility. Resetting the run does not change the switches.

Verification is in the desktop UI suite: actual QML controls, all four visibility
combinations, toggling while driving, unchanged plant state while toggling paused,
closed ribbon edges, every vertex inside the conditioned corridor, all three colours,
and leaving the mode. Captures show settings, the full lap and the drive. Measured
results are recorded in `docs/STATUS.md` after the checks run.
