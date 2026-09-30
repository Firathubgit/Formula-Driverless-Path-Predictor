# 0031: Cone path planning and driving on cones

Date: 2026-09-24. Status: accepted for TrackWayFastPlan Phases 7.3 and 7.4, which it completes. Recording schema 15
becomes 16.

Phase 7.3 asks for FaSTTUBe's cone sorting, matching with virtual cones and path calculation ported to C++, first
reproducing the published Python on fixed cone sets saved as fixtures; for the local path to feed the planner instead of
the known line; and for FS-FEUP's Delaunay method, implemented from its description and never from its GPL code, to run
on the same fixtures as a cross-check, where the two disagreeing marks a good test. Phase 7.4 asks for an adapter
boundary test proving that, without a known track, the planner and controller receive observations only.

FaSTTUBe's `ft-fsd-path-planning` 0.4.3.2 (MIT) and netlib's FITPACK (Dierckx) were fetched with the user's approval;
the Python ran unmodified in an isolated environment under `.tools/fsd-python` (numpy 2.2.6, scipy 1.15.3, numba 0.61.2)
to write the fixtures. Only FS-FEUP's README was read.

## Decision

**The port reproduces the published planner, function by function.** `core/cone_path.*` keeps the name and order of
operations of each Python function it ports, down to NumPy's pairwise summation, a `-1` index reading the last element
and the order in which ties fall, because the point is to compare results rather than ideas. FaSTTUBe fits its path with
`scipy.interpolate.splprep`, so `core/fitpack.*` ports the FITPACK routines behind it, `parcur` and `fppara` with their
helpers, and SciPy's extrapolating `splev`, keeping the Fortran's labels so it reads against the original. Where
FaSTTUBe's own code divides by zero, on cones in an exactly straight line, the port treats the curvature as zero and
plans straight on. Both keep their licence notices in `THIRD_PARTY_NOTICES.md`.

**The fixtures come from this project's own perception.** `tools/cone_path_fixtures/make_fixtures.py` takes twelve frames
of a recorded lap of the preset laid to 5 m, the width FaSTTUBe is tuned for, and at each four cone sets in the car's
frame: the course's cones within 30 m, the same uncoloured, the frame's own simulated detections, and the right-hand
cones alone. It runs FaSTTUBe on each and writes every step's result beside the input: 44 fixtures, and 4 sets on which
FaSTTUBe raised `ZeroDivisionError`, kept as `.failed` files. Each fixture also carries the course's true centreline from
10 samples behind the car to 60 ahead, for evaluation only.

**A second method, written from a description.** `core/delaunay_path.*` triangulates the cones, takes the midpoints of
edges 2 to 7 m long that do not join two cones of one side's colour, walks from the nearest midpoint ahead through
neighbouring midpoints with three steps of lookahead, and fits the walk with the same FITPACK spline. Its
triangulation first used a finite super-triangle, whose far corners fell inside the circumcircles of thin triangles on
the hull and lost them; its own test found that, and it now uses one ghost vertex at infinity, which also handles the
exactly collinear cone rows of a straight.

**Agreement is not the yardstick; the true centreline is.** Two methods agreeing says nothing of which is right, so both
are measured against the centreline neither is given, over their first 15 m, and where they part the fixture is listed.
That is what found the one thing FaSTTUBe does on these cones that a reader might not expect: where a cone has no partner
within its 5 m search range, it places a virtual cone its minimum track width, 3 m, across, and on a 5 m course a cone's
partner often stands just beyond 5 m, so on frames of detections its path runs a metre off the middle. Told the course's
width it keeps to the middle. The port keeps FaSTTUBe's default, the published tuning and the Formula Student minimum;
the offset is recorded, drawn and named rather than tuned away.

**Driving on cones is driving an open path, not a lap.** Every track in the core is closed. A believed path is 20 m from
where the car was when the frame was sampled, and closing it into a lap would add a chord back to its start that the
projection could snap to on a straight, and a periodic plan that never comes to rest. `core/path_following.*` therefore
gives the open path its own versions of the known track's rules: `plan_open_path` plans speeds by the reference plan's
curvature and reachability rules without the seam, at rest at the last sample, since nothing is known beyond it;
`follow_open_path` is the policy along it, Pure Pursuit or MAP toward a lookahead point with the same gains and bounds,
aiming along the last segment beyond the end; `predict_open_path` rolls it forward as the known-track prediction does.
The planner's part on the known track, choosing among actions around stated blockages, has nothing to choose here: a
blockage is stated in the known track's stations, which the car does not know, so it is refused, and the one action is to
follow the believed path. MPCC, which plans along the known track, is refused too.

**The driver is given observations and nothing else.** `ConeDriver` (`adapters/simulation/cone_driving.*`) is the whole of
what decides on cones, and everything it is given is in its interface: the pose the car was placed at, where its map
begins, as a team knows where it put its car; pose and motion readings; and perception frames. With instruments the
readings are what they delivered; without them the simulation hands it the true state the instant it is, the ideal-state
assumption, recorded as such. Each frame's detections are placed on the map by the pose believed at the frame's
sampling, interpolated between pose readings, and FaSTTUBe's planner makes the believed path from them; a frame whose
path FaSTTUBe made from its previous one leaves the driver on the path it already follows. At each decision the driver
carries its latest pose forward to the decision's time at the latest speed and yaw rate, plans the believed path, and
follows it; before its first path it brakes to rest. The simulation feeds it through `feed_driver` alone, and its known
track then only judges the run: cross-track error, the corridor margin and laps.

**Phase 7.4's boundary is proved twice.** In `tests/cone_driving_tests.cpp` a second driver, fed only what the
simulation publishes of the car's measurements, or its state under the ideal assumption, and of the frames delivered,
decides exactly as the simulation's driver did, command for command and belief for belief. And every recording of a run
on cones proves it for itself: the loader runs a driver again on the recorded readings and frames alone and requires the
same beliefs, believed paths and commands. For that, telemetry, measurements and the frames' times are written at full
precision from schema 16.

**Schema 16.** `metadata.cone_driving` records the belief source and every cone path setting; `beliefs.csv` holds one row
per decision of a run on cones, the state believed, when the pose it was carried from was sampled and the believed path
followed; `believed_paths.csv` every path believed, with its frame, the pose that placed it and the corridor width it
believed. Each prediction of a run on cones starts at the belief, not the true state. A run on the known track writes the
two headers alone; every older schema still loads.

**The desktop draws the belief beside the truth.** The sensors card's DRIVE ON row offers Track or Cones while the car
perceives; Cones lays the preset to 5 m and starts the run over, Track restores it. In violet the scene draws the believed
path, the corridor believed around it and a ring where the car believes it is, over the true track and cones; the card
states how much of the path lies ahead, how old its frame is and how far the car believes itself from where it is.
Replay draws the recorded belief of the decision in force.

## Verification

`tests/cone_path_tests.cpp` (`fd_cone_path`, four groups). Over 30 SciPy fits of 2 to 40 points at three smoothings the
FITPACK port's knots match exactly and its curve within 1.1e-10 m, extrapolation included, and it refuses what SciPy
refuses. On the 44 fixtures the port's sorted cones, virtual cones, matches and fitted points are FaSTTUBe's exactly; its
path within 2.1e-10 m, parameter within 8.8e-11 m and curvature within 4.4e-11 1/m, except 70 samples on straights,
where FaSTTUBe signs a clamped 3000 m radius by a determinant that is a rounding error from zero and averages twelve
windows, differing by whole windows of 1/36000 1/m; and one set sampled a step apart, whose samples lie on FaSTTUBe's own
path within 0.41 mm. On the four sets where FaSTTUBe raised, the port plans straight down the middle. The triangulation is
Delaunay, counterclockwise and has 2n - 2 - h triangles on 60 random points. The cross-check: of 28 sets both methods
planned from, the port strays a median 0.140 m from the true centreline over 15 m and at most 1.05 m, Delaunay a median
0.143 m and at most 2.5 m; on the course's own cones 0.091 m and 0.143 m. Twelve sets part them by 0.5 m or more: the
port the closer on four, where Delaunay's walk ran down a boundary of uncoloured cones on a straight; Delaunay the closer
on the frames of detections, where the port's virtual cones stand 3 m across. Delaunay finds no start with one side's
cones alone (11 sets). On eight frames of detections the port strays a median 1.035 m with FaSTTUBe's 3 m and 0.077 m
told the course's 5 m.

`tests/cone_driving_tests.cpp` (`fd_cone_driving`, five groups): an open path's plan is at rest at its end and within the
curvature, reachability and speed cap rules, and malformed paths are refused; the policy pulls a car from 0.8 m off a
path onto it within 5e-6 m in six seconds and stops it 4 cm before the end. On the preset laid to 5 m with cones, the
kinematic car drives a lap on cones alone in 48.96 s with the ideal state and 49.14 s from its own instruments, never
outside the corridor's margin and at most 0.41 m and 1.40 m from the true centreline, with the cone memory decision 0032
added; from single frames it had been 49.2 s and 48.3 s, and 1.06 m and 1.01 m. The boundary group compares 1500
decisions over 30 s from each source, all identical. Driving on cones without perception, with a blockage, a blockage
stated while on cones, perception taken off while on cones and refused settings are all refused.

`tests/recording_tests.cpp` adds `cone_driven_runs_round_trip` and `rejects_cone_driving_violations` (24 groups): ten
seconds on cones from the instruments and six from the ideal state round trip with a belief per decision and every
believed path; a known-track run records the files empty. Nine tamperings are refused by name: a belief moved by a
millimetre, a belief forgetting its path, a belief dropped, a believed path moved or dropped, the belief source changed,
a cone path setting changed (the driver run again believes otherwise), cone driving in schema 15, and a believed path
followed on a known-track run. `tests/recording_contract.cmake` records three seconds on cones through the headless
runner, checks the source, validates it by a driver run again, and refuses driving on cones without perception before
any output.

Headless, on the preset laid to 5 m with `--perception 3`: on the known track the kinematic car laps in 33.52 s with no
invalid sample. On cones, planning from single frames (schema 16, `out/runs/p73-cones-*-a`), it lapped in 48.47 s with
the ideal state and 48.54 s with `--sensors 3`, at most 1.08 m and 1.15 m from the true centreline; the check reported 341
and 349 believed paths lying a median 1.02 m and 0.97 m and at worst 1.26 m and 1.94 m from the true centreline over
their first 15 m, the metre being the virtual cones' 3 m width; and about a fifth of the samples were flagged as a
braking transient, the car riding the limit of what it can see, braking toward the end of a 20 m path at the rate the
plan assumes while each new frame moved that end on. With the cone memory (schema 17, `out/runs/p73b-cones-*`) it laps
in 49.22 s and 49.31 s, at most 0.43 m and 1.04 m off; the believed paths lie a median 0.065 m and 0.344 m from the true
centreline, at worst 1.10 m and 4.03 m where measured poses placed a frame askew; 285 and 549 samples are braking
transients; and the judge times the lap from the start line, crossed at 0.3 s, at 48.91 s and 49.00 s, with no cone
down. Each check runs a driver again on the recorded readings and frames alone and finds the same at every decision.

## What this is not

Not SLAM: the driver places each frame by a pose it reads rather than estimates, and its cone memory (decision 0032)
is a record of those placements, not a map estimated with the pose. Not a first-lap exploration followed by a mapped second lap. Not a search for the fastest path through the
cones: FaSTTUBe's path is the midline of what it matched. Not a filter: nothing fuses the readings. Not collision with
cones: Phase 7.5 counts them.

## Consequences

- Phases 7.3 and 7.4 are complete. Phase 7.5 adds the competition's penalties and more courses.
- Recordings are schema 16, with telemetry, measurements and frame times at full precision.
- `fd_core` carries two ported components, with their notices in `THIRD_PARTY_NOTICES.md`.
- The instruments and the perception are no longer only recorded: on cones they are all the car decides from.
- Amended by decision 0032: the driver remembers the cones it has placed, which judging the Formula Student layouts
  showed it needs in every hairpin.
