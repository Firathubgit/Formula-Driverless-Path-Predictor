# 0032: Judging a run, and Formula Student courses

Date: 2026-09-25. Status: accepted for TrackWayFastPlan Phase 7.5, which it completes, and with it Phase 7. Recording
schema 16 becomes 17.

Phase 7.5 asks for PacSim's competition logic, cone hits, off course, unsafe stop, lap and sector timing, and for the
Formula Student layouts in PacSim's `tracks` to be imported as additional courses once their provenance is checked; the
plan's visible part is the penalties in the timing panel. PacSim (MIT) is on this machine. `src/competitionLogic.cpp`,
`include/competitionLogic.hpp`, `config/mainConfig.yaml` and the track files were read for structure and for the rules
they encode; nothing was copied, and nothing was downloaded.

## Decision

**A judge, in the core, that nothing driving reads.** `Judge` (`core/competition.*`) observes the car's true pose every
fixed tick on a `Course`, the track whose corridor is the course's boundary, its cones and its timekeeping gates, by a
discipline's `CompetitionRules`, and lists what it sees as `TimingEvent`s. It is the evaluation's, like the corridor
margin: the simulation observes it after every step and after a reset, and neither the planner, the policy, MPCC nor
the cone driver ever reads it.

**PacSim's rules, computed exactly.** The rules are PacSim's: a cone down or out costs 2 s, the car off course 10 s each
time, and more than 8 s of it is a DNF; after the finish the car must stop within 30 m of the line within 33 s, else a
trackdrive pays 10 s for an unsafe stop and an autocross is a DNF; the start must come within 60 s, an autocross within
300 s and a trackdrive's first lap within 300 s and all within 1000 s. A trackdrive is ten laps, an autocross one. The
geometry is this project's own: a gate is crossed when the rear axle passes from behind its line to ahead of it between
its ends, so the first crossing of the first gate starts the clock and each later one times a lap, and any other gate
splits a sector; off course is all four wheels' contact points outside the corridor; a cone is down when the body's
rectangle overlaps the cone's 0.228 m base, by the separating axis test rather than PacSim's point-in-polygon checks, and
each cone counts once. The footprint keeps the simulation's wheelbase with a Formula Student car's 0.6 m half track and
overhangs, a body 1.4 m wide from 0.6 m behind the rear axle to 0.6 m ahead of the front.

**Every run is judged, and every recording proves it.** A course without a layout has the cones laid along its corridor
(decision 0030) and gates across the start line and at a third and two thirds of the lap. Schema 17 records the rules,
the footprint, the gates, where the course came from and a fingerprint of its cones and gates in `metadata.competition`;
the course's cones in `cones.csv` whether or not the car perceived them; and every event in `timing.csv`. The loader
judges the recorded samples again on the recorded course by the recorded rules and requires the same events, and
checks that a course "laid along the track" is the one this build lays along the recorded track. The track, the samples
and the track's length are written at full precision so the judge decides alike again. The headless runner now ends a
lap when the judge has timed it, not only when the car has come round, since a layout's start line may stand ahead of
where the car is placed, and a DNF ends the run.

**PacSim's Formula Student layouts are read where they are, never copied.** Their provenance: PacSim's repository is
MIT-licensed, but the layouts are traces of real competitions' courses (FSG, FSE, FSS, FSCZ, FSI, FSO), whose origin
the repository does not state. So `load_pacsim_course` reads a file from wherever it is, `--course FILE` in the headless
runner and the desktop, and nothing of them enters this repository; a recording names the file and keeps a fingerprint.
`course_from_layout` traces a closed course from one: the centreline through the midpoints between each left cone and
the right lane, conditioned (decision 0019) to start beside the start pose, with the distance from each sample to either
lane as its corridor edges; the cones as laid; the timekeeping gates, left end first by the direction of travel, or the
start line and thirds laid as for any course where a layout has none (FSG24 and FSE24). Classes `big-orange` and
`small-orange` are read as orange; `unknown` and `invisible` cones are left out.

**A Formula Student car to drive them.** Their hairpins, 9 m across at the outside by the rules, bend the traced
centreline to radii of 2.8 to 4.1 m, tighter than the default 2.6 m wheelbase and 0.55 rad lock can turn (4.24 m); the
runner refuses such a course and says so. `configs/formula-student.cfg` gives a Formula Student car's 1.53 m wheelbase
and 0.6 rad lock, 2.25 m, and Pure Pursuit's aim at 2.5 m plus a quarter second rather than 4 m, because the longer aim
cut corners into the cones: on FSG23 nine cones at 4 m, one at 3 m, none at 2.5 m; on FSE24 four, none, none; for about
0.3 s of lap. The desktop's course takes the same geometry.

**The cone driver remembers its cones.** Judging the layouts on cones found what the preset had hidden: planning from a
single frame of a sensor that sees 60° either side, the driver lost its path in every hairpin, where the cones it needs
stand beside and behind it, followed the last path straight off the course and came to rest there, a DNF on four of
thirteen layouts even from the ideal state. FaSTTUBe's planner is meant to see the cones around the car, so the driver
now keeps a `ConeMemory`: each detection placed within 1 m of a remembered cone refines it, its position the mean of its
placements and its colour the one most reported, any other is a new cone, and a cone more than 30 m from the car is
forgotten. It is a memory of placed detections, not a map estimated with the pose. Schema 17 records it in
`metadata.cone_driving.memory`; a schema 16 run, which planned from each frame alone, reads back with none and is run
again the same.

**The desktop's timing panel.** Either side of the circuit map the desktop states the last lap as the judge timed it and
the penalties, with what they were for and the verdict, OFF COURSE, FINISHED or the DNF's reason, and draws each cone
knocked down as a red base with a cross, live or as recorded, replay reading only the recorded events up to the cursor.
The map leaves them 80 px either side. `--course FILE` opens a layout, with its cones and gates and the Formula Student
geometry; the preset's racing line, solved for the preset, is not offered on it.

## Verification

`tests/competition_tests.cpp` (`fd_competition`, seven groups): at 10 m/s on the preset laid to 5 m the clock starts the
tick the car leaves the line, two laps of 50.755 s each finish a two-lap trackdrive, and three sectors of 16.915, 16.92
and 16.92 s sum to each; a body 0.7 m either side touches a cone whose centre is 0.7 m aside once, for 2 s, and passes one
1.2 m aside; all four wheels 0.1 m beyond the edge is 10 s off course, one wheel 0.1 m inside is not, and eight seconds
off is a DNF after which nothing is judged; driving on 30 m past the finish is an unsafe stop in a trackdrive and a DNF in
an autocross, at rest near the line a safe stop; no start in 60 s is a DNF; an oval traced from a layout written in
PacSim's format has corridor edges a mean 0.050 m and at most 0.315 m from its cones' 2 m, the chords between cones 3 m
apart on a 5.5 m inner radius, starts beside the start pose, keeps its gate's left end first and laps clean; a position
that is not a number is refused naming its line; bad rules, a body short of its front axle and a course with no start
line are refused.

`tests/recording_tests.cpp` adds `judged_runs_round_trip` and `rejects_judging_violations` (26 groups): twelve seconds
of an autocross with three cones left across the lane record the rules, the course's source, its gates, every cone and
four events, the three hits 6 s; a run given no course is judged as a trackdrive on the course laid along its track, its
lap clean. Seven tamperings are refused by name: a hit moved in time, a hit dropped, a cone moved off the lane, the cone
penalty changed, the laps required changed, a gate moved, and `timing.csv` missing. A cone driver's memory changed in
metadata is refused as a driver run again believing otherwise. `tests/recording_contract.cmake` checks a headless run is
judged as a trackdrive on the course laid along its track, its clock starting at 5 ms, with the course's cones recorded.

Headless, every PacSim Formula Student layout as an autocross with `configs/formula-student.cfg`, on the known track, on
cones from the ideal state and on cones from `--sensors 5`, all with `--perception 5`: all 39 runs finish and every
recording validates, each judged again. On the known track 10 of 13 are clean, 5 cones down in all and never off course;
on cones from the ideal state 8 are clean, 12 cones and 3 excursions; from the instruments 4 are clean, 35 cones and 4
excursions. Before the cone memory, 4 of the 13 on cones were DNFs from either state. The runs and their lap times are in
`docs/evidence/competition.json`.

## What this is not

Not a race director: nobody waves a flag, and no run is re-run for a DNF. Not the handbook: PacSim's times and distances
are taken as its configuration states them, and there is no skidpad or acceleration event, whose layouts are open and
which this project's closed tracks do not describe. The unsafe stop is judged, but runs end at the finish line, so the
runner reports it as not judged; the judge's own tests exercise it. The traced courses are this project's reading of
the layouts, not the layouts themselves: a centreline between cone lanes, conditioned, with the corridor a polyline of
cones apart.

## Consequences

- Phase 7.5 is complete, and with it Phase 7: sensors, perception, cone path planning, the boundary, and judging.
- Recordings are schema 17; every older schema still loads. Every run carries its judging.
- A headless lap now ends when the judge has timed it as well as when the car has come round.
- The cone driver keeps a memory of the cones it placed; decision 0031's single-frame driver is schema 16's.
