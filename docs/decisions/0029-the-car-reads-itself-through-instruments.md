# 0029: The car reads itself through instruments

Date: 2026-09-20 to 2026-09-23. Status: accepted for TrackWayFastPlan Phase 7.1, which it completes. Recording schema
13 becomes 14. Independent of decision 0028, which is the showroom's own scope.

Phase 7 asks for decisions made from delayed, noisy, incomplete observations instead of ground truth. Phase 7.1 is its
first step: the instruments themselves, with rate, a dead-time queue and Gaussian error on pose, speed, wheel speeds,
the inertial unit and steering, in a new `adapters/sensors/`, and the seed recorded. PacSim (MIT), which the plan names
as the source, was already on this machine and was read for structure only: its sensors hold a rate, a queue and a
normal error per channel, sample when their own turn has come round rather than on every tick, and release a sample
once its dead time has passed. Nothing was copied, and nothing was downloaded.

## Decision

**The instruments are a seam of their own, beside the vehicle seam.** `fd_sensors` (`adapters/sensors/`) depends on
`fd_core` and on nothing else. A `SensorChannel` is a rate, a dead time and the standard deviation of the error on each
value it carries; a `SensorSettings` is the five channels the car has and the seed every error is drawn from. A
`SensorSuite`, observed with the plant's true state, samples each channel when its own turn has come, gives each value
its error, queues the sample, and delivers it once its dead time has passed. What `measurements()` returns is the newest
sample of each channel that has arrived, and the time it was taken from the plant. The defaults follow PacSim's own
`sensors.yaml` where the channels correspond: GNSS at 20 Hz behind 50 ms, steering, wheel speeds and the inertial unit
at 200 Hz behind 5 ms.

**Nothing detects, classifies or tracks anything.** These are the car's own instruments reading its own state. Cones,
and a planner that sees only what is measured, are Phases 7.2 to 7.4.

**Nothing they measure drives the car.** The simulation observes them at every fixed tick and exposes what they
delivered; it never reads them itself. The plant still advances on its own state, every decision is still made from it,
and `--check-recording` says so in as many words. This is deliberate: Phase 7.1 is the instruments, and moving the
controller onto them is Phase 7.4's boundary, where it can be tested as the change it is. Until then a run measures
itself beside driving itself, and the difference is visible.

**The seed is the run.** Each channel draws from its own stream, derived from the seed and the channel, so adding a
channel or switching one off does not disturb another's errors. Two runs of one seed measure alike, value for value; a
fingerprint over the settings and the seed identifies them as the steering table and the envelope are identified.

**Instruments start their cadence where they are fitted.** The first observation sets each channel's clock, so a suite
fitted part way through a run samples from there instead of catching up on every sample it would have taken since the
run began. A reset run starts its instruments again from the seed.

**Schema 14 records what was measured and how to measure it again.** `metadata.sensors` holds the channels, the seed and
their fingerprint; `measurements.csv` holds one row per recorded tick, with what each channel had delivered by then and
when it was sampled; and `telemetry.csv` gains the acceleration the car achieved along its body forward axis over the
tick, which is what an inertial unit reads and what the loader needs to run the instruments again. Loading a recording
does run them again, from the recorded seed over the recorded states, and requires the same stream: a measurement
nudged by a millimetre is refused, and so is a seed changed without its fingerprint. A run without instruments records
none, as every older run did.

## Verification

`tests/sensor_tests.cpp` (`fd_sensors`, eight groups) measures the seam's five claims and the simulation's one. A 20 Hz
channel takes its first sample one period in and forty over two seconds, whatever the tick, and what it reports is the
truth at the moment it sampled, not at the moment it was read. A sample taken at 0.010 s behind a 20 ms dead time
arrives at 0.030 s carrying what was true then, and once the queue has run through the value stays about a dead time
behind. Over 39 997 samples of a standing car the position error has mean 4.3e-5 m and standard deviation 0.0299 m
against the 0.03 m the channel states, the yaw error 0.0199 rad against 0.02, and the speed error 0.0199 m/s against
0.02. Each channel draws from its own stream: switching the pose channel off, changing the wheel speeds' error or the
inertial unit's rate leaves the steering's errors exactly as they were, and with every channel sampling together no two
first errors are alike. One seed measures a run twice over, value for value; another seed measures
differently, and the fingerprint changes with either the seed or an instrument. Rates, dead times and errors outside
their ranges are refused, as is an observation that goes backwards.

The seam's own claim is that watching the car does not move it: over two thousand ticks a dynamic car with instruments
and one without hold identical positions, speeds and yaw rates. What the instruments made of that run: over ten seconds
the pose read was up to 95 ms old, 72.5 ms on average, and out by up to 1.96 m at 10.6 m/s, with the speed out by up to
0.56 m/s. Nearly all of that is age, not noise: 95 ms of a 20 Hz channel behind a 50 ms dead time, at speed, is metres.

`tests/recording_tests.cpp` adds `measured_run_round_trips` and `rejects_measurement_violations` (20 groups). Eight
seconds of the dynamic car measured with seed 2026 records one measurement per tick, the pose delivered on 1581 of 1601
of them, up to 95 ms old and 1.96 m out, with the wheel speeds out by up to 0.37 rad/s; the recorded truth stays apart
from the recorded measurement. Five rules are broken one at a time and each is named: a value nudged by a millimetre
("is not what these instruments measure from the recorded seed and states"), a seed changed to 2027, a sample dated
after the tick that read it, a missing row, and an achieved acceleration that is not the speed gained over the tick
before. `tests/recording_contract.cmake` drives the same path through the headless runner: a run without `--sensors`
records the header alone and no metadata block, a run with `--sensors 99` records the seed and each channel's rate and
validates, `--sensors 100` measures the same run differently, and a seed that is not whole is refused before anything
is written.

A one-lap run of the dynamic car with `--sensors 7` (`out/runs/p71-measured-lap`, 6760 ticks): the pose was delivered
on 6740 of them, up to 95.0 ms old and 72.5 ms on average, out by up to 1.974 m, the speed by up to 0.609 m/s and the
yaw rate by up to 0.007 rad/s. `--check-recording` reports all of it, with "the car drove on its true state" at the end
of the line. The same lap without instruments (`out/runs/p71-true-lap`) wrote a `telemetry.csv` and a `decisions.csv`
byte for byte identical to the measured lap's: 33.795 s either way, with only `measurements.csv` and the metadata
between them.

The desktop offers the instruments in a card of their own beneath the tire card, SENSORS with True state and
Measured, while paused, and when they are fitted draws WHAT THE CAR MEASURES there: the pose's age, how far out it is,
the speed read with its error, and the yaw rate's error, with a caption saying the car drives on its true state. The
choice first sat in the column of run choices on the left; the compact window's check that a passing decision ends above
the run metrics failed with it there, so it moved beside the tires, where there was room. The UI verification fits them,
drives four seconds, and checks that the age is under 110 ms, that the position is out by between 0.05 and 8 m, that the
readouts agree with the bridge, that the card ends above the bottom bar in the compact window beneath the four-wheel
car's tire card and setup button, that a fresh run keeps its instruments, and that taking them off hides the readouts.
The desktop's G-G point now reads the achieved acceleration the simulation and the recording carry, instead of working
it out again from the speed.

Five mutations were run and reverted. Delivering a sample before its dead time had passed failed two groups of
`fd_sensors`; sampling at every observation instead of at the channel's rate failed one; writing the true pose where the
measured one belongs failed `measured_run_round_trips`; and dropping the loader's comparison of the recorded pose with
the pose measured again failed `rejects_measurement_violations`. The fifth, giving the speed channel the pose channel's
stream, passed every group at first: the independence check then watched only the steering. The check that no two
channels' first errors are alike was added, and it fails that mutation.

`scripts/build.ps1`: 21 of 21 CTest suites pass in 400.3 s. `scripts/build.ps1 -Desktop`: 23 of 23 in 576.0 s, the QML
controls suite running 539 checks in 127.9 s with no QML warning; neither build reports a compiler warning. With this
build 305 of the 307 recordings in `out/runs` pass `--check-recording` (schemas 1 to 14); the two refused are the known
ones, `mpcc-dynamic-laps-mpcc` and `p62x-dynamic-1789837332`. Source fingerprint
`40bab28c858e354f3b9aed81af362549d203f3facb54ca8fac24673d70039a69`. Machine-readable evidence is in
`docs/evidence/sensors.json`, and the capture of the card in `docs/assets/sensors.png`.

## What this is not

Not perception: nothing here sees anything outside the car. Not an estimator: there is no filter, and what a channel
delivers is its last sample, not a fusion of several. Not yet in the loop: the car drives on its true state, and the
known track and ideal state remain the recorded assumptions. Not a mounted accelerometer: the inertial unit reads the
body-frame accelerations the vehicle seam reports, with no lever arm, no gravity component and no road grade. Not a
model of dropout, bias drift, scale error or quantisation, none of which is modelled.

## Consequences

- Phase 7.1 is complete. Phase 7.2 (mock cone perception) and 7.4 (the boundary that keeps the map from the controller)
  can be built against this seam, and 7.4 has something concrete to move the controller onto.
- Recordings are schema 14. Every older schema still loads and reports what it did not record.
- A controller that reads measurements will inherit a pose that is up to 95 ms and about two metres out at racing speed.
  That is the size of the problem Phase 7.3 and the rest of Phase 7 have to solve.
