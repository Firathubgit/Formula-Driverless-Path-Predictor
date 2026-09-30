# 0002: One validated recording contract, cursor playback, no plant advance

Date: 2026-09-13. Status: accepted for the current milestone.

The first milestone wrote run files directly from the headless runner. Nothing read them
back, so their structure was asserted only by a CMake script matching a few substrings.
Replay needs a reader, and a reader that trusts its input silently turns a corrupt or
foreign run into a convincing-looking animation.

## Decision

`adapters/recording/` owns schema 1 as a single contract with three parts. `RecordingWriter`
produces the run directory, `load_recording()` reads and cross-checks it, and `Playback`
moves a cursor over the result. The headless runner and the Qt application share all three.
The library depends only on the C++ standard library and the existing simulation adapter;
it is not a Qt, ROS or rosbag2 integration and does not claim to be one.

Loading validates the files against each other, not merely each file's own syntax. Metadata
configuration must pass the same `validate_config` the simulation uses. Every plan row must
match the corresponding `track.csv` sample. Each recorded sample's revision must agree with
the timing in `events.csv`, its grip must equal the configuration chain built from the initial
configuration plus accepted events, and its limiting index and reason must match the plan
revision it names. The summary is recomputed from telemetry and compared. Any violation
throws and names the rule; a partially readable directory is never returned.

Because plan speeds cannot be checked against anything else in the directory,
`plan_reproduction_error_mps()` re-runs this build's planner over the recorded track for each
revision's configuration and reports the largest speed difference. `--check-recording` prints
it beside the recorded source fingerprint, so a plan edited by hand or produced by different
algorithm source is visible as a number rather than assumed correct.

Playback is a cursor, not a second simulator. Seeking selects the last recorded sample at or
before the requested time and then adopts exactly that sample's plan revision and its
configuration. A backward seek therefore restores the earlier plan and hides later parameter
changes without re-deriving anything. The live plant is never stepped in replay mode, and the
Qt timer feeds wall time to the cursor instead of to the simulation.

## Consequences

A change accepted at tick boundary T governs motion after T, so the sample recorded at T still
carries the old revision and the sample at T plus one tick carries the new one. The reader,
the playback cursor and the tests all encode that one rule; it is the project's definition of
when a parameter change becomes visible, and changing it would break recorded runs.

Recordings stay plain CSV and JSON that any tool can read, at the cost of file size and parse
time. Adding a recorded field means extending the header, the reader, the writer and the
telemetry check together; the header comparison is exact, so an older file is rejected rather
than silently misread. A future rosbag2 adapter should translate this contract rather than
replace it, and a future schema 2 needs its own version branch in the reader.

Replay reconstructs a recorded run. It does not evaluate a different controller: recorded
state describes the path the recorded policy drove, so a changed policy needs closed-loop
simulation instead. That limitation is a property of recorded-input replay, not of this format.
