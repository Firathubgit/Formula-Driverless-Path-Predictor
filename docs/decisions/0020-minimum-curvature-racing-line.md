# 0020: A minimum-curvature racing line, solved with OSQP

Date: 2026-09-17. Status: accepted for TrackWayFastPlan Phase 3.4 and Phase 3's remaining visible items.

Phase 3.4 asks for a minimum-curvature racing line in a new `fd_raceline` target outside `fd_core`, with OSQP. The source
is Heilmeier et al. 2019 (DOI 10.1080/00423114.2019.1631455) as TUM's `main_globaltraj.py` uses it through `opt_min_curv`
and `iqp_handler`: a lateral shift per sample, curvature linearised in the shifts, a quadratic cost on curvature, box
constraints from the track width less the vehicle width and a margin, and re-linearisation until the curvature error is
small. FS-FEUP's OSQP formulation in `smoothing.cpp` was read for its structure. Output: a `Track`-compatible line with
curvature and width to each boundary, so the controller, prediction and planner consume it unchanged. Tests: it stays
inside the corridor; its curvature is within the steering limit; its estimated lap with Phase 3.2's plan is shorter than
the centreline's for the same car; a straight track gives a straight line. Phase 3's visible items still open were the
centreline and racing line drawn together, and an estimated lap time for each.

TUM's optimiser is LGPL-3.0 and its helper package, which holds `opt_min_curv`, is not on this machine; FS-FEUP is
GPL-3.0. Nothing was copied from either: the formulation below is written from the paper's equations.

## Decision

**OSQP v1.0.0, vendored as source.** No solver sources were available locally (FS-FEUP's `ext/osqp` submodule is
empty). With the user's approval, `scripts/bootstrap-osqp.ps1` downloads the OSQP v1.0.0 source release and qdldl v0.1.8,
its linear-system dependency, from the osqp organisation's GitHub release and tag, checks both archives against pinned
SHA-256 hashes before extracting anything, and places them in `.tools/osqp`, which Git ignores. OSQP is Apache-2.0 and
the plan's own choice, the licensing table allows it in a separate planning library, and Phase 6's MPCC can reuse it,
so one QP solver serves both. The root `CMakeLists.txt` builds the static `osqpstatic` in C with its shared library, demo,
unit tests, printing, profiling and interrupt handling off, and points OSQP's own CMake at the local qdldl with
`FETCHCONTENT_SOURCE_DIR_QDLDL`, so configuring never clones anything. `fd_raceline` links it privately; `fd_core` still
has no dependency. Without the sources, CMake says how to get them and skips `fd_raceline` and its tests. The headless
runner then refuses `--line racing` before writing anything, naming the bootstrap script. The desktop disables Racing
line and names the script too, but a desktop build without OSQP was not run. The MSVC build of OSQP needed no patch.

**The quadratic program** (`raceline/raceline.hpp`, `make_minimum_curvature_line(conditioned, config, options)`). The
input is a conditioned track (decision 0019), which has continuous curvature and a left normal at each sample. Each
iteration takes the current line's samples, spacing `ds` and curvature `kappa_i`, and a shift `alpha_i` along each left
normal, positive to the left as elsewhere in this project; TUM's normals point right.

- Linearised curvature of the shifted line: `kappa_i + (alpha_{i-1} - 2 alpha_i + alpha_{i+1}) / ds^2 + kappa_i^2 alpha_i`,
  periodic around the loop.
- Objective: the summed squared linearised curvature, plus `1e-12 / ds^4` times the squared shifts. That regularisation
  only makes the minimum unique where curvature is indifferent, such as along a straight.
- Box constraints: each sample's position after the shift stays within the corridor's edge, measured from the
  original centreline at its projection, less `vehicle_half_width_m` (0.9 m, the local planner's margin) and `margin_m`
  (0.6 m). A corridor narrower than both is refused, naming the station.
- Steering rows: each linearised curvature within 0.98 of `tan(max_steering_rad) / wheelbase_m`, the kinematic turning
  limit, so the refitted line still meets it. An infeasible program is refused as "No line within the corridor meets
  the steering limit of ... 1/m (radius ... m)"; the finished line is checked against the limit again.
- OSQP: absolute and relative tolerance 1e-7, polishing on, at most 100000 iterations.
- Re-linearisation: the shifted samples are conditioned again at the same spacing with a 0.001 m rms allowance and
  corridor checks off. A racing line hugs an edge, so it is judged only for crossing itself. The loop repeats until no
  sample moves more than `converged_shift_m` (0.02 m), at most `max_iterations` (30). A line that did not converge is
  returned marked as such, and both applications say so.

**Width to each boundary.** `Track` gains `left_edge_m` and `right_edge_m`: each sample's distance to the corridor's left
and right edge, empty for a centreline, whose edges are half the width away. For the racing line they are the
corridor's edges at the sample's projection onto the centreline, less its signed offset. `validate_track` checks that
they come in pairs, have one value per sample and are finite and non-negative. `corridor_at(track, projection)` gives
the edges at a projection, and `within_corridor(track, projection, margin)` replaces the old half-width rule. For a
centreline it computes exactly the old rule. The simulation's track check, the prediction's corridor check and the local
planner's candidate offsets use them, so the controller, prediction and planner run the racing line as they run any
track. `ControlResult` now carries its `reference` projection for those checks.

**Recording schema 9.** `track.csv` adds `left_edge_m,right_edge_m`. A centreline writes half the width on both sides and
reads back with empty edges, so earlier schemas and centreline runs replay unchanged. The recording contract checks that
a racing line run records edges that differ from half the width.

**Headless.** `fd_headless --line racing [--line-margin M]` conditions the preset, sampled every metre, or the
`--track` file, and solves the line before anything is written. It prints the iterations, convergence, solve time,
length, peak curvature and the estimated lap on the line and on that centreline under the chosen plan, then drives and
records the line. `--line-margin` without `--line racing` is refused. So is `--obstruct` with it: stated blockages are
positions across the centreline, and the line has none of its own yet.

**Shared helpers in `fd_core`.** `estimated_lap_time(track, plan)` covers each span, the closing one included, at
constant acceleration between planned speeds. It replaces three copies, in two test suites and the headless runner.
`condition_track(track, options)` conditions a track from points taken along it every `input_spacing_m`, keeping its
name and width. It gives the same result as a centreline file of those points, and the headless runner, the desktop and
the tests use it for the preset.

**Desktop.** A LINE row under PLAN offers Centreline and Racing line while paused. The racing line is solved once, on a
worker thread, from the preset conditioned every metre; it depends only on the track, wheelbase and steering limit, none
of which the desktop changes. Choosing it starts a fresh run on the line, keeping grip, car, setup, steering and speed
plan. It is refused, with the reason, while running, in replay, before the solve finishes, and with stated blockages.
While blockages are stated the row is hidden, and the scenario buttons refuse a blockage on the racing line. Beneath the
row, the plan in force's estimated lap is given for the racing line, for the preset the car drives on the centreline,
and for the smoothed centreline the line was solved against: "Plan lap: racing line 26.20 s, centreline 30.29 s
(29.40 s smoothed)" under 80% of the envelope. The smoothed figure separates what conditioning the preset's curvature
steps gains (0.9 s) from what the line gains (3.2 s). The road and kerbs are drawn between each track's own edges. The
line the car is not driving is drawn beside the one it is, as a ribbon in the scene and a thin line on the circuit map,
and named in the legend: the racing line in light blue on a centreline run, the centreline in grey on a racing line
run. Replay shows the recorded track and its edges only; the recording does not carry the centreline a line was solved
against.

## Found by building it

- **Driving the line from rest spun the inside wheel.** The line starts on a bend out of the last corner. With decision
  0018's bound, driving was limited by the car's straight-line forward capacity. Flat out from rest on that curve, the
  unloaded inside wheel spun past its peak for 226 samples in the first 50 m, and for 429 with the centre of gravity
  0.6 m high. **This amends decision 0018:** with an envelope, `compute_control` now bounds driving by
  `forward_limit(v, v^2 kappa_ref)` at the nearest reference sample. Braking keeps `braking_limit(v, 0)`, so the car can
  always slow down. The launch spin is gone: 0 samples, and 0 with the tall car. At 12 m/s in the preset's 18 m corner,
  the driving bound is 4.28 m/s² against 7.72 straight ahead. On the preset, 7 of decision 0018's 8 envelope runs record
  byte-identical telemetry. The whole-envelope run differs by one sample and has the same 1091 invalid samples. The
  conditioned centreline runs differ by at most 0.06 s a lap, with the same invalid counts, except the whole envelope
  there (4586 against 4747). Grip fraction and kinematic runs are untouched.
- **A 0.3 m margin leaves the corridor.** Pure Pursuit tracks the line within about 0.45 m, and with 0.3 m the car's rear
  axle crossed the corridor's margin in 250 samples. At 0.6 m it does not, so 0.6 m is the default. It
  is a measured reserve for this controller, not a derived one.
- **The grip fraction plan cannot drive the line.** As on the conditioned centreline (decision 0019), its separate
  lateral and braking caps ask the four-wheel car for more than it has. On the line the car spins and leaves the
  corridor: rear sideslip 1.42 rad, 18.9 m tracking error, 1544 samples past a tire's peak and 1596 outside the margin,
  a second lap of 30.02 s against a plan estimate of 26.89 s. The kinematic bicycle drives it within 0.08 m, at
  28.80 / 26.88 s. Only the envelope plan drives the four-wheel car on the line cleanly.
- **At grip 0.7 the line still slides on corner entry** in 361 samples, a tire past its peak, where the conditioned
  centreline has none. Why the reserve that suffices at full grip does not suffice there was not investigated.
- **The line uses the corridor.** On Foundry Circuit it reaches 3.504 m from the centreline, of 3.5 m usable. Its peak
  curvature halves, 0.029 against 0.060 1/m, and its squared curvature over the lap falls 46%. It is 3.5 m shorter.
- **Cost.** The Foundry line converges in 6 iterations and about 1.1 s on this machine when alone, up to 4.3 s beside
  other runs (measurement only). Where that time goes was not profiled.

## Verification

`tests/raceline_tests.cpp` (`fd_raceline`, 7 groups, about 25 s, most of it the closed-loop drive):

- **the plan's test**: on Foundry Circuit, with the defaults, every sample of the line stays within the corridor less
  the vehicle and margin, to 0.02 m. The refit lets the widest offset reach 3.504 m of 3.5 usable, and the closest edge
  is 1.496 m away. Both edges span the corridor's width at every sample.
- **the plan's test**: the line's curvature peaks at 0.0295 1/m against the steering limit of 0.236. A 20 m circle
  12 m wide cannot be driven with a 39.9 m turning radius and is refused with the steering limit named.
- **the plan's test**: under the envelope plan the estimated lap is 26.20 s on the line against 29.40 s on the
  conditioned centreline, and 26.89 against 29.96 s under the grip fractions. The squared curvature is 0.118 against
  0.219 1/m.
- **the plan's test**: on a stadium the middle of each straight stays straight, with curvature below 6.2e-7 1/m, within
  0.0001 m of a straight line, over 167 samples.
- a 40 m circle 12 m wide moves to its outer edge less the vehicle and margin, within 0.008 m of that radius, in 2
  iterations;
- bad input is refused with its reason: an iteration limit out of range, a corridor narrower than the vehicle and
  margin, and a centreline without normals;
- the four-wheel car at 80% of its envelope drives two laps of each. The second lap takes 29.78 s on the centreline and
  26.595 s on the line, which it tracks within 0.45 m, with no sample outside the corridor and none past a tire's peak.

Also: `fd_envelope_speed_plan` (11 groups) checks that the controller's driving bound in a corner is the forward limit
at the reference's lateral acceleration, and that braking there keeps the straight-line capacity. It also checks the
estimated lap time exactly on a circle and refuses a plan for another track and a span planned at rest.
`fd_track_conditioning` (12 groups) checks that conditioning a track matches conditioning the same points.
`fd_planner_tests` checks that an off-centre reference offers offsets within its own corridor. `fd_recording_playback`
round-trips corridor edges, reads a centreline's back as empty and strips them when downgrading.
`fd_recording_contract` derives, drives, records and validates the preset's racing line, and refuses `--obstruct` with
it before any output. The desktop UI verification (398 checks) covers the LINE row, its refusals, the fresh run on the
line, the road drawn at the line's own edges, the comparison line, the estimates against `estimated_lap_time`, and the
compact layout. It saves `compact-racing-line.png` (inspected, copied to `docs/assets/racing-line.png`),
`compact-racing-line-overview.png` and `compact-centreline-overview.png`.

Headless runs, two laps each, into fresh `out/runs` directories, from `out/tracks/foundry-centreline.csv` unless noted.
All use the four-wheel car at 80% of its envelope unless noted, and all were made with the amended controller bound:

| Run | Line | Laps | Plan estimate | Tracking error | Rear sideslip | Invalid samples |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `centreline-envelope` | conditioned centreline | 31.120 / 29.790 s | 29.400 s | 0.354 m | 0.063 rad | 0 of 12183 |
| `raceline-envelope` | racing line | 28.045 / 26.595 s | 26.204 s | 0.452 m | 0.094 rad | 0 of 10929 |
| `raceline-envelope-traced` | racing line of the noisy trace | 28.040 / 26.590 s | 26.201 s | 0.451 m | 0.094 rad | 0 of 10927 |
| `raceline-envelope-cg-0.6` | racing line, 0.6 m high | 28.585 / 26.920 s | 26.548 s | 0.409 m | 0.075 rad | 0 of 11102 |
| `raceline-envelope-margin-0.3` | racing line, 0.3 m margin | 28.070 / 26.615 s | 26.229 s | 0.442 m | 0.086 rad | 250 of 10938, all outside the margin |
| `raceline-envelope-grip-0.7` | racing line, grip 0.7 | 30.630 / 28.750 s | 28.327 s | 0.556 m | 0.139 rad | 361 of 11877 |
| `centreline-envelope-grip-0.7` | conditioned centreline, grip 0.7 | 34.845 / 33.250 s | 32.811 s | 0.444 m | 0.073 rad | 0 of 13620 |
| `raceline-envelope-whole` | racing line, whole envelope | 27.395 / 25.940 s | 25.416 s | 1.250 m | 0.326 rad | 902 of 10668 |
| `raceline-fractions` | racing line, grip fractions | 29.565 / 30.015 s | 26.887 s | 18.930 m | 1.416 rad | 3140 of 11917 |
| `raceline-kinematic` | racing line, kinematic bicycle | 28.795 / 26.880 s | 26.887 s | 0.082 m | 0 | 0 of 11136 |
| `centreline-kinematic` | conditioned centreline, kinematic bicycle | 31.740 / 29.900 s | 29.963 s | 0.229 m | 0 | 0 of 12329 |

The same set re-records decisions 0018's and 0019's envelope runs as `preset-envelope*` and `centreline-envelope*`,
compared run by run in `docs/evidence/raceline.json`. Each of the 23 runs validates with `--check-recording` as
schema 9: plans reproduced within 3.8e-10 m/s and plant motion within 1.0e-9 m. They were recorded with source
fingerprint `6bf390c3…`, before `estimated_lap_time` moved into `fd_core` and the desktop gained the line. The final
build re-recorded `raceline-envelope` as `raceline-envelope-final`, and its telemetry, track, plan, decisions,
trajectories and events are byte-identical. All 135 recordings in `out/runs`, schemas 1 to 9, still load. A core build
in a fresh directory with OSQP's source paths pointing nowhere configured with the bootstrap message, built, and passed
its 15 suites in 189.17 s, and its headless runner refused `--line racing` without creating the run directory.
`scripts/build.ps1 -Desktop`: 17/17 suites passed, 272.39 s of tests, 398 QML checks, no runtime warnings.
`scripts/build.ps1`: 16/16 suites passed, 216.38 s of tests. The UI check took 52 s of its 60 s timeout, which is now
120 s; the final desktop run after that change passed 17/17 in 278.22 s, the UI check taking 56.6 s. `scripts/common.ps1` no longer treats a native tool's stderr as a failure when output is redirected. qdldl's CMake
deprecation warning made redirected builds fail although CMake succeeded; the exit code is the failure signal.

## What this is not

Not a minimum-time line: minimum curvature is a proxy that ignores where the car is fast or slow, and Phase 4's
optimal-control lap is the reference for time. Not a closed-loop policy: the line is solved once before a run, and the
controller still follows it with Pure Pursuit. It does not replace local action selection: blockages are not placed or
avoided on the line yet. Not solved with the car's own dynamics: the only vehicle inputs are its half width and the
kinematic steering limit. Not cross-checked against TUM's `opt_min_curv` or fastest-lap, neither of which runs on this
machine.

## Consequences

- Phase 3 is complete as the plan lists it: envelope, envelope plan, conditioning, racing line, and in the desktop the
  live G-G diagram, the envelope plan, both lines drawn together and an estimated lap for each.
- Phase 4 has a conditioned track and a racing line to compare an optimal lap against. Its fastest-lap build still
  needs the user's approval to download.
- Phase 6 can reuse OSQP from `fd_raceline`'s build.
- A track can now carry per-sample corridor edges, the first step toward the per-point widths decision 0019 deferred.
- The grip fraction plan is unsuitable for the four-wheel car on smooth lines. The envelope plan is the one to use on
  them.
- Blockages on the racing line wait for obstructions stated in corridor coordinates rather than centreline ones.
