# 0019: Conditioning a traced centreline into a track

Date: 2026-09-16. Status: accepted for TrackWayFastPlan Phase 3.3.

Phase 3.3 asks for track conditioning after TUM's `prep_track.py`, reimplemented: spline regression, uniform
resampling, normal vectors and rejection of normals that cross. It is also the first half of the future "draw a track"
feature. Its tests: a noisy version of Foundry Circuit recovers its curvature within tolerance; a self-intersecting
input is rejected.

`prep_track.py` resamples the imported centreline linearly at 1 m, fits a closed B-spline of order 3 with smoothing
factor 10 (`tph.spline_approximation`), resamples it at 3 m, computes splines and normals (`tph.calc_splines`), refuses
normals that cross within ten samples (`tph.check_normals_crossing`), and optionally inflates narrow sections. The
spline work lives in `trajectory_planning_helpers`, which is not on this machine, so the method below follows that
pipeline and its published behaviour, not its code (LGPL-3.0; nothing was copied).

## Decision

**`condition_track(centreline, width, name, options)`** in `fd_core` (`core/track_conditioning.hpp`) turns a closed,
ordered list of points into a `ConditionedTrack`: a `Track` accepted by `validate_track`, the left normal at every
sample, and how far the result lies from the input. `load_centreline(file)` reads `x_m,y_m` lines with comments and one
header line. Nothing else in the project changes: the preset stays the default track, and a conditioned track is an
ordinary `Track` that the planner, controller, simulation and recordings already handle.

1. **Linear resampling** of the closed polyline at `input_spacing_m` (default 1 m), after dropping repeated points and a
   repeated closing point.
2. **A periodic cubic B-spline regression** with a knot every two resampled points, parameterised by the polyline's arc
   length. Among such splines the fit is the smoothest one, by the summed squared second differences of its control
   points (a periodic P-spline), whose root-mean-square distance from the resampled points at their parameters is at
   most `smoothing_rms_m` (default 0.1 m). The distance grows with the penalty's weight, so the weight is found by
   bisection on its logarithm; each trial solves the cyclic banded normal equations by Cholesky over the matrix's row
   envelope, which stays narrow in all but the last three rows. TUM's smoothing factor is the same allowance as a sum,
   points times the allowance squared (10 m² at 1 m spacing is about 0.14 m rms on a 500 m loop); it is stated per
   point here so it does not depend on the loop's length. FITPACK chooses its knots and measures smoothness by jumps of
   the third derivative; a P-spline with fixed knots and a second-difference penalty gives the same kind of result with
   a solver of a few dozen lines.
3. **Uniform resampling by arc length** every `output_spacing_m` (default 0.6 m, the preset's spacing, rounded so the
   samples close the loop evenly): arc length by five-point Gauss-Legendre quadrature over quarter knot spans, each
   sample's parameter by Newton's method on arc length. Sample 0 is where the fitted curve begins, beside the first
   point, and the direction of travel is kept.
4. **Analytic curvature and normals** from the spline's derivatives: curvature is continuous, and the left normal points
   to the side offsets are positive on, this project's convention (TUM's normals point right).
5. **Checks**, each refusing with the station and reason:
   - the conditioned line crossing itself (any two non-adjacent segments meeting);
   - normals across the width crossing within the distance over which a bend of half the width turns back, pi times
     half the width, rather than TUM's ten samples: a bend tighter than half the width folds the corridor at its inside;
   - the corridor overlapping itself: parts of the loop farther apart along it than that distance, but closer than the
     width. TUM does not check this; a drawn track needs it, because the projection and the corridor model cannot
     represent shared tarmac.

**One width.** TUM tracks carry a width to each side per point, and `prep_track` can inflate narrow sections. This
project's `Track` has one width, which conditioning keeps; per-point widths wait for a track model that needs them.

**Headless.** `fd_headless --track CENTRELINE.csv [--track-width M] [--track-smoothing M]` conditions the file before
anything is written, prints what it did, and drives and records the run on that track; a refused centreline leaves no
run directory, and the width and smoothing flags are refused without `--track`. Recordings need no change: they already
store the track's samples, width and name.

## Found by building it

- **The allowance is used in full.** The fit is the smoothest curve within the allowance, so a clean shape moves by
  about the allowance where smoothing helps: a clean 30 m circle traced every 1.3 m shrank 0.23 m in radius under
  TUM's absolute factor, and keeps its radius within 0.005 m with a 0.002 m allowance. That is also why the allowance
  is stated per point.
- **Below the trace's own noise the fit follows the noise.** A 0.05 m allowance on a trace with 0.1 m of noise per axis
  bends into kinks tighter than half the width, which the normals check refuses; the refusal now says a larger
  allowance removes such kinks.
- **Rounding a corner tightens its apex.** The preset's curvature steps from 0 to 1/18 at the first corner; conditioned,
  it ramps over about 10 m either side and peaks 8% higher, 0.060 1/m: the corner turns through the same angle, and
  the gentler entry and exit take some of it, so the middle turns more sharply.
- **A continuous-curvature track exposes the grip fraction plan's uncombined limits.** That plan caps lateral demand at
  65% and braking at 55% of mu g separately. On the preset all braking ends on the straight before the curvature
  step, so the two never meet; on the conditioned preset braking runs into the curvature ramp and the four-wheel car
  is asked up to 0.85 of mu g combined, beyond what it has: 562 samples with a tire past its peak braking into corners at
  -4.4 to -4.8 m/s² with 6.9 to 7.2 m/s² lateral. The envelope plan combines them by construction and keeps every tire
  within its peak at 80%. The kinematic bicycle, the default model, has no such limit and drives the traced track
  without an invalid sample. Changing the baseline would change every recorded plan, so it is recorded here, not fixed.
- **Continuity helps the envelope plan's reserve, but does not replace it.** At 80% the conditioned preset removes the
  tall car's invalid samples (0 against 97) and cuts the default car's tracking error from 0.51 to 0.35 m; planned with
  the whole envelope, the car spins at the first corner (rear sideslip 0.876 rad, 4747 invalid samples against 1091 on
  the preset), braking into it from 130 m and sliding through it; why it spins there rather than sliding as on the
  preset was not investigated.
- **Cost.** Conditioning takes 0.007 s for a 500 m loop, 0.15 s for 2.5 km and 0.58 s for 5 km on this machine
  (measurement only); the crossing and overlap checks are quadratic in the number of samples.

## Verification

`tests/track_conditioning_tests.cpp` (`fd_track_conditioning`, 11 groups, under a second):

- **the plan's test**: Foundry Circuit sampled every metre with 0.1 m of seeded noise per axis, conditioned with the
  defaults, has curvature within 0.0041 1/m of the preset's from 10 m beyond any join (bound 0.0056, a tenth of the
  tightest corner's), at most 0.028 within 10 m of a join (bound 0.033), stays within 0.19 m of the preset's
  centreline (bound 0.3) and is 507.05 m long against 507.56;
- **the plan's test**: a figure eight is refused because it crosses itself;
- 4 m hairpins on a 10 m wide track are refused because their normals cross, 7 m ones accepted; a loop with a 9 m waist
  is refused on a 10 m wide track because its corridor overlaps itself, and accepted 8 m wide;
- a clean 30 m circle stays within 0.005 m of its radius and 8e-4 1/m of its curvature;
- on the clean preset the largest curvature change between neighbouring samples is 0.0025 1/m against the preset's
  0.056 step, and the largest curvature 0.060;
- the allowance is effective: 0.3 m moves the line further and varies its curvature less than 0.1 m, each fit stays
  within its allowance, and 0.05 m under 0.1 m noise is refused;
- a clockwise loop keeps negative curvature and outward left normals, samples are evenly spaced at the requested
  spacing, and sample 0 is beside the first point;
- bad input (three distinct points, a NaN, zero width, an input spacing out of range, a loop too short) is refused with
  its reason; a centreline file with a header, comments and blank lines loads, and malformed rows are refused by line;
- the kinematic car drives the conditioned trace for 20 s, 315 m, within 0.24 m and without an invalid sample.

`fd_recording_contract` conditions a 72-sided polygon around a 60 m circle with `--track --track-width 8`, checks the
report, the recorded width and length (376.1 m) and `--check-recording`, and refuses a figure eight and a width without
a track before any output.

Headless runs, two laps each on Foundry Circuit, into fresh `out/runs` directories, from `out/tracks/foundry-centreline.csv`
(the preset's 851 recorded samples) and `out/tracks/foundry-traced.csv` (the preset every metre with 0.1 m of seeded
noise per axis), both conditioned with the defaults; the preset runs are decision 0018's:

| Run | Track | Plan | Laps | Tracking error | Rear sideslip | Invalid samples |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| `plan-envelope` | preset | 80% envelope | 31.950 / 30.695 s | 0.514 m | 0.057 rad | 0 of 12530 |
| `cond-envelope` | conditioned preset | 80% envelope | 31.105 / 29.795 s | 0.354 m | 0.062 rad | 0 of 12181 |
| `traced-envelope` | conditioned trace | 80% envelope | 31.245 / 29.940 s | 0.397 m | 0.065 rad | 0 of 12238 |
| `plan-envelope-cg-0.6` | preset | 80% envelope, 0.6 m high | 32.655 / 31.260 s | 0.474 m | 0.053 rad | 97 of 12784 |
| `cond-envelope-cg-0.6` | conditioned preset | 80% envelope, 0.6 m high | 32.185 / 30.755 s | 0.316 m | 0.055 rad | 0 of 12589 |
| `plan-envelope-whole` | preset | whole envelope | 30.330 / 29.025 s | 1.305 m | 0.097 rad | 1091 of 11872 |
| `cond-envelope-whole` | conditioned preset | whole envelope | 30.320 / 28.940 s | 2.592 m | 0.876 rad | 4747 of 11853 |
| `plan-fractions` | preset | grip fractions | 34.045 / 32.345 s | 0.344 m | 0.041 rad | 0 of 13279 |
| `cond-fractions` | conditioned preset | grip fractions | 32.310 / 30.445 s | 0.434 m | 0.109 rad | 562 of 12552 |
| `traced-fractions` | conditioned trace | grip fractions | 32.505 / 30.660 s | 0.435 m | 0.097 rad | 323 of 12634 |
| `traced-kinematic` | conditioned trace | grip fractions, kinematic bicycle | 31.935 / 30.125 s | 0.244 m | 0 | 0 of 12413 |

The four-wheel car is used unless noted. The conditioned preset is 506.87 m long with 845 samples, 0.25 m at most from
the preset's samples; the trace conditions to 507.07 m, 0.30 m at most from its noisy points. Each run validates with
`--check-recording` as schema 8 from this build, plans reproduced within 3.4e-10 m/s and plant motion within 1.0e-9 m;
all 102 recordings in `out/runs`, schemas 1 to 8, still load. `scripts/build.ps1 -Desktop`: 16/16 suites passed, 180.70 s of tests, with 360 QML checks and no runtime warnings;
`scripts/build.ps1`: 15/15 suites passed, 146.68 s of tests.

## What this is not

Not a racing line: conditioning smooths the centreline, it does not move it toward a faster line; that is Phase 3.4.
Not the "draw a track" feature itself, which still needs a way to draw. Not a boundary-based preprocessor like
fastest-lap's, which fits a centreline between traced edges by optimisation. One width, no per-point widths and no
inflation of narrow sections. The desktop still drives the preset only; a recording made on a conditioned track
replays as any recording does.

## Consequences

- Phase 3.4 has continuous curvature and left normals to build a minimum-curvature line on, and a conditioned track to
  compare it against.
- A traced or drawn centreline can be driven and recorded today from the headless runner.
- The grip fraction plan's separate lateral and longitudinal caps are now a known limit on smooth tracks for the
  four-wheel car; the envelope plan is the one to use there.
