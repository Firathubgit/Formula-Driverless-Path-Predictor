# 0026: MAP steers the four-wheel car, and the head-to-head against MPCC

Date: 2026-09-20. Status: accepted for TrackWayFastPlan Phase 6.3. Extends the steady-state steering table of decision
0010 to a car with rotating wheels, so that the plan's baseline and MPCC can drive the same car; no recording schema
change. The comparison uses the plants of decisions 0012 to 0015, the envelope speed plan of decision 0018, the lattice
planner of decisions 0022 and 0023, and MPCC of decisions 0024 and 0025.

Phase 6.3 asks for the same scenarios, the same car and the same seed: MAP with the envelope speed profile against MPCC,
reporting lap time, maximum and RMS tracking error, peak per-wheel grip utilisation, lock events and the solve-time
distribution against the 20 ms period, with MAP kept selectable forever as the baseline. Only the four-wheel car has a
performance envelope, and only that car has wheels to lock, so the comparison belongs on it. MAP had no table for it.

## Decision

**A steady turn carries the car's wheels.** `core/steering.*` solved each row's turns for three unknowns: lateral
velocity, yaw rate and the command that holds the speed. A car with rotating wheels has four more states, so the search
now solves for its wheel speeds too, and carries its quasi-static wheel loads into the solve, refreshing them with the
loads the solved turn produces until they settle within a newton. Everything else is unchanged: the rows, the sweep that
ends at the first angle that does not settle or leaves the tire envelope, the boundary halving, the lookup, and the
fingerprint over the generator and the model identity. A turn is still judged holdable by the chassis eigenvalues of the
control period's map; the wheel speeds are the fastest states and follow it. The dynamic car's tables are untouched: the
same run of decision 0010's car reproduces its recorded lap to the last digit.

**MAP is offered wherever there is a table.** `--steering map` and the desktop's CONTROL row take the four-wheel car;
only the kinematic bicycle is refused, whose table would be its own geometry. A recording of the four-wheel car under
MAP records the table's fingerprint like any other, and the loader no longer refuses that pairing.

**`--check-recording` reports the peak friction use of any wheel**, from the recorded plant state through the vehicle
seam, which is what Phase 6.3 asks a four-wheel run to report beside its lock events.

**A UI verification that ends without its verdict fails.** During one desktop build this session the UI checks exited
zero without writing their evidence, and ctest reported a pass; only the file's timestamp gave it away. `--verify-ui`
now returns 7 unless the run reached its own verdict, so a suite that verified nothing can no longer pass.

## The head-to-head

The same four-wheel car, the same scenarios, the same stated blockages; runs one at a time into `out/runs/p63-*`, so
each solve time is its own run's. The baseline is MAP with 80% of the car's own performance envelope; Pure Pursuit with
the grip fractions is the original policy, for context.

| Scenario and driver | Laps | Tracking error, max / RMS | Peak friction use of any wheel | Samples with a wheel locked | Invalid samples |
| --- | ---: | ---: | ---: | ---: | ---: |
| Two clear laps, Pure Pursuit, grip fractions | 34.05 / 32.34 s | 0.34 / 0.160 m | 0.826 | 0 | 0 of 13279 |
| Two clear laps, MAP, 80% of its envelope | 31.95 / 30.70 s | 0.41 / 0.193 m | 0.978 | 0 | 0 of 12530 |
| Two clear laps, MPCC | 26.97 / 25.33 s | 3.67 / 1.499 m | 0.958 | 0 | 0 of 10461 |
| Lane blocked, one lap, Pure Pursuit, grip fractions | 34.01 s | 3.23 / 0.645 m | 0.831 | 0 | 0 of 6803 |
| Lane blocked, one lap, MAP, 80% of its envelope | 31.94 s | 2.80 / 0.592 m | 0.977 | 0 | 0 of 6389 |
| Lane blocked, one lap, MPCC | 27.01 s | 3.68 / 1.603 m | 0.957 | 0 | 0 of 5403 |
| Bollard entering the first corner, Pure Pursuit, grip fractions | 33.87 s | 3.25 / 1.050 m | 0.975 | 0 | 0 of 6774 |
| Bollard entering the first corner, MAP, 80% of its envelope | 31.66 s | 3.66 / 1.192 m | 0.997 | 0 | 0 of 6334 |
| Bollard entering the first corner, MPCC | 47.35 s | 44.44 / 14.986 m | 1.000 | 165 | 3615 of 9471 |
| Grip 0.7 from 10 s, two laps, Pure Pursuit, grip fractions | 38.12 / 37.39 s | 0.40 / 0.154 m | 0.965 | 0 | 89 of 15103 |
| Grip 0.7 from 10 s, two laps, MAP, 80% of its envelope | 35.11 / 34.69 s | 0.52 / 0.196 m | 1.000 | 0 | 131 of 13960 |
| Grip 0.7 from 10 s, two laps, MPCC | 28.59 / 27.40 s | 3.87 / 1.756 m | 0.958 | 0 | 0 of 11200 |
| Whole corridor blocked, 20 s, Pure Pursuit, grip fractions | no lap | 0.00 / 0.000 m | 0.600 | 0 | 0 of 4001 |
| Whole corridor blocked, 20 s, MAP, 80% of its envelope | no lap | 0.00 / 0.000 m | 0.941 | 0 | 0 of 4001 |
| Whole corridor blocked, 20 s, MPCC | no lap | 0.02 / 0.004 m | 0.900 | 0 | 0 of 4001 |

MPCC solves a decision in 18.5 ms at the median on the clear laps and 29.6 ms after the grip change, against the 20 ms
control period, with 95th percentiles of 71 and 191 ms. The corner run is the slowest at 41.0 ms, and it is the run
where the car is off the track and most of its plans are refused.

**On clear laps MPCC wins by using the corridor.** It laps in 26.97 / 25.33 s against the baseline's 31.95 / 30.70 s and
the policy's 34.05 / 32.34 s, cutting to 3.7 m off the centreline where the baseline holds 0.41 m. Tracking error is the
wrong measure of MPCC: it is not trying to follow the reference line. The baseline asks the most of a single tire (0.978
of its friction circle against MPCC's 0.958), because the envelope plan drives the whole car to its limit while MPCC
keeps each axle within 0.95 of its ellipse. No wheel locks in any of the three.

**At the corner bollard MPCC loses the car.** It arrives at the blockage at 19.5 m/s, where the baseline arrives at 16.5
m/s and the policy at 15.5 m/s, both slow enough to pass it without ever holding. At that speed the planner finds no
lattice path past the bollard within the grip it plans with, so it holds, and the hold is the policy's to brake: at 7.38
s the car is 15.3 m short of the blockage at 19.5 m/s, which no braking can stop. Its tires go past their peak, a rear
wheel locks from 8.3 s, it leaves the track margin at 9.9 s, reaches 44 m off the reference at 18.5 s, is back within 3
m at 22.4 s and finishes the lap in 47.35 s, with 3615 of 9471 samples invalid. Both baselines pass the same bollard
without leaving the track, because at their speed the planner still finds a clear path. The dynamic car, whose wheels
cannot lock, survives the same scenario under MPCC (decision 0024's run: 27.99 s, 84 invalid samples).

That is the head-to-head's main negative result, and its cause is in the seam, not in MPCC's optimisation: the planner
chooses actions by rolling the **policy** forward within the reference plan's grip fractions, while MPCC drives to 0.95
of each axle's friction ellipse. The two disagree about what is reachable, and the disagreement is discovered late, at a
speed the fallback cannot retrieve.

**Under a grip change MPCC stays ahead**: 28.59 / 27.40 s against 35.11 / 34.69 s, with no wheel locked and no invalid
sample, where the baseline has 131 and the policy 89, each running faster than its revised plan while it brakes. With
the whole corridor blocked all three hold and stop, MPCC driving 87 of its 1000 decisions before the hold.

## Verification

`tests/steering_tests.cpp` adds two groups. `four_wheel_table_holds_the_turns_it_claims` generates the four-wheel car's
table, requires each row to start straight and to increase in steering and curvature together, and then drives the car
into the turns the table claims: eased into each turn with a throttle loop holding the speed, the curvature it settles
at is within 0.5% of the table's inside a row and within 4% at a row's last turn, which sits at the tires' limit where
the turn is only weakly attracting. It also checks that the car is nearly neutral in gentle turns, that its correction
to geometry grows with lateral acceleration to 0.039 rad at 9.7 m/s^2, and that the table knows which car it was
generated for. `map_drives_the_four_wheel_car` drives the car through the simulation under Pure Pursuit, under MAP, and
under MAP with the envelope speed plan: MAP tracks the reference better than geometry does, and the envelope run keeps
its tracking error bounded while driving the envelope's speeds. The dynamic car's tables are unchanged: rerunning
decision 0010's two-lap MAP run reproduces the recorded summary exactly, to every digit, with the same table
fingerprint.

`fd_recording_contract` records a four-wheel MAP run, checks that its metadata names MAP with a table fingerprint, and
validates it; `fd_recording_playback` and `fd_wheel_model` lose the refusals that pairing used to carry. The desktop's
UI verification checks that MAP is offered for the four-wheel car, that choosing it builds a table generated for that
car, and that Pure Pursuit returns. The desktop build was run again after the guard above was added, and its evidence
file is from that run: 488 checks, no QML warning.

Four mutations were run and reverted, each failing a group. Leaving a car's wheel speeds out of the search, and never
refreshing the wheel loads carried into it, both leave even the straight-ahead turn unsolved, so the table refuses to
generate at all (`four_wheel_table_holds_the_turns_it_claims` and `map_drives_the_four_wheel_car`: "the plant does not
settle driving straight"). Letting a row's sweep accept turns past the tires' peak fails
`kinematic_table_is_the_geometric_law`, whose saturation then runs to the steering limit. Leaving the centre of
gravity's height out of a table's identity lets a table generated for one car be accepted for another.

`scripts/build.ps1`: 20 of 20 CTest suites pass in 479.8 s, `fd_mpcc` taking 181.9 s and `fd_steering_law` 46.9 s.
`scripts/build.ps1 -Desktop`: 21 of 21 in 444.9 s, the QML controls suite running 488 checks in 71.6 s with no QML
warning; neither build reports a compiler warning.

The head-to-head ran into fresh `out/runs/p63-*` directories, one run at a time, so each solve time is its own run's:
five scenarios times three drivers, every run validated with `--check-recording`. With this build 303 of the 305
recordings in `out/runs` pass `--check-recording` (schemas 1 to 13, 48 of them schema 13). The two refused are the same
as before: `mpcc-dynamic-laps-mpcc`, cut off while it was written two sessions ago, and the exploratory
`p62x-dynamic-1789837332`, written before the dynamic car's schema 13 metadata was settled. Source fingerprint
`97e22d0a1fb3c7418aba429bee0bc7eac157be2de21cd40e1280a0715917487c`. Machine-readable evidence is in
`docs/evidence/head-to-head.json`.

## What this is not

Not a fix for what the corner exposed: this decision measures it. Not a lateral-load-transfer model in MAP's steering:
the table is the plant's own steady turns, so it carries whatever the plant does, but MAP remains a steady-state
correction and says nothing about transients. Not a table that follows a setup change for free: it is fingerprinted for
one car and configuration and regenerated when either changes, which costs about a third of a second for the four-wheel
car. Not a claim beyond one track and one car.

## Consequences

- Phase 6.3 is answered on the car that has both a baseline and wheels to lock.
- The planner's yardstick, the policy at the reference plan's grip fractions, is now a measured problem rather than a
  suspicion: it should know which controller drives, or hold earlier when it does not.
- Phase 6.4's per-wheel friction margin along the horizon remains; `--check-recording` already reports the peak.
- MAP stays selectable for every car with tires, as the plan requires of the baseline.
