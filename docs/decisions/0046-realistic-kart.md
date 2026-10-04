# 0046 — A rental kart fitted to the poster's line and to real lap times

2026-10-01. The user drives Gokartcentralen Göteborg for real; their best lap there is 36.38 s (the operator's leaderboard:
11 laps in their last session, ranking 3). They said the simulator's kart did not feel like the real one: fix one corner and the
kart can no longer follow the line the poster recommends or brake where it brakes. They asked for a kart whose times and
braking zones match, from research, "added as a toggle feature", with the showroom and its car selection left alone.

## What the research gave, and did not

The operator's electric karts are Sodi RSX2s: 186 kg with batteries, a brushless IPM motor of 14 hp and 42 N m, 48 V 86 Ah
lithium batteries, about 65 km/h (Sodikart, ERKC, the operator says 60). Not published anywhere found: the tires' grip, the brakes,
the gearing, the wheelbase, and any lap benchmark beyond the operator's Speedkart qualifying time of 34.0 s for a standard kart
(from the dossiers the user supplied, decision 0038) and the user's own 36.38 s. So the kart is fitted, not looked up, and
the fit says what it rests on.

The lap the user hoped for as a theoretical best, "25 s", is not reachable by any kart there: 60 km/h along the 363 m of the
poster's line is 21.9 s with no corner at all. What the line itself allows is the finding below.

## What the poster's line says

`tracks/gokartcentralen-goteborg-guide.csv` (decision 0039) is the poster's coloured stroke, 1600 samples: 264 m gas, 45 m coast,
54 m brake over 362.6 m, six short braking zones of 6 to 10 m. Its raw curvature is tracing noise (radii under a metre);
smoothed by a periodic spline within 0.25 m the tightest radius is 2.8 m and the line is a path a kart can drive.
`tools/kart/make_poster_line.py` writes it as `tracks/gokartcentralen-goteborg-poster-line.csv`.

A quasi-steady-state lap on it, a kart of 261 kg, 14 hp, drag area 0.78 m^2, the 60 km/h controller, braking 0.8 g, cornering
at one coefficient of grip, speed limited by corners, power and brakes forwards and backwards:

| Grip both ways | Lap | Colours matched | Within 3 m |
| ---: | ---: | ---: | ---: |
| 0.9 | 35.57 s | 70% | 95% |
| 1.0 | 34.12 s | 71% | 95% |
| 1.1 | 32.94 s | 70% | 94% |
| 1.2 | 31.96 s | 68% | 91% |

A grip of 1.0 gives the operator's own 34.0 s, and the model's gas, coast and brake, taken from its acceleration (brake past
0.4 g, coast while the speed is held), match the poster's colours to within 3 m on 95% of the lap. So the poster's line is the
real kart's line, and the real kart is a kart of about 1 g that laps it in about 34 s at its limit.

## Why the earlier simulated kart was not

Decision 0038's kart (a grip of 1.0 on every tire) is allowed 37.5 s on the poster's line by the plant's own performance envelope
at its whole. Driven round it in the plant as the desktop's kart is, it spins at 90% to 100% of that envelope (rear sideslip
1.3 rad; 57 cones and 25 excursions at the limit) and laps in 40.8 s at 85%: the symptom the user described, measured. Three
causes, found by changing one value at a time in the real plant (`fd_headless --car-set`, the new tuning option below):

- **Cornering.** The plant corners at 0.89 g at best but only 0.81 g at 6 m/s and 0.46 g at 4 m/s, where the poster's hairpins
  are: at a 3 m radius a tire's slip angles are fixed by geometry and pass its peak. The line asks about 1 g.
- **Softening the tires to reach the hairpins makes the plant spin.** A peak slip angle of 0.15 rad on both axles, which raises
  the 4 m/s limit from 0.46 to 0.59 g, put the kart into a spin at every share of its envelope; wider at the front only is
  stable but tracks the line worse (35.7 s against 34.5 s at the limit). Raising grip alone is stable.
- **Steering rate.** The configuration allows the front wheels 0.9 rad/s, a third of a kart's. It was not the cause of the
  spins (tested), and a person's hands feel it.

The axle's coupling decides cornering strongly, as 0038 found: a coupling of 10 N m s/rad takes the lap to 43.6 s, 50 to 53.1 s;
the profile keeps 1.

## The fitted kart

`gokartcentralen-realistic` (`core/vehicle_profiles.cpp`, decision 0033's rules): the rental kart of 0038 with three values
changed, each marked in the profile.

- **Tires grip 1.35 sideways and 1.0 along**, instead of 1.0 both ways. Published kart tires grip more sideways than along (Lot's
  racing kart 1.5 and 0.9); 1.35 is the value at which this plant laps the poster's line in 34.5 s at its limit. It is an
  equivalent value, not a measurement: it also absorbs what the model lacks of a solid axle and a flexing chassis.
- **Braking stays tire-limited at 0.74 g**, as the 0.6 to 0.7 g rental karts brake at; the brake torque cap makes no
  difference to it.
- **Steering rate 3 rad/s** (`VehicleProfile::max_steering_rate_radps`, applied by `configured_for` when set; the earlier kart
  leaves the configuration's).

Power (14 hp), top speed (the 60 km/h controller), mass (186 kg and a 75 kg driver), geometry and the axle are the earlier
kart's.

Driven in the plant round the poster's line, Pure Pursuit aiming 2 m ahead as the desktop's kart is driven, on the project's
speed plan at a share of the kart's envelope (the share is the driver's effort: 1 is the kart's limit), judged from the start
line (`tools/kart/fit_kart.py`, `docs/evidence/kart-realistic.json`):

| Share of the envelope | Fitted kart's flying lap | Earlier kart |
| ---: | ---: | ---: |
| 100% | 34.46 s | spins (57 cones, 25 excursions) |
| 95% | 35.10 s | spins |
| 90% | 36.10 s | spins |
| 85% | 37.26 s | 40.82 s |
| 80% | 38.54 s | 42.55 s |

Every fitted lap is clean (no cone, no excursion, rear sideslip 0.04 to 0.13 rad). The user's 36.38 s is 89% of the fitted kart's
limit; an average driver's 38 s about 82%. That mapping from effort to lap is this model's, not a measurement of the user: it
says only that the kart's limit is where the operator's standard puts it and that a driver using nine tenths of it laps as the
user does. The plant's colours, from its achieved acceleration against the poster's at the same place, match 74% exactly
and 83% within 3 m of travel (the quasi-steady-state model 95%): the plant coasts about 100 m where the poster paints 45,
holding a corner's speed where a driver would hold a touch of throttle; the brake zones match in length, 50 m against 54 m.

## The toggle

Gokart mode's settings have a Realistic kart switch under the Gokart one, on by default, "Fitted to the poster line" or
"Earlier rental kart". Off, the mode drives decision 0038's kart as it was. In the mode the switch is another kart, so the run
starts afresh, as leaving the mode and entering it again would, practice session included (a lap time in one kart means nothing
in another); outside it the switch sets the next run's. Paused only, like every change of car. The note beneath states the kart's
numbers and, when realistic, what shares of its limit the real laps are.

## A tuning option for the headless runner

`fd_headless --car-set NAME=VALUE` (repeatable) tunes one value of the four-wheel car after the profile: any `FourWheelCar`
number by name, a tire's as `tires.NAME`, `front_tire.NAME` or `rear_tire.NAME`, and the configuration's `wheelbase_m`,
`max_steering_rad` and `max_speed_mps` (`fd::set_car_value`, `fd::car_value_names`, which list the valid names when given an
unknown one). A tuned car no longer names its profile in a recording. The fit above ran on it, and it needs no rebuild to
try a value.

## Checks

`fd_kart_fit` (`tests/kart_fit_tests.cpp`, two groups, about 100 s): the poster's inside line is 361 m and bends no tighter than a
3.8 m radius once conditioned; the fitted kart drives two clean laps at 100%, 90% and 80% of its envelope in 34.0 to 35.0 s,
35.7 to 36.6 s and 38.0 to 39.0 s, a good driver's 36.38 s falls between the last two, and the earlier kart laps over 40 s even at
85%, four seconds slower than the fitted kart at 90%. `fd_vehicle_profiles` gains the seventh profile: its values are the
earlier kart's but for the three, and its steering rate reaches the configuration while the earlier kart's does not. The
desktop's UI suite starts gokart mode on the fitted kart, switches to the earlier kart and back through the actual QML switch
and checks the car's tires, the steering rate and the note each time.

Last run with the whole suite on 2026-10-04: all 33 core suites and all 40 desktop suites passed, `fd_kart_fit` its two groups in
67 s. Since this decision the desktop's kart has been tuned away from this fitted profile by feel (decisions 0047, 0053 to 0055); the
profile, its fit and these checks are unchanged, and the lap times above are the fitted profile's, not the desktop kart's.

## What this is not

Not a measured kart: the tires' grip, the brakes and the axle are fitted to a line a poster paints and to two lap times, the
operator's 34.0 s and the user's 36.38 s. Not a driver model: a share of the envelope stands in for how hard a person drives,
and a person's laps vary by more than the steps between shares. The plant still cornered at 0.5 to 0.6 g at 4 m/s where the
line's hairpins ask for more; the kart makes it up in the corners at 8 m/s and above, where the plant's 1.18 g stands for
what the real kart does with its solid axle. Not a coach: the poster's colours are static advice and nothing here changes
them or measures a person against them.

## Consequences

- Gokart mode drives the fitted kart by default; the earlier one stays selectable and unchanged.
- `VehicleProfile` has an optional steering rate; `fd_headless` has `--car-set`; `tools/kart/` is how the kart is checked
  again if the plant changes.
- `tracks/gokartcentralen-goteborg-poster-line.csv` is the smoothed line the plant is driven round: derived from the
  committed poster line, never from the poster itself.
