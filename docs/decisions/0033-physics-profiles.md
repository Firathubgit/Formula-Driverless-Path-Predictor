# 0033: Physics profiles

Date: 2026-09-25. Status: accepted for TrackWayFastPlan §8, "Selectable vehicles with honest physics", as physics only.
Amended the same day by [decision 0034](0034-theoretical-best-lap.md): the minimum-time cross-check with TUM's own
formulation found that this decision read TUM's load sensitivity wrongly and gave TUM's car 0.1 too much lateral
friction; its tire is corrected below, and the desktop now takes `--profile ID` at start.

§8 asks for each selectable car to carry a distinct, documented physics profile instead of sharing one: a Formula One
style car from fastest-lap's Limebeer 2014 file, a Formula Student electric car from PacSim and MPCC, an electric race
car from TUM's example parameters, a racing kart from fastest-lap's Lot 2016 file, and a road car only once someone sources
one. Each profile states its source and whether each value is measured, fitted or illustrative; a team-named file is
never presented as team data; appearance and physics stay separable. The user asked for the technical side only, and for
the showroom's car selection and its Blender videos to be left alone, so this decision builds the profiles, their tests
and their recording, and leaves every appearance as it is. All four sources were already on this machine; nothing was
downloaded.

## Decision

**A profile is the four-wheel car with every value accounted for.** `VehicleProfile` (`core/vehicle_profiles.*`) is a
`FourWheelCar`, the configuration members that belong to the car (wheelbase, steering lock, speed cap), its source, and a
`ParameterSource` for every value it sets: published as the source prints it, derived from the source by stated
arithmetic, fitted to the source's own model, or chosen here because the source gives none. Five are offered: the
project's roadster, as before; `formula-one-style` from Limebeer et al. 2014 through fastest-lap; `formula-student-electric`
from PacSim, whose authors call their values made up, and MPCC; `electric-race-car` from TUM's example; and
`racing-kart` from Lot 2016 through fastest-lap. There is no road car, which no audited source describes.

**The sources' tires are fitted, not copied.** Each source's Magic Formula differs from this project's tire, which is
described by its peak friction and slip at two loads and the share of grip kept at large slip.
`tools/vehicle_profiles/fit_tires.py` finds each source curve's peak numerically and the share it keeps at 0.5 rad: TUM's
tire peaks at 5.09° and keeps 0.709. Its lateral friction is 0.933 at 2000 N and 0.800 at 6000 N, because TUM's solver
scales the peak by 1 + eps Fz / f_z0, the load itself over f_z0. The first version read the load's increment over f_z0,
the usual Magic Formula form, and gave 1.033 and 0.900. Its longitudinal force is bounded by the friction circle of mue
alone, 1.0 at every load, and its floor is chosen at 0.6, below any load's friction. MPCC's
Formula Student tire peaks at 12.4° and keeps 0.948 (PacSim's own, with C −1.39 and E 1, never peaks at all); the kart's
MF 5.2 tire peaks at 4.44° at 300 N and 5.27° at 700 N laterally, 0.079 and 0.097 in slip ratio, keeping 0.61 and 0.71.
The Formula One file's frictions and slips are this project's default tire already; its shape factor Q 1.9 implies
only 0.156 of grip kept when sliding, too little for asphalt, so the share stays at the 0.75 chosen before, as §4.1(a)
asks it to be chosen again.

**Where the project's ranges bind, the profile says so.** The configuration's speed cap is at most 40 m/s, so the Formula
One car (about 90 m/s) and TUM's (v_max 70 m/s) are capped there and marked chosen; the preset never reaches it. The
wheelbase range now starts at 1.0 m instead of 1.5 m, for the kart's 1.045 m and the Formula Student car's 1.44 m; no test
depended on the old bound. Values a source does not give are chosen and marked: every car's wheel inertia, the Formula
One car's drive torque and lock, the Formula Student car's centre of gravity, power (the rules' 80 kW), brakes and
drive, and the kart's drive torque. The kart's solid rear axle is the model's stiffest viscous coupling, 200 N m s/rad,
which behaves as locked.

**Selected in the headless runner, recorded and checked.** `--profile ID` builds the car from a profile and sets its
configuration members; it refuses `--plant`, `--car` and the four-wheel car's setup flags beside it, since the profile
is the whole car. The recording names the profile in `metadata.vehicle_profile`, and the loader requires the recorded car
and configuration to be exactly that profile's in this build, so a recording cannot claim a profile it was not. The
desktop's car choice is the showroom's, which this work was asked to leave alone; since decision 0034 the desktop takes
`--profile ID` at start, and nothing in the showroom changes.

## Verification

`tests/vehicle_profile_tests.cpp` (`fd_vehicle_profiles`, three groups): every profile is a car the model accepts with
its configuration, named and sourced, each value's provenance stated (the Formula One car 12 published, 4 derived and 5
chosen; the Formula Student car 6, 2, 2 fitted and 12 chosen; TUM's 11, 5, 1 and 3; the kart's 11, 4, 1 and 3); ids are
unique, no road car is offered and an unknown id is refused naming the others. Spot values from each source hold, with
the arithmetic of the derived ones. At the same speeds for every car: from 15 to 30 m/s the lateral limit grows 25% for
the Formula One car, 67% for the Formula Student car and 5% for TUM's, whose tires lose friction as its load grows, and
not at all for the two without wings; the 20 kW kart drives weakest at 25 m/s (2.77 m/s²); on the preset the Formula One
car's envelope plan laps in 20.25 s, before the roadster's 30.29, TUM's 29.75 (28.70 before its tire was corrected) and
the kart's 28.24, and the Formula
Student car's in 19.77 s, quickest of all on this tight track, its downforce larger for its mass than the Formula One
car's. Every profile drives ten seconds from rest on its own envelope plan without leaving the course.

`tests/recording_tests.cpp` adds `profiled_run_round_trips` (27 groups): two seconds of the kart record its profile and
load; the car's mass changed, or an unknown profile named, is refused by name. After the correction the suite passes
again, with TUM's lateral friction 0.933 and 0.800 at its reference loads and 1.0 longitudinally.

Headless, a lap of the preset with each profile on its envelope plan (`out/runs/p8-profile-*`): the Formula One car in
22.34 s, the Formula Student car in 22.67 s, TUM's in 30.98 s, the kart in 31.39 s and the roadster in 31.95 s, none
leaving the course or knocking a cone, each recording validating with its profile confirmed; TUM's corrected car is
reported in decision 0034. `--profile` beside
`--mass`, and an unknown profile, are refused before any output.

## What this is not

Not measured cars: two of the sources say their values are examples or made up, and the fits carry this project's tire
model's limits. Not the sources' own dynamics: each profile is this project's four-wheel car with quasi-static load
transfer, whatever chassis its source modelled; the kart's six-degree chassis, the Formula One car's energy recovery and
TUM's powertrain temperatures are not here. Not an appearance: no car's look is tied to a profile.

## Consequences

- §8's profiles exist for the physics; the desktop takes one at start with `--profile ID` (decision 0034), and offering
  them in its car choice, beside the showroom's appearances, is left for when the user wants the showroom touched.
- The configuration accepts wheelbases from 1.0 m.
- Recordings of a profile name it, and are refused if the car is not that profile's.
