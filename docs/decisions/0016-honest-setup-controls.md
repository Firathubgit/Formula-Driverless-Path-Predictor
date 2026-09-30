# 0016: Honest setup controls

Date: 2026-09-16. Status: accepted for TrackWayFastPlan Phase 2.4.

Phase 2.4 asks to split `Config` into vehicle parameters and planning or control configuration, bump the
configuration schema, validate every parameter before mutation, and expose mass, centre of gravity height,
brake bias, cd and cl, power and differential stiffness, each only if a test shows it changes a measured
outcome in a documented direction on a fixed scenario. AGENTS.md's "the kinematic bicycle has no mass
sensitivity" changes in the same change that ships the evidence (plan section 9), and section 9 adds the
invariant that every exposed vehicle parameter has a test proving its effect. Phase 2's visible list asks
for brake-bias and weight controls that visibly move braking points and lock behaviour.

## Decision

**The split already exists in the code, so no configuration field moves and the schema stays 1.** Since
decisions 0005, 0008 and 0012 the car's parameters live in the vehicle model (`FourWheelCar`), apart from
`Config`. What remains in `Config` is road grip, timing, the planner's speed cap and grip fractions, the
controller's gains and lookahead, and three values every plant, Pure Pursuit and the curvature cap share:
wheelbase and the steering angle and rate limits. Moving those three would touch about fifty uses across
core, planner, controller and recordings for no new behaviour, and none of them is on the plan's list of
controls. The configuration files keep schema 1. This is a deviation from the plan's wording, recorded here.

**Setup controls are a registry in the core** (`core/setup.hpp`, `four_wheel_setup_controls`): the plan's
seven parameters, each with a label, unit, display scale, offered range and step, and the effect its test
measures. `with_setup_value(car, config, key, value)` refuses an unknown key, a non-finite value or one outside
the offered range, then runs `validate_vehicle`, so a value is validated before anything changes.

| Control | Offered | Default | Effect proved on the fixed scenario |
| --- | --- | ---: | --- |
| Mass | 600 to 1400 kg | 800 | heavier uses more of the tires' grip, because friction falls as load rises |
| Centre of gravity height | 0 to 0.6 m | 0.35 | taller puts more load on the front axle under braking and deepens rear braking slip |
| Brake bias front | 20 to 80 % | 50 | rearward locks the rear wheels into a corner; the deepest braking slip is on the favoured axle |
| Drag area | 0 to 1.5 m² | 0.6 | more covers less distance in the same time |
| Downforce area | 0 to 4 m² | 0 | more leaves more grip unused |
| Power | 30 to 120 kW | 100 | less pulls away more slowly; above what the plan asks, more changes nothing |
| Viscous coupling | 0 to 100 N·m·s/rad | 0 | stiffer holds the driven wheels together and makes the car understeer |

**Mass keeps the mass distribution.** Changing mass scales yaw inertia in proportion, so a heavier car is
the same car made heavier rather than a car whose mass moved outward.

**Changes apply between runs, not during one.** A setup change is refused while running or in replay, and
otherwise starts a fresh run on the new car, keeping the applied grip and the stated scenario, exactly as
choosing a vehicle model does. Every setup value is already in the recording's metadata, so recordings need
no schema change; a change during a run would need parameter events and revisions like grip's, and is not
offered.

**Headless.** `--mass` (600 to 1400 kg) and `--max-power-kw` (30 to 120) apply through the registry, so their
ranges and the mass distribution match the desktop. The earlier four-wheel flags keep their full validated
ranges for experiments beyond the offered ones.

**Desktop.** A Car setup button under the TIRE SLIP card opens a panel beside it with a slider per control.
Sliders are enabled only while paused on the live four-wheel car; releasing one applies the value as a fresh
run. The panel names what the last touched control does, as its test measures it; Defaults restores the
standard car. In replay the panel shows the recorded car's values, read only.

**The rule is held for every exposed parameter**, including those only the headless runner offers: roll
balance, aero balance and drive split each get a test on the same scenario (`fd_setup_effects`).

**AGENTS.md** now reads: "The kinematic baseline has no mass sensitivity; the dynamic plants do, through their
equations. Expose only implemented, effective controls: every exposed vehicle parameter has a test proving
its effect."

## Found by building it

- **Braking points do not move yet.** The plan's visible item has brake bias and weight moving braking
  points. The speed plan still brakes where the fixed grip fractions say, whatever the car, so what visibly
  moves is lock behaviour and the car's achieved braking; at 20% front bias the rear tiles read LOCK at 8.8 s
  into the first corner. Braking points follow the car once Phase 3.1 derives the envelope from the plant.
- **Some parameters move a different outcome than expected.** Mass barely changes the 12 s distance
  (166.09 m at 600 kg, 166.63 m at 1400 kg) because the plan asks the same accelerations and 100 kW covers
  them; what it moves is grip use, 0.805 against 0.868. A lighter car also laps slower than the default
  because drag is a larger share of its force. Power above about 90 kW changes nothing on this plan.
- **A viscous coupling was slow to solve.** Alternating the two wheel updates of a coupled axle took about
  fourteen sweeps at 80 N·m·s/rad, and a 12 s scenario 5.3 s of wall time against 1.8 s open. A secant step on
  the sweep's change, falling back to the plain step when it does not shrink, gives the same results to the
  printed digits in fewer sweeps. It is still expensive: at 80 N·m·s/rad a prediction measured 6.4 to 7.7 ms and a
  blocked decision 48 to 50 ms, against 2.8 to 4.2 ms and 17.5 to 27.6 ms open (measurement only), about two and
  a half control periods. The simulation stays on fixed ticks, so a slow decision delays the picture, not the
  physics; solving a coupled axle's two wheels together by Newton's method is the obvious next step.
- **The desktop check needed more time.** The setup checks drive a fresh four-wheel run into the first corner
  to show the rear lock; stepping it in 180 small increments pushed `fd_desktop_controls` past its 30 s timeout.
  The search now jumps to 7.5 s first, the suite takes about 26 s, and its timeout is 60 s.

## Verification

`tests/setup_tests.cpp` (`fd_setup_effects`, 11 groups) drives the first 12 s of the preset from rest, under
the unchanged planner and Pure Pursuit, at two values of each parameter:

| Parameter | Values | Measured outcome |
| --- | --- | --- |
| Mass | 600, 1400 kg | largest grip use 0.805 → 0.868 |
| Centre of gravity height | 0.1, 0.6 m | largest front axle load 4087 → 4891 N; deepest rear braking slip −0.024 → −0.229 |
| Brake bias front | 0.2, 0.8 | rear locked 102 samples → 0; deepest slip rear −1 (front −0.007) → front −0.059 (rear −0.010) |
| Drag area | 0, 1.5 m² | distance 167.48 → 165.48 m |
| Downforce area | 0, 4 m² | largest grip use 0.826 → 0.788 |
| Power | 40, 100 kW | 15 m/s reached at 3.29 → 2.91 s |
| Viscous coupling | 0, 80 N·m·s/rad | driven wheels up to 3.19 → 2.14 rad/s apart; understeer angle up to 0.0102 → 0.0341 rad |
| Roll balance front (headless) | 0.2, 0.8 | largest grip use 0.937 → 0.790 |
| Aero balance front (headless, 4 m²) | 0.2, 0.8 | largest front axle load 4661 → 5230 N |
| Drive front fraction (headless) | 0, 1 | 15 m/s reached at 2.92 → 4.34 s |

A registry group checks that the offered controls are exactly the plan's seven, that each has an effect test,
a described unit and effect, an ordered range containing the default car's value, and reads back both ends,
that values beyond the range, non-finite values, unknown keys and a height the car's track cannot take are
refused, and that mass scales yaw inertia.

`fd_recording_contract` records `--mass 1200 --max-power-kw 60` and finds yaw inertia 2028 kg·m² and 60 kW in
the metadata, and refuses a mass of 2000 kg and a power limit for the single-track car before any output.
`fd_desktop_controls` (331 checks, up from 303) checks the panel's controls against the registry and the live
car, refusal while running and outside the range with the reason, a brake bias change as a fresh run shown
on the slider and value, the rear wheels locking at 20% front bias, Defaults, the recorded car read only in
replay, and no setup for the kinematic bicycle. Captures `docs/assets/car-setup.png` and
`docs/assets/car-setup-rear-lock.png` were inspected.

Headless runs on the preset, each validated with `--check-recording`, plant reproduction between 9.64e-10 and
9.96e-10 m; `setup-default-lap` is byte-identical to decision 0015's `aero-lap` apart from metadata identity:

| Run | Lap | Max tracking error | Largest grip use | Max rear sideslip | Rear locked samples | Invalid samples |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `setup-default-lap` | 34.045 s | 0.344 m | 0.826 | 0.041 rad | 0 | 0 of 6810 |
| `setup-mass-600-lap`, `--mass 600` | 34.130 s | 0.335 m | 0.805 | 0.040 rad | 0 | 0 of 6827 |
| `setup-mass-1400-lap`, `--mass 1400` | 34.190 s | 0.372 m | 0.868 | 0.044 rad | 0 | 0 of 6839 |
| `setup-power-40-lap`, `--max-power-kw 40` | 34.840 s | 0.342 m | 0.826 | 0.041 rad | 0 | 0 of 6969 |
| `setup-bias-0.2`, 20 s, `--brake-bias-front 0.2` | no lap | 0.355 m | 1.000 | 0.147 rad | 102 | 387 of 4001 |
| `setup-cg-0.6-lap`, `--cg-height 0.6` | 34.045 s | 0.347 m | 1.000 | 0.041 rad | 0 | 38 of 6810 |
| `setup-drag-1.5-lap`, `--drag-area 1.5` | 34.340 s | 0.336 m | 0.808 | 0.040 rad | 0 | 0 of 6869 |
| `setup-downforce-4-lap`, `--downforce-area 4` | 34.025 s | 0.329 m | 0.788 | 0.037 rad | 0 | 0 of 6806 |
| `setup-coupling-80-lap`, `--viscous-coupling 80` | 34.175 s | 0.515 m | 0.922 | 0.033 rad | 0 | 0 of 6836 |

The 1400 kg run records yaw inertia 2366 kg·m² and the 600 kg run 1014, both 1.69 per kilogram like the default.

## What this is not

Not a setup optimiser, and not live tuning: every change is a fresh run. The planner's braking points,
corner speeds and grip fractions do not respond to the car until Phase 3.1. Wheelbase, track widths, tires,
roll and aero balance and drive split are not desktop controls. Parameters remain illustrative.

## Consequences

- Phase 2 is complete as the plan lists it. Phase 3.1 derives the car's G-G-V envelope from this plant, which is
  where mass, height, bias and downforce start moving planned speed and braking points.
- Any new exposed parameter needs an entry in the registry and a group in `fd_setup_effects`; the registry
  group fails if an offered control has no test.
