# 0047 — A grip aid for the stick and the pedals

2026-10-02. The user drove the fitted kart (decision 0046) with the Xbox controller and found it glided and lost grip in every
corner. They pointed at why: a thumbstick is easy to push to its end and a kart's steering wheel is almost never at full lock, so
the controller asks the kart for turns no one makes. They asked for the sliding toned down "a lot", with some unrealistic help
for playability, only here.

## What the aid is

`fd::GripAid` (`core/human_driving.hpp`) is a limit on what a person's controls ask of the plant, given the car's own
performance envelope (decision 0017) and a share, 0.9 for the realistic kart:

- **The stick.** A full stick asks for the share of the plant's own lateral limit at the car's speed (the lesser of the two
  sides) and no more: the wheel angle that geometry needs for that acceleration, plus the share of the front tires' peak slip
  angle. Past it the kart pushes wide, as a kart steered beyond its grip does, instead of spinning. The aid only ever takes steering
  away: where the plain limit is already lower, it stands.
- **The pedals.** A floored throttle or brake asks for no more than the plant has left of its friction circle beside the cornering
  already under way, with `1 - share` of the corner kept in reserve. On a straight they are untouched; a brake is never taken away
  altogether, at least a third of the straight-line braking remains.

It is apart from Forgiveness (decisions 0040, 0042), which is off in the realistic kart, and it is never a change to the plant: the plant
is the kart as fitted, and the command it receives is a person's command, limited. `Simulation::set_grip_aid(share)` turns it on,
in force from the next control decision, for the four-wheel car only; it uses the plan's performance envelope when one was derived
for this car and grip, else derives one of its own (about 0.2 s). A person's drive is never recorded (decision 0037). The earlier
kart has none.

## Checks

2026-10-04: at the user's request, You > Gokart > Realistic kart scales the aided steering command by 1.848 (initially 1.4,
then another 10% increase, then 20% over that), capped at
the existing physical lock. This deliberately relaxes the steering restriction above; the pedal aid remains at 0.9.
The earlier kart and autonomous driving retain their existing commands. Forgiveness retains its separate steering controls.
The gain defaults to 1 for other callers. Tests cover both directions, partial/full stick, speed, lock saturation,
unchanged pedals, invalid gains and application through the simulation's steering actuator.

`fd_human_driving` (`tests/human_driving_tests.cpp`): at every speed the aided steering is no greater than the plain, and at 6 m/s and
under (the hairpins) it is clearly less; a gentle pedal on a straight is unchanged; mid-corner the aided throttle and brake ask for less
and the brake still brakes; with no envelope the aid changes nothing, and a share outside (0, 1] is refused; and a full stick held for
four seconds from 6 and from 9 m/s, with the throttle floored, steps the kart's rear out less with the aid than without. The
desktop's UI suite checks that the realistic kart has the aid at 0.9 and the earlier kart has none.

## What it is not

Not a measurement of a kart's steering: it is a playability help the user asked for, and it changes how far a stick can turn the
wheels, not how the kart behaves once it does. The share, 0.9, is a feel, to be changed if the kart still slides or is too
planted.
