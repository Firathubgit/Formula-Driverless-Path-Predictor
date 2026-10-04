# 0049 — A coach, a cockpit and a controller that feels the kart

2026-10-03. The user, driving the fitted kart (decisions 0046, 0047), asked for what would help them drive "the perfect lap": the
steering wheel turning, the kart's vibration, and on-screen cues such as "apex now" and more turning angle, as a toggle.

## The coach

`fd::KartCoach` (`core/kart_coach.hpp`, no Qt) reads the poster's coloured line (decision 0039) and the car's pose and speed, and
says what to do next. It decides nothing, drives nothing and is never in a recording or the judge's reading.

- **Corners from the line's own curvature.** The line is resampled every 0.25 m and smoothed by a 2 m Gaussian to remove the hand-drawn
  stroke's wobble. A run of one sign tighter than a 20 m radius is a corner; runs under 3 m apart are one corner, and runs under 3 m long
  are dropped. The poster line gives ten, in the order and directions of the track's ten turns (decision 0038): right, left, right,
  right, left, left, right, right, left, right. The apex is the middle of where a corner is within 15% of its tightest, so a long
  hairpin's apex is the middle of its arc.
- **Where the car is.** The nearest point of the line, within 25 m of where the car was while it is on the line, projected onto its
  segment for a station along the line and a signed offset (left positive). The car is on the line within 8 m of it.
- **The call.** One at a time, in this order: **APEX NOW** (inside a corner, within 0.15 s of travel of the apex), **BRAKE** (the line is
  red, or its next red is within 0.3 s of travel plus 2 m), **TURN IN** (before a corner, within 0.3 s of travel or 2 m of where its
  curvature begins), **LIFT** (the line is blue), **GAS**. Also: which corner (T1 to T10), its side, how far to its turn-in or apex,
  how far to the next red, and how far the car is from the line.
- **Steering.** The kinematic wheel angle that follows the line's curvature a tenth of a second ahead, `atan(wheelbase * curvature)`,
  positive left, drawn beside the car's actual wheel angle with "MORE LEFT 4°", "MORE RIGHT" or, within 1.5 degrees, "STEERING OK". A tire
  needs more angle than geometry at its limit, so in a fast bend the card may ask for a little less than the car needs: it is the
  line's geometry, not the grip.

The card sits at the top in kart mode with a person driving, clear of the circuit card; Settings has Coach cues to hide it. The coloured
line stays on the road as ever (decision 0039); the coach adds to it. Like it, the coach is static advice read from a poster, not a solved
optimum and not adapted to the driver, except for the lead times above.

## The cockpit

First-person view (FP) in kart mode draws a kart's steering wheel (a 28-piece rim, three spokes, buttons, a yellow top marker) with gloved
hands at nine and three, turned 3.5 degrees for each degree of the front wheels: a kart's wheel turns about 110 degrees either side, and the
plant's lock is 0.55 rad. The ratio is a drawing assumption; the angle it turns is the plant's actual steering angle. The seat's vibration
moves the driver's eye, never the kart: a road's hum that grows with speed, a buzz when the tires scrub (lateral acceleration beyond 7 m/s^2,
full at 12), a lean toward the outside of a bend, and none while paused. Settings has Seat vibration.

## The controller

The Xbox controller's two motors follow the same things (`rumble_for` in `apps/desktop/gamepad.cpp`, through XInput): the heavy motor hums
with speed (0.12 at 60 km/h) and shudders with the scrub (0.5 more); the light one buzzes with the scrub alone (0.55). Nothing while paused or
when not driving the kart. Settings has Controller rumble. The plant never reads it.

## Checks

`fd_kart_coach` (six groups): a stadium's corners, radius, apex, direction and steering angle, anticlockwise and clockwise; the calls along its
brake zone and turn; the offset's sign, "off the line" at 8 m, and that a hint keeps a hairpin's other leg from being mistaken for the line;
bad lines refused (too few points, a repeated point, a gap, a non-number, a bad wheelbase); the poster line's ten corners in order and
direction, each apex called as an apex with a steering angle on its own side and of a kart's size; and a walk along the whole poster line,
every point on the line, the station never going back, every corner called to its apex, and brake, lift and turn-in calls all made.
`fd_gamepad`: the rumble is nothing paused or standing, hums at speed, shudders at the limit either way and stays within the motors'
strength. The desktop's UI suite: the coach reads ten corners at the start line and says gas, with the brake zone and T1 right counted
down; its switch, and the hall's, work.

## What it is not

Not a coach that watches you: it reads the car's place on a line, not how well it was driven, and it does not score. Not the optimum: the
poster's line is what the poster paints, and the cues are its colours and bends. The wheel's ratio and the vibration are drawn to feel
right, not measured.
