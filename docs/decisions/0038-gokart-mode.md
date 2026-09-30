# 0038 — Gokart mode: Gokartcentralen Göteborg and its rental kart

2026-09-28. The user drives a real indoor kart track, Gokartcentralen Göteborg by the central station, and asked for it
as a hidden training mode under You (decision 0037): the track as the operator's poster draws it, the car's statistics
replaced by the rental kart's (size, speed, acceleration, braking, grip), the chosen car's looks kept. They supplied the
poster and two research dossiers, and asked that the track be checked by another agent scoring its resemblance to the
poster from 1 to 10, 9 a pass.

## The track

`tracks/gokartcentralen-goteborg.csv` is traced from the operator's published poster (1500 x 2000 px; the user's image,
kept out of the repository as a private reference) by `tools/tracks/trace_poster.py` from the control points in
`tools/tracks/gokartcentralen-goteborg.points.json`: points in the poster's pixels, in lap order from the chequered line,
each snapped perpendicular to the lap onto the middle of the drawn asphalt between the white lines unless fixed, a periodic
smoothing spline through them resampled every metre and scaled so the lap is exactly the published 400 m. Clockwise, as
published (högervarv). The pit lane is not part of the lap; the footbridge's grey band and the lettering on the asphalt are
not barriers; the velodrome's shaded banking at T9 is asphalt, its drawn wall face outside the black lip is not.

Scaled to 400 m the drawing is close to scale: the drawn asphalt's median width about the traced line is 6.5 m (10th and
90th percentiles 5.5 and 7.9 m) against the published 6 m, and the footprint is 45 x 75 m. The simulator lays the published
6 m corridor about the line. Where the poster draws the asphalt wider (the outer bulb of T4, the velodrome apron) the extra
is left outside; where narrower (the T9 exit leg, 4.2 m drawn) the corridor overhangs the drawing evenly. The traced line
has the poster's ten turns in its order and directions, T1 right, T2 a left kink, T3 right, T4 right, T5 left, T6 left, T7
right, T8 right, T9 left, T10 right, with centreline radii of 3.5 to 9 m in the hairpins.

Verification, as the user asked: each round three independent agents judged the trace against the poster through a
different lens (path and topology; geometry and proportions, measured along normals and scans; racing context and
recognisability, including whether the poster's racing line fits the corridor), each scoring 1 to 10, every judge to pass
at 9; below that a fixer agent checked each issue against the poster, moved control points and traced again. Version 1
scored 8, 8 and 8: the bottom of T9 ran 1.3 m outside the middle, on the velodrome's wall face, and the T6 hairpin was
squashed toward its island. Version 2 scored 9, 9 and 9: every straight within about 4 px (0.2 m) of the drawn band's
middle, every turn centred, its only deviations the intended ones at T4 and T9. Everything the judges said and the
fixer did is in `docs/evidence/gokart-track-verification.json`.

The deep-research dossiers' corner radii (T7 5 m, T1 and T4 10 m and so on) and their 53 x 93 m footprint are their own
tracings' estimates; the poster itself is the reference here, and the dossiers' published facts (400 m, 6 m, clockwise,
three sectors, electric karts) are what the trace honours. The dossiers also warn that the poster's file name dates from
2021 and the operator describes a new hall; the trace is of the poster the user gave.

## The rental kart

A physics profile, `gokartcentralen-rsx2` (decision 0033's rules: every value marked published, derived, fitted or
chosen): a Sodi RSX2 rental kart, 186 kg with its batteries (Sodikart) plus a 75 kg driver; about 14 hp (Gokartcentralen),
10.3 kW; about 60 km/h on the straight (Gokartcentralen); on the racing kart's (Lot 2016) geometry and tire shape, which
neither source publishes, with a rental kart's friction of 1.0 both ways. It floors from rest to 50 km/h in 3.2 s, tops out
just under 60 km/h, corners at 0.89 g at 8 m/s and brakes at 0.75 g.

The 60 km/h is the kart's drive controller, not a cap on the driver: the four-wheel car gained `max_drive_speed_mps`, the
controller's top speed at the driven wheels' rims, where driving torque falls from full at 95% of it to none at it, braking
untouched. Infinity, the default, is no limit, so every other car drives exactly as before and its fingerprints are
unchanged; a car that has one records it, which is recording schema 18, and a schema 17 recording that claims one is
refused. fastest-lap's model has no such limit, so a car with one has no lap problem (decision 0034).

Two chosen values rest on what the model cannot do. A kart's solid rear axle turns by lifting its inner rear wheel, which
this model's load transfer does not reproduce: locked (200 N m s/rad) the axle scrubs the kart to 0.4 g at 8 m/s; open, the
unloaded inner rear wheel locks under braking and spins it. 1 N m s/rad corners at the tires' grip and brakes straight, and
was stable in every drive tried. And a rear axle alone locks near 0.45 g, below the 0.60 to 0.68 g the dossier gives rental
karts; with the RSX2's brake layout unpublished, 40% of the braking at the front brakes at the tires' 0.75 g without
locking an axle first.

At its limit, planned from its own performance envelope on the minimum-curvature line of the traced track, the kart laps in
about 40.5 s; the operator's Speedkart qualification asks a standard kart under 34.0 s. Raising the tires' friction to 1.2
gains only 3 s, so most of the gap is the poster's tight hairpins, 3.5 to 6 m at the centre, and the line a solver lays in
them; the grip was not raised to meet the number.

## Gokart mode

Under You in the settings, Gokart turns the mode on while paused: a fresh run on the traced track, conditioned with light
smoothing (the file is already smooth), the rental kart, Pure Pursuit aiming 2 m ahead as on the traced Formula Student
courses, and the judge on its laps, sectors and excursions with no cones, as the track's barriers make it. Whoever drives
keeps the car; the car, its control, plan and line stay the kart's until the mode is left, which returns the track, car
and configuration it came from; the car's setup stays open, so a heavier driver can be weighed in. The car keeps its looks
and is drawn scaled uniformly about its rear axle so its wheelbase is the kart's 1.045 m, its reflection with it; the
chase camera comes to 0.4 of its distance, and the plan's band and the sideslip arrow are drawn finer. `fd_desktop --kart`
starts in it, for captures.

## Checks

`fd_track_conditioning`: the file is 400 m, conditions as a 6 m corridor the simulator accepts, runs clockwise and has
the poster's ten turns in order. `fd_vehicle_profiles`: the profile's sourced values, its top speed and 0 to 50 km/h, and
its cornering at 8 m/s. `fd_wheel_model`: the controller's top speed holds a floored car just below it while the same car
without one runs past, leaves braking untouched, and is refused unless positive. `fd_recording_playback`: the rental kart's
top speed round-trips, is checked against its profile and is refused under schema 17. The UI suite switches gokart mode on
through its switch and checks the track, the kart, the display and the drawn size, the locked car, a drive from the line
under 60 km/h, and the return to the car and track it came from.
