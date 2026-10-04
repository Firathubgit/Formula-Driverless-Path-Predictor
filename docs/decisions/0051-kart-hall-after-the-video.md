# 0051 — The kart hall redrawn after an onboard video of a lap

2026-10-04. The user sent their own onboard video of a lap of Gokartcentralen Göteborg (40.6 s, a 30.101 s lap) and asked for
"a mega audit of the video and environment", the hall made "pristine and close to the real thing", so that what they learn in
the simulator they can recognise on the real track. Decision 0050's hall was a box with flat barriers; this replaces its
drawing. The banked bend (0050) and the coach (0049) stand.

## What was and was not used

The video is a private reference kept out of the repository, as the poster is. Nothing of it is in the repository: the hall is
drawn from scratch in the project's own flat-shaded style, after what the video shows, and no lettering, logo or sponsor's
banner is reproduced (the banner on the ninth corner's island is a plain dark panel). The operator's Unity build is not a source
of assets (decision 0050). **Nothing is measured.** Heights, distances to walls, lamp counts and the bank's angle are chosen to
look as the video looks.

## The audit

The video was read frame by frame at two frames a second and laid against the traced track (decision 0038), whose ten turns it
follows in order. Stations are metres round the 400 m lap from the start line; the times are the video's.

| Video | Station | What the video shows | What is drawn |
| ---: | ---: | --- | --- |
| 30.1 to 33 s | 366 to 400 | The start straight along a red wall with a band of dark windows; a red footbridge overhead; numbered grid boxes | The gallery wall along the outside, glazed for 27 m; the footbridge at 378 m; eight grid boxes with their numbers |
| 3.5 to 4.5 s | 9 to 21 | The red wall ends at a door; a black wall with arches | The gallery runs to 18 m with a door near its end; arches on the south wall |
| 4.5 to 6 s | T1, 24 to 40 | A green island behind a red-rimmed barrier with green braces; a long yellow wedge on a far black wall | Islands, rims and braces inside every bend; the wedge and two thin rays on the west wall |
| 7 s | T2, 49 to 55 | An island on the left; a round red deck up to the right | The same; a round red deck on one column over the ninth corner's island, violet lights under its rim |
| 7.5 to 9.5 s | 60 to 100 | A straight along a black wall under two rows of lamps, toward a timing board with painted wedges and a lit panel | The west wall 1.6 m behind the barrier; rows of lamps; the board as rows of coloured bars, wedges and panel beside it |
| 11 to 13 s | 115 to 140 | A straight toward red walls with a balcony | The east wall red, with a balcony's deck edge, posts and yellow rail |
| 13.5 to 14.5 s | T4, 142 to 169 | A hairpin round a large green island with a pillar; its outside a wall of red, white and black blocks | The same |
| 16 to 24 s | T5 to T8 | Bends with red-rimmed islands, kerbs, stacked black barriers between the legs | The same, by the general rules below |
| 24.5 to 27 s | T9, 299 to 329 | The camera tilts: a banked left-hander with a pale scuffed surface, round an island with a slatted green fence, then a dark banner under a green rail | The bank (0050); a faint pale band on its road; the fence from 3 m before the corner, a plain banner for 18 m after it |
| 28 to 29.5 s | T10, 348 to 365 | An island with a pillar; the outside barrier blue-rimmed | The same |
| everywhere | | A dark polished floor in which every lamp stands as a long streak toward the camera; black twin-tube barriers; a red band and a white line at their foot; red and white kerbs inside the bends; a dark pitched steel roof | See below |

The comparison was made with the simulator's own scene captured at 24 stations round the lap (`--at-station`, `--capture-scene`,
below) beside the video's frame at the matching moment, twice: after the first pass the floor was too blue and too even, the
fence ran 25 m too far up the straight, the bridge stood 10 m too early, the gallery ended 8 m too soon, the east wall was a
bare red plane and the round deck was missing; the second pass corrected those.

## How it is drawn

`apps/desktop/kart_hall.{hpp,cpp}` builds the hall as six parts of plain coloured triangles from the track, its bends (the
coach's corners, decision 0049) and the bank; `TrackGeometry` kinds 14 to 19 hand them to the scene. A part is built once for a
track and kept (the whole hall builds in about 0.1 s).

- **Barriers** (14): two stacked eight-sided tubes, 0.45 m high, their faces shaded as light from above leaves them, standing
  0.05 m outside each edge of the corridor. Black; the upper tube red inside every bend and along the fence; red, white and black
  blocks outside T4; blue outside T10. Green braces lean on the island side of the red rims. At a hairpin's hub, where the inside
  edge closes to a point, nothing is placed past the bend's centre.
- **Floor** (15): the hall's floor in half-metre squares coloured at their corners, and a strip behind every barrier so the
  squares' stepped border is never seen. Islands are the floor within 6.5 m of the inside barrier of a bend, wherever nothing of the
  road lies between, fading into bare floor over the last 1.2 m. On the road's edges: a red and white kerb 0.6 m wide inside each
  bend, a white line and a red band everywhere else. The start grid: eight staggered boxes, each a painted bracket with its number
  in seven strokes.
- **Shell** (16): four walls, a pitched roof in strips on rafters, the gallery, the footbridge, the pillars in the islands of T4 and
  T10, the round deck, the fence and banner, the timing board, the wedges, the arches, the balcony. Hidden in the overview, whose
  camera is above the roof.
- **Lamps** (17) and their **glow** (18): a grid of round lamps 5.6 m up, rows 9 m apart, a lamp every 3.6 m, each with a haze
  that is brightest at the lamp and gone at its rim; small strip lights on the roof between the rows.
- **Wear** (19): the pale band on the banked bend, blended over the road.

Barriers, floor and wear rise with the banked bend; the shell and the lamps do not. The bank's ground now holds for the barrier's
footing (0.32 m beyond the edge) and is flat behind it, on a retaining wall, where 0050 held the height indefinitely and so lifted
the floor behind the bend.

## The floor

The floor is what makes the place recognisable, so it has a shader of its own (`qml/shaders/ground.frag`, off for every other
track). For each point of the floor the mirrored ray toward the eye is followed along the ceiling across the lamp grid's lines,
whichever family it crosses more squarely; each crossing counts by how near the ray passes a lamp and how near that lamp's
elevation is to the mirrored one, spread by 0.13 rad up and down and hardly at all sideways. That anisotropy is what draws a lamp
out into a streak toward the eye, as a polished floor does. On top: a faint mirror of the lit roof that grows as the eye looks
along the floor, a grain so it is never one flat colour, and dark surfaces lose their tint (asphalt under white lamps is a neutral
grey) while paint keeps its colour. The shader is told the walls and the lamp grid by the bridge (`hallBounds`, `hallLampGrid`,
`hallLampHeight`), from the same `KartHallLayout` the lamps are drawn from, so the lamps mirrored are the lamps drawn.

It is an appearance, not a light simulation: only the lamps and the roof are mirrored, not the walls, the barriers or the kart.

## The cockpit

From the driver's seat (FP) the kart's own front is drawn (decision 0049's wheel, extended): the steering wheel with hands at nine and
three, the nose panel with its neon chevron, the bumper, the floor tray and the two front tires, which steer with the plant's own
front wheels. The eye is 0.82 m up, looks 8 degrees down and sees 85 degrees, so the tires stand at the foot of the view as in the
video. It is part of the kart, so it stays with the kart while the seat's vibration moves the eye, and it is drawn only from
the seat: from outside the car keeps the looks the showroom gave it (decision 0038).

## Captures for comparing

`fd_desktop --at-station METRES` drives the run, tick by tick as any run, until the car reaches a station of the lap (at most three
minutes of it), and `--capture-scene FILE` saves the driving scene alone, without the display over it, 1920 pixels wide. Together
they give the same place again after every change, which is how the audit above was made and how it can be repeated.

## Checks

`fd_kart_hall` (`apps/desktop/kart_hall_tests.cpp`, 852,557 checks): the ten bends in order and direction; the walls outside the
corridor, every lamp inside the room and under its roof, and no node of the lamp grid inside the walls without a lamp, so the
floor's shader invents none; every drawn lamp on the grid at the mirrored height, each haze brightest at its lamp and gone at its
rim; no part of a barrier in the road and none away from its edge, with red rims, green braces, T10's blue rim and T4's white
blocks present, 0.45 m high on flat ground and raised by the bank's 0.63 m at T9's outside; the hall's floor flat everywhere
behind the barriers, the bank's surroundings included; inside the corridor only paint, and no island reaching the road; the
shell within its own walls, with nothing of it in the road below 2.7 m (probed at every triangle's middle and sides), and the
gallery, fence and footbridge present; the wear band on the ninth corner's road alone, faint, lying on the banked surface;
a part asked for again the same part; and any other track given a plain room with no fence and plain black barriers. The same
suite holds decision 0052's collision layout against this shell.

`fd_track_relief` (four groups): the bank's height holds under the barrier's footing and the floor is flat again 0.4 m beyond the
edge. The desktop's UI suite (1,001 checks, no QML warnings): the six hall parts are whole triangles; the bridge hands the shader the
walls and lamp grid the hall is drawn with, and every lamp of the grid is drawn at the mirrored height; the floor is polished in the
hall, matt with the hall switched off (when the plain edge lines return) and in the overview, where the roof and lamps are hidden; the
cockpit is not drawn from outside, is drawn from the driver's seat, its wheel turns 3.5 times the front wheels' angle and its front
tires turn exactly with them.

Run on 2026-10-04 with everything else: `./scripts/build.ps1` passed all 33 core suites (423 s) and `./scripts/build.ps1 -Desktop`
all 40 (593 s). The look itself was judged by eye, on the 24 station captures beside the video's frames (kept in
`out/kart-hall-audit/`, outside Git); it was not driven with a controller in this session, and no frame rate was measured after the
second pass.

## What it is not

Not a survey: every dimension is chosen. Not the operator's model. Not complete: no people, no other karts, no lettering, no red
stair tower, and the gallery's inside is a row of lights. The hall's plan is a rectangle round the track's footprint; the real
hall's is not known. Walls are not mirrored in the floor. The bank is 6 degrees because the user called the bend "a bit higher";
the video, whose camera leans with the driver, does not give its angle. And the corridor is the published 6 m everywhere, where
the real asphalt varies, so hairpins' islands are smaller here than there.

## Consequences

- Gokart mode is drawn in the hall by default; Settings (Hall and barriers) returns the bare track, with its matt road and plain
  lines, exactly as before.
- The hall's ground-level shell is also what a person's kart collides with (decision 0052, which slices this shell at bumper
  height), so a wall added to the shell at ground level is a wall the kart meets; `fd_kart_hall` holds that nothing of the shell
  stands in the road below the footbridge.
- `KartHallLayout` is the one description of the room; the lamps, the floor's shader and the collision layout all read it.
