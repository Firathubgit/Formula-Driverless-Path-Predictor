# 0050 — The hall, its barriers, and the one banked bend

*The hall's drawing below (the box, the flat barriers, the island decks) is superseded by decision 0051, which redraws it after the
user's onboard video; so is the ground behind the bank, which 0051 makes flat beyond the barrier's footing. The banked bend itself,
and what was and was not used, stand.*

2026-10-03. The user asked for the kart's surroundings to look like the real place so that they can learn the lap against what they see: the
hall, a bend that is a little higher than the rest, the roof and walls. They pointed at the operator's Unity build
(gokartcentralen.wekart.fun) as a source, "get as many assets as you need", and later sent their own onboard video of a 30.101 s
fastest lap there.

## What was and was not used

The Unity build's models and textures are the operator's or their developer's work, shipped to run in a browser, not offered for use
here, and the repository is public: none of them was taken and none is in the repository. The surroundings are drawn from scratch, in the
project's own unlit style, after what the user's video shows (a private reference kept out of the repository, like the poster): black
rubber barriers with a red rim along the track; red and white kerbs on the road's edge at the inside of the bends, with green island decks
behind them; a dark steel roof with beams under rows of white lights; red walls; dark glossy asphalt. None of it is measured. The hall is a
box 14 m round the track's footprint with its roof 8 m up, the long walls red and the short ones dark. Photographs or drawings of the hall
would replace these numbers, which are in `Main.qml`.

- **Barriers** (`TrackGeometry` kind 14): a wall 0.45 m high along each edge of the corridor, just outside it, its face near black and its
  rim red. At the inside of each bend (the coach's corners, decision 0049, projected onto the track the kart drives, with 2 m either side)
  a kerb 0.4 m wide on the road's own edge, red and white by the metre, and the island's deck, 1.9 m wide, at the wall's height. Where the
  car leaves the corridor the judge calls it off course (decision 0032); the wall is drawn and is not a collision.
- **The hall**: shown from the follow and driver's views, hidden in the overview, where its roof would hide the circuit. Settings has
  Hall and barriers to hide the room and the barriers.

## The banked bend

The poster draws the velodrome's banking at T9 (decision 0038), and the user says one bend is a little higher, so one bend is.
`fd::TrackRelief` (`core/track_relief.hpp`) gives the ground a height: a banking tilts the road across its width, over smooth ramps at each
end, the lower edge at the floor and the higher one the width times the tangent of the tilt above it. The kart's is T9, the left-hand
bend 299 to 329 m round the lap, 6 degrees, banked into the bend, so its outside, on the right, stands 0.63 m above its inside at full
tilt, with ramps of 8 m at each end. **The 6 degrees, and T9 as the bend, are chosen, not measured**: no source gives them, and whoever has
stood on it can correct both in `gokartcentralen_relief`.

The relief is drawing only. The plant is planar and never reads it: the kart is seen to climb the bank and tilt with it, the road, edges,
barriers, lines and plans rise with it and the cockpit's view tilts, and the kart drives it as if it were flat. A real bank would give grip
and push the kart down the road; this does neither, and says nothing about lap times.

## What the video says about the kart

The video's lap is 30.101 s. The fitted kart (decision 0046) laps the poster's line in 34.46 s at the whole of its envelope, so the model's
limit is 4.4 s slower than that real lap, and the user's 36.38 s is 6.3 s behind it. Either the real limit is nearer 30 s, which a better
line, a stronger or lighter kart, or a looser fit would give, or that lap was set in something other than the kart modelled. It was not
refitted here, and the fit's tests are unchanged. It is a finding for the user to decide on.

## Checks

`fd_track_relief` (four groups): flat ground without a banking and on any track but the kart's; the tilt over its ramps, half at half the
ramp, full through the middle; the surface across the road, the left edge at the floor, the right edge the width times the tangent above
it, linear between, held beyond; a negative tilt lifts the left edge; bad bankings refused (no ramp, shorter than its ramps, before the line,
past the lap, 50 degrees, overlapping, not a number); and on the real conditioned Göteborg track, T9's outside is 0.5 to 0.8 m above its
inside, the ground is flat at seven other places round the lap, and it never changes by more than 8 cm between half-metre steps. The
desktop's UI suite: the road geometry's highest vertex is the banked bend's outside, about 0.64 m, and its lowest is the road's own 0.01 m;
every coaching ribbon vertex is on the ground at the relief's height; the hall and its barriers are drawn, have their switch, and the roof
leaves the overview.
