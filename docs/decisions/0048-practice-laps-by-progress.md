# 0048 — Practice laps timed by progress along the track

2026-10-02. The user reported that the kart's lap clock did not always register the lap: sometimes a lap read about 72 s, two laps
in one, and sometimes three laps in a row were missed. They asked that every lap of about 35 s be timed, no matter what, and that a
double lap never appear again.

## Why twelve gates missed laps

Decision 0041's clock timed a lap by twelve gates spread round the track, crossed in order. A kart that cut inside a bend, or ran
wide of a gate's width, crossed a gate beside it and the clock waited for a gate it had already passed; the lap was then timed
when the kart crossed that gate on the next lap, and the lap read as two. Rocking across the line, driving backwards and a pose
that jumps were all kept out by those same gates, so loosening them would have let those in.

## Progress instead of gates

`fd::PracticeLaps` (`core/competition.hpp`) now follows where the kart is along the track, by projecting its pose onto it, from a
start at the first accelerator input (still before, or at, the line). Three rules:

- **The furthest point reached.** The place along the track is accumulated step by step from the two projections of each tick, then
  compared with the furthest it has been. On the inside of a tight bend a projection ticks back as it passes each vertex of the track
  and forward again after it; that is nothing, because only a new furthest point counts. Rocking over the line, reversing and turning
  round add no new furthest point.
- **A lap is a whole track length from where the lap began.** When the furthest point passes the lap's end, the lap is timed at that
  instant, interpolated within the tick, and the next lap begins there. A lap needs at least 60% of the track length in ground
  actually covered, which keeps a shortcut from counting (the inside line is about 90% of the middle's).
- **Nothing counts that the kart's own travel does not explain.** A step with the kart further than half the corridor and 4 m
  from the track, a pose that moves faster than 150 m/s, or one whose projection advances more than six times its own travel
  plus 2 m, adds nothing. A jump of the pose round the track is no lap.

There is no gate to miss. A kart that cuts a hairpin, runs wide or takes the inside of every bend times every lap, because the
track is complete when the kart has made it round.

## Checks

`fd_competition` (`practice_laps_start_on_gas_and_time_every_lap`): waiting or pressing Play alone does not start the clock; gas
starts it at the beginning of its tick; rocking across the line, a jump of the pose round the track, reverse travel and six metres
outside the corridor complete no lap; a lap on a line at the middle, 2 m either side and right at either edge of the corridor is
timed, and a faster and a slower lap update Previous and Best; twenty laps in a row are all timed, each alone and none merged with the
next; restart keeps history and a new session clears it. `fd_track_conditioning` drives the real Göteborg track two laps at offsets
of 2 m left, the middle and 2 m right and finds each lap at its true time to 0.01 s. The desktop's UI suite runs with
the new clock.

## What it is not

The clock reads the true pose only and is for practice: it is not the judge (decision 0032), whose laps, penalties and verdict
are separate and unchanged. It times what a kart drives on this track; a kart that is lifted onto the track and set down further
round is not timed, by design.
