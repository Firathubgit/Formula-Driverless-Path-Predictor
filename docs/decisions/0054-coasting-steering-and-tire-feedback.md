# 0054 — Coasting steering travel and tire-based feedback

2026-10-04. The user asked for much more steering while neither pedal is pressed,
specified at least 17 degrees at all speeds, and requested a light road vibration
while coasting, with stronger feedback when pushing the kart too hard.

The realistic kart's grip aid now has a coasting steering floor of 17 degrees.
With both pedals released its steering limit is the larger of the ordinary
speed-dependent limit and that floor, never beyond the physical lock. The maximum
of throttle and brake, over 0..0.25 travel, smoothly blends the coast limit back to
the ordinary speed-dependent limit with `t*t*(3-2*t)`. Thus both pedals restore
the usual restriction progressively; acceleration and braking cannot cancel each
other to enable the coast allowance. Stick sensitivity remains 1.1 and clamps at
the selected limit. Controls remain sampled at control decisions; the ordinary
steering actuator rate limits the wheels' response.

This is wheel travel only. The fitted tire model, forces, motor limits, wall
contacts and absence of artificial grip remain as in decision 0053. Asking for
17 degrees at high speed can cause understeer/sliding; it does not guarantee a
17-degree no-slip turn. Other callers keep the previous behavior by defaulting
the coast floor to zero. Invalid floor values are refused before mutation.

Controller and cockpit overload vibration now read actual tire slip relative to
its peak: the largest combined wheel slip (or axle slip/peak for tire models),
smoothly mapped from 0.85..1.5 of peak to 0..1 feedback. Lateral acceleration alone
no longer implies tire scrub. This covers cornering, wheelspin and braking slip.
The plant never reads this feedback.

At 60 km/h with no scrub the chosen road hum is 6.3% heavy motor and 1.05% light
motor; the existing softened overload contributions remain 17.5% and 19.25%.
Motor output stops while stationary/paused. The cockpit's speed-based road motion
is at most 1.2 mm vertically and 0.08 degrees in pitch; the softer scrub shake and
lateral lean are retained. These are presentation settings, not measured vibration.

Checks cover both steering directions at 0/20/40/60/80 km/h, progressive response
to either pedal, physical lock and invalid settings, high-speed tire saturation,
light road hum, actual slip rather than lateral-g thresholds, and bounded/muted
motor output. UI checks cover the coasting description and camera bindings.
