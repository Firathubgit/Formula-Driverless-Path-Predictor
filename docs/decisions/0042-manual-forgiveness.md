# 0042 — Independent manual forgiveness controls

2026-09-28. The user wants the existing overall 1–100 assistance control and a
simple manual alternative that allows strong acceleration with weak braking,
gentle joystick steering and independently chosen grip.

Forgiveness now offers Overall and Manual. Overall keeps decision 0040's curve
unchanged. Manual offers five independent 1–100 sliders: Acceleration, Braking,
Joystick steering, Grip assistance and Top speed. Higher always means more of
that quantity. Top speed also shows km/h. The driver label says Manual assist.
Both modes remember their values when switching between them or switching the
main toggle off. The main toggle off restores the original kart regardless of
the manual settings. Reset retains tuning; leaving kart discards it.

For each manual value v, u = (v − 1) / 99:

| Control | Mapping |
| --- | --- |
| Acceleration | Full throttle requests 0.5 + 11.5u m/s² |
| Braking | Full brake requests −(1 + 21u) m/s² |
| Joystick steering | Scales the existing arcade steering limit by 0.15 + 1.35u, bounded by the kart's lock; actuator rate is 0.3 + 2.7u rad/s |
| Grip assistance | Existing artificial rolling relaxation, strength 1 − (1 − u)³; 1 retains ordinary tire grip, 100 uses exact no-slip rolling |
| Top speed | 10 + 50u km/h, no higher than the source car's controller limit |

Steering uses the existing arcade stick shaping (power 1.15) and speed-dependent
limit (the geometric turn at 3 g before the manual multiplier). Grip does not
change pedal or steering requests. With less grip, actual forces can still limit
the requested acceleration/braking; these are deliberately artificial training
controls, not modifications to the stored physical profile. The existing speed
governor tapers acceleration near the cap and can brake if already above it.
Manual defaults: 50 acceleration, braking, steering and speed; 100 grip.

`DrivingAssistance` carries the settings through the existing vehicle seam.
Manual remains an active override even at grip 1, since pedals, steering and
speed remain adjustable. No second plant or path snapping is introduced.
Every setting validates before mutation, changes commit together at the next
fixed tick, and changed fields receive parameter events with the same revision
and time. Changes do not reset the kart or the practice lap clock. Human-only,
no recording; ordinary tire-limit display/setup restrictions remain in force.

Tests measure each control's effect on actual motion, verify independent pedal
and steering requests, normal-grip manual activity, off restoration, field
validation, atomic tick events and reset retention. UI checks operate all five
sliders with a high-acceleration/low-braking/low-steering combination, reject a
bad edit, verify live application and switch back and forth between modes.
Build and visual evidence is recorded in `docs/STATUS.md`.
