# 0040 — Opt-in kart forgiveness for learning to drive

2026-09-28. The user wants to start with a planted, deliberately unrealistic game-like
kart, then work down toward realism. Their final scale definition governs: **1 is the
existing kart, 100 is maximum help**, and 50 is still very easy. A separate toggle restores
the original handling regardless of the remembered slider value. This is available in
Settings under You → Gokart, while paused or driving; it starts off with 100 ready.

## One plant, with explicit artificial assistance

`DrivingAssistance` is ordinary data in the vehicle seam. Its level is validated in
1..100. Disabled and level 1 delegate to the original `human_command` and `advance`
without changing any result. The source kart's configuration and physics profile are
never overwritten. Enabled above 1, only a person driving a four-wheel car may use it;
the desktop exposes it only in kart mode. Autonomous driving, replay and recordings do
not use this override. The person's drive remains unrecordable.

For t = (level − 1) / 99, assistance is a = 1 − (1 − t)³. This gives substantial help
at 50 (about 87%) and a gentle progression through 70, 80 and 90. `human_command` blends
the normal pedals toward 6 m/s² acceleration and 18 m/s² braking, and steering toward
a direct, speed-sensitive geometric turn allowing up to 3 g, bounded by the kart's
existing steering lock. Stick shaping changes from power 1.5 to 1.15; actuator rate
blends toward 3 rad/s. The speed target falls linearly from the kart's controller limit
(60 km/h) to 35 km/h. Positive acceleration tapers within 1 m/s of that target; excess
speed requests braking of min(12, 4 × excess) m/s². Speed is never assigned to the target.

`advance_assisted`, in `core/vehicle.hpp`, advances the same four-wheel state through
the existing integrator. Its dynamic derivative's weight is multiplied by 1 − a, so
the existing relaxation toward rolling motion also acts at speed. Wheel speeds relax
toward rolling at 80a per second (the exponential uses elapsed substep time), providing
deliberate anti-lock and anti-spin assistance. At 100 the integrator uses its exact
no-slip rolling branch at every speed. No second plant advances, no result of two
integrated poses is mixed, and no track, suggested path, cone or judge steers the kart.
The actual position is always integrated and is what the display and judge observe.

These virtual forces deliberately override normal grip, power and tire behaviour. They
are not a physical ABS/traction-control implementation or a calibrated real kart. The
driver label says Assist and the level. While active, ordinary tire-limit and envelope
cards give way to an explanation of assistance; wheel appearance says ASSIST rather
than claiming physical grip/lock. Body sideslip still comes from actual state, and the
instruments' lateral acceleration is measured from the integrated motion. Setup sliders
are unavailable while forgiveness is enabled. The planner remains unassisted, labelled
as such in the assistance explanation; the poster's fixed coaching line is unchanged.

## Lifecycle

Changes validate before mutation and coalesce until the next fixed simulation tick.
That tick commits the level/enabled state with one revision and parameter events at its
starting simulation time. The sample already at T keeps its old revision; motion after
T uses the new setting. A change between control decisions recomputes the person's
command and leaves the regular control schedule intact. Changing a slider never advances
time, resets the run or snaps the pose. Reset retains the selected assistance and releases
the pedals. Switching it off resumes ordinary physics from the actual state. Handing
back to autonomy disables it; leaving kart mode discards it with the kart's run. Replay
refuses assistance changes and displays only its recorded run.

## Measured behaviour

The deterministic `fd_human_driving` checks use the rental kart, 5 ms simulation ticks,
20 ms held controls, eight seconds of full throttle, straight braking from 9 m/s, and
1.5 s of 25% brake plus 80% left steering from 9 m/s. These are synthetic input tests,
not physical controller driving or real-kart validation.

| Level | Speed after 8 s (km/h) | Stopping distance (m) | Peak rear sideslip in brake/turn (degrees) |
| --- | ---: | ---: | ---: |
| 1 | 59.5821 | 5.59351 | 14.7909 |
| 50 | 47.6029 | 2.56153 | 0.732298 |
| 70 | 42.5715 | 2.31384 | 0.183228 |
| 80 | 40.0494 | 2.26872 | 0.0559601 |
| 90 | 37.5251 | 2.25233 | 0.00707633 |
| 100 | 35.0000 | 2.25000 | 0 |

Checks also cover original-model equality when off/at 1, invalid values and unsupported
plants, left/right steering, steering response, timestep convergence, tick/revision
boundaries, invalid pending changes, reset, handing back and restoration while moving.
The UI suite operates the switch and slider, checks live changes, 1/50/100, the displayed
assistance and unavailable tire/setup controls, and leaving kart mode. Final build and
regression evidence is recorded in `docs/STATUS.md` after it runs.
