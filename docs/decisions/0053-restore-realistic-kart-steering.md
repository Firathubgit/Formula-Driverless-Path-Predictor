# 0053 — Restore tire physics and bound steering by speed

2026-10-04. The user rejected the preceding change: full steering at a reported
80 km/h behaved unrealistically. They wanted a small adjustment and steering that
depends on speed, not an artificial grip override. Decision 0052 overinterpreted
their request and changed the plant's lateral response toward rolling motion.

Remove that override entirely: `advance_direct_kart`, its lateral-only derivative
blend and the environment's grip flag. Walls remain physical, but can no longer
select a different handling model. Away from contact, human Realistic kart driving
uses the ordinary four-wheel tire model through `advance_assisted` with assistance
disabled. Settings and diagnostics no longer describe or mask artificial grip.

The accumulated 1.848 steering multiplier also bypassed the grip aid's speed-dependent
bound by clamping only at physical lock. Replace the desktop default with a modest
1.1 sensitivity factor and clamp the result at the speed-dependent limit itself.
Partial-stick response increases by at most 10%; full-stick wheel travel still falls
with speed. The existing envelope-based 0.9 steering/pedal aid remains a limit on
commands. It never grants the tires more force. Tire parameters, motor torque/power,
speed limit, steering lock and actuator rate are unchanged. Forgiveness remains
explicitly selectable only on the earlier kart.

Tests check full-stick angles from rest through 80 km/h, both steering directions,
the sensitivity bound and actual actuator response. An 80 km/h initial state with
physical full lock must exceed the tires' envelope and turn substantially less than
a no-slip model; a 20-second full-throttle straight must respect the motor limit.
Barrier tests compare every motion field, wheel speed and wheel load with the
ordinary plant away from walls, including at 80 km/h, and retain impact tests.

These are model regression checks, not validation against instrumented kart data
or a physical controller driving session.
