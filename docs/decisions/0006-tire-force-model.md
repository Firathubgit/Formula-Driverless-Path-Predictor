# 0006: Tire forces, with the peak where it is named

Date: 2026-09-14. Status: accepted for Phase 1.1 of TrackWayFastPlan. The module is tested on
its own and is not yet connected to any plant.

Every later realism step, sliding, locking, brake bias, load transfer and a vehicle-derived
envelope, needs one function: tire force from slip and vertical load. It has to be honest
about the two things a braking demonstration depends on, where peak grip is and how much
grip a sliding tire keeps.

## Decision

`core/tire.*` provides `tire_force(parameters, slip_ratio, slip_angle, vertical_load)`,
`validate_tire` and `tire_shape_for_sliding_fraction`. Its structure follows fastest-lap's
`Pacejka_simple_model` (MIT, `src/core/tire/tire_pacejka.hpp` in that project), reimplemented
from the equations rather than copied:

- Peak friction and peak slip, longitudinal and lateral, vary linearly with vertical load
  between two reference loads. A heavier tire gives more force at a lower friction
  coefficient.
- Both directions share one normalised slip `rho = hypot(kappa/kappa_peak, alpha/alpha_peak)`
  and each force is its direction's curve times its share of `rho`. That is the similarity
  approximation of combined slip, not measured combined-slip data, and it keeps the resultant
  inside one friction ellipse.
- The curve is `sin(Q*atan(S*rho))`.

Two things differ from fastest-lap, deliberately:

1. **The peak is placed at the named slip.** fastest-lap uses `S = pi/(2*atan(Q))`, which
   moves the peak to `rho = 0.751` for its F1 value `Q = 1.9`, although its parameter is
   documented as the slip at the friction peak. Here `S = tan(pi/(2Q))`, so the curve reaches
   exactly one at `rho = 1`.
2. **The shape is set by the sliding fraction, not by Q.** The curve tends to `sin(Q*pi/2)` at
   large slip, so the parameter is that fraction and `Q = 2 - 2*asin(fraction)/pi`. The
   default is 0.75. fastest-lap's `Q = 1.9` corresponds to 0.156, which leaves a locked wheel
   with about a third of peak grip; dry asphalt keeps roughly 0.7 to 0.85. Ported unchanged,
   every lock would look about three times worse than it should, and the project's "braking
   without locking" story would be exaggerated by the model. The harsh shape stays
   representable and a test documents the difference.

Beyond the reference loads, friction continues its slope down to a floor, as in fastest-lap
(`minimum_friction`, 1.0 by default, fastest-lap's value). Peak slip is held at its reference
value instead of extrapolated, so it can never reach zero or change sign at very high load;
fastest-lap extrapolates it. A load at or below zero is a lifted wheel and transmits nothing.

Default parameters are the Limebeer 2014 F1 loads, frictions and peak slips as they appear in
fastest-lap's `database/vehicles/f1/limebeer-2014-f1.xml`, with the sliding fractions chosen
here. They are illustrative F1-scale values, not a measured tire, and not yet matched to this
project's 20 m/s demonstrator car.

## Verification

`tests/tire_tests.cpp` (`fd_tire_model`) checks, through the public interface only: zero
force at zero slip; odd symmetry and force direction; numerical peaks within 1% of the named
slip ratio and slip angle at four loads with peak force equal to friction times load; force
falling monotonically past the peak, a locked wheel keeping more than the sliding fraction and
very large slip tending to it; the fastest-lap shape keeping less than half the locked-wheel
grip; the small-slip stiffness; peak force rising and friction falling with load; friction extrapolation, its floor and
held peak slip; braking at peak slip removing lateral grip and 20,000 random combined slips
staying inside the friction ellipse; and rejection of non-finite inputs and every invalid
parameter. Substituting fastest-lap's shape factor made five of the then nine groups fail,
so the tests detect the difference this decision is about.

## What this is not

A steady-state force law. There is no relaxation length, camber, temperature, pressure, rolling
resistance or aligning moment, and no fitted tire data. Nothing in the simulation uses it yet,
so no claim about sliding, locking or load transfer follows from this change alone.

## Consequences

The dynamic plant (Phase 1.2, decision 0005) takes slip from its body and wheel velocities and
asks this module for force.

Wheel-speed stiffness comes from this curve's slope at small slip, `mu*Fz*Q*S/kappa_peak`. For
the default sliding fraction `Q*S = 2.705`, so the slope is 2.7 times the `mu*Fz/kappa_peak`
that TrackWayFastPlan Phase 2.1 used in its estimate. With that plan's other numbers (4000 N,
`I = 1 kg*m^2`, `R = 0.33 m`) the slip stiffness is 162 kN, the wheel time constant is
`v/17 500 s`, about 1.1 ms at 20 m/s, and an explicit 1 ms step is unstable below about
8.8 m/s. The plan's conclusion stands and is stronger than it states: wheel speed must be
integrated implicitly. A test pins the slope. No code was copied, so no third-party notice file is added; fastest-lap is credited
here and in the header.
