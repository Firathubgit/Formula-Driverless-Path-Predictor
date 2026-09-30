#!/usr/bin/env python3
"""The tire values of core/vehicle_profiles.cpp (decision 0033), fitted from each source's Magic Formula.

This project's tire (core/tire.hpp) is described by its peak friction and the slip at which it peaks, at two loads, and
the share of peak grip it keeps at large slip. Each source's Magic Formula, F = D sin(C atan(B x - E (B x - atan(B x)))),
is reduced to those numbers here: the peak found numerically on a fine grid, the share kept at 0.5 rad (or slip ratio
0.5), where every curve here has settled. Standard library only; run it and compare with the profile values.

    python fit_tires.py
"""

import math


def magic(x, b, c, e):
    bx = b*x
    return math.sin(c*math.atan(bx-e*(bx-math.atan(bx))))


def peak(b, c, e, limit=1.0):
    """Slip at the curve's highest point on (0, limit], its value, and its value at 0.5."""
    best_x, best = 0.0, -1.0
    steps = 200000
    for k in range(1, steps+1):
        x = limit*k/steps
        v = magic(x, b, c, e)
        if v > best:
            best, best_x = v, x
    return best_x, best, magic(0.5, b, c, e)


def report(name, b, c, e):
    x, value, at_half = peak(b, c, e)
    print(f"  {name}: B {b:.4f} C {c} E {e}: peak at {x:.4f} ({math.degrees(x):.2f} deg), {value:.4f} of D; "
          f"kept at 0.5: {at_half/value:.4f} of peak")
    return x, value, at_half/value


print("TUM racecar.ini, tire_params_mintime (lateral; B 10, C 2.5, E 1, eps -0.1 about 3000 N, mue 1):")
report("both axles", 10.0, 2.5, 1.0)
# TUM's solver (opt_mintime.py) scales the lateral peak by 1 + eps Fz / f_z0, the load itself over f_z0 and not its
# increment over f_z0; its longitudinal force is bounded by the friction circle of mue alone. The first profile read the
# increment and gave 0.1 too much lateral friction; the minimum-time cross-check found it (decision 0034).
for load in (2000.0, 6000.0):
    print(f"  lateral friction at {load:.0f} N: {1.0*(1-0.1*load/3000.0):.4f}; longitudinal 1.0000")

print("MPCC model.json (Formula Student, lateral; B 10, C 1.38, E 0; D 1500 N on each axle of a 190 kg car):")
report("both axles", 10.0, 1.38, 0.0)
print(f"  friction D over the static axle load: {1500.0/(190.0*9.81/2):.4f}")

print("Lot 2016 kart (fastest-lap roberto-lot-kart-2016.xml, rear tire, MF 5.2 pure slip, Fz0' = 560 N x 1.6):")
fz0 = 560.0*1.6
for load in (300.0, 700.0):
    ky = 37.6*fz0*math.sin(2.0*math.atan(load/(1.6*fz0)))            # pKy1, pKy2, pKy4
    d = 1.5*load                                                       # pDy1
    b = ky/(2.3*d)                                                     # pCy1
    report(f"lateral at {load:.0f} N", b, 2.3, 0.9)                   # pEy1
    dfz = (load-fz0)/fz0
    kx = load*(20.0+1.0*dfz)*math.exp(-0.5*dfz)                        # pKx1, pKx2, pKx3
    bx = kx/(2.3*0.9*load)                                             # pCx1, pDx1
    report(f"longitudinal at {load:.0f} N", bx, 2.3, 0.95)            # pEx1
