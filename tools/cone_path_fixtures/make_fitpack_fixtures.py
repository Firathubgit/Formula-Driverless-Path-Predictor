#!/usr/bin/env python3
"""Reference fits of scipy.interpolate.splprep and evaluations of splev, for the C++ port of FITPACK's parametric
curve fit (core/src/fitpack.*, decision 0031).

Each case is a plane curve of a few to forty points with chord-length parameters, fitted with the degree and smoothing
FaSTTUBe's planner uses, and evaluated inside and beyond its parameter range. Written one line per value:

    case,NAME,DEGREE,SMOOTHING
    point,X,Y          (with its parameter, the cumulative chord length)
    knot,T
    eval,U,X,Y

    python make_fitpack_fixtures.py OUTPUT_FILE
"""

import sys

import numpy as np
from scipy.interpolate import splev, splprep


def main() -> int:
    rng = np.random.default_rng(20260923)
    lines = []
    cases = 0
    for size in (2, 3, 4, 5, 7, 10, 14, 20, 30, 40):
        for smoothing in (0.2, 0.01, 2.0):
            # A bending road of cones' midpoints: an arc with noise, and some straight.
            radius = rng.uniform(8, 60)
            angles = np.cumsum(rng.uniform(1.5, 5.0, size)) / radius
            points = np.column_stack((radius * np.sin(angles), radius * (1 - np.cos(angles))))
            points += rng.normal(0, rng.uniform(0.0, 0.3), points.shape)
            u = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))))
            k = int(np.clip(size - 1, 1, 3))
            (t, c, k_out), _ = splprep(points.T, s=smoothing, k=k, u=u)
            lines.append(f"case,n{size}-s{smoothing},{k},{smoothing!r}")
            for x, y in points:
                lines.append(f"point,{float(x)!r},{float(y)!r}")
            for knot in t:
                lines.append(f"knot,{float(knot)!r}")
            evaluate_at = np.concatenate((np.linspace(-2, u[-1] + 5, 23), u))
            xs, ys = splev(evaluate_at, (t, c, k_out))
            for at, x, y in zip(evaluate_at, xs, ys):
                lines.append(f"eval,{float(at)!r},{float(x)!r},{float(y)!r}")
            cases += 1
    with open(sys.argv[1], "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"wrote {cases} cases")
    return 0


if __name__ == "__main__":
    sys.exit(main())
