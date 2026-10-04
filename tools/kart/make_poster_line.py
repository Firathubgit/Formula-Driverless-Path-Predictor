"""Writes tracks/gokartcentralen-goteborg-poster-line.csv: the poster's coloured line (decision 0039) smoothed into a closed
centreline file, x_m,y_m every half metre, for driving the plant round the line the poster recommends rather than round
the track's middle (decision 0046). Needs numpy and scipy.

    python make_poster_line.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kart_model as km

line = km.PosterLine(sigma_m=0.25, step_m=0.5)
with open(km.POSTER_LINE, "w", newline="\n") as f:
    f.write("x_m,y_m\n")
    for x, y in zip(line.x, line.y):
        f.write("%.5f,%.5f\n" % (x, y))
print("%d points, %.1f m, tightest radius %.2f m -> %s" % (len(line.s), line.length_m, 1 / np.abs(line.kappa).max(), km.POSTER_LINE))
