"""Renders one region of a poster (or of another image in its frame, such as trace_poster.py's overlay) enlarged, with a
labelled 25 px grid, to read control points off it: python zoom_poster.py IMAGE OUT.png X0 X1 Y0 Y1 [SCALE]."""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

source, target = Path(sys.argv[1]), Path(sys.argv[2])
x0, x1, y0, y1 = map(int, sys.argv[3:7])
scale = float(sys.argv[7]) if len(sys.argv) > 7 else 2.0
poster = np.asarray(Image.open(source).convert("RGB"))
fig = plt.figure(figsize=((x1-x0)*scale/100, (y1-y0)*scale/100), dpi=100)
ax = fig.add_axes([0.05, 0.03, 0.94, 0.96])
ax.imshow(poster[y0:y1, x0:x1], extent=(x0, x1, y1, y0), interpolation="nearest")
for x in range((x0//25)*25, x1+1, 25):
    ax.axvline(x, color="#ff00ff" if x % 100 == 0 else "#00e5ff", lw=0.8 if x % 100 == 0 else 0.4, alpha=0.8)
for y in range((y0//25)*25, y1+1, 25):
    ax.axhline(y, color="#ff00ff" if y % 100 == 0 else "#00e5ff", lw=0.8 if y % 100 == 0 else 0.4, alpha=0.8)
ax.set_xticks(range((x0//50)*50, x1+1, 50))
ax.set_yticks(range((y0//50)*50, y1+1, 50))
ax.tick_params(labelsize=8)
ax.set_xlim(x0, x1)
ax.set_ylim(y1, y0)
fig.savefig(target)
print(target)
