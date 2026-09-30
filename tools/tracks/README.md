# Tracing a track poster

`trace_poster.py` turns a published track map into the simulator's centreline file (decision 0038). It was made for
Gokartcentralen Göteborg's poster, which is not in the repository: the user supplies it.

```bash
.tools/tum-python/Scripts/python.exe tools/tracks/trace_poster.py POSTER.webp tools/tracks/gokartcentralen-goteborg.points.json OUT_DIR --track-out tracks/gokartcentralen-goteborg.csv
```

The control points are poster pixels in lap order from the start line. Each is snapped perpendicular to the lap onto the
middle of the drawn asphalt, between the poster's white lines, unless it is marked `"fixed"`; `asphalt_boxes` mark light
shading that is asphalt (the velodrome at T9) and `pit_walls` close the pit lane for the overlap figures. The spline through
the points is resampled every metre and scaled to the published 400 m.

`OUT_DIR` receives what the verification judged: `overlay.png` (the trace and a 6 m corridor on the poster),
`side-by-side.png`, `differences.png` (blue: drawn asphalt outside the corridor; red: corridor off the drawing),
`summary.json` (scale, direction, the drawn width about the line, overlap) and the centreline in pixels and metres.
`zoom_poster.py IMAGE OUT.png X0 X1 Y0 Y1 [SCALE]` enlarges any part of the poster or an overlay with a labelled 25 px grid,
for reading control points off it.

The committed points are version 2 of the verification in `docs/evidence/gokart-track-verification.json`: three independent
judges scored it 9, 9 and 9 against the poster. Run the tracer with the same points and poster and it writes the committed
track file byte for byte.
