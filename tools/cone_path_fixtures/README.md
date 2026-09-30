# Cone path fixtures

Reference outputs for the C++ port of FaSTTUBe's cone path planner and of FITPACK's parametric curve fit
(`core/src/cone_path.cpp`, `core/src/fitpack.cpp`, decision 0031). The fixtures they write are kept in Git under
`tests/fixtures/cone_path` and `tests/fixtures/fitpack`; the tools and packages that wrote them are not.

## Environment

An isolated Python environment under `.tools/fsd-python` (not in Git), created from Python 3.13.7:

| Package | Version |
| --- | --- |
| numpy | 2.2.6 |
| scipy | 1.15.3 |
| numba | 0.61.2 |
| llvmlite | 0.44.0 |
| scikit-learn | 1.6.1 |
| icecream | 2.1.4 |
| typing_extensions | 4.14.0 |

FaSTTUBe's `ft-fsd-path-planning` 0.4.3.2 (MIT) is not installed: `make_fixtures.py` adds its unpacked directory to the
path and runs it exactly as published. Numba's cache path must be short on Windows, or its cache files pass 260
characters; set `NUMBA_CACHE_DIR` to a short directory.

## Regenerating

The cone sets come from a recorded lap of the dynamic car on the preset laid to 5 m with simulated perception:

```
fd_headless --plant dynamic --track-width 5 --perception 4 --out out/runs/p73-narrow-perceived-lap
```

Then, with `FT` the unpacked FaSTTUBe directory:

```
NUMBA_CACHE_DIR=C:/Users/Firat/AppData/Local/Temp/nbc PYTHONDONTWRITEBYTECODE=1 \
  .tools/fsd-python/Scripts/python.exe tools/cone_path_fixtures/make_fixtures.py \
  out/runs/p73-narrow-perceived-lap "$FT" tests/fixtures/cone_path
.tools/fsd-python/Scripts/python.exe tools/cone_path_fixtures/make_fitpack_fixtures.py tests/fixtures/fitpack/cases.csv
```

`make_fixtures.py` takes twelve frames spread along the lap and, at each, four cone sets in the car's frame at the
moment the frame was sampled: the course's cones within 30 m, the same without colour, the frame's own simulated
detections, and the right-hand cones alone. FaSTTUBe runs on each; its sorted cones, cones with virtual cones, matches,
the points its path is fitted through and the path are written beside the input, with the course's true centreline
around the car for evaluation only: no planner is given it. Where FaSTTUBe's own code raises, the cone set is kept as a
`.failed` file with the error. The run above wrote 44 fixtures and 4 failures.
