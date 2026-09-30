#!/usr/bin/env python3
"""Reference outputs of FaSTTUBe's path planner (MIT) on fixed cone sets, for the C++ port of Phase 7.3 (decision 0031).

The cone sets come from a recorded run of this project with simulated cone perception: at chosen frames, the course's
true cones around the car, the same without colour, the frame's own simulated detections, and the true cones of one
side only, each in the car's frame at the moment the frame was sampled. FaSTTUBe runs on each exactly as published, and
its intermediate and final results are written beside the input, one line per value, for tests/cone_path_tests.cpp.

Run with the isolated environment under .tools (see tools/cone_path_fixtures/README.md):

    python make_fixtures.py RUN_DIRECTORY FASTTUBE_DIRECTORY OUTPUT_DIRECTORY
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import numpy as np

# FaSTTUBe's own cone types: unknown 0, yellow (right) 1, blue (left) 2, small orange 3, big orange 4.
COLOUR_TO_TYPE = {"unknown": 0, "yellow": 1, "blue": 2, "orange": 3, "big orange": 4}
# The frames of the recorded lap the fixtures are taken at, spread along it.
FRAMES = [5, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 325]
# True cones within this distance of the car are what a fixture built from the course sees.
AROUND_M = 30.0
# The true centreline written for evaluation: this many of the course's samples, 0.6 m apart, behind and ahead of the car.
CENTRE_BEHIND, CENTRE_AHEAD = 10, 60


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def fmt(v: float) -> str:
    return repr(float(v))


def run_fasttube(cones_by_type: list[np.ndarray]):
    from fsd_path_planning import MissionTypes, PathPlanner
    from fsd_path_planning.calculate_path.core_calculate_path import PathCalculationInput
    from fsd_path_planning.cone_matching.core_cone_matching import ConeMatchingInput
    from fsd_path_planning.sorting_cones.core_cone_sorting import ConeSortingInput
    from fsd_path_planning.utils.cone_types import ConeTypes

    planner = PathPlanner(MissionTypes.trackdrive)
    position = np.array([0.0, 0.0])
    direction = np.array([1.0, 0.0])
    # The published pipeline's own steps, so the midpoints the path is fitted through can be kept too.
    planner.cone_sorting.set_new_input(ConeSortingInput(cones_by_type, position, direction))
    sorted_left, sorted_right = planner.cone_sorting.run_cone_sorting()
    matched_input = [np.zeros((0, 2)) for _ in ConeTypes]
    matched_input[ConeTypes.LEFT] = sorted_left
    matched_input[ConeTypes.RIGHT] = sorted_right
    planner.cone_matching.set_new_input(ConeMatchingInput(matched_input, position, direction))
    left_virtual, right_virtual, left_to_right, right_to_left = planner.cone_matching.run_cone_matching()
    planner.pathing.set_new_input(
        PathCalculationInput(left_virtual, right_virtual, left_to_right, right_to_left, position, direction, None))
    path, basis = planner.pathing.run_path_calculation()
    return sorted_left, sorted_right, left_virtual, right_virtual, left_to_right, right_to_left, basis, path


def write_fixture(path: Path, description: str, cones: list[tuple[int, float, float]],
                  centre: list[tuple[float, float]]) -> None:
    by_type = [np.array([(x, y) for t, x, y in cones if t == k], dtype=float).reshape(-1, 2) for k in range(5)]
    (sorted_left, sorted_right, left_virtual, right_virtual, left_to_right, right_to_left, basis,
     final_path) = run_fasttube(by_type)
    lines = [f"# {description}", "pose,0.0,0.0,1.0,0.0"]
    for k in range(5):
        for x, y in by_type[k]:
            lines.append(f"cone,{k},{fmt(x)},{fmt(y)}")
    for name, rows in (("left", sorted_left), ("right", sorted_right), ("left_virtual", left_virtual),
                       ("right_virtual", right_virtual), ("basis", basis)):
        for x, y in np.asarray(rows).reshape(-1, 2):
            lines.append(f"{name},{fmt(x)},{fmt(y)}")
    for name, rows in (("left_to_right", left_to_right), ("right_to_left", right_to_left)):
        for j in np.asarray(rows).reshape(-1):
            lines.append(f"{name},{int(j)}")
    for theta, x, y, curvature in np.asarray(final_path).reshape(-1, 4):
        lines.append(f"path,{fmt(theta)},{fmt(x)},{fmt(y)},{fmt(curvature)}")
    # The course's true centreline around the car, for evaluation only: no planner is given it.
    for x, y in centre:
        lines.append(f"centre,{fmt(x)},{fmt(y)}")
    path.write_text("\n".join(lines) + "\n")


def main() -> int:
    run, fasttube, out = (Path(a) for a in sys.argv[1:4])
    sys.path.insert(0, str(fasttube))
    out.mkdir(parents=True, exist_ok=True)
    cones = read_csv(run / "cones.csv")
    track = read_csv(run / "track.csv")
    telemetry = read_csv(run / "telemetry.csv")
    frames = read_csv(run / "perception_frames.csv")
    detections = read_csv(run / "detections.csv")
    by_time = {round(float(t["time_s"]), 6): t for t in telemetry}
    by_frame: dict[int, list[dict[str, str]]] = {}
    for d in detections:
        by_frame.setdefault(int(d["frame"]), []).append(d)
    written = 0
    for f in FRAMES:
        frame = frames[f]
        pose = by_time[round(float(frame["sampled_at_s"]), 6)]
        px, py, yaw = float(pose["x_m"]), float(pose["y_m"]), float(pose["yaw_rad"])
        c, s = math.cos(yaw), math.sin(yaw)

        def to_car(x: float, y: float) -> tuple[float, float]:
            dx, dy = x - px, y - py
            return c * dx + s * dy, -s * dx + c * dy

        around = []
        for cone in cones:
            x, y = to_car(float(cone["x_m"]), float(cone["y_m"]))
            if math.hypot(x, y) <= AROUND_M:
                around.append((COLOUR_TO_TYPE[cone["colour"]], x, y))
        seen = [(COLOUR_TO_TYPE[d["colour"]], float(d["x_m"]), float(d["y_m"])) for d in by_frame.get(f, [])]
        # The car's own stretch of the course, from a few metres behind it to beyond what it can see, so that a path
        # which jumps to a neighbouring leg of the course is measured as far from it.
        nearest = int(pose["nearest_index"])
        centre = []
        for k in range(nearest - CENTRE_BEHIND, nearest + CENTRE_AHEAD + 1):
            sample = track[k % len(track)]
            centre.append(to_car(float(sample["x_m"]), float(sample["y_m"])))
        at = f"frame {f} of {run.name}, sampled at {float(frame['sampled_at_s']):.3f} s"
        scenarios = [
            ("course", f"the course's cones within {AROUND_M:.0f} m, {at}", around),
            ("course-uncoloured", f"the same cones with no colour, {at}", [(0, x, y) for _, x, y in around]),
            ("detected", f"the frame's own simulated detections, {at}", seen),
            ("right-only", f"the course's right-hand cones alone, {at}", [(t, x, y) for t, x, y in around if t == 1]),
        ]
        for name, description, cone_set in scenarios:
            try:
                write_fixture(out / f"frame{f:03d}-{name}.csv", description, cone_set, centre)
                written += 1
            except Exception as error:  # a published crash is itself a finding, recorded rather than hidden
                lines = [f"# {description}", f"# FaSTTUBe raised {type(error).__name__}: {error}", "pose,0.0,0.0,1.0,0.0"]
                lines += [f"cone,{t},{fmt(x)},{fmt(y)}" for k in range(5) for t, x, y in cone_set if t == k]
                (out / f"frame{f:03d}-{name}.failed").write_text("\n".join(lines) + "\n")
    print(f"wrote {written} fixtures to {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
