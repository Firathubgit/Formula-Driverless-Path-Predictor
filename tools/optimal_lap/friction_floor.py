"""Show fastest-lap 0.5's floor on peak friction with its own G-G diagram (decision 0034).

    python friction_floor.py --fastest-lap RELEASE_DIR --problem PROBLEM_DIR

The problem's car is put into fastest-lap as solve_fastest_lap.py puts it, then every tire's peak friction is set to one
value at both reference loads and both axes, and fastest-lap's G-G diagram gives the car's largest lateral acceleration at
30 m/s for each value. Grip below the floor gives the same limit as the floor itself; above it the limit grows with it.
fastest-lap is not part of this repository; see README.md beside this script.
"""

import argparse
import json
import pathlib
import re
import sys
import tempfile

import solve_fastest_lap as solver

TAGS = ("mu-x-max-1", "mu-x-max-2", "mu-y-max-1", "mu-y-max-2")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--fastest-lap", required=True, type=pathlib.Path, help="the unpacked release: bin and include")
    parser.add_argument("--problem", required=True, type=pathlib.Path, help="a directory written by fd_headless --write-lap-problem")
    parser.add_argument("--speed", type=float, default=30.0, help="the speed of the G-G diagram (default 30 m/s)")
    args = parser.parse_args()
    if solver.sha256(args.fastest_lap / "bin" / solver.PINNED["library"]) != solver.PINNED["library_sha256"]:
        raise SystemExit("Not the pinned fastest-lap %s library" % solver.PINNED["version"])
    sys.path.insert(0, str(args.fastest_lap / "include"))
    import fastest_lap as fl  # noqa: E402  (the release's own wrapper)

    problem = json.loads((args.problem / "problem.json").read_text(encoding="utf-8"))
    base, _ = solver.vehicle_xml(problem, accept_floor=True)
    work = pathlib.Path(tempfile.mkdtemp(prefix="fd-friction-floor-"))
    print("peak friction, largest lateral acceleration at %.0f m/s (fastest-lap's G-G diagram)" % args.speed)
    for k, value in enumerate((0.5, 0.8, 1.0, 1.2, 1.5)):
        text = base
        for tag in TAGS:
            text = re.sub("<%s>[^<]*</%s>" % (tag, tag), "<%s>%r</%s>" % (tag, value, tag), text)
        path = work / ("friction-%d.xml" % k)
        path.write_text(text, encoding="utf-8")
        fl.create_vehicle_from_xml("car%d" % k, str(path))
        lateral, _, _, _ = fl.gg_diagram("car%d" % k, args.speed, 20)
        print("  %.1f: %.3f g" % (value, max(lateral)))


if __name__ == "__main__":
    main()
