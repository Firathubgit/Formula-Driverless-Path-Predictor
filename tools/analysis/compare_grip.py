"""Compare two recorded first laps; standard-library only, no simulation reimplementation."""
import argparse
import csv
import json
import hashlib
from pathlib import Path


def read_run(path: Path):
    metadata = json.loads((path / "metadata.json").read_text())
    summary = json.loads((path / "summary.json").read_text())
    with (path / "plan-rev-1.csv").open(newline="") as file:
        plan = list(csv.DictReader(file))
    corner = next(row for row in plan if abs(float(row["curvature_1pm"])) > 1e-9)
    braking = next(row for row in plan if float(row["acceleration_mps2"]) < -metadata["plan_color_deadband_mps2"])
    if float(braking["s_m"]) >= float(corner["s_m"]):
        raise ValueError("Comparison requires braking on the straight before the first corner")
    if not summary["completed"] or summary["invalid_samples"]:
        raise ValueError("Comparison requires a completed nominal run without invalid samples")
    return metadata, {
        "grip_mu": metadata["initial_config"]["grip_mu"],
        "first_lap_s": summary["lap_crossing_times_s"][0],
        "max_cross_track_error_m": summary["max_cross_track_error_m"],
        "max_combined_grip_utilization": summary["max_combined_grip_utilization"],
        "planned_braking_station_m": float(braking["s_m"]),
        "first_corner_station_m": float(corner["s_m"]),
        "first_corner_speed_mps": float(corner["speed_mps"]),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("high", type=Path)
    parser.add_argument("low", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    high_meta, high = read_run(args.high)
    low_meta, low = read_run(args.low)
    high_config = {key: value for key, value in high_meta["initial_config"].items() if key != "grip_mu"}
    low_config = {key: value for key, value in low_meta["initial_config"].items() if key != "grip_mu"}
    if high_config != low_config or high_meta["initial_state"] != low_meta["initial_state"]:
        raise ValueError("Comparison requires identical initial conditions except grip")
    for field in ("source_fingerprint", "compiler_id", "compiler_version"):
        if high_meta[field] != low_meta[field]:
            raise ValueError(f"Mismatched build identity: {field}")
    digest = lambda path: hashlib.sha256((path / "track.csv").read_bytes()).hexdigest()
    if digest(args.high) != digest(args.low):
        raise ValueError("Comparison requires identical track samples")
    if high_meta["requested_grip_events"] or low_meta["requested_grip_events"]:
        raise ValueError("Compare constant-grip runs; live events need separate analysis")
    if low["grip_mu"] >= high["grip_mu"]:
        raise ValueError("Expected lower grip in the second run")
    result = {
        "scope": f"This preset, {high_config['max_speed_mps']} m/s cap, same standing start; not a universal monotonicity claim",
        "source_fingerprint": high_meta["source_fingerprint"],
        "high": high,
        "low": low,
        "lower_grip_brakes_earlier_by_m": high["planned_braking_station_m"] - low["planned_braking_station_m"],
    }
    output = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output)
    print(output, end="")


if __name__ == "__main__":
    main()
