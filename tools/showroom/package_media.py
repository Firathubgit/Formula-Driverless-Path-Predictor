"""Validate and encode the Blender showroom graph without publishing a partial pack.

Example (run with Python and Pillow, after all Blender frames have rendered)::

    python tools/showroom/package_media.py --overview

The default inputs and outputs are repository-relative and ignored by Git. All
four cars must have frames 0001.png through 0228.png. The final manifest is
replaced atomically only after input, seam, encoding and decode checks pass.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import uuid

from PIL import Image, ImageChops


ROOT = Path(__file__).resolve().parents[2]
FPS = 24
CLIPS = {"enter": (1, 72), "idle": (73, 120), "exit": (121, 156), "select": (157, 228)}
FRAME_COUNTS = {role: end - start + 1 for role, (start, end) in CLIPS.items()}
FRAMES_PER_CAR = max(end for _, end in CLIPS.values())
POSTER_FRAME = CLIPS["idle"][0]
CARS = (
    ("amr23", "Aston Martin", "AMR23 · 2023"),
    ("jesko", "Koenigsegg", "Jesko Attack · 2024"),
    ("urus", "Lamborghini", "Urus · 2023"),
    ("rb19", "Red Bull Racing", "RB19 · 2023"),
)
BLACK_FRAMES = (CLIPS["enter"][0], CLIPS["exit"][1], CLIPS["select"][1])
HERO_FRAMES = (CLIPS["enter"][1], CLIPS["idle"][0], CLIPS["idle"][1], CLIPS["exit"][0], CLIPS["select"][0])


def resolve_ffmpeg(explicit: str | None) -> Path:
    candidates = [explicit, os.environ.get("FFMPEG"), shutil.which("ffmpeg")]
    # Known full local distribution. CapCut can decode these sources, but its
    # bundled builds may omit the libx264 encoder required for normalized clips.
    candidates.append(str(Path.home() / "Downloads/VisoMaster-Fusion-main/VisoMaster-Fusion-main/"
                          "dependencies/ffmpeg-7.1.1-essentials_build/bin/ffmpeg.exe"))
    capcut = Path(os.environ.get("ProgramFiles", "C:/Program Files")) / "CapCut/Apps"
    if capcut.is_dir():
        candidates.extend(str(path) for path in sorted(capcut.glob("*/ffmpeg.exe"), reverse=True))
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            binary = Path(candidate).resolve()
            result = subprocess.run([str(binary), "-hide_banner", "-encoders"],
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, encoding="utf-8", errors="replace", check=False)
            if result.returncode == 0 and any("libx264" in line.split() for line in result.stdout.splitlines()):
                return binary
            if candidate == explicit:
                raise RuntimeError(f"The requested ffmpeg lacks libx264: {binary}")
    raise RuntimeError("Full ffmpeg with libx264 was not found. Supply --ffmpeg PATH or set FFMPEG.")


def frame_path(frames: Path, car: str, frame: int) -> Path:
    return frames / car / f"{frame:04d}.png"


def image_rmse(first: Image.Image, second: Image.Image) -> tuple[float, int]:
    difference = ImageChops.difference(first, second)
    histogram = difference.histogram()
    squared = sum((i % 256) ** 2 * count for i, count in enumerate(histogram))
    return math.sqrt(squared / (first.width * first.height * 3)), max(v[1] for v in difference.getextrema())


def validate_frames(frames: Path, seam_rmse: float, black_maximum: int) -> dict:
    """Read every expected PNG, then compare all shared boundary poses in RGB."""
    missing = [str(frame_path(frames, car, number))
               for car, _, _ in CARS for number in range(1, FRAMES_PER_CAR + 1)
               if not frame_path(frames, car, number).is_file()]
    if missing:
        shown = "\n".join(missing[:16])
        raise RuntimeError(f"{len(missing)} source frames missing; first paths:\n{shown}")
    size = None
    report = {"black_maximum_limit": black_maximum, "hero_rmse_limit_8bit": seam_rmse, "cars": []}
    failures = []
    for car, _, _ in CARS:
        boundaries = {}
        for number in range(1, FRAMES_PER_CAR + 1):
            path = frame_path(frames, car, number)
            with Image.open(path) as im:
                if im.format != "PNG":
                    raise RuntimeError(f"Expected PNG: {path}")
                im.load()  # Decode every frame, including those that are not seam samples.
                if size is None:
                    size = im.size
                if im.size != size:
                    raise RuntimeError(f"Mixed frame sizes: {path} is {im.size}, expected {size}")
                if im.mode not in ("RGB", "RGBA"):
                    raise RuntimeError(f"Expected 8-bit RGB/RGBA render, got {im.mode}: {path}")
                if im.mode == "RGBA" and im.getchannel("A").getextrema() != (255, 255):
                    raise RuntimeError(f"Render must have opaque black background: {path}")
                if number in BLACK_FRAMES or number in HERO_FRAMES:
                    boundaries[number] = im.convert("RGB")
        row = {"id": car, "frames_checked": FRAMES_PER_CAR, "black_endpoints": [], "hero_pairs": []}
        for number in BLACK_FRAMES:
            maximum = max(value[1] for value in boundaries[number].getextrema())
            row["black_endpoints"].append({"frame": number, "maximum_rgb_8bit": maximum})
            if maximum > black_maximum:
                failures.append(f"{car} frame {number} is not canonical black: maximum {maximum}")
        for first, second in itertools.combinations(HERO_FRAMES, 2):
            rmse, maximum = image_rmse(boundaries[first], boundaries[second])
            row["hero_pairs"].append({"frames": [first, second], "rmse_rgb_8bit": round(rmse, 6),
                                      "maximum_difference_8bit": maximum})
            if rmse > seam_rmse:
                failures.append(f"{car} frames {first}/{second} RMSE {rmse:.4f} exceeds {seam_rmse}")
        report["cars"].append(row)
        print(f"Validated {car}: {FRAMES_PER_CAR} PNGs; hero seam max RMSE "
              f"{max(pair['rmse_rgb_8bit'] for pair in row['hero_pairs']):.4f}/255", flush=True)
    report.update({"width": size[0], "height": size[1], "fps": FPS,
                   "source_frames": FRAMES_PER_CAR * len(CARS), "passed": not failures, "failures": failures})
    if size[0] % 2 or size[1] % 2:
        raise RuntimeError(f"H.264 yuv420p needs even width and height; renders are {size[0]}×{size[1]}")
    return report


def run(ffmpeg: Path, args: list[str], log: Path) -> str:
    command = [str(ffmpeg), "-hide_banner", "-nostdin", *args]
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, encoding="utf-8", errors="replace", check=False)
    log.write_text(result.stdout, encoding="utf-8")
    if result.returncode:
        raise RuntimeError(f"ffmpeg failed ({result.returncode}); see {log}\n{result.stdout[-2500:]}")
    return result.stdout


def verify_video(ffmpeg: Path, output: Path, expected_frames: int, log: Path) -> dict:
    result = run(ffmpeg, ["-v", "error", "-i", str(output), "-map", "0:v:0", "-an",
                          "-progress", "pipe:1", "-f", "null", "-"], log)
    counts = [int(line.split("=", 1)[1]) for line in result.splitlines() if line.startswith("frame=")]
    if not counts or counts[-1] != expected_frames:
        raise RuntimeError(f"{output}: decoded {counts[-1] if counts else 'unknown'} frames; "
                           f"expected {expected_frames}")
    return {"frames": expected_frames, "duration_seconds": expected_frames / FPS,
            "bytes": output.stat().st_size, "sha256": hashlib.sha256(output.read_bytes()).hexdigest()}


def concat(ffmpeg: Path, stage: Path, clips: list[str], destination: str,
           expected_frames: int, logs: Path) -> dict:
    target = stage / destination
    target.parent.mkdir(parents=True, exist_ok=True)
    listing = logs / (target.stem + "_concat.txt")
    # These paths are built only from constant car IDs/roles, never shell strings.
    # Resolve relative to the list in logs, avoiding absolute path escaping.
    listing.write_text("".join(f"file '../{clip}'\n" for clip in clips), encoding="utf-8")
    run(ffmpeg, ["-v", "warning", "-y", "-f", "concat", "-safe", "0", "-i", str(listing),
                 "-map", "0:v:0", "-c", "copy", "-an", "-movflags", "+faststart", str(target)],
        logs / (target.stem + "_encode.log"))
    return verify_video(ffmpeg, target, expected_frames, logs / (target.stem + "_decode.log"))


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def package(args: argparse.Namespace) -> Path:
    frames, output = args.frames.resolve(), args.out.resolve()
    validation = validate_frames(frames, args.seam_rmse, args.black_maximum)
    output.mkdir(parents=True, exist_ok=True)
    # A failed validation is reviewable and never creates/replaces manifest.json.
    write_json(output / "input_validation.json", validation)
    if not validation["passed"]:
        raise RuntimeError("Source seam validation failed:\n" + "\n".join(validation["failures"]))
    if args.verify_only:
        return output / "input_validation.json"
    ffmpeg = resolve_ffmpeg(args.ffmpeg)
    stage = output / (".package-" + uuid.uuid4().hex[:12])
    logs = stage / "logs"
    logs.mkdir(parents=True)
    manifest = {"schema_version": 1, "fps": FPS, "width": validation["width"],
                "height": validation["height"], "codec": "h264", "pixel_format": "yuv420p",
                "encoding": {"encoder": "libx264", "crf": args.crf, "preset": args.preset},
                "frame_counts": FRAME_COUNTS,
                "source_frames_per_car": FRAMES_PER_CAR, "cars": [], "transitions": [],
                "seam_validation": "seam_validation.json"}
    encoded = []
    for car, label, subtitle in CARS:
        (stage / car).mkdir()
        row = {"id": car, "label": label, "subtitle": subtitle, "poster": f"{car}/poster.png",
               "fps": FPS, "width": validation["width"], "height": validation["height"],
               "clip_metadata": {}}
        shutil.copyfile(frame_path(frames, car, POSTER_FRAME), stage / row["poster"])
        for role, (first, last) in CLIPS.items():
            relative = f"{car}/{role}.mp4"
            count = last - first + 1
            print(f"Encoding {relative}: {count} frames", flush=True)
            run(ffmpeg, ["-v", "warning", "-y", "-framerate", str(FPS), "-start_number", str(first),
                         "-i", str(frames / car / "%04d.png"), "-frames:v", str(count), "-an",
                         "-c:v", "libx264", "-preset", args.preset, "-crf", str(args.crf),
                         "-pix_fmt", "yuv420p", "-r", str(FPS), "-g", str(FPS),
                         "-force_key_frames", f"expr:eq(n,0)+eq(n,{count - 1})",
                         "-keyint_min", str(FPS), "-sc_threshold", "0", "-video_track_timescale", "24000",
                         "-movflags", "+faststart", str(stage / relative)], logs / f"{car}_{role}_encode.log")
            metadata = verify_video(ffmpeg, stage / relative, count, logs / f"{car}_{role}_decode.log")
            metadata["source_frame_range"] = [first, last]
            row[role], row["clip_metadata"][role] = relative, metadata
            encoded.append(relative)
        manifest["cars"].append(row)
    for (first, _, _), (second, _, _) in itertools.permutations(CARS, 2):
        clips = [f"{first}/exit.mp4", f"{second}/enter.mp4"]
        relative = f"transitions/{first}_to_{second}.mp4"
        print(f"Assembling {relative}", flush=True)
        metadata = concat(ffmpeg, stage, clips, relative, FRAME_COUNTS["exit"] + FRAME_COUNTS["enter"], logs)
        manifest["transitions"].append({"from": first, "to": second, "clips": clips,
                                         "video": relative, **metadata})
        encoded.append(relative)
    if args.overview:
        # Each selection exits to canonical black, matching the next car's entrance.
        clips = [f"{car}/{role}.mp4" for car, _, _ in CARS for role in ("enter", "idle", "select")]
        count = len(CARS) * sum(FRAME_COUNTS[role] for role in ("enter", "idle", "select"))
        manifest["overview"] = {"video": "overview.mp4", "clips": clips,
                                **concat(ffmpeg, stage, clips, "overview.mp4", count, logs)}
        encoded.append("overview.mp4")
    validation["encoded_videos_checked"] = len(encoded)
    validation["decoded_frame_counts_passed"] = True
    write_json(stage / "seam_validation.json", validation)
    write_json(stage / "manifest.json", manifest)
    # P-frame prediction at a final black frame can leave tiny nonzero pixels.
    # Check the actual decoded boundaries before replacing any published asset.
    from verify_encoded_seams import validate_encoded_seams
    decoded_seams = validate_encoded_seams(stage, ffmpeg)
    if not decoded_seams["passed"]:
        raise RuntimeError(f"Encoded seam validation failed; see {stage / 'encoded-seams.json'}")
    # The existing manifest stays in place throughout encoding. Promote checked
    # assets, reports and logs first; atomic replace of the manifest is the commit.
    for source in sorted(stage.rglob("*")):
        if not source.is_file() or source == stage / "manifest.json":
            continue
        target = output / source.relative_to(stage)
        target.parent.mkdir(parents=True, exist_ok=True)
        os.replace(source, target)
    os.replace(stage / "manifest.json", output / "manifest.json")
    for directory in sorted((path for path in stage.rglob("*") if path.is_dir()),
                            key=lambda path: len(path.parts), reverse=True):
        directory.rmdir()  # Only remove this invocation's now-empty staging folders.
    stage.rmdir()
    print(f"Published {len(encoded)} verified videos and 4 posters: {output / 'manifest.json'}", flush=True)
    return output / "manifest.json"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--frames", type=Path, default=ROOT / "artifacts/showroom/studio/frames")
    parser.add_argument("--out", type=Path, default=ROOT / "artifacts/showroom/media")
    parser.add_argument("--ffmpeg", help="Full ffmpeg binary with PNG, H.264 and libx264 support")
    parser.add_argument("--crf", type=int, default=16)
    parser.add_argument("--preset", choices=("ultrafast", "veryfast", "fast", "medium", "slow"), default="medium")
    parser.add_argument("--seam-rmse", type=float, default=2.0, help="Maximum RGB RMSE on the 0–255 scale")
    parser.add_argument("--black-maximum", type=int, default=0, help="Maximum RGB value in canonical black endpoints")
    parser.add_argument("--overview", action="store_true", help="Also concatenate all four entrance/idle/selection sequences")
    parser.add_argument("--verify-only", action="store_true", help="Validate PNG inputs and write their report without encoding")
    args = parser.parse_args()
    if not 0 <= args.crf <= 51 or args.seam_rmse < 0 or not 0 <= args.black_maximum <= 255:
        parser.error("CRF must be 0..51, seam RMSE nonnegative, and black maximum 0..255")
    try:
        result = package(args)
        print(result)
        return 0
    except (OSError, RuntimeError, ValueError) as error:
        print(f"Showroom packaging failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
