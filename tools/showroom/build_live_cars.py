"""Build the live driving scene's cars from the prepared showroom cars: export each with Blender, convert every part with
Qt's balsam into meshes and a QML component the desktop loads, render the studio lighting, and write the index.

    python tools/showroom/build_live_cars.py [--cars amr23 rb19 ...] [--skip-export]

Output (ignored by Git, like every generated asset): artifacts/live-cars/{car}/ with body.glb, wheel_*.glb, shadow.png,
manifest.json and qt/{part}/ from balsam; artifacts/live-cars/studio.hdr; and artifacts/live-cars/index.json, which the
desktop reads. Balsam's meshes are shared by path, so the car and its reflection load them once.
"""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CARS = ("amr23", "jesko", "urus", "rb19")  # the showroom's appearance order: 0 AMR23, 1 Jesko, 2 Urus, 3 RB19
PARTS = ("body", "wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr")


def run(command, log, env=None):
    with open(log, "w", encoding="utf-8") as handle:
        subprocess.run([str(c) for c in command], stdout=handle, stderr=subprocess.STDOUT, check=True, env=env, cwd=ROOT)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cars", nargs="+", default=list(CARS), choices=CARS)
    parser.add_argument("--out", type=Path, default=ROOT / "artifacts/live-cars")
    parser.add_argument("--blender", default="C:/Program Files/Blender Foundation/Blender 4.5/blender.exe")
    parser.add_argument("--qt", type=Path, default=ROOT / ".tools/Qt/6.8.3/msvc2022_64")
    parser.add_argument("--skip-export", action="store_true", help="convert the GLB files already exported")
    options = parser.parse_args()
    out = options.out.resolve()
    logs = out / "logs"
    logs.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, PATH=str(options.qt / "bin") + os.pathsep + os.environ.get("PATH", ""))
    balsam = options.qt / "bin" / "balsam.exe"
    if not (out / "studio.hdr").exists():
        run([options.blender, "-b", "--factory-startup", "--python-exit-code", "1", "--python",
             ROOT / "tools/showroom/make_live_environment.py", "--", "--out", out / "studio.hdr"], logs / "environment.log")
    for car in options.cars:
        started = time.time()
        folder = out / car
        if not options.skip_export:
            run([options.blender, "-b", ROOT / f"artifacts/showroom/cars/{car}.blend", "--python-exit-code", "1",
                 "--python", ROOT / "tools/showroom/export_live_car.py", "--", "--car", car, "--out", out],
                logs / f"export-{car}.log")
        qt = folder / "qt"
        if qt.exists():
            shutil.rmtree(qt)
        for part in PARTS:
            run([balsam, "-o", qt / part, folder / f"{part}.glb"], logs / f"balsam-{car}-{part}.log", env)
            components = list((qt / part).glob("*.qml"))
            if len(components) != 1:
                raise RuntimeError(f"balsam made {len(components)} components for {car} {part}")
            components[0].rename(qt / part / "Part.qml")
        print(f"{car}: {time.time() - started:.1f} s", flush=True)
    index = {"schema_version": 1, "environment": "studio.hdr", "cars": {}}
    for car in CARS:
        manifest_path = out / car / "manifest.json"
        if not manifest_path.exists() or not all((out / car / "qt" / p / "Part.qml").exists() for p in PARTS):
            continue
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        index["cars"][car] = {
            "body": f"{car}/qt/body/Part.qml",
            "wheels": [dict(w, component=f"{car}/qt/wheel_{w['name']}/Part.qml") for w in manifest["wheels"]],
            "shadow": dict(manifest["shadow"], file=f"{car}/{manifest['shadow']['file']}"),
            "wheelbase_m": manifest["wheelbase_m"], "source_sha256": manifest["source_sha256"],
        }
    (out / "index.json").write_text(json.dumps(index, indent=2), encoding="utf-8")
    print("INDEX", sorted(index["cars"]), flush=True)


if __name__ == "__main__":
    sys.exit(main())
