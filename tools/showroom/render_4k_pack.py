"""Render the approved studios, verify 4K playback, then activate the new pack.

Run with Python/Pillow. Progress and subprocess logs remain in the revision.
Rerunning resumes completed frames. The old default pack is backed up before
activation; rendering and validation never change the active media.
"""
from datetime import datetime
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
CARS = ('amr23', 'jesko', 'urus', 'rb19')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--revision', type=Path, default=ROOT / 'artifacts/showroom/revision-4k')
    parser.add_argument('--blender', default='C:/Program Files/Blender Foundation/Blender 4.5/blender.exe')
    parser.add_argument('--samples', type=int, default=256)
    args = parser.parse_args()
    revision = args.revision.resolve()
    destination = ROOT / 'artifacts/showroom'
    studio, media = revision / 'studio', revision / 'media'
    logs = revision / 'render-logs'
    logs.mkdir(parents=True, exist_ok=True)
    status_path = revision / 'job-status.json'
    status = {'pid': os.getpid(), 'started': datetime.now().isoformat(),
              'resolution': [3840, 2160], 'samples': args.samples, 'passed': False}

    def update(stage, **details):
        status.update(stage=stage, updated=datetime.now().isoformat(), **details)
        pending = status_path.with_suffix('.pending.json')
        pending.write_text(json.dumps(status, indent=2), encoding='utf-8')
        pending.replace(status_path)
        print(stage, flush=True)

    def run(label, command):
        update(label)
        env = dict(os.environ, PYTHONUNBUFFERED='1')
        with (logs / f'{label}.log').open('w', encoding='utf-8') as log:
            subprocess.run([str(p) for p in command], cwd=ROOT, env=env,
                           stdout=log, stderr=subprocess.STDOUT, check=True)
        if label.startswith('render-') and 'Shadow buffer full' in (logs / f'{label}.log').read_text(encoding='utf-8', errors='replace'):
            raise RuntimeError(f'Shadow allocation failed; inspect {label}.log before publishing')

    def blender(label, script, arguments, blend=None):
        command = [args.blender, '-b']
        command += [str(blend)] if blend else ['--factory-startup']
        command += ['--python-exit-code', '1', '--python', ROOT / 'tools/showroom' / script]
        if arguments:
            command += ['--', *arguments]
        run(label, command)

    def atomic_copy(source, target):
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary = target.with_name(target.name + '.incoming')
        shutil.copy2(source, temporary)
        temporary.replace(target)

    # Keep an unattended render alive without changing persistent power settings.
    if os.name == 'nt':
        ctypes.windll.kernel32.SetThreadExecutionState(0x80000001)
    try:
        update('preparing')
        # Immutable source snapshot also makes resuming independent of later
        # edits to the default Blender files.
        sources = revision / 'source-studio'
        sources.mkdir(exist_ok=True)
        for car in CARS:
            source = sources / f'{car}-studio.blend'
            if not source.exists():
                shutil.copy2(destination / 'studio' / source.name, source)
        for car in CARS:
            source = sources / f'{car}-studio.blend'
            blender(f'render-{car}', 'rerender_existing.py',
                    ['--out', studio, '--samples', str(args.samples)], source)
        blender('assemble', 'assemble_project.py',
                ['--studio', studio, '--out', revision / 'Black_Showroom.blend'])
        blender('audit-animation', 'audit_animation.py', [], revision / 'Black_Showroom.blend')
        run('package', [sys.executable, ROOT / 'tools/showroom/package_media.py',
                        '--frames', studio / 'frames', '--out', media, '--overview',
                        '--crf', '12', '--preset', 'slow'])
        run('verify-native-playback', [sys.executable, ROOT / 'run.py',
                                       '--showroom-media', media,
                                       '--verify-showroom', revision / 'ui-verification'])
        for report in (media / 'seam_validation.json', media / 'encoded-seams.json',
                       revision / 'ui-verification/verification.json'):
            if not json.loads(report.read_text(encoding='utf-8'))['passed']:
                raise RuntimeError(f'Validation failed: {report}')
        manifest = json.loads((media / 'manifest.json').read_text(encoding='utf-8'))
        if (manifest['width'], manifest['height']) != (3840, 2160):
            raise RuntimeError('The new media is not native 4K')
        movies = [(car[role], data) for car in manifest['cars']
                  for role, data in car['clip_metadata'].items()]
        movies += [(item['video'], item) for item in [*manifest['transitions'], manifest['overview']]]
        for path, metadata in movies:
            if hashlib.sha256((media / path).read_bytes()).hexdigest() != metadata['sha256']:
                raise RuntimeError(f'Media hash mismatch: {path}')
        update('activate')
        backup = destination / ('previous-default-' + datetime.now().strftime('%Y%m%d-%H%M%S'))
        backup.mkdir()
        shutil.copytree(destination / 'media', backup / 'media')
        shutil.copy2(destination / 'Black_Showroom.blend', backup / 'Black_Showroom.blend')
        shutil.copytree(destination / 'studio', backup / 'studio',
                        ignore=shutil.ignore_patterns('frames', '*.blend1'))
        update('activate', backup=str(backup))
        try:
            for source in media.rglob('*'):
                if source.is_file() and source.name != 'manifest.json':
                    atomic_copy(source, destination / 'media' / source.relative_to(media))
            for source in studio.iterdir():
                if source.is_file() and source.suffix in ('.blend', '.json'):
                    atomic_copy(source, destination / 'studio' / source.name)
            atomic_copy(revision / 'Black_Showroom.blend', destination / 'Black_Showroom.blend')
            atomic_copy(media / 'manifest.json', destination / 'media/manifest.json')
            run('verify-default-launch', [sys.executable, ROOT / 'run.py', '--capture-showroom',
                                          revision / 'default-4k-capture.png'])
        except Exception:
            # Restore all previously active assets if activation or launch fails.
            shutil.copytree(backup / 'media', destination / 'media', dirs_exist_ok=True)
            shutil.copytree(backup / 'studio', destination / 'studio', dirs_exist_ok=True)
            shutil.copy2(backup / 'Black_Showroom.blend', destination / 'Black_Showroom.blend')
            raise
        verification = json.loads((revision / 'ui-verification/verification.json').read_text(encoding='utf-8'))
        note = (f"\n{datetime.now():%Y-%m-%d} 4K showroom pack activated: 912 frames at 3840x2160, "
                f"24 fps, {args.samples} Eevee samples and full-resolution ray tracing. "
                "16 state clips, 12 switch videos and overview encoded with H.264 CRF 12/slow. "
                "Source and decoded seam checks passed; actual Qt playback passed "
                f"{verification['checks']} checks in {verification['elapsed_s']} s. "
                f"Evidence: `{revision.relative_to(ROOT).as_posix()}/job-status.json`, "
                "its media reports and ui-verification. Previous defaults backed up at "
                f"`{backup.relative_to(ROOT).as_posix()}`.\n")
        with (ROOT / 'docs/STATUS.md').open('a', encoding='utf-8') as handle:
            handle.write(note)
        update('complete', passed=True, videos=len(movies), active_media=str(destination / 'media'))
    except Exception as error:
        update('failed', error=str(error))
        raise
    finally:
        if os.name == 'nt':
            ctypes.windll.kernel32.SetThreadExecutionState(0x80000000)


if __name__ == '__main__':
    main()
