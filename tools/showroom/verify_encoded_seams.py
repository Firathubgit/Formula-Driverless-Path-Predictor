"""Check decoded MP4 boundary pixels after encoding, independently of source PNGs."""
import argparse
import itertools
import json
from pathlib import Path
import subprocess

from PIL import Image
from package_media import image_rmse, resolve_ffmpeg


def validate_encoded_seams(media, ffmpeg=None):
    media = Path(media)
    manifest = json.loads((media / 'manifest.json').read_text(encoding='utf-8'))
    ffmpeg = ffmpeg or resolve_ffmpeg(None)
    size = (manifest['width'], manifest['height'])
    frame_bytes = size[0] * size[1] * 3
    report = {'passed': True, 'hero_rmse_limit_8bit': 2.0, 'black_maximum_limit': 0, 'cars': []}
    for car in manifest['cars']:
        edges = {}
        for role in ('enter', 'idle', 'exit', 'select'):
            count = car['clip_metadata'][role]['frames']
            result = subprocess.run([str(ffmpeg), '-hide_banner', '-v', 'error', '-nostdin',
                                     '-i', str(media / car[role]), '-vf',
                                     f'select=eq(n\\,0)+eq(n\\,{count-1})', '-fps_mode', 'passthrough',
                                     '-pix_fmt', 'rgb24', '-f', 'rawvideo', 'pipe:1'], capture_output=True, check=True)
            assert len(result.stdout) == 2 * frame_bytes, (car['id'], role, 'boundary decode size')
            edges[role] = [Image.frombytes('RGB', size, result.stdout[:frame_bytes]),
                           Image.frombytes('RGB', size, result.stdout[frame_bytes:])]
        black_maximum = max(max(v[1] for v in image.getextrema()) for image in
                            (edges['enter'][0], edges['exit'][1], edges['select'][1]))
        hero = [edges['enter'][1], *edges['idle'], edges['exit'][0], edges['select'][0]]
        rmse = max(image_rmse(a, b)[0] for a, b in itertools.combinations(hero, 2))
        passed = black_maximum == 0 and rmse <= 2.0
        report['cars'].append({'id': car['id'], 'black_maximum': black_maximum,
                               'hero_max_rmse_8bit': rmse, 'passed': passed})
        report['passed'] &= passed
    (media / 'encoded-seams.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--media', required=True, type=Path)
    args = parser.parse_args()
    report = validate_encoded_seams(args.media)
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report['passed'] else 1)
