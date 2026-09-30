"""Rerender an approved studio without rebuilding materials, lights or animation.

Run inside Blender with an existing *-studio.blend open. Output is resumable;
only PNGs with the requested dimensions are reused.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import bpy


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--out', type=Path, required=True)
parser.add_argument('--width', type=int, default=3840)
parser.add_argument('--samples', type=int, default=256)
parser.add_argument('--frames', type=int, nargs='+')
args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
scene = bpy.context.scene
car = scene['appearance_id']
out = args.out.resolve()
frames = out / 'frames' / car
frames.mkdir(parents=True, exist_ok=True)
scene.use_preview_range = False
scene.render.resolution_x = args.width
scene.render.resolution_y = args.width * 9 // 16
scene.render.resolution_percentage = 100
scene.render.dither_intensity = 0
scene.eevee.taa_render_samples = args.samples
scene.eevee.use_raytracing = True
scene.eevee.shadow_pool_size = '1024'
scene.eevee.ray_tracing_options.resolution_scale = '1'
scene.eevee.ray_tracing_options.screen_trace_quality = 1.0
scene.render.image_settings.file_format = 'PNG'
scene.render.image_settings.color_mode = 'RGB'
scene.render.image_settings.color_depth = '8'
scene.render.image_settings.compression = 30
scene.render.filepath = str(frames) + '/'
settings = {
    'car': car, 'width': scene.render.resolution_x,
    'height': scene.render.resolution_y, 'fps': scene.render.fps,
    'samples': args.samples, 'engine': scene.render.engine,
    'ray_tracing_resolution_scale': scene.eevee.ray_tracing_options.resolution_scale,
    'shadow_pool_size_mb': scene.eevee.shadow_pool_size,
    'source': bpy.data.filepath, 'shot_design': scene.get('shot_design'),
    'source_sha256': hashlib.sha256(Path(bpy.data.filepath).read_bytes()).hexdigest(),
}
settings_path = out / f'{car}-quality.json'
if settings_path.exists():
    previous = json.loads(settings_path.read_text(encoding='utf-8'))
    if any(previous.get(key) != value for key, value in settings.items() if key not in ('source', 'source_sha256')):
        raise RuntimeError('Existing frames use different render settings; choose a fresh output directory')
    if previous.get('source_sha256', settings['source_sha256']) != settings['source_sha256']:
        raise RuntimeError('The approved source changed; choose a fresh output directory')
settings_path.write_text(json.dumps(settings, indent=2), encoding='utf-8')
bpy.ops.wm.save_as_mainfile(filepath=str(out / f'{car}-studio.blend'), compress=True)
for number in args.frames or range(1, 229):
    target = frames / f'{number:04d}.png'
    if target.exists():
        # PNG signature and IHDR dimensions are enough here; packaging fully
        # decodes every frame before any media can be activated.
        with target.open('rb') as source:
            header = source.read(24)
        if (header[:8] == b'\x89PNG\r\n\x1a\n'
                and int.from_bytes(header[16:20], 'big') == scene.render.resolution_x
                and int.from_bytes(header[20:24], 'big') == scene.render.resolution_y):
            print(f'REUSED {car} {number:04d}', flush=True)
            continue
    started = time.monotonic()
    scene.frame_set(number)
    scene.render.filepath = str(target.with_name(target.stem + '.pending.png'))
    bpy.ops.render.render(write_still=True)
    Path(scene.render.filepath).replace(target)
    seconds = time.monotonic() - started
    progress = {'car': car, 'last_frame': number, 'last_frame_seconds': round(seconds, 2),
                'completed_frames': sum(1 for path in (out / 'frames').glob('*/*.png')
                                        if not path.name.endswith('.pending.png')),
                'total_frames': 912, 'updated_unix': time.time()}
    pending = out.parent / 'render-progress.pending.json'
    pending.write_text(json.dumps(progress, indent=2), encoding='utf-8')
    pending.replace(out.parent / 'render-progress.json')
    print(f'RENDERED {car} {number:04d} seconds={seconds:.2f}', flush=True)
