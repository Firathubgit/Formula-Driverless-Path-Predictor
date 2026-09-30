"""Assemble four editable studio scenes into one self-contained Blender project."""
import argparse
import bpy
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[2] / 'artifacts/showroom'
parser = argparse.ArgumentParser()
parser.add_argument('--studio', type=Path, default=root / 'studio')
parser.add_argument('--out', type=Path, default=root / 'Black_Showroom.blend')
args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else [])
args.studio = args.studio.resolve()
args.out = args.out.resolve()
bpy.ops.wm.read_factory_settings(use_empty=True)
initial = bpy.context.scene
scenes = []
for car in ('amr23', 'jesko', 'urus', 'rb19'):
    with bpy.data.libraries.load(str(args.studio / f'{car}-studio.blend'), link=False) as (source, target):
        target.scenes = [f'SHOWROOM_{car}']
    scenes.extend(target.scenes)
if not all(scenes) or len(scenes) != 4:
    raise RuntimeError('All four studio scenes are required')
bpy.context.window.scene = scenes[0]
bpy.data.scenes.remove(initial)
for scene in scenes:
    scene.frame_set(73)
    scene.frame_preview_start, scene.frame_preview_end = 73, 120
    scene.use_preview_range = True
    scene.unit_settings.system = 'METRIC'
    scene.unit_settings.scale_length = 1.0
    scene.render.dither_intensity = 0.0
    scene.render.filepath = f'//studio/frames/{scene["appearance_id"]}/'
for screen in bpy.data.screens:
    for area in screen.areas:
        if area.type == 'VIEW_3D':
            space = area.spaces.active
            space.region_3d.view_perspective = 'CAMERA'
            space.shading.type = 'RENDERED'
            if hasattr(space.shading, 'use_compositor'):
                space.shading.use_compositor = 'CAMERA'
notes = bpy.data.texts.new('START HERE | four-car black showroom')
notes.write('''FOUR-CAR BLACK SHOWROOM

Choose SHOWROOM_amr23 / jesko / urus / rb19 in Blender's Scene selector.
Every scene contains exactly one car with packed textures and editable motion.
Camera view: numpad 0. Play idle loop: Space. Render a still: F12.
Change the preview range to the clip's markers to inspect another animation.
Timeline markers delimit the four clips at 24 fps:
  ENTER 1-72    black -> close detail -> hero
  IDLE 73-120   hero -> hero, closed camera/light motion
  EXIT 121-156  hero -> black
  SELECT 157-228 hero -> cut to reserved car angle -> black

Every ordered switch A->B = EXIT(A) then ENTER(B). 12 combinations.
Wait for an idle endpoint before exiting/selecting. Never cut idle mid-orbit.
The compositor CANONICAL_BLACK_GATE provides identical black endpoints.
The car stays parked. No racing ribbon exists in the showroom. Selection cuts to
a unique angle and fades to black; the live simulation starts separately, paused.
Appearance changes no physics. Shot names are in each scene's shot_design property.

Generated media/manifest.json defines the native player's media contract.
Rebuild: scripts/build-showroom.ps1 (see docs/SHOWROOM.md).
The supplied source assets and reference scene were read without modification.
''')
args.out.parent.mkdir(parents=True, exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=str(args.out), compress=True)
print('SHOWROOM_PROJECT ' + str(args.out))
