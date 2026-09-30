"""Audit the opened four-scene cinematic project without rendering it."""
import json
from pathlib import Path

import bpy


def flattened(matrix):
    return [v for row in matrix for v in row]


def near(first, second):
    return len(first) == len(second) and all(abs(a - b) < 1e-5 for a, b in zip(first, second))


report = {}
for scene in bpy.data.scenes:
    car = scene['appearance_id']
    bpy.context.window.scene = scene
    # Initialize the appended scene's dependency graph before its first sample.
    bpy.context.evaluated_depsgraph_get().update()
    scene.frame_set(2)
    camera = scene.camera
    root = next(o for o in scene.objects if o.name.startswith('MOTION_'))
    gate = scene.node_tree.nodes['CANONICAL_BLACK_GATE'].inputs[0]
    assert not any('Racing line' in o.name or 'illustrative handoff' in o.name for o in scene.objects)
    assert len([c for c in scene.collection.children if c.name.startswith('CAR_')]) == 1
    poses = {}
    for frame in range(1, 229):
        scene.frame_set(frame)
        root_pose = flattened(root.matrix_world)
        if frame == 1:
            parked_pose = root_pose
        assert near(root_pose, parked_pose), (car, frame, 'car moved')
        if frame in (1, 156, 228):
            assert abs(gate.default_value) < 1e-8, (car, frame, 'black gate')
        poses[frame] = flattened(camera.matrix_world) + [camera.data.lens]
    for frame in (72, 120, 121, 157, 162):
        assert near(poses[73], poses[frame]), (car, frame, 'hero seam')
    for frame in (24, 48, 162):
        curves = camera.animation_data.action.fcurves
        assert all(next(k for k in c.keyframe_points if int(k.co.x) == frame).interpolation == 'CONSTANT'
                   for c in curves), (car, frame, 'cut interpolation')
    assert all(not near(poses[185], poses[f]) for f in range(1, 163)), (car, 'reused select angle')
    report[car] = {'parked_frames': 228, 'racing_lines': 0, 'hero_boundaries_match': True,
                   'hard_cuts': [25, 49, 163], 'selection_view': json.loads(scene['shot_design'])['select'][0]}
    scene.frame_set(73)
assert set(report) == {'amr23', 'rb19', 'jesko', 'urus'}
path = Path(bpy.data.filepath).parent / 'animation-audit.json'
path.write_text(json.dumps(report, indent=2), encoding='utf-8')
print('ANIMATION_AUDIT ' + json.dumps(report))
