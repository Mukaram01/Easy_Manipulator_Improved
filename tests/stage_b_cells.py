"""Stage-B: a perception-to-physics cell authored only through the Workcell Builder's entry points.

Create New Cell with two physical test objects (box collision bodies; their visuals are declared meshes),
Studio Save of an oblique camera pose, the Task Editor's simulate/pick/exclude edits, Generate and Validate.
Then the camera-free perception steps (``perception_replay.py record`` and ``detect``) and the Task Editor's
"Replayed snapshot" input. No generated or runtime file is edited by hand.

Objects carry no perception truth: the authored ``class_id`` is ``unclassified``; classes come only from EPD.
Visual assets (copied into the cell): Google Scanned Objects "ACE Coffee Mug Kristen 16 oz cup" (CC-BY 4.0,
scaled 0.6, handle visual-only) and a procedurally generated beer bottle (revolved profile).
"""
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from test_r4_1_authored_cell_flow import SIMULATE, create_cell, generate_and_validate, task_editor_save  # noqa: E402

ASSETS = Path.home() / 'workcell_ws/stage_b_assets/meshes'
# The mug scan's body is 81.6 mm across and 134.9 mm tall, centred 17.6 mm from its origin (handle on +x).
# Authored inside the qualified top-grasp envelope: 30 mm wide (closed-tool penetration 0.3-0.8 mm on the
# 25-30 mm Stage-A/R4.1b parts) and 45 mm tall. Taller bodies (66-88 mm) put the finger links above the pads
# inside the part's extent and exceed the 1 mm closed-tool penetration limit at any width 30-50 mm (measured).
# The result is an espresso cup (scale 0.368 across, 0.333 in height); the handle is visual-only.
CUP_SCALE = [0.3676, 0.3676, 0.3336]
CUP_BODY = [round(0.0816 * CUP_SCALE[0], 4), round(0.0816 * CUP_SCALE[1], 4), round(0.1349 * CUP_SCALE[2], 4)]
CUP = {'object_id': 'object_cup', 'class_id': 'unclassified', 'material': 'qualified_test_polymer', 'shape': 'box',
       'dimensions': CUP_BODY, 'pose': {'frame_id': 'world', 'xyz': [0.50, -0.25, CUP_BODY[2] / 2], 'rpy': [0, 0, 0]},
       'visual': {'mesh': 'assets/visuals/mug_ace/model.obj', 'scale': list(CUP_SCALE),
                  'offset_xyz': [0.0176 * CUP_SCALE[0], 0.0007 * CUP_SCALE[1], -CUP_BODY[2] / 2],
                  'source': 'Google Scanned Objects: ACE_Coffee_Mug_Kristen_16_oz_cup (Gazebo Fuel)', 'license': 'CC-BY-4.0'}}
BOTTLE = {'object_id': 'object_bottle', 'class_id': 'unclassified', 'material': 'qualified_test_polymer', 'shape': 'box',
          'dimensions': [0.062, 0.062, 0.23], 'pose': {'frame_id': 'world', 'xyz': [0.32, -0.33, 0.115], 'rpy': [0, 0, 0]},
          'visual': {'mesh': 'assets/visuals/beer_bottle_procedural/model.obj', 'scale': [1, 1, 1], 'offset_xyz': [0, 0, -0.115],
                     'source': 'procedurally generated 0.33 l beer bottle (revolved profile)', 'license': 'generated'}}
# Camera beyond the pick zone looking back low at the cup: EPD's COCO model needs an oblique view, and a 40 mm
# cup needs ~0.45 m range and a side-on view for a confident detection with an unambiguous principal axis
# (from 40 deg above, its rim and mouth make the visible cloud axis-degenerate and EPD rejects it).
CAMERA_XYZ = [0.85, -0.25, 0.28]                   # 0.89 m from the robot base: outside the UR5's 0.85 m reach
CAMERA_RPY = [0.0, 0.47, math.pi]                  # 27 deg down: the cup and the whole bottle in view


def create_stage_b_cell(root, name, objects=(CUP, BOTTLE)):
    staging, destination = Path(root) / 'staging' / name, Path(root) / name
    scene = create_cell(staging, destination, objects[0])
    doc = yaml.safe_load((scene / 'environment.yaml').read_text())
    source = doc['workcell_studio']['pick_zone']['object_source']
    source.pop('manual_observation', None)
    source['manual_observations'] = [dict(o) for o in objects]
    (scene / 'environment.yaml').write_text(yaml.safe_dump(doc, sort_keys=False))
    for mesh in ('mug_ace', 'beer_bottle_procedural'):
        shutil.copytree(ASSETS / mesh, scene / 'assets/visuals' / mesh, dirs_exist_ok=True)
    return scene


def camera_save(scene, camera_id):
    """Studio: move and turn the camera in the saved layout, then the Save projection MainWindow runs."""
    from workcell_studio_layout_merge import merge
    path = scene / 'layout/workcell_studio_layout.yaml'
    layout = yaml.safe_load(path.read_text())
    items = [item for item in layout['items'] if item.get('id') == camera_id]
    if len(items) != 1:
        raise ValueError(f'layout has no camera item {camera_id!r}')
    items[0]['pose'] = dict(items[0].get('pose') or {}, xyz=list(CAMERA_XYZ), rpy=list(CAMERA_RPY))
    path.write_text(yaml.safe_dump(layout, sort_keys=False))
    report = merge(scene, save_authored=True)
    if report['status'] != 'READY':
        raise RuntimeError(report)
    return report


def replay_input_edits(replay_path, max_age_s=180.0):
    """The Task Editor's perception mode "Replayed snapshot" with its saved input.

    In the physics simulation the planner consumes the measured object, so max_age follows the simulator
    measurement policy; the replayed detection itself is capped at the 2 s replay ceiling (recording clock).
    """
    return [(('perception', 'mode'), 'replayed_snapshot'), (('perception', 'replay_path'), str(replay_path)),
            (('pick', 'selection', 'source_type'), 'perception'), (('pick', 'selection', 'source_ref'), 'detected_objects/v1'),
            (('pick', 'selection', 'object_filter', 'max_age_seconds'), max_age_s)]


def author_stage_b_cell(root, name='stage_b_cell', pick='cup', exclude=('bottle',), min_confidence=0.5):
    scene = create_stage_b_cell(root, name)
    camera_id = yaml.safe_load((scene / 'environment.yaml').read_text())['camera']['camera_id']
    studio = camera_save(scene, camera_id)
    saved = task_editor_save(scene, [*SIMULATE, (('pick', 'selection', 'object_filter', 'class_id'), pick),
                                     (('pick', 'selection', 'object_filter', 'exclude_class_ids'), list(exclude)),
                                     (('pick', 'selection', 'object_filter', 'min_confidence'), min_confidence)])
    rc, validation = generate_and_validate(scene)
    return scene, dict(studio=studio, saved=saved, validation_rc=rc, validation=validation)


def perceive(scene, workspace):
    """record -> detect -> Task Editor replay input -> Generate/Validate."""
    recording = scene / 'perception/recording'
    def run(*argv):
        done = subprocess.run([sys.executable, str(SCRIPTS / 'perception_replay.py'), *argv], capture_output=True, text=True)
        report = json.loads(done.stdout.strip().splitlines()[-1]) if done.stdout.strip() else {}
        if done.returncode != 0:
            raise RuntimeError(f'{argv[0]}: {report or done.stderr[-2000:]}')
        return report
    recorded = run('record', '--cell', str(scene), '--output', str(recording))
    detected = run('detect', '--recording', str(recording), '--output', str(scene / 'perception/perception_replay.yaml'),
                   '--workspace', str(workspace), '--cell', str(scene))
    saved = task_editor_save(scene, replay_input_edits('perception/perception_replay.yaml'))
    rc, validation = generate_and_validate(scene)
    return dict(recorded=recorded, detected=detected, saved_status=saved['status'], errors=saved.get('errors'),
                planning_input=saved.get('planning_input'), planning_input_blocker=saved.get('planning_input_blocker'),
                validation_rc=rc)


if __name__ == '__main__':
    out, workspace = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=False)
    scene, steps = author_stage_b_cell(out)
    record = dict(scene=str(scene), authored_status=steps['saved']['status'], authored_validation_rc=steps['validation_rc'])
    record.update(perception=perceive(scene, workspace))
    (out / 'authoring.json').write_text(json.dumps(record, indent=2, default=str) + '\n')
    print(json.dumps(record, indent=2, default=str))
