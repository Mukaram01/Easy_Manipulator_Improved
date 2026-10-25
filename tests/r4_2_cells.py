"""R4.2: a live-perception cell authored through the Workcell Builder's entry points.

Create New Cell with object source "Live EPD / RealSense" (the wizard's staged environment + instantiate
profile), the Task Editor's helper and save format for the pick/exclusion policy, and the Live Perception
panel's camera binding (live_perception_session.py bind). Nothing generated is edited by hand.
"""
import json
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(Path(__file__).resolve().parent))


def create_live_cell(staging, destination, *, required_class, minimum_confidence=0.5):
    from test_new_cell_physical_profile import wizard_input
    from instantiate_workcell_studio_profile import materialize
    wizard_input(staging)
    doc = yaml.safe_load((staging / 'environment.yaml').read_text())
    source = doc['workcell_studio']['pick_zone']['object_source']
    source.update(mode='live_epd_realsense', geometry_origin='perception_observation', dynamic_pose=True,
                  perception_binding='/easy_perception_deployment/epd_localize_output', required_class=required_class,
                  minimum_confidence=minimum_confidence)
    (staging / 'environment.yaml').write_text(yaml.safe_dump(doc))
    materialize(staging, 'ur5_2f_workbench', destination)
    staging.rename(destination)
    return destination


def task_editor_save(scene, edits):
    from test_r4_1_authored_cell_flow import task_editor_save as save
    return save(scene, edits)


def bind_camera(scene, camera, camera_id):
    """The Live Perception panel's Save Binding for a discovered camera."""
    done = subprocess.run([sys.executable, str(SCRIPTS / 'live_perception_session.py'), 'bind', '--cell', str(scene),
                           '--camera-id', camera_id, '--rgb-topic', camera['rgb_topic'],
                           '--aligned-depth-topic', camera['aligned_depth_topics'][0],
                           '--camera-info-topic', camera['camera_info_topic'], '--optical-frame', camera['optical_frame']]
                          + (['--metadata-topic', camera['metadata_topic']] if camera.get('metadata_topic') else []),
                          capture_output=True, text=True, check=True)
    return json.loads(done.stdout)


def author_live_cell(root, camera, *, name='live_floor_cell', pick_class='mouse', exclude=('cell phone',)):
    root = Path(root)
    scene = create_live_cell(root / 'staging' / name, root / name, required_class=pick_class)
    saved = task_editor_save(scene, [(('pick', 'selection', 'object_filter', 'class_id'), pick_class),
                                     (('pick', 'selection', 'object_filter', 'exclude_class_ids'), list(exclude))])
    camera_id = (yaml.safe_load((scene / 'environment.yaml').read_text()).get('camera') or {}).get('camera_id')
    bound = bind_camera(scene, camera, camera_id)
    return scene, dict(saved=saved, bound=bound)


if __name__ == '__main__':
    out, camera = Path(sys.argv[1]), json.loads(sys.argv[2])
    out.mkdir(parents=True, exist_ok=False)
    scene, steps = author_live_cell(out, camera)
    record = dict(scene=str(scene), saved_status=steps['saved']['status'], errors=steps['saved'].get('errors'),
                  task_filter=steps['saved']['task_intent']['pick']['selection'], live_camera=steps['bound']['live_camera'])
    (out / 'authoring.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(record, indent=2))
