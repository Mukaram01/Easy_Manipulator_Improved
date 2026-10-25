"""R4.1 offline: Create New Cell -> Task Editor save -> reopen -> Generate -> Validate -> physics world.

Each step uses the production entry point the Workcell Builder GUI uses (instantiate profile, the Task
Editor's authoring helper and save format, the Generate and Validate commands). Nothing is copied from the
reference scene's task and no generated value is replaced.
"""
import copy
import json
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(Path(__file__).resolve().parent))

# The beginner's manual test object (Create New Cell > object source: manual simulated).
MANUAL_OBJECT = {'object_id': 'part_04', 'shape': 'box', 'dimensions': [0.025, 0.025, 0.025],
                 'pose': {'frame_id': 'world', 'xyz': [0.40, -0.20, 0.0125], 'rpy': [0, 0, 0]}}


def create_cell(staging, destination, manual_object=MANUAL_OBJECT):
    """Create New Cell for the reviewed UR5 + Robotiq 2F-85 workbench profile (wizard selections)."""
    from test_new_cell_physical_profile import wizard_input
    from instantiate_workcell_studio_profile import materialize
    wizard_input(staging)
    doc = yaml.safe_load((staging / 'environment.yaml').read_text())
    source = doc['workcell_studio']['pick_zone']['object_source']
    source.update(geometry_origin='manual_authored', manual_observation=copy.deepcopy(manual_object))
    (staging / 'environment.yaml').write_text(yaml.safe_dump(doc))
    materialize(staging, 'ur5_2f_workbench', destination)
    staging.rename(destination)  # the wizard publishes the staged cell into place
    return destination


def task_editor_save(scene, edits):
    """The Task Editor: validate the edited draft with its helper, then save the normalized intent as JSON."""
    path = scene / 'config/workcell_builder_task_intent.yaml'
    draft = json.loads(path.read_text())
    for keys, value in edits:
        cursor = draft
        for key in keys[:-1]:
            cursor = cursor.setdefault(key, {})
        cursor[keys[-1]] = value
    report = json.loads(subprocess.run([sys.executable, str(SCRIPTS / 'task_intent_authoring.py'), str(scene), '--stdin'],
                                       input=json.dumps(draft), capture_output=True, text=True, check=True).stdout)
    path.write_text(json.dumps(report['task_intent'], indent=4))
    return report


# The edits a beginner makes in the Task Editor: run the cycle in the qualified physics simulation.
SIMULATE = [(('safety', 'execution_backend'), 'simulator'), (('safety', 'require_fake_hardware'), False)]


def generate_and_validate(scene):
    """The GUI's Generate (YAML draft, then package) and Validate commands."""
    subprocess.run([sys.executable, str(SCRIPTS / 'export_builder_scene_to_cell_definition.py'), str(scene),
                    '--output-dir', str(scene), '--validate'], check=True, capture_output=True, text=True)
    subprocess.run([sys.executable, str(SCRIPTS / 'generate_workcell_from_cell_definition.py'), str(scene / 'cell_definition.yaml'),
                    '--output-dir', str(scene.parent), '--package-name', scene.name, '--existing-package-dir', str(scene)],
                   check=True, capture_output=True, text=True)
    validation = subprocess.run([sys.executable, str(SCRIPTS / 'validate_builder_generated_scene.py'), str(scene), '--json'],
                                capture_output=True, text=True)
    return validation.returncode, json.loads(validation.stdout)


def build_reference_beginner_cell(root):
    """The full offline authoring flow; returns the generated cell and the recorded step results."""
    root = Path(root)
    scene = create_cell(root / 'staging' / 'beginner_cell', root / 'beginner_cell')
    from task_intent_authoring import load_authoring
    created = load_authoring(scene)
    saved = task_editor_save(scene, SIMULATE)
    reopened = load_authoring(scene)
    rc, validation = generate_and_validate(scene)
    return scene, dict(created=created, saved=saved, reopened=reopened, validation_rc=rc, validation=validation)


pytestmark = pytest.mark.skipif(not Path('/opt/ros/humble/bin/xacro').exists(), reason='Real Humble xacro required')


@pytest.fixture(scope='module')
def flow(tmp_path_factory):
    return build_reference_beginner_cell(tmp_path_factory.mktemp('r4_1'))


def test_created_cell_binds_the_reviewed_tool_capability_and_a_fresh_v2_task(flow):
    scene, steps = flow
    environment = yaml.safe_load((scene / 'environment.yaml').read_text())
    assert environment['end_effector']['capability'] == 'robotiq_2f_85'
    created = steps['created']['task_intent']
    assert created['schema'] == 'workcell_builder_task_intent/v2'
    assert created['pick']['selection']['source_type'] == 'manual_simulated'
    assert created['pick']['selection']['object_filter']['max_age_seconds'] == 180.0   # simulated snapshot policy
    assert created['safety']['execution_backend' if 'execution_backend' in created['safety'] else 'require_fake_hardware'] in ('fake', True)


def test_task_editor_save_and_reopen_keep_the_authored_simulator_task_identical(flow):
    scene, steps = flow
    saved, reopened = steps['saved'], steps['reopened']
    assert saved['status'] == 'PASS' and reopened['status'] == 'PASS', (saved.get('errors'), reopened.get('errors'))
    assert reopened['task_intent'] == saved['task_intent']
    assert reopened['normalized_intent_sha256'] == saved['normalized_intent_sha256']
    safety = reopened['task_intent']['safety']
    assert safety['execution_backend'] == 'simulator' and safety['require_fake_hardware'] is False
    assert safety['real_hardware_enabled'] is False
    on_disk = json.loads((scene / 'config/workcell_builder_task_intent.yaml').read_text())
    assert on_disk == saved['task_intent']                                          # no hidden rewrite


def test_generate_binds_the_saved_task_and_validates_the_handoff(flow):
    scene, steps = flow
    assert steps['validation_rc'] == 0, steps['validation']
    cell = yaml.safe_load((scene / 'cell_definition.yaml').read_text())
    from task_intent_v2 import normalized_intent_hash
    intent = json.loads((scene / 'config/workcell_builder_task_intent.yaml').read_text())
    assert cell['normalized_intent_sha256'] == normalized_intent_hash(intent)        # executor's TASK_HANDOFF check
    assert cell['builder_task_intent']['schema'] == 'workcell_builder_task_intent/v2'
    for rel in ('urdf/scene.urdf.xacro', 'urdf/arm_hand.srdf.xacro', 'config/ros2_controllers.yaml',
                'config/moveit_collision_objects.yaml', 'launch/demo.launch.py'):
        assert (scene / rel).is_file(), rel


def test_the_saved_cell_derives_a_physics_world_from_its_own_data(flow):
    scene, _ = flow
    from simulator_world import derive_world
    text, report = derive_world(scene)
    assert report['objects'][0]['object_id'] == 'part_04'
    assert report['objects'][0]['pose'][:2] == pytest.approx([0.40, -0.20])
    assert report['destination']['asset_ref'] == 'target_bin_default'


def test_the_runner_accepts_only_a_saved_simulator_task(flow, tmp_path):
    scene, _ = flow
    import importlib.util
    spec = importlib.util.spec_from_file_location('runner', SCRIPTS / 'run_stage_a1_finish.py')
    runner = importlib.util.module_from_spec(spec); spec.loader.exec_module(runner)
    cell = runner.authored_cell(scene)
    assert cell['name'] == 'beginner_cell' and cell['class_id'] == 'unclassified'
    copied = runner.prepare_authored_scene(ROOT, scene, tmp_path / 'session' / 'beginner_cell')
    assert (copied / 'config/workcell_builder_task_intent.yaml').read_bytes() == \
        (scene / 'config/workcell_builder_task_intent.yaml').read_bytes()              # used unchanged
    preview = tmp_path / 'preview_cell'
    import shutil
    shutil.copytree(scene, preview)
    task_editor_save(preview, [(('safety', 'execution_backend'), 'fake'), (('safety', 'require_fake_hardware'), True)])
    with pytest.raises(RuntimeError, match='AUTHORED_TASK_NOT_SIMULATOR'):
        runner.authored_cell(preview)
