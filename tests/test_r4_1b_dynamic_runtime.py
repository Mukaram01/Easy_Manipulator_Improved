"""R4.1b: the runtime takes robot, tool and environment identity from the saved cell, and its qualified
physics runtime is discovered from a generated, re-validated record rather than a hand-edited script."""
import copy
import importlib.util
import json
import shutil
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parent))

pytestmark = pytest.mark.skipif(not Path('/opt/ros/humble/bin/xacro').exists(), reason='Real Humble xacro required')


@pytest.fixture(scope='module')
def cell(tmp_path_factory):
    from r4_1b_cells import author_cell_b
    return author_cell_b(tmp_path_factory.mktemp('r4_1b_runtime'))[0]


def launch_module():
    pytest.importorskip('launch_ros')
    spec = importlib.util.spec_from_file_location('qualified_demo_launch_r41b', ROOT / 'scenes/ur5_2f_test/launch/demo.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_launch_takes_robot_tool_controllers_and_mount_ids_from_the_bound_cell(cell, tmp_path):
    launch = launch_module()
    assert launch._cell_runtime() == launch.REFERENCE_RUNTIME                       # reference package unchanged
    launch._bind_scene_package_dir(str(cell))
    runtime = launch._cell_runtime()
    controllers = yaml.safe_load((cell / 'config/ros2_controllers.yaml').read_text())
    runtime_binding = yaml.safe_load((cell / 'cell_definition.yaml').read_text())['runtime']
    assert runtime['arm_controller'] == runtime_binding['arm_controller']
    assert runtime['arm_joints'] == controllers[runtime['arm_controller']]['ros__parameters']['joints']
    assert runtime['tool_joints'] == runtime_binding['tool_command_joints']
    environment = yaml.safe_load((cell / 'environment.yaml').read_text())
    assert runtime['camera_pose_id'] == environment['camera']['camera_id']
    assert runtime['robot_moveit_pkg'] == runtime_binding['robot_moveit_package']
    # A renamed controller in the cell is followed; a missing binding is refused, never defaulted.
    renamed = tmp_path / 'renamed'; shutil.copytree(cell, renamed)
    definition = yaml.safe_load((renamed / 'cell_definition.yaml').read_text())
    data = yaml.safe_load((renamed / 'config/ros2_controllers.yaml').read_text())
    data['arm_cell_b'] = data.pop(definition['runtime']['arm_controller'])
    definition['runtime']['arm_controller'] = 'arm_cell_b'
    (renamed / 'config/ros2_controllers.yaml').write_text(yaml.safe_dump(data))
    (renamed / 'cell_definition.yaml').write_text(yaml.safe_dump(definition))
    launch._bind_scene_package_dir(str(renamed))
    assert launch._cell_runtime()['arm_controller'] == 'arm_cell_b'
    definition['runtime'].pop('robot_moveit_package')
    (renamed / 'cell_definition.yaml').write_text(yaml.safe_dump(definition))
    with pytest.raises(RuntimeError, match='robot_moveit_pkg'):
        launch._cell_runtime()
    launch._bind_scene_package_dir('')
    source = (ROOT / 'scenes/ur5_2f_test/launch/demo.launch.py').read_text()
    setup = '\n'.join(line for line in source[source.index('def _launch_setup'):].splitlines()
                      if not line.strip().startswith('#'))
    for literal in ('"ur5_arm_controller"', '"ur5_gripper_controller"', '"shoulder_pan_joint"', '"realsense_overhead"',
                    'robot_moveit_pkg,'):
        assert literal not in setup, literal                                       # only via _cell_runtime()


def test_the_grasp_contract_binds_the_tool_command_joint_group_and_range_to_the_launched_model(cell):
    from perceived_object_grasp_plan import bind_tool_actuation, load_grasp_contract
    contract = load_grasp_contract(cell)
    assert contract['tool_command_joint'] == 'gripper_finger1_joint' and contract['tool_aperture_max_m'] == 0.085
    saved = json.loads((cell / 'config/workcell_builder_task_intent.yaml').read_text())
    assert saved['pick']['grasp']['aperture'] == {'min_m': 0.0, 'max_m': 0.085}         # the cell's own tool opening
    urdf = '<robot name="r"><joint name="gripper_finger1_joint" type="revolute"><limit lower="0" upper="0.804"/></joint>' \
           '<joint name="gripper_finger2_joint" type="revolute"><mimic joint="gripper_finger1_joint"/></joint></robot>'
    srdf = '<robot name="r"><group name="manipulator"><joint name="shoulder_pan_joint"/></group>' \
           '<group name="hand"><joint name="gripper_finger1_joint"/></group>' \
           '<end_effector name="ee" parent_link="tool0" group="hand" parent_group="manipulator"/></robot>'
    bound = bind_tool_actuation(dict(contract), urdf, srdf)
    assert (bound['tool_group'], bound['tool_open_rad'], bound['tool_closed_limit_rad']) == ('hand', 0.0, 0.804)
    with pytest.raises(RuntimeError, match='TOOL_ACTUATION_UNBOUND'):
        bind_tool_actuation(dict(contract, tool_command_joint='gripper_finger2_joint'), urdf, srdf)   # a mimic follower
    with pytest.raises(RuntimeError, match='no unique SRDF end_effector group'):
        bind_tool_actuation(dict(contract), urdf, srdf.replace('group="hand"', 'group="manipulator"'))
    source = (ROOT / 'scripts/full_cycle_preplanner.py').read_text() + (ROOT / 'scripts/perceived_object_grasp_execute.py').read_text()
    assert "{'gripper_finger1_joint'" not in source and "group='gripper'" not in source
    assert '0.804*' not in source and 'close > 0.804' not in source       # the qualified closure reserve constant stays


def test_the_qualified_runtime_record_is_revalidated_and_fails_closed(tmp_path):
    import qualified_physics_runtime as qpr
    with pytest.raises(qpr.RuntimeUnavailable, match='no qualified physics runtime is registered'):
        qpr.load(tmp_path)
    (tmp_path / qpr.RECORD_NAME).write_text(json.dumps({'schema': 'something/v0'}))
    with pytest.raises(qpr.RuntimeUnavailable, match='schema'):
        qpr.load(tmp_path)
    record = dict(schema=qpr.SCHEMA, ros_setup='/opt/ros/humble/setup.bash', workspace_setup=str(tmp_path / 'install/setup.bash'))
    (tmp_path / qpr.RECORD_NAME).write_text(json.dumps(record))
    with pytest.raises(qpr.RuntimeUnavailable, match='build the workspace'):
        qpr.load(tmp_path)
    assert qpr.main(['env', '--workspace', str(tmp_path)]) == 2                   # env prints a failing shell line


def test_planning_moveit_environment_sources_only_the_bound_teardown_overlay(tmp_path):
    import qualified_physics_runtime as qpr
    assert qpr.moveit_environment(tmp_path) == ''  # no record: installed MoveIt, reported by the runner
    overlay = tmp_path / 'moveit overlay'
    (overlay / 'install').mkdir(parents=True)
    (overlay / 'install/setup.bash').write_text('')
    manifest = overlay / 'provenance.json'
    manifest.write_text('{}')
    record = {'schema': qpr.SCHEMA, 'moveit_overlay': {'manifest': str(manifest),
              'manifest_sha256': qpr._sha256(manifest)}}
    (tmp_path / qpr.RECORD_NAME).write_text(json.dumps(record))
    assert qpr.moveit_environment(tmp_path) == f"source '{overlay}/install/local_setup.bash'\n"
    manifest.write_text('{"changed": true}')
    with pytest.raises(qpr.RuntimeUnavailable, match='changed since qualification'):
        qpr.moveit_environment(tmp_path)


def test_the_shell_environment_comes_only_from_the_record():
    import qualified_physics_runtime as qpr
    record = dict(ros_setup='/opt/ros/humble/setup.bash', rmw_implementation='rmw_fastrtps_cpp',
                  workspace_setup='/w s/install/setup.bash', moveit_overlay={'root': '/m'}, bridge_overlay={'root': '/b'},
                  physics_overlay={'root': '/p'}, stage_a_acceptance={'evidence': '/e'})
    text = qpr.shell_environment(record)
    assert "source '/w s/install/setup.bash'" in text and 'source /m/install/setup.bash' in text and 'source /b/setup.bash' in text
    assert 'WORKCELL_SIMULATOR_PHYSICS_ROOT=/p' in text and 'WORKCELL_STAGE_A_PRIOR_EVIDENCE=/e' in text
    assert '/home/' not in (ROOT / 'scripts/qualified_physics_runtime.py').read_text()


def test_physics_preflight_reports_the_first_blocker_in_runner_order(cell, tmp_path):
    from physics_cycle_preflight import preflight
    from r4_1b_cells import task_editor_save
    report = preflight(cell, tmp_path)                       # a workspace with no registered runtime
    assert [(c['name'], c['status']) for c in report['checks']] == [
        ('saved_task', 'PASS'), ('generated_current', 'PASS'), ('physics_envelope', 'PASS'),
        ('perception_replay', 'PASS'), ('qualified_runtime', 'BLOCKED')]        # no replayed perception: passes
    assert report['status'] == 'BLOCKED' and 'no qualified physics runtime is registered' in report['blocker']
    assert report['cell']['objects'] == ['bracket_b (bracket, qualified_test_polymer)']
    preview = tmp_path / 'preview'; shutil.copytree(cell, preview)
    task_editor_save(preview, [(('safety', 'execution_backend'), 'fake'), (('safety', 'require_fake_hardware'), True)])
    report = preflight(preview, tmp_path)
    assert report['checks'][0]['name'] == 'saved_task' and 'Qualified physics simulation' in report['blocker']
    stale = tmp_path / 'stale'; shutil.copytree(cell, stale)
    task_editor_save(stale, [(('place', 'placement', 'clearance_m'), 0.012)])        # saved after Generate
    assert 'Generate the cell again' in preflight(stale, tmp_path)['blocker']
