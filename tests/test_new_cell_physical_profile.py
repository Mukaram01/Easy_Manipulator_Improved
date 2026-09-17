"""Real profile handoff: no cached meshes, copied task, or substituted geometry."""
import copy
import json
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from instantiate_workcell_studio_profile import describe, materialize, load_profile
from task_intent_authoring import load_authoring
from task_intent_v2 import normalized_intent_hash
from physical_destination import resolve_destination


def wizard_input(scene):
    profile = describe('ur5_2f_workbench')
    robot, tool = profile['robot'], profile['tool']
    studio = {
        'scenario': {'id': 'static_table_pick_place'},
        'robot': {'model': profile['robot_model'], 'base': robot['base_frame'],
                  'tip': robot['tool_mount_link'], 'planning_group': robot['planning_group']},
        'tool': {'model': profile['tool_model'], 'tcp': tool['grasp_frame']},
        'frames': {'robot_mount_pose': {'xyz': robot['pose_xyz'], 'rpy': robot['pose_rpy']},
                   'tool_mount_pose': {'xyz': tool['mount_pose_xyz'], 'rpy': tool['mount_pose_rpy']}},
        'environment_objects': [dict(row, parent_object='world', parent_link='world', child_link=row['id'], joint_type='fixed', semantic_role=row['role']) for row in profile['rows']],
        'pick_zone': {'pick_object_source': profile['pick_zone'],
                      'object_source': {'mode': 'manual_simulated', 'source_zone_ref': profile['pick_zone']}},
        'place_zone': {'target': profile['place_asset']},
        'task_intent': {'grasp_strategy': 'auto'},
    }
    (scene / 'config').mkdir(parents=True)
    (scene / 'environment.yaml').write_text(yaml.safe_dump({'scene_name': scene.name, 'workcell_studio': studio}))


def test_reviewed_profile_uses_physical_definitions_not_reference_task():
    profile, source = load_profile('ur5_2f_workbench')
    ui = describe(profile['id'])
    assert any(row['id'] == profile['pick_zone'] for row in ui['rows'])
    assert any(row['id'] == profile['place_asset'] for row in ui['rows'])
    assert resolve_destination(source['environment'], profile['place_region'])['target_id'] == profile['place_asset']
    assert 'task' not in ui


def test_existing_destination_is_never_overwritten(tmp_path):
    scene, destination = tmp_path / 'staging' / 'own_cell', tmp_path / 'own_cell'
    wizard_input(scene)
    destination.mkdir()
    before = (scene / 'environment.yaml').read_bytes()
    with pytest.raises(ValueError, match='already exists'):
        materialize(scene, 'ur5_2f_workbench', destination)
    assert (scene / 'environment.yaml').read_bytes() == before
    assert not list(destination.iterdir())


def test_changed_recommended_asset_is_not_silently_replaced(tmp_path):
    scene = tmp_path / 'staging' / 'own_cell'
    wizard_input(scene)
    path = scene / 'environment.yaml'
    authored = yaml.safe_load(path.read_text())
    authored['workcell_studio']['environment_objects'][0]['pose']['xyz'][0] += .2
    path.write_text(yaml.safe_dump(authored))
    before = path.read_bytes()
    with pytest.raises(ValueError, match='reviewed pose'):
        materialize(scene, 'ur5_2f_workbench', tmp_path / 'own_cell')
    assert path.read_bytes() == before


def test_real_humble_creation_and_regeneration_preserve_physical_truth(tmp_path):
    if not Path('/opt/ros/humble/bin/xacro').exists():
        pytest.skip('Real Humble xacro required for production profile acceptance')
    scene, destination = tmp_path / 'staging' / 'independent_cell', tmp_path / 'independent_cell'
    wizard_input(scene)
    materialize(scene, 'ur5_2f_workbench', destination)
    for required in ('scene_manifest.yaml', 'launch/demo.launch.py', 'package.xml', 'config/task_recipe.yaml'):
        assert (scene / required).is_file(), required
    from extract_scene_urdf_visual_mesh_index import _extract_scene_launch_xacro_request
    request = _extract_scene_launch_xacro_request(scene, {})
    assert request is not None, "Generated launch must publish the authored model"
    assert request["mappings"]["use_fake_hardware"] == "true"
    config = yaml.safe_load((scene / "generated/physical_review.rviz").read_text())
    marker = next(d for d in config["Visualization Manager"]["Displays"] if d["Class"] == "rviz_default_plugins/MarkerArray")
    assert marker["Topic"]["Value"] == f"/{scene.name}/canonical_mesh_markers"
    assert marker["Topic"]["Durability Policy"] == "Transient Local"
    scene.rename(destination)
    summary = json.loads((destination / 'generated/generated_workcell_summary.json').read_text())
    for key in ('task_recipe_path', 'detected_objects_example_path', 'environment_objects_path', 'destinations_path', 'source_cell_definition'):
        assert Path(summary[key]).is_file(), (key, summary[key])
    assert summary['blockers'] and not summary['recommended_commands']
    package_xml = (destination / 'package.xml').read_text()
    assert '<exec_depend>ur5_moveit_config</exec_depend>' in package_xml
    assert '<exec_depend>robotiq_85_moveit_config</exec_depend>' in package_xml
    recipe = yaml.safe_load((destination / 'config/task_recipe.yaml').read_text())
    assert recipe['enabled'] is False
    assert recipe['expected']['allow_fallback_rule'] is False
    assert not recipe['decision_rules'] and not recipe['pick']['allowed_grasp_methods']
    blocked_recipe = yaml.safe_load((destination / 'task_recipe_from_builder_intent.yaml').read_text())
    assert blocked_recipe['enabled'] is False
    assert blocked_recipe['task_intent_resolution']['readiness_status'] == 'BLOCKED'
    assert not (destination / 'offline_plan_preview_request.yaml').exists()
    import subprocess
    command = subprocess.run(['bash', str(destination / 'generated/generated_gated_dry_run_command.sh')], capture_output=True, text=True)
    assert command.returncode == 2 and 'BLOCKED' in command.stderr
    _, reviewed = load_profile('ur5_2f_workbench')
    authored = yaml.safe_load((destination / 'environment.yaml').read_text())
    assert authored['environment'] == reviewed['environment']
    assert authored['scene']['id'] == destination.name
    assert 'task' not in authored  # The reference scene task was not cloned.
    intent = load_authoring(destination)['task_intent']
    assert intent['scene_package'] == str(destination)
    assert intent['task']['id'] == destination.name + '_pick_place'
    assert intent['pick']['selection']['source_ref'] == 'pick_zone_main'
    assert intent['place']['target'] == {'asset_ref': 'target_bin_default', 'region_ref': 'default_drop_zone'}
    assert intent['pick']['grasp']['policy'] == 'AUTO'
    assert intent['place']['placement']['policy'] == 'AUTO'
    index = json.loads((destination / 'generated/scene_visual_mesh_index.json').read_text())
    assert index['scene_name'] == destination.name
    assert index['xacro_real_command_succeeded'] is True
    assert index['static_robot_primitive_fallback_count'] == 0
    assert index['unresolved_placeholder_count'] == 0
    assert index['renderable_mesh_count'] >= 15
    assert 'ur5_2f_test' not in (destination / 'urdf/scene.urdf.xacro').read_text()
    before = {rel: (destination / rel).read_bytes() for rel in (
        'environment.yaml', 'layout/workcell_studio_layout.yaml', 'config/workcell_builder_task_intent.yaml')}
    import generate_workcell_from_cell_definition as generator
    assert generator.generate_package(destination / 'cell_definition.yaml', destination.parent,
                                      destination.name, False, False, existing_package_dir=destination) == 0
    assert all((destination / rel).read_bytes() == value for rel, value in before.items())
    assert normalized_intent_hash(load_authoring(destination)['task_intent']) == normalized_intent_hash(intent)
    from validate_workcell_studio_generated_scene import validate
    acceptance = validate(destination)
    assert not acceptance['blockers'], acceptance['blockers']
    invalid = copy.deepcopy(intent)
    invalid['place']['placement']['policy'] = 'EXACT'
    invalid['place']['placement']['requested_local_pose'] = {'xyz_m': [10., 0., 0.], 'rpy_rad': [0., 0., 0.]}
    report = load_authoring(destination, invalid)
    assert report['status'] == 'FAIL'
    assert report['task_intent']['place']['placement']['policy'] == 'EXACT'
    assert any('OUTSIDE_REGION' in error for error in report['errors'])


def test_legacy_converter_cannot_substitute_v2_exact(tmp_path):
    from convert_builder_task_intent_to_task_recipe import convert
    path = tmp_path / 'intent.yaml'
    path.write_text(json.dumps({'schema': 'workcell_builder_task_intent/v2',
                                'pick': {'grasp': {'policy': 'EXACT'}}}))
    with pytest.raises(ValueError, match='shared resolver/preplanner'):
        convert(path)


def test_only_exact_generated_profile_launch_is_upgraded(tmp_path):
    import generate_workcell_from_cell_definition as generator
    scene = tmp_path / 'new_profile'
    (scene / 'launch').mkdir(parents=True)
    (scene / 'urdf').mkdir()
    (scene / 'urdf/scene.urdf.xacro').write_text('<robot/>')
    (scene / 'environment.yaml').write_text('workcell_studio: {recommended_profile: reviewed}\n')
    launch = scene / 'launch/demo.launch.py'
    legacy = generator._render_demo_launch(scene.name, Path('/old/staging/cell_definition.yaml'))
    launch.write_text(legacy)
    assert generator._profile_review_launch_owned(scene, scene.name, 'world')
    launch.write_text(legacy + '# user change\n')
    assert not generator._profile_review_launch_owned(scene, scene.name, 'world')
    launch.write_text(generator._render_physical_review_launch(scene.name, 'world'))
    assert generator._profile_review_launch_owned(scene, scene.name, 'world')
    for data in ('{}', 'workcell_studio: null', '- no profile', '[invalid'):
        (scene / 'environment.yaml').write_text(data)
        assert not generator._profile_review_launch_owned(scene, scene.name, 'world')
    (scene / 'environment.yaml').unlink()
    assert not generator._profile_review_launch_owned(scene, scene.name, 'world')


def test_review_launch_rejects_real_hardware_before_model_or_nodes(tmp_path, monkeypatch):
    launch = pytest.importorskip('launch')
    import runpy
    import subprocess
    import generate_workcell_from_cell_definition as generator
    path = tmp_path / 'demo.launch.py'
    path.write_text(generator._render_physical_review_launch('independent_cell', 'world'))
    module = runpy.run_path(str(path))
    context = launch.LaunchContext()
    context.launch_configurations['use_fake_hardware'] = 'false'
    def forbidden(*args, **kwargs):
        raise AssertionError('Model expansion must not start when real hardware was requested')
    monkeypatch.setattr(subprocess, 'run', forbidden)
    with pytest.raises(RuntimeError, match='real hardware is locked'):
        module['_review'](context)
