"""Real profile handoff: no cached meshes, copied task, or substituted geometry."""
import copy
import json
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from instantiate_workcell_studio_profile import describe, equipment, materialize, load_profile
from task_intent_authoring import load_authoring
from task_intent_v2 import normalized_intent_hash
from physical_destination import resolve_destination


def wizard_input(scene, scenario='static_table_pick_place', objects=None):
    profile = describe('ur5_2f_workbench', scenario)
    robot, tool = profile['robot'], profile['tool']
    studio = {
        'scenario': {'id': scenario},
        'robot': {'capability': profile['robot_capability'], 'base': robot['base_frame'],
                  'tip': robot['tool_mount_link'], 'planning_group': robot['planning_group']},
        'tool': {'capability': profile['tool_capability'], 'tcp': tool['grasp_frame']},
        'frames': {'robot_mount_pose': {'xyz': robot['pose_xyz'], 'rpy': robot['pose_rpy']},
                   'tool_mount_pose': {'xyz': tool['mount_pose_xyz'], 'rpy': tool['mount_pose_rpy']}},
        'environment_objects': [dict(row, parent_object='world', parent_link='world', child_link=row['id'], joint_type='fixed', semantic_role=row['role']) for row in profile['rows']],
        'pick_zone': {'pick_object_source': profile['pick_zone'],
                      'object_source': {'mode': 'manual_simulated', 'source_zone_ref': profile['pick_zone'],
                                        **({'manual_observations': objects} if objects is not None else {})}},
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


def test_equipment_choices_join_catalog_installed_models_and_reviewed_profiles():
    """New Cell robot/tool choices: catalog presence never implies runtime readiness."""
    found = equipment()
    status = {(c['robot'], c['tool']): c['status'] for c in found['combinations']}
    assert status[('ur5', 'robotiq_2f_85')] == 'PLAN_CAPABLE'                 # reviewed profile ur5_2f_workbench
    assert status[('ur5', 'onrobot_airpick_style')] == 'CONFIGURABLE'         # installed, no reviewed runtime binding
    assert status[('ur5', 'suction_basic')] == 'UNSUPPORTED'                  # declares no compatible robot family
    assert status[('generic_scara_600', 'robotiq_2f_85')] == 'UNCOMMISSIONED'  # catalog record, no installed model
    ur5 = next(r for r in found['robots'] if r['id'] == 'ur5')
    # Frames and groups from the installed MoveIt config (SRDF) and the record.
    assert ur5['installed'] and 'manipulator' in ur5['planning_groups'] and ur5['tip_frames'][0] == 'tool0'
    airpick = next(t for t in found['tools'] if t['id'] == 'onrobot_airpick_style')
    assert airpick['default_attach'] == 'gripper_base_link'                    # the installed macro's root link
    assert airpick['grasp_strategies'] == ['suction_top_basic'] and airpick['release'] == 'vacuum_off'
    profile = next(p for p in found['profiles'] if p['id'] == 'ur5_2f_workbench')
    assert profile['arm_controller'] == 'ur5_arm_controller' and profile['defaults']['tcp'] == 'ee_palm'
    assert any('ur10_moveit_config has no capability record' in note for note in found['notes'])


def test_profile_must_launch_the_equipment_it_names(tmp_path, monkeypatch):
    import instantiate_workcell_studio_profile as helper
    profile = yaml.safe_load((helper.PROFILES / 'ur5_2f_workbench.yaml').read_text())
    (tmp_path / 'mislabelled.yaml').write_text(yaml.safe_dump(dict(profile, tool_capability='onrobot_airpick_style')))
    monkeypatch.setattr(helper, 'PROFILES', tmp_path)
    with pytest.raises(ValueError, match='binds tool'):
        load_profile('mislabelled')


def test_export_refuses_equipment_that_is_not_plan_capable(tmp_path):
    from export_builder_scene_to_cell_definition import export_scene
    (tmp_path / 'environment.yaml').write_text(yaml.safe_dump({'workcell_studio': {'equipment': {
        'status': 'CONFIGURABLE', 'reason': 'UR5 + AirPick: no reviewed New Cell profile.'}}}))
    with pytest.raises(ValueError, match='EQUIPMENT_NOT_PLAN_CAPABLE'):
        export_scene(tmp_path, tmp_path, validate=False)


def configurable_input(scene, tcp='suction_tip', tip='tool0'):
    """What the wizard saves for UR5 + AirPick (CONFIGURABLE): equipment, frames and the prefilled mount."""
    pair = next(c for c in equipment()['combinations'] if (c['robot'], c['tool']) == ('ur5', 'onrobot_airpick_style'))
    studio = {'equipment': {'status': 'CONFIGURABLE', 'reason': pair['reason'], 'profile': '',
                            'robot_capability': 'ur5', 'tool_capability': 'onrobot_airpick_style'},
              'robot': {'capability': 'ur5', 'base': 'base_link', 'tip': tip, 'planning_group': 'manipulator',
                        'controller': '', 'readiness': 'CONFIGURABLE'},
              'tool': {'capability': 'onrobot_airpick_style', 'attach': 'gripper_base_link', 'tcp': tcp,
                       'readiness': 'CONFIGURABLE'},
              'frames': {'robot_mount_pose': {'xyz': [0.1, -0.2, 0.0], 'rpy': [0.0, 0.0, 0.5]},
                         'tool_mount_pose': pair['assembly']['tool_mount']}}
    scene.mkdir(parents=True)
    (scene / 'environment.yaml').write_text(yaml.safe_dump({'scene_name': scene.name, 'robot': 'ur5',
                                                            'end_effector': 'onrobot_airpick_style',
                                                            'workcell_studio': studio}))


def test_configurable_pair_reuses_the_supported_scene_that_assembles_it():
    """A configurable pair's 3D model comes from an existing supported scene that mounts that installed tool on that
    robot (its macros and literal mount), never from an invented transform; without one it fails closed."""
    pairs = {(c['robot'], c['tool']): c for c in equipment()['combinations']}
    airpick = pairs[('ur5', 'onrobot_airpick_style')]
    assert airpick['status'] == 'CONFIGURABLE' and airpick['profile'] is None
    assert airpick['assembly'] == {
        'scene': 'ur5_airpick4_test', 'source': 'scenes/ur5_airpick4_test/urdf/scene.urdf.xacro',
        'robot_macro': 'ur_robot', 'tool_macro': 'onrobot_airpick4_gripper', 'tool_parent': 'tool0',
        'tool_mount': {'xyz': [0.0, 0.0, 0.0324], 'rpy': [-1.5707, 0.0, 0.0]}}
    # The Robotiq 3F record names no tool model file, so there is no macro to match: no geometry is guessed.
    three_finger = pairs[('ur5', 'robotiq_3f')]
    assert three_finger['assembly'] is None and 'no validated tool mount' in three_finger['assembly_reason']
    assert 'assembly' not in pairs[('ur5', 'robotiq_2f_85')]     # plan-capable cells use their reviewed profile
    tools = {t['id']: t for t in equipment()['tools']}
    assert tools['onrobot_airpick_style']['model_links'] == ['gripper_base_link']   # no suction_tip link installed


def test_authoring_model_is_only_for_configurable_equipment(tmp_path):
    from instantiate_workcell_studio_profile import assemble_authoring_model
    configurable_input(tmp_path / 'wrong_tip', tip='ee_link')
    with pytest.raises(ValueError, match='relative to tool0'):
        assemble_authoring_model(tmp_path / 'wrong_tip')
    configurable_input(tmp_path / 'reviewed')
    env = yaml.safe_load((tmp_path / 'reviewed' / 'environment.yaml').read_text())
    env['workcell_studio']['robot']['capability'], env['workcell_studio']['tool']['capability'] = 'ur5', 'robotiq_2f_85'
    (tmp_path / 'reviewed' / 'environment.yaml').write_text(yaml.safe_dump(env))
    with pytest.raises(ValueError, match='PLAN_CAPABLE, not CONFIGURABLE'):
        assemble_authoring_model(tmp_path / 'reviewed')


def test_configurable_cell_gets_installed_geometry_but_stays_blocked(tmp_path):
    if not Path('/opt/ros/humble/bin/xacro').exists():
        pytest.skip('Real Humble xacro required for the authoring model')
    from instantiate_workcell_studio_profile import assemble_authoring_model
    from export_builder_scene_to_cell_definition import export_scene
    scene = tmp_path / 'airpick_cell'
    configurable_input(scene)
    record = assemble_authoring_model(scene)
    assert record['status'] == 'ASSEMBLED' and record['purpose'] == 'layout_authoring_only'
    assert record['reference_scene'] == 'ur5_airpick4_test' and record['attach_link'] == 'gripper_base_link'
    assert record['tcp_defined'] is False and 'no installed model defines it' in record['warnings'][0]
    model = (scene / 'urdf/scene.urdf.xacro').read_text()
    # Installed macros only: no table, camera, task or reference environment.
    assert 'ur_macro.xacro' in model and 'onrobot_airpick4_gripper.urdf.xacro' in model
    assert 'table' not in model and 'd435i' not in model and 'ur5_airpick4_test' not in model
    index = json.loads((scene / 'generated/scene_visual_mesh_index.json').read_text())
    assert index['xacro_real_command_succeeded'] and index['safe_for_preview'] and not index['blockers']
    poses = {i['link']: i['link_world_pose'] for i in index['visual_items']}
    assert {'base_link_inertia', 'wrist_3_link', 'gripper_base_link'} <= set(poses)
    assert poses['base_link_inertia']['xyz'] == pytest.approx([0.1, -0.2, 0.0], abs=1e-6)  # the cell's own mount
    # The tool sits 32.4 mm out along tool0's z axis: the reference mount, applied in this cell's robot pose.
    import numpy as np
    from scipy.spatial.transform import Rotation
    tool0_z = Rotation.from_euler('xyz', poses['tool0']['rpy']).apply([0, 0, 1])
    offset = np.array(poses['gripper_base_link']['xyz']) - np.array(poses['tool0']['xyz'])
    assert offset == pytest.approx(0.0324 * tool0_z, abs=1e-4)
    saved = yaml.safe_load((scene / 'environment.yaml').read_text())['workcell_studio']
    assert saved['equipment']['status'] == 'CONFIGURABLE' and saved['authoring_model']['tool_parent'] == 'tool0'
    with pytest.raises(ValueError, match='EQUIPMENT_NOT_PLAN_CAPABLE'):
        export_scene(scene, scene, validate=False)
    # Product View loads it with only its equipment required; workbench and camera are laid out by the user.
    from export_workcell_studio_web_scene_impl import build_web_scene
    metadata = build_web_scene(scene)['metadata']
    assert metadata['required_physical_categories'] == ['robot_arm', 'attached_tool_gripper']
    assert metadata['authoring_only_equipment']['status'] == 'CONFIGURABLE'


def test_product_view_explains_a_cell_without_an_equipment_model(tmp_path):
    from ensure_workcell_studio_web_scene_fresh_impl import missing_equipment_model
    configurable_input(tmp_path / 'older_cell')
    message = missing_equipment_model(tmp_path / 'older_cell')
    assert message.startswith('ur5 + onrobot_airpick_style (CONFIGURABLE) has no 3D equipment model')
    assert 'create it again with New Cell' in message and 'Generate and planning stay blocked' in message
    (tmp_path / 'legacy').mkdir()
    assert missing_equipment_model(tmp_path / 'legacy') is None      # other scenes keep the extractor's report
    assert missing_equipment_model(ROOT / 'scenes/ur5_airpick4_test') is None


def test_export_verifies_actual_reviewed_pair_not_just_saved_ready_label():
    from export_builder_scene_to_cell_definition import _require_reviewed_new_cell_equipment
    _, reviewed = load_profile('ur5_2f_workbench')
    valid = copy.deepcopy(reviewed)
    valid['workcell_studio'] = {
        'equipment': {'status': 'PLAN_CAPABLE', 'profile': 'ur5_2f_workbench',
                      'robot_capability': 'ur5', 'tool_capability': 'robotiq_2f_85'},
        'robot': {'capability': 'ur5', 'controller': 'ur5_arm_controller', 'readiness': 'PLAN_CAPABLE'},
        'tool': {'capability': 'robotiq_2f_85'},
    }
    _require_reviewed_new_cell_equipment(valid)

    # An authoring-only pair cannot become plan-capable by changing YAML's
    # status or forging the reviewed profile name.
    wrong_pair = copy.deepcopy(valid)
    wrong_pair['workcell_studio']['tool']['capability'] = 'onrobot_airpick_style'
    wrong_pair['workcell_studio']['equipment']['tool_capability'] = 'onrobot_airpick_style'
    with pytest.raises(ValueError, match='EQUIPMENT_NOT_PLAN_CAPABLE'):
        _require_reviewed_new_cell_equipment(wrong_pair)

    for section, path, value in [
        ('robot', ('srdf', 'package'), 'ur10_moveit_config'),
        ('robot', ('description', 'package'), 'another_robot_description'),
        ('robot', ('planning_group',), 'unsupported_group'),
        ('tool', ('model',), 'airpick'),
        ('tool', ('description', 'xacro'), 'urdf/airpick.xacro'),
        ('end_effector', ('capability',), 'onrobot_airpick_style'),
        ('end_effector', ('model',), 'airpick'),
        ('end_effector', ('description', 'xacro'), 'urdf/airpick.xacro'),
    ]:
        changed = copy.deepcopy(valid)
        field = changed[section]
        for key in path[:-1]:
            field = field[key]
        field[path[-1]] = value
        with pytest.raises(ValueError, match='EQUIPMENT_NOT_PLAN_CAPABLE'):
            _require_reviewed_new_cell_equipment(changed)

    removed = copy.deepcopy(valid)
    del removed['workcell_studio']['equipment']
    with pytest.raises(ValueError, match='EQUIPMENT_NOT_PLAN_CAPABLE'):
        _require_reviewed_new_cell_equipment(removed)

    # Older scenes without a new-wizard equipment binding retain their
    # established export/validation behaviour.
    _require_reviewed_new_cell_equipment({'robot': {'model': 'legacy'}})


def test_sorting_cell_has_two_bins_camera_and_camera_visible_test_objects(tmp_path):
    """New Cell > Sorting: the reviewed two-bin layout, the test objects the camera can classify, and a cycle task."""
    from simulator_world import derive_world
    layout = describe('ur5_2f_workbench', 'multi_bin_sorting_cell')['scenario']
    objects = [{'object_id': row['object_id'], 'appearance': row['appearance'],
                'pose': {'frame_id': 'world', 'xyz': [*row['xy'], 0.0], 'rpy': [0.0, 0.0, 0.0]}}
               for row in layout['test_objects']]
    scene, destination = tmp_path / 'staging' / 'sorting_cell', tmp_path / 'sorting_cell'
    wizard_input(scene, 'multi_bin_sorting_cell', objects)
    materialize(scene, 'ur5_2f_workbench', destination)
    env = yaml.safe_load((scene / 'environment.yaml').read_text())
    physical = env['environment']
    assert resolve_destination(physical, 'default_drop_zone')['target_pose_xyz'] == [0.58, 0.22, 0.1]
    bin_b = resolve_destination(physical, 'place_zone_b')
    assert bin_b['target_id'] == 'target_bin_b' and bin_b['target_pose_xyz'] == [0.25, 0.26, 0.1]
    assert env['camera']['pose'] == [0.44, -0.62, 0.17, 0.0, 0.40, 1.5707963267948966]
    rows = env['workcell_studio']['pick_zone']['object_source']['manual_observations']
    assert [(o['object_id'], o['class_id'], o['dimensions'], o['pose']['xyz'][2]) for o in rows] == [
        ('cup_1', 'unclassified', [0.025] * 3, 0.013), ('cup_2', 'unclassified', [0.025] * 3, 0.013),
        ('block_1', 'unclassified', [0.025] * 3, 0.013)]
    assert (scene / rows[0]['visual']['mesh']).is_file() and 'visual' not in rows[2]
    intent = json.loads((scene / 'config/workcell_builder_task_intent.yaml').read_text())
    assert intent['place']['target'] == {'asset_ref': 'target_bin_default', 'region_ref': 'default_drop_zone'}
    assert {r['id'] for r in yaml.safe_load((scene / 'layout/workcell_studio_layout.yaml').read_text())['items']} >= {
        'target_bin_b', 'place_zone_b'}
    _, report = derive_world(scene, physical_destinations='all')
    assert {d['asset_ref'] for d in report['destinations']} == {'target_bin_default', 'target_bin_b'}
    assert [o['object_id'] for o in report['objects']] == ['cup_1', 'cup_2', 'block_1']


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
    typed = yaml.safe_load((scene / 'environment.yaml').read_text())
    typed['scene'] = {'id': scene.name, 'display_name': 'Independent Cell', 'description': 'Typed in the wizard'}
    (scene / 'environment.yaml').write_text(yaml.safe_dump(typed))
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
    readiness = json.loads((destination / 'generated/scene_package_readiness.json').read_text())
    assert not readiness['blockers']
    assert all(record['exists'] for record in readiness['contract_files'])
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
    profile, reviewed = load_profile('ur5_2f_workbench')
    authored = yaml.safe_load((destination / 'environment.yaml').read_text())
    # Identical to the reviewed environment except the destination regions measured down to the bin floor.
    from instantiate_workcell_studio_profile import apply_destination_regions, restore_authored_bin_roles
    expected = copy.deepcopy(reviewed['environment'])
    restore_authored_bin_roles(expected)
    apply_destination_regions(expected, profile['destination_regions'])
    assert authored['environment'] == expected and expected != reviewed['environment']
    # The selected tool keeps its reviewed catalog capability (the qualified simulator binds its hold profile).
    assert authored['end_effector']['capability'] == reviewed['end_effector']['capability'] == 'robotiq_2f_85'
    assert authored['scene']['id'] == destination.name
    # The wizard's display name and description survive creation (Home lists the display name).
    assert authored['scene']['display_name'] == authored['scene']['name'] == 'Independent Cell'
    assert authored['scene']['description'] == 'Typed in the wizard'
    assert 'task' not in authored  # The reference scene task was not cloned.
    assert authored['perception']['epd_input']['topic']  # Keep reviewed input metadata when switching modes later.
    assert authored['perception']['enabled'] is False
    assert authored['perception']['mode'] == 'off'
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


def test_generated_cell_launch_plans_the_executors_certified_segments(tmp_path):
    """A Create New Cell launch loads the workcell request adapters the executor requires (as the canonical launch)
    and replaces the earlier generated template that lacked them; a hand-edited launch is still preserved."""
    import subprocess
    import generate_workcell_from_cell_definition as generator
    text = generator._render_physical_review_launch('cell_x', 'world')
    assert 'workcell/InitialSupportContact workcell/StraightCartesianPath' in text
    old = subprocess.check_output(['git', 'show', 'b452724a:workcell_builder/workcell_builder/templates/ros2/humble/'
                                   'physical_review.launch.py.in'], cwd=ROOT, text=True)
    scene = tmp_path / 'cell_x'
    (scene / 'urdf').mkdir(parents=True)
    (scene / 'launch').mkdir()
    (scene / 'urdf/scene.urdf.xacro').write_text('<robot/>')
    (scene / 'environment.yaml').write_text(yaml.safe_dump({'workcell_studio': {'recommended_profile': 'ur5_2f_workbench'}}))
    (scene / 'launch/demo.launch.py').write_text(old.replace('@PACKAGE@', repr('cell_x')).replace('@WORLD@', repr('world')))
    assert generator._profile_review_launch_owned(scene, 'cell_x', 'world')
    (scene / 'launch/demo.launch.py').write_text(text + '# edited\n')
    assert not generator._profile_review_launch_owned(scene, 'cell_x', 'world')
