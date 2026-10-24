from pathlib import Path

import yaml

from test_task_intent_resolver import valid_intent, environment
from scripts.validate_builder_task_intent import validate

ROOT = Path(__file__).resolve().parents[1]


def scene(tmp_path):
    env = environment()
    env['task_zones'] = [z for z in env['task_zones'] if z['id'] != 'pick_zone_main']
    env['task_zones'].append({'id': 'pick_zone_main', 'type': 'pick_zone', 'frame': 'world', 'pose_xyz': [0.3, 0, 0.2], 'pose_rpy': [0,0,0], 'dimensions': [1,1,1]})
    env['end_effector'] = {'id': 'robotiq_85_gripper', 'type': 'finger'}
    (tmp_path / 'environment.yaml').write_text(yaml.safe_dump(env))
    (tmp_path / 'config').mkdir()
    return tmp_path


def test_exact_requested_pose_is_validated_without_rewriting(tmp_path):
    root = scene(tmp_path)
    intent = valid_intent('EXACT', 'EXACT')
    intent['place']['placement']['requested_local_pose']['xyz_m'][0] = 0.30
    path = root / 'config/workcell_builder_task_intent.yaml'
    path.write_text(yaml.safe_dump(intent))
    before = path.read_bytes()
    report = validate(path, root)
    assert report['status'] == 'FAIL'
    assert any('PLACE_LOCAL_POSE_OUTSIDE_REGION' in error for error in report['errors'])
    assert path.read_bytes() == before


def test_authoring_load_keeps_v2_semantics_and_hash(tmp_path):
    from scripts.task_intent_authoring import load_authoring
    from scripts.task_intent_v2 import normalized_intent_hash
    root = scene(tmp_path)
    intent = valid_intent('PREFERRED', 'EXACT')
    intent['pick']['selection']['object_filter']['min_confidence'] = None
    path = root / 'config/workcell_builder_task_intent.yaml'
    path.write_text(yaml.safe_dump(intent))
    before = path.read_bytes()
    report = load_authoring(root)
    assert report['task_intent'] == intent
    assert report['normalized_intent_sha256'] == normalized_intent_hash(intent)
    assert path.read_bytes() == before


def test_existing_wizard_draft_retains_bindings_without_writing(tmp_path):
    from scripts.task_intent_authoring import load_authoring
    env = {'robot': 'UR5', 'end_effector': 'robotiq_85', 'workcell_studio': {
        'scenario': {'id': 'static_table_pick_place'},
        'pick_zone': {'pick_object_source': 'source_bin_01', 'object_source': {
            'mode': 'manual_simulated', 'source_zone_ref': 'source_bin_01'}},
        'place_zone': {'target': 'place_fixture_01'},
        'task_intent': {'grasp_strategy': 'top_2f', 'approach_axis': 'z_down',
                        'approach_distance_m': 0.17, 'retreat_distance_m': 0.21},
    }}
    path = tmp_path / 'environment.yaml'
    path.write_text(yaml.safe_dump(env))
    before = path.read_bytes()
    report = load_authoring(tmp_path)
    model = report['task_intent']
    assert model['pick']['selection']['source_ref'] == 'source_bin_01'
    assert model['place']['target']['asset_ref'] == 'place_fixture_01'
    assert model['pick']['grasp']['approach']['distance_m'] == 0.17
    assert model['pick']['grasp']['lift']['distance_m'] == 0.21
    assert model['safety']['real_hardware_enabled'] is False
    assert report['normalized_intent_sha256']
    assert not (tmp_path / 'config/workcell_builder_task_intent.yaml').exists()
    assert path.read_bytes() == before


def test_legacy_v1_load_uses_existing_migration_and_never_writes(tmp_path):
    from scripts.task_intent_authoring import load_authoring
    from scripts.task_intent_v2 import migrate_v1
    root = scene(tmp_path)
    v1 = {'schema': 'workcell_builder_task_intent/v1', 'task': {'id': 'old', 'type': 'pick_place'},
          'pick': {'source': {'id': 'source', 'type': 'perception'},
                   'zone': {'id': 'pick_zone_main'}, 'object_filter': {'class_id': 'bottle'}},
          'grasp': {'strategy_ref': 'top_2f'}, 'place': {'target': {'id': 'drop'}}}
    path = root / 'config/workcell_builder_task_intent.yaml'
    path.write_text(yaml.safe_dump(v1))
    before = path.read_bytes()
    report = load_authoring(root)
    assert report['task_intent'] == migrate_v1(v1, yaml.safe_load((root / 'environment.yaml').read_text()))
    assert path.read_bytes() == before


def test_preferred_pose_reports_warning_but_structural_errors_block(tmp_path):
    root = scene(tmp_path)
    intent = valid_intent('AUTO', 'PREFERRED')
    intent['place']['placement']['requested_local_pose']['xyz_m'][0] = 0.30
    path = root / 'config/workcell_builder_task_intent.yaml'
    path.write_text(yaml.safe_dump(intent))
    report = validate(path, root)
    assert report['status'] == 'PASS'
    assert any('PLACE_LOCAL_POSE_OUTSIDE_REGION' in w for w in report['warnings'])
    intent['place']['target']['asset_ref'] = 'missing'
    path.write_text(yaml.safe_dump(intent))
    assert validate(path, root)['status'] == 'FAIL'


def test_tool_identity_outside_environment_geometry_wrapper_is_preserved(tmp_path):
    from scripts.task_intent_authoring import load_authoring
    root = scene(tmp_path)
    env = yaml.safe_load((root / 'environment.yaml').read_text())
    tool = env.pop('end_effector')
    (root / 'environment.yaml').write_text(yaml.safe_dump({'end_effector': tool, 'environment': env}))
    (root / 'config/workcell_builder_task_intent.yaml').write_text(yaml.safe_dump(valid_intent()))
    report = load_authoring(root)
    assert report['strategies'] == ['top_2f', 'side_grip_basic', 'finger_pinch_basic']
    assert report['status'] == 'PASS'


def test_wizard_perception_modes_use_supported_normalized_source(tmp_path):
    from scripts.task_intent_authoring import _wizard_intent
    for mode, expected in [('recorded_perception', 'replayed_snapshot'), ('live_epd_realsense', 'live_epd')]:
        env = {'workcell_studio': {'scenario': {'id': 'static_table_pick_place'},
            'pick_zone': {'object_source': {'mode': mode, 'source_zone_ref': 'pick', 'required_class': 'cup'}},
            'place_zone': {'target': 'bin'}, 'task_intent': {'grasp_strategy': 'auto'}},
            'task_zones': [{'id': 'pick', 'type': 'pick_zone'}, {'id': 'drop', 'target_ref': 'bin'}]}
        intent = _wizard_intent(tmp_path, env)
        assert intent['pick']['selection']['source_type'] == 'perception'
        assert intent['pick']['selection']['source_ref'] == 'detected_objects/v1'
        assert intent['pick']['selection']['zone_ref'] == 'pick'
        assert intent['perception']['mode'] == expected


def test_explicit_task_input_has_no_hidden_replay_fallback(tmp_path):
    from scripts.task_intent_authoring import planning_input
    import pytest
    scene(tmp_path)
    intent = valid_intent()
    intent['perception'] = {'mode': 'replayed_snapshot', 'replay_path': ''}
    with pytest.raises(ValueError, match='TASK_INPUT_UNAVAILABLE'):
        planning_input(tmp_path, intent)
    source = tmp_path / 'recording.yaml'
    source.write_text((ROOT / 'config/runtime/r1_4_replay.yaml').read_text())
    intent['perception']['replay_path'] = 'recording.yaml'
    assert planning_input(tmp_path, intent) == (source, True)
    intent['perception']['mode'] = 'live_epd'
    with pytest.raises(ValueError, match='LIVE_INPUT_REQUIRED'):
        planning_input(tmp_path, intent)
    intent['perception']['mode'] = 'off'
    intent['pick']['selection'].update(source_type='manual_simulated', source_ref='pick_zone_main')
    assert planning_input(tmp_path, intent) == (source, True)
    source.unlink()
    with pytest.raises(ValueError, match='TASK_INPUT_UNAVAILABLE'):
        planning_input(tmp_path, intent)


def test_invalid_perception_mode_and_source_mismatch_block(tmp_path):
    root = scene(tmp_path)
    intent = valid_intent()
    intent['perception'] = {'mode': 'unknown'}
    from scripts.validate_builder_task_intent import validate_payload
    assert 'PERCEPTION_MODE_UNSUPPORTED' in str(validate_payload(intent, root)['errors'])
    intent['perception']['mode'] = 'off'
    assert 'PICK_SOURCE_MODE_MISMATCH' in str(validate_payload(intent, root)['errors'])


def test_snapshot_without_eligible_workpiece_blocks_before_launch(tmp_path):
    from scripts.task_intent_authoring import planning_input
    import pytest
    root = scene(tmp_path)
    intent = valid_intent()
    intent['perception'] = {'mode': 'replayed_snapshot', 'replay_path': 'observations.yaml'}
    snapshot = yaml.safe_load((ROOT / 'config/runtime/r1_4_replay.yaml').read_text())
    intent['pick']['selection']['object_filter']['class_id'] = 'not-recorded'
    (root / 'observations.yaml').write_text(yaml.safe_dump(snapshot))
    with pytest.raises(ValueError, match='OBSERVATION_UNAVAILABLE'):
        planning_input(root, intent)
    intent['pick']['selection']['object_filter']['class_id'] = 'bottle'
    for obj in snapshot['objects']:
        obj['age_seconds'] = 30
    (root / 'observations.yaml').write_text(yaml.safe_dump(snapshot))
    with pytest.raises(ValueError, match='OBSERVATION_UNAVAILABLE'):
        planning_input(root, intent)
    snapshot['source'] = []
    (root / 'observations.yaml').write_text(yaml.safe_dump(snapshot))
    with pytest.raises(ValueError, match='TASK_INPUT_INVALID'):
        planning_input(root, intent)
    snapshot['source'] = {'mode': 'replayed_snapshot'}
    snapshot['objects'] = [None]
    (root / 'observations.yaml').write_text(yaml.safe_dump(snapshot))
    with pytest.raises(ValueError, match='TASK_INPUT_INVALID'):
        planning_input(root, intent)


def test_direct_resolved_task_does_not_require_invented_fallback_routes():
    from scripts.generate_workcell_from_cell_definition import _normalize_task_recipe
    warnings = []
    direct = {'task_type': 'pick_place', 'enabled': True,
              'decision_rules': [{'when': {'always': True}, 'destination': 'drop'}],
              'destinations': [{'id': 'drop'}], 'expected': {'allow_fallback_rule': False}}
    assert _normalize_task_recipe(direct, warnings) == direct
    assert warnings == []
    warnings = []
    _normalize_task_recipe({'task_type': 'pick_place', 'enabled': False}, warnings)
    assert warnings == []


def test_refresh_publishes_current_task_identity_without_replacing_curated_metadata():
    from scripts.generate_workcell_from_cell_definition import _updated_task_manifest
    existing = {'robot': {'custom': 'reviewed'}, 'operator_note': 'retain',
                'task_recipe': {'enabled': False}, 'grasp_strategy': {'strategy_ref': 'old'},
                'task_intent_resolution': {'normalized_intent_sha256': 'old', 'resolution_sha256': 'old'}}
    generated = {'robot': {'custom': 'generic scaffold'}, 'task_recipe': {'enabled': True},
                 'task_intent_resolution': {'normalized_intent_sha256': 'current-task', 'resolution_sha256': 'current-result'}}
    refreshed = _updated_task_manifest(existing, generated)
    assert refreshed['robot'] == existing['robot']
    assert refreshed['operator_note'] == 'retain'
    assert refreshed['task_recipe']['enabled'] is True
    assert refreshed['task_intent_resolution'] == generated['task_intent_resolution']
    assert refreshed['normalized_intent_sha256'] == 'current-task'
    assert refreshed['resolution_sha256'] == 'current-result'
    assert 'grasp_strategy' not in refreshed
    assert existing['task_recipe']['enabled'] is False


def test_copied_install_validates_using_installed_strategy_catalog(tmp_path):
    """Ordinary installs must validate the same task as source/symlink installs."""
    import json
    import os
    import shutil
    import subprocess
    import sys

    root = scene(tmp_path)
    task = root / 'config/workcell_builder_task_intent.yaml'
    task.write_text(yaml.safe_dump(valid_intent('EXACT', 'EXACT')))
    prefix = tmp_path / 'install/workcell_builder'
    installed = prefix / 'lib/workcell_builder'
    installed.mkdir(parents=True)
    for name in ('validate_builder_task_intent.py', 'task_intent_v2.py', 'capability_registry.py',
                 'workcell_studio_layout_source.py', 'physical_destination.py'):
        shutil.copy2(ROOT / 'scripts' / name, installed / name)
    shutil.copytree(ROOT / 'catalog/grasp_strategies',
                    prefix / 'share/workcell_builder/catalog/grasp_strategies')
    env = dict(os.environ)
    env.pop('PYTHONPATH', None)
    result = subprocess.run([sys.executable, str(installed / 'validate_builder_task_intent.py'),
                             str(task), '--scene-package', str(root), '--json'],
                            cwd=tmp_path, env=env, text=True, capture_output=True)
    report = json.loads(result.stdout)
    assert result.returncode == 0, report
    assert report['status'] == 'PASS'
