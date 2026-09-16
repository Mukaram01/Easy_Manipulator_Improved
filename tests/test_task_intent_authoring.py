from pathlib import Path

import yaml

from test_task_intent_resolver import valid_intent, environment
from scripts.validate_builder_task_intent import validate

ROOT = Path(__file__).resolve().parents[1]


def scene(tmp_path):
    env = environment()
    env['task_zones'].append({'id': 'pick_zone_main', 'type': 'pick_zone'})
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
