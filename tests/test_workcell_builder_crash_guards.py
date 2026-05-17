from pathlib import Path
import importlib.util

ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / 'scripts' / 'workcell_builder_gui_workflow.py'
spec = importlib.util.spec_from_file_location('workflow', MODULE_PATH)
workflow = importlib.util.module_from_spec(spec)
spec.loader.exec_module(workflow)


def test_missing_perception_normalizes_safely():
    out = workflow.normalize_perception_config_from_environment({})
    assert out['status'] == 'MISSING_PERCEPTION'
    assert out['enabled'] is False


def test_scalar_perception_normalizes_to_disabled():
    for token in ['disabled', 'none', 'false']:
        out = workflow.normalize_perception_config_from_environment({'perception': token})
        assert out['status'] == 'PERCEPTION_DISABLED'
        assert out['enabled'] is False


def test_empty_perception_mapping_guarded():
    out = workflow.normalize_perception_config_from_environment({'perception': {}})
    assert out['status'] == 'PERCEPTION_EMPTY_CONFIG'


def test_partial_or_malformed_legacy_environment_yaml_is_guarded():
    malformed = workflow.parse_environment_yaml_safely('robot: [bad')
    assert malformed['ok'] is False
    assert malformed['environment'] == {}

    partial = workflow.parse_environment_yaml_safely('robot: {name: ur5}\nend_effector: {name: rg2}\n')
    assert partial['ok'] is True
    normalized = workflow.normalize_perception_config_from_environment(partial['environment'])
    assert normalized['status'] == 'MISSING_PERCEPTION'


def test_command_generation_refuses_when_required_args_absent_for_targeted_scripts(tmp_path):
    epd = workflow.build_epd_snapshot_adapter_command(profile=None, input_snapshot=tmp_path/'snap.yaml', output_payload=tmp_path/'out.json')
    assert epd['ok'] is False
    assert 'missing required args' in epd['error']

    bridge = workflow.build_perception_bridge_preview_command(
        perception_profile=tmp_path/'profile.yaml',
        detected_objects=None,
        task_intent=tmp_path/'intent.yaml',
        output_payload=tmp_path/'payload.json',
        output_report=tmp_path/'report.json',
    )
    assert bridge['ok'] is False
    assert 'missing required args' in bridge['error']
