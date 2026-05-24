from pathlib import Path


def test_scratch_cell_template_includes_safe_joint_state_key() -> None:
    text = Path('scripts/generate_scratch_cell_acceptance.py').read_text(encoding='utf-8')
    assert 'CELL_TMPL' in text
    assert 'schema_version: cell_definition/v1' in text
    assert 'safe_joint_state: []' in text
    assert 'commissioning:' in text
    assert 'self_test_enabled: true' in text
    assert 'export_bundle: true' in text
    assert 'require_operator_review: true' in text
    assert 'fake_hardware_default: true' in text
