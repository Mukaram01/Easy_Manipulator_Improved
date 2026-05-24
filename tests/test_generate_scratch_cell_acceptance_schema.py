from pathlib import Path


def test_scratch_cell_template_includes_safe_joint_state_key() -> None:
    text = Path('scripts/generate_scratch_cell_acceptance.py').read_text(encoding='utf-8')
    assert 'CELL_TMPL' in text
    assert 'schema_version: cell_definition/v1' in text
    assert 'safe_joint_state: []' in text
