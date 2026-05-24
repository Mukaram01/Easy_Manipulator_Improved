from pathlib import Path
import importlib.util


def test_scratch_cell_template_includes_safe_joint_state_key() -> None:
    path = Path('scripts/generate_scratch_cell_acceptance.py')
    spec = importlib.util.spec_from_file_location('gsca', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(mod)
    tmpl = mod.CELL_TMPL
    assert 'robot:' in tmpl
    assert 'safe_joint_state: []' in tmpl
