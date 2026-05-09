from pathlib import Path
from scripts import workcell_builder_gui_workflow as wf


def test_generation_buttons_exist_and_disabled_until_prereqs(tmp_path):
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')
    assert 'Generate YAML files for scene' in ui
    assert 'Generate files from YAML' in ui
    empty_state = {'scene_name': '', 'selected': {}, 'current_cell_assets': []}
    gate = wf.generation_prerequisites(empty_state, tmp_path / 'scene')
    assert gate['generate_yaml_enabled'] is False
    assert 'Disabled:' in gate['generate_yaml_tooltip']
    assert gate['generate_files_enabled'] is False
    assert 'environment.yaml missing' in gate['generate_files_tooltip']
