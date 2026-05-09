from pathlib import Path
from scripts import workcell_builder_gui_workflow as wf


def test_cell_name_and_output_folder_controls_and_backend_mapping(tmp_path):
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')
    assert 'name="cell_name"' in ui
    assert 'name="output_folder"' in ui
    assert 'name="browse_output_folder"' in ui
    valid = wf.validate_cell_name('new_scene')
    invalid = wf.validate_cell_name('scene-name')
    assert valid['ok']
    assert not invalid['ok']
    created = wf.create_new_cell('new_scene', tmp_path)
    assert created['ok']
    assert (Path(created['scene_dir']) / 'environment.yaml').exists()


def test_status_message_contains_paths_and_next_action(tmp_path):
    result = wf.create_golden_demo_cell('golden_cell', tmp_path)
    assert result['ok']
    assert str(tmp_path) in result['status']
    assert 'Next:' in result['status']
