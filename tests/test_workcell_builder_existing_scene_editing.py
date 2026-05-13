from pathlib import Path

GUI_CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
GUI_UI = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')


def test_open_edit_cell_action_wired_and_scene_loading_paths_exist():
    assert 'void SceneSelect::on_edit_scene_clicked()' in GUI_CPP
    assert 'Could not load scene from environment.yaml.' in GUI_CPP
    assert 'No scene selected to edit.' in GUI_CPP


def test_missing_and_invalid_yaml_paths_are_explicit_not_silent():
    assert 'environment.yaml missing' in GUI_CPP
    assert 'YAML parse failure' in GUI_CPP
    assert 'on_repair_scene_yaml_clicked' in GUI_CPP


def test_scene_page_exposes_existing_scene_management_controls():
    for token in ['Generate Full Scene Package', 'Delete Cell', 'current_cell_assets_table', 'visual_layout_canvas']:
        assert token in GUI_UI
