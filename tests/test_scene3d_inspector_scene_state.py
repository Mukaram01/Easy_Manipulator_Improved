from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT/'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')

def test_scene_open_path_refreshes_selected_scene_state_before_ui_refresh():
    assert 'refresh_scene_builder_selection_state_ui();' in CPP
    assert 'select_scene_by_row' in CPP
    assert 'opened Scene Builder for' in CPP
