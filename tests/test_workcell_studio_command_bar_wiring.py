from pathlib import Path

MAIN_CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_command_bar_labels_exist():
    for label in ['New Cell', 'Open Scene', 'Validate', 'Demo Mode', 'Preview Launch', 'Generate Scene', 'Export']:
        assert label in MAIN_CPP


def test_generate_scene_uses_layout_merge_path():
    assert 'run_layout_merge_for_selected_scene(true);' in MAIN_CPP


def test_command_bar_logs_no_motion_for_sensitive_actions():
    assert "Validate: offline validation" in MAIN_CPP
    assert "Demo Mode: switched" in MAIN_CPP
    assert "Preview Launch: prepared fake-hardware commands" in MAIN_CPP
    assert 'No robot motion commanded' in MAIN_CPP
