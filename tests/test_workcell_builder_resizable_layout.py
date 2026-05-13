from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()
UTILS = Path('workcell_builder/workcell_builder/gui/workcell_builder_ui_utils.cpp').read_text()


def test_scene_select_has_resizable_default_and_minimum_size():
    assert 'setMinimumSize(1100, 720)' in CPP
    assert 'resize(1450, 900)' in CPP


def test_no_hard_dialog_maximum_cap():
    assert 'setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX)' in UTILS
