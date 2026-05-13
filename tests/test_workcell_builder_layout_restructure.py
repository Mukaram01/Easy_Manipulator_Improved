from pathlib import Path

UI = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text()
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()


def test_workflow_pages_exist():
    for token in ['Start', 'Templates', 'Assets', 'Scenes', 'Layout', 'Task', 'Perception', 'Grasp', 'Validate &amp; Generate']:
        assert token in UI


def test_home_not_overloaded_and_catalogs_moved():
    assert 'name="templates_tab"' in UI
    assert 'name="assets_tab"' in UI
    assert 'name="scenes_tab"' in UI
    assert 'scene_catalog_table' in UI


def test_fullscreen_and_window_controls_tokens_exist():
    assert 'Qt::WindowMinimizeButtonHint' in CPP
    assert 'Qt::WindowMaximizeButtonHint' in CPP
    assert 'Qt::WindowCloseButtonHint' in CPP
    assert 'Qt::Key_F11' in CPP
    assert 'Qt::Key_Escape' in CPP
