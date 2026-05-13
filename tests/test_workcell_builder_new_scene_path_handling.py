from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_scene_root_resolution_sources_present():
    for token in ['WORKCELL_BUILDER_SCENE_ROOT', '--scene-root', 'select_scene_root']:
        assert token in CPP

def test_open_scene_folder_handler_exists_once():
    assert CPP.count('on_open_scene_folder_clicked') >= 1
