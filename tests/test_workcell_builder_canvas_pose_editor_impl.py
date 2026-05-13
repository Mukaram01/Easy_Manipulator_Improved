from pathlib import Path
cpp=Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_pose_editor_behaviour_markers():
    assert 'layout_unsaved=true' in cpp
    assert 'rerun zone validation' in cpp
    assert 'x=%1 y=%2' in cpp
