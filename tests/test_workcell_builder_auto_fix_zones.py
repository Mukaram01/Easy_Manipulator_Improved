from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_auto_fix_action_exists_and_marks_unsaved():
    for token in ['auto_fix_pick_place_zones_layout_yaml', 'Auto-fix Pick/Place Zones', 'scene_marked_unsaved=true']:
        assert token in CPP

def test_no_robot_motion_command_tokens():
    assert 'robot_motion_commanded=false' in CPP
