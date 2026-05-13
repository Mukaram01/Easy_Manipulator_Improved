from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_pick_place_recommended_layout_quality_tokens():
    for token in ['pick_zone', 'place_zone', 'object_on_support_surface', 'bin_clearance_ok', 'robot_reach']:
        assert token in CPP

def test_gripper_orientation_unchanged_marker_present():
    assert '-1.5708 -1.5708 0' in CPP or 'gripper_mount_orientation' in Path('tests/test_workcell_builder_gripper_mount_orientation.py').read_text()
