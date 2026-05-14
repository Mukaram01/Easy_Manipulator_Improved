from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_tool_defaults_logic_present():
    for tok in ['apply_tool_defaults','suction_top','vacuum_off','finger_top','open_gripper','Unsaved task edits present; reset skipped']:
        assert tok in CPP
