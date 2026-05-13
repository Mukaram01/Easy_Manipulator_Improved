from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')

def test_tool_default_tokens():
    for t in ['Robotiq defaults finger_top/open_gripper','suction defaults suction_top/vacuum_off','unknown tool safe defaults warning','gripper mount RPY remains -1.5708 -1.5708 0']:
        assert t in CPP
