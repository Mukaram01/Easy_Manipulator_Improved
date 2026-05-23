from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text()

def test_inspector_scene_state_includes_scene_robot_tool_launch():
    for token in ['Scene path:', 'Scene status:', 'Robot:', 'End effector:', 'Launch status:']:
        assert token in CPP
