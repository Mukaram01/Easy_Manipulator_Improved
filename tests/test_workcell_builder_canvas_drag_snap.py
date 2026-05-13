from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()
UI = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text()


def test_tokens_present():
    required = ['snap to grid', 'hold Shift for fine movement', 'drag ghost preview', 'live x/y coordinates', 'undo last move', 'no robot motion', 'unlock robot base']
    # replaced per-file by sed
    for token in required:
        assert token in CPP or token in UI
