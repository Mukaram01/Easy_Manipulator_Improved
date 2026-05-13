from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()
UI = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text()


def test_tokens_present():
    required = ['Fit Cell', 'Fit Selection', 'Reset View', 'Zoom In', 'Zoom Out', 'Zoom 100%', 'mouse wheel zoom', 'right-drag pan']
    # replaced per-file by sed
    for token in required:
        assert token in CPP or token in UI
