from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()
UI = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text()


def test_tokens_present():
    required = ['Grid', 'Reach', 'Objects', 'Bins', 'Conveyors', 'Cameras', 'Pick/place zones', 'Camera ROI/FOV', 'Warnings/blockers', 'Labels', 'legend']
    # replaced per-file by sed
    for token in required:
        assert token in CPP or token in UI
