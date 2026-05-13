from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()
UI = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text()


def test_tokens_present():
    required = ['Align selected to table centre', 'Move pick object onto support surface', 'Place bin inside reachable area', 'Centre camera ROI on pick zone', 'Auto-space pick/place zones', 'Auto-fix invalid placement', 'Duplicate selected item', 'Delete selected item with confirmation']
    # replaced per-file by sed
    for token in required:
        assert token in CPP or token in UI
