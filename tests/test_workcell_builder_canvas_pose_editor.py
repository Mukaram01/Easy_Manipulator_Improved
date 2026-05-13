from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()
UI = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text()


def test_tokens_present():
    required = ['name/id', 'type/category', 'x | y | z', 'roll | pitch | yaw', 'layout unsaved', 'metadata-only', 'rerun zone validation']
    # replaced per-file by sed
    for token in required:
        assert token in CPP or token in UI
