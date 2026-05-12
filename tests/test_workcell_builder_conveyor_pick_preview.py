from pathlib import Path

def test_ui_static_markers():
    txt = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()
    assert 'Preview Conveyor Pick Flow' in txt
    assert 'time_to_pick_s' in txt
    assert 'preview_only' in txt
