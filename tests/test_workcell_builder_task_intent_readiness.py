from pathlib import Path

def test_ui_and_status_markers_present():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()
    assert 'Preview Task Intent' in cpp
    assert 'Generate Task Intent Preview' in cpp
    assert 'Open Task Intent Preview' in cpp
    assert 'preview_only' in cpp
