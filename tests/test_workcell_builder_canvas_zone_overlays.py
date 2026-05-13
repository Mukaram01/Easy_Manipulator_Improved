from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_canvas_zone_overlay_markers_present():
    for token in ['pick_zone','place_zone','camera_roi','reach_warning']:
        assert token in CPP

def test_canvas_click_updates_inspector_token():
    assert 'zone_inspector_token' in CPP
    assert 'selectionChanged' in CPP

def test_toggles_affect_zone_overlays():
    for token in ['toggle_roi_action','toggle_reach_action','fitInView']:
        assert token in CPP
