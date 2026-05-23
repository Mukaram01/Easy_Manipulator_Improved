from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
VIEW_CPP = (ROOT/'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')

def test_scene_fit_uses_visible_items_bounds_and_has_diagnostics():
    assert 'scene_bounds_from_visible_items' in VIEW_CPP
    assert 'include_overlays' in VIEW_CPP
    assert 'Scene3D diagnostics {viewport_received_count=' in VIEW_CPP
