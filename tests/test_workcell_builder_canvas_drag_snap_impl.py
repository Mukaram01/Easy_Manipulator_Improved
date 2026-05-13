from pathlib import Path
s=Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_drag_snap_impl_markers():
    assert 'ItemIsMovable' in s
    assert 'snap_to_grid_enabled' in s
    assert 'Shift' in s or 'fine movement' in s
    assert 'no robot motion' in s
