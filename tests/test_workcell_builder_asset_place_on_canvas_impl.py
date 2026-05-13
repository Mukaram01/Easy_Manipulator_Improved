from pathlib import Path
s=Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_asset_place_mode_markers():
    assert 'asset_placement_mode' in s
    assert 'Add to Layout' in s
    assert 'Place on Canvas' in s
