from pathlib import Path
s=Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_alignment_autofix_helpers_markers():
    for t in ['Align selected to table centre', 'Auto-fix invalid placement', 'Duplicate selected item', 'Delete selected item with confirmation']:
        assert t in s
    assert 'auto_fix_pick_place_zones_layout_yaml' in s
