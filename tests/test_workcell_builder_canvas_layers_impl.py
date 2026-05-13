from pathlib import Path
s=Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_layer_state_impl_exists():
    assert 'layer_visibility' in s
    for t in ['Grid', 'Robot', 'Reach', 'Tables', 'Objects', 'Bins', 'Conveyors', 'Cameras', 'Pick/place zones', 'Camera ROI/FOV', 'Warnings/blockers', 'Labels']:
        assert t in s
