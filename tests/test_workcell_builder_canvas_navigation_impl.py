from pathlib import Path
s=Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_navigation_impl_tokens():
    for t in ['wheelEvent', 'fitInView', 'resetTransform', 'ScrollHandDrag', 'scale(']:
        assert t in s
