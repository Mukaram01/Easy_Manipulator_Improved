from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
MODEL = Path('workcell_builder/workcell_builder/src_workcell_studio_canvas_model.cpp').read_text(encoding='utf-8')

def test_canvas_drag_drop_tokens_present():
    for token in ['Snap to Grid', 'Fine Move Mode', 'Unlock Robot Base', 'Duplicate Selected', 'Delete Selected']:
        assert token in CPP
    assert 'layout/workcell_studio_layout.yaml' in MODEL
    assert 'Malformed layout/workcell_studio_layout.yaml; falling back safely' in MODEL
