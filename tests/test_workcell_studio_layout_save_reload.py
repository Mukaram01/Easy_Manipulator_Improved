from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/src_workcell_studio_layout_editor.cpp').read_text(encoding='utf-8')
MODEL = Path('workcell_builder/workcell_builder/src_workcell_studio_canvas_model.cpp').read_text(encoding='utf-8')

def test_layout_save_reload_markers_present():
    assert 'workcell_studio_layout/v1' in CPP
    assert 'saved_at_utc' in CPP
    assert 'layout/workcell_studio_layout.yaml' in MODEL
