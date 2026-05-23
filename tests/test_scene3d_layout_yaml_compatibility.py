from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/src_workcell_studio_canvas_model.cpp').read_text()

def test_layout_accepts_legacy_name_items():
    assert 'items[].name' in CPP
    assert 'schema_legacy' in CPP
