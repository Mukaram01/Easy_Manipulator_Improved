from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_recommended_layout_uses_canonical_path():
    assert 'on_use_recommended_layout_clicked' in CPP
    assert 'apply_recommended_layout_to_scene(scene_dir' in CPP
