from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')

def test_inspector_scene_fields_exported():
    for token in ['inspector_scene_display_name', 'inspector_scene_path', 'inspector_scene_status']:
        assert token in CPP
