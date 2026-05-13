from pathlib import Path

def test_layout_preview_export_markers_present():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    for token in ['layout_preview.svg','layout_preview.html','layout_preview.json','fake_hardware_first','no_runtime_motion','preview_items']:
        assert token in cpp
