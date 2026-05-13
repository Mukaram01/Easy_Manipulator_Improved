from pathlib import Path
cpp=Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_save_roundtrip_impl_markers():
    for t in ['GenerateYAML::generate_yaml', 'load_scene_from_yaml', 'layout_preview.json', 'fake_hardware_first', 'no_runtime_motion']:
        assert t in cpp
