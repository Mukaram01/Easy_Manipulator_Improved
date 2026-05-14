from pathlib import Path
HPP = Path('workcell_builder/workcell_builder/gui/scene_select.h').read_text()
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_sorting_route_metadata_exists():
    assert 'class_routing' in HPP
    for tok in ['EPD remains external/separate','runtime_execution_enabled: false']:
        assert tok in CPP
