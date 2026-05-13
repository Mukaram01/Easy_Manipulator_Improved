from pathlib import Path
UI = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')

def test_sorting_preview_tokens():
    for t in ['Class Routing (sorting preview)','class_route_target','EPD remains external/separate','runtime_execution_enabled: false']:
        assert t in UI or t in CPP
