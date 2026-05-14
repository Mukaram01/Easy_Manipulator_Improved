from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_canvas_assignment_impl():
    for tok in ['assign_selected_canvas_item','No canvas selection','use_selected_item_as_pick_source_button','use_selected_zone_as_place_zone_button']:
        assert tok in CPP
