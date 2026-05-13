from pathlib import Path
H = Path('workcell_builder/workcell_builder/gui/scene_select.h').read_text()
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_slot_wiring_names():
    assert 'on_open_conveyor_sorting_run_console_button_clicked' in H
    assert 'on_open_conveyor_sorting_run_console_button_clicked' in CPP
    assert 'on_refresh_preview_clicked' not in H
    assert 'on_export_preview_clicked' not in H
