from pathlib import Path


def test_new_cell_flow_tokens_present():
    text = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()
    for token in [
        'Pick and Place Cell',
        'Conveyor Sorting Cell',
        'Camera Inspection Cell',
        'Machine Tending Placeholder',
        'Bin Picking Placeholder',
        'Palletizing Placeholder',
        'on_use_recommended_layout_clicked',
        'create_scene_from_template',
    ]:
        assert token in text
