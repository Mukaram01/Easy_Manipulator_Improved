from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_template_labels_and_preview_markers_present():
    for token in [
        'Pick and Place Cell: UR5 + Robotiq 2F + table + cube + bin',
        'Conveyor Sorting - Live EPD Preview',
        'Camera Inspection Cell (PREVIEW ONLY)',
        'UR5 + Suction Pick Cell',
        'Placeholder Delta/Cartesian + Suction (PREVIEW ONLY)',
        'preview_only',
    ]:
        assert token in CPP
