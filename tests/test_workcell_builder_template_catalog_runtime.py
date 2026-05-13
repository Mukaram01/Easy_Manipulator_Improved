from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()


def test_runtime_template_catalog_widget_and_minimum_templates():
    assert 'scenario_template_catalog' in CPP
    for item in [
        'Pick and Place Cell: UR5 + Robotiq 2F + table + cube + bin',
        'Conveyor Sorting - Live EPD Preview',
        'Camera Inspection Cell (PREVIEW ONLY)',
        'UR5 + Suction Pick Cell',
        'Placeholder Delta/Cartesian + Suction (PREVIEW ONLY)',
    ]:
        assert item in CPP
