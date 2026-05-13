from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()


def test_minimum_templates_listed():
    required = [
        'Pick and Place Cell: UR5 + Robotiq 2F + table + cube + bin',
        'Conveyor Sorting - Live EPD Preview',
        'Camera Inspection Cell (PREVIEW ONLY)',
        'UR5 + Suction Pick Cell',
        'Placeholder Delta/Cartesian + Suction (PREVIEW ONLY)',
    ]
    for item in required:
        assert item in CPP


def test_preview_only_templates_have_safety_tokens():
    assert 'runtime_execution_enabled: false' in CPP
    assert 'fake_hardware_first: true' in CPP
    assert 'motion_command_sent: false' in CPP
