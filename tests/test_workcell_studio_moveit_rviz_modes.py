from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_plan_simulate_labels_and_actions_exist():
    for token in [
        'Plan & Simulate',
        'Open RViz2 / MoveIt',
        'Run Fake-Hardware Simulation',
        'Copy Launch Command',
        'Mode: Plan / Simulate',
    ]:
        assert token in CPP


def test_launch_command_is_fake_hardware_rviz_first():
    for token in ['ros2 launch', 'demo.launch.py', 'use_fake_hardware:=true', 'launch_rviz:=true']:
        assert token in CPP


def test_hardware_guardrails_and_qprocess_wiring():
    assert 'Real robot motion: locked' in CPP
    assert 'Simulation motion: allowed with fake hardware' in CPP
    assert 'Real hardware execution requires explicit guarded setup and is not launched from this mode.' in CPP
    assert 'Run Real Hardware' not in CPP
    assert 'preview_process_->start(' in CPP
