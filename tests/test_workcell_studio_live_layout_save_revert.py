from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text()

def test_save_layout_serializes_live_canvas_state():
    for needle in ['schema_version: workcell_studio_layout/v1', 'fake_hardware_first: true', 'runtime_execution_enabled: false', 'motion_command_sent: false', 'persist_workcell_studio_layout']:
        assert needle in CPP

def test_revert_rebuilds_canvas():
    assert 'rebuild_digital_twin_canvas();' in CPP
