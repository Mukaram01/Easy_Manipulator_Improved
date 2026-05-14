from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')

def test_diagnostics_checks_helper_scripts_and_golden_flow_command():
    for needle in [
        'run_workcell_studio_golden_flow.py',
        'workcell_studio_layout_merge.py',
        'validate_workcell_studio_generated_scene.py',
        'workcell_studio_demo_mode.py',
        'workcell_studio_preview_launch.py',
        '--scene-dir /tmp/workcell_studio_diag_scene --json',
    ]:
        assert needle in CPP
