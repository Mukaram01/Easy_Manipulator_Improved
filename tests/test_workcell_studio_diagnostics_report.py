from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')

def test_diagnostics_report_filenames_and_safety_marker_present():
    assert 'workcell_studio_diagnostics_report.json' in CPP
    assert 'workcell_studio_diagnostics_summary.txt' in CPP
    assert 'workcell_studio_diagnostics_dashboard.html' in CPP
    assert 'no_robot_motion_commanded' in CPP
