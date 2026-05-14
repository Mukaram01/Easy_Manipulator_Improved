from pathlib import Path

MAIN_CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
ACC_PY = Path('scripts/validate_workcell_studio_generated_scene.py').read_text(encoding='utf-8')

def test_acceptance_layout_merge_auto_or_warn_flow_present():
    assert 'Acceptance: running safe offline layout merge first' in MAIN_CPP
    for key in ['layout_applied', 'generated_from_saved_layout', 'merge_report_path', 'layout_stale', 'merge_warnings', 'merge_blockers']:
        assert key in ACC_PY
