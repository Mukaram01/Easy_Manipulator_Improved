from pathlib import Path

SCRIPT = Path('scripts/audit_workcell_studio_regressions.py')
TEXT = SCRIPT.read_text(encoding='utf-8')
DOC = Path('docs/manuals/WORKCELL_STUDIO_NEW_CELL_FLOW.md').read_text(encoding='utf-8')


def test_regression_script_exists():
    assert SCRIPT.is_file()


def test_regression_references_required_audits_and_scratch_generator():
    for token in [
        'audit_new_cell_file_outputs.py',
        'audit_new_cell_state_transitions.py',
        'generate_scratch_cell_acceptance.py',
    ]:
        assert token in TEXT


def test_regression_includes_known_scene_names():
    assert 'ur5_2f_test' in TEXT
    assert 'ur5_airpick4_test' in TEXT


def test_regression_report_fields_present():
    for token in [
        'checked_scenes',
        'per_scene_results',
        'regression_status',
        'launch_command_by_scene',
    ]:
        assert token in TEXT


def test_fake_hardware_tokens_present_and_real_hardware_path_absent():
    assert 'use_fake_hardware:=true' in TEXT
    assert 'launch_rviz:=true' in TEXT
    assert 'use_fake_hardware:=false' not in TEXT


def test_run_smoke_opt_in_only():
    assert '--run-smoke' in TEXT


def test_docs_include_point_6_regression_audit():
    assert 'Point 6: Regression Audit' in DOC
