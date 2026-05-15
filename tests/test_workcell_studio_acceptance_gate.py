from pathlib import Path

SCRIPT = Path('scripts/run_workcell_studio_acceptance_gate.py')
TEXT = SCRIPT.read_text(encoding='utf-8')
DOC = Path('docs/manuals/WORKCELL_STUDIO_ACCEPTANCE_GATE.md').read_text(encoding='utf-8')
CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_master_script_exists_and_references_all_six_audits():
    assert SCRIPT.is_file()
    for token in [
        'generate_scratch_cell_acceptance.py',
        'audit_new_cell_file_outputs.py',
        'audit_new_cell_state_transitions.py',
        'audit_new_cell_error_messages.py',
        'smoke_test_scratch_cell_workspace.py',
        'audit_workcell_studio_regressions.py',
    ]:
        assert token in TEXT


def test_json_and_summary_fields_exist():
    for token in [
        'overall_status', 'audit_results', 'blockers', 'warnings', 'next_recommended_action',
        'build_command', 'source_command', 'launch_command', 'report_paths', 'logs_tail',
        'acceptance_summary.md', 'PASS', 'WARNINGS', 'BLOCKED'
    ]:
        assert token in TEXT


def test_modes_and_smoke_option_exist():
    for token in ['--mode', 'scratch', 'existing-scene', 'regression', '--run-smoke', '--timeout-sec']:
        assert token in TEXT


def test_fake_hardware_defaults_and_no_real_hardware_execution_path():
    assert 'use_fake_hardware:=true' in TEXT
    assert 'launch_rviz:=true' in TEXT
    assert 'use_fake_hardware:=false' in TEXT  # guard check in script


def test_docs_include_unified_acceptance_gate_section():
    assert 'Unified Workcell Studio Acceptance Gate' in DOC
    assert 'acceptance_summary.md' in DOC


def test_ui_mentions_acceptance_gate_hint_and_command():
    assert 'Full Workcell Studio acceptance gate available' in CPP
    assert 'run_workcell_studio_acceptance_gate.py --mode scratch' in CPP
