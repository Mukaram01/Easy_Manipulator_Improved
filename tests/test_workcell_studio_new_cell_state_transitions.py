from pathlib import Path

MAIN_CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
SCRIPT = Path('scripts/audit_new_cell_state_transitions.py').read_text(encoding='utf-8')
DOC = Path('docs/manuals/WORKCELL_STUDIO_NEW_CELL_FROM_SCRATCH.md').read_text(encoding='utf-8')


def test_state_names_exist_in_ui_and_script():
    for token in [
        'NO_WORKSPACE','WORKSPACE_READY','CELL_DRAFT_CREATED','LAYOUT_CREATED','LAYOUT_SAVED',
        'TASK_INTENT_CREATED','SCENE_PACKAGE_GENERATED','FILE_OUTPUTS_CHECKED','VALIDATION_READY',
        'VALIDATION_PASSED','VALIDATION_BLOCKED','PLAN_SIMULATE_READY','SIMULATION_RUNNING','SIMULATION_STOPPED'
    ]:
        assert token in MAIN_CPP or token in SCRIPT


def test_state_conditions_reference_real_files():
    for token in ['layout/workcell_studio_layout.yaml','config/workcell_builder_task_intent.yaml','package.xml','CMakeLists.txt','launch/demo.launch.py']:
        assert token in MAIN_CPP
        assert token in SCRIPT


def test_next_actions_exist():
    for token in ['Save Layout','Generate/Update Task Intent','Generate Scene Package','Run Offline Validation','Open Plan & Simulate']:
        assert token in MAIN_CPP or token in SCRIPT


def test_refresh_called_after_key_actions():
    for token in ['run_offline_validation() {', 'run_fake_hardware_preview(){', 'stop_preview_process(){', 'handle_preview_finished(int exit_code']:
        assert token in MAIN_CPP
    assert MAIN_CPP.count('refresh_new_cell_checklist();') >= 4


def test_json_report_fields_exist():
    for token in ['scene_name','scene_dir','current_state','completed_states','pending_states','blocked_states','next_recommended_action','state_conditions','blockers','warnings']:
        assert token in SCRIPT


def test_docs_include_point_3_state_transition_audit():
    assert 'Point 3: State-transition Audit' in DOC


def test_no_fake_hardware_false_execution_path_added():
    merged = MAIN_CPP + SCRIPT
    assert 'use_fake_hardware:=false execution path' not in merged
