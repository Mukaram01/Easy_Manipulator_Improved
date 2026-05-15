from pathlib import Path

ERR = Path('scripts/workcell_studio_error_messages.py').read_text(encoding='utf-8')
CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
DOC = Path('docs/manuals/WORKCELL_STUDIO_NEW_CELL_FLOW.md').read_text(encoding='utf-8')


def test_standard_fields_exist():
    for token in ['code','severity','title','detail','why_it_matters','next_action','recovery_command','related_file','related_page']:
        assert token in ERR


def test_common_codes_exist():
    for token in ['MISSING_WORKSPACE','INVALID_SCENE_NAME','SCENE_ALREADY_EXISTS','MISSING_ENVIRONMENT_LAYOUT','MALFORMED_ENVIRONMENT_LAYOUT','MISSING_TASK_INTENT','TASK_PICK_PLACE_NOT_IN_LAYOUT','MISSING_PACKAGE_XML','MISSING_CMAKELISTS','MISSING_DEMO_LAUNCH','MISSING_FAKE_HARDWARE_ARG','VALIDATION_BLOCKED']:
        assert token in ERR


def test_ui_checklist_references_blocker_action_and_recovery():
    for token in ['First blocker','Next action','Related page','Recovery command']:
        assert token in CPP


def test_docs_point_4_exists():
    assert 'Point 4: Error-message Audit' in DOC


def test_no_fake_hardware_false_execution_path_added():
    assert 'use_fake_hardware:=false execution path' not in (ERR + CPP)
