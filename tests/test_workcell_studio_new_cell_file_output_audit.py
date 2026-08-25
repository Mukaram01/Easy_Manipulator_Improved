from pathlib import Path

AUDIT = Path('scripts/audit_new_cell_file_outputs.py')
ACCEPTANCE = Path('scripts/generate_scratch_cell_acceptance.py').read_text(encoding='utf-8')
DOC = Path('docs/manuals/WORKCELL_STUDIO_NEW_CELL_FROM_SCRATCH.md').read_text(encoding='utf-8')
TEXT = AUDIT.read_text(encoding='utf-8')
LAYOUT_SOURCE = Path('scripts/workcell_studio_layout_source.py').read_text(encoding='utf-8')


def test_audit_script_exists_and_cli_args_present():
    assert AUDIT.is_file()
    for token in ['--scene-dir', '--scene-name', '--json-out']:
        assert token in TEXT + LAYOUT_SOURCE


def test_required_filenames_and_report_fields_present():
    for token in [
        'environment.yaml', 'layout/workcell_studio_layout.yaml', 'config/workcell_builder_task_intent.yaml',
        'cell_definition.yaml', 'scene_manifest.yaml', 'package.xml', 'CMakeLists.txt', 'launch/demo.launch.py',
        'scene_name', 'scene_dir', 'required_files', 'optional_files', 'present_files', 'missing_files',
        'malformed_files', 'content_checks', 'cross_reference_checks', 'blockers', 'warnings', 'file_output_status'
    ]:
        assert token in TEXT + LAYOUT_SOURCE


def test_required_content_tokens_and_cross_checks_exist():
    for token in ['ur5', 'robotiq', 'pick_place', 'use_fake_hardware', 'launch_rviz']:
        assert token in TEXT
    assert 'package.xml scene-name consistency' in TEXT
    assert 'task pick/place cross-reference check exists' in TEXT


def test_acceptance_generator_references_file_output_audit():
    assert 'audit_new_cell_file_outputs.py' in ACCEPTANCE
    assert 'file_output_audit' in ACCEPTANCE


def test_docs_mention_point_2_file_output_audit():
    assert 'Point 2: File-output Audit' in DOC
    assert 'PASS' in DOC and 'WARNINGS' in DOC and 'BLOCKED' in DOC


def test_no_real_hardware_path_added():
    merged = TEXT + ACCEPTANCE
    assert 'use_fake_hardware:=false execution button' not in merged
