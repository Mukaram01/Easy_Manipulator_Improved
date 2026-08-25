from pathlib import Path

SCRIPT = Path('scripts/generate_scratch_cell_acceptance.py')
TEXT = SCRIPT.read_text(encoding='utf-8')
MAIN = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_script_exists_and_defaults_present():
    assert SCRIPT.is_file()
    for token in ['scratch_ur5_2f_acceptance', 'ur5', 'robotiq_2f', 'pick_place']:
        assert token in TEXT


def test_required_output_filenames_checked_and_report_fields_present():
    for token in [
        'environment.yaml', 'layout/workcell_studio_layout.yaml', 'config/workcell_builder_task_intent.yaml',
        'cell_definition.yaml', 'scene_manifest.yaml', 'package.xml', 'CMakeLists.txt', 'launch/demo.launch.py',
        'scene_name', 'scene_dir', 'generated_files', 'missing_files', 'validation_status', 'blockers',
        'warnings', 'build_command', 'source_command', 'launch_command', 'ready_for_plan_simulate'
    ]:
        assert token in TEXT


def test_launch_command_contract_and_fake_hardware_safety():
    assert 'ros2 launch' in TEXT
    assert 'demo.launch.py' in TEXT
    assert 'use_fake_hardware:=true' in TEXT
    assert 'launch_rviz:=true' in TEXT
    assert 'use_fake_hardware:=false' in TEXT  # explicit blocker guard text exists


def test_safe_suffix_and_missing_asset_blockers_exist():
    for token in ['while d.exists()', 'Missing UR5 assets', 'Missing Robotiq 2F assets', 'invalid output root']:
        assert token in TEXT


def test_ui_has_scratch_acceptance_status_lines():
    for token in ['Scratch cell generated', 'File outputs checked', 'Metadata coherent', 'Package files present', 'Plan & Simulate command ready']:
        assert token in MAIN
