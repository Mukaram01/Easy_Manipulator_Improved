from pathlib import Path

SMOKE = Path('scripts/smoke_test_scratch_cell_workspace.py')
TEXT = SMOKE.read_text(encoding='utf-8')
ERR = Path('scripts/workcell_studio_error_messages.py').read_text(encoding='utf-8')
DOC = Path('docs/manuals/WORKCELL_STUDIO_NEW_CELL_FROM_SCRATCH.md').read_text(encoding='utf-8')


def test_smoke_script_exists_and_references_acceptance_generator():
    assert SMOKE.is_file()
    assert 'generate_scratch_cell_acceptance.py' in TEXT


def test_smoke_checks_workspace_and_tools():
    for token in ['src', 'shutil.which("ros2")', 'shutil.which("colcon")', 'MISSING_WORKSPACE_SRC', 'MISSING_ROS2', 'MISSING_COLCON']:
        assert token in TEXT


def test_smoke_runs_expected_commands_and_fake_hardware_only():
    for token in [
        'colcon build --symlink-install --packages-select',
        'ros2 pkg prefix',
        'ros2 launch',
        '--show-args',
        'use_fake_hardware:=true',
        'launch_rviz:=false',
    ]:
        assert token in TEXT
    assert 'use_fake_hardware:=false' not in TEXT


def test_json_report_fields_exist():
    for token in [
        'scene_name', 'workspace', 'scene_dir', 'workspace_scene_dir', 'placement_method',
        'build_command', 'build_returncode', 'package_discovery_command', 'package_discovery_returncode',
        'launch_show_args_command', 'launch_show_args_returncode', 'launch_smoke_command',
        'launch_smoke_returncode', 'ready_for_rviz_moveit', 'blockers', 'warnings', 'logs_tail',
    ]:
        assert token in TEXT


def test_structured_error_codes_exist_for_real_build_run_audit():
    for token in [
        'MISSING_ROS2', 'MISSING_COLCON', 'MISSING_WORKSPACE_SRC', 'COLCON_BUILD_FAILED',
        'ROS_PACKAGE_NOT_DISCOVERABLE', 'LAUNCH_SHOW_ARGS_FAILED', 'LAUNCH_SMOKE_FAILED',
    ]:
        assert token in ERR


def test_docs_include_point_5_real_build_run_audit():
    assert 'Point 5: Real Build/Run Audit' in DOC
