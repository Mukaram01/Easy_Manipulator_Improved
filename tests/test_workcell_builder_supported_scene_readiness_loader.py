from pathlib import Path


def test_loader_supports_registry_yaml_and_readiness_json_fields():
    hdr = Path('workcell_builder/workcell_builder/include/supported_scene_readiness_loader.hpp').read_text(encoding='utf-8')
    src = Path('workcell_builder/workcell_builder/src_supported_scene_readiness_loader.cpp').read_text(encoding='utf-8')
    for token in [
        'load_supported_scene_registry',
        'load_latest_all_scenes_readiness_report',
        'support_level',
        'readiness_status',
        'required_files_status',
        'static_validation_status',
        'guided_build_launch_readiness',
        'fake_hardware_launch_readiness',
        'blockers_summary',
    ]:
        assert token in hdr or token in src


def test_missing_report_returns_unknown_not_crash_path():
    src = Path('workcell_builder/workcell_builder/src_supported_scene_readiness_loader.cpp').read_text(encoding='utf-8')
    assert 'All-scenes readiness report missing' in src
    assert 'UNKNOWN' in src


def test_scene_select_shows_blocker_text_and_report_path_and_run_action():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')
    assert 'Run All-Scenes Readiness' in ui
    assert 'Blockers: ' in cpp
    assert 'Latest all-scenes readiness report:' in cpp
    assert '--skip-build --skip-launch-smoke' in cpp


def test_scene_select_derives_workspace_root_from_repo_checkout_for_readiness_command():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'derive_ros_workspace_root(workcell_path)' in cpp
    assert 'normalized.filename() == "easy_manipulation_deployment"' in cpp
    assert 'normalized.parent_path().filename() == "src"' in cpp
    assert 'return normalized.parent_path().parent_path();' in cpp
    readiness_fn = cpp.split('void SceneSelect::on_run_all_scenes_readiness_clicked()', 1)[1]
    readiness_fn = readiness_fn.split('void SceneSelect::', 1)[0]
    assert 'fs::path(workcell_path).parent_path().string()' not in readiness_fn
    assert '--workspace-root "" + workspace_root.string()' in readiness_fn

    repo_root = Path('/home/user/workcell_ws/src/easy_manipulation_deployment')
    expected_workspace_root = repo_root.parent.parent
    simulated_command = (
        'python3 scripts/validate_supported_scenes_readiness.py '
        f'--repo-root {repo_root} --workspace-root {expected_workspace_root} '
        '--json --skip-build --skip-launch-smoke'
    )
    assert '--workspace-root /home/user/workcell_ws ' in simulated_command
    assert '--workspace-root /home/user/workcell_ws/src ' not in simulated_command
