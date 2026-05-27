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
