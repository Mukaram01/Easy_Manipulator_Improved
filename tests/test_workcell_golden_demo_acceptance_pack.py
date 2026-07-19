from pathlib import Path


def test_golden_scripts_exist_and_markers_present():
    gen = Path('scripts/generate_golden_workcell_demo.py')
    val = Path('scripts/validate_golden_workcell_demo.py')
    assert gen.exists()
    assert val.exists()
    vtxt = val.read_text(encoding='utf-8')
    assert 'WORKCELL_GOLDEN_DEMO: PASS' in vtxt
    assert 'WORKCELL_GOLDEN_DEMO: WARN' in vtxt
    assert 'WORKCELL_GOLDEN_DEMO: FAIL' in vtxt


def test_golden_generator_tokens_and_safety_markers():
    txt = Path('scripts/generate_golden_workcell_demo.py').read_text(encoding='utf-8')
    for needle in [
        'ur5_2f_golden_demo',
        'schema_version: workcell_scene/v1',
        'schema_version: workcell_task/v1',
        'realsense_d435i',
        'robotiq',
        'fake_hardware_first: true',
        'real_hardware_enabled: false',
        'motion_command_sent: false',
        'runtime_execution_enabled: false',
        'placed_objects',
        'visual_layout_metadata',
        'perception_metadata.json',
        'compatibility_metadata.json',
        'readiness_overlay_metadata.json',
        '--install-into-scenes',
    ]:
        assert needle in txt


def test_no_forbidden_runtime_or_deps_or_main_button_tokens():
    txt = Path('scripts/generate_golden_workcell_demo.py').read_text(encoding='utf-8').lower()
    for forbidden in ['import yaml', 'pyyaml', 'getmotionplan', 'execute_trajectory', '/plan_kinematic_path', 'streamlit']:
        assert forbidden not in txt
    scene_cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8').lower()
    assert 'golden demo pack' not in scene_cpp


def test_workspace_alias_policy_files_untouched_markers_present():
    fix = Path('scripts/fix_workspace_layout.sh').read_text(encoding='utf-8')
    verify = Path('scripts/verify_workspace_discovery.sh').read_text(encoding='utf-8')
    assert 'ensure_workspace_alias "assets"' in fix
    assert 'ensure_workspace_alias "scenes"' in fix
    assert 'CANONICAL_REPO="$SRC_DIR/easy_manipulation_deployment"' in verify
    assert 'easy_manipulation_deployment' in verify
    assert 'assets' in verify
    assert 'scenes' in verify
    assert 'Duplicate package discovered' in verify
    assert 'Workspace validation passed:' in verify
    assert 'required packages are discoverable exactly once' in verify
    assert 'for alias in assets scenes' not in verify
