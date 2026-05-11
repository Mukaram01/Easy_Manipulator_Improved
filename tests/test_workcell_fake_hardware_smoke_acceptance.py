from pathlib import Path


def test_script_exists_and_markers_exist():
    p = Path('scripts/run_workcell_fake_hardware_smoke.py')
    assert p.exists()
    txt = p.read_text(encoding='utf-8')
    for marker in [
        'WORKCELL_FAKE_HARDWARE_SMOKE: PASS',
        'WORKCELL_FAKE_HARDWARE_SMOKE: WARN',
        'WORKCELL_FAKE_HARDWARE_SMOKE: FAIL',
        'WORKCELL_FAKE_HARDWARE_SMOKE: SKIP',
    ]:
        assert marker in txt


def test_default_mode_safe_static_and_opt_in_launch_flags():
    txt = Path('scripts/run_workcell_fake_hardware_smoke.py').read_text(encoding='utf-8')
    for flag in ['--run-launch', '--skip-launch', '--generate-golden-demo', '--use-golden-demo', '--headless', '--timeout-seconds']:
        assert flag in txt
    assert 'if args.run_launch and not args.skip_launch:' in txt


def test_fake_hardware_safety_checks_and_launch_guidance_markers():
    txt = Path('scripts/run_workcell_fake_hardware_smoke.py').read_text(encoding='utf-8')
    for marker in [
        'schema_version: workcell_scene/v1',
        'use_fake_hardware:=true',
        'launch_rviz:=false',
        'real_hardware_enabled: false',
        'runtime_execution_enabled: false',
        'motion_command_sent: false',
        'moveit_plan_service_called: false',
        'validate_workcell_scene.py',
        'validate_workcell_asset_catalog.py',
        'generate_golden_workcell_demo.py',
    ]:
        assert marker in txt


def test_no_pyyaml_epd_or_forbidden_planning_motion_apis():
    txt = Path('scripts/run_workcell_fake_hardware_smoke.py').read_text(encoding='utf-8').lower()
    for forbidden in ['import yaml', 'pyyaml', 'getmotionplan', 'execute_trajectory', '/plan_kinematic_path', 'streamlit', 'easy_manipulation_deployment/emd_demo_nodes']:
        assert forbidden not in txt
