from pathlib import Path


def test_healthcheck_script_exists_and_has_pass_fail_markers():
    path = Path("scripts/validate_workcell_builder_healthcheck.py")
    assert path.exists()
    txt = path.read_text(encoding="utf-8")
    assert "WORKCELL_BUILDER_HEALTHCHECK: PASS" in txt
    assert "WORKCELL_BUILDER_HEALTHCHECK: FAIL" in txt


def test_healthcheck_cli_defaults_safe_and_optional_flags_present():
    txt = Path("scripts/validate_workcell_builder_healthcheck.py").read_text(encoding="utf-8")
    assert "--run-colcon" in txt
    assert "--smoke-launch" in txt
    assert "run_launch = args.smoke_launch" in txt


def test_healthcheck_checks_build_integration_and_launch_safety():
    txt = Path("scripts/validate_workcell_builder_healthcheck.py").read_text(encoding="utf-8")
    assert "gui/asset_picker_dialog.cpp" in txt
    assert "src_asset_discovery_helper.cpp" in txt
    assert "gui/scene_select.cpp" in txt
    assert "demo.launch.py use_fake_hardware:=true" in txt
    assert "use_fake_hardware:=false" in txt
    assert "launch_rviz" in txt
    assert "launch_rviz:=false" in txt or "launch_rviz:=false\"" in txt


def test_healthcheck_checks_artifacts_strings_and_placeholders_and_asset_ui():
    txt = Path("scripts/validate_workcell_builder_healthcheck.py").read_text(encoding="utf-8")
    assert "workcell_studio_summary.json" in txt
    assert "workcell_studio_summary.md" in txt
    assert "preview/workcell_preview.svg" in txt
    assert "preview/workcell_preview.html" in txt
    assert "Select Robot Asset" in txt
    assert "Select End Effector Asset" in txt
    assert "Select Existing STL" in txt
    assert "unknown_description" in txt
    assert "unknown_moveit_config" in txt
    assert "none_moveit_config" in txt


def test_healthcheck_includes_task_recipe_preview_checks():
    txt = Path("scripts/validate_workcell_builder_healthcheck.py").read_text(encoding="utf-8")
    for needle in ["preview_task_recipe.py", "task_recipe.py", "WORKCELL_TASK_RECIPE_PREVIEW: PASS", "task_recipe_path", "OFFLINE_ONLY", "NO_MOTION_COMMAND", "NO_MOVEIT_PLAN", "NO_REAL_HARDWARE"]:
        assert needle in txt


def test_healthcheck_tracks_operator_flow_and_validation_dashboard_markers():
    txt = Path("scripts/validate_workcell_builder_healthcheck.py").read_text(encoding="utf-8")
    for needle in [
        "Operator Workflow (Main)",
        "Validation Dashboard",
        "Run Offline Validation",
        "Developer Tools: Create Golden UR5 + Robotiq 2F Cell",
        "blocker_count",
        "warning_count",
    ]:
        assert needle in txt
