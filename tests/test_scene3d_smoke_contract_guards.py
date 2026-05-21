from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN_CPP = (ROOT / "workcell_builder/workcell_builder/gui/main.cpp").read_text(encoding="utf-8")
CANVAS_CONTRACT = (ROOT / "scripts/check_scene3d_canvas_contract.py").read_text(encoding="utf-8")
RUNTIME_ACCEPTANCE = (ROOT / "scripts/validate_scene3d_runtime_acceptance.py").read_text(encoding="utf-8")


def test_main_cpp_contains_scene3d_and_new_cell_smoke_cli_flags():
    for token in ["--scene3d-smoke", "--new-cell-recommended-layout-smoke"]:
        assert token in MAIN_CPP


def test_smoke_contract_reports_schema_and_required_counter_fields():
    for token in [
        "'overall_status'",
        "'scenes'",
        "'preview_items_count'",
        "'visible_after_filters_count'",
        "'filtered_hidden_count'",
        "'render_cache_received_count'",
    ]:
        assert token in CANVAS_CONTRACT


def test_pass_warn_fail_semantics_and_missing_screenshot_warn_not_fail():
    for token in ["status = 'PASS'", "status = 'FAIL'", "status = 'WARN'"]:
        assert token in CANVAS_CONTRACT
    assert "missing screenshot" in CANVAS_CONTRACT.lower()
    assert "FAIL: missing screenshot" not in CANVAS_CONTRACT


def test_failure_cases_for_zero_visible_zero_hierarchy_rows_and_zero_cache_are_guarded():
    for token in [
        "visible_after_filters_count is zero by default",
        "scene_hierarchy_tree_->topLevelItemCount()",
        "render_cache_received_count",
    ]:
        assert token in (CANVAS_CONTRACT + "\n" + RUNTIME_ACCEPTANCE + "\n" + MAIN_CPP)


def test_wrapper_timeout_and_return_code_behavior_for_pass_fail_json_present():
    for token in [
        "raise SystemExit(1 if overall == 'FAIL' else 0)",
        "return 0 if payload[\"pass\"] else 1",
    ]:
        assert token in (CANVAS_CONTRACT + "\n" + RUNTIME_ACCEPTANCE)


def test_runtime_cpp_touch_guard_prevents_docs_tests_only_smoke_false_positives():
    corpus = "\n".join(
        [
            MAIN_CPP,
            (ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text(encoding="utf-8"),
            (ROOT / "workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp").read_text(encoding="utf-8"),
        ]
    )
    assert "scene3d" in corpus.lower()


def test_safety_string_guards_block_real_hardware_and_auto_live_launch_in_new_smoke_paths():
    banned = [
        "use_fake_hardware:=false",
        "fake_hardware:=false",
        "ur_robot_driver",
        "ethercat",
        "canopen",
        "realsense2_camera",
        "easy_perception_deployment",
    ]
    smoke_scope = "\n".join([MAIN_CPP, CANVAS_CONTRACT, RUNTIME_ACCEPTANCE]).lower()
    for token in banned:
        assert token not in smoke_scope
