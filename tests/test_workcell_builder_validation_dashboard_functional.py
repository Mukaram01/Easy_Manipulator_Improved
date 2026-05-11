from pathlib import Path


def _txt(path: str) -> str:
    return Path(path).read_text(encoding="utf-8")


def test_dashboard_model_files_and_cmake_wiring_exist():
    hpp = "workcell_builder/workcell_builder/include/validation_dashboard_model.hpp"
    cpp = "workcell_builder/workcell_builder/src_validation_dashboard_model.cpp"
    assert Path(hpp).exists()
    assert Path(cpp).exists()
    cmake = _txt("workcell_builder/workcell_builder/CMakeLists.txt")
    assert "src_validation_dashboard_model.cpp" in cmake


def test_required_dashboard_rows_and_statuses_present():
    blob = _txt("workcell_builder/workcell_builder/src_validation_dashboard_model.cpp")
    for row in [
        "Scene Schema",
        "Asset Catalog",
        "Robot / Tool Compatibility",
        "Object Placement",
        "Camera Metadata",
        "Task Recipe",
        "Readiness Overlay",
        "Fake-Hardware Smoke Static",
        "Generation Safety",
    ]:
        assert row in blob
    for status in ["PASS", "WARN", "FAIL", "SKIP", "UNKNOWN"]:
        assert status in _txt("workcell_builder/workcell_builder/include/validation_dashboard_model.hpp")


def test_scene_select_wires_run_validation_and_generation_gating_to_dashboard():
    cpp = _txt("workcell_builder/workcell_builder/gui/scene_select.cpp")
    for marker in [
        "collect_validation_dashboard_results",
        "refresh_validation_dashboard_table",
        "validation_dashboard_status",
        "validation_dashboard_warning_count",
        "validation_dashboard_blocker_count",
        "validation_dashboard_rows",
        "Run Offline Validation only: no ROS launch",
    ]:
        assert marker in cpp


def test_forbidden_additions_not_present():
    blob = (_txt("workcell_builder/workcell_builder/gui/scene_select.cpp") + _txt("workcell_builder/workcell_builder/src_validation_dashboard_model.cpp")).lower()
    for forbidden in ["pyyaml", "import yaml", "getmotionplan", "execute_trajectory", "followjointtrajectory", "real_hardware_enabled: true"]:
        assert forbidden not in blob
