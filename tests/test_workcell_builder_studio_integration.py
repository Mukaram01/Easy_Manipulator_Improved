from __future__ import annotations

from pathlib import Path

from scripts import workcell_builder_studio_panel as panel

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENE_SELECT_CPP = REPO_ROOT / "workcell_builder/workcell_builder/gui/scene_select.cpp"
FIXTURE_SCENE = REPO_ROOT / "scenes/ur5_2f_test"


def test_workcell_studio_command_panel_hints_present() -> None:
    text = SCENE_SELECT_CPP.read_text(encoding="utf-8")
    assert "Workcell Studio command centre" in text
    assert "Validate Scene" in text
    assert "Export Workcell Studio Sources" in text
    assert "Generate Readiness Pack" in text
    assert "Open Static Preview" in text
    assert "Open Readiness Dashboard" in text
    assert "Copy RViz Preview Command" in text
    assert "Copy Grasp Flow Preview Command" in text


def test_panel_backend_detects_scene_path_and_not_hardcoded() -> None:
    context = panel.detect_scene_context(FIXTURE_SCENE)
    assert context["scene_package_path"].endswith("scenes/ur5_2f_test")
    other = panel.detect_scene_context(REPO_ROOT / "scenes" / "scene_1")
    assert other["scene_package_path"].endswith("scenes/scene_1")


def test_validation_export_and_readiness_commands() -> None:
    validate = panel.build_validate_scene_command(FIXTURE_SCENE)
    export_cmd = panel.build_export_sources_command(FIXTURE_SCENE)
    readiness = panel.build_readiness_pack_command(FIXTURE_SCENE, Path("/tmp/out"), "demo")
    assert "validate_builder_generated_scene.py" in " ".join(validate)
    assert str(FIXTURE_SCENE) in validate
    assert "export_workcell_studio_sources.sh" in " ".join(export_cmd)
    assert "generate_workcell_studio_readiness_pack.py" in " ".join(readiness)


def test_preview_commands_are_fake_hardware_only() -> None:
    rviz = panel.build_rviz_preview_command("my_scene")
    grasp = panel.build_grasp_flow_preview_command("my_scene", Path("/tmp/bridge_payload.json"))
    assert "use_fake_hardware:=true" in rviz
    assert "use_fake_hardware:=true" in grasp
    assert "real_hardware" not in rviz + grasp
    assert "use_fake_hardware:=false" not in rviz + grasp


def test_warning_blocker_propagation() -> None:
    report = {
        "status": "FAIL",
        "readiness": "physical_scene_only",
        "warnings": ["TODO value found in task intent", "fallback coordinates used"],
        "errors": ["place target missing", "pick source missing", "object missing", "exported files missing"],
    }
    state = panel.panel_state_from_validation(report, FIXTURE_SCENE)
    assert state["scene_validation_status"] == "FAIL"
    assert "fallback coordinates used" in state["warnings"]
    assert "place target missing" in state["blockers"]
    assert "fake hardware first" in state["safety_banner"]
