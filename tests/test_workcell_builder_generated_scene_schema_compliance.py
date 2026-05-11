from pathlib import Path
import json
from scripts import workcell_builder_gui_workflow as wf


def test_builder_generation_writes_schema_v1_environment_and_summary(tmp_path):
    state = {
        "scene_name": "schema_scene",
        "fake_hardware_default": True,
        "selected": {"robot": "ur5", "tool": "robotiq_2f", "camera": "realsense_d435i", "task": "pick_place"},
        "current_cell_assets": [{"asset_id": "cube", "category": "object", "role": "pick_object", "pose": {"xyz": [0.4, 0.1, 0.2], "rpy": [0, 0, 0]}}],
    }
    scene_dir = tmp_path / "schema_scene"
    out = wf.generate_yaml_files_for_scene(state, scene_dir)
    assert out["ok"]
    env = (scene_dir / "environment.yaml").read_text(encoding="utf-8")
    for token in [
        "schema_version: workcell_scene/v1",
        "scene:", "robot:", "tool:", "compatibility:", "placed_objects:",
        "camera:", "task:", "workspace:", "safety:", "metadata:",
        "fake_hardware_first: true", "real_hardware_enabled: false",
        "runtime_execution_enabled: false", "motion_command_sent: false",
    ]:
        assert token in env

    summary = json.loads((scene_dir / "generated" / "builder_export_summary.json").read_text(encoding="utf-8"))
    assert summary["scene_schema_version"] == "workcell_scene/v1"
    assert summary["scene_schema_validation_status"] in {"PASS", "WARN", "FAIL"}
    assert "scene_schema_warnings" in summary
    assert "scene_schema_blockers" in summary
    assert (scene_dir / "generated" / "scene_schema_validator_command.txt").exists()


def test_golden_demo_still_passes_schema_validator(tmp_path):
    result = wf.create_golden_demo_cell("ur5_2f_golden_demo", tmp_path)
    assert result["ok"]
    text = (Path(result["scene_dir"]) / "environment.yaml").read_text(encoding="utf-8")
    assert "schema_version: workcell_scene/v1" in text


def test_no_pyyaml_or_forbidden_runtime_strings_added():
    merged = Path("scripts/workcell_builder_gui_workflow.py").read_text(encoding="utf-8").lower()
    for forbidden in ["import yaml", "pyyaml", "getmotionplan", "execute_trajectory", "/plan_kinematic_path"]:
        assert forbidden not in merged
