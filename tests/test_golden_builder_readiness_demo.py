from __future__ import annotations

import json
import shutil
import subprocess
import sys

import yaml
from pathlib import Path


def test_golden_builder_readiness_demo_end_to_end(tmp_path: Path) -> None:
    scene = tmp_path / "scene"
    shutil.copytree("scenes/ur5_2f_test", scene)
    out = tmp_path / "golden_pack"
    run = subprocess.run([
        sys.executable,
        "scripts/run_golden_builder_readiness_demo.py",
        "--scene-package", str(scene),
        "--output-dir", str(out),
        "--force",
        "--json",
    ], capture_output=True, text=True, check=False)
    assert run.returncode == 0

    manifest_path = out / "readiness_pack_manifest.json"
    assert manifest_path.exists()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    assert Path(manifest["artifacts"]["cell_definition"]).exists()
    assert Path(manifest["artifacts"]["environment_layout"]).exists()
    assert Path(manifest["artifacts"]["builder_task_intent"]).exists()
    assert Path(manifest["artifacts"]["task_flow_summary"]).exists()
    assert Path(manifest["artifacts"]["readiness_dashboard"]).exists()
    marker_path = Path(manifest["artifacts"]["static_preview"]["markers"])
    assert marker_path.exists()
    marker_payload = json.loads(marker_path.read_text(encoding="utf-8"))
    marker_names = {m.get("name") for m in marker_payload.get("markers", []) if isinstance(m, dict)}
    assert "pick_zone_source" in marker_names
    assert "place_zone_target" in marker_names
    exported_layout = yaml.safe_load(Path(manifest["artifacts"]["environment_layout"]).read_text(encoding="utf-8"))
    assert "pick_zone_main" in json.dumps(exported_layout)
    assert "bin_red" in json.dumps(exported_layout)
    assert "red_box_001" in json.dumps(exported_layout)
    assert any(str(name).startswith("task_flow") for name in marker_names)

    assert manifest["results"]["task_intent_status"] == "PASS"
    ti = json.loads(Path(manifest["artifacts"]["task_flow_summary"]).read_text(encoding="utf-8"))
    assert ti.get("pick_source_id") == "pick_zone_main"
    assert ti.get("place_target_id") == "bin_red"
    assert ti.get("visual_resolution", {}).get("approximate_coordinates_used") is False
    assert "TODO" not in json.dumps(ti)
    assert manifest["results"].get("classification") != "physical_scene_only"
    assert manifest["safety"]["real_hardware_enabled"] is False
    assert manifest["safety"]["motion_command_sent"] is False
    assert manifest["safety"]["runtime_execution_called"] is False

    dashboard = Path(manifest["artifacts"]["readiness_dashboard"]).read_text(encoding="utf-8")
    assert "Task flow: pick → grasp → place → release" in dashboard
    assert "Offline/fake-hardware readiness review only" in dashboard

    summary_path = out / "golden_builder_demo_summary.json"
    assert summary_path.exists()
    summary = json.loads(summary_path.read_text(encoding="utf-8"))

    visual = summary.get("visual_preview", {})
    assert visual.get("present") is True
    assert visual.get("marker_count", 0) > 0
    assert visual.get("task_flow_marker_count", 0) > 0
    assert visual.get("safety_banner_present") is True

    rviz = summary.get("rviz_moveit_preview", {})
    assert isinstance(rviz, dict)
    assert "status" in rviz
    assert rviz.get("classification") in ("rviz_preview_ready", "rviz_preview_partial")
    assert "robot_description" in rviz
    assert "end_effector_metadata" in rviz
    assert "support_surface_or_table" in rviz
    assert "pick_zone" in rviz
    assert "place_zone" in rviz
    assert "task_flow_markers" in rviz
    assert "fake_hardware_launch_command" in rviz
    assert "rviz_config_or_preview_markers" in rviz

    if rviz.get("fake_hardware_launch_command"):
        assert isinstance(rviz.get("preview_command"), str)
        assert "use_fake_hardware:=true" in rviz["preview_command"]
    else:
        assert rviz.get("classification") == "rviz_preview_partial"
        assert rviz.get("blockers") or rviz.get("warnings")

    safety = summary.get("safety", {})
    assert safety.get("use_fake_hardware") is True
    assert safety.get("real_hardware_enabled") is False
    assert safety.get("motion_command_sent") is False
    assert safety.get("runtime_execution_called") is False
    assert safety.get("moveit_plan_service_called") is False
    bridge = summary.get("perception_bridge", {})
    assert bridge.get("status") in ("bridge_preview_ready", "bridge_preview_partial", "bridge_preview_blocked")
    assert Path(bridge.get("emd_bridge_payload_preview")).exists()
    assert Path(bridge.get("perception_bridge_preview_report")).exists()

    handoff = summary.get("plan_preview_handoff", {})
    assert handoff.get("status") in ("plan_preview_ready", "plan_preview_partial", "plan_preview_blocked")
    assert Path(handoff.get("bridge_payload_plan_preview_handoff")).exists()
