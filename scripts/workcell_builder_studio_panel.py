from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from scripts.workcell_builder_studio_summary import summarize_builder_scene


def _read_yaml_like(path: Path) -> dict[str, Any]:
    try:
        import yaml  # type: ignore
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def _find_intent(scene_package: Path) -> Path | None:
    for rel in ("config/workcell_builder_task_intent.yaml", "generated/workcell_builder_task_intent.yaml", "workcell_builder_task_intent.yaml"):
        p = scene_package / rel
        if p.is_file():
            return p
    return None


def detect_scene_context(scene_package: Path) -> dict[str, str]:
    scene_package = Path(scene_package)
    intent = _read_yaml_like(_find_intent(scene_package) or scene_package / "missing.yaml")
    env = _read_yaml_like(scene_package / "environment.yaml")
    layout = _read_yaml_like(scene_package / "generated" / "environment_layout.yaml") or _read_yaml_like(scene_package / "environment_layout.yaml")

    pick = intent.get("pick", {}) if isinstance(intent.get("pick"), dict) else {}
    place = intent.get("place", {}) if isinstance(intent.get("place"), dict) else {}
    grasp = intent.get("grasp", {}) if isinstance(intent.get("grasp"), dict) else {}
    routing = intent.get("routing", {}) if isinstance(intent.get("routing"), dict) else {}
    rules = routing.get("rules", []) if isinstance(routing.get("rules"), list) else []

    zones = layout.get("zones", []) if isinstance(layout.get("zones"), list) else []
    camera = next((a for a in (layout.get("assets") or []) if isinstance(a, dict) and a.get("type") == "camera"), {})
    table = next((a for a in (layout.get("assets") or []) if isinstance(a, dict) and a.get("type") in {"table", "support_surface"}), {})

    return {
        "scene_package_path": str(scene_package),
        "scene_package_name": scene_package.name,
        "selected_robot": str((env.get("robot") or {}).get("name", "unknown")),
        "selected_end_effector": str((env.get("end_effector") or {}).get("name", "unknown")),
        "selected_camera": str(camera.get("id", "unknown")),
        "selected_table": str(table.get("id", "unknown")),
        "selected_pick_source": str((pick.get("source") or {}).get("type", "unknown")),
        "selected_pick_zone": str((pick.get("source") or {}).get("id", "unknown")),
        "selected_place_target": str((place.get("target") or {}).get("id", "unknown")),
        "selected_task_type": str((intent.get("task") or {}).get("type", "unknown")),
        "selected_task_template": str((intent.get("task_template") or {}).get("id", (intent.get("task") or {}).get("template", "unknown"))),
        "selected_grasp_strategy": str(grasp.get("strategy_ref", "unknown")),
        "selected_release_strategy": str(place.get("release_strategy", "unknown")),
        "selected_routing_target": str((rules[0].get("place_target") if rules and isinstance(rules[0], dict) else "unknown")),
        "pick_zone_count": str(sum(1 for z in zones if isinstance(z, dict) and z.get("type") == "pick_zone")),
        "place_target_count": str(sum(1 for z in zones if isinstance(z, dict) and z.get("type") in {"bin", "place_target", "table_zone", "fixture", "conveyor_drop", "custom_pose"})),
    }


def build_validate_scene_command(scene_package: Path) -> list[str]:
    return ["python3", "scripts/validate_builder_generated_scene.py", "--scene-package", str(scene_package), "--json"]


def build_export_sources_command(scene_package: Path) -> list[str]:
    return ["bash", str(Path(scene_package) / "generated" / "export_workcell_studio_sources.sh")]


def build_readiness_pack_command(scene_package: Path, output_dir: Path, project_name: str = "builder_scene") -> list[str]:
    return ["python3", "scripts/generate_workcell_studio_readiness_pack.py", "--scene-package", str(scene_package), "--output-dir", str(output_dir), "--project-name", project_name, "--validate", "--smoke-dry-run", "--force", "--json"]


def build_rviz_preview_command(scene_package_name: str) -> str:
    return " \\\n  ".join([
        f"ros2 launch {scene_package_name} demo.launch.py",
        "use_fake_hardware:=true",
        "launch_rviz:=true",
    ])


def build_grasp_flow_preview_command(scene_package_name: str, bridge_payload_path: Path) -> str:
    return " \\\n  ".join([
        "ros2 launch run_grasp_execution grasp_execution.launch.py",
        f"scene_package:={scene_package_name}",
        "launch_rviz:=true",
        "use_fake_hardware:=true",
        "explicit_release_pose_source:=bridge_payload",
        f"explicit_release_pose_bridge_payload_path:={bridge_payload_path}",
    ])


def panel_state_from_validation(report: dict[str, Any], scene_package: Path) -> dict[str, Any]:
    warnings = list(report.get("warnings") or [])
    blockers = list(report.get("errors") or report.get("blockers") or [])
    summary = summarize_builder_scene(scene_package)
    return {
        **detect_scene_context(scene_package),
        "studio_summary": summary,
        "summary_panel_title": "Workcell Studio Summary",
        "scene_validation_status": report.get("status", "UNKNOWN"),
        "export_status": "READY" if (Path(scene_package) / "generated" / "export_workcell_studio_sources.sh").exists() else "MISSING",
        "readiness_status": report.get("readiness", report.get("readiness_classification", "unknown")),
        "preview_status": "MANUAL_ONLY",
        "safety_status": "FAKE_HARDWARE_ONLY",
        "workflow_steps": ["Build Cell", "Select Task Template", "Define Task", "Validate", "Export", "Preview", "Review Safety"],
        "sections": ["Cell Setup", "Task Template", "Pick Source", "Place Target", "Grasp Strategy", "Validation", "Export", "Preview Commands", "Safety"],
        "help_text": {
            "pick_source": "Pick source: where the object comes from.",
            "epd_detected": "Use EPD detected object when the camera/perception system supplies the object pose.",
            "pick_zone": "Pick zone: area where detected objects are allowed to be picked.",
            "place_target": "Place target: where the robot releases the object.",
            "routing_rule": "Routing rule: decides which place target receives which object.",
            "fake_hw": "Fake hardware preview does not move a real robot.",
            "safety_notice": "Generated readiness reports are not safety certificates.",
        },
        "warnings": warnings,
        "blockers": blockers,
        "safety_banner": [
            "fake hardware first",
            "no robot motion by default",
            "no runtime execution from this panel",
            "no real hardware enabled",
            "generated reports are not safety certificates",
        ],
    }


def read_validation_report(path: Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))
