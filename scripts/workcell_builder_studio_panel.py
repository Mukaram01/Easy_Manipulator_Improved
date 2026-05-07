from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def _read_yaml_like(path: Path) -> dict[str, Any]:
    try:
        import yaml  # type: ignore
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def detect_scene_context(scene_package: Path) -> dict[str, str]:
    scene_package = Path(scene_package)
    intent = _read_yaml_like(scene_package / "generated" / "workcell_builder_task_intent.yaml")
    env = _read_yaml_like(scene_package / "environment.yaml")
    return {
        "scene_package_path": str(scene_package),
        "scene_package_name": scene_package.name,
        "selected_robot": str((env.get("robot") or {}).get("name", "unknown")),
        "selected_end_effector": str((env.get("end_effector") or {}).get("name", "unknown")),
        "selected_pick_source": str(intent.get("pick_source", "unknown")),
        "selected_place_target": str(intent.get("place_target", "unknown")),
        "selected_object": str(intent.get("object", "unknown")),
        "selected_grasp_strategy": str(intent.get("grasp_strategy", "unknown")),
        "selected_release_strategy": str(intent.get("release_strategy", "unknown")),
        "selected_routing_target": str(intent.get("routing_target", "unknown")),
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
    text = "\n".join(warnings + blockers)
    for needle in ["place target", "pick source", "object", "TODO", "fallback", "missing"]:
        if needle.lower() in text.lower():
            pass
    return {
        **detect_scene_context(scene_package),
        "scene_validation_status": report.get("status", "UNKNOWN"),
        "export_status": "READY" if (Path(scene_package) / "generated" / "export_workcell_studio_sources.sh").exists() else "MISSING",
        "readiness_status": report.get("readiness", report.get("readiness_classification", "unknown")),
        "preview_status": "MANUAL_ONLY",
        "safety_status": "FAKE_HARDWARE_ONLY",
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
