from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from scripts.validate_builder_generated_scene import validate_scene
from scripts.capability_registry import load_structured_data


def _load_yaml_like(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    data, _ = load_structured_data(path)
    return data if isinstance(data, dict) else {}


def _read_task_intent(scene_package: Path) -> dict[str, Any]:
    for rel in ("config/workcell_builder_task_intent.yaml", "generated/workcell_builder_task_intent.yaml", "workcell_builder_task_intent.yaml"):
        p = scene_package / rel
        if p.is_file():
            return _load_yaml_like(p)
    return {}


def _read_layout(scene_package: Path) -> dict[str, Any]:
    for rel in ("layout/workcell_studio_layout.yaml", "generated/environment_layout.yaml", "environment_layout.yaml"):
        p = scene_package / rel
        if p.is_file():
            return _load_yaml_like(p)
    return {}


def _status_from_validation(report: dict[str, Any]) -> str:
    if report.get("errors"):
        return "FAIL"
    if report.get("warnings"):
        return "WARN"
    return "OK"


def summarize_builder_scene(scene_package: Path, *, source_project_path: Path | None = None) -> dict[str, Any]:
    scene_package = Path(scene_package)
    env = _load_yaml_like(scene_package / "environment.yaml")
    metadata = _load_yaml_like(scene_package / "workcell_builder_metadata.yaml")
    layout = _read_layout(scene_package)
    intent = _read_task_intent(scene_package)

    canonical_items = layout.get("items", []) if isinstance(layout.get("items"), list) else []
    assets = layout.get("assets", []) if isinstance(layout.get("assets"), list) else [
        item for item in canonical_items if isinstance(item, dict) and item.get("role") not in {"pick_zone", "place_zone", "keepout"}
    ]
    zones = layout.get("zones", []) if isinstance(layout.get("zones"), list) else [
        item for item in canonical_items if isinstance(item, dict) and item.get("role") in {"pick_zone", "place_zone", "keepout"}
    ]
    asset_types = {str(a.get("type")) for a in assets if isinstance(a, dict) and a.get("type")}
    zone_types = {str(z.get("type")) for z in zones if isinstance(z, dict) and z.get("type")}

    validation = validate_scene(scene_package)
    status = _status_from_validation(validation)

    custom_stl_assets = metadata.get("custom_stl_assets", []) if isinstance(metadata.get("custom_stl_assets"), list) else []

    summary = {
        "project_name": scene_package.name,
        "cell_name": scene_package.name,
        "selected_robot": (env.get("robot") or {}).get("name", "unknown"),
        "selected_end_effector": (env.get("end_effector") or {}).get("name", "unknown"),
        "selected_sensor_camera": metadata.get("sensor_capability_id", "unknown"),
        "environment_assets": {
            "table": "table" in asset_types or "support_surface" in asset_types,
            "bins": any(t in zone_types for t in ("bin", "place_target")),
            "conveyor": "conveyor" in asset_types or "conveyor_drop" in zone_types,
            "camera": "camera" in asset_types,
            "robot_base": "robot_base" in asset_types,
            "object_spawn_area": "pick_zone" in zone_types,
        },
        "task_type": ((intent.get("task") or {}).get("type") if isinstance(intent.get("task"), dict) else "unknown") or "unknown",
        "task_template": ((intent.get("task_template") or {}).get("id") if isinstance(intent.get("task_template"), dict) else ((intent.get("task") or {}).get("template") if isinstance(intent.get("task"), dict) else "unknown")) or "unknown",
        "task_template_metadata": intent.get("task_template") if isinstance(intent.get("task_template"), dict) else {},
        "grasp_strategy": ((intent.get("grasp") or {}).get("strategy_ref") if isinstance(intent.get("grasp"), dict) else "unknown") or "unknown",
        "generated_scene_package_path": str(scene_package),
        "use_fake_hardware_default": True,
        "readiness_status": status,
        "compatibility_status": ((metadata.get("compatibility") or {}).get("status") if isinstance(metadata.get("compatibility"), dict) else "unknown"),
        "compatibility_reasons": ((metadata.get("compatibility") or {}).get("reasons") if isinstance(metadata.get("compatibility"), dict) else []),
        "blockers": list(validation.get("errors") or []),
        "warnings": list(validation.get("warnings") or []),
        "validation_readiness_classification": validation.get("readiness", "unknown"),
        "generator": {"name": "workcell_builder", "version": metadata.get("generator_version", "unknown")},
        "source_builder_project_path": str(source_project_path or scene_package),
        "no_runtime_execution_default": True,
        "custom_stl_assets": custom_stl_assets,
    }
    return summary


def write_builder_export_summary(scene_package: Path, output_path: Path | None = None) -> Path:
    scene_package = Path(scene_package)
    output = output_path or (scene_package / "generated" / "builder_export_summary.json")
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = summarize_builder_scene(scene_package)
    output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return output


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Summarize builder scene for Workcell Studio panel")
    ap.add_argument("--scene-package", required=True, type=Path)
    ap.add_argument("--output", type=Path, default=None)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    out = write_builder_export_summary(args.scene_package, args.output)
    if args.json:
        print(out.read_text(encoding="utf-8"))
    else:
        print(out)
