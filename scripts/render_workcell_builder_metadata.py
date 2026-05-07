#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path
from typing import Any

from scripts.workcell_builder_capability_catalog import load_workcell_capability_catalog
from scripts.workcell_builder_grasp_strategy import normalize_grasp_strategy


def _norm(text: str | None) -> str:
    return (text or "").strip().lower().replace("_", " ")


def _match(name: str | None, mapping: list[tuple[list[str], dict[str, Any]]]) -> dict[str, Any]:
    n = _norm(name)
    for terms, payload in mapping:
        if any(t in n for t in terms):
            return payload.copy()
    return {"capability_id": None, "family": None, "runtime_supported": False, "preview_only": True, "warning": f"Could not resolve catalog capability id for selection '{name}'"}


def _catalog_lookup(group: list[dict[str, Any]], selected_name: str | None) -> dict[str, Any]:
    selected = _norm(selected_name)
    for item in group:
        name = _norm(item.get("display_name"))
        cid = _norm(item.get("capability_id"))
        if selected and (selected in name or selected in cid):
            return item
    return {}


def render_metadata(robot: str | None, end_effector: str | None, sensor: str | None, grasp_strategy: str | None, scene_path: Path | None = None, grasp_config: dict[str, Any] | None = None) -> dict[str, Any]:
    catalog = load_workcell_capability_catalog()
    robot_cat = _catalog_lookup(catalog.get("robots", []), robot)
    ee_cat = _catalog_lookup(catalog.get("end_effectors", []), end_effector)
    sensor_cat = _catalog_lookup(catalog.get("sensors", []), sensor)

    robot_info = _match(robot, [
        (["ur5", "universal robot ur5", "universalrobot ur5"], {"capability_id": "ur5", "family": "articulated", "runtime_supported": True, "preview_only": False}),
        (["generic delta", "delta"], {"capability_id": "generic_delta_900", "family": "delta", "runtime_supported": False, "preview_only": True}),
        (["generic gantry", "cartesian", "gantry"], {"capability_id": "generic_gantry_xyz", "family": "cartesian", "runtime_supported": False, "preview_only": True}),
    ])
    ee_info = _match(end_effector, [
        (["robotiq", "2f"], {"capability_id": "robotiq_2f_85", "family": "finger", "runtime_supported": True, "preview_only": False, "required_io_signals": []}),
        (["airpick", "suction", "onrobot"], {"capability_id": "onrobot_airpick_style", "family": "suction", "runtime_supported": False, "preview_only": True, "required_io_signals": ["vacuum_enable", "vacuum_feedback"]}),
    ])
    sensor_info = _match(sensor, [(["realsense", "d435i"], {"capability_id": "realsense_d435i", "family": "depth_camera", "runtime_supported": True, "preview_only": False})])
    grasp_payload, grasp_warnings = normalize_grasp_strategy({"strategy_id": grasp_strategy, **(grasp_config or {})}, ee_info)

    warnings = [x for x in [robot_info.pop("warning", None), ee_info.pop("warning", None), sensor_info.pop("warning", None)] if x]
    warnings.extend(robot_cat.get("warnings", []))
    warnings.extend(ee_cat.get("warnings", []))
    warnings.extend(grasp_warnings)
    status = "preview_only" if robot_info.get("preview_only") or ee_info.get("preview_only") else "fake_hardware_ready"
    imported = []
    object_count = 0
    if scene_path:
        env = scene_path / "environment.yaml"
        if env.is_file():
            txt = env.read_text(encoding="utf-8")
            for line in txt.splitlines():
                if "filepath:" in line:
                    imported.append(line.split("filepath:", 1)[1].strip().strip('"'))
            object_count = txt.count("filepath:")

    return {
        "selected_capabilities": {
            "robot": {"capability_id": robot_cat.get("capability_id", robot_info.get("capability_id")), "display_name": robot_cat.get("display_name", robot)},
            "end_effector": {"capability_id": ee_cat.get("capability_id", ee_info.get("capability_id")), "display_name": ee_cat.get("display_name", end_effector)},
            "sensor": {"capability_id": sensor_cat.get("capability_id", sensor_info.get("capability_id")), "display_name": sensor_cat.get("display_name", sensor)},
            "environment_assets": [],
            "task_template": {"capability_id": None, "display_name": None},
        },
        "schema_version": "workcell_builder_metadata/v1",
        "generated_by": "workcell_builder",
        "workcell_studio_compatible": True,
        "robot": {"selected_name": robot, **robot_info},
        "end_effector": {"selected_name": end_effector, **ee_info, "runtime_io_applied": False},
        "grasp_strategy": {"selected_name": grasp_strategy, **grasp_payload},
        "sensors": [{"selected_name": sensor, **sensor_info}] if sensor else [],
        "environment": {"generated_objects_count": object_count, "imported_stl_references": imported},
        "readiness": {"status": status, "blockers": [], "warnings": warnings},
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot")
    ap.add_argument("--end-effector")
    ap.add_argument("--sensor")
    ap.add_argument("--grasp-strategy")
    ap.add_argument("--scene-path", type=Path)
    ap.add_argument("--grasp-config", type=Path)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    grasp_config = None
    if args.grasp_config and args.grasp_config.is_file():
        grasp_config = json.loads(args.grasp_config.read_text(encoding="utf-8"))
    data = render_metadata(args.robot, args.end_effector, args.sensor, args.grasp_strategy, args.scene_path, grasp_config)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
