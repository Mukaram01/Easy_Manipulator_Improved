#!/usr/bin/env python3
"""MVP-1 generated-cell acceptance flow (offline-first, ROS-optional)."""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

SCRIPTS_DIR = Path(__file__).resolve().parent
VALIDATE_OBJECTS = SCRIPTS_DIR / "validate_detected_objects.py"
VALIDATE_RECIPE = SCRIPTS_DIR / "validate_task_recipe.py"
ADAPTER = SCRIPTS_DIR / "run_task_recipe_adapter.py"
BRIDGE = SCRIPTS_DIR / "convert_runtime_plan_to_emd_grasp.py"

KNOWN_RUNTIME_BOUNDARY = (
    "destination_resolved is present in the bridge payload; runtime release execution may still "
    "use existing release fallback until runtime destination support is implemented."
)


def _run(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, capture_output=True, text=True, check=False)


def _extract_json(text: str) -> dict[str, Any]:
    start = text.find("{")
    return {} if start < 0 else json.loads(text[start:])


def _scene_readiness(scene_package: str) -> tuple[list[str], list[str]]:
    warnings: list[str] = []
    errors: list[str] = []
    candidates = [
        Path.cwd() / "scenes" / scene_package,
        Path.cwd() / "install" / scene_package / "share" / scene_package,
        Path.cwd() / "install" / "share" / scene_package,
    ]
    existing = next((p for p in candidates if p.exists()), None)
    if existing is None:
        warnings.append(f"Scene package '{scene_package}' not found under workspace scenes/install paths.")
        return warnings, errors

    required = ["package.xml", "environment.yaml"]
    optional = ["launch", "config"]
    for name in required:
        if not (existing / name).exists():
            warnings.append(f"Scene package '{scene_package}' missing expected file: {name}")
    for name in optional:
        if not (existing / name).exists():
            warnings.append(f"Scene package '{scene_package}' missing expected directory: {name}")
    return warnings, errors


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-package", required=True)
    parser.add_argument("--task-recipe", type=Path, required=True)
    parser.add_argument("--detected-objects", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=Path("reports/generated_cell_acceptance"))
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    runtime_plan = args.output_dir / "runtime_execution_plan.json"
    bridge_payload = args.output_dir / "emd_grasp_bridge_payload.json"

    validate_objects_cmd = [sys.executable, str(VALIDATE_OBJECTS), str(args.detected_objects), "--json"]
    validate_recipe_cmd = [sys.executable, str(VALIDATE_RECIPE), str(args.task_recipe), "--json"]
    if args.strict:
        validate_objects_cmd.append("--strict")
        validate_recipe_cmd.append("--strict")

    obj_proc = _run(validate_objects_cmd)
    recipe_proc = _run(validate_recipe_cmd)

    adapter_cmd = [
        sys.executable,
        str(ADAPTER),
        "--task-recipe",
        str(args.task_recipe),
        "--objects",
        str(args.detected_objects),
        "--output",
        str(runtime_plan),
        "--json",
        "--mode",
        "offline",
    ]
    if args.strict:
        adapter_cmd.append("--strict")
    adapter_proc = _run(adapter_cmd)

    bridge_cmd = [
        sys.executable,
        str(BRIDGE),
        "--runtime-plan",
        str(runtime_plan),
        "--output",
        str(bridge_payload),
        "--json",
        "--mode",
        "offline",
    ]
    if args.strict:
        bridge_cmd.append("--strict")
    bridge_proc = _run(bridge_cmd)

    obj_json = _extract_json(obj_proc.stdout)
    recipe_json = _extract_json(recipe_proc.stdout)
    plan_json = _extract_json(adapter_proc.stdout)
    bridge_json = _extract_json(bridge_proc.stdout)

    warnings: list[str] = []
    blockers: list[str] = []
    scene_warnings, scene_errors = _scene_readiness(args.scene_package)
    warnings.extend(scene_warnings)
    blockers.extend(scene_errors)

    if obj_proc.returncode != 0:
        blockers.append("Detected objects validation failed.")
    if recipe_proc.returncode != 0:
        blockers.append("Task recipe validation failed.")
    if adapter_proc.returncode != 0:
        blockers.append("Runtime plan generation failed.")
    if bridge_proc.returncode != 0:
        blockers.append("EMD grasp bridge conversion failed.")

    warnings.extend(bridge_json.get("warnings", []))
    files = recipe_json.get("files")
    if isinstance(files, list):
        for item in files:
            if isinstance(item, dict):
                for warning in item.get("warnings", []):
                    if isinstance(warning, str):
                        warnings.append(warning)
    warnings.append(KNOWN_RUNTIME_BOUNDARY)

    status = "PASS"
    if blockers:
        status = "FAIL"
    elif warnings:
        status = "WARN"
    if args.strict and warnings:
        status = "FAIL"
        blockers.append("Strict mode treats warnings as blockers.")

    steps = plan_json.get("steps", []) if isinstance(plan_json.get("steps"), list) else []
    selected = steps[0] if steps else {}
    selected_obj = selected.get("object", {}) if isinstance(selected.get("object"), dict) else {}
    routing = selected.get("routing", {}) if isinstance(selected.get("routing"), dict) else {}
    dest = routing.get("destination", {}) if isinstance(routing.get("destination"), dict) else {}

    payload = {
        "schema_version": "generated_cell_acceptance/v1",
        "status": status,
        "scene_package": args.scene_package,
        "selected_object": selected_obj.get("id"),
        "matched_rule": routing.get("matched_rule_id"),
        "destination_selected": routing.get("destination_id"),
        "destination_pose": {"frame": dest.get("frame"), "xyz": dest.get("pose_xyz")},
        "runtime_plan_path": str(runtime_plan),
        "emd_bridge_payload_path": str(bridge_payload),
        "warnings": warnings,
        "blockers": blockers,
        "step_status": {
            "detected_objects": obj_json.get("status"),
            "task_recipe": recipe_json.get("result"),
            "runtime_plan": "PASS" if adapter_proc.returncode == 0 else "FAIL",
            "emd_bridge": bridge_json.get("status"),
        },
    }

    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(f"Generated cell acceptance: {status}")
        print(f"- selected object: {payload['selected_object']}")
        print(f"- matched rule: {payload['matched_rule']}")
        print(f"- destination selected: {payload['destination_selected']}")
        print(f"- destination pose/frame: {payload['destination_pose']}")
        print(f"- generated runtime plan path: {runtime_plan}")
        print(f"- generated EMD bridge payload path: {bridge_payload}")
        for w in warnings:
            print(f"WARN: {w}")
        for b in blockers:
            print(f"FAIL: {b}")

    return 1 if status == "FAIL" else 0


if __name__ == "__main__":
    raise SystemExit(main())
