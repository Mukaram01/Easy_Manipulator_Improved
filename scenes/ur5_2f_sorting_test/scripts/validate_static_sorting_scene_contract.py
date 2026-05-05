#!/usr/bin/env python3
"""Offline contract validator for static sorting scenes."""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import sys


SCHEMA = "static_sorting_scene_contract/v1"
DEFAULT_MANIFEST = Path("scenes/ur5_2f_sorting_test/config/sorting_manifest.yaml")
DEFAULT_PAYLOAD_OUTPUT = Path("/tmp/ur5_2f_sorting_contract_payload.json")


def _load_sibling_script_module(module_name: str):
    script_dir = Path(__file__).resolve().parent
    candidate = script_dir / f"{module_name}.py"
    unique_name = f"_ur5_2f_sorting_test_{module_name}"
    spec = importlib.util.spec_from_file_location(unique_name, candidate)
    if spec is None or spec.loader is None:
        raise ModuleNotFoundError(f"Could not load sibling script module '{module_name}'")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


payload_generator = _load_sibling_script_module("generate_static_sorting_runtime_bridge_payload")
sequence_runner = _load_sibling_script_module("static_sorting_sequence_runner")


def _resolve_manifest_path(manifest: Path) -> Path:
    if manifest.exists():
        return manifest
    fallback = Path(__file__).resolve().parents[1] / "sorting_manifest.yaml"
    if manifest == DEFAULT_MANIFEST and fallback.exists():
        return fallback
    return manifest


def _load_manifest(path: Path):
    return payload_generator.runtime_plan.load_manifest(path)


def build_report(args: argparse.Namespace) -> tuple[dict, int]:
    warnings: list[str] = []
    errors: list[str] = []
    checks = {k: False for k in [
        "manifest_loaded", "manifest_structure_valid", "no_duplicate_targets", "no_duplicate_destinations",
        "routing_valid", "payload_generated", "payload_contract_valid", "destination_poses_world_frame",
        "sequence_runner_dry_run_valid",
    ]}
    resolved_manifest = _resolve_manifest_path(args.manifest)
    report = {
        "schema": SCHEMA,
        "scene_package": args.scene_package,
        "manifest_path": str(resolved_manifest),
        "payload_path": str(args.payload_output),
        "target_count": 0,
        "destination_count": 0,
        "checks": checks,
        "targets": [],
        "warnings": warnings,
        "errors": errors,
        "result": {"status": "FAIL"},
    }

    try:
        manifest = _load_manifest(resolved_manifest)
        checks["manifest_loaded"] = True
    except Exception as exc:
        errors.append(f"Manifest load failed: {exc}")
        return report, 2

    objects = manifest.get("objects", [])
    destinations = manifest.get("destinations", [])
    routing = manifest.get("routing", [])
    if isinstance(objects, list) and isinstance(destinations, list) and isinstance(routing, list):
        checks["manifest_structure_valid"] = True
    else:
        errors.append("Manifest must include list sections: objects, destinations, routing.")
        return report, 2

    report["target_count"] = len(objects)
    report["destination_count"] = len(destinations)

    target_ids = [o.get("id") for o in objects]
    dest_ids = [d.get("id") for d in destinations]
    if len(set(target_ids)) == len(target_ids):
        checks["no_duplicate_targets"] = True
    else:
        errors.append("Duplicate target ids detected.")

    if len(set(dest_ids)) == len(dest_ids):
        checks["no_duplicate_destinations"] = True
    else:
        errors.append("Duplicate destination ids detected.")

    dest_set = set(dest_ids)
    routing_map = {}
    for obj in objects:
        oid = obj.get("id")
        obj_frame = obj.get("frame_id")
        dest = obj.get("destination")
        pick_hint = obj.get("pick_hint") or obj.get("grasp_hint")
        if not oid or not obj_frame or not dest:
            errors.append(f"Target missing required fields: {obj}")
        if not pick_hint:
            warnings.append(f"Target '{oid}' missing pick_hint/grasp_hint.")
        if dest not in dest_set:
            errors.append(f"Target '{oid}' references missing destination '{dest}'.")
        routing_map[oid] = dest

    manifest_routes = {r.get("object"): r.get("destination") for r in routing}
    if manifest_routes == routing_map:
        checks["routing_valid"] = True
    else:
        errors.append("Manifest routing does not match object destination mapping.")

    try:
        plan = payload_generator.runtime_plan.build_runtime_plan(payload_generator.runtime_plan.load_manifest(payload_generator.runtime_plan.find_manifest_path()))
        payload = payload_generator.build_payload(
            plan, "service", "grasp_requests", "grasp_tasks", "world", "robotiq_2f", selected_targets=None
        )
        args.payload_output.parent.mkdir(parents=True, exist_ok=True)
        args.payload_output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        checks["payload_generated"] = True
    except Exception as exc:
        errors.append(f"Payload generation failed: {exc}")
        payload = {}

    payload_targets = {t.get("object_id"): t for t in payload.get("grasp_task", {}).get("grasp_targets", [])}
    if payload.get("status") != "PASS":
        errors.append("Generated payload status is not PASS.")
    if payload.get("summary", {}).get("target_count") != len(objects):
        errors.append("Generated payload target_count mismatch.")

    payload_routes = {t.get("object_id"): t.get("destination_id") for t in payload_targets.values()}
    if payload_routes != routing_map:
        errors.append("Generated payload routing does not match manifest routing.")

    world_ok = True
    contract_ok = True
    for oid in target_ids:
        target = payload_targets.get(oid, {})
        gmethods = target.get("grasp_methods") or []
        gposes = (gmethods[0].get("grasp_poses") if gmethods else []) or []
        z_offset = None
        if gposes:
            xyz = gposes[0].get("xyz") if isinstance(gposes[0], dict) else None
            if isinstance(xyz, list) and len(xyz) >= 3:
                z_offset = xyz[2]
        entry = {
            "object_id": oid,
            "object_frame": next((o.get("frame_id") for o in objects if o.get("id") == oid), None),
            "destination_id": routing_map.get(oid),
            "generated_payload_present": bool(target),
            "grasp_pose_count": len(gposes),
            "top_grasp_z_offset": z_offset,
            "destination_frame": (target.get("destination_pose") or {}).get("frame_id"),
        }
        report["targets"].append(entry)

        if not target.get("object_id") or not (target.get("target_pose") or {}).get("frame_id") or not target.get("target_shape"):
            contract_ok = False
        if not gmethods or not gposes:
            contract_ok = False
        if not isinstance(z_offset, (float, int)) or float(z_offset) <= 0.0:
            contract_ok = False
            errors.append(f"Target '{oid}' has non-positive top grasp local Z offset.")
        if not target.get("destination_id") or not target.get("destination_pose"):
            contract_ok = False
        if entry["destination_frame"] != "world":
            world_ok = False
            contract_ok = False
            errors.append(f"Target '{oid}' destination_pose.frame_id must be world.")

    checks["destination_poses_world_frame"] = world_ok
    checks["payload_contract_valid"] = contract_ok and checks["payload_generated"]

    try:
        original_manual_build_report = sequence_runner.manual_executor.build_report
        sequence_runner.manual_executor.build_report = lambda _ns: ({"result": {"status": "dry_run_only"}}, 0)
        s_args = argparse.Namespace(json=True, print_commands=False, target=None, all_targets=True, payload_output_dir=Path("/tmp/ur5_2f_sorting_contract_sequence"), require_active_runtime=False, execute=False, confirm_runtime_send=False, allow_batch_runtime_send=False)
        seq_report, seq_rc = sequence_runner.build_report(s_args)
        if seq_rc == 0 and seq_report.get("mode") == "dry_run" and not seq_report.get("safety", {}).get("robot_motion_requested", True):
            checks["sequence_runner_dry_run_valid"] = True
        else:
            errors.append("Sequence runner dry-run contract failed.")
        sequence_runner.manual_executor.build_report = original_manual_build_report
    except Exception as exc:
        errors.append(f"Sequence runner dry-run failed: {exc}")

    if errors:
        status = "FAIL"
        rc = 2
    elif warnings:
        status = "WARN"
        rc = 2 if args.strict else 1
    else:
        status = "PASS"
        rc = 0
    report["result"] = {"status": status}
    return report, rc


def parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--json", action="store_true")
    p.add_argument("--scene-package", default="ur5_2f_sorting_test")
    p.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    p.add_argument("--payload-output", type=Path, default=DEFAULT_PAYLOAD_OUTPUT)
    p.add_argument("--strict", action="store_true")
    p.add_argument("--print-summary", action="store_true")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    report, rc = build_report(args)
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print("Static sorting scene contract validation (offline)")
        print(f"status: {report['result']['status']}")
        print(f"checks passed: {sum(1 for v in report['checks'].values() if v)}/{len(report['checks'])}")
        if args.print_summary:
            print(json.dumps(report, indent=2))
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
