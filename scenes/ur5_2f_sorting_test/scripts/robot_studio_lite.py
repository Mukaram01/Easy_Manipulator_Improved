#!/usr/bin/env python3
"""Robot Studio Lite CLI dashboard for ur5_2f_sorting_test dry-run workflow."""

from __future__ import annotations

import argparse
import contextlib
import io
import importlib.machinery
import importlib.util
import json
from pathlib import Path
import sys

SCHEMA = "robot_studio_lite_dashboard/v1"
SCENE_PACKAGE = "ur5_2f_sorting_test"
MODE = "dry_run"


def _load_sibling_script_module(module_name: str):
    """Load sibling helper script from source or installed ROS 2 executable layout."""
    script_dir = Path(__file__).resolve().parent
    candidates = [
        script_dir / f"{module_name}.py",
        script_dir / module_name,
    ]

    for candidate in candidates:
        if not candidate.exists():
            continue

        unique_name = f"_ur5_2f_sorting_test_{module_name}"
        if candidate.suffix == ".py":
            spec = importlib.util.spec_from_file_location(unique_name, candidate)
            if spec is None or spec.loader is None:
                continue
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module

        loader = importlib.machinery.SourceFileLoader(unique_name, str(candidate))
        spec = importlib.util.spec_from_loader(unique_name, loader)
        if spec is None:
            continue
        module = importlib.util.module_from_spec(spec)
        loader.exec_module(module)
        return module

    searched = ", ".join(str(candidate) for candidate in candidates)
    raise ModuleNotFoundError(f"Could not load sibling script module '{module_name}'. Searched: {searched}")


validate_scene_layout = _load_sibling_script_module("validate_scene_layout")
runtime_plan = _load_sibling_script_module("generate_sorting_runtime_plan")
bridge_payload = _load_sibling_script_module("generate_sorting_emd_bridge_payload")
bridge_from_detections = _load_sibling_script_module("generate_bridge_payload_from_detections")
handoff_preview = _load_sibling_script_module("preview_sorting_execution_handoff")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quick", action="store_true", help="Only show manifest and safety status")
    parser.add_argument("--json", action="store_true", help="Print dashboard as JSON")
    parser.add_argument("--detections", type=Path, help="Path to detected_objects/v1 JSON fixture")
    parser.add_argument("--skip-layout-validation", action="store_true", help="Skip validate_scene_layout")
    parser.add_argument("--output", type=Path, help="Optional output file for JSON dashboard")
    return parser.parse_args(argv)


def _default_detections_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        share_dir = Path(get_package_share_directory(SCENE_PACKAGE))
        candidate = share_dir / "detected_objects_sample.json"
        if candidate.exists():
            return candidate
    except Exception:
        pass

    return Path(__file__).resolve().parents[1] / "fixtures" / "detected_objects_sample.json"


def _build_manifest_summary(plan: dict) -> dict:
    place_steps = [step for step in plan.get("steps", []) if step.get("type") == "place"]
    routes = [
        {
            "object_id": step.get("object_id"),
            "destination_id": step.get("destination_id"),
            "destination_frame": step.get("destination_frame"),
        }
        for step in place_steps
    ]
    return {"routes": routes}


def _build_detection_summary(payload: dict, source: Path) -> dict:
    routes = []
    for target in payload.get("targets", []):
        destination = target.get("destination", {})
        metadata = target.get("metadata", {})
        routes.append(
            {
                "detected_object_id": metadata.get("detected_object_id", target.get("object_id")),
                "class_label": metadata.get("class_label"),
                "destination_id": destination.get("id"),
                "destination_frame": destination.get("frame_id"),
            }
        )
    return {"source": str(source), "routes": routes}


def build_dashboard(args: argparse.Namespace) -> dict:
    dashboard = {
        "schema": SCHEMA,
        "scene_package": SCENE_PACKAGE,
        "mode": MODE,
        "safety": {
            "robot_motion_requested": False,
            "live_epd_called": False,
            "execution_enabled": False,
        },
        "next_commands": [
            "ros2 launch ur5_2f_sorting_test demo.launch.py",
            "ros2 run ur5_2f_sorting_test generate_sorting_runtime_plan --json",
            "ros2 run ur5_2f_sorting_test generate_bridge_payload_from_detections --detections <path> --json",
            "ros2 run ur5_2f_sorting_test preview_sorting_execution_handoff --from-static-manifest --json",
        ],
    }

    if args.quick:
        manifest = runtime_plan.load_manifest(runtime_plan.find_manifest_path())
        plan = runtime_plan.build_runtime_plan(manifest)
        dashboard["layout_validation"] = {"status": "skipped_quick_mode"}
        dashboard["static_manifest"] = _build_manifest_summary(plan)
        dashboard["offline_detections"] = {"source": None, "routes": []}
        dashboard["bridge_payload"] = {"schema": "emd_grasp_bridge_payload/v1", "target_count": 0}
        dashboard["handoff_preview"] = {
            "schema": "sorting_execution_handoff_preview/v1",
            "execution_ready_preview": False,
            "robot_motion_requested": False,
            "requires_manual_execution_enable": True,
        }
        return dashboard

    if args.skip_layout_validation:
        layout_validation = {"status": "skipped"}
    else:
        with contextlib.redirect_stdout(io.StringIO()):
            validate_scene_layout.main()
        layout_validation = {"status": "pass"}

    manifest = runtime_plan.load_manifest(runtime_plan.find_manifest_path())
    plan = runtime_plan.build_runtime_plan(manifest)
    static_bridge_payload = bridge_payload.build_bridge_payload(plan)

    detections_path = args.detections if args.detections is not None else _default_detections_path()
    _runtime_plan_from_detections, detections_bridge_payload = bridge_from_detections.build_payload(
        detections_path, bridge_from_detections.runtime_plan_from_detections.DEFAULT_MIN_CONFIDENCE
    )
    preview = handoff_preview.build_handoff_preview(detections_bridge_payload, "detections")

    dashboard["layout_validation"] = layout_validation
    dashboard["static_manifest"] = _build_manifest_summary(plan)
    dashboard["offline_detections"] = _build_detection_summary(detections_bridge_payload, detections_path)
    dashboard["bridge_payload"] = {
        "schema": static_bridge_payload.get("schema"),
        "target_count": len(static_bridge_payload.get("targets", [])),
    }
    dashboard["handoff_preview"] = {
        "schema": preview.get("schema"),
        "execution_ready_preview": preview.get("execution_ready_preview"),
        "robot_motion_requested": preview.get("robot_motion_requested"),
        "requires_manual_execution_enable": preview.get("requires_manual_execution_enable"),
    }
    return dashboard


def _print_text_dashboard(dashboard: dict) -> None:
    print("=== Robot Studio Lite :: ur5_2f_sorting_test ===")
    print(f"scene_package: {dashboard['scene_package']}")
    print(f"mode: {dashboard['mode']}")
    print()
    print(f"layout_validation: {dashboard['layout_validation']['status']}")
    print("static_manifest routes:")
    for route in dashboard["static_manifest"].get("routes", []):
        print(f"  - {route.get('object_id')} -> {route.get('destination_id')} ({route.get('destination_frame')})")

    print("offline_detections routes:")
    source = dashboard["offline_detections"].get("source")
    if source:
        print(f"  source: {source}")
    for route in dashboard["offline_detections"].get("routes", []):
        print(
            f"  - {route.get('detected_object_id')} ({route.get('class_label')}) -> "
            f"{route.get('destination_id')} ({route.get('destination_frame')})"
        )

    bridge = dashboard["bridge_payload"]
    print(f"bridge_payload: {bridge.get('schema')} targets={bridge.get('target_count')}")
    handoff = dashboard["handoff_preview"]
    print(
        "handoff_preview: "
        f"{handoff.get('schema')} ready={handoff.get('execution_ready_preview')} "
        f"robot_motion_requested={handoff.get('robot_motion_requested')}"
    )
    print("safety:")
    for key, value in dashboard["safety"].items():
        print(f"  {key}: {value}")

    print("next_commands:")
    for command in dashboard["next_commands"]:
        print(f"  - {command}")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    dashboard = build_dashboard(args)
    dashboard_json = json.dumps(dashboard, indent=2)

    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(dashboard_json + "\n", encoding="utf-8")

    if args.json:
        print(dashboard_json)
    else:
        _print_text_dashboard(dashboard)

    return 0


if __name__ == "__main__":
    sys.exit(main())
