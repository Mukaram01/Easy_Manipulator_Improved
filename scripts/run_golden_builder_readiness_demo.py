#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, shutil, subprocess, sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent


def _run_json(cmd: list[str], label: str) -> dict[str, Any]:
    run = subprocess.run(cmd, capture_output=True, text=True, check=False)
    try:
        payload = json.loads(run.stdout) if run.stdout.strip() else {}
    except Exception:
        payload = {"status": "FAIL", "errors": [f"{label} returned non-JSON output", run.stdout.strip(), run.stderr.strip()]}
    payload["returncode"] = run.returncode
    payload["command"] = cmd
    return payload


def main() -> int:
    ap = argparse.ArgumentParser(description="Canonical offline builder→readiness demo flow")
    ap.add_argument("--scene-package", type=Path, default=REPO_ROOT / "scenes" / "ur5_2f_test")
    ap.add_argument("--output-dir", type=Path, required=True)
    ap.add_argument("--project-name", default="golden_builder_readiness_demo")
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    scene = args.scene_package.resolve()
    output = args.output_dir.resolve()
    generated = scene / "generated"
    generated.mkdir(parents=True, exist_ok=True)
    if output.exists() and args.force:
        shutil.rmtree(output)

    intent_path = generated / "workcell_builder_task_intent.yaml"
    steps: dict[str, Any] = {}
    steps["create_or_update_builder_task_intent"] = _run_json([
        sys.executable, str(SCRIPT_DIR / "create_or_update_builder_task_intent.py"),
        "--scene-package", str(scene),
        "--task-id", "golden_pick_place_001",
        "--task-type", "pick_place",
        "--pick-source", "pick_zone_main",
        "--place-target", "bin_red",
        "--grasp-strategy", "finger_pinch_basic",
        "--approach-axis", "z_down",
        "--approach-distance-m", "0.12",
        "--retreat-axis", "z_up",
        "--retreat-distance-m", "0.10",
        "--release-strategy", "tool_release",
        "--output", str(intent_path),
        "--validate", "--json",
    ], "create task intent")

    steps["validate_builder_generated_scene"] = _run_json([sys.executable, str(SCRIPT_DIR / "validate_builder_generated_scene.py"), str(scene), "--json"], "scene validation")

    steps["generate_workcell_studio_readiness_pack"] = _run_json([
        sys.executable, str(SCRIPT_DIR / "generate_workcell_studio_readiness_pack.py"),
        "--scene-package", str(scene),
        "--output-dir", str(output),
        "--project-name", args.project_name,
        "--validate", "--prepare-rviz-preview", "--smoke-dry-run", "--force", "--json",
    ], "readiness pack")

    manifest = output / "readiness_pack_manifest.json"
    manifest_json = json.loads(manifest.read_text(encoding="utf-8")) if manifest.exists() else {}
    artifacts = manifest_json.get("artifacts", {})

    def _path_exists(value: Any) -> bool:
        return isinstance(value, str) and Path(value).exists()

    rviz_session_path = artifacts.get("rviz_moveit_plan_preview_session")
    rviz_session = json.loads(Path(rviz_session_path).read_text(encoding="utf-8")) if _path_exists(rviz_session_path) else {}
    rviz_readiness = (rviz_session.get("readiness") or {}) if isinstance(rviz_session, dict) else {}
    rviz_inputs = (((rviz_session.get("rviz_moveit") or {}).get("expected_inputs")) or {}) if isinstance(rviz_session, dict) else {}
    plan_preview = (rviz_session.get("plan_preview") or {}) if isinstance(rviz_session, dict) else {}
    launch_cmd = ((((rviz_session.get("rviz_moveit") or {}).get("suggested_launch")) or {}).get("command")) if isinstance(rviz_session, dict) else None

    rviz_blockers = list(rviz_readiness.get("blockers") or [])
    rviz_warnings = list(rviz_readiness.get("warnings") or [])
    rviz_status = rviz_readiness.get("status", "WARN")
    rviz_classification = "rviz_preview_ready" if rviz_status == "PASS" and not rviz_blockers else "rviz_preview_partial"

    visual_markers_path = artifacts.get("static_preview", {}).get("markers") if isinstance(artifacts.get("static_preview"), dict) else None
    visual_markers = json.loads(Path(visual_markers_path).read_text(encoding="utf-8")) if _path_exists(visual_markers_path) else {}

    rviz_preview_readiness = {
        "status": rviz_status,
        "classification": rviz_classification,
        "robot_description": bool(rviz_inputs.get("scene_package_exists")),
        "end_effector_metadata": bool(plan_preview.get("grasp_strategy")),
        "support_surface_or_table": bool(plan_preview.get("pick_source_id")),
        "pick_zone": bool(plan_preview.get("pick_source_id")),
        "place_zone": bool(plan_preview.get("place_target_id")),
        "task_flow_markers": bool((manifest_json.get("artifacts", {}) or {}).get("task_flow_summary")),
        "fake_hardware_launch_command": bool(launch_cmd),
        "rviz_config_or_preview_markers": _path_exists(artifacts.get("static_preview", {}).get("summary") if isinstance(artifacts.get("static_preview"), dict) else None),
        "preview_command": launch_cmd,
        "blockers": rviz_blockers,
        "warnings": rviz_warnings,
    }

    results = {
        "result": "PASS" if manifest.exists() else "FAIL",
        "scene_package": str(scene),
        "output_dir": str(output),
        "steps": steps,
        "artifacts": artifacts,
        "readiness": manifest_json.get("results", {}),
        "rviz_moveit_preview": rviz_preview_readiness,
        "visual_preview": {
            "present": bool(visual_markers),
            "marker_count": int(visual_markers.get("marker_count", 0)) if isinstance(visual_markers, dict) else 0,
            "task_flow_marker_count": int(visual_markers.get("task_flow_marker_count", 0)) if isinstance(visual_markers, dict) else 0,
            "safety_banner_present": bool((json.loads(Path(artifacts.get("static_preview", {}).get("summary", "")).read_text(encoding="utf-8")) if _path_exists(artifacts.get("static_preview", {}).get("summary") if isinstance(artifacts.get("static_preview"), dict) else None) else {}).get("safety_banner_present", False)),
            "paths": {
                "dashboard": artifacts.get("readiness_dashboard"),
                "static_preview_html": artifacts.get("static_preview", {}).get("html") if isinstance(artifacts.get("static_preview"), dict) else None,
                "static_preview_svg": artifacts.get("static_preview", {}).get("svg") if isinstance(artifacts.get("static_preview"), dict) else None,
                "marker_artifact": visual_markers_path,
            },
        },
        "safety": {
            "use_fake_hardware": True,
            "real_hardware_enabled": False,
            "motion_command_sent": False,
            "runtime_execution_called": False,
            "moveit_plan_service_called": False,
            **(manifest_json.get("safety", {}) if isinstance(manifest_json.get("safety", {}), dict) else {}),
        },
        "next_commands": [
            f"python3 scripts/validate_builder_generated_scene.py {scene}",
            f"python3 scripts/workcell_studio.py validate-readiness-pack --manifest {manifest} --json",
            f"python3 scripts/workcell_studio.py generate-readiness-dashboard --manifest {manifest} --output {output / 'readiness_dashboard_regenerated.html'} --json",
            f"python3 scripts/run_fake_hardware_smoke_launch.py --session {output / 'plan_preview' / 'rviz_moveit_plan_preview_session.json'} --output-dir {output / 'smoke_launch'} --dry-run --json",
        ],
    }
    (output / "golden_builder_demo_summary.json").write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    if args.json:
        print(json.dumps(results, indent=2))
    return 0 if results["result"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
