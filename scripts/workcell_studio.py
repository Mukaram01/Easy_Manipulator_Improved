#!/usr/bin/env python3
"""Workcell Studio CLI orchestration tools."""
from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
DEMO_CATALOG_PATH = REPO_ROOT / "catalog" / "workcell_studio_demos.yaml"


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        import yaml
        loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
        return loaded if isinstance(loaded, dict) else {}
    except Exception:
        if str(SCRIPT_DIR) not in sys.path:
            sys.path.insert(0, str(SCRIPT_DIR))
        import validate_cell_definition as yaml_support

        loaded, _, _ = yaml_support.load_yaml(path)
        return loaded if isinstance(loaded, dict) else {}


def _run_json(cmd: list[str], label: str) -> tuple[int, dict[str, Any]]:
    proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
    payload: dict[str, Any]
    try:
        payload = json.loads(proc.stdout) if proc.stdout.strip() else {}
    except Exception:
        payload = {"result": "FAIL", "errors": [f"{label} returned non-JSON output", proc.stdout.strip(), proc.stderr.strip()]}
    payload.setdefault("stdout", proc.stdout.strip())
    payload.setdefault("stderr", proc.stderr.strip())
    payload.setdefault("returncode", proc.returncode)
    return proc.returncode, payload




def _generate_static_preview(cell_def: Path, output_dir: Path, title: str, environment_layout: Path | None = None) -> tuple[int, dict[str, Any]]:
    cmd = [sys.executable, str(SCRIPT_DIR / "generate_workcell_static_preview.py"), "--cell-definition", str(cell_def), "--output-dir", str(output_dir), "--title", title, "--json"]
    if environment_layout:
        cmd.extend(["--environment-layout", str(environment_layout)])
    return _run_json(cmd, "static preview")


def _catalog_index() -> dict[str, dict[str, dict[str, Any]]]:
    def _entries(group: str, key: str) -> dict[str, dict[str, Any]]:
        out: dict[str, dict[str, Any]] = {}
        for path in sorted((REPO_ROOT / "catalog" / "capabilities" / group).glob("*.yaml")):
            payload = _load_yaml(path)
            data = payload.get(key, {}) if isinstance(payload.get(key), dict) else {}
            if data.get("id"):
                out[data["id"]] = {"id": data["id"], "family": data.get("family"), "compatible_families": data.get("compatible_tool_families") or data.get("supported_task_families") or []}
        return out
    grasp: dict[str, dict[str, Any]] = {}
    for path in sorted((REPO_ROOT / "catalog" / "grasp_strategies").glob("*.yaml")):
        payload = _load_yaml(path)
        data = payload.get("grasp_strategy", {}) if isinstance(payload.get("grasp_strategy"), dict) else {}
        if data.get("id"):
            grasp[data["id"]] = {"id": data["id"], "compatible_families": data.get("compatible_tool_families") or []}
    return {"robots": _entries("robots", "robot"), "end_effectors": _entries("end_effectors", "end_effector"), "sensors": _entries("sensors", "sensor"), "tasks": _entries("tasks", "task"), "grasp_strategies": grasp}


def _resolve(group: str, value: str, ids: dict[str, dict[str, Any]]) -> tuple[str, list[str]]:
    aliases = {"robotiq_2f": "robotiq_2f_85", "suction": "onrobot_airpick_style", "realsense_d435i": "intel_realsense_d435i", "sort_by_colour": "task_magnetic_pick_place"}
    resolved = aliases.get(value, value)
    if resolved not in ids:
        raise ValueError(f"Unknown {group}: {value}")
    out = []
    if resolved != value:
        out.append(f"Resolved alias '{value}' -> '{resolved}'")
    return resolved, out


def _default_environment_layout(cell_id: str, sensor_id: str, task_id: str) -> dict[str, Any]:
    assets = [{"id": "main_table", "asset_ref": "table_standard_1200", "type": "table", "pose": {"frame": "world", "xyz": [0.5, 0.0, 0.0], "rpy": [0, 0, 0]}}, {"id": "overhead_sensor", "asset_ref": sensor_id, "type": "sensor", "pose": {"frame": "world", "xyz": [0.65, 0.0, 1.2], "rpy": [0, 0, 0]}}]
    if "conveyor" in task_id:
        assets.append({"id": "optional_conveyor", "asset_ref": "conveyor_2m", "type": "conveyor", "pose": {"frame": "world", "xyz": [1.2, 0.0, 0.0], "rpy": [0, 0, 0]}})
    return {"schema_version": "environment_layout/v1", "layout_id": f"{cell_id}_layout", "metadata": {"name": f"{cell_id} generated layout", "generated_defaults": True, "placement_accuracy": "approximate_defaults_for_preview_only"}, "assets": assets, "zones": [{"id": "pick_zone", "type": "pick", "frame": "world", "bounds_xyz": {"min": [0.2, -0.5, 0.35], "max": [0.8, -0.1, 0.75]}}, {"id": "bin_a_zone", "type": "place", "frame": "world", "bounds_xyz": {"min": [0.55, 0.2, 0.35], "max": [0.8, 0.45, 0.75]}}, {"id": "bin_b_zone", "type": "place", "frame": "world", "bounds_xyz": {"min": [0.55, -0.45, 0.35], "max": [0.8, -0.2, 0.75]}}]}


def create_cell(args: argparse.Namespace) -> int:
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    import create_cell_definition_wizard as wizard
    out = args.output_dir.resolve()
    if out.exists() and args.force:
        shutil.rmtree(out)
    out.mkdir(parents=True, exist_ok=True)
    idx = _catalog_index()
    warnings: list[str] = []
    robot_id, w = _resolve("robot", args.robot, idx["robots"]); warnings += w
    ee_id, w = _resolve("end_effector", args.end_effector, idx["end_effectors"]); warnings += w
    sensor_id, w = _resolve("sensor", args.sensor, idx["sensors"]); warnings += w
    task_id, w = _resolve("task", args.task, idx["tasks"]); warnings += w
    grasp_id, w = _resolve("grasp_strategy", args.grasp_strategy, idx["grasp_strategies"]); warnings += w
    blockers = []
    compat = "compatible"
    allowed_fams = idx["end_effectors"][ee_id]["compatible_families"] or []
    robot_family = idx["robots"][robot_id].get("family")
    if allowed_fams and robot_family and robot_family not in allowed_fams:
        blockers.append(f"Tool {ee_id} incompatible with robot family {robot_family}")
    grasp_fams = idx["grasp_strategies"][grasp_id]["compatible_families"] or []
    if grasp_fams and idx["end_effectors"][ee_id].get("family") not in grasp_fams:
        blockers.append(f"Grasp {grasp_id} incompatible with end-effector family")
    if blockers and not args.allow_incompatible:
        print(json.dumps({"result": "FAIL", "blockers": blockers}, indent=2))
        return 2
    if blockers:
        compat = "incompatible_allowed_preview_only"
    cell_def_data = {"schema_version": "cell_definition/v1", "cell": {"id": args.cell_id, "name": args.cell_id, "description": "Generated by Workcell Studio create-cell wizard"}, "robot": wizard.ROBOT_PRESETS.get(args.robot, wizard.ROBOT_PRESETS["ur5"]), "end_effector": {"id": ee_id, "type": "suction" if "suction" in ee_id or "airpick" in ee_id else "finger", "brand": "catalog", "grasp_frame": "tool0", "allowed_touch_links": []}, "camera": {"id": sensor_id, "type": "depth_camera", "frame": "camera_depth_optical_frame"}, "environment": {"frame": "world", "support_surfaces": [{"id": "table_main", "type": "table", "frame": "world", "pose_xyz": [0.0, 0.0, 0.0], "pose_rpy": [0, 0, 0], "dimensions": [1.0, 1.0, 0.05]}]}, "objects": [{"id": "spawn_object", "class": "box", "shape": "box", "color": "unknown", "material": "unknown", "frame": "world", "dimensions": [0.05, 0.05, 0.05], "pose_xyz": [0.45, 0.0, 0.08], "pose_rpy": [0, 0, 0]}], "task": {"id": task_id, "type": "sort_by_colour", "source_object": "spawn_object", "destinations": [{"id": "bin_a", "frame": "world", "pose_xyz": [0.30, 0.30, 0.1], "pose_rpy": [0, 0, 0]}, {"id": "bin_b", "frame": "world", "pose_xyz": [0.30, -0.30, 0.1], "pose_rpy": [0, 0, 0]}], "rules": [{"id": "route_default", "when": {"always": True}, "destination": "bin_a"}]}, "grasp": {"strategy_ref": grasp_id}, "commissioning": {"self_test_enabled": True, "export_bundle": True, "require_operator_review": True}}
    cell_def_path = out / "cell_definition.yaml"
    cell_def_path.write_text(wizard.to_yaml_text(cell_def_data), encoding="utf-8")
    layout_path = out / "environment_layout.yaml"
    layout_path.write_text(wizard.to_yaml_text(_default_environment_layout(args.cell_id, sensor_id, task_id)), encoding="utf-8")
    validation = {}
    if args.validate:
        _, validation = _run_json([sys.executable, str(SCRIPT_DIR / "validate_cell_definition.py"), str(cell_def_path), "--json"], "validate")
    preview = {}
    if args.preview:
        _, preview = _generate_static_preview(cell_def_path, out / "preview", args.cell_id, layout_path)
    bundle_path = ""
    if args.generate_bundle:
        subprocess.run([sys.executable, str(SCRIPT_DIR / "create_workcell_project.py"), "--cell-definition", str(cell_def_path), "--output-dir", str(out / "project"), "--force"], check=False)
        bundle_path = str(out / "project")
    runtime_mode = "fake_hardware_ready" if robot_id == "ur5" and not blockers else "preview_only"
    summary = {"selected": {"robot": args.robot, "end_effector": args.end_effector, "sensor": args.sensor, "task": args.task, "grasp_strategy": args.grasp_strategy}, "resolved_catalog_ids": {"robot": robot_id, "end_effector": ee_id, "sensor": sensor_id, "task": task_id, "grasp_strategy": grasp_id}, "compatibility_status": compat, "warnings": warnings, "blockers": blockers, "cell_definition_path": str(cell_def_path), "environment_layout_path": str(layout_path), "validation": validation, "preview_paths": {"svg": str(out / "preview" / "static_preview.svg"), "html": str(out / "preview" / "static_preview.html"), "summary_json": str(out / "preview" / "static_preview_summary.json")}, "generated_bundle_project_path": bundle_path, "runtime_mode": runtime_mode, "safety_status": {"fake_hardware_default": True, "runtime_io_applied": False, "motion_started": False, "ros_launch_started": False}}
    (out / "create_cell_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    (out / "create_cell_summary.md").write_text("# Create Cell Summary\n\n- Runtime mode: `%s`\n- Compatibility: `%s`\n- Cell definition: `%s`\n- Environment layout: `%s`\n" % (runtime_mode, compat, cell_def_path, layout_path), encoding="utf-8")
    print(json.dumps({"result": "PASS", "summary": str(out / "create_cell_summary.json")}, indent=2))
    return 0

def load_demo_catalog() -> list[dict[str, Any]]:
    if not DEMO_CATALOG_PATH.is_file():
        return []
    payload = _load_yaml(DEMO_CATALOG_PATH)
    demos = payload.get("demos") if isinstance(payload.get("demos"), list) else []
    out = []
    for item in demos:
        if isinstance(item, dict) and item.get("id") and item.get("cell_definition"):
            out.append(item)
    return out


def list_demos(args: argparse.Namespace) -> int:
    demos = load_demo_catalog()
    if args.json:
        print(json.dumps({"catalog": str(DEMO_CATALOG_PATH), "demos": demos}, indent=2))
        return 0
    print("Workcell Studio Demo Gallery")
    print(f"Catalog: {DEMO_CATALOG_PATH}")
    for demo in demos:
        tags = ",".join(demo.get("tags", []))
        print(f"- {demo['id']}: {demo.get('title','')} | runtime_mode={demo.get('runtime_mode','unknown')} | tags=[{tags}]")
    return 0


def generate_demo_bundle(args: argparse.Namespace) -> int:
    demos = load_demo_catalog()
    by_id = {d["id"]: d for d in demos}
    selected = demos if args.all else ([by_id.get(args.demo_id)] if args.demo_id else [])
    selected = [s for s in selected if s]
    if not selected:
        print(json.dumps({"result": "FAIL", "error": f"Unknown or missing demo id: {args.demo_id}"}, indent=2))
        return 2

    output_root = args.output_dir.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    failures = []
    for demo in selected:
        demo_dir = output_root / demo["id"]
        if demo_dir.exists() and args.force:
            shutil.rmtree(demo_dir)
        demo_dir.mkdir(parents=True, exist_ok=True)
        cell_def = (REPO_ROOT / demo["cell_definition"]).resolve()
        if not cell_def.is_file():
            msg = f"Missing cell_definition for demo {demo['id']}: {cell_def}"
            failures.append(msg)
            if not args.continue_on_error:
                print(msg)
                return 3
            continue

        _, val = _run_json([sys.executable, str(SCRIPT_DIR / "validate_cell_definition.py"), str(cell_def), "--json"], "validate")
        if val.get("result") == "FAIL":
            msg = f"Invalid cell_definition for demo {demo['id']}"
            failures.append(msg)
            if not args.continue_on_error:
                print(msg)
                return 4

        project_rc = subprocess.run([
            sys.executable, str(SCRIPT_DIR / "create_workcell_project.py"),
            "--cell-definition", str(cell_def), "--output-dir", str(demo_dir), "--force"
        ], capture_output=True, text=True, check=False)

        copied_cell = demo_dir / "cell_definition.yaml"
        shutil.copy2(cell_def, copied_cell)
        preview_dir = demo_dir / "preview"
        _, preview_payload = _generate_static_preview(cell_def, preview_dir, demo.get("title", demo["id"]))
        manifests = list(demo_dir.glob("*/project_manifest.json"))
        project_path = str(manifests[0].parent) if manifests else ""
        summary = {
            "result": "PASS" if project_rc.returncode == 0 else "FAIL",
            "demo": demo,
            "runtime_mode": demo.get("runtime_mode"),
            "preview_only": demo.get("runtime_mode") == "preview_only",
            "source_cell_definition": str(cell_def),
            "copied_cell_definition": str(copied_cell),
            "generated_project_path": project_path,
            "dashboard_path": str(manifests[0].parent / "dashboard" / "index.html") if manifests else "",
            "preflight_report_path": str(manifests[0].parent / "reports" / "validation_summary.md") if manifests else "",
            "next_commands_path": str(demo_dir / "next_commands.md"),
            "preview_svg": str(preview_dir / "static_preview.svg"),
            "preview_html": str(preview_dir / "static_preview.html"),
            "preview_summary_json": str(preview_dir / "static_preview_summary.json"),
            "preview_warnings": preview_payload.get("warnings", []),
            "stdout": project_rc.stdout.strip(),
            "stderr": project_rc.stderr.strip(),
        }
        (demo_dir / "demo_bundle_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        (demo_dir / "demo_bundle_summary.md").write_text(
            "# Demo Bundle Summary\n\n"
            f"- Demo id: `{demo['id']}`\n"
            f"- Title: `{demo.get('title','')}`\n"
            f"- Runtime mode: `{demo.get('runtime_mode','unknown')}`\n"
            f"- Preview only: `{summary['preview_only']}`\n"
            f"- Generated project path: `{project_path or '(none)'}`\n"
            f"- Dashboard: `{summary['dashboard_path'] or '(none)'}`\n"
            f"- Preflight report: `{summary['preflight_report_path'] or '(none)'}`\n"
            f"- Preview SVG: `{summary['preview_svg']}`\n"
            f"- Preview HTML: `{summary['preview_html']}`\n"
            f"- Preview Summary JSON: `{summary['preview_summary_json']}`\n",
            encoding="utf-8",
        )
        demo_title = demo.get("title", demo["id"])
        (demo_dir / "next_commands.md").write_text(
            "# Next Commands\n\n```bash\n"
            "python3 scripts/workcell_studio.py list-demos\n"
            f"python3 scripts/workcell_studio.py generate-demo-bundle --demo-id {demo['id']} --output-dir {output_root} --force\n"
            f"python3 scripts/validate_cell_definition.py {copied_cell} --json\n"
            f"python3 scripts/generate_workcell_static_preview.py --cell-definition {copied_cell} --output-dir {preview_dir} --title '{demo_title}'\n"
            "```\n",
            encoding="utf-8",
        )
        if project_rc.returncode != 0:
            failures.append(f"Project generation failed for {demo['id']}")
            if not args.continue_on_error:
                return 5

    if failures and not args.continue_on_error:
        return 6
    if failures:
        print(json.dumps({"result": "WARN", "failures": failures}, indent=2))
        return 1
    print(json.dumps({"result": "PASS", "generated": [d["id"] for d in selected], "output_dir": str(output_root)}, indent=2))
    return 0


def _scene_name(path: Path) -> str:
    return path.name

# existing import_builder_scene unchanged below
def import_builder_scene(args: argparse.Namespace) -> int:
    scene_pkg = args.scene_package.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    if not scene_pkg.exists() or not scene_pkg.is_dir():
        print(f"FAIL: scene package path does not exist or is not a directory: {scene_pkg}")
        return 1

    generated_dir = scene_pkg / "generated"
    cell_def = generated_dir / "cell_definition.yaml"
    env_layout = generated_dir / "environment_layout.yaml"

    export_status = "reused" if (cell_def.is_file() and env_layout.is_file()) else "generated"
    export_payload: dict[str, Any] = {}
    if export_status == "generated":
        export_cmd = [
            sys.executable,
            str(SCRIPT_DIR / "export_builder_scene_to_cell_definition.py"),
            str(scene_pkg),
            "--output-dir",
            str(generated_dir),
        ]
        if args.validate:
            export_cmd.append("--validate")
        export_proc = subprocess.run(export_cmd, capture_output=True, text=True, check=False)
        if export_proc.returncode != 0:
            print("FAIL: unable to export builder scene to cell definition")
            print(export_proc.stdout)
            print(export_proc.stderr)
            return export_proc.returncode or 2
        summary_path = generated_dir / "builder_export_summary.json"
        if summary_path.is_file():
            export_payload = json.loads(summary_path.read_text(encoding="utf-8"))

    if not cell_def.is_file() or not env_layout.is_file():
        print("FAIL: export did not produce generated/cell_definition.yaml and generated/environment_layout.yaml")
        return 2

    validations: dict[str, Any] = {}
    hard_fail = False
    if args.validate:
        rc_scene, scene_payload = _run_json([sys.executable, str(SCRIPT_DIR / "validate_builder_generated_scene.py"), str(scene_pkg), "--json"], "builder scene validator")
        validations["builder_scene"] = scene_payload
        if not bool(scene_payload.get("ok", False)):
            hard_fail = True

        rc_cell, cell_payload = _run_json([sys.executable, str(SCRIPT_DIR / "validate_cell_definition.py"), str(cell_def), "--json"], "cell definition validator")
        validations["cell_definition"] = cell_payload
        if cell_payload.get("result") == "FAIL" and not cell_payload.get("errors"):
            hard_fail = True

        rc_layout, layout_payload = _run_json([sys.executable, str(SCRIPT_DIR / "validate_environment_layout.py"), str(env_layout), "--json"], "environment layout validator")
        validations["environment_layout"] = layout_payload
        if layout_payload.get("result") == "FAIL" and not layout_payload.get("warnings"):
            hard_fail = True

    generated_project_path = ""
    dashboard_path = ""
    preflight_report_path = ""

    if args.generate_project and not hard_fail:
        project_cmd = [
            sys.executable,
            str(SCRIPT_DIR / "create_workcell_project.py"),
            "--cell-definition",
            str(cell_def),
            "--output-dir",
            str(output_dir),
            "--force",
        ]
        project_proc = subprocess.run(project_cmd, capture_output=True, text=True, check=False)
        if project_proc.returncode != 0:
            print("FAIL: create_workcell_project.py failed")
            print(project_proc.stdout)
            print(project_proc.stderr)
            return project_proc.returncode or 3

        manifest_candidates = list(output_dir.glob("*/project_manifest.json"))
        if manifest_candidates:
            manifest = json.loads(manifest_candidates[0].read_text(encoding="utf-8"))
            generated_project_path = str(manifest_candidates[0].parent)
            artifacts = manifest.get("artifacts", {}) if isinstance(manifest.get("artifacts"), dict) else {}
            dashboard_rel = artifacts.get("dashboard")
            preflight_rel = artifacts.get("validation_summary")
            if dashboard_rel:
                dashboard_path = str(manifest_candidates[0].parent / dashboard_rel)
            if preflight_rel:
                preflight_report_path = str(manifest_candidates[0].parent / preflight_rel)

    preview_dir = output_dir / "preview"
    recipe_gen = ((export_payload.get("task_recipe_generation", {}) if isinstance(export_payload, dict) else {}) or {})
    task_intent = ((export_payload.get("builder_task_intent", {}) if isinstance(export_payload, dict) else {}) or {})
    task_intent_path = Path(task_intent.get('source_file', generated_dir / 'workcell_builder_task_intent.yaml')) if isinstance(task_intent, dict) and task_intent else None
    task_recipe_path = generated_dir / 'task_recipe_from_builder_intent.yaml'
    cmd=[sys.executable, str(SCRIPT_DIR / 'generate_workcell_static_preview.py'), '--cell-definition', str(cell_def), '--output-dir', str(preview_dir), '--title', f"Builder Import: {_scene_name(scene_pkg)}", '--json']
    if env_layout: cmd += ['--environment-layout', str(env_layout)]
    if task_intent_path and task_intent_path.exists(): cmd += ['--task-intent', str(task_intent_path)]
    if task_recipe_path.exists(): cmd += ['--task-recipe', str(task_recipe_path)]
    _, preview_payload = _run_json(cmd, 'static preview')
    if task_intent and recipe_gen.get("status") != "PASS":
        recipe_path = generated_dir / "task_recipe_from_builder_intent.yaml"
        rc, recipe_gen = _run_json([sys.executable, str(SCRIPT_DIR / "convert_builder_task_intent_to_task_recipe.py"), "--task-intent", str(Path(task_intent.get("source_file", generated_dir / "workcell_builder_task_intent.yaml"))), "--output", str(recipe_path), "--scene-package", str(scene_pkg), "--validate", "--json"], "task recipe conversion")

    builder_scene_validation = (validations.get("builder_scene", {}) or {})
    builder_readiness = builder_scene_validation.get("readiness")
    runtime_readiness = builder_scene_validation.get("runtime_readiness")
    safety = {
        "fake_hardware_default": True,
        "runtime_io_applied": False,
        "runtime_status": runtime_readiness or builder_readiness or "unknown",
    }
    task_intent_status = "missing"
    if task_intent:
        task_intent_status = "present"
    if recipe_gen.get("status") == "PASS":
        task_intent_status = "task_recipe_generated"

    summary = {
        "source_scene_package": str(scene_pkg),
        "detected_package_name": _scene_name(scene_pkg),
        "export_status": export_status,
        "cell_definition_path": str(cell_def),
        "environment_layout_path": str(env_layout),
        "validation": validations,
        "generated_project_path": generated_project_path,
        "dashboard_path": dashboard_path,
        "preflight_report_path": preflight_report_path,
        "safety_status": safety,
        "preview_svg": str(preview_dir / "static_preview.svg"),
        "preview_html": str(preview_dir / "static_preview.html"),
        "preview_summary_json": str(preview_dir / "static_preview_summary.json"),
        "preview_warnings": preview_payload.get("warnings", []),
        "task_intent_status": task_intent_status,
        "generated_task_recipe_path": recipe_gen.get("generated_task_recipe_path", ""),
        "pick_source": recipe_gen.get("pick_source"),
        "place_target": recipe_gen.get("place_target"),
        "grasp_strategy": recipe_gen.get("grasp_strategy"),
        "release_strategy": recipe_gen.get("release_strategy"),
        "routing_rule_count": recipe_gen.get("routing_rule_count", 0),
        "task_flow_summary": preview_payload.get("task_flow_summary", {}),
        "readiness_classification": (preview_payload.get("task_flow_summary", {}) or {}).get("readiness_classification", builder_readiness),
        "missing_required_fields": (preview_payload.get("task_flow_summary", {}) or {}).get("missing_required_fields", []),
        "visual_resolution": (preview_payload.get("task_flow_summary", {}) or {}).get("visual_resolution", {}),
        "task_recipe_safety": {"metadata_only": True, "runtime_io_applied": False, "motion_started": False, "ros_launch_started": False},
        "next_commands": [
            f"python3 scripts/validate_builder_generated_scene.py {scene_pkg} --json",
            f"python3 scripts/validate_cell_definition.py {cell_def} --json",
            f"python3 scripts/validate_environment_layout.py {env_layout} --json",
            f"python3 scripts/create_workcell_project.py --cell-definition {cell_def} --output-dir {output_dir} --force",
        ],
        "export_summary": export_payload,
        "rviz_plan_preview_session_path": "",
        "rviz_plan_preview_suggested_commands_path": "",
        "planning_scene_readiness_report_path": "",
        "planning_scene_readiness_markdown_path": "",
    }
    if args.generate_readiness_pack:
        summary["next_commands"].append(
            f"python3 scripts/workcell_studio.py generate-readiness-pack --scene-package {scene_pkg} --output-dir {output_dir / 'readiness_pack'} --project-name {args.project_name} --validate --smoke-dry-run --force --json"
        )

    
    if summary.get("generated_task_recipe_path"):
        sess_dir = output_dir / "plan_preview_session"
        preview_cmd = [sys.executable, str(SCRIPT_DIR / "generate_rviz_moveit_plan_preview_session.py"), "--scene-package", str(scene_pkg), "--plan-preview-request", str(generated_dir / "offline_plan_preview_request.yaml"), "--output-dir", str(sess_dir), "--allow-missing-launch", "--json"]
        if Path(summary.get("generated_task_recipe_path")).exists():
            req_cmd=[sys.executable, str(SCRIPT_DIR/"generate_offline_plan_preview_request.py"), "--task-recipe", summary.get("generated_task_recipe_path"), "--output", str(generated_dir / "offline_plan_preview_request.yaml"), "--validate", "--json"]
            _run_json(req_cmd, "offline request")
        _run_json(preview_cmd, "rviz plan preview prepare")
        summary["rviz_plan_preview_session_path"] = str(sess_dir / "rviz_moveit_plan_preview_session.json")
        summary["rviz_plan_preview_suggested_commands_path"] = str(sess_dir / "suggested_commands.sh")

    if summary.get("generated_task_recipe_path"):
        read_dir = output_dir / "planning_scene_readiness"
        read_cmd = [sys.executable, str(SCRIPT_DIR / "check_planning_scene_readiness.py"), "--scene-package", str(scene_pkg), "--output-dir", str(read_dir), "--task-recipe", str(summary.get("generated_task_recipe_path")), "--plan-preview-request", str(generated_dir / "offline_plan_preview_request.yaml"), "--plan-preview-session", str(sess_dir / "rviz_moveit_plan_preview_session.json"), "--json"]
        _run_json(read_cmd, "planning scene readiness")
        summary["planning_scene_readiness_report_path"] = str(read_dir / "planning_scene_readiness_report.json")
        summary["planning_scene_readiness_markdown_path"] = str(read_dir / "planning_scene_readiness_report.md")

    summary_json = output_dir / "workcell_studio_import_summary.json"
    summary_md = output_dir / "workcell_studio_import_summary.md"
    summary_json.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    md_lines = [
        "# Workcell Studio Import Summary",
        "",
        f"- Source scene package: `{scene_pkg}`",
        f"- Package name: `{_scene_name(scene_pkg)}`",
        f"- Export status: **{export_status}**",
        f"- Cell definition: `{cell_def}`",
        f"- Environment layout: `{env_layout}`",
        f"- Generated project path: `{generated_project_path or '(not generated)'}`",
        f"- Runtime status: `{safety['runtime_status']}`",
        f"- Preview SVG: `{summary['preview_svg']}`",
        f"- Preview HTML: `{summary['preview_html']}`",
        f"- Task intent status: `{task_intent_status}`",
        f"- Generated task recipe: `{summary.get('generated_task_recipe_path') or '(none)'}`",
        "",
        "## Validation",
        f"- Builder scene: `{(validations.get('builder_scene', {}) or {}).get('ok', 'not-run')}`",
        f"- Cell definition result: `{(validations.get('cell_definition', {}) or {}).get('result', 'not-run')}`",
        f"- Environment layout result: `{(validations.get('environment_layout', {}) or {}).get('result', 'not-run')}`",
    ]
    summary_md.write_text("\n".join(md_lines) + "\n", encoding="utf-8")

    if hard_fail:
        print("FAIL: validation failed; see summary artifacts")
        return 1

    print(f"PASS: import complete. Summary: {summary_json}")
    return 0




def prepare_rviz_plan_preview(args: argparse.Namespace) -> int:
    cmd=[sys.executable, str(SCRIPT_DIR / "generate_rviz_moveit_plan_preview_session.py"), "--scene-package", str(args.scene_package), "--plan-preview-request", str(args.plan_preview_request), "--output-dir", str(args.output_dir)]
    if args.cell_definition: cmd += ["--cell-definition", str(args.cell_definition)]
    if args.task_recipe: cmd += ["--task-recipe", str(args.task_recipe)]
    if args.project_dir: cmd += ["--project-dir", str(args.project_dir)]
    if args.allow_missing_launch: cmd.append("--allow-missing-launch")
    if args.json: cmd.append("--json")
    run=subprocess.run(cmd, capture_output=True, text=True, check=False)
    print(run.stdout if run.stdout else run.stderr)
    return run.returncode


def validate_rviz_plan_preview(args: argparse.Namespace) -> int:
    cmd=[sys.executable, str(SCRIPT_DIR / "validate_rviz_moveit_plan_preview_session.py"), str(args.session)]
    if args.json: cmd.append("--json")
    run=subprocess.run(cmd, capture_output=True, text=True, check=False)
    print(run.stdout if run.stdout else run.stderr)
    return run.returncode

def smoke_launch_preview(args: argparse.Namespace) -> int:
    cmd=[sys.executable, str(SCRIPT_DIR / "run_fake_hardware_smoke_launch.py"), "--session", str(args.session), "--output-dir", str(args.output_dir), "--timeout-s", str(args.timeout_s)]
    cmd.append("--execute" if args.execute else "--dry-run")
    if args.json: cmd.append("--json")
    run=subprocess.run(cmd, capture_output=True, text=True, check=False)
    print(run.stdout if run.stdout else run.stderr)
    return run.returncode

def validate_smoke_launch_report(args: argparse.Namespace) -> int:
    cmd=[sys.executable, str(SCRIPT_DIR / "validate_fake_hardware_smoke_launch_report.py"), str(args.report)]
    if args.json: cmd.append("--json")
    run=subprocess.run(cmd, capture_output=True, text=True, check=False)
    print(run.stdout if run.stdout else run.stderr)
    return run.returncode



def check_planning_scene_readiness(args: argparse.Namespace) -> int:
    cmd=[sys.executable, str(SCRIPT_DIR / "check_planning_scene_readiness.py"), "--scene-package", str(args.scene_package), "--output-dir", str(args.output_dir)]
    for key,flag in [("cell_definition","--cell-definition"),("task_recipe","--task-recipe"),("plan_preview_request","--plan-preview-request"),("plan_preview_session","--plan-preview-session"),("smoke_report","--smoke-report")]:
        v=getattr(args,key,None)
        if v: cmd += [flag, str(v)]
    if args.strict: cmd.append("--strict")
    if args.json: cmd.append("--json")
    run=subprocess.run(cmd, capture_output=True, text=True, check=False)
    print(run.stdout if run.stdout else run.stderr)
    return run.returncode

def validate_planning_scene_readiness(args: argparse.Namespace) -> int:
    cmd=[sys.executable, str(SCRIPT_DIR / "validate_planning_scene_readiness_report.py"), str(args.report)]
    if args.json: cmd.append("--json")
    run=subprocess.run(cmd, capture_output=True, text=True, check=False)
    print(run.stdout if run.stdout else run.stderr)
    return run.returncode

def validate_builder_task(args: argparse.Namespace) -> int:
    cmd = [sys.executable, str(SCRIPT_DIR / "validate_builder_task_intent.py"), str(args.task_intent)]
    if args.scene_package:
        cmd.extend(["--scene-package", str(args.scene_package)])
    if args.json:
        cmd.append("--json")
    run = subprocess.run(cmd, capture_output=True, text=True, check=False)
    print(run.stdout if run.stdout else run.stderr)
    return run.returncode

def generate_readiness_pack(args: argparse.Namespace) -> int:
    cmd=[sys.executable, str(SCRIPT_DIR / "generate_workcell_studio_readiness_pack.py"), "--scene-package", str(args.scene_package), "--output-dir", str(args.output_dir), "--project-name", args.project_name, "--smoke-timeout-s", str(args.smoke_timeout_s)]
    for flag,name in [(args.validate,"--validate"),(args.prepare_rviz_preview,"--prepare-rviz-preview"),(args.smoke_dry_run,"--smoke-dry-run"),(args.smoke_execute,"--smoke-execute"),(args.strict,"--strict"),(args.continue_on_error,"--continue-on-error"),(args.no_dashboard,"--no-dashboard"),(args.force,"--force"),(args.json,"--json")]:
        if flag: cmd.append(name)
    run=subprocess.run(cmd, capture_output=True, text=True, check=False)
    print(run.stdout if run.stdout else run.stderr)
    try:
        payload = json.loads(run.stdout) if run.stdout.strip() else {}
    except Exception:
        payload = {}
    result = str(payload.get("result", "")).upper()
    status = str(payload.get("status", "")).upper()
    manifest_path = payload.get("manifest_path") or payload.get("manifest")
    safety_violation = any(
        bool(payload.get(flag))
        for flag in (
            "real_hardware_enabled",
            "motion_command_sent",
            "runtime_execution_called",
            "moveit_plan_service_called",
        )
    )
    downgraded_nonfatal = result in {"WARN", "PARTIAL"} or status in {"WARN", "PARTIAL"}
    downgraded_pass = result == "PASS" or status == "PASS"
    if run.returncode == 2 and not safety_violation and (
        downgraded_pass or downgraded_nonfatal or manifest_path
    ):
        return 0 if downgraded_pass else 1
    return run.returncode

def validate_readiness_pack(args: argparse.Namespace) -> int:
    cmd=[sys.executable, str(SCRIPT_DIR / "validate_workcell_studio_readiness_pack.py"), str(args.manifest)]
    if args.json: cmd.append("--json")
    run=subprocess.run(cmd, capture_output=True, text=True, check=False)
    print(run.stdout if run.stdout else run.stderr)
    return run.returncode



def generate_readiness_dashboard(args: argparse.Namespace) -> int:
    cmd=[sys.executable, str(SCRIPT_DIR / "generate_readiness_pack_dashboard.py"), "--manifest", str(args.manifest), "--output", str(args.output)]
    for flag,name in [(args.embed_static_preview,"--embed-static-preview"),(args.strict_links,"--strict-links"),(args.json,"--json")]:
        if flag: cmd.append(name)
    if args.title: cmd.extend(["--title", args.title])
    run=subprocess.run(cmd, capture_output=True, text=True, check=False)
    print(run.stdout if run.stdout else run.stderr)
    return run.returncode



def list_builder_targets(args: argparse.Namespace) -> int:
    cmd=[sys.executable, str(SCRIPT_DIR/"list_builder_scene_authoring_targets.py"), "--scene-package", str(args.scene_package)]
    if args.json: cmd.append("--json")
    run=subprocess.run(cmd, capture_output=True, text=True, check=False)
    print(run.stdout if run.stdout else run.stderr)
    return run.returncode

def author_builder_task(args: argparse.Namespace) -> int:
    cmd=[sys.executable, str(SCRIPT_DIR/"create_or_update_builder_task_intent.py"), "--scene-package", str(args.scene_package), "--task-id", args.task_id, "--task-type", args.task_type, "--pick-source", args.pick_source, "--place-target", args.place_target, "--grasp-strategy", args.grasp_strategy, "--approach-axis", args.approach_axis, "--approach-distance-m", str(args.approach_distance_m), "--retreat-axis", args.retreat_axis, "--retreat-distance-m", str(args.retreat_distance_m), "--release-strategy", args.release_strategy, "--object-class", args.object_class, "--object-color", args.object_color]
    if args.output: cmd += ["--output", str(args.output)]
    if args.validate: cmd.append("--validate")
    if args.json: cmd.append("--json")
    run=subprocess.run(cmd, capture_output=True, text=True, check=False)
    print(run.stdout if run.stdout else run.stderr)
    return run.returncode

def author_environment_target(args: argparse.Namespace) -> int:
    cmd=[sys.executable, str(SCRIPT_DIR/"create_or_update_environment_target.py"), "--environment-layout", str(args.environment_layout), "--target-id", args.target_id, "--target-type", args.target_type, "--label", args.label, "--frame", args.frame, "--xyz", *[str(x) for x in args.xyz], "--rpy", *[str(x) for x in args.rpy], "--size", *[str(x) for x in args.size], "--output", str(args.output)]
    if args.json: cmd.append("--json")
    run=subprocess.run(cmd, capture_output=True, text=True, check=False)
    print(run.stdout if run.stdout else run.stderr)
    return run.returncode
def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    demos_list = sub.add_parser("list-demos", help="List curated Workcell Studio demos")
    demos_list.add_argument("--json", action="store_true")
    demos_list.set_defaults(func=list_demos)

    demos_gen = sub.add_parser("generate-demo-bundle", help="Generate one or all curated demo bundles")
    demos_gen.add_argument("--demo-id", type=str)
    demos_gen.add_argument("--all", action="store_true")
    demos_gen.add_argument("--output-dir", type=Path, required=True)
    demos_gen.add_argument("--force", action="store_true")
    demos_gen.add_argument("--continue-on-error", action="store_true")
    demos_gen.set_defaults(func=generate_demo_bundle)

    vbt = sub.add_parser("validate-builder-task", help="Validate builder task intent sidecar")
    vbt.add_argument("--task-intent", type=Path, required=True)
    vbt.add_argument("--scene-package", type=Path)
    vbt.add_argument("--json", action="store_true")
    vbt.set_defaults(func=validate_builder_task)

    import_parser = sub.add_parser("import-builder-scene", help="Import a workcell_builder-generated scene package")
    import_parser.add_argument("--scene-package", type=Path, required=True)
    import_parser.add_argument("--output-dir", type=Path, required=True)
    import_parser.add_argument("--project-name", type=str, required=True)
    import_parser.add_argument("--validate", action="store_true")
    import_parser.add_argument("--generate-project", action="store_true")
    import_parser.add_argument("--generate-readiness-pack", action="store_true")
    import_parser.set_defaults(func=import_builder_scene)

    prep = sub.add_parser("prepare-rviz-plan-preview", help="Prepare guarded fake-hardware RViz/MoveIt plan preview session artifacts")
    prep.add_argument("--scene-package", required=True)
    prep.add_argument("--plan-preview-request", type=Path, required=True)
    prep.add_argument("--output-dir", type=Path, required=True)
    prep.add_argument("--cell-definition", type=Path)
    prep.add_argument("--task-recipe", type=Path)
    prep.add_argument("--project-dir", type=Path)
    prep.add_argument("--allow-missing-launch", action="store_true")
    prep.add_argument("--json", action="store_true")
    prep.set_defaults(func=prepare_rviz_plan_preview)

    vprep = sub.add_parser("validate-rviz-plan-preview", help="Validate rviz_moveit_plan_preview_session/v1 artifacts")
    vprep.add_argument("--session", type=Path, required=True)
    vprep.add_argument("--json", action="store_true")
    vprep.set_defaults(func=validate_rviz_plan_preview)
    smoke = sub.add_parser("smoke-launch-preview", help="Guarded fake-hardware RViz/MoveIt smoke launch verifier")
    smoke.add_argument("--session", type=Path, required=True)
    smoke.add_argument("--output-dir", type=Path, required=True)
    smoke.add_argument("--dry-run", action="store_true")
    smoke.add_argument("--execute", action="store_true")
    smoke.add_argument("--timeout-s", type=int, default=20)
    smoke.add_argument("--json", action="store_true")
    smoke.set_defaults(func=smoke_launch_preview)

    vsmoke = sub.add_parser("validate-smoke-launch-report", help="Validate fake_hardware_smoke_launch_report/v1 artifacts")
    vsmoke.add_argument("--report", type=Path, required=True)
    vsmoke.add_argument("--json", action="store_true")
    vsmoke.set_defaults(func=validate_smoke_launch_report)


    cpsr = sub.add_parser("check-planning-scene-readiness", help="Check file/metadata planning scene readiness artifacts")
    cpsr.add_argument("--scene-package", type=Path, required=True)
    cpsr.add_argument("--output-dir", type=Path, required=True)
    cpsr.add_argument("--cell-definition", type=Path)
    cpsr.add_argument("--task-recipe", type=Path)
    cpsr.add_argument("--plan-preview-request", type=Path)
    cpsr.add_argument("--plan-preview-session", type=Path)
    cpsr.add_argument("--smoke-report", type=Path)
    cpsr.add_argument("--strict", action="store_true")
    cpsr.add_argument("--json", action="store_true")
    cpsr.set_defaults(func=check_planning_scene_readiness)

    vpsr = sub.add_parser("validate-planning-scene-readiness", help="Validate planning_scene_readiness_report/v1 artifact")
    vpsr.add_argument("--report", type=Path, required=True)
    vpsr.add_argument("--json", action="store_true")
    vpsr.set_defaults(func=validate_planning_scene_readiness)

    grp = sub.add_parser("generate-readiness-pack", help="Generate Workcell Studio readiness pack")
    grp.add_argument("--scene-package", type=Path, required=True)
    grp.add_argument("--output-dir", type=Path, required=True)
    grp.add_argument("--project-name", type=str, required=True)
    grp.add_argument("--validate", action="store_true")
    grp.add_argument("--prepare-rviz-preview", action="store_true")
    grp.add_argument("--smoke-dry-run", action="store_true")
    grp.add_argument("--smoke-execute", action="store_true")
    grp.add_argument("--smoke-timeout-s", type=int, default=20)
    grp.add_argument("--strict", action="store_true")
    grp.add_argument("--continue-on-error", action="store_true")
    grp.add_argument("--no-dashboard", action="store_true")
    grp.add_argument("--force", action="store_true")
    grp.add_argument("--json", action="store_true")
    grp.set_defaults(func=generate_readiness_pack)



    grd = sub.add_parser("generate-readiness-dashboard", help="Generate readiness dashboard HTML")
    grd.add_argument("--manifest", type=Path, required=True)
    grd.add_argument("--output", type=Path, required=True)
    grd.add_argument("--title", type=str, default="Workcell Studio Readiness Dashboard")
    grd.add_argument("--embed-static-preview", action="store_true")
    grd.add_argument("--strict-links", action="store_true")
    grd.add_argument("--json", action="store_true")
    grd.set_defaults(func=generate_readiness_dashboard)

    vrp = sub.add_parser("validate-readiness-pack", help="Validate readiness pack manifest")
    vrp.add_argument("--manifest", type=Path, required=True)
    vrp.add_argument("--json", action="store_true")
    vrp.set_defaults(func=validate_readiness_pack)

    cc = sub.add_parser("create-cell", help="Create a catalog-driven cell_definition + environment layout + optional preview/bundle")
    cc.add_argument("--cell-id", required=True)
    cc.add_argument("--robot", required=True)
    cc.add_argument("--end-effector", required=True)
    cc.add_argument("--sensor", required=True)
    cc.add_argument("--task", required=True)
    cc.add_argument("--grasp-strategy", required=True)
    cc.add_argument("--output-dir", type=Path, required=True)
    cc.add_argument("--validate", action="store_true")
    cc.add_argument("--preview", action="store_true")
    cc.add_argument("--generate-bundle", action="store_true")
    cc.add_argument("--allow-incompatible", action="store_true")
    cc.add_argument("--force", action="store_true")
    cc.set_defaults(func=create_cell)
    lbt = sub.add_parser("list-builder-targets", help="List candidate builder scene authoring targets")
    lbt.add_argument("--scene-package", type=Path, required=True)
    lbt.add_argument("--json", action="store_true")
    lbt.set_defaults(func=list_builder_targets)

    abt = sub.add_parser("author-builder-task", help="Create or update builder task intent")
    abt.add_argument("--scene-package", type=Path, required=True)
    abt.add_argument("--task-id", required=True)
    abt.add_argument("--task-type", required=True)
    abt.add_argument("--pick-source", required=True)
    abt.add_argument("--place-target", required=True)
    abt.add_argument("--grasp-strategy", required=True)
    abt.add_argument("--approach-axis", default="z_down")
    abt.add_argument("--approach-distance-m", type=float, default=0.1)
    abt.add_argument("--retreat-axis", default="z_up")
    abt.add_argument("--retreat-distance-m", type=float, default=0.1)
    abt.add_argument("--release-strategy", default="tool_release")
    abt.add_argument("--object-class", default="any")
    abt.add_argument("--object-color", default="any")
    abt.add_argument("--output", type=Path)
    abt.add_argument("--validate", action="store_true")
    abt.add_argument("--json", action="store_true")
    abt.set_defaults(func=author_builder_task)
    aet = sub.add_parser("author-environment-target", help="Create or update environment pick/place target")
    aet.add_argument("--environment-layout", type=Path, required=True)
    aet.add_argument("--target-id", required=True)
    aet.add_argument("--target-type", choices=["pick_zone","place_target","bin"], required=True)
    aet.add_argument("--label", default="")
    aet.add_argument("--frame", default="world")
    aet.add_argument("--xyz", nargs=3, type=float, required=True)
    aet.add_argument("--rpy", nargs=3, type=float, required=True)
    aet.add_argument("--size", nargs=3, type=float, required=True)
    aet.add_argument("--output", type=Path, required=True)
    aet.add_argument("--json", action="store_true")
    aet.set_defaults(func=author_environment_target)

    return parser

def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
