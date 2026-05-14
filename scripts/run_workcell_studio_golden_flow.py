#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, subprocess, sys, tempfile
from datetime import datetime, timezone
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"


def _run_json(cmd: list[str]) -> tuple[int, dict]:
    p = subprocess.run(cmd, text=True, capture_output=True, check=False)
    try:
        data = json.loads(p.stdout) if p.stdout.strip() else {}
    except Exception:
        data = {"status": "BLOCKED", "blockers": [f"non-json output: {p.stdout[:200]}"]}
    data["_stdout"] = p.stdout.strip()
    data["_stderr"] = p.stderr.strip()
    return p.returncode, data


def _scene_template(scene_dir: Path) -> None:
    (scene_dir / "config").mkdir(parents=True, exist_ok=True)
    (scene_dir / "layout").mkdir(parents=True, exist_ok=True)
    (scene_dir / "generated").mkdir(parents=True, exist_ok=True)
    (scene_dir / "preview").mkdir(parents=True, exist_ok=True)
    (scene_dir / "smoke").mkdir(parents=True, exist_ok=True)
    (scene_dir / "launch").mkdir(parents=True, exist_ok=True)
    (scene_dir / "package.xml").write_text("<package format='3'><name>%s</name></package>\n" % scene_dir.name, encoding="utf-8")
    (scene_dir / "CMakeLists.txt").write_text("cmake_minimum_required(VERSION 3.8)\nproject(%s)\n" % scene_dir.name, encoding="utf-8")
    (scene_dir / "environment.yaml").write_text(yaml.safe_dump({"objects": []}, sort_keys=False), encoding="utf-8")
    (scene_dir / "scene_manifest.yaml").write_text(yaml.safe_dump({"objects": []}, sort_keys=False), encoding="utf-8")
    (scene_dir / "config" / "workcell_builder_task_intent.yaml").write_text(yaml.safe_dump({"safety": {"fake_hardware_first": True, "runtime_execution_enabled": False, "motion_command_sent": False}}, sort_keys=False), encoding="utf-8")
    (scene_dir / "config" / "task_recipe.yaml").write_text(yaml.safe_dump({"builder_task_intent": {}}, sort_keys=False), encoding="utf-8")
    (scene_dir / "preview" / "static_preview.html").write_text("<html>preview</html>\n", encoding="utf-8")
    (scene_dir / "smoke" / "offline_smoke_summary.txt").write_text("status=PASS\n", encoding="utf-8")
    (scene_dir / "smoke" / "offline_smoke_report.json").write_text(json.dumps({"status": "PASS"}, indent=2) + "\n", encoding="utf-8")
    (scene_dir / "launch" / "demo.launch.py").write_text("# use_fake_hardware launch\n", encoding="utf-8")


def run(scene_dir: Path | None = None) -> dict:
    scene_dir = scene_dir or Path(tempfile.mkdtemp(prefix="workcell_studio_golden_")) / "ur5_robotiq_pick_place"
    scene_dir.mkdir(parents=True, exist_ok=True)
    _scene_template(scene_dir)
    now = datetime.now(timezone.utc).isoformat()
    layout = {
        "saved_at_utc": now,
        "items": [
            {"id": "table_1", "type": "table", "pose": {"x": 0.7, "y": 0.0, "z": 0.0}},
            {"id": "bin_1", "type": "bin", "pose": {"x": 0.9, "y": -0.2, "z": 0.0}},
            {"id": "object_1", "type": "object", "pose": {"x": 0.6, "y": 0.1, "z": 0.1}},
            {"id": "camera_1", "type": "camera", "pose": {"x": -0.2, "y": 0.4, "z": 0.8}},
            {"id": "pick_zone_1", "type": "pick_zone", "pose": {"x": 0.6, "y": 0.0, "z": 0.0}},
            {"id": "place_zone_1", "type": "place_zone", "pose": {"x": 0.9, "y": -0.2, "z": 0.0}},
        ],
    }
    (scene_dir / "layout" / "workcell_studio_layout.yaml").write_text(yaml.safe_dump(layout, sort_keys=False), encoding="utf-8")

    _, merge = _run_json([sys.executable, str(SCRIPTS / "workcell_studio_layout_merge.py"), str(scene_dir), "--json"])
    _, acceptance = _run_json([sys.executable, str(SCRIPTS / "validate_workcell_studio_generated_scene.py"), str(scene_dir), "--json"])
    _, demo = _run_json([sys.executable, str(SCRIPTS / "workcell_studio_demo_mode.py"), str(scene_dir), "--json"])

    import importlib.util
    spec = importlib.util.spec_from_file_location("workcell_studio_preview_launch", SCRIPTS / "workcell_studio_preview_launch.py")
    mod = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(mod)
    commands = mod.build_preview_commands(scene_dir.name, "<workspace_root>")
    safe, preview_blockers = mod.command_is_safe(commands["launch"])
    mod.write_preview_launch_artifacts(scene_dir, scene_dir.name, commands["launch"], run=False, event="copy_only")

    artifacts = {
        "layout": scene_dir / "layout" / "workcell_studio_layout.yaml",
        "merge": scene_dir / "generated" / "workcell_studio_layout_merge_report.json",
        "acceptance": scene_dir / "acceptance" / "generated_scene_acceptance.json",
        "demo": scene_dir / "demo" / "workcell_studio_demo_report.json",
        "preview": scene_dir / "preview_launch" / "preview_launch_session.json",
    }
    warnings, blockers = [], []
    for k, v in artifacts.items():
        if not v.is_file():
            blockers.append(f"missing artifact: {k} -> {v}")
    if not safe:
        blockers.extend(preview_blockers)

    golden_dir = scene_dir / "golden_flow"
    golden_dir.mkdir(parents=True, exist_ok=True)
    report = {
        "scene_name": scene_dir.name,
        "generated_files_present": {k: v.is_file() for k, v in artifacts.items()},
        "saved_layout_present": artifacts["layout"].is_file(),
        "layout_merge_status": merge.get("layout_applied"),
        "acceptance_status": acceptance.get("status"),
        "demo_status": demo.get("status"),
        "preview_command_status": "SAFE" if safe else "BLOCKED",
        "stale_layout_status": acceptance.get("layout_stale", False),
        "warnings": warnings,
        "blockers": blockers,
        "artifact_paths": {k: str(v) for k, v in artifacts.items()},
        "safety_flags": {
            "fake_hardware_first": True,
            "runtime_execution_enabled": False,
            "motion_command_sent": False,
            "no_robot_motion_commanded": True,
        },
    }
    (golden_dir / "workcell_studio_golden_flow_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    (golden_dir / "workcell_studio_golden_flow_summary.txt").write_text(f"scene={scene_dir.name}\nblockers={len(blockers)}\npreview_status={report['preview_command_status']}\n", encoding="utf-8")
    (golden_dir / "workcell_studio_golden_flow_dashboard.html").write_text(f"<html><body><h1>Workcell Studio Golden Flow</h1><pre>{json.dumps(report, indent=2)}</pre></body></html>\n", encoding="utf-8")
    return report


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene-dir", type=Path, default=None)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()
    out = run(args.scene_dir)
    if args.json:
        print(json.dumps(out, indent=2))
    raise SystemExit(0 if not out["blockers"] else 1)
