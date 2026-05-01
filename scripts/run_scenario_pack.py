#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, subprocess, sys
from pathlib import Path
from typing import Any

SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))
from validate_detected_objects import _load_yaml_or_json

REPO_ROOT = Path(__file__).resolve().parents[1]
SCHEMA = "scenario_pack/v1"


def _load_yaml(path: Path) -> dict[str, Any]:
    doc, _, _ = _load_yaml_or_json(path)
    if not isinstance(doc, dict):
        raise SystemExit(f"FAIL: invalid YAML object: {path}")
    return doc


def load_and_validate_scenario(path: Path) -> tuple[dict[str, Any], list[str]]:
    s = _load_yaml(path)
    errs: list[str] = []
    if s.get("schema_version") != SCHEMA: errs.append("schema_version must be scenario_pack/v1")
    for key in ("name", "cell_definition", "expected"):
        if key not in s: errs.append(f"missing required key: {key}")
    return s, errs


def _run(cmd: list[str], cwd: Path) -> tuple[int, dict[str, Any], str, str]:
    p = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, check=False)
    payload = {}
    if p.stdout.strip():
        try: payload = json.loads(p.stdout)
        except Exception: payload = {}
    return p.returncode, payload, p.stdout, p.stderr


def run_scenario(scenario_path: Path, output_root: Path, as_json: bool=False) -> dict[str, Any]:
    scenario, validation_errors = load_and_validate_scenario(scenario_path)
    name = scenario.get("name", scenario_path.stem)
    out_root = output_root / name
    for sub in ("generated_workcell", "preview", "dry_run", "logs"):
        (out_root / sub).mkdir(parents=True, exist_ok=True)
    report = {
      "schema_version": "scenario_run_report/v1", "scenario": name, "status": "FAIL", "enabled": bool(scenario.get("enabled", True)),
      "cell_definition": scenario.get("cell_definition"), "generated_workcell_path": "", "steps": {
        "validate_scenario": "FAIL", "generate_workcell": "SKIPPED", "visual_preview": "SKIPPED", "gated_dry_run": "SKIPPED", "task_flow_preview": "SKIPPED", "expected_output_check": "SKIPPED"},
      "selected_object": None, "selected_destination": None, "task_type": (scenario.get("expected") or {}).get("task_type"),
      "safe_for_robot_motion": False, "warnings": [], "blockers": [], "report_paths": {}
    }
    if not report["enabled"]:
        report["status"] = "SKIPPED"
        report["steps"]["validate_scenario"] = "PASS"
        report["warnings"].append(scenario.get("disabled_reason", "scenario disabled"))
        return report
    if validation_errors:
        report["blockers"].extend(validation_errors)
        return report
    report["steps"]["validate_scenario"] = "PASS"

    cell_definition = REPO_ROOT / str(scenario["cell_definition"])
    if not cell_definition.exists():
        report["blockers"].append(f"cell_definition not found: {cell_definition}")
        return report

    gen_dir = out_root / "generated_workcell"
    gen_cmd = [sys.executable, str(REPO_ROOT / "scripts/generate_workcell_from_cell_definition.py"), "--cell-definition", str(cell_definition), "--output-dir", str(gen_dir), "--package-name", name, "--json"]
    rc, payload, so, se = _run(gen_cmd, REPO_ROOT)
    (out_root / "logs/generate_workcell.log").write_text(so + "\n" + se, encoding="utf-8")
    if rc != 0:
        report["blockers"].append("generate_workcell_from_cell_definition.py failed")
        return report
    report["steps"]["generate_workcell"] = "PASS"
    workcell_path = gen_dir / name
    report["generated_workcell_path"] = str(workcell_path)

    prev_cmd = [sys.executable, str(REPO_ROOT / "scripts/preview_generated_workcell_bundle.py"), "--workcell", str(workcell_path), "--json"]
    rc, prev_payload, so, se = _run(prev_cmd, REPO_ROOT)
    (out_root / "preview/visual_preview_output.json").write_text(json.dumps(prev_payload or {"stdout": so, "stderr": se}, indent=2), encoding="utf-8")
    report["steps"]["visual_preview"] = "PASS" if rc == 0 else "FAIL"

    dry_cmd = [sys.executable, str(REPO_ROOT / "scripts/run_generated_workcell_bundle.py"), "--workcell", str(workcell_path), "--output-dir", str(out_root / "dry_run"), "--gated-dry-run", "--dry-run", "--no-replay", "--json", "--preview-task-flow"]
    rc, dry_payload, so, se = _run(dry_cmd, REPO_ROOT)
    (out_root / "logs/gated_dry_run.log").write_text(so + "\n" + se, encoding="utf-8")
    dry_status = str((dry_payload or {}).get("status", "FAIL")).upper()
    report["steps"]["gated_dry_run"] = dry_status if dry_status in {"PASS","WARN","FAIL"} else ("PASS" if rc==0 else "FAIL")

    tf_path = out_root / "dry_run/task_flow_preview.json"
    if tf_path.exists():
        tcmd = [sys.executable, str(REPO_ROOT / "scripts/preview_generated_workcell_bundle.py"), "--workcell", str(workcell_path), "--show-task-flow", "--task-flow-preview", str(tf_path), "--json"]
        trc, _, _, _ = _run(tcmd, REPO_ROOT)
        report["steps"]["task_flow_preview"] = "PASS" if trc == 0 else "WARN"
    else:
        report["steps"]["task_flow_preview"] = "SKIPPED"

    expected = scenario.get("expected") if isinstance(scenario.get("expected"), dict) else {}
    summary = workcell_path / "generated/generated_workcell_summary.json"
    selection_destination = None
    selected_object = None
    if summary.exists():
      s = json.loads(summary.read_text(encoding="utf-8"))
      selection_destination = s.get("selected_destination") or s.get("selected_destination_id")
      selected_object = s.get("selected_object") or s.get("selected_object_id")
      report["selected_destination"] = selection_destination
      report["selected_object"] = selected_object
    missing_outputs = [x for x in expected.get("required_outputs", []) if not (workcell_path / x).exists()]
    if missing_outputs:
      report["warnings"].append(f"missing required outputs: {missing_outputs}")
    if selection_destination and expected.get("allowed_destinations") and selection_destination not in expected.get("allowed_destinations"):
      report["warnings"].append("selected destination not in allowed_destinations")
    report["steps"]["expected_output_check"] = "WARN" if report["warnings"] else "PASS"

    blocked = report["steps"]["visual_preview"] == "FAIL" or report["steps"]["gated_dry_run"] == "FAIL"
    report["status"] = "FAIL" if blocked else ("WARN" if report["warnings"] or report["steps"]["gated_dry_run"]=="WARN" else "PASS")
    report["report_paths"] = {"scenario_output_root": str(out_root), "dry_run_report": str(out_root / "dry_run/bundle_run_report.json")}
    return report


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", type=Path, required=True)
    ap.add_argument("--output-root", type=Path, required=True)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)
    report = run_scenario(args.scenario, args.output_root, args.json)
    out = args.output_root / report["scenario"]
    out.mkdir(parents=True, exist_ok=True)
    rp = out / "scenario_run_report.json"
    rp.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.json: print(json.dumps(report, indent=2, sort_keys=True))
    else: print(f"{report['status']}: {rp}")
    return 0 if report["status"] in {"PASS","WARN","SKIPPED"} else 1

if __name__ == "__main__":
    raise SystemExit(main())
