#!/usr/bin/env python3
"""Workcell Studio CLI orchestration tools."""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent


def _run_json(cmd: list[str], label: str) -> tuple[int, dict[str, Any]]:
    proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
    payload: dict[str, Any]
    try:
        payload = json.loads(proc.stdout) if proc.stdout.strip() else {}
    except Exception:
        payload = {
            "result": "FAIL",
            "errors": [f"{label} returned non-JSON output", proc.stdout.strip(), proc.stderr.strip()],
        }
    payload.setdefault("stdout", proc.stdout.strip())
    payload.setdefault("stderr", proc.stderr.strip())
    payload.setdefault("returncode", proc.returncode)
    return proc.returncode, payload


def _scene_name(path: Path) -> str:
    return path.name


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

    builder_readiness = (validations.get("builder_scene", {}) or {}).get("readiness")
    safety = {
        "fake_hardware_default": True,
        "runtime_io_applied": False,
        "runtime_status": builder_readiness or "unknown",
    }

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
        "next_commands": [
            f"python3 scripts/validate_builder_generated_scene.py {scene_pkg} --json",
            f"python3 scripts/validate_cell_definition.py {cell_def} --json",
            f"python3 scripts/validate_environment_layout.py {env_layout} --json",
            f"python3 scripts/create_workcell_project.py --cell-definition {cell_def} --output-dir {output_dir} --force",
        ],
        "export_summary": export_payload,
    }

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


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    import_parser = sub.add_parser("import-builder-scene", help="Import a workcell_builder-generated scene package")
    import_parser.add_argument("--scene-package", type=Path, required=True)
    import_parser.add_argument("--output-dir", type=Path, required=True)
    import_parser.add_argument("--project-name", type=str, required=True)
    import_parser.add_argument("--validate", action="store_true")
    import_parser.add_argument("--generate-project", action="store_true")
    import_parser.set_defaults(func=import_builder_scene)
    return parser


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
