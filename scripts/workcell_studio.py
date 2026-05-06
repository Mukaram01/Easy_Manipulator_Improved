#!/usr/bin/env python3
"""Workcell Studio CLI orchestration tools."""
from __future__ import annotations

import argparse
import json
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
        (demo_dir / "next_commands.md").write_text(
            "# Next Commands\n\n```bash\n"
            "python3 scripts/workcell_studio.py list-demos\n"
            f"python3 scripts/workcell_studio.py generate-demo-bundle --demo-id {demo['id']} --output-dir {output_root} --force\n"
            f"python3 scripts/validate_cell_definition.py {copied_cell} --json\n"
            f"python3 scripts/generate_workcell_static_preview.py --cell-definition {copied_cell} --output-dir {preview_dir} --title '{demo.get("title",demo["id"])}'\n"
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
    _, preview_payload = _generate_static_preview(cell_def, preview_dir, f"Builder Import: {_scene_name(scene_pkg)}", env_layout)

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
        "preview_svg": str(preview_dir / "static_preview.svg"),
        "preview_html": str(preview_dir / "static_preview.html"),
        "preview_summary_json": str(preview_dir / "static_preview_summary.json"),
        "preview_warnings": preview_payload.get("warnings", []),
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
        f"- Preview SVG: `{summary['preview_svg']}`",
        f"- Preview HTML: `{summary['preview_html']}`",
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
