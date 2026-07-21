#!/usr/bin/env python3
"""Guided backend workflow for Workcell Studio Web 3D edit patches.

The workflow is safe by default: it exports a before web_scene, validates the
browser-produced edit_patch, and runs the existing applicator in dry-run mode.
Source YAML is mutated only when --write is explicitly provided. Browser output
is never trusted to write scene YAML directly.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
for import_path in (SCRIPT_DIR, REPO_ROOT):
    if str(import_path) not in sys.path:
        sys.path.insert(0, str(import_path))

from export_workcell_studio_web_scene import build_web_scene  # noqa: E402
from workcell_builder_studio_panel import build_export_sources_command  # noqa: E402
from validate_workcell_studio_web_scene_edit_patch import _load_json, _scene_id, validate  # noqa: E402


def _scene_id_from_dir(scene: Path) -> str:
    return scene.resolve().name


def _write_web_scene(scene: Path, output: Path) -> dict[str, Any]:
    payload = build_web_scene(scene)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload


def _format_cmd(cmd: list[str]) -> str:
    return " ".join(str(part) for part in cmd)


def _run_step(label: str, cmd: list[str]) -> int:
    print(f"\n== {label} ==")
    print("command: " + _format_cmd(cmd))
    result = subprocess.run(cmd, cwd=REPO_ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.stdout:
        print(result.stdout, end="")
    if result.stderr:
        print(result.stderr, end="", file=sys.stderr)
    print(f"{label} result: {'PASS' if result.returncode == 0 else 'FAIL'}")
    return result.returncode


def _validate_patch(before: dict[str, Any], patch_path: Path) -> tuple[bool, dict[str, Any] | None, list[str]]:
    try:
        patch = _load_json(patch_path)
    except ValueError as exc:
        return False, None, [str(exc)]
    errors = validate(before, patch)
    return not errors, patch, errors


def _print_next_write_command(args: argparse.Namespace) -> None:
    print("\nNext recommended action:")
    print(
        "  python3 scripts/run_workcell_studio_web_edit_workflow.py "
        f"--scene {args.scene} --patch {args.patch} --output-dir {args.output_dir} --write"
    )


def _print_next_generate_validate(scene: Path, *, generated: bool = False, validated: bool = False, failed: bool = False) -> None:
    print("\nNext recommended action:")
    if failed:
        print("  Review the failed command output above, fix the selected scene inputs, then rerun this workflow.")
    elif not generated:
        print(f"  python3 scripts/run_workcell_studio_web_edit_workflow.py --scene {scene} --generate")
    elif not validated:
        print(f"  python3 scripts/run_workcell_studio_web_edit_workflow.py --scene {scene} --validate")
    else:
        print("  Review generated validation output and keep using fake-hardware-first validation outside this web edit workflow.")


def _generation_cmd(scene: Path) -> list[str]:
    export_cmd = build_export_sources_command(scene)
    if (scene / "generated" / "export_workcell_studio_sources.sh").is_file():
        return export_cmd
    helper = (
        "import json; "
        "from pathlib import Path; "
        "from scripts.workcell_builder_gui_workflow import generate_files_from_yaml; "
        f"scene=Path({str(scene)!r}); "
        "print(json.dumps(generate_files_from_yaml(scene), indent=2, sort_keys=True))"
    )
    return [sys.executable, "-c", helper]


def _validation_cmd(scene: Path) -> list[str]:
    return [sys.executable, str(SCRIPT_DIR / "validate_builder_generated_scene.py"), str(scene), "--json"]


def _readiness_cmd(output_dir: Path) -> list[str]:
    return [
        sys.executable,
        str(SCRIPT_DIR / "run_workcell_studio_scene_readiness_matrix.py"),
        "--supported-scenes",
        "scenes/supported_scenes.yaml",
        "--output-dir",
        str(output_dir),
    ]


def _product_view_refresh_cmd(scene: Path, output_dir: Path, scene_id: str) -> list[str]:
    return [
        sys.executable,
        str(SCRIPT_DIR / "ensure_workcell_studio_web_scene_fresh.py"),
        "--scene",
        str(scene),
        "--output",
        str(output_dir / f"{scene_id}.web_scene.json"),
        "--stage-assets",
        "--force",
    ]


def _generated_summary_paths(scene: Path) -> list[Path]:
    return [
        scene / "package.xml",
        scene / "CMakeLists.txt",
        scene / "generated" / "cell_definition.yaml",
        scene / "generated" / "environment_layout.yaml",
    ]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run the safe guided backend workflow for Workcell Studio Web 3D edit patches.")
    parser.add_argument("--scene", required=True, type=Path, help="Scene directory, for example scenes/ur5_2f_test")
    parser.add_argument("--patch", type=Path, help="Browser-exported workcell_studio_web_scene_edit_patch/v1 JSON")
    parser.add_argument("--output-dir", default=Path("build/workcell_studio_web_scene"), type=Path)
    parser.add_argument("--export-only", action="store_true", help="Only export the before web_scene; does not require --patch.")
    parser.add_argument("--validate-only", action="store_true", help="Export before web_scene and validate the patch; do not apply.")
    parser.add_argument("--dry-run-apply", action="store_true", help="Run the existing applicator in dry-run mode. This is also the default when --write is omitted.")
    parser.add_argument("--write", action="store_true", help="Explicitly mutate editable source YAML through the safe applicator after validation and dry-run pass.")
    parser.add_argument("--verify-persistence", action="store_true", help="After --write, re-export and run persistence verification. --write already enables this.")
    parser.add_argument("--run-readiness", action="store_true", help="Optionally run the scene readiness matrix after the edit workflow.")
    parser.add_argument("--generate", action="store_true", help="Regenerate selected-scene package artifacts using the existing Workcell Builder generation flow.")
    parser.add_argument("--validate", action="store_true", help="Run scripts/validate_builder_generated_scene.py for the selected scene.")
    parser.add_argument("--generate-and-validate", action="store_true", help="Run generation first, then selected-scene validation.")
    args = parser.parse_args(argv)

    scene = args.scene
    output_dir = args.output_dir
    scene_id = _scene_id_from_dir(scene)
    before_path = output_dir / f"{scene_id}.before.web_scene.json"
    after_path = output_dir / f"{scene_id}.after.web_scene.json"

    print("Workcell Studio Web 3D guided edit workflow")
    print(f"selected scene: {scene}")
    print(f"patch path: {args.patch if args.patch else '(not required for --export-only)'}")
    print(f"before web_scene path: {before_path}")
    generate_requested = bool(args.generate or args.generate_and_validate)
    validate_requested = bool(args.validate or args.generate_and_validate)
    print(f"write enabled: {bool(args.write)}")
    print(f"generation requested: {generate_requested}")
    print(f"validation requested: {validate_requested}")

    if not scene.exists() or not scene.is_dir():
        print(f"FAIL: scene missing or not a directory: {scene}", file=sys.stderr)
        return 2
    if not args.export_only and args.patch is None and not (generate_requested or validate_requested or args.run_readiness):
        print("FAIL: --patch is required unless --export-only, --generate, --validate, --generate-and-validate, or --run-readiness is used", file=sys.stderr)
        return 2
    if args.patch is not None and not args.patch.exists():
        print(f"FAIL: patch missing: {args.patch}", file=sys.stderr)
        return 2

    try:
        before = _write_web_scene(scene, before_path)
    except Exception as exc:  # noqa: BLE001 - CLI needs clear operator failure.
        print(f"FAIL: before web_scene export failed for {scene}: {exc}", file=sys.stderr)
        return 1
    print(f"export before result: PASS ({before_path})")

    if args.export_only:
        print("patch result: SKIPPED (--export-only)")
        print("export-only result: PASS; no patch was required and no source files were modified.")
        print("Next recommended action: open the web viewer, edit an editable object, and export edit_patch.json.")
        return 0

    patch_applied = False
    if args.patch is None:
        print("patch applied/skipped: SKIPPED (no --patch provided)")
    else:
        ok, patch, errors = _validate_patch(before, args.patch)
        if not ok:
            print("patch validation result: FAIL", file=sys.stderr)
            for error in errors:
                print(f"FAIL: {error}", file=sys.stderr)
            print("write/apply result: SKIPPED because validation failed")
            return 1
        print(f"patch validation result: PASS (scene_id={patch.get('scene_id') if patch else _scene_id(before)})")

        if args.validate_only:
            print("validate-only result: PASS; no apply was attempted and no source files were modified.")
            _print_next_write_command(args)
            return 0

        dry_cmd = [sys.executable, str(SCRIPT_DIR / "apply_workcell_studio_web_scene_edit_patch.py"), "--scene", str(scene), "--web-scene", str(before_path), "--patch", str(args.patch)]
        dry_rc = _run_step("dry-run apply", dry_cmd)
        if dry_rc != 0:
            print("dry-run result: FAIL; write/apply is blocked", file=sys.stderr)
            return dry_rc
        print("dry-run result: PASS; no source files were modified by the dry run")

        if not args.write:
            print("write/apply result: SKIPPED; pass --write to mutate editable source YAML through the safe applicator")
            if generate_requested or validate_requested or args.run_readiness:
                print("generation/validation result: SKIPPED because --patch was supplied without --write")
            _print_next_write_command(args)
            return 0

        write_cmd = [*dry_cmd, "--write", "--backup"]
        write_rc = _run_step("write apply", write_cmd)
        if write_rc != 0:
            print("write/apply result: FAIL", file=sys.stderr)
            return write_rc
        patch_applied = True
        print("write/apply result: PASS (timestamped source backups created)")

        try:
            _write_web_scene(scene, after_path)
        except Exception as exc:  # noqa: BLE001
            print(f"FAIL: after web_scene export failed for {scene}: {exc}", file=sys.stderr)
            return 1
        print(f"after web_scene path: {after_path}")
        print("re-export after result: PASS")

        verify_cmd = [sys.executable, str(SCRIPT_DIR / "verify_workcell_studio_web_scene_edit_persistence.py"), "--scene", str(scene), "--web-scene-before", str(before_path), "--patch", str(args.patch), "--web-scene-after", str(after_path)]
        verify_rc = _run_step("persistence verification", verify_cmd)
        print(f"persistence verification result: {'PASS' if verify_rc == 0 else 'FAIL'}")
        if verify_rc != 0:
            return verify_rc
        print("patch applied/skipped: APPLIED")

    generation_rc: int | None = None
    validation_rc: int | None = None
    if generate_requested:
        generation_cmd = _generation_cmd(scene)
        generation_rc = _run_step("scene generation", generation_cmd)
        print(f"generation command/result: {_format_cmd(generation_cmd)} -> {'PASS' if generation_rc == 0 else 'FAIL'}")
        if generation_rc != 0:
            _print_next_generate_validate(scene, generated=False, validated=False, failed=True)
            return generation_rc
    else:
        print("generation command/result: SKIPPED (pass --generate or --generate-and-validate)")

    if validate_requested:
        validation_cmd = _validation_cmd(scene)
        validation_rc = _run_step("selected-scene validation", validation_cmd)
        print(f"validation command/result: {_format_cmd(validation_cmd)} -> {'PASS' if validation_rc == 0 else 'FAIL'}")
        if validation_rc != 0:
            _print_next_generate_validate(scene, generated=generate_requested, validated=False, failed=True)
            return validation_rc
    else:
        print("validation command/result: SKIPPED (pass --validate or --generate-and-validate)")

    if patch_applied:
        product_view_output = output_dir / f"{scene_id}.web_scene.json"
        refresh_cmd = _product_view_refresh_cmd(scene, output_dir, scene_id)
        refresh_rc = _run_step("Product View refresh", refresh_cmd)
        print(f"Product View refresh result: {'PASS' if refresh_rc == 0 else 'FAIL'} ({product_view_output})")
        if refresh_rc != 0:
            return refresh_rc

    readiness_output_dir: Path | None = None
    if args.run_readiness:
        readiness_output_dir = Path("build/workcell_studio_scene_readiness")
        readiness_cmd = _readiness_cmd(readiness_output_dir)
        print(f"readiness output path: {readiness_output_dir / 'scene_readiness_summary.json'}")
        readiness_rc = _run_step("readiness matrix", readiness_cmd)
        print(f"readiness result: {'PASS' if readiness_rc == 0 else 'FAIL'}")
        if readiness_rc != 0:
            return readiness_rc
    else:
        print("readiness result: SKIPPED (pass --run-readiness to run it)")

    if generation_rc == 0:
        print("generated output paths:")
        for generated_path in _generated_summary_paths(scene):
            print(f"  - {generated_path}")
    if readiness_output_dir is not None:
        print("readiness output paths:")
        print(f"  - {readiness_output_dir / 'scene_readiness_summary.json'}")

    print(f"\nWorkflow summary: PASS (selected scene: {scene}; patch_applied={patch_applied}; generation={'PASS' if generation_rc == 0 else 'SKIPPED'}; validation={'PASS' if validation_rc == 0 else 'SKIPPED'})")
    _print_next_generate_validate(scene, generated=bool(generation_rc == 0), validated=bool(validation_rc == 0))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
