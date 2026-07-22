#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import sys

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from scripts.workcell_studio_script_bootstrap import ensure_repo_root_on_sys_path

ensure_repo_root_on_sys_path(__file__)

from scripts.workcell_studio_path_resolver import (
    resolve_install_setup,
    resolve_repo_root,
    resolve_workcell_builder_executable,
    resolve_workspace_root,
)

PASS = "PASS"
WARN = "WARN"
FAIL = "FAIL"
SKIP = "SKIP"
STATUSES = (PASS, WARN, FAIL, SKIP)
READINESS_DIMENSIONS = (
    "artifact_readiness",
    "preview_readiness",
    "moveit_launch_readiness",
    "grasp_planner_readiness",
    "grasp_execution_readiness",
    "real_hardware_readiness",
)

DEFAULT_SIM_REPORT = Path("build/workcell_studio/rviz_moveit_simulation_launch_report.json")
DEFAULT_JSON_OUTPUT = Path("build/workcell_studio/workcell_studio_scene_readiness_gate.json")
DEFAULT_MD_OUTPUT = Path("build/workcell_studio/workcell_studio_scene_readiness_gate.md")
DEFAULT_AUDIT_REPORT = Path("build/workcell_studio/all_scene_reproducibility_report.json")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run fake-hardware scene readiness gate across all Workcell Studio scenes.")
    p.add_argument("--timeout-sec", type=int, default=45)
    p.add_argument("--dry-run-launches", action="store_true", help="Forward --dry-run to launch validator")
    p.add_argument("--launch-rviz", action="store_true", help="Run launch validator in GUI mode")
    p.add_argument("--json-output", type=Path, default=DEFAULT_JSON_OUTPUT)
    p.add_argument("--markdown-output", type=Path, default=DEFAULT_MD_OUTPUT)
    p.add_argument("--strict", action="store_true", help="Exit non-zero if any supported scene has FAIL readiness")
    p.add_argument("--include-visual-assets", action="store_true", help="Run visual asset inventory and include report links")
    p.add_argument("--include-scene3d-gui-smoke", action="store_true", help="Run Scene3D GUI smoke for ur5_2f_test and include result")
    p.add_argument("--repo-root", type=Path, default=None)
    p.add_argument("--workspace-root", type=Path, default=None)
    p.add_argument("--workcell-builder-executable", type=Path, default=None)
    p.add_argument("--include-local-validation", action="store_true")
    p.add_argument("--output-dir", type=Path, default=Path("build/workcell_studio"))
    return p.parse_args()


def _run_command(cmd: list[str], repo_root: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, cwd=repo_root, capture_output=True, text=True)


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _status_counts() -> dict[str, int]:
    return {k: 0 for k in STATUSES}


def _is_ci_environment() -> bool:
    return str(os.environ.get("CI", "")).strip().lower() in {"1", "true", "yes", "on"}


def _resolve_setup_candidates(workspace_root: Path | None, repo_root: Path) -> list[Path]:
    candidates: list[Path] = []
    if workspace_root is not None:
        candidates.append(workspace_root / "install" / "setup.bash")
    candidates.extend([repo_root / "install" / "setup.bash", repo_root.parent / "install" / "setup.bash"])
    return candidates


def _resolve_executable_candidates(workspace_root: Path | None, repo_root: Path) -> list[Path]:
    candidates: list[Path] = []
    if workspace_root is not None:
        candidates.extend(
            [
                workspace_root / "install" / "workcell_builder" / "lib" / "workcell_builder" / "workcell_builder",
                workspace_root / "build" / "workcell_builder" / "workcell_builder",
            ]
        )
    candidates.extend(
        [
            repo_root / "install" / "workcell_builder" / "lib" / "workcell_builder" / "workcell_builder",
            repo_root / "build" / "workcell_builder" / "workcell_builder",
        ]
    )
    return candidates


def _run_scene3d_consolidated_gate(repo_root: Path, scenes: list[str]) -> dict[str, Any]:
    gate: dict[str, Any] = {"status": PASS, "scenes": [], "blockers": [], "warnings": []}
    out_root = repo_root / "build/workcell_studio"
    out_root.mkdir(parents=True, exist_ok=True)

    for scene in scenes:
        runtime_json = out_root / f"scene3d_runtime_acceptance_{scene}.json"
        runtime_md = out_root / f"scene3d_runtime_acceptance_{scene}.md"
        contract_json = out_root / f"scene3d_canvas_contract_{scene}.json"
        contract_md = out_root / f"scene3d_canvas_contract_{scene}.md"
        smoke_json = out_root / f"scene3d_gui_smoke_{scene}.json"
        smoke_png = out_root / f"scene3d_gui_smoke_{scene}.png"

        runtime_cmd = ["python3", "scripts/validate_scene3d_runtime_acceptance.py", "--scene", scene, "--json", str(runtime_json), "--markdown", str(runtime_md), "--smoke-json", str(smoke_json)]
        contract_cmd = ["python3", "scripts/check_scene3d_canvas_contract.py", "--scene", scene, "--json", str(contract_json), "--markdown", str(contract_md)]
        smoke_cmd = ["python3", "scripts/run_workcell_builder_scene3d_gui_smoke.py", "--scene", scene, "--output", str(smoke_json), "--screenshot", str(smoke_png)]

        smoke_proc = _run_command(smoke_cmd, repo_root)
        runtime_proc = _run_command(runtime_cmd, repo_root)
        contract_proc = _run_command(contract_cmd, repo_root)

        runtime_payload = _load_json(runtime_json) if runtime_json.exists() else {}
        contract_payload = _load_json(contract_json) if contract_json.exists() else {}
        smoke_payload = _load_json(smoke_json) if smoke_json.exists() else {}

        contract_scene = next((s for s in contract_payload.get("scenes", []) if isinstance(s, dict) and s.get("scene") == scene), {})
        runtime_scene = next((s for s in runtime_payload.get("scenes", []) if isinstance(s, dict) and s.get("scene") == scene), {})

        primitive_count = int(contract_scene.get("primitive_fallback_count", 0) or 0)
        mesh_count = int(contract_scene.get("mesh_preview_count", 0) or 0)
        editable_layout_count = int(contract_scene.get("editable_layout_count", 0) or 0)
        unresolved_placeholders = int(contract_scene.get("unresolved_placeholder_count", 0) or 0)
        visible_after_filters = int(contract_scene.get("visible_after_filters_count", 0) or 0)
        hierarchy_rows = int(smoke_payload.get("hierarchy_rows_count", 0) or 0)
        unique_visible_item_count = smoke_payload.get("unique_visible_item_count", smoke_payload.get("visible_unique_item_count", visible_after_filters))
        unique_visible_item_count = int(unique_visible_item_count or 0)
        selected_scene = str(smoke_payload.get("selected_scene") or smoke_payload.get("open_scene") or "").strip()
        selected_item = str(smoke_payload.get("selected_item_id") or smoke_payload.get("selected_item") or "").strip()

        scene_blockers: list[str] = []
        if visible_after_filters > 0 and unresolved_placeholders >= visible_after_filters:
            scene_blockers.append(f"{scene}: visible objects exist but are placeholder-only (visible={visible_after_filters}, unresolved_placeholders={unresolved_placeholders})")
        if mesh_count == 0 and primitive_count == 0 and editable_layout_count == 0:
            scene_blockers.append(f"{scene}: mesh_preview_count + primitive_fallback_count + editable_layout_count are all zero")
        if scene in {"ur5_2f_test", "ur5_2f_sorting_test"} and unique_visible_item_count == 1:
            scene_blockers.append(f"{scene}: unique visible item count is 1")
        if hierarchy_rows == 0:
            scene_blockers.append(f"{scene}: hierarchy_rows_count is zero")
        if not selected_scene and not selected_item:
            scene_blockers.append(f"{scene}: inspector has no selected/open scene")

        scene_report = {
            "scene": scene,
            "status": FAIL if scene_blockers or smoke_proc.returncode != 0 or runtime_proc.returncode != 0 or contract_proc.returncode != 0 else PASS,
            "returncodes": {"smoke": smoke_proc.returncode, "runtime_acceptance": runtime_proc.returncode, "canvas_contract": contract_proc.returncode},
            "diagnostics": {"visible_after_filters_count": visible_after_filters, "unresolved_placeholder_count": unresolved_placeholders, "mesh_preview_count": mesh_count, "primitive_fallback_count": primitive_count, "editable_layout_count": editable_layout_count, "unique_visible_item_count": unique_visible_item_count, "hierarchy_rows_count": hierarchy_rows, "selected_scene": selected_scene, "selected_item": selected_item},
            "blockers": scene_blockers,
            "artifacts": {"runtime_json": str(runtime_json), "contract_json": str(contract_json), "smoke_json": str(smoke_json), "smoke_screenshot": str(smoke_png)},
        }
        gate["scenes"].append(scene_report)
        gate["blockers"].extend(scene_blockers)
        if scene_report["status"] == FAIL:
            gate["status"] = FAIL
    return gate


def build_gate_report(sim_report_path: Path, audit_report_path: Path, audit_payload: dict[str, Any], visual_assets: dict[str, Any] | None = None, mesh_index_regeneration: dict[str, Any] | None = None, scene3d_gui_smoke: dict[str, Any] | None = None) -> dict[str, Any]:
    scenes_out: list[dict[str, Any]] = []
    counts: dict[str, dict[str, int]] = {dim: _status_counts() for dim in READINESS_DIMENSIONS}

    for scene in audit_payload.get("scenes", []):
        readiness = scene.get("readiness", {}) if isinstance(scene, dict) else {}
        summary = {
            "scene": scene.get("scene_name", "unknown"),
            "artifact_readiness": str(readiness.get("artifact_readiness", {}).get("status", WARN)).upper(),
            "preview_readiness": str(readiness.get("preview_readiness", {}).get("status", WARN)).upper(),
            "moveit_launch_readiness": str(readiness.get("moveit_launch_readiness", {}).get("status", WARN)).upper(),
            "grasp_planner_readiness": str(readiness.get("grasp_planner_readiness", {}).get("status", WARN)).upper(),
            "grasp_execution_readiness": str(readiness.get("grasp_execution_readiness", {}).get("status", WARN)).upper(),
            "real_hardware_readiness": str(readiness.get("real_hardware_readiness", {}).get("status", WARN)).upper(),
            "blockers": scene.get("blockers", []),
            "warnings": scene.get("warnings", []),
        }
        for dim in READINESS_DIMENSIONS:
            status = summary[dim]
            if status not in STATUSES:
                status = WARN
                summary[dim] = WARN
            counts[dim][status] += 1
        scenes_out.append(summary)

    return {
        "schema": "workcell_studio_scene_readiness_gate/v1",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "simulation_launch_report_path": str(sim_report_path),
        "all_scene_audit_report_path": str(audit_report_path),
        "scenes": scenes_out,
        "counts": counts,
        "safety_note": "This is fake-hardware simulation readiness only, not real-hardware validation.",
        "visual_asset_inventory": visual_assets or {},
        "visual_mesh_index_regeneration": mesh_index_regeneration or {},
        "scene3d_gui_smoke": scene3d_gui_smoke or {},
    }


def write_markdown(path: Path, report: dict[str, Any]) -> None:
    lines = [
        "# Workcell Studio Scene Readiness Gate",
        "",
        "This is fake-hardware simulation readiness only, not real-hardware validation.",
        "",
        "## Scene Summary",
        "",
        "| Scene | Artifact | Preview | MoveIt Launch | Grasp Planner | Grasp Execution | Real Hardware |",
        "|---|---|---|---|---|---|---|",
    ]
    for row in report["scenes"]:
        lines.append(
            f"| {row['scene']} | {row['artifact_readiness']} | {row['preview_readiness']} | {row['moveit_launch_readiness']} | {row['grasp_planner_readiness']} | {row['grasp_execution_readiness']} | {row['real_hardware_readiness']} |"
        )

    lines.extend(["", "## Blockers by Scene", ""])
    for row in report["scenes"]:
        lines.append(f"### {row['scene']}")
        if row["blockers"]:
            lines.extend([f"- {b}" for b in row["blockers"]])
        else:
            lines.append("- None")

    lines.extend(["", "## Warnings by Scene", ""])
    for row in report["scenes"]:
        lines.append(f"### {row['scene']}")
        if row["warnings"]:
            lines.extend([f"- {w}" for w in row["warnings"]])
        else:
            lines.append("- None")

    lines.extend(
        [
            "",
            "## Visual Asset Summary",
            "",
            json.dumps(report.get("visual_asset_inventory", {}).get("summary", {}), indent=2),
            "",
            "## Mesh Index Summary",
            "",
            json.dumps(report.get("visual_mesh_index_regeneration", {}).get("summary", {}), indent=2),
            "",
            "## Scene3D GUI Smoke",
            "",
            json.dumps(report.get("scene3d_gui_smoke", {}), indent=2),
            "",
            "## Scene3D GUI Smoke Artifacts (Absolute Paths)",
            "",
        ]
    )
    smoke_artifacts = report.get("scene3d_gui_smoke", {}).get("artifacts", {})
    if isinstance(smoke_artifacts, dict) and smoke_artifacts:
        for scene_name, artifact in smoke_artifacts.items():
            if isinstance(artifact, dict):
                lines.append(f"### {scene_name}")
                lines.append(f"- JSON: `{artifact.get('json_path_abs', '')}`")
                lines.append(f"- Screenshot: `{artifact.get('screenshot_path_abs', '')}`")
    else:
        lines.append("- None")

    lines.extend(
        [
            "",
            "## Suggested Fixes",
            "",
            "- Regenerate visual mesh index on this workspace",
            "- Check package map for missing mesh package",
            "- Add asset fallback or primitive placeholder",
            "",
            "",
            "## Next Manual Commands",
            "",
            "```bash",
            "python3 scripts/run_workcell_studio_scene_readiness_gate.py --dry-run-launches",
            "python3 scripts/run_workcell_studio_scene_readiness_gate.py --timeout-sec 45",
            "python3 scripts/run_workcell_studio_scene_readiness_gate.py --launch-rviz --timeout-sec 60",
            "python3 scripts/run_workcell_studio_scene_readiness_gate.py --dry-run-launches --strict",
            "```",
            "",
            "Safety constraints enforced in this gate include use_fake_hardware:=true.",
        ]
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    repo_arg = getattr(args, "repo_root", None)
    workspace_arg = getattr(args, "workspace_root", None)
    repo_root = (repo_arg.resolve() if repo_arg else Path(__file__).resolve().parents[1])
    workspace_root = workspace_arg.resolve() if workspace_arg else resolve_workspace_root(repo_root)

    sim_report_path = args.json_output.parent / "rviz_moveit_simulation_launch_report.json"
    sim_report_path = DEFAULT_SIM_REPORT if sim_report_path != DEFAULT_SIM_REPORT else sim_report_path

    sim_cmd = [
        "python3",
        "scripts/validate_rviz_moveit_simulation_launches.py",
        "--all",
        "--timeout-sec",
        str(args.timeout_sec),
        "--json-output",
        str(DEFAULT_SIM_REPORT),
        "--headless" if not args.launch_rviz else "--launch-rviz",
    ]
    if args.dry_run_launches:
        sim_cmd.append("--dry-run")

    if "use_fake_hardware:=false" in " ".join(sim_cmd) or "fake_hardware:=false" in " ".join(sim_cmd):
        raise SystemExit("Safety violation: fake hardware must remain true")

    sim_proc = _run_command(sim_cmd, repo_root)

    audit_cmd = [
        "python3",
        "scripts/validate_all_workcell_studio_scenes.py",
        "--simulation-launch-report",
        str(DEFAULT_SIM_REPORT),
    ]
    audit_proc = _run_command(audit_cmd, repo_root)

    audit_report_path = repo_root / DEFAULT_AUDIT_REPORT
    audit_payload = _load_json(audit_report_path) if audit_report_path.exists() else {"scenes": []}

    visual_assets = None
    if getattr(args, "include_visual_assets", False):
        subprocess.run(["python3", "scripts/regenerate_scene_visual_mesh_indexes.py", "--all", "--portable"], cwd=repo_root, capture_output=True, text=True)
        va_cmd = ["python3", "scripts/audit_workcell_studio_visual_assets.py"]
        va_proc = _run_command(va_cmd, repo_root)
        va_json = repo_root / "build/workcell_studio/visual_asset_inventory.json"
        visual_assets = {"command_returncode": va_proc.returncode, "json_report": str(va_json), "markdown_report": "build/workcell_studio/visual_asset_inventory.md"}
        if va_json.exists():
            visual_assets["summary"] = _load_json(va_json).get("summary", {})
    mesh_index_regeneration = None
    if getattr(args, "include_visual_assets", False):
        regen_json = repo_root / "build/workcell_studio/visual_mesh_index_regeneration_report.json"
        if regen_json.exists():
            regen = _load_json(regen_json)
            mesh_index_regeneration = {"json_report": str(regen_json), "summary": {"scene_count": len(regen.get("scenes", [])), "xacro_fallback_scenes": [r.get("scene") for r in regen.get("scenes", []) if r.get("extraction_mode") != "xacro_expanded"], "unsafe_scenes": [r.get("scene") for r in regen.get("scenes", []) if not r.get("safe_for_preview", False)]}}
    scene3d_gui_smoke = None
    if getattr(args, "include_scene3d_gui_smoke", False):
        smoke_script = repo_root / "scripts/run_workcell_builder_scene3d_gui_smoke.py"
        smoke_scenes = ("ur5_2f_test", "ur5_2f_sorting_test")
        mode = "ci_or_dry_run" if (args.dry_run_launches or _is_ci_environment()) else "local_validation"
        scene3d_gui_smoke = {
            "mode": mode,
            "status": WARN if mode == "ci_or_dry_run" else PASS,
            "artifacts": {},
            "scenes": [],
            "warnings": [],
            "blockers": [],
        }
        if not smoke_script.exists():
            marker = "CI_OR_DRY_RUN_DOWNGRADE" if mode == "ci_or_dry_run" else "LOCAL_VALIDATION_REQUIRED"
            msg = f"{marker}: GUI smoke script missing: {smoke_script}"
            if mode == "ci_or_dry_run":
                scene3d_gui_smoke["warnings"].append(msg)
            else:
                scene3d_gui_smoke["blockers"].append(msg)
                scene3d_gui_smoke["status"] = FAIL
        else:
            resolved_setup = resolve_install_setup(workspace_root)
            executable_arg = getattr(args, "workcell_builder_executable", None)
            resolved_executable = executable_arg.resolve() if executable_arg else resolve_workcell_builder_executable(workspace_root)
            setup_candidates = _resolve_setup_candidates(workspace_root, repo_root)
            executable_candidates = [executable_arg.resolve()] if executable_arg else _resolve_executable_candidates(workspace_root, repo_root)
            if resolved_setup is None:
                msg = "missing install setup.bash; searched=" + " | ".join(str(p) for p in setup_candidates)
                if mode == "ci_or_dry_run":
                    scene3d_gui_smoke["warnings"].append(f"CI_OR_DRY_RUN_DOWNGRADE: {msg}")
                else:
                    scene3d_gui_smoke["blockers"].append(msg)
                    scene3d_gui_smoke["status"] = FAIL
            if resolved_executable is None:
                msg = "missing workcell_builder executable; searched=" + " | ".join(str(p) for p in executable_candidates)
                if mode == "ci_or_dry_run":
                    scene3d_gui_smoke["warnings"].append(f"CI_OR_DRY_RUN_DOWNGRADE: {msg}")
                else:
                    scene3d_gui_smoke["blockers"].append(msg)
                    scene3d_gui_smoke["status"] = FAIL
            required_fields = ("status", "scene", "timestamp")
            for scene_name in smoke_scenes:
                if mode == "local_validation" and scene3d_gui_smoke.get("status") == FAIL:
                    continue
                smoke_json = repo_root / f"build/workcell_studio/scene3d_gui_smoke_{scene_name}.json"
                smoke_png = repo_root / f"build/workcell_studio/scene3d_gui_smoke_{scene_name}.png"
                scene3d_gui_smoke["artifacts"][scene_name] = {
                    "json_path_abs": str(smoke_json.resolve()),
                    "screenshot_path_abs": str(smoke_png.resolve()),
                }
                smoke_cmd = [
                    "python3",
                    str(smoke_script),
                    "--repo-root",
                    str(repo_root),
                    "--workspace-root",
                    str(workspace_root or repo_root),
                    "--scene",
                    scene_name,
                    "--output",
                    str(smoke_json),
                    "--screenshot",
                    str(smoke_png),
                ]
                if resolved_executable is not None:
                    smoke_cmd.extend(["--executable", str(resolved_executable)])
                smoke_proc = _run_command(smoke_cmd, repo_root)
                scene_result: dict[str, Any] = {"scene": scene_name, "returncode": smoke_proc.returncode}
                scene3d_gui_smoke["scenes"].append(scene_result)

                if mode == "local_validation" and smoke_proc.returncode != 0:
                    scene3d_gui_smoke["status"] = FAIL
                    scene3d_gui_smoke["blockers"].append(f"{scene_name}: local smoke launch failed; stderr:\n{(smoke_proc.stderr or '').strip()}")
                    continue

                missing: list[str] = []
                if not smoke_json.exists():
                    missing.append("smoke_json")
                if not smoke_png.exists():
                    missing.append("smoke_screenshot")
                if missing:
                    msg = f"{scene_name}: missing artifacts: {', '.join(missing)}"
                    if mode == "ci_or_dry_run":
                        scene3d_gui_smoke["warnings"].append(f"CI_OR_DRY_RUN_DOWNGRADE: {msg}")
                    else:
                        scene3d_gui_smoke["blockers"].append(msg)
                        scene3d_gui_smoke["status"] = FAIL
                    continue

                smoke_payload = _load_json(smoke_json)
                scene_result["smoke_report"] = smoke_payload
                missing_fields = [field for field in required_fields if field not in smoke_payload]
                if missing_fields:
                    msg = f"{scene_name}: required smoke JSON fields missing: {', '.join(missing_fields)}"
                    if mode == "ci_or_dry_run":
                        scene3d_gui_smoke["warnings"].append(f"CI_OR_DRY_RUN_DOWNGRADE: {msg}")
                    else:
                        scene3d_gui_smoke["blockers"].append(msg)
                        scene3d_gui_smoke["status"] = FAIL

                reported_status = str(smoke_payload.get("status", "")).upper()
                if reported_status == FAIL:
                    msg = f"{scene_name}: smoke status is FAIL"
                    if mode == "ci_or_dry_run":
                        scene3d_gui_smoke["warnings"].append(f"CI_OR_DRY_RUN_DOWNGRADE: {msg}")
                    else:
                        scene3d_gui_smoke["blockers"].append(msg)
                        scene3d_gui_smoke["status"] = FAIL
                elif reported_status not in {PASS, "OK"}:
                    scene3d_gui_smoke["warnings"].append(f"{scene_name}: unrecognized smoke status '{reported_status or 'UNKNOWN'}'")

            if mode == "ci_or_dry_run" and scene3d_gui_smoke["status"] != FAIL:
                scene3d_gui_smoke["status"] = WARN

    local_validation = {}
    if getattr(args, "include_local_validation", False):
        lv_cmd = [
            "python3",
            "scripts/run_workcell_studio_local_validation.py",
            "--repo-root", str(repo_root),
            "--output-dir", str(getattr(args, "output_dir", Path("build/workcell_studio")) / "local_validation"),
            "--include-gui-smoke",
            "--dry-run" if args.dry_run_launches or _is_ci_environment() else "--require-gui-smoke",
        ]
        if workspace_root:
            lv_cmd += ["--workspace-root", str(workspace_root)]
        if getattr(args, "workcell_builder_executable", None):
            lv_cmd += ["--workcell-builder-executable", str(args.workcell_builder_executable.resolve())]
        lv_proc = _run_command([c for c in lv_cmd if c], repo_root)
        lv_json = repo_root / (args.output_dir / "local_validation/workcell_studio_local_validation.json")
        local_validation = {"returncode": lv_proc.returncode, "json_report": str(lv_json)}

    scene_names = [row.get("scene_name") for row in audit_payload.get("scenes", []) if isinstance(row, dict) and row.get("scene_name")]
    if getattr(args, "dry_run_launches", False):
        scene3d_consolidated_gate = {"status": WARN, "scenes": [], "blockers": [], "warnings": ["dry_run_launches: Scene3D consolidated gate not run"]}
    else:
        scene3d_consolidated_gate = _run_scene3d_consolidated_gate(repo_root, scene_names)
    final_report = build_gate_report(DEFAULT_SIM_REPORT, DEFAULT_AUDIT_REPORT, audit_payload, visual_assets, mesh_index_regeneration, scene3d_gui_smoke)
    final_report["scene3d_consolidated_gate"] = scene3d_consolidated_gate
    final_report["local_validation"] = local_validation
    args.json_output.parent.mkdir(parents=True, exist_ok=True)
    args.json_output.write_text(json.dumps(final_report, indent=2) + "\n", encoding="utf-8")
    write_markdown(args.markdown_output, final_report)

    strict_fail = args.strict and any(scene["artifact_readiness"] == FAIL or scene["preview_readiness"] == FAIL or scene["moveit_launch_readiness"] == FAIL for scene in final_report["scenes"])

    scene3d_fail = getattr(args, "include_scene3d_gui_smoke", False) and bool(scene3d_gui_smoke) and scene3d_gui_smoke.get("status") == FAIL
    scene3d_consolidated_fail = scene3d_consolidated_gate.get("status") == FAIL

    local_validation_fail = bool(local_validation) and local_validation.get("returncode",0) != 0
    if sim_proc.returncode != 0 or audit_proc.returncode != 0 or strict_fail or scene3d_fail or scene3d_consolidated_fail or local_validation_fail:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
