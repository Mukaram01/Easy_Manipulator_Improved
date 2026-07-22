#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml  # type: ignore

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from supported_scene_catalog import SupportedSceneEntry, default_catalog_path, load_supported_scene_catalog

WARNING_CATEGORIES = ("metadata", "preview", "generation", "launch_simulation", "runtime_smoke")
READINESS_DIMENSIONS = (
    "artifact_readiness",
    "preview_readiness",
    "moveit_launch_readiness",
    "grasp_planner_readiness",
    "grasp_execution_readiness",
    "real_hardware_readiness",
)

PASS = "PASS"
WARN = "WARN"
FAIL = "FAIL"
SKIP = "SKIP"
BLOCKED = "BLOCKED"
VALID_STATUSES = {PASS, WARN, FAIL, SKIP, BLOCKED}

REQUIRED_FILES = {
    "scene_manifest": "scene_manifest.yaml",
    "environment": "environment.yaml",
    "layout": "layout/workcell_studio_layout.yaml",
    "demo_launch": "launch/demo.launch.py",
}

OPTIONAL_FILES = {
    "cell_definition": "cell_definition.yaml",
    "scene_urdf_xacro": "urdf/scene.urdf.xacro",
    "scene_urdf": "urdf/scene.urdf",
    "task_recipe": "config/task_recipe.yaml",
    "task_intent": "config/workcell_builder_task_intent.yaml",
    "generated_environment_assets": "generated/environment_assets.yaml",
    "generated_layout": "layout/workcell_studio_layout.generated.yaml",
}



@dataclass
class ReadinessDimension:
    status: str
    reasons: list[str]


@dataclass
class SceneAudit:
    scene_name: str
    support_level: str
    catalog_status: str
    known_blocker: str
    status: str
    files: dict[str, bool]
    optional_files: dict[str, bool]
    generated_mesh_index_present: bool
    mesh_index_regeneration_status: str
    mesh_index_renderable_items: int
    preview_readiness_status: str
    generated_artifacts_present: dict[str, bool]
    fake_hardware_smoke_command_available: bool
    fake_hardware_smoke_command: str
    blockers: list[str]
    warnings: list[str]
    warning_groups: dict[str, list[str]]
    readiness: dict[str, ReadinessDimension]


def _dim(status: str, reasons: list[str]) -> ReadinessDimension:
    if status not in VALID_STATUSES:
        raise ValueError(f"invalid readiness status: {status}")
    return ReadinessDimension(status=status, reasons=reasons)


def _with_prefix(prefix: str, entries: list[str]) -> list[str]:
    return [f"{prefix}: {entry}" for entry in entries]


def resolve_scenes_root(repo_root: Path) -> Path:
    for candidate in [repo_root / "scenes", repo_root / "easy_manipulation_deployment" / "scenes"]:
        if candidate.is_dir():
            return candidate
    raise FileNotFoundError("Could not find scenes directory")


def _load_yaml(path: Path) -> tuple[Any | None, str]:
    if not path.exists():
        return None, "missing"
    try:
        with path.open("r", encoding="utf-8") as handle:
            return yaml.safe_load(handle), "ok"
    except Exception as exc:  # noqa: BLE001
        return None, f"parse_error: {exc.__class__.__name__}: {exc}"


def _layout_metadata_issues(layout_data: Any) -> list[str]:
    if not isinstance(layout_data, dict):
        return ["layout root must be a YAML map"]
    issues: list[str] = []
    if layout_data.get("schema_version") != "workcell_studio_layout/v1":
        issues.append("schema_version must be 'workcell_studio_layout/v1'")
    items = layout_data.get("items")
    if not isinstance(items, list):
        issues.append("items must be a sequence")
        return issues
    if not items:
        issues.append("items is empty")
    return issues


def _read_mesh_index(scene_dir: Path) -> tuple[int, str]:
    index_path = scene_dir / "generated" / "scene_visual_mesh_index.json"
    if not index_path.exists():
        return 0, "missing"
    try:
        payload = json.loads(index_path.read_text(encoding="utf-8"))
        items = None
        if isinstance(payload, dict):
            items = payload.get("items")
            if not isinstance(items, list):
                items = payload.get("visual_items")
        if not isinstance(items, list):
            return 0, "invalid_json"
        renderable = sum(1 for item in items if isinstance(item, dict) and item.get("render_expected", True))
        return renderable, "ok"
    except Exception as exc:  # noqa: BLE001
        return 0, f"error: {exc.__class__.__name__}: {exc}"


def _run_extract_for_scene(repo_root: Path, scene_name: str) -> tuple[str, str]:
    extractor = repo_root / "scripts" / "extract_scene_urdf_visual_mesh_index.py"
    if not extractor.exists():
        return "extractor_missing", "extractor script missing"
    proc = subprocess.run(
        ["python3", str(extractor), "--scene", scene_name, "--prefer-xacro"],
        cwd=repo_root,
        capture_output=True,
        text=True,
    )
    if proc.returncode == 0:
        return "ok", ""
    reason = (proc.stderr or proc.stdout).strip().splitlines()
    return "failed", (reason[-1] if reason else "extractor failed")


def _fake_hardware_command_available(scene_dir: Path) -> bool:
    return (scene_dir / "launch" / "demo.launch.py").exists()


def _load_launch_simulation_report(scene_dir: Path) -> tuple[str | None, list[str]]:
    report_path = scene_dir / "generated" / "fake_hardware_smoke_launch_report.json"
    if not report_path.exists():
        return None, []
    try:
        payload = json.loads(report_path.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001
        return FAIL, [f"launch simulation report unreadable: {exc.__class__.__name__}: {exc}"]
    if payload.get("schema") != "fake_hardware_smoke_launch_report/v1":
        return SKIP, ["launch simulation report schema unsupported"]
    status = str((payload.get("result") or {}).get("status") or "").upper()
    if status not in {PASS, WARN, FAIL, SKIP}:
        return WARN, [f"launch simulation report returned unknown status: {status or '(missing)'}"]
    reasons: list[str] = []
    for key in ("warnings", "errors"):
        for msg in (payload.get("result") or {}).get(key, []) or []:
            if isinstance(msg, str) and msg.strip():
                reasons.append(f"launch simulation report {key[:-1]}: {msg}")
    return status, reasons




def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate Workcell Studio scenes and readiness dimensions.")
    parser.add_argument(
        "--catalog",
        type=Path,
        default=None,
        help="Supported-scene catalog path. Defaults to scenes/supported_scenes.yaml.",
    )
    parser.add_argument(
        "--simulation-launch-report",
        type=Path,
        default=Path("build/workcell_studio/rviz_moveit_simulation_launch_report.json"),
        help="Path to rviz/moveit simulation launch report JSON.",
    )
    parser.add_argument(
        "--include-experimental",
        action="store_true",
        help="Audit scenes marked support_level: experimental. By default they are reported as SKIP without filesystem validation.",
    )
    parser.add_argument(
        "--regenerate-missing-mesh-indexes",
        action="store_true",
        help="Regenerate missing generated/scene_visual_mesh_index.json files during audit. Disabled by default to avoid mutating the repository.",
    )
    return parser.parse_args()


def _load_simulation_launch_report(
    repo_root: Path, report_path: Path
) -> tuple[dict[str, dict[str, Any]] | None, str | None]:
    resolved = report_path if report_path.is_absolute() else repo_root / report_path
    if not resolved.exists():
        return None, "no simulation launch evidence"
    try:
        payload = json.loads(resolved.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001
        return None, f"simulation launch report unreadable: {exc.__class__.__name__}: {exc}"
    if payload.get("schema") != "rviz_moveit_simulation_launch_report/v1":
        return None, "simulation launch report schema unsupported"

    results = payload.get("results")
    if not isinstance(results, list):
        return None, "simulation launch report missing results list"

    by_scene: dict[str, dict[str, Any]] = {}
    for entry in results:
        if not isinstance(entry, dict):
            continue
        scene = entry.get("scene")
        if isinstance(scene, str) and scene.strip():
            by_scene[scene] = entry
    return by_scene, None

def audit_scene(
    repo_root: Path,
    scene_dir: Path,
    simulation_results_by_scene: dict[str, dict[str, Any]] | None = None,
    simulation_report_error: str | None = None,
    support_level: str = "ad_hoc",
    catalog_status: str = "ad_hoc",
    known_blocker: str = "",
    regenerate_missing_mesh_indexes: bool = False,
) -> SceneAudit:
    files = {k: (scene_dir / rel).exists() for k, rel in REQUIRED_FILES.items()}
    optional = {k: (scene_dir / rel).exists() for k, rel in OPTIONAL_FILES.items()}

    blockers: list[str] = []
    warning_groups: dict[str, list[str]] = {k: [] for k in WARNING_CATEGORIES}

    for key, present in files.items():
        if not present:
            blockers.append(f"missing required file: {REQUIRED_FILES[key]}")

    manifest_data, manifest_status = _load_yaml(scene_dir / "scene_manifest.yaml")
    env_data, env_status = _load_yaml(scene_dir / "environment.yaml")
    layout_data, layout_status = _load_yaml(scene_dir / "layout" / "workcell_studio_layout.yaml")

    for name, status in {
        "scene_manifest.yaml": manifest_status,
        "environment.yaml": env_status,
        "layout/workcell_studio_layout.yaml": layout_status,
    }.items():
        if status.startswith("parse_error"):
            blockers.append(f"YAML parse failure in {name}: {status}")

    layout_issues = _layout_metadata_issues(layout_data) if layout_status == "ok" else ["layout not parseable"]
    if layout_issues:
        warning_groups["metadata"].extend([f"layout metadata issue: {issue}" for issue in layout_issues])
        warning_groups["preview"].append("preview readiness degraded by layout metadata issues")

    generated_index = (scene_dir / "generated" / "scene_visual_mesh_index.json").exists()
    regen_status = "not_needed" if generated_index else "skipped_missing_regeneration_disabled"
    regen_reason = ""
    if not generated_index and regenerate_missing_mesh_indexes:
        regen_status = "attempted"
        regen_status, regen_reason = _run_extract_for_scene(repo_root, scene_dir.name)
        if regen_status != "ok":
            blockers.append(f"visual mesh index regeneration failed: {regen_reason}")
    elif not generated_index:
        blockers.append(
            "generated mesh index missing; rerun with --regenerate-missing-mesh-indexes to recreate it"
        )

    renderable_items, mesh_index_status = _read_mesh_index(scene_dir)
    if mesh_index_status != "ok":
        blockers.append(f"generated mesh index unreadable: {mesh_index_status}")
    elif renderable_items <= 0:
        blockers.append("generated mesh index has no renderable items")

    if not optional["scene_urdf_xacro"] and not optional["scene_urdf"]:
        blockers.append("missing urdf/scene.urdf.xacro or urdf/scene.urdf")

    generated_artifacts_present = {
        "generated_environment_assets": optional["generated_environment_assets"],
        "generated_layout": optional["generated_layout"],
        "task_recipe": optional["task_recipe"],
        "task_intent": optional["task_intent"],
    }

    fake_cmd = f"python3 scripts/run_fake_hardware_smoke_launch.py --scene {scene_dir.name}"
    smoke_available = _fake_hardware_command_available(scene_dir)

    for key, present in optional.items():
        if not present and key not in {"scene_urdf", "scene_urdf_xacro"}:
            if key in {"task_recipe", "task_intent", "generated_environment_assets", "generated_layout"}:
                warning_groups["generation"].append(f"optional file missing: {OPTIONAL_FILES[key]}")
            else:
                warning_groups["metadata"].append(f"optional file missing: {OPTIONAL_FILES[key]}")

    preview_readiness = "ready" if not blockers and not layout_issues else ("degraded" if not blockers else "blocked")

    warnings = [w for group in WARNING_CATEGORIES for w in warning_groups[group] if w]

    metadata_warnings = warning_groups["metadata"]
    generation_warnings = warning_groups["generation"]
    preview_warnings = warning_groups["preview"]
    runtime_warnings = warning_groups["runtime_smoke"]
    launch_warnings = warning_groups["launch_simulation"]

    artifact_reasons: list[str] = []
    artifact_reasons.extend([b for b in blockers if "required file" in b or "YAML parse" in b or "PyYAML" in b or "urdf/scene" in b])
    artifact_reasons.extend(_with_prefix("metadata warning", metadata_warnings))
    artifact_reasons.extend(_with_prefix("generation warning", generation_warnings))
    artifact_status = FAIL if any("required file" in b or "YAML parse" in b or "PyYAML" in b or "urdf/scene" in b for b in blockers) else (WARN if artifact_reasons else PASS)

    preview_reasons: list[str] = []
    preview_reasons.extend([b for b in blockers if "mesh index" in b or "visual mesh" in b])
    preview_reasons.extend(_with_prefix("preview warning", preview_warnings))
    preview_reasons.extend(_with_prefix("metadata warning", metadata_warnings))
    preview_status = FAIL if any("mesh index" in b or "visual mesh" in b for b in blockers) else (WARN if preview_reasons or preview_readiness == "degraded" else PASS)

    moveit_reasons: list[str] = []
    launch_report_status, launch_report_reasons = _load_launch_simulation_report(scene_dir)
    simulation_entry = (simulation_results_by_scene or {}).get(scene_dir.name)

    if not smoke_available:
        moveit_reasons.append("launch/demo.launch.py missing; fake-hardware smoke command unavailable")

    if simulation_report_error:
        moveit_reasons.append(simulation_report_error)
    elif simulation_entry is None:
        moveit_reasons.append("no simulation launch evidence")
    else:
        sim_status = str(simulation_entry.get("status") or "").upper()
        sim_blockers = [b for b in (simulation_entry.get("blockers") or []) if isinstance(b, str) and b.strip()]
        sim_warnings = [w for w in (simulation_entry.get("warnings") or []) if isinstance(w, str) and w.strip()]
        if sim_status == PASS:
            moveit_reasons.append("simulation launch evidence: PASS")
        elif sim_status == FAIL:
            moveit_reasons.append("simulation launch evidence: FAIL")
            moveit_reasons.extend(_with_prefix("simulation blocker", sim_blockers))
        elif sim_status == WARN:
            moveit_reasons.append("simulation launch evidence: WARN")
            moveit_reasons.extend(_with_prefix("simulation warning", sim_warnings))
            moveit_reasons.extend(_with_prefix("simulation blocker", sim_blockers))
        elif sim_status == SKIP:
            moveit_reasons.append("simulation launch evidence: SKIP")
            moveit_reasons.extend(_with_prefix("simulation warning", sim_warnings))
        elif sim_status == BLOCKED:
            moveit_reasons.append("simulation launch evidence: BLOCKED")
            moveit_reasons.extend(_with_prefix("simulation blocker", sim_blockers))
        else:
            moveit_reasons.append(f"simulation launch report returned unknown status: {sim_status or '(missing)'}")

    moveit_reasons.extend(launch_report_reasons)
    moveit_reasons.extend(_with_prefix("launch warning", launch_warnings))
    if not smoke_available:
        moveit_status = FAIL
    elif simulation_report_error or simulation_entry is None:
        if launch_report_status in {PASS, WARN, FAIL, SKIP}:
            moveit_status = launch_report_status
        else:
            moveit_status = WARN
    else:
        sim_status = str(simulation_entry.get("status") or "").upper()
        if sim_status == PASS:
            moveit_status = PASS
        elif sim_status == FAIL:
            moveit_status = FAIL
        elif sim_status == WARN:
            moveit_status = WARN
        elif sim_status == SKIP:
            moveit_status = SKIP
        elif sim_status == BLOCKED:
            moveit_status = BLOCKED
        elif launch_report_status in {PASS, WARN, FAIL, SKIP}:
            moveit_status = launch_report_status
        else:
            moveit_status = WARN if moveit_reasons else PASS

    grasp_planner_reasons: list[str] = []
    if not optional["task_recipe"]:
        grasp_planner_reasons.append("missing optional planner input: config/task_recipe.yaml")
    if not optional["task_intent"]:
        grasp_planner_reasons.append("missing optional planner input: config/workcell_builder_task_intent.yaml")
    grasp_planner_reasons.extend(_with_prefix("runtime warning", runtime_warnings))
    grasp_planner_status = WARN if grasp_planner_reasons else PASS

    grasp_exec_reasons: list[str] = [
        "no explicit grasp execution validation evidence collected by this audit"
    ]
    if not smoke_available:
        grasp_exec_reasons.append("fake-hardware smoke launch command unavailable")
    grasp_execution_status = WARN

    real_hw_reasons: list[str] = [
        "real hardware must remain WARN until explicit hardware validation evidence is provided"
    ]
    real_hardware_status = WARN

    readiness = {
        "artifact_readiness": _dim(artifact_status, artifact_reasons),
        "preview_readiness": _dim(preview_status, preview_reasons),
        "moveit_launch_readiness": _dim(moveit_status, moveit_reasons),
        "grasp_planner_readiness": _dim(grasp_planner_status, grasp_planner_reasons),
        "grasp_execution_readiness": _dim(grasp_execution_status, grasp_exec_reasons),
        "real_hardware_readiness": _dim(real_hardware_status, real_hw_reasons),
    }

    status = PASS
    if blockers:
        status = FAIL
    elif warnings:
        status = WARN

    return SceneAudit(
        scene_name=scene_dir.name,
        support_level=support_level,
        catalog_status=catalog_status,
        known_blocker=known_blocker,
        status=status,
        files=files,
        optional_files=optional,
        generated_mesh_index_present=(scene_dir / "generated" / "scene_visual_mesh_index.json").exists(),
        mesh_index_regeneration_status=regen_status,
        mesh_index_renderable_items=renderable_items,
        preview_readiness_status=preview_readiness,
        generated_artifacts_present=generated_artifacts_present,
        fake_hardware_smoke_command_available=smoke_available,
        fake_hardware_smoke_command=fake_cmd,
        blockers=blockers,
        warnings=warnings,
        warning_groups=warning_groups,
        readiness=readiness,
    )


def _empty_readiness(status: str, reason: str) -> dict[str, ReadinessDimension]:
    return {dim: _dim(status, [reason]) for dim in READINESS_DIMENSIONS}


def _catalog_only_audit(
    entry: SupportedSceneEntry,
    scene_dir: Path,
    status: str,
    reason: str,
) -> SceneAudit:
    warning_groups: dict[str, list[str]] = {k: [] for k in WARNING_CATEGORIES}
    blockers: list[str] = []
    warnings: list[str] = []
    if status == BLOCKED:
        blockers.append(reason)
    else:
        warning_groups["generation"].append(reason)
        warnings.append(reason)

    return SceneAudit(
        scene_name=entry.scene_name,
        support_level=entry.support_level,
        catalog_status=entry.status,
        known_blocker=entry.known_blocker,
        status=status,
        files={k: False for k in REQUIRED_FILES},
        optional_files={k: False for k in OPTIONAL_FILES},
        generated_mesh_index_present=False,
        mesh_index_regeneration_status="not_attempted_catalog_only",
        mesh_index_renderable_items=0,
        preview_readiness_status="blocked" if status == BLOCKED else "skipped",
        generated_artifacts_present={
            "generated_environment_assets": False,
            "generated_layout": False,
            "task_recipe": False,
            "task_intent": False,
        },
        fake_hardware_smoke_command_available=False,
        fake_hardware_smoke_command=entry.fake_hardware_launch_command,
        blockers=blockers,
        warnings=warnings,
        warning_groups=warning_groups,
        readiness=_empty_readiness(status, reason),
    )


def audit_catalog_entry(
    repo_root: Path,
    entry: SupportedSceneEntry,
    *,
    include_experimental: bool,
    regenerate_missing_mesh_indexes: bool,
    simulation_results_by_scene: dict[str, dict[str, Any]] | None = None,
    simulation_report_error: str | None = None,
) -> SceneAudit:
    scene_dir = (repo_root / entry.scene_path).resolve()
    support_level = entry.support_level.strip().lower()
    catalog_status = entry.status.strip().lower()

    if catalog_status == "blocked":
        reason = entry.known_blocker or "catalog marks scene as blocked without a known_blocker"
        return _catalog_only_audit(entry, scene_dir, BLOCKED, reason)

    if support_level == "experimental" and not include_experimental:
        reason = "experimental scene skipped by default; rerun with --include-experimental to audit it"
        return _catalog_only_audit(entry, scene_dir, SKIP, reason)

    audit = audit_scene(
        repo_root,
        scene_dir,
        simulation_results_by_scene=simulation_results_by_scene,
        simulation_report_error=simulation_report_error,
        support_level=entry.support_level,
        catalog_status=entry.status,
        known_blocker=entry.known_blocker,
        regenerate_missing_mesh_indexes=regenerate_missing_mesh_indexes,
    )
    audit.scene_name = entry.scene_name
    audit.fake_hardware_smoke_command = entry.fake_hardware_launch_command
    return audit


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    scenes_root = resolve_scenes_root(repo_root)
    catalog_path = (args.catalog or default_catalog_path(repo_root)).resolve()
    catalog, catalog_entries, catalog_errors = load_supported_scene_catalog(catalog_path)
    if catalog_errors:
        print("Supported scene catalog is invalid:")
        for error in catalog_errors:
            print(f"- {error}")
        return 2
    enabled_entries = sorted((entry for entry in catalog_entries if entry.enabled), key=lambda e: e.scene_name)

    simulation_results_by_scene, simulation_report_error = _load_simulation_launch_report(repo_root, args.simulation_launch_report)

    audits = [
        audit_catalog_entry(
            repo_root,
            entry,
            include_experimental=args.include_experimental,
            regenerate_missing_mesh_indexes=args.regenerate_missing_mesh_indexes,
            simulation_results_by_scene=simulation_results_by_scene,
            simulation_report_error=simulation_report_error,
        )
        for entry in enabled_entries
    ]

    counts = {PASS: 0, WARN: 0, FAIL: 0, SKIP: 0, BLOCKED: 0}
    readiness_counts = {
        dim: {PASS: 0, WARN: 0, FAIL: 0, SKIP: 0, BLOCKED: 0}
        for dim in READINESS_DIMENSIONS
    }
    for audit in audits:
        counts[audit.status] += 1
        for dim in READINESS_DIMENSIONS:
            status = audit.readiness[dim].status
            readiness_counts[dim][status] += 1

    report = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "scenes_root": str(scenes_root),
        "supported_scene_catalog": str(catalog_path),
        "supported_scene_catalog_schema": catalog.get("schema_version"),
        "scene_count": len(audits),
        "status_counts": counts,
        "readiness_dimension_counts": readiness_counts,
        "scenes": [asdict(a) for a in audits],
    }

    output_dir = repo_root / "build" / "workcell_studio"
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "all_scene_reproducibility_report.json"
    output_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print("Scene | Support | Catalog | Status | Required | Renderable | Preview | Blockers")
    print("-|-|-|-|-|-|-|-")
    for a in audits:
        req_ok = sum(1 for v in a.files.values() if v)
        blocker_text = a.blockers[0] if a.blockers else "-"
        print(f"{a.scene_name} | {a.support_level} | {a.catalog_status} | {a.status} | {req_ok}/{len(a.files)} | {a.mesh_index_renderable_items} | {a.preview_readiness_status} | {blocker_text}")

    print(f"\nSummary PASS={counts[PASS]} WARN={counts[WARN]} FAIL={counts[FAIL]} SKIP={counts[SKIP]} BLOCKED={counts[BLOCKED]}")
    print(f"JSON report: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
