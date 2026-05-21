#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

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
VALID_STATUSES = {PASS, WARN, FAIL, SKIP}

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



def _normalize_status(value: Any) -> str | None:
    if isinstance(value, str):
        candidate = value.strip().upper()
        if candidate in VALID_STATUSES:
            return candidate
    return None


def _load_simulation_launch_report(path: Path) -> tuple[dict[str, dict[str, Any]], str | None]:
    if not path.exists():
        return {}, f"simulation launch report missing: {path}"
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001
        return {}, f"simulation launch report unreadable: {exc.__class__.__name__}: {exc}"

    scene_map: dict[str, dict[str, Any]] = {}
    scenes = payload.get("scenes") if isinstance(payload, dict) else None
    if isinstance(scenes, list):
        for row in scenes:
            if not isinstance(row, dict):
                continue
            name = row.get("scene_name")
            if isinstance(name, str) and name.strip():
                scene_map[name.strip()] = row

    if not scene_map:
        per_scene = payload.get("scene_results") if isinstance(payload, dict) else None
        if isinstance(per_scene, dict):
            for name, row in per_scene.items():
                if isinstance(name, str) and isinstance(row, dict):
                    merged = dict(row)
                    merged.setdefault("scene_name", name)
                    scene_map[name] = merged

    return scene_map, None


def _simulation_launch_evidence_for_scene(scene_name: str, row: dict[str, Any] | None) -> tuple[str | None, list[str], list[str]]:
    if not row:
        return None, ["no simulation launch evidence"], []

    status = _normalize_status(row.get("status"))
    warnings = [str(w) for w in (row.get("warnings") or []) if isinstance(w, str)]
    blockers = [str(b) for b in (row.get("blockers") or []) if isinstance(b, str)]

    if status == PASS:
        reasons = [f"simulation launch evidence PASS for {scene_name}"]
        reasons.extend(_with_prefix("simulation warning", warnings))
        return PASS, reasons, []

    if status == FAIL:
        reasons = [f"simulation launch evidence FAIL for {scene_name}"]
        if blockers:
            reasons.extend(_with_prefix("simulation blocker", blockers))
        else:
            reasons.append("simulation launch report marked FAIL without blockers")
        reasons.extend(_with_prefix("simulation warning", warnings))
        return FAIL, reasons, []

    if status == WARN:
        reasons = [f"simulation launch evidence WARN for {scene_name}"]
        reasons.extend(_with_prefix("simulation warning", warnings))
        reasons.extend(_with_prefix("simulation blocker", blockers))
        return WARN, reasons, []

    if status == SKIP:
        reasons = [f"simulation launch evidence SKIP for {scene_name}"]
        reasons.extend(_with_prefix("simulation warning", warnings))
        reasons.extend(_with_prefix("simulation blocker", blockers))
        return SKIP, reasons, []

    return None, ["simulation launch evidence status missing or invalid"], []

KNOWN_SCENES = {
    "suction_test",
    "ur5_2f_test",
    "ur5_3f_test",
    "ur5_2f_builder_pick_place_demo",
    "ur5_2f_sorting_test",
    "ur3_suction_test",
    "ur10_2f_test",
    "ur5_airpick4_test",
}


@dataclass
class ReadinessDimension:
    status: str
    reasons: list[str]


@dataclass
class SceneAudit:
    scene_name: str
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
    try:
        import yaml  # type: ignore
    except ModuleNotFoundError:
        return None, "dependency_missing: pyyaml"

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


def audit_scene(repo_root: Path, scene_dir: Path, simulation_scene_report: dict[str, Any] | None = None) -> SceneAudit:
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
        if status == "dependency_missing: pyyaml":
            blockers.append("PyYAML dependency missing. Install with: sudo apt install python3-yaml")
            break
        if status.startswith("parse_error"):
            blockers.append(f"YAML parse failure in {name}: {status}")

    layout_issues = _layout_metadata_issues(layout_data) if layout_status == "ok" else ["layout not parseable"]
    if layout_issues:
        warning_groups["metadata"].extend([f"layout metadata issue: {issue}" for issue in layout_issues])
        warning_groups["preview"].append("preview readiness degraded by layout metadata issues")

    generated_index = (scene_dir / "generated" / "scene_visual_mesh_index.json").exists()
    regen_status = "not_needed" if generated_index else "attempted"
    regen_reason = ""
    if not generated_index:
        regen_status, regen_reason = _run_extract_for_scene(repo_root, scene_dir.name)
        if regen_status != "ok":
            blockers.append(f"visual mesh index regeneration failed: {regen_reason}")

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

    if scene_dir.name not in KNOWN_SCENES:
        warning_groups["metadata"].append("scene is not in known-scenes audit set")

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
    if not smoke_available:
        moveit_reasons.append("launch/demo.launch.py missing; fake-hardware smoke command unavailable")
    moveit_reasons.extend(_with_prefix("launch warning", launch_warnings))

    sim_status, sim_reasons, sim_launch_warnings = _simulation_launch_evidence_for_scene(scene_dir.name, simulation_scene_report)
    moveit_reasons.extend(sim_reasons)
    warning_groups["launch_simulation"].extend(sim_launch_warnings)

    if not smoke_available:
        moveit_status = FAIL
    elif sim_status == FAIL:
        moveit_status = FAIL
    elif sim_status == SKIP:
        moveit_status = SKIP
    elif sim_status == PASS:
        moveit_status = PASS
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


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate all Workcell Studio scenes and emit readiness report")
    parser.add_argument(
        "--simulation-launch-report",
        type=Path,
        default=Path("build/workcell_studio/rviz_moveit_simulation_launch_report.json"),
        help="Optional simulation launch report JSON used for moveit_launch_readiness evidence",
    )
    args = parser.parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    scenes_root = resolve_scenes_root(repo_root)
    scene_dirs = sorted([p for p in scenes_root.iterdir() if p.is_dir()])

    simulation_scene_map, simulation_report_error = _load_simulation_launch_report(repo_root / args.simulation_launch_report)

    audits = [
        audit_scene(repo_root, scene_dir, simulation_scene_map.get(scene_dir.name))
        for scene_dir in scene_dirs
    ]

    counts = {PASS: 0, WARN: 0, FAIL: 0, SKIP: 0}
    readiness_counts = {
        dim: {PASS: 0, WARN: 0, FAIL: 0, SKIP: 0}
        for dim in READINESS_DIMENSIONS
    }
    for audit in audits:
        counts[audit.status] += 1
        for dim in READINESS_DIMENSIONS:
            status = audit.readiness[dim].status
            readiness_counts[dim][status] += 1

    report = {
        "simulation_launch_report_path": str(repo_root / args.simulation_launch_report),
        "simulation_launch_report_loaded": simulation_report_error is None,
        "simulation_launch_report_error": simulation_report_error,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "scenes_root": str(scenes_root),
        "scene_count": len(audits),
        "status_counts": counts,
        "readiness_dimension_counts": readiness_counts,
        "scenes": [asdict(a) for a in audits],
    }

    output_dir = repo_root / "build" / "workcell_studio"
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "all_scene_reproducibility_report.json"
    output_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print("Scene | Status | Required | Renderable | Preview | Blockers")
    print("-|-|-|-|-|-")
    for a in audits:
        req_ok = sum(1 for v in a.files.values() if v)
        blocker_text = a.blockers[0] if a.blockers else "-"
        print(f"{a.scene_name} | {a.status} | {req_ok}/{len(a.files)} | {a.mesh_index_renderable_items} | {a.preview_readiness_status} | {blocker_text}")

    print(f"\nSummary PASS={counts[PASS]} WARN={counts[WARN]} FAIL={counts[FAIL]} SKIP={counts[SKIP]}")
    print(f"JSON report: {output_path}")
    return 1 if counts[FAIL] > 0 else 0


if __name__ == "__main__":
    raise SystemExit(main())
