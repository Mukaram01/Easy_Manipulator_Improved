#!/usr/bin/env python3
from __future__ import annotations

import json
import subprocess
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

WARNING_CATEGORIES = ("metadata", "preview", "generation", "launch_simulation", "runtime_smoke")

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


def audit_scene(repo_root: Path, scene_dir: Path) -> SceneAudit:
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
    }

    fake_cmd = f"python3 scripts/run_fake_hardware_smoke_launch.py --scene {scene_dir.name}"
    smoke_available = _fake_hardware_command_available(scene_dir)

    for key, present in optional.items():
        if not present and key not in {"scene_urdf", "scene_urdf_xacro"}:
            if key in {"task_recipe", "task_intent"}:
                warning_groups["runtime_smoke"].append(f"optional file missing: {OPTIONAL_FILES[key]}")
            elif key in {"generated_environment_assets", "generated_layout"}:
                warning_groups["generation"].append(f"optional file missing: {OPTIONAL_FILES[key]}")
            else:
                warning_groups["metadata"].append(f"optional file missing: {OPTIONAL_FILES[key]}")

    preview_readiness = "ready" if not blockers and not layout_issues else ("degraded" if not blockers else "blocked")

    if scene_dir.name not in KNOWN_SCENES:
        warning_groups["metadata"].append("scene is not in known-scenes audit set")

    warnings = [w for group in WARNING_CATEGORIES for w in warning_groups[group] if w]

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
    )


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    scenes_root = resolve_scenes_root(repo_root)
    scene_dirs = sorted([p for p in scenes_root.iterdir() if p.is_dir()])

    audits = [audit_scene(repo_root, scene_dir) for scene_dir in scene_dirs]

    counts = {PASS: 0, WARN: 0, FAIL: 0, SKIP: 0}
    for audit in audits:
        counts[audit.status] += 1

    report = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "scenes_root": str(scenes_root),
        "scene_count": len(audits),
        "status_counts": counts,
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
