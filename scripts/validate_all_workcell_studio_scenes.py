#!/usr/bin/env python3
from __future__ import annotations

import json
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any

import yaml

REQUIRED_FILES = {
    "scene_manifest": "scene_manifest.yaml",
    "environment": "environment.yaml",
    "cell_definition": "cell_definition.yaml",
    "layout": "layout/workcell_studio_layout.yaml",
    "demo_launch": "launch/demo.launch.py",
    "scene_urdf_xacro": "urdf/scene.urdf.xacro",
}

OPTIONAL_ARTIFACTS = [
    "config/task_recipe.yaml",
    "config/workcell_builder_task_intent.yaml",
    "generated/environment_assets.yaml",
    "layout/workcell_studio_layout.generated.yaml",
]

@dataclass
class SceneAudit:
    scene_name: str
    scene_path: str
    status: str
    required_files: dict[str, bool]
    optional_artifacts: dict[str, bool]
    yaml_parse: dict[str, str]
    layout_previewable: bool
    missing_layout_metadata: list[str]
    messages: list[str]


def resolve_scenes_root(repo_root: Path) -> Path:
    candidates = [
        repo_root / "easy_manipulation_deployment" / "scenes",
        repo_root / "scenes",
    ]
    for candidate in candidates:
        if candidate.exists() and candidate.is_dir():
            return candidate
    raise FileNotFoundError("Could not find scenes directory. Expected easy_manipulation_deployment/scenes or scenes/")


def _safe_yaml_load(path: Path) -> tuple[Any | None, str]:
    if not path.exists():
        return None, "missing"
    try:
        with path.open("r", encoding="utf-8") as handle:
            return yaml.safe_load(handle), "ok"
    except yaml.YAMLError as exc:
        return None, f"parse_error: {exc.__class__.__name__}: {exc}"
    except Exception as exc:  # noqa: BLE001
        return None, f"error: {exc.__class__.__name__}: {exc}"


def _layout_metadata_issues(layout_data: Any) -> list[str]:
    issues: list[str] = []
    if not isinstance(layout_data, dict):
        return ["layout root must be a YAML map"]
    if layout_data.get("schema_version") != "workcell_studio_layout/v1":
        issues.append("schema_version must be 'workcell_studio_layout/v1'")
    items = layout_data.get("items")
    if items is None:
        issues.append("missing items sequence")
        return issues
    if not isinstance(items, list):
        issues.append("items must be a sequence")
        return issues
    for idx, item in enumerate(items):
        prefix = f"items[{idx}]"
        if not isinstance(item, dict):
            issues.append(f"{prefix} must be a map")
            continue
        if not item.get("id"):
            issues.append(f"{prefix}.id missing")
        pose = item.get("pose")
        if pose is None:
            issues.append(f"{prefix}.pose missing")
            continue
        if not isinstance(pose, dict):
            issues.append(f"{prefix}.pose must be a map")
            continue
        for key in ("xyz", "rpy"):
            value = pose.get(key)
            if not isinstance(value, list) or len(value) < 3:
                issues.append(f"{prefix}.pose.{key} must be sequence length >= 3")
    return issues


def audit_scene(scene_dir: Path) -> SceneAudit:
    required = {name: (scene_dir / rel).exists() for name, rel in REQUIRED_FILES.items()}
    optional = {rel: (scene_dir / rel).exists() for rel in OPTIONAL_ARTIFACTS}

    yaml_targets = {
        "scene_manifest.yaml": scene_dir / "scene_manifest.yaml",
        "environment.yaml": scene_dir / "environment.yaml",
        "cell_definition.yaml": scene_dir / "cell_definition.yaml",
        "layout/workcell_studio_layout.yaml": scene_dir / "layout" / "workcell_studio_layout.yaml",
    }
    yaml_parse: dict[str, str] = {}
    parsed_layout = None
    for key, path in yaml_targets.items():
        loaded, status = _safe_yaml_load(path)
        yaml_parse[key] = status
        if key.endswith("workcell_studio_layout.yaml"):
            parsed_layout = loaded if status == "ok" else None

    missing_layout_metadata = _layout_metadata_issues(parsed_layout) if parsed_layout is not None else ["layout not parseable"]
    layout_previewable = len(missing_layout_metadata) == 0

    messages: list[str] = []
    for name, ok in required.items():
        if not ok:
            messages.append(f"missing required file: {REQUIRED_FILES[name]}")
    for yaml_name, status in yaml_parse.items():
        if status not in {"ok", "missing"}:
            messages.append(f"YAML parse failure in {yaml_name}: {status}")
        elif status == "missing":
            messages.append(f"missing YAML file: {yaml_name}")
    for issue in missing_layout_metadata:
        messages.append(f"layout metadata issue: {issue}")

    blocked = any(not present for present in required.values()) or any(v.startswith("parse_error") or v.startswith("error") for v in yaml_parse.values())
    warning = (not blocked) and (not layout_previewable or not all(optional.values()))
    status = "blocked" if blocked else ("warning" if warning else "ready")

    return SceneAudit(
        scene_name=scene_dir.name,
        scene_path=str(scene_dir),
        status=status,
        required_files=required,
        optional_artifacts=optional,
        yaml_parse=yaml_parse,
        layout_previewable=layout_previewable,
        missing_layout_metadata=missing_layout_metadata,
        messages=messages,
    )


def print_table(rows: list[SceneAudit]) -> None:
    headers = ["Scene", "Status", "Req OK", "YAML OK", "Layout", "Notes"]
    print(" | ".join(headers))
    print("-|-|-|-|-|-")
    for row in rows:
        req_ok = sum(1 for v in row.required_files.values() if v)
        yaml_ok = sum(1 for v in row.yaml_parse.values() if v == "ok")
        notes = "; ".join(row.messages[:2]) if row.messages else "-"
        print(f"{row.scene_name} | {row.status} | {req_ok}/{len(row.required_files)} | {yaml_ok}/{len(row.yaml_parse)} | {'ok' if row.layout_previewable else 'issues'} | {notes}")


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    scenes_root = resolve_scenes_root(repo_root)
    scene_dirs = sorted([p for p in scenes_root.iterdir() if p.is_dir()])
    audits = [audit_scene(scene_dir) for scene_dir in scene_dirs]

    counts = {"ready": 0, "warning": 0, "blocked": 0}
    for audit in audits:
        counts[audit.status] += 1

    report = {
        "scenes_root": str(scenes_root),
        "scene_count": len(audits),
        "status_counts": counts,
        "scenes": [asdict(audit) for audit in audits],
    }

    build_dir = repo_root / "build"
    build_dir.mkdir(exist_ok=True)
    report_path = build_dir / "workcell_studio_scene_reproducibility_report.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print_table(audits)
    print(f"\nScanned {len(audits)} scenes: ready={counts['ready']}, warning={counts['warning']}, blocked={counts['blocked']}")
    print(f"JSON report: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
