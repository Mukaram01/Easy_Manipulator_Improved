#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]


@dataclass
class SceneRecord:
    package_name: str
    source_path: str
    installed: bool
    generated: bool
    warnings: list[str] = field(default_factory=list)


@dataclass
class RecipeRecord:
    display_name: str
    path: str
    task_type: str | None
    version: str | None
    category: str
    warnings: list[str] = field(default_factory=list)


@dataclass
class DetectedObjectsRecord:
    display_name: str
    path: str
    object_count: int | None
    version: str | None
    category: str
    warnings: list[str] = field(default_factory=list)


@dataclass
class CellDefinitionRecord:
    display_name: str
    path: str
    schema_version: str | None
    warnings: list[str] = field(default_factory=list)


def _iter_scene_dirs() -> list[Path]:
    roots = [REPO_ROOT / "install" / "share", REPO_ROOT / "scenes", REPO_ROOT / "src"]
    extra_patterns = [
        "generated_workcell/*",
        "generated_workcell/*/*",
        "**/generated_workcell/*",
    ]
    candidates: list[Path] = []
    for root in roots:
        if root.exists():
            if root.name == "src":
                candidates.extend([p for p in root.glob("**/scenes/*") if p.is_dir()])
            else:
                candidates.extend([p for p in root.glob("*") if p.is_dir()])
    for pat in extra_patterns:
        candidates.extend([p for p in REPO_ROOT.glob(pat) if p.is_dir()])
    uniq = {p.resolve(): p for p in candidates}
    return list(uniq.values())


def discover_scene_packages() -> list[SceneRecord]:
    records: dict[str, SceneRecord] = {}
    for path in _iter_scene_dirs():
        package_xml = path / "package.xml"
        if not package_xml.exists() and not any((path / d).exists() for d in ("launch", "config", "urdf", "moveit_config")):
            continue
        package_name = path.name
        warnings: list[str] = []
        has_emd_layout = (path / "launch").exists() and (path / "environment.yaml").exists()
        expected_dirs = ("package.xml", "launch") if has_emd_layout else ("package.xml", "launch", "config")
        for requiredish in expected_dirs:
            if not (path / requiredish).exists():
                warnings.append(f"missing {requiredish}")
        installed = "/install/" in str(path.resolve()) or str(path).startswith(str(REPO_ROOT / "install"))
        generated = "generated_workcell" in str(path) or "generated" in str(path)
        key = package_name
        existing = records.get(key)
        rec = SceneRecord(package_name, str(path), installed, generated, warnings)
        if existing is None or (rec.installed and not existing.installed):
            records[key] = rec
    return sorted(records.values(), key=lambda r: r.package_name)


def _load_structured_file(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    try:
        if path.suffix.lower() == ".json":
            loaded = json.loads(path.read_text(encoding="utf-8"))
            return loaded if isinstance(loaded, dict) else None, None
        import yaml  # type: ignore

        loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
        return loaded if isinstance(loaded, dict) else None, None
    except Exception as exc:  # noqa: BLE001
        return None, str(exc)




def _fixture_category(name: str) -> str:
    lowered = name.lower()
    if lowered.startswith(("fail_", "missing_", "low_confidence_", "unknown_")):
        return "failure_test"
    if lowered.startswith("warn_"):
        return "warning"
    return "valid"

def discover_task_recipes() -> list[RecipeRecord]:
    dirs = [
        REPO_ROOT / "tests/fixtures/task_recipes",
        REPO_ROOT / "config/task_recipes",
        REPO_ROOT / "docs/examples",
    ]
    dirs.extend([REPO_ROOT / p for p in ["generated_workcell"]])
    records: list[RecipeRecord] = []
    for base in dirs:
        if not base.exists():
            continue
        for path in [*base.rglob("*.yaml"), *base.rglob("*.yml"), *base.rglob("*.json")]:
            data, err = _load_structured_file(path)
            warnings: list[str] = []
            if err:
                records.append(RecipeRecord(path.stem, str(path), None, None, _fixture_category(path.stem), [f"parse error: {err}"]))
                continue
            if not data:
                continue
            version = data.get("schema_version")
            if version != "task_recipe/v1":
                continue
            task = data.get("task") if isinstance(data.get("task"), dict) else {}
            records.append(RecipeRecord(path.stem, str(path), task.get("type"), version, _fixture_category(path.stem), warnings))
    return sorted(records, key=lambda r: r.display_name)


def discover_detected_objects() -> list[DetectedObjectsRecord]:
    dirs = [REPO_ROOT / "tests/fixtures/detected_objects", REPO_ROOT / "config/detected_objects"]
    records: list[DetectedObjectsRecord] = []
    for base in dirs:
        if not base.exists():
            continue
        for path in [*base.rglob("*.yaml"), *base.rglob("*.yml"), *base.rglob("*.json")]:
            data, err = _load_structured_file(path)
            if err:
                records.append(DetectedObjectsRecord(path.stem, str(path), None, None, _fixture_category(path.stem), [f"parse error: {err}"]))
                continue
            if not data:
                continue
            version = data.get("schema_version")
            if version != "detected_objects/v1":
                continue
            objects = data.get("objects") if isinstance(data.get("objects"), list) else None
            records.append(DetectedObjectsRecord(path.stem, str(path), len(objects) if objects is not None else None, version, _fixture_category(path.stem), []))
    return sorted(records, key=lambda r: r.display_name)


def discover_all() -> dict[str, list[dict[str, Any]]]:
    return {
        "scenes": [asdict(r) for r in discover_scene_packages()],
        "task_recipes": [asdict(r) for r in discover_task_recipes()],
        "detected_objects": [asdict(r) for r in discover_detected_objects()],
        "cell_definitions": [asdict(r) for r in discover_cell_definitions()],
        "generated_workcell_summaries": discover_generated_workcell_summaries(),
    }


def discover_cell_definitions() -> list[CellDefinitionRecord]:
    dirs = [REPO_ROOT / "cell_definitions", REPO_ROOT / "docs/examples"]
    records: list[CellDefinitionRecord] = []
    for base in dirs:
        if not base.exists():
            continue
        for path in [*base.rglob("*.yaml"), *base.rglob("*.yml")]:
            data, err = _load_structured_file(path)
            if err:
                records.append(CellDefinitionRecord(path.stem, str(path), None, [f"parse error: {err}"]))
                continue
            if not data:
                continue
            schema = data.get("schema_version")
            if schema == "cell_definition/v1":
                records.append(CellDefinitionRecord(path.stem, str(path), schema, []))
    return sorted(records, key=lambda r: r.display_name)


def discover_generated_workcell_summaries() -> list[dict[str, Any]]:
    roots = [REPO_ROOT / "generated_workcell", REPO_ROOT / "tests/fixtures/generated_workcell"]
    results: list[dict[str, Any]] = []
    for root in roots:
        if not root.exists():
            continue
        for path in [*root.rglob("summary.json"), *root.rglob("generated_workcell_summary.json")]:
            data, err = _load_structured_file(path)
            if err or not data:
                continue
            results.append(
                {
                    "path": str(path),
                    "cell_id": data.get("cell_id"),
                    "package_name": data.get("package_name"),
                    "task_recipe_path": data.get("task_recipe_path"),
                    "detected_objects_example_path": data.get("detected_objects_example_path"),
                    "warnings": data.get("warnings", []),
                    "blockers": data.get("blockers", []),
                    "approval_status": ((data.get("approval") or {}).get("status") or "unapproved"),
                    "status": "FAIL" if data.get("blockers") else ("WARN" if data.get("warnings") else "PASS"),
                }
            )
    return sorted(results, key=lambda x: x["path"])


def _print_summary(data: dict[str, list[dict[str, Any]]]) -> None:
    scene_warnings = sum(len(x.get("warnings", [])) for x in data["scenes"])
    recipe_warnings = sum(len(x.get("warnings", [])) for x in data["task_recipes"])
    obj_warnings = sum(len(x.get("warnings", [])) for x in data["detected_objects"])
    print(f"Scenes found: {len(data['scenes'])}")
    print(f"Task recipes found: {len(data['task_recipes'])}")
    print(f"Detected object fixtures found: {len(data['detected_objects'])}")
    print(f"Cell definition templates found: {len(data.get('cell_definitions', []))}")
    print(f"Generated workcell summaries found: {len(data.get('generated_workcell_summaries', []))}")
    print(f"Warnings: {scene_warnings + recipe_warnings + obj_warnings}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Discover workcell scenes/recipes/fixtures")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--summary", action="store_true")
    args = parser.parse_args(argv)
    payload = discover_all()
    if args.summary:
        _print_summary(payload)
    else:
        print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
