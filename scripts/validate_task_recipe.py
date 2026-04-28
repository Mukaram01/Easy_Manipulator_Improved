#!/usr/bin/env python3
"""Validate offline task_recipe/v1 YAML files."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

try:
    import yaml as _pyyaml
except Exception:  # pragma: no cover
    _pyyaml = None

SUPPORTED_TASK_TYPES = {
    "pick_place",
    "sort_by_colour",
    "sort_by_shape",
    "sort_by_class",
    "garbage_sorting",
    "inspection_then_place",
    "custom",
}
SORTING_TYPES = {"sort_by_colour", "sort_by_shape", "sort_by_class", "garbage_sorting"}


@dataclass
class RecipeSummary:
    path: Path
    parser: str
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors


def _as_task_doc(raw: dict[str, Any]) -> dict[str, Any]:
    if isinstance(raw.get("task"), dict):
        return raw
    if raw.get("schema_version") == "task_recipe/v1":
        return {"schema_version": raw.get("schema_version"), "task": {k: v for k, v in raw.items() if k != "schema_version"}}
    return raw


def _is_num_list(value: Any, n: int | None = None) -> bool:
    return isinstance(value, list) and (n is None or len(value) == n) and all(isinstance(v, (int, float)) for v in value)


def _is_reject_like(destination: dict[str, Any]) -> bool:
    text = " ".join(
        str(destination.get(key, ""))
        for key in ("id", "bin", "type", "label")
    ).lower()
    return "reject" in text or "unknown" in text


def validate_task_recipe_doc(data: dict[str, Any], path: Path, parser: str, notes: list[str], strict: bool = False) -> RecipeSummary:
    summary = RecipeSummary(path=path, parser=parser, notes=list(notes))
    doc = _as_task_doc(data)

    if doc.get("schema_version") != "task_recipe/v1":
        summary.errors.append("schema_version must be exactly 'task_recipe/v1'.")

    task = doc.get("task") if isinstance(doc.get("task"), dict) else {}
    task_id = task.get("id")
    if not isinstance(task_id, str) or not task_id.strip():
        summary.errors.append("task.id must be a non-empty string.")

    task_type = task.get("type")
    if not isinstance(task_type, str) or not task_type.strip():
        summary.errors.append("task.type must be a non-empty string.")
        task_type = ""
    elif task_type not in SUPPORTED_TASK_TYPES:
        summary.errors.append(f"task.type '{task_type}' is unsupported for task_recipe/v1.")

    if not any(isinstance(task.get(key), str) and str(task.get(key)).strip() for key in ("source", "source_object", "perception_source")):
        summary.errors.append("task must declare a source via task.source, task.source_object, or task.perception_source.")

    destinations = task.get("destinations")
    if not isinstance(destinations, list) or not destinations:
        summary.errors.append("task.destinations must be a non-empty list.")
        destinations = []

    destination_ids: set[str] = set()
    duplicate_ids: set[str] = set()
    all_zero = True
    reject_like_present = False

    for idx, dest in enumerate(destinations):
        if not isinstance(dest, dict):
            summary.errors.append(f"task.destinations[{idx}] must be a mapping.")
            continue
        dest_id = dest.get("id")
        if not isinstance(dest_id, str) or not dest_id.strip():
            summary.errors.append(f"task.destinations[{idx}].id must be a non-empty string.")
        else:
            if dest_id in destination_ids:
                duplicate_ids.add(dest_id)
            destination_ids.add(dest_id)

        if not isinstance(dest.get("frame"), str) or not dest.get("frame", "").strip():
            summary.warnings.append(
                f"task.destinations[{idx}].frame is missing/empty; runtime adapter defaults destination frame to planning frame/world."
            )
        if not _is_num_list(dest.get("pose_xyz"), 3):
            summary.warnings.append(
                f"task.destinations[{idx}].pose_xyz is missing/malformed; runtime release will fall back to legacy release_x_offset behavior."
            )
        else:
            if any(float(v) != 0.0 for v in dest["pose_xyz"]):
                all_zero = False
        if "pose_rpy" in dest and not _is_num_list(dest.get("pose_rpy"), 3):
            summary.warnings.append(
                f"task.destinations[{idx}].pose_rpy is malformed; runtime release will fall back to legacy release behavior."
            )

        reject_like_present = reject_like_present or _is_reject_like(dest)

    for duplicate in sorted(duplicate_ids):
        summary.errors.append(f"task.destinations contains duplicate destination id '{duplicate}'.")

    rules = task.get("decision_rules")
    if not isinstance(rules, list) or not rules:
        summary.errors.append("task.decision_rules must be a non-empty list.")
        rules = []

    fallback_present = False
    for idx, rule in enumerate(rules):
        if not isinstance(rule, dict):
            summary.errors.append(f"task.decision_rules[{idx}] must be a mapping.")
            continue
        rule_dest = rule.get("destination")
        if not isinstance(rule_dest, str) or not rule_dest.strip():
            summary.errors.append(f"task.decision_rules[{idx}].destination must be a non-empty string.")
        elif rule_dest not in destination_ids:
            summary.errors.append(f"task.decision_rules[{idx}] destination '{rule_dest}' does not exist in task.destinations.")

        when = rule.get("when") if isinstance(rule.get("when"), dict) else {}
        if when.get("default") is True or when.get("always") is True or rule.get("fallback") is True:
            fallback_present = True

        has_condition = False
        if isinstance(when.get("attribute"), str):
            has_condition = True
            if "equals" not in when:
                summary.errors.append(f"task.decision_rules[{idx}].when.equals is required when when.attribute is set.")
        if "confidence_below" in when:
            has_condition = True
            threshold = when.get("confidence_below")
            if not isinstance(threshold, (int, float)) or float(threshold) < 0.0 or float(threshold) > 1.0:
                summary.errors.append(
                    f"task.decision_rules[{idx}].when.confidence_below must be numeric and between 0.0 and 1.0."
                )
        if not has_condition and not (when.get("default") is True or when.get("always") is True):
            summary.errors.append(f"task.decision_rules[{idx}].when must declare attribute/equals, confidence_below, or default.")

    if not fallback_present:
        (summary.errors if strict else summary.warnings).append("Task recipe has no explicit fallback/default decision rule.")

    if task_type in SORTING_TYPES and not reject_like_present:
        (summary.errors if strict else summary.warnings).append(
            "Sorting/classification task has no reject/unknown destination metadata."
        )

    if destinations and all_zero:
        summary.warnings.append("All task destination pose_xyz values are [0, 0, 0]; verify placement poses.")

    if task_type == "custom":
        if not any(isinstance(task.get(k), str) and task.get(k).strip() for k in ("notes", "handler_hint")):
            summary.warnings.append("Custom task recipe should include task.notes or task.handler_hint for downstream tooling.")

    return summary


def validate_path(path: Path, strict: bool = False) -> RecipeSummary:
    notes: list[str] = []
    text = path.read_text(encoding="utf-8")
    if _pyyaml is not None:
        loaded = _pyyaml.safe_load(text) or {}
        parser = "pyyaml"
    else:
        import validate_cell_definition as cell_yaml

        notes.append("PyYAML unavailable; using fallback parser with limited YAML support.")
        loaded = cell_yaml.parse_cell_definition_yaml(text)
        parser = "fallback"
    return validate_task_recipe_doc(loaded, path, parser, notes, strict=strict)


def _discover_recipe_files(root: Path) -> list[Path]:
    if root.is_file():
        return [root]
    if not root.is_dir():
        raise FileNotFoundError(f"Path not found: {root}")
    return sorted([p for p in root.rglob("*") if p.is_file() and p.suffix.lower() in {".yaml", ".yml"}])


def _result(summary: RecipeSummary) -> str:
    return "PASS" if summary.ok and not summary.warnings else "WARN" if summary.ok else "FAIL"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path, help="Task recipe YAML file or directory")
    parser.add_argument("--strict", action="store_true", help="Treat warnings as failures")
    parser.add_argument("--json", action="store_true", help="Print machine-readable output")
    parser.add_argument("--quiet", action="store_true", help="Suppress detailed output")
    args = parser.parse_args(argv)

    try:
        files = _discover_recipe_files(args.path)
    except Exception as exc:
        if args.json:
            print(json.dumps({"result": "FAIL", "error": str(exc)}, indent=2))
        else:
            print(f"FAIL: {exc}")
        return 2

    summaries: list[RecipeSummary] = []
    for file_path in files:
        try:
            summaries.append(validate_path(file_path, strict=args.strict))
        except Exception as exc:
            summaries.append(RecipeSummary(path=file_path, parser="unknown", errors=[f"Malformed YAML: {exc}"]))

    blocking_fail = False
    payload_rows = []

    for summary in summaries:
        result = _result(summary)
        strict_fail = args.strict and result == "WARN"
        if result == "FAIL" or strict_fail:
            blocking_fail = True
        payload_rows.append(
            {
                "path": str(summary.path),
                "parser": summary.parser,
                "result": "FAIL" if strict_fail else result,
                "errors": summary.errors,
                "warnings": summary.warnings,
                "notes": summary.notes,
            }
        )
        if not args.json and not args.quiet:
            print(f"Task recipe: {summary.path}")
            print(f"Parser: {summary.parser}")
            for note in summary.notes:
                print(f"NOTE: {note}")
            for warning in summary.warnings:
                print(f"WARN: {warning}")
            for error in summary.errors:
                print(f"FAIL: {error}")
            print(f"RESULT: {'FAIL' if strict_fail else result}")

    overall = "FAIL" if blocking_fail else ("WARN" if any(row["result"] == "WARN" for row in payload_rows) else "PASS")
    if args.json:
        print(json.dumps({"result": overall, "files": payload_rows}, indent=2, sort_keys=True))
    elif not args.quiet:
        print(f"SUMMARY: {overall} (files={len(payload_rows)})")

    return 1 if blocking_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
