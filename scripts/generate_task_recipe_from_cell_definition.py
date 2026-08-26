#!/usr/bin/env python3
"""Generate task_recipe/v1 preview YAML from a cell_definition/v1 file."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import generate_scene_from_cell_definition as scene_generator
import validate_cell_definition as cell_validator
import validate_task_recipe as task_validator


def _to_v1_task(cell_def: dict[str, Any]) -> dict[str, Any]:
    task = cell_def.get("task", {}) if isinstance(cell_def.get("task"), dict) else {}
    perception = cell_def.get("perception", {}) if isinstance(cell_def.get("perception"), dict) else {}
    perception_backed = str(task.get("object_source", "")).lower() == "perception" or bool(perception.get("enabled", False))
    source = task.get("perception_source") if perception_backed else task.get("source_object")
    task_id = str(task.get("id", "generated_task_recipe"))

    destinations = []
    for destination in task.get("destinations", []) if isinstance(task.get("destinations"), list) else []:
        if not isinstance(destination, dict):
            continue
        destinations.append(
            {
                "id": destination.get("id"),
                "frame": destination.get("frame", "world"),
                "pose_xyz": destination.get("pose_xyz", [0.0, 0.0, 0.0]),
                "pose_rpy": destination.get("pose_rpy", [0.0, 0.0, 0.0]),
                "type": destination.get("type", "bin"),
                "label": destination.get("label", destination.get("id", "destination")),
            }
        )

    decision_rules = []
    for rule in task.get("rules", []) if isinstance(task.get("rules"), list) else []:
        if not isinstance(rule, dict):
            continue
        mapped = scene_generator._map_rule(rule)
        when = mapped.get("when", {}) if isinstance(mapped.get("when"), dict) else {}
        item = {
            "id": mapped.get("id", "rule"),
            "when": when,
            "destination": mapped.get("destination"),
        }
        decision_rules.append(item)

    if not any(isinstance(r.get("when"), dict) and (r["when"].get("default") is True or r["when"].get("always") is True) for r in decision_rules):
        fallback_dest = destinations[-1]["id"] if destinations else "unknown_reject_bin"
        decision_rules.append(
            {
                "id": "auto_fallback",
                "when": {"default": True},
                "destination": fallback_dest,
            }
        )

    return {
        "schema_version": "task_recipe/v1",
        "task": {
            "id": task_id,
            "type": task.get("type", "custom"),
            "source": source or ("detected_objects/v1" if perception_backed else "detected_object"),
            "perception_source": perception.get("normalized_output_contract", "detected_objects/v1") if perception_backed else "cell_definition",
            "object_source": "perception" if perception_backed else task.get("object_source", "fixed_object"),
            **({"pick_zone": task.get("pick_zone")} if task.get("pick_zone") else {}),
            "object_attributes": ["colour", "shape", "class", "material", "confidence", "inspection_result"],
            "destinations": destinations,
            "decision_rules": decision_rules,
            "notes": "Generated preview from cell_definition/v1. Offline metadata only.",
        },
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("cell_definition", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args(argv)

    try:
        loaded, parser_name, parser_notes = cell_validator.load_yaml(args.cell_definition)
    except Exception as exc:
        print(f"FAIL: Unable to load cell definition: {exc}")
        return 1

    summary = cell_validator.validate_cell_definition(loaded, args.cell_definition, parser_name, parser_notes, strict=args.strict)
    if not summary.ok:
        print("FAIL: Input cell definition failed validation.")
        return 1

    recipe_doc = _to_v1_task(loaded)
    task_summary = task_validator.validate_task_recipe_doc(recipe_doc, args.output, "generated", [], strict=args.strict)
    result = "PASS" if task_summary.ok and not task_summary.warnings else "WARN" if task_summary.ok else "FAIL"

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(scene_generator._to_yaml_text(recipe_doc), encoding="utf-8")

    if args.json:
        print(
            json.dumps(
                {
                    "result": result,
                    "output": str(args.output),
                    "warnings": task_summary.warnings,
                    "errors": task_summary.errors,
                },
                indent=2,
                sort_keys=True,
            )
        )
    else:
        for warning in task_summary.warnings:
            print(f"WARN: {warning}")
        for error in task_summary.errors:
            print(f"FAIL: {error}")
        print(f"RESULT: {result}")
        print(f"Wrote: {args.output}")

    return 1 if (result == "FAIL" or (args.strict and result == "WARN")) else 0


if __name__ == "__main__":
    raise SystemExit(main())
