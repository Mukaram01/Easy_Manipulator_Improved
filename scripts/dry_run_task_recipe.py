#!/usr/bin/env python3
"""Offline task recipe dry-run resolver for scene manifests."""

from __future__ import annotations

import argparse
import importlib.util
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
MANIFEST_CANDIDATES = ("scene_manifest.yaml", "workcell.yaml")
VALIDATOR_PATH = REPO_ROOT / "scripts" / "validate_scene_contract.py"


def _load_validator_module():
    spec = importlib.util.spec_from_file_location("validate_scene_contract", VALIDATOR_PATH)
    if not spec or not spec.loader:
        raise RuntimeError(f"Unable to load validator module from {VALIDATOR_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules["validate_scene_contract"] = module
    spec.loader.exec_module(module)
    return module


validator = _load_validator_module()


@dataclass
class DryRunResult:
    scene: str
    status: str
    recipe_id: str
    task_type: str
    object_id: str
    object_attributes: dict[str, Any]
    matched_rule_id: str
    selected_destination_id: str
    selected_action: str
    notes: list[str]


def discover_scene_manifests() -> list[tuple[str, Path]]:
    scenes_dir = REPO_ROOT / "scenes"
    if not scenes_dir.is_dir():
        return []

    discovered: list[tuple[str, Path]] = []
    for entry in sorted(scenes_dir.iterdir()):
        if not entry.is_dir():
            continue
        for candidate in MANIFEST_CANDIDATES:
            path = entry / candidate
            if path.is_file():
                discovered.append((entry.name, path))
                break
    return discovered


def _recipe_enabled(recipe: Any) -> bool:
    return isinstance(recipe, dict) and recipe.get("enabled") is True


def _self_test_enabled(self_test: Any) -> bool:
    return isinstance(self_test, dict) and self_test.get("enabled") is True


def _normalize_object_attributes(object_block: dict[str, Any]) -> dict[str, Any]:
    attrs = object_block.get("attributes")
    if not isinstance(attrs, dict):
        attrs = {}
    object_shape = object_block.get("shape") if isinstance(object_block.get("shape"), str) else "unknown"
    return {
        "class": attrs.get("class", "part"),
        "shape": attrs.get("shape", object_shape),
        "colour": attrs.get("colour", "unknown"),
        "material": attrs.get("material", "unknown"),
    }


def _evaluate_rules(decision_rules: Any, object_attributes: dict[str, Any]) -> tuple[dict[str, Any] | None, str | None]:
    if not isinstance(decision_rules, list):
        return None, "task_recipe.decision_rules must be a list."

    for rule in decision_rules:
        if not isinstance(rule, dict):
            continue
        when = rule.get("when")
        if not isinstance(when, dict):
            continue

        if when.get("default") is True or when.get("always") is True:
            return rule, None

        attribute = when.get("attribute")
        expected_value = when.get("equals")
        if isinstance(attribute, str) and attribute in object_attributes:
            if object_attributes.get(attribute) == expected_value:
                return rule, None

    return None, "No decision rule matched the simulated self-test object attributes."


def _resolve_destination(recipe: dict[str, Any], destination_id: Any) -> tuple[dict[str, Any] | None, str | None]:
    if not isinstance(destination_id, str) or not destination_id.strip():
        return None, "Matched rule is missing a non-empty destination id."
    destinations = recipe.get("destinations")
    if not isinstance(destinations, list):
        return None, "task_recipe.destinations must be a list."

    for destination in destinations:
        if isinstance(destination, dict) and destination.get("id") == destination_id:
            if not validator._is_numeric_list(destination.get("pose_xyz"), 3):
                return None, f"Destination '{destination_id}' has malformed pose_xyz; expected numeric list length 3."
            if not validator._is_numeric_list(destination.get("pose_rpy"), 3):
                return None, f"Destination '{destination_id}' has malformed pose_rpy; expected numeric list length 3."
            action = destination.get("action")
            if not isinstance(action, str) or not action.strip():
                return None, f"Destination '{destination_id}' is missing a non-empty action."
            return destination, None

    return None, f"Matched destination '{destination_id}' not found in task_recipe.destinations."


def evaluate_scene(scene_name: str, manifest_path: Path) -> DryRunResult:
    try:
        manifest, _, parser_notes = validator._read_manifest(str(manifest_path))
    except Exception as exc:  # pragma: no cover
        return DryRunResult(
            scene=scene_name,
            status="FAIL",
            recipe_id="(n/a)",
            task_type="(n/a)",
            object_id="(n/a)",
            object_attributes={},
            matched_rule_id="(n/a)",
            selected_destination_id="(n/a)",
            selected_action="(n/a)",
            notes=[f"Manifest parse failed: {exc}"],
        )

    notes = list(parser_notes)
    recipe = manifest.get("task_recipe")
    self_test = manifest.get("self_test")

    if recipe is None:
        notes.append("task_recipe not defined; dry-run not applicable.")
        return DryRunResult(scene_name, "WARN", "(n/a)", "(n/a)", "(n/a)", {}, "(n/a)", "(n/a)", "(n/a)", notes)

    if not _recipe_enabled(recipe):
        notes.append("task_recipe missing or disabled; dry-run skipped for this scene.")
        return DryRunResult(
            scene_name,
            "WARN",
            str(recipe.get("id", "(n/a)")) if isinstance(recipe, dict) else "(n/a)",
            str(recipe.get("type", "(n/a)")) if isinstance(recipe, dict) else "(n/a)",
            "(n/a)",
            {},
            "(n/a)",
            "(n/a)",
            "(n/a)",
            notes,
        )

    if not _self_test_enabled(self_test):
        notes.append("self_test missing or disabled; dry-run skipped for this scene.")
        return DryRunResult(
            scene_name,
            "SKIP",
            str(recipe.get("id", "(n/a)")),
            str(recipe.get("type", "(n/a)")),
            "(n/a)",
            {},
            "(n/a)",
            "(n/a)",
            "(n/a)",
            notes,
        )

    contract_status, contract_notes = validator.validate_task_recipe_block(manifest)
    if contract_status == "FAIL":
        notes.extend(contract_notes)
        return DryRunResult(
            scene_name,
            "FAIL",
            str(recipe.get("id", "(n/a)")),
            str(recipe.get("type", "(n/a)")),
            "(n/a)",
            {},
            "(n/a)",
            "(n/a)",
            "(n/a)",
            notes,
        )

    object_block = self_test.get("object") if isinstance(self_test, dict) else None
    if not isinstance(object_block, dict):
        notes.append("self_test.object missing or invalid; cannot build simulated object.")
        return DryRunResult(
            scene_name,
            "FAIL",
            str(recipe.get("id", "(n/a)")),
            str(recipe.get("type", "(n/a)")),
            "(n/a)",
            {},
            "(n/a)",
            "(n/a)",
            "(n/a)",
            notes,
        )

    object_id = str(object_block.get("id", "(n/a)"))
    object_attributes = _normalize_object_attributes(object_block)

    matched_rule, match_error = _evaluate_rules(recipe.get("decision_rules"), object_attributes)
    if match_error:
        notes.append(match_error)
        return DryRunResult(
            scene_name,
            "FAIL",
            str(recipe.get("id", "(n/a)")),
            str(recipe.get("type", "(n/a)")),
            object_id,
            object_attributes,
            "(n/a)",
            "(n/a)",
            "(n/a)",
            notes,
        )

    destination_id = matched_rule.get("destination")
    destination, destination_error = _resolve_destination(recipe, destination_id)
    if destination_error:
        notes.append(destination_error)
        return DryRunResult(
            scene_name,
            "FAIL",
            str(recipe.get("id", "(n/a)")),
            str(recipe.get("type", "(n/a)")),
            object_id,
            object_attributes,
            str(matched_rule.get("id", "(n/a)")),
            str(destination_id) if destination_id is not None else "(n/a)",
            "(n/a)",
            notes,
        )

    notes.append("Dry-run resolved a matching rule, destination, and action.")
    return DryRunResult(
        scene_name,
        "PASS",
        str(recipe.get("id", "(n/a)")),
        str(recipe.get("type", "(n/a)")),
        object_id,
        object_attributes,
        str(matched_rule.get("id", "(n/a)")),
        str(destination.get("id", "(n/a)")),
        str(destination.get("action", "(n/a)")),
        notes,
    )


def _format_attrs(attrs: dict[str, Any]) -> str:
    if not attrs:
        return "(n/a)"
    return ", ".join(f"{key}={attrs[key]}" for key in sorted(attrs.keys()))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenes", nargs="*", help="Optional scene names to include.")
    parser.add_argument("--check", action="store_true", help="Print concise PASS/WARN/FAIL/SKIP summary.")
    args = parser.parse_args()

    discovered = discover_scene_manifests()
    if args.scenes:
        requested = set(args.scenes)
        discovered = [entry for entry in discovered if entry[0] in requested]

    if not discovered:
        print("No scene manifests discovered for task recipe dry-run.")
        return 2

    rows = [evaluate_scene(scene, path) for scene, path in discovered]

    if args.check:
        for row in rows:
            print(
                f"{row.status:4} {row.scene:24} rule={row.matched_rule_id} "
                f"dest={row.selected_destination_id} action={row.selected_action} attrs={_format_attrs(row.object_attributes)}"
            )
        pass_count = sum(1 for row in rows if row.status == "PASS")
        warn_count = sum(1 for row in rows if row.status == "WARN")
        fail_count = sum(1 for row in rows if row.status == "FAIL")
        skip_count = sum(1 for row in rows if row.status == "SKIP")
        print(f"Summary: PASS={pass_count} WARN={warn_count} FAIL={fail_count} SKIP={skip_count}")

    return 1 if any(row.status == "FAIL" for row in rows) else 0


if __name__ == "__main__":
    raise SystemExit(main())
