#!/usr/bin/env python3
"""Validate Cell Definition v1 YAML files for offline commissioning workflows."""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

try:  # Optional dependency.
    import yaml as _pyyaml
except Exception:  # pragma: no cover - environment dependent
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
SORTING_TASK_TYPES = {"sort_by_colour", "sort_by_shape", "sort_by_class", "garbage_sorting"}
KNOWN_END_EFFECTOR_TYPES = {"finger", "suction", "magnetic", "vacuum", "custom"}


class SimpleYamlError(ValueError):
    """Raised when the fallback parser cannot handle the YAML structure."""


@dataclass
class ValidationSummary:
    path: Path
    parser: str
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors


def _strip_comment(value: str) -> str:
    quote: str | None = None
    for idx, ch in enumerate(value):
        if ch in {'"', "'"}:
            if quote is None:
                quote = ch
            elif quote == ch:
                quote = None
        if ch == "#" and quote is None and idx > 0 and value[idx - 1].isspace():
            return value[:idx].rstrip()
    return value.rstrip()


def _parse_scalar(value: str, line_no: int) -> Any:
    if (value.startswith('"') and value.endswith('"')) or (value.startswith("'") and value.endswith("'")):
        return value[1:-1]

    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    if lowered == "null":
        return None

    if re.fullmatch(r"[+-]?\d+", value):
        return int(value)
    if re.fullmatch(r"[+-]?(?:\d+\.\d*|\d*\.\d+)(?:[eE][+-]?\d+)?", value) or re.fullmatch(
        r"[+-]?\d+[eE][+-]?\d+", value
    ):
        return float(value)

    if value.startswith("["):
        if not value.endswith("]"):
            raise SimpleYamlError(f"Line {line_no}: malformed inline list")
        inner = value[1:-1].strip()
        if not inner:
            return []
        parts = [item.strip() for item in inner.split(",")]
        if any(part == "" for part in parts):
            raise SimpleYamlError(f"Line {line_no}: malformed inline list entries")
        return [_parse_scalar(part, line_no) for part in parts]

    if any(tok in value for tok in ("{", "}", "|", ">", "&", "*", "!!")):
        raise SimpleYamlError(f"Line {line_no}: unsupported YAML token for fallback parser")

    return value


def _tokenize_yaml(text: str) -> list[tuple[int, int, str]]:
    tokens: list[tuple[int, int, str]] = []
    for idx, raw in enumerate(text.splitlines(), start=1):
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        if "\t" in raw:
            raise SimpleYamlError(f"Line {idx}: tabs are not supported")
        indent = len(raw) - len(raw.lstrip(" "))
        content = _strip_comment(raw[indent:]).strip()
        if content:
            tokens.append((idx, indent, content))
    return tokens


def _parse_mapping(tokens: list[tuple[int, int, str]], start: int, indent: int) -> tuple[dict[str, Any], int]:
    data: dict[str, Any] = {}
    i = start
    while i < len(tokens):
        line_no, current_indent, content = tokens[i]
        if current_indent < indent:
            break
        if current_indent > indent:
            raise SimpleYamlError(f"Line {line_no}: unexpected indentation")
        if content.startswith("- "):
            raise SimpleYamlError(f"Line {line_no}: list item where mapping key expected")
        if ":" not in content:
            raise SimpleYamlError(f"Line {line_no}: missing ':' in mapping entry")

        key, remainder = content.split(":", 1)
        key = key.strip()
        if not key:
            raise SimpleYamlError(f"Line {line_no}: empty mapping key")
        remainder = remainder.strip()
        i += 1

        if remainder:
            data[key] = _parse_scalar(remainder, line_no)
            continue

        if i >= len(tokens) or tokens[i][1] <= current_indent:
            data[key] = {}
            continue

        child_indent = tokens[i][1]
        if tokens[i][2].startswith("- "):
            parsed, i = _parse_list(tokens, i, child_indent)
        else:
            parsed, i = _parse_mapping(tokens, i, child_indent)
        data[key] = parsed

    return data, i


def _parse_list(tokens: list[tuple[int, int, str]], start: int, indent: int) -> tuple[list[Any], int]:
    out: list[Any] = []
    i = start
    while i < len(tokens):
        line_no, current_indent, content = tokens[i]
        if current_indent < indent:
            break
        if current_indent != indent or not content.startswith("- "):
            break

        payload = content[2:].strip()
        i += 1

        if not payload:
            raise SimpleYamlError(f"Line {line_no}: empty list item unsupported")

        if payload.endswith(":") or (":" in payload and not payload.startswith(("'", '"'))):
            end = i
            while end < len(tokens) and tokens[end][1] > current_indent:
                end += 1
            synthetic = [(line_no, indent + 2, payload)] + tokens[i:end]
            parsed, consumed = _parse_mapping(synthetic, 0, indent + 2)
            if consumed != len(synthetic):
                raise SimpleYamlError(f"Line {synthetic[consumed][0]}: unsupported list mapping structure")
            out.append(parsed)
            i = end
            continue

        out.append(_parse_scalar(payload, line_no))

    return out, i


def parse_cell_definition_yaml(text: str) -> dict[str, Any]:
    tokens = _tokenize_yaml(text)
    if not tokens:
        return {}
    root_indent = min(item[1] for item in tokens)
    parsed, consumed = _parse_mapping(tokens, 0, root_indent)
    if consumed != len(tokens):
        raise SimpleYamlError(f"Line {tokens[consumed][0]}: unsupported YAML structure")
    return parsed


def load_yaml(path: Path) -> tuple[dict[str, Any], str, list[str]]:
    text = path.read_text(encoding="utf-8")
    notes: list[str] = []
    if _pyyaml is not None:
        loaded = _pyyaml.safe_load(text)
        if loaded is None:
            loaded = {}
        if not isinstance(loaded, dict):
            raise ValueError("Top-level YAML must be a mapping/object.")
        return loaded, "pyyaml", notes

    notes.append("PyYAML unavailable; using fallback parser with limited YAML support.")
    loaded = parse_cell_definition_yaml(text)
    if not isinstance(loaded, dict):
        raise ValueError("Top-level YAML must be a mapping/object.")
    return loaded, "fallback", notes


def _is_numeric_list(value: Any, expected_len: int | None = None) -> bool:
    if not isinstance(value, list):
        return False
    if expected_len is not None and len(value) != expected_len:
        return False
    return all(isinstance(v, (int, float)) for v in value)


def _validate_pose(summary: ValidationSummary, owner: str, block: dict[str, Any]) -> None:
    pose_xyz = block.get("pose_xyz")
    pose_rpy = block.get("pose_rpy")
    if not _is_numeric_list(pose_xyz, 3):
        summary.errors.append(f"{owner}.pose_xyz must be numeric list length 3.")
    if not _is_numeric_list(pose_rpy, 3):
        summary.errors.append(f"{owner}.pose_rpy must be numeric list length 3.")


def _validate_dimensions(summary: ValidationSummary, owner: str, dims: Any) -> None:
    if not isinstance(dims, list) or not dims:
        summary.errors.append(f"{owner}.dimensions must be a non-empty list of positive numbers.")
        return
    for idx, val in enumerate(dims):
        if not isinstance(val, (int, float)) or val <= 0:
            summary.errors.append(f"{owner}.dimensions[{idx}] must be a positive number.")


def validate_cell_definition(defn: dict[str, Any], path: Path, parser: str, parser_notes: list[str]) -> ValidationSummary:
    result = ValidationSummary(path=path, parser=parser, notes=list(parser_notes))

    required_keys = [
        "schema_version",
        "cell",
        "robot",
        "end_effector",
        "camera",
        "environment",
        "objects",
        "task",
        "commissioning",
    ]
    for key in required_keys:
        if key not in defn:
            result.errors.append(f"Missing required top-level key: {key}")

    if defn.get("schema_version") != "cell_definition/v1":
        result.errors.append("schema_version must be exactly 'cell_definition/v1'.")

    robot = defn.get("robot") if isinstance(defn.get("robot"), dict) else {}
    end_effector = defn.get("end_effector") if isinstance(defn.get("end_effector"), dict) else {}
    environment = defn.get("environment") if isinstance(defn.get("environment"), dict) else {}
    objects = defn.get("objects") if isinstance(defn.get("objects"), list) else []
    task = defn.get("task") if isinstance(defn.get("task"), dict) else {}

    if "safe_joint_state" not in robot:
        result.errors.append("robot.safe_joint_state key is required (empty list allowed when home_named_target exists).")
    else:
        safe_joint_state = robot.get("safe_joint_state")
        if not isinstance(safe_joint_state, list):
            result.errors.append("robot.safe_joint_state must be a list.")
        elif not safe_joint_state and not isinstance(robot.get("home_named_target"), str):
            result.errors.append("robot.safe_joint_state is empty and robot.home_named_target is missing.")
        elif safe_joint_state and not all(isinstance(v, (int, float)) for v in safe_joint_state):
            result.errors.append("robot.safe_joint_state entries must be numeric.")

    ee_type = end_effector.get("type")
    if isinstance(ee_type, str) and ee_type not in KNOWN_END_EFFECTOR_TYPES:
        result.warnings.append(
            f"Unknown end_effector.type '{ee_type}'. Continuing because this is metadata-only validation."
        )

    support_surfaces = environment.get("support_surfaces")
    if isinstance(support_surfaces, list):
        for idx, surface in enumerate(support_surfaces):
            if not isinstance(surface, dict):
                result.errors.append(f"environment.support_surfaces[{idx}] must be a mapping.")
                continue
            _validate_pose(result, f"environment.support_surfaces[{idx}]", surface)
            _validate_dimensions(result, f"environment.support_surfaces[{idx}]", surface.get("dimensions"))

    for idx, obj in enumerate(objects):
        if not isinstance(obj, dict):
            result.errors.append(f"objects[{idx}] must be a mapping.")
            continue
        _validate_pose(result, f"objects[{idx}]", obj)
        _validate_dimensions(result, f"objects[{idx}]", obj.get("dimensions"))

    task_type = task.get("type")
    if not isinstance(task_type, str):
        result.errors.append("task.type must be a non-empty string.")
    elif task_type not in SUPPORTED_TASK_TYPES:
        result.errors.append(f"task.type '{task_type}' is unsupported for cell_definition/v1.")
    elif task_type == "custom":
        result.warnings.append("task.type is 'custom'; ensure downstream tooling knows how to interpret it.")

    destinations = task.get("destinations")
    if not isinstance(destinations, list) or not destinations:
        result.errors.append("task.destinations must contain at least one destination.")
        destination_ids: set[str] = set()
    else:
        destination_ids = set()
        for idx, destination in enumerate(destinations):
            if not isinstance(destination, dict):
                result.errors.append(f"task.destinations[{idx}] must be a mapping.")
                continue
            destination_id = destination.get("id")
            if not isinstance(destination_id, str) or not destination_id.strip():
                result.errors.append(f"task.destinations[{idx}].id must be a non-empty string.")
            else:
                destination_ids.add(destination_id)
            _validate_pose(result, f"task.destinations[{idx}]", destination)

    rules = task.get("rules")
    if task_type in SORTING_TASK_TYPES and (not isinstance(rules, list) or not rules):
        result.errors.append(f"task.rules must be a non-empty list for sorting task type '{task_type}'.")

    fallback_present = False
    if isinstance(rules, list):
        for idx, rule in enumerate(rules):
            if not isinstance(rule, dict):
                result.errors.append(f"task.rules[{idx}] must be a mapping.")
                continue
            dest = rule.get("destination")
            if not isinstance(dest, str) or not dest.strip():
                result.errors.append(f"task.rules[{idx}].destination must be a non-empty string.")
            elif dest not in destination_ids:
                result.errors.append(f"task.rules[{idx}] destination '{dest}' does not exist in task.destinations.")
            when = rule.get("when")
            if isinstance(when, dict) and when.get("always") is True:
                fallback_present = True

    if task_type in SORTING_TASK_TYPES and not fallback_present:
        result.warnings.append("Sorting task has no explicit fallback rule with when.always=true.")

    return result


def print_summary(summary: ValidationSummary) -> None:
    print(f"Cell definition: {summary.path}")
    print(f"Parser: {summary.parser}")
    for note in summary.notes:
        print(f"NOTE: {note}")
    for warning in summary.warnings:
        print(f"WARN: {warning}")
    for error in summary.errors:
        print(f"FAIL: {error}")

    status = "PASS" if summary.ok and not summary.warnings else "WARN" if summary.ok else "FAIL"
    print(f"RESULT: {status}")
    print(
        "SUMMARY: "
        f"PASS={'1' if summary.ok and not summary.warnings else '0'} "
        f"WARN={len(summary.warnings)} "
        f"FAIL={len(summary.errors)}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("yaml_path", type=Path, help="Path to a cell_definition YAML file")
    args = parser.parse_args()

    try:
        loaded, parser_name, notes = load_yaml(args.yaml_path)
    except FileNotFoundError:
        print(f"FAIL: File not found: {args.yaml_path}")
        return 2
    except Exception as exc:
        print(f"FAIL: Malformed YAML: {exc}")
        return 1

    summary = validate_cell_definition(loaded, args.yaml_path, parser_name, notes)
    print_summary(summary)
    return 0 if summary.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
