#!/usr/bin/env python3
"""Validate Cell Definition v1 YAML files for offline commissioning workflows."""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from capability_registry import DEFAULT_CAPABILITIES_DIR, load_capability_registry

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
    capability_summary: dict[str, Any] = field(default_factory=dict)
    environment_layout_summary: dict[str, Any] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return not self.errors

    @property
    def has_warnings(self) -> bool:
        return bool(self.warnings)


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
        if current_indent < indent or current_indent != indent or not content.startswith("- "):
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
                raise SimpleYamlError(f"Line {line_no}: unsupported list mapping structure")
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
    return isinstance(value, list) and (expected_len is None or len(value) == expected_len) and all(
        isinstance(v, (int, float)) for v in value
    )


def _validate_pose(summary: ValidationSummary, owner: str, block: dict[str, Any]) -> None:
    if not _is_numeric_list(block.get("pose_xyz"), 3):
        summary.errors.append(f"{owner}.pose_xyz must be numeric list length 3.")
    if not _is_numeric_list(block.get("pose_rpy"), 3):
        summary.errors.append(f"{owner}.pose_rpy must be numeric list length 3.")


def _validate_dimensions(summary: ValidationSummary, owner: str, dims: Any) -> None:
    if not isinstance(dims, list) or not dims:
        summary.errors.append(f"{owner}.dimensions must be a non-empty list of positive numbers.")
        return
    for idx, val in enumerate(dims):
        if not isinstance(val, (int, float)) or val <= 0:
            summary.errors.append(f"{owner}.dimensions[{idx}] must be a positive number.")


def _extract_refs(defn: dict[str, Any]) -> dict[str, Any]:
    robot = defn.get("robot") if isinstance(defn.get("robot"), dict) else {}
    end_effector = defn.get("end_effector") if isinstance(defn.get("end_effector"), dict) else {}
    sensors = defn.get("sensors") if isinstance(defn.get("sensors"), list) else []
    if not sensors and isinstance(defn.get("camera"), dict) and defn["camera"].get("capability"):
        sensors = [{"capability": defn["camera"].get("capability")}]
    task = defn.get("task") if isinstance(defn.get("task"), dict) else {}
    environment = defn.get("environment") if isinstance(defn.get("environment"), dict) else {}
    assets = environment.get("assets") if isinstance(environment.get("assets"), list) else []
    return {
        "robot": robot.get("capability"),
        "end_effector": end_effector.get("capability"),
        "sensors": [item.get("capability") for item in sensors if isinstance(item, dict) and item.get("capability")],
        "task": task.get("capability"),
        "environment_assets": [
            item.get("capability") for item in assets if isinstance(item, dict) and item.get("capability")
        ],
    }


def _check_capabilities(defn: dict[str, Any], summary: ValidationSummary, strict: bool, capabilities_dir: Path | None) -> None:
    refs = _extract_refs(defn)
    registry = load_capability_registry(capabilities_dir)
    summary.notes.extend(registry.parser_notes)
    resolved: dict[str, Any] = {}
    statuses: list[str] = []

    def resolve(kind: str, cap_id: str | None) -> dict[str, Any] | None:
        if not cap_id:
            return None
        rec = registry.get(cap_id)
        if rec is None:
            message = f"Unknown {kind} capability id '{cap_id}'."
            (summary.errors if strict else summary.warnings).append(message)
            statuses.append("FAIL" if strict else "WARN")
            return None
        statuses.append("PASS")
        return {"id": rec.capability_id, "family": rec.family, "schema_version": rec.schema_version}

    robot_meta = resolve("robot", refs["robot"])
    ee_meta = resolve("end_effector", refs["end_effector"])
    sensor_meta = [meta for cap in refs["sensors"] if (meta := resolve("sensor", cap))]
    task_meta = resolve("task", refs["task"])
    asset_meta = [meta for cap in refs["environment_assets"] if (meta := resolve("environment asset", cap))]

    if robot_meta:
        resolved["robot"] = robot_meta
    if ee_meta:
        resolved["end_effector"] = ee_meta
    if sensor_meta:
        resolved["sensors"] = sensor_meta
    if task_meta:
        resolved["task"] = task_meta
    if asset_meta:
        resolved["environment_assets"] = asset_meta

    robot_rec = registry.get(refs["robot"])
    ee_rec = registry.get(refs["end_effector"])
    task_rec = registry.get(refs["task"])
    sensor_recs = [registry.get(cap) for cap in refs["sensors"] if registry.get(cap)]

    if ee_rec and robot_rec:
        compat = ee_rec.payload.get("compatible_robot_families")
        if isinstance(compat, list) and robot_rec.family and robot_rec.family not in compat:
            summary.errors.append(
                f"Capability mismatch: end_effector '{ee_rec.capability_id}' not compatible with robot family '{robot_rec.family}'."
            )

    if task_rec and robot_rec:
        required_family = task_rec.payload.get("task_family")
        supported = robot_rec.payload.get("supported_task_families")
        if isinstance(supported, list) and required_family and required_family not in supported:
            summary.errors.append(
                f"Capability mismatch: robot '{robot_rec.capability_id}' does not support task family '{required_family}'."
            )

    if task_rec and ee_rec:
        task_family = str(task_rec.payload.get("task_family", ""))
        ee_family = str(ee_rec.family or "")
        required_ee_caps = task_rec.payload.get("required_end_effector_capabilities")
        needs_magnetic = isinstance(required_ee_caps, list) and any(str(item).lower() == "magnetic" for item in required_ee_caps)
        if (any(tok in task_family for tok in ("magnetic", "metal_magnetic")) or needs_magnetic) and "magnetic" not in ee_family:
            summary.errors.append(
                f"Capability mismatch: task '{task_rec.capability_id}' requires magnetic gripping but end effector '{ee_rec.capability_id}' is '{ee_family}'."
            )
        if task_family == "conveyor_sorting" and ee_family in {"finger_gripper", "three_finger_gripper"}:
            summary.warnings.append(
                "Task conveyor_sorting typically prefers suction/vacuum EOAT; finger gripper may still work depending on parts."
            )

    if task_rec and sensor_recs:
        req_attrs = task_rec.payload.get("required_sensor_attributes")
        if isinstance(req_attrs, list):
            sensor_outputs: set[str] = set()
            sensor_attrs: set[str] = set()
            for rec in sensor_recs:
                sensor_outputs.update(rec.payload.get("detection_outputs", []) if isinstance(rec.payload.get("detection_outputs"), list) else [])
                sensor_attrs.update(rec.payload.get("supported_object_attributes", []) if isinstance(rec.payload.get("supported_object_attributes"), list) else [])
            for attr in req_attrs:
                if attr not in sensor_outputs and attr not in sensor_attrs:
                    summary.warnings.append(
                        f"Task requires sensor attribute '{attr}' but selected sensors do not advertise it."
                    )

    if refs["environment_assets"] and not asset_meta:
        summary.warnings.append("Environment assets were referenced but no asset capability could be resolved.")

    summary.capability_summary = {
        "capability_refs": refs,
        "resolved": resolved,
        "checks": {
            "status": "FAIL" if summary.errors else ("WARN" if summary.warnings else "PASS"),
            "events": statuses,
        },
    }


def validate_cell_definition(
    defn: dict[str, Any], path: Path, parser: str, parser_notes: list[str], strict: bool = False, capabilities_dir: Path | None = None
) -> ValidationSummary:
    result = ValidationSummary(path=path, parser=parser, notes=list(parser_notes))

    required_keys = ["schema_version", "cell", "robot", "end_effector", "camera", "environment", "objects", "task", "commissioning"]
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
        result.warnings.append(f"Unknown end_effector.type '{ee_type}'. Continuing because this is metadata-only validation.")

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

    layout_path_value = environment.get("layout")
    if layout_path_value is not None:
        if not isinstance(layout_path_value, str) or not layout_path_value.strip():
            result.errors.append("environment.layout must be a non-empty string path when provided.")
        else:
            layout_path = (Path(__file__).resolve().parents[1] / layout_path_value).resolve()
            if not layout_path.exists():
                msg = f"environment.layout path not found: {layout_path_value}"
                (result.errors if strict else result.warnings).append(msg)
            else:
                try:
                    import validate_environment_layout as env_layout_validator

                    loaded_layout, layout_parser, layout_notes = env_layout_validator.load_layout(layout_path)
                    layout_summary = env_layout_validator.validate_layout(
                        loaded_layout,
                        layout_path,
                        layout_parser,
                        layout_notes,
                        strict=strict,
                    )
                    result.warnings.extend(layout_summary.warnings)
                    result.errors.extend(layout_summary.errors)
                    result.notes.extend([f"environment.layout: {note}" for note in layout_summary.notes])
                    result.environment_layout_summary = {
                        "path": layout_path_value,
                        "layout_id": loaded_layout.get("layout_id"),
                        "asset_count": layout_summary.summary.get("asset_count", 0),
                        "zone_count": layout_summary.summary.get("zone_count", 0),
                        "safety_zone_count": layout_summary.summary.get("safety_zone_count", 0),
                        "result": "PASS" if layout_summary.ok and not layout_summary.warnings else "WARN" if layout_summary.ok else "FAIL",
                    }
                except Exception as exc:
                    (result.errors if strict else result.warnings).append(
                        f"environment.layout validation failed for '{layout_path_value}': {exc}"
                    )

    _check_capabilities(defn, result, strict=strict, capabilities_dir=capabilities_dir)
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
    print(f"SUMMARY: PASS={'1' if summary.ok and not summary.warnings else '0'} WARN={len(summary.warnings)} FAIL={len(summary.errors)}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("yaml_path", type=Path, help="Path to a cell_definition YAML file")
    parser.add_argument("--strict", action="store_true", help="Treat uncertain warnings as failures where supported")
    parser.add_argument("--json", action="store_true", help="Print machine-readable output")
    parser.add_argument("--quiet", action="store_true", help="Suppress human-readable output")
    parser.add_argument("--capabilities-dir", type=Path, default=DEFAULT_CAPABILITIES_DIR)
    args = parser.parse_args()

    try:
        loaded, parser_name, notes = load_yaml(args.yaml_path)
    except FileNotFoundError:
        print(f"FAIL: File not found: {args.yaml_path}")
        return 2
    except Exception as exc:
        print(f"FAIL: Malformed YAML: {exc}")
        return 1

    summary = validate_cell_definition(loaded, args.yaml_path, parser_name, notes, strict=args.strict, capabilities_dir=args.capabilities_dir)
    if args.json:
        payload = {
            "path": str(summary.path),
            "parser": summary.parser,
            "errors": summary.errors,
            "warnings": summary.warnings,
            "notes": summary.notes,
            "result": "PASS" if summary.ok and not summary.warnings else "WARN" if summary.ok else "FAIL",
            "capabilities": summary.capability_summary,
            "environment_layout": summary.environment_layout_summary,
        }
        print(json.dumps(payload, indent=2, sort_keys=True))
    elif not args.quiet:
        print_summary(summary)
    return 0 if summary.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
