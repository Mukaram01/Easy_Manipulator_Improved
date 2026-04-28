#!/usr/bin/env python3
"""Validate offline capability contract files (YAML/JSON)."""

from __future__ import annotations

import argparse
import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

try:  # Optional dependency.
    import yaml as _pyyaml
except Exception:  # pragma: no cover
    _pyyaml = None

SUPPORTED_EXTENSIONS = {".yaml", ".yml", ".json"}
SCHEMA_TYPES = {
    "robot_capability/v1": "robot",
    "end_effector_capability/v1": "end_effector",
    "sensor_capability/v1": "sensor",
    "task_capability/v1": "task",
    "environment_asset/v1": "asset",
}

ENUMS = {
    "robot.family": {
        "serial_6_axis",
        "collaborative_6_axis",
        "scara",
        "delta",
        "gantry",
        "mobile_manipulator",
        "custom",
    },
    "robot.mounting": {"floor", "table", "wall", "ceiling", "overhead", "custom"},
    "end_effector.family": {
        "finger_gripper",
        "three_finger_gripper",
        "suction",
        "vacuum_array",
        "magnetic",
        "tool_changer",
        "custom",
    },
    "sensor.family": {
        "rgbd_camera",
        "2d_camera",
        "depth_camera",
        "barcode_reader",
        "force_torque",
        "proximity",
        "custom",
    },
    "sensor.mounting": {"fixed", "wrist", "overhead", "custom"},
    "task.task_family": {
        "pick_place",
        "sort_by_colour",
        "sort_by_shape",
        "garbage_sorting",
        "machine_tending",
        "palletising",
        "inspection_routing",
        "conveyor_sorting",
        "custom",
    },
    "asset.family": {
        "table",
        "workbench",
        "bin",
        "tray",
        "conveyor",
        "fixture",
        "machine",
        "safety_fence",
        "camera_mount",
        "custom",
    },
}

LIST_FIELDS = {
    "robot": [
        "planning_groups",
        "tool_frames",
        "named_targets",
        "supported_interfaces",
        "supported_task_families",
        "limitations_warnings",
    ],
    "end_effector": [
        "compatible_robot_families",
        "required_frames",
        "grasp_frames",
        "contact_links",
        "allowed_touch_links",
        "supported_grasp_strategies",
        "required_io_signals",
        "limitations_warnings",
    ],
    "sensor": [
        "frames",
        "topics",
        "detection_outputs",
        "supported_object_attributes",
        "calibration_requirements",
        "limitations_warnings",
    ],
    "task": [
        "required_robot_capabilities",
        "required_end_effector_capabilities",
        "required_sensor_attributes",
        "required_destinations",
        "expected_validation_checks",
        "runtime_requirements",
        "limitations_warnings",
    ],
    "asset": ["dimensions", "limitations_warnings"],
}


class SimpleYamlError(ValueError):
    """Raised when fallback parser hits unsupported syntax."""


@dataclass
class _Line:
    line_no: int
    indent: int
    content: str


@dataclass
class FileValidationResult:
    path: str
    status: str
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)


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


def _tokenize_simple_yaml(text: str) -> list[_Line]:
    tokens: list[_Line] = []
    for idx, raw_line in enumerate(text.splitlines(), start=1):
        if not raw_line.strip() or raw_line.lstrip().startswith("#"):
            continue
        if "\t" in raw_line:
            raise SimpleYamlError(f"Line {idx}: tabs are not supported")
        indent = len(raw_line) - len(raw_line.lstrip(" "))
        content = _strip_comment(raw_line[indent:]).strip()
        if not content:
            continue
        if any(marker in content for marker in ("|", ">", "&", "*", "!!")):
            raise SimpleYamlError(f"Line {idx}: advanced YAML tokens are not supported")
        tokens.append(_Line(line_no=idx, indent=indent, content=content))
    return tokens


def _parse_scalar(value: str, line_no: int) -> Any:
    value = value.strip()
    if not value:
        return ""
    if value.startswith("["):
        if not value.endswith("]"):
            raise SimpleYamlError(f"Line {line_no}: malformed inline list")
        inner = value[1:-1].strip()
        if not inner:
            return []
        parts = [part.strip() for part in inner.split(",")]
        if any(not part for part in parts):
            raise SimpleYamlError(f"Line {line_no}: malformed inline list entries")
        return [_parse_scalar(part, line_no) for part in parts]
    if (value.startswith('"') and value.endswith('"')) or (value.startswith("'") and value.endswith("'")):
        return value[1:-1]
    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    if re.fullmatch(r"[+-]?\d+", value):
        return int(value)
    if re.fullmatch(r"[+-]?(?:\d+\.\d*|\d*\.\d+)(?:[eE][+-]?\d+)?", value) or re.fullmatch(
        r"[+-]?\d+[eE][+-]?\d+", value
    ):
        return float(value)
    if value.startswith("{"):
        raise SimpleYamlError(f"Line {line_no}: flow mappings are not supported")
    return value


def _parse_mapping(lines: list[_Line], start: int, indent: int) -> tuple[dict[str, Any], int]:
    result: dict[str, Any] = {}
    i = start
    while i < len(lines):
        line = lines[i]
        if line.indent < indent:
            break
        if line.indent > indent:
            raise SimpleYamlError(f"Line {line.line_no}: unexpected indentation")
        if line.content.startswith("- "):
            raise SimpleYamlError(f"Line {line.line_no}: list item where mapping key expected")
        if ":" not in line.content:
            raise SimpleYamlError(f"Line {line.line_no}: missing ':'")
        key, remainder = line.content.split(":", 1)
        key = key.strip()
        value_text = remainder.strip()
        i += 1
        if value_text:
            result[key] = _parse_scalar(value_text, line.line_no)
            continue
        if i >= len(lines) or lines[i].indent <= indent:
            result[key] = {}
            continue
        child_indent = lines[i].indent
        if lines[i].content.startswith("- "):
            parsed, i = _parse_list(lines, i, child_indent)
            result[key] = parsed
        else:
            parsed, i = _parse_mapping(lines, i, child_indent)
            result[key] = parsed
    return result, i


def _parse_list(lines: list[_Line], start: int, indent: int) -> tuple[list[Any], int]:
    items: list[Any] = []
    i = start
    while i < len(lines):
        line = lines[i]
        if line.indent < indent:
            break
        if line.indent != indent or not line.content.startswith("- "):
            break
        item_text = line.content[2:].strip()
        if not item_text:
            raise SimpleYamlError(f"Line {line.line_no}: empty list item")
        if item_text.endswith(":") or (":" in item_text and not item_text.startswith(("'", '"'))):
            end = i + 1
            while end < len(lines) and lines[end].indent > indent:
                end += 1
            synthetic = [_Line(line_no=line.line_no, indent=indent + 2, content=item_text)]
            synthetic.extend(lines[i + 1 : end])
            parsed_map, consumed = _parse_mapping(synthetic, 0, indent + 2)
            if consumed != len(synthetic):
                raise SimpleYamlError(f"Line {line.line_no}: unsupported list mapping")
            items.append(parsed_map)
            i = end
            continue
        items.append(_parse_scalar(item_text, line.line_no))
        i += 1
    return items, i


def parse_simple_yaml(text: str) -> dict[str, Any]:
    lines = _tokenize_simple_yaml(text)
    if not lines:
        return {}
    root_indent = min(line.indent for line in lines)
    parsed, idx = _parse_mapping(lines, 0, root_indent)
    if idx != len(lines):
        raise SimpleYamlError(f"Line {lines[idx].line_no}: unsupported YAML structure")
    return parsed


def _dig(mapping: dict[str, Any], dotted_path: str) -> Any:
    current: Any = mapping
    for part in dotted_path.split("."):
        if not isinstance(current, dict) or part not in current:
            return None
        current = current[part]
    return current


def _is_non_empty_string(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _is_numeric(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _validate_enum(value: Any, enum_key: str, warnings: list[str], errors: list[str], strict: bool) -> None:
    if value is None:
        return
    if not _is_non_empty_string(value):
        errors.append(f"Field '{enum_key}' must be a non-empty string.")
        return
    allowed = ENUMS[enum_key]
    if value not in allowed:
        msg = f"Field '{enum_key}' uses unknown value '{value}' (allowed: {sorted(allowed)})."
        if strict:
            errors.append(msg)
        else:
            warnings.append(msg)


def _validate_common(contract: dict[str, Any], errors: list[str]) -> None:
    if not _is_non_empty_string(contract.get("schema_version")):
        errors.append("Missing or empty required field 'schema_version'.")


def validate_contract(contract: dict[str, Any], strict: bool = False) -> tuple[list[str], list[str]]:
    errors: list[str] = []
    warnings: list[str] = []

    _validate_common(contract, errors)
    schema_version = contract.get("schema_version")
    kind = SCHEMA_TYPES.get(schema_version)
    if not kind:
        errors.append(f"Unsupported schema_version '{schema_version}'.")
        return errors, warnings

    if kind == "robot":
        robot = contract.get("robot")
        if not isinstance(robot, dict):
            errors.append("Field 'robot' must be a mapping/object.")
            return errors, warnings
        for field in ["id", "label", "brand", "family", "mounting", "base_frame", "default_tool_frame"]:
            if not _is_non_empty_string(robot.get(field)):
                errors.append(f"Field 'robot.{field}' is required and must be a non-empty string.")
        for list_field in LIST_FIELDS["robot"]:
            if list_field in robot and not isinstance(robot[list_field], list):
                errors.append(f"Field 'robot.{list_field}' must be a list.")
        _validate_enum(robot.get("family"), "robot.family", warnings, errors, strict)
        _validate_enum(robot.get("mounting"), "robot.mounting", warnings, errors, strict)
        for frame_field in ["base_frame", "default_tool_frame"]:
            if frame_field in robot and not _is_non_empty_string(robot.get(frame_field)):
                errors.append(f"Field 'robot.{frame_field}' must be a non-empty string.")
        for num_field in ["payload_kg", "reach_m"]:
            if num_field in robot and not _is_numeric(robot.get(num_field)):
                errors.append(f"Field 'robot.{num_field}' must be numeric.")

    elif kind == "end_effector":
        ee = contract.get("end_effector")
        if not isinstance(ee, dict):
            errors.append("Field 'end_effector' must be a mapping/object.")
            return errors, warnings
        for field in ["id", "label", "family", "release_behaviour"]:
            if not _is_non_empty_string(ee.get(field)):
                errors.append(f"Field 'end_effector.{field}' is required and must be a non-empty string.")
        for list_field in LIST_FIELDS["end_effector"]:
            if list_field in ee and not isinstance(ee[list_field], list):
                errors.append(f"Field 'end_effector.{list_field}' must be a list.")
        _validate_enum(ee.get("family"), "end_effector.family", warnings, errors, strict)
        if "max_object_mass_kg" in ee and not _is_numeric(ee.get("max_object_mass_kg")):
            errors.append("Field 'end_effector.max_object_mass_kg' must be numeric.")

    elif kind == "sensor":
        sensor = contract.get("sensor")
        if not isinstance(sensor, dict):
            errors.append("Field 'sensor' must be a mapping/object.")
            return errors, warnings
        for field in ["id", "label", "family", "mounting"]:
            if not _is_non_empty_string(sensor.get(field)):
                errors.append(f"Field 'sensor.{field}' is required and must be a non-empty string.")
        for list_field in LIST_FIELDS["sensor"]:
            if list_field in sensor and not isinstance(sensor[list_field], list):
                errors.append(f"Field 'sensor.{list_field}' must be a list.")
        _validate_enum(sensor.get("family"), "sensor.family", warnings, errors, strict)
        _validate_enum(sensor.get("mounting"), "sensor.mounting", warnings, errors, strict)

    elif kind == "task":
        task = contract.get("task")
        if not isinstance(task, dict):
            errors.append("Field 'task' must be a mapping/object.")
            return errors, warnings
        if not _is_non_empty_string(task.get("task_family")):
            errors.append("Field 'task.task_family' is required and must be a non-empty string.")
        _validate_enum(task.get("task_family"), "task.task_family", warnings, errors, strict)
        if not _is_non_empty_string(task.get("rule_model")):
            errors.append("Field 'task.rule_model' is required and must be a non-empty string.")
        for list_field in LIST_FIELDS["task"]:
            if list_field in task and not isinstance(task[list_field], list):
                errors.append(f"Field 'task.{list_field}' must be a list.")

    elif kind == "asset":
        asset = contract.get("asset")
        if not isinstance(asset, dict):
            errors.append("Field 'asset' must be a mapping/object.")
            return errors, warnings
        for field in ["id", "label", "family", "frame"]:
            if not _is_non_empty_string(asset.get(field)):
                errors.append(f"Field 'asset.{field}' is required and must be a non-empty string.")
        _validate_enum(asset.get("family"), "asset.family", warnings, errors, strict)
        dims = asset.get("dimensions")
        if dims is not None:
            if not isinstance(dims, dict):
                errors.append("Field 'asset.dimensions' must be a mapping/object when provided.")
            else:
                for key, value in dims.items():
                    if not _is_numeric(value):
                        errors.append(f"Field 'asset.dimensions.{key}' must be numeric.")

    return errors, warnings


def load_contract(path: Path) -> tuple[dict[str, Any], list[str]]:
    notes: list[str] = []
    content = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        loaded = json.loads(content)
    elif _pyyaml is not None:
        loaded = _pyyaml.safe_load(content) or {}
    else:
        notes.append("PyYAML not available: using built-in fallback parser.")
        loaded = parse_simple_yaml(content)
    if not isinstance(loaded, dict):
        raise ValueError("Contract root must be a mapping/object.")
    return loaded, notes


def collect_files(path: Path) -> list[Path]:
    if path.is_file():
        return [path]
    files: list[Path] = []
    for candidate in sorted(path.rglob("*")):
        if candidate.is_file() and candidate.suffix.lower() in SUPPORTED_EXTENSIONS:
            files.append(candidate)
    return files


def validate_path(path: Path, strict: bool = False) -> list[FileValidationResult]:
    results: list[FileValidationResult] = []
    for file_path in collect_files(path):
        try:
            contract, notes = load_contract(file_path)
            errors, warnings = validate_contract(contract, strict=strict)
            status = "FAIL" if errors else ("WARN" if warnings else "PASS")
            results.append(
                FileValidationResult(
                    path=str(file_path), status=status, errors=errors, warnings=warnings, notes=notes
                )
            )
        except Exception as exc:
            results.append(
                FileValidationResult(path=str(file_path), status="FAIL", errors=[f"Load error: {exc}"])
            )
    if path.is_file() and not results:
        results.append(FileValidationResult(path=str(path), status="FAIL", errors=["Unsupported file extension."]))
    return results


def build_json_summary(results: list[FileValidationResult]) -> str:
    payload = {
        "summary": {
            "pass": sum(1 for r in results if r.status == "PASS"),
            "warn": sum(1 for r in results if r.status == "WARN"),
            "fail": sum(1 for r in results if r.status == "FAIL"),
            "total": len(results),
        },
        "results": [r.__dict__ for r in results],
    }
    return json.dumps(payload, indent=2, sort_keys=True)


def print_human(results: list[FileValidationResult]) -> None:
    for result in results:
        print(f"[{result.status}] {result.path}")
        for note in result.notes:
            print(f"  NOTE: {note}")
        for warning in result.warnings:
            print(f"  WARN: {warning}")
        for error in result.errors:
            print(f"  FAIL: {error}")
    pass_count = sum(1 for r in results if r.status == "PASS")
    warn_count = sum(1 for r in results if r.status == "WARN")
    fail_count = sum(1 for r in results if r.status == "FAIL")
    print(f"SUMMARY: PASS={pass_count} WARN={warn_count} FAIL={fail_count} TOTAL={len(results)}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate capability contracts (offline).")
    parser.add_argument("target", help="Contract file or directory")
    parser.add_argument("--json", action="store_true", dest="json_output", help="Emit JSON output")
    parser.add_argument("--strict", action="store_true", help="Treat unknown enum values as FAIL")
    args = parser.parse_args()

    target = Path(args.target)
    if not target.exists():
        if args.json_output:
            print(json.dumps({"summary": {"pass": 0, "warn": 0, "fail": 1, "total": 0}, "error": "Target not found"}))
        else:
            print(f"FAIL: Target does not exist: {target}")
        return 2

    results = validate_path(target, strict=args.strict)
    if args.json_output:
        print(build_json_summary(results))
    else:
        print_human(results)
    return 1 if any(r.status == "FAIL" for r in results) else 0


if __name__ == "__main__":
    raise SystemExit(main())
