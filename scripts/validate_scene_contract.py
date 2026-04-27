#!/usr/bin/env python3
"""Validate scene manifest contract for Easy Manipulation scenes."""

from __future__ import annotations

import argparse
import math
import os
import re
import subprocess
from dataclasses import dataclass, field
from typing import Any

try:  # Optional dependency.
    import yaml as _pyyaml
except Exception:  # pragma: no cover - runtime environment dependent
    _pyyaml = None

MANIFEST_CANDIDATES = ("scene_manifest.yaml", "workcell.yaml")
UNSUPPORTED_YAML_MESSAGE = (
    "Unsupported YAML syntax in manifest. Install python3-yaml or simplify the manifest."
)


class SimpleYamlError(ValueError):
    """Raised when the fallback YAML parser encounters unsupported syntax."""


@dataclass
class ValidationResult:
    package: str
    manifest_path: str | None
    parser: str
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors

    @property
    def has_warnings(self) -> bool:
        return bool(self.warnings)


@dataclass
class _Line:
    line_no: int
    indent: int
    content: str


def _ros2_pkg_prefix(package: str) -> str | None:
    try:
        completed = subprocess.run(
            ["ros2", "pkg", "prefix", package],
            check=False,
            text=True,
            capture_output=True,
            timeout=5,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None

    if completed.returncode != 0:
        return None

    prefix = completed.stdout.strip()
    return prefix or None


def _ament_index_share(package: str) -> str | None:
    try:
        from ament_index_python.packages import get_package_share_directory
    except Exception:
        return None

    try:
        return get_package_share_directory(package)
    except Exception:
        return None


def resolve_package_share(package: str) -> tuple[str | None, str]:
    """Return (share_path, resolver_name)."""
    prefix = _ros2_pkg_prefix(package)
    if prefix:
        share_dir = os.path.join(prefix, "share", package)
        if os.path.isdir(share_dir):
            return share_dir, "ros2 pkg prefix"

    share = _ament_index_share(package)
    if share and os.path.isdir(share):
        return share, "ament_index"

    return None, "unresolved"


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
        line = raw_line.rstrip("\n")
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if "\t" in line:
            raise SimpleYamlError(f"Line {idx}: tabs are not supported")
        indent = len(line) - len(line.lstrip(" "))
        content = _strip_comment(line[indent:]).strip()
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

    if (value.startswith('"') and value.endswith('"')) or (
        value.startswith("'") and value.endswith("'")
    ):
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

    if value.startswith(("{", "[")) and not value.startswith("["):
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
            raise SimpleYamlError(f"Line {line.line_no}: missing ':' in mapping entry")

        key, remainder = line.content.split(":", 1)
        key = key.strip()
        if not key:
            raise SimpleYamlError(f"Line {line.line_no}: empty mapping key")

        value_text = remainder.strip()
        if value_text:
            result[key] = _parse_scalar(value_text, line.line_no)
            i += 1
            continue

        i += 1
        if i >= len(lines) or lines[i].indent <= indent:
            result[key] = {}
            continue

        child_indent = lines[i].indent
        if lines[i].content.startswith("- "):
            parsed_list, i = _parse_list(lines, i, child_indent)
            result[key] = parsed_list
        else:
            parsed_map, i = _parse_mapping(lines, i, child_indent)
            result[key] = parsed_map

    return result, i


def _parse_list(lines: list[_Line], start: int, indent: int) -> tuple[list[Any], int]:
    items: list[Any] = []
    i = start
    while i < len(lines):
        line = lines[i]
        if line.indent < indent:
            break
        if line.indent != indent:
            raise SimpleYamlError(f"Line {line.line_no}: inconsistent list indentation")
        if not line.content.startswith("- "):
            break

        item_text = line.content[2:].strip()
        if not item_text:
            raise SimpleYamlError(f"Line {line.line_no}: empty block list entries are unsupported")
        if item_text.endswith(":") or (":" in item_text and not item_text.startswith(("'", '"'))):
            raise SimpleYamlError(
                f"Line {line.line_no}: mapping entries inside lists are unsupported by fallback parser"
            )

        items.append(_parse_scalar(item_text, line.line_no))
        i += 1

    return items, i


def parse_manifest_yaml(text: str) -> dict[str, Any]:
    lines = _tokenize_simple_yaml(text)
    if not lines:
        return {}
    root_indent = min(line.indent for line in lines)
    parsed, index = _parse_mapping(lines, 0, root_indent)
    if index != len(lines):
        extra = lines[index]
        raise SimpleYamlError(f"Line {extra.line_no}: unsupported YAML structure")
    return parsed


def _read_manifest(manifest_path: str) -> tuple[dict[str, Any], str, list[str]]:
    with open(manifest_path, "r", encoding="utf-8") as stream:
        content = stream.read()

    notes: list[str] = []
    if _pyyaml is not None:
        loaded = _pyyaml.safe_load(content) or {}
        parser_name = "pyyaml"
    else:
        notes.append("PyYAML not available: using built-in fallback parser.")
        try:
            loaded = parse_manifest_yaml(content)
        except SimpleYamlError as exc:
            raise ValueError(f"{UNSUPPORTED_YAML_MESSAGE} ({exc})") from exc
        parser_name = "fallback"

    if not isinstance(loaded, dict):
        raise ValueError("Manifest root must be a YAML mapping/object")
    return loaded, parser_name, notes


def _dig(data: dict[str, Any], dotted_path: str) -> Any:
    current: Any = data
    for part in dotted_path.split("."):
        if not isinstance(current, dict) or part not in current:
            return None
        current = current[part]
    return current


def _is_non_empty(value: Any) -> bool:
    if value is None:
        return False
    if isinstance(value, str):
        return bool(value.strip())
    if isinstance(value, (list, tuple, dict, set)):
        return len(value) > 0
    return True


def _check_required(manifest: dict[str, Any], errors: list[str]) -> None:
    required_fields = [
        "robot.model",
        "robot.planning_group",
        "robot.base_frame",
        "robot.ee_link",
        "end_effector.type",
        "end_effector.brand",
        "end_effector.grasp_frame",
        "end_effector.allowed_touch_links",
        "environment.support_surface_link",
    ]

    for field in required_fields:
        value = _dig(manifest, field)
        if not _is_non_empty(value):
            errors.append(
                f"Missing/empty required field '{field}'. "
                f"Add '{field}' with a non-empty value in the scene manifest."
            )

    home_named = _dig(manifest, "robot.home_named_target")
    safe_state = _dig(manifest, "home_return.safe_joint_state")
    if not _is_non_empty(home_named) and not _is_non_empty(safe_state):
        errors.append(
            "Missing home return definition. Provide either "
            "'robot.home_named_target' or 'home_return.safe_joint_state'."
        )


def _check_types(manifest: dict[str, Any], errors: list[str], warnings: list[str], package: str) -> None:
    string_fields = [
        "robot.planning_group",
        "robot.base_frame",
        "robot.ee_link",
        "end_effector.grasp_frame",
        "environment.support_surface_link",
    ]
    for field in string_fields:
        value = _dig(manifest, field)
        if value is not None and not isinstance(value, str):
            errors.append(f"Field '{field}' must be a string (got {type(value).__name__}).")

    scene_name = _dig(manifest, "scene.name")
    if scene_name is not None:
        if not isinstance(scene_name, str) or not scene_name.strip():
            warnings.append("Field 'scene.name' is present but empty/non-string.")
        elif scene_name.strip() != package:
            warnings.append(
                f"Field 'scene.name' ('{scene_name.strip()}') does not match package name '{package}'."
            )

    touch_links = _dig(manifest, "end_effector.allowed_touch_links")
    if touch_links is not None:
        if not isinstance(touch_links, list):
            errors.append(
                "Field 'end_effector.allowed_touch_links' must be a YAML list of strings."
            )
        else:
            if not touch_links:
                errors.append("Field 'end_effector.allowed_touch_links' must not be empty.")
            bad_items = [item for item in touch_links if not isinstance(item, str) or not item.strip()]
            if bad_items:
                errors.append(
                    "Field 'end_effector.allowed_touch_links' must contain only non-empty strings."
                )

    input_frames = _dig(manifest, "perception.input_frame_options")
    if input_frames is not None:
        if not isinstance(input_frames, list):
            errors.append("Field 'perception.input_frame_options' must be a YAML list.")

    safe_state = _dig(manifest, "home_return.safe_joint_state")
    if safe_state is not None:
        if not isinstance(safe_state, list):
            errors.append("Field 'home_return.safe_joint_state' must be a YAML list of numbers.")
            return
        if not safe_state:
            errors.append("Field 'home_return.safe_joint_state' must be non-empty when provided.")
            return
        non_numeric = [v for v in safe_state if not isinstance(v, (int, float))]
        non_finite = [v for v in safe_state if isinstance(v, float) and not math.isfinite(v)]
        if non_numeric:
            errors.append(
                "Field 'home_return.safe_joint_state' must contain only numeric entries (int/float)."
            )
        if non_finite:
            errors.append(
                "Field 'home_return.safe_joint_state' cannot contain NaN/Infinity values."
            )


def _resolve_declared_path(share_dir: str, path_value: str) -> str:
    candidate = path_value.strip()
    if os.path.isabs(candidate):
        return candidate
    return os.path.join(share_dir, candidate)


def _check_declared_files(manifest: dict[str, Any], share_dir: str, errors: list[str]) -> None:
    declared_paths = {
        "URDF": ["robot.urdf", "robot.urdf_path", "scene.urdf", "scene.urdf_path"],
        "SRDF": ["robot.srdf", "robot.srdf_path", "scene.srdf", "scene.srdf_path"],
        "launch": ["launch.file", "launch.path", "scene.launch_file", "launch_file"],
    }

    for label, paths in declared_paths.items():
        for dotted in paths:
            value = _dig(manifest, dotted)
            if value is None:
                continue
            if not isinstance(value, str) or not value.strip():
                errors.append(f"Declared {label} path '{dotted}' must be a non-empty string.")
                continue
            resolved = _resolve_declared_path(share_dir, value)
            if not os.path.isfile(resolved):
                errors.append(
                    f"Declared {label} path '{dotted}' does not exist: {resolved}"
                )


def validate_scene_contract(package: str) -> tuple[ValidationResult, int]:
    share_dir, resolver = resolve_package_share(package)
    if not share_dir:
        result = ValidationResult(
            package=package,
            manifest_path=None,
            parser="n/a",
            errors=[
                "Scene package is not discoverable via 'ros2 pkg prefix' or ament index. "
                "Source /opt/ros/humble/setup.bash and your workspace install/setup.bash first."
            ],
        )
        return result, 3

    if not os.path.isdir(share_dir):
        result = ValidationResult(
            package=package,
            manifest_path=None,
            parser="n/a",
            errors=[f"Resolved package share directory does not exist: {share_dir}"],
        )
        return result, 1

    manifest_path = None
    for candidate in MANIFEST_CANDIDATES:
        candidate_path = os.path.join(share_dir, candidate)
        if os.path.isfile(candidate_path):
            manifest_path = candidate_path
            break

    if manifest_path is None:
        result = ValidationResult(
            package=package,
            manifest_path=None,
            parser="n/a",
            errors=[
                "Scene manifest not found. Add one of: "
                f"{', '.join(MANIFEST_CANDIDATES)} in package share ({share_dir})."
            ],
        )
        return result, 1

    errors: list[str] = []
    warnings: list[str] = []
    notes: list[str] = [f"Package share resolved by: {resolver}"]
    try:
        manifest, parser_name, parser_notes = _read_manifest(manifest_path)
        notes.extend(parser_notes)
    except Exception as exc:
        result = ValidationResult(
            package=package,
            manifest_path=manifest_path,
            parser="fallback" if _pyyaml is None else "pyyaml",
            errors=[f"Failed to parse manifest YAML: {exc}"],
        )
        return result, 1

    _check_required(manifest, errors)
    _check_types(manifest, errors, warnings, package)
    _check_declared_files(manifest, share_dir, errors)

    result = ValidationResult(
        package=package,
        manifest_path=manifest_path,
        parser=parser_name,
        errors=errors,
        warnings=warnings,
        notes=notes,
    )
    return result, 0 if result.ok else 1


def _print_report(result: ValidationResult, exit_code: int, resolver_hint: str = "") -> None:
    print(f"Scene package: {result.package}")
    if resolver_hint:
        print(f"Resolver: {resolver_hint}")
    print(f"Parser: {result.parser}")
    print(f"Manifest: {result.manifest_path or 'not found'}")

    for note in result.notes:
        print(f"NOTE: {note}")

    if result.ok and result.has_warnings:
        print("RESULT: WARN")
    elif result.ok:
        print("RESULT: PASS")
    elif exit_code == 3:
        print("RESULT: SKIP")
    else:
        print("RESULT: FAIL")

    for index, warning in enumerate(result.warnings, start=1):
        print(f"  W{index}. {warning}")
    for index, error in enumerate(result.errors, start=1):
        print(f"  E{index}. {error}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate scene contract manifest for a scene package")
    parser.add_argument("scene_package", help="ROS package name of the scene (e.g. ur5_2f_test)")
    args = parser.parse_args()

    _, resolver = resolve_package_share(args.scene_package)
    result, exit_code = validate_scene_contract(args.scene_package)
    _print_report(result, exit_code, resolver)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
