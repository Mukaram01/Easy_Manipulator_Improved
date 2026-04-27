#!/usr/bin/env python3
"""Validate scene manifest contract for Easy Manipulation scenes."""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
from dataclasses import dataclass
from typing import Any

try:
    import yaml
except Exception as exc:  # pragma: no cover - handled at runtime
    print(
        "ERROR: Missing PyYAML. Install python3-yaml or source a ROS environment that provides it.",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc


MANIFEST_CANDIDATES = ("scene_manifest.yaml", "workcell.yaml")


@dataclass
class ValidationResult:
    package: str
    manifest_path: str | None
    errors: list[str]

    @property
    def ok(self) -> bool:
        return not self.errors


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


def _read_manifest(manifest_path: str) -> dict[str, Any]:
    with open(manifest_path, "r", encoding="utf-8") as stream:
        loaded = yaml.safe_load(stream) or {}
    if not isinstance(loaded, dict):
        raise ValueError("Manifest root must be a YAML mapping/object")
    return loaded


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
        "scene.name",
        "robot.model",
        "robot.planning_group",
        "robot.base_frame",
        "robot.ee_link",
        "end_effector.type",
        "end_effector.brand",
        "end_effector.grasp_frame",
        "end_effector.allowed_touch_links",
        "environment.support_surface_link",
        "perception.input_frame_options",
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


def _check_types(manifest: dict[str, Any], errors: list[str]) -> None:
    string_fields = [
        "robot.ee_link",
        "end_effector.grasp_frame",
        "environment.support_surface_link",
    ]
    for field in string_fields:
        value = _dig(manifest, field)
        if value is not None and not isinstance(value, str):
            errors.append(f"Field '{field}' must be a string (got {type(value).__name__}).")

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
            errors.append("Field 'perception.input_frame_options' must be a YAML list of strings.")
        elif not input_frames:
            errors.append("Field 'perception.input_frame_options' must not be empty.")

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


def validate_scene_contract(package: str) -> tuple[ValidationResult, int]:
    share_dir, resolver = resolve_package_share(package)
    if not share_dir:
        result = ValidationResult(
            package=package,
            manifest_path=None,
            errors=[
                "Scene package is not discoverable via 'ros2 pkg prefix' or ament index. "
                "Source /opt/ros/humble/setup.bash and your workspace install/setup.bash first."
            ],
        )
        return result, 3

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
            errors=[
                "Scene manifest not found. Add one of: "
                f"{', '.join(MANIFEST_CANDIDATES)} in package share ({share_dir})."
            ],
        )
        return result, 1

    errors: list[str] = []
    try:
        manifest = _read_manifest(manifest_path)
    except Exception as exc:
        result = ValidationResult(
            package=package,
            manifest_path=manifest_path,
            errors=[f"Failed to parse manifest YAML: {exc}"],
        )
        return result, 1

    _check_required(manifest, errors)
    _check_types(manifest, errors)

    result = ValidationResult(package=package, manifest_path=manifest_path, errors=errors)
    return result, 0 if result.ok else 1


def _print_report(result: ValidationResult, exit_code: int, resolver_hint: str = "") -> None:
    print(f"Scene package: {result.package}")
    if resolver_hint:
        print(f"Resolver: {resolver_hint}")
    print(f"Manifest: {result.manifest_path or 'not found'}")

    if result.ok:
        print("RESULT: PASS")
        return

    if exit_code == 3:
        print("RESULT: SKIP")
    else:
        print("RESULT: FAIL")
    for index, error in enumerate(result.errors, start=1):
        print(f"  {index}. {error}")


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
