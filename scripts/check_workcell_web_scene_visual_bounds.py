#!/usr/bin/env python3
"""Validate Workcell Studio web scene visual-bounds metadata.

This checker is intentionally dependency-free: it reads a staged/exported web scene
JSON file and verifies the metadata used by the browser viewer to frame physical
scene content without requiring ROS, a browser, or network access.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

Json = dict[str, Any]

DEFAULT_WEB_SCENE = Path("build/workcell_studio_web_scene/ur5_2f_test.web_scene.json")
ITEM_SECTIONS = ("robots", "tools", "assets", "sensors", "zones")
CORE_MESH_CATEGORIES = {"robot_link", "gripper", "tool", "table", "camera", "object"}
ROBOT_OR_TOOL_CATEGORIES = {"robot_link", "gripper", "tool"}
MAX_OBVIOUS_DIMENSION_M = 100.0
MIN_DIMENSION_M = 1e-9
DIMENSION_KEYS = (
    "expected_dimensions_m",
    "dimensions_m",
    "dimensions",
    "size_m",
    "size",
    "scale_m",
)
PASS_STATUSES = {"pass", "passed"}


def _load_json(path: Path) -> Json:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ValueError(f"web scene JSON does not exist: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid JSON in {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"web scene JSON root must be an object: {path}")
    return payload


def _iter_items(payload: Mapping[str, Any]) -> Iterable[tuple[str, Json]]:
    for section in ITEM_SECTIONS:
        values = payload.get(section) or []
        if not isinstance(values, list):
            continue
        for item in values:
            if isinstance(item, dict):
                yield section, item


def _text_for(item: Mapping[str, Any], *keys: str) -> str:
    parts: list[str] = []
    for key in keys:
        value = item.get(key)
        if isinstance(value, (str, int, float, bool)):
            parts.append(str(value))
    return " ".join(parts).lower()


def _item_label(section: str, item: Mapping[str, Any]) -> str:
    item_id = item.get("id") or item.get("display_name") or item.get("name") or "<unknown>"
    category = item.get("mesh_contract_category") or item.get("core_mesh_category") or item.get("category") or item.get("role")
    return f"{section}:{item_id}" + (f" ({category})" if category else "")


def _category(section: str, item: Mapping[str, Any]) -> str:
    raw = item.get("mesh_contract_category") or item.get("core_mesh_category")
    if isinstance(raw, str) and raw.strip():
        return raw.strip().lower()
    text = _text_for(item, "id", "name", "display_name", "role", "category", "source_layer", "active_visual_source")
    if section == "robots" or "robot" in text or "ur5" in text or "ur10" in text or "ur3" in text:
        return "robot_link"
    if section == "tools" or "gripper" in text or "tool" in text or "robotiq" in text:
        return "gripper"
    if section == "sensors" or "camera" in text or "realsense" in text:
        return "camera"
    if "table" in text or "workbench" in text or "support_surface" in text:
        return "table"
    return str(item.get("category") or item.get("role") or "object").lower()


def _is_table(item: Mapping[str, Any], section: str) -> bool:
    return _category(section, item) == "table" or "table" in _text_for(item, "id", "name", "display_name", "role", "category") or "workbench" in _text_for(item, "id", "name", "display_name", "role", "category")


def _is_camera(item: Mapping[str, Any], section: str) -> bool:
    text = _text_for(item, "id", "name", "display_name", "role", "category", "model")
    return _category(section, item) == "camera" or "camera" in text or "realsense" in text or "real sense" in text


def _as_number_list(value: Any) -> list[float] | None:
    if isinstance(value, Mapping):
        candidates = [value.get(k) for k in ("x", "y", "z", "width", "depth", "height")]
        if all(v is not None for v in candidates[:3]):
            value = candidates[:3]
        elif all(v is not None for v in candidates[3:]):
            value = candidates[3:]
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        return None
    if len(value) != 3:
        return None
    try:
        return [float(v) for v in value]
    except (TypeError, ValueError):
        return None


def _dimension_errors(label: str, key: str, value: Any) -> list[str]:
    dims = _as_number_list(value)
    if dims is None:
        return [f"{label} has invalid {key}; expected a 3-number dimension vector"]
    errors: list[str] = []
    for index, dim in enumerate(dims):
        axis = "xyz"[index]
        if not math.isfinite(dim):
            errors.append(f"{label} has non-finite {key}.{axis}: {dim}")
        elif dim <= MIN_DIMENSION_M:
            errors.append(f"{label} has non-positive {key}.{axis}: {dim}")
        elif dim > MAX_OBVIOUS_DIMENSION_M:
            errors.append(f"{label} has obviously impossible {key}.{axis}: {dim} m")
    return errors


def check(payload: Mapping[str, Any]) -> tuple[Json, list[str]]:
    errors: list[str] = []
    items = list(_iter_items(payload))
    table_items: list[str] = []
    camera_items: list[str] = []
    core_mesh_items = 0

    metadata = payload.get("metadata")
    visual_contract = metadata.get("visual_bounds_contract") if isinstance(metadata, Mapping) else None
    if isinstance(visual_contract, Mapping):
        status = visual_contract.get("status")
        if status is None:
            errors.append("metadata.visual_bounds_contract.status is absent even though visual_bounds_contract is present")
        elif str(status).strip().lower() not in PASS_STATUSES:
            errors.append(f"metadata.visual_bounds_contract.status must be pass or passed, got {status!r}")
    # If the entire visual_bounds_contract object is absent, treat the file as a
    # pre-viewer/runtime-validation export and do not fail solely for that reason.

    for section, item in items:
        label = _item_label(section, item)
        category = _category(section, item)

        if _is_table(item, section):
            table_items.append(label)
            if "expected_dimensions_m" not in item:
                errors.append(f"{label} is a table/workbench item but lacks expected_dimensions_m")
        if _is_camera(item, section):
            camera_items.append(label)
            if "expected_dimensions_m" not in item:
                errors.append(f"{label} is a camera/Realsense item but lacks expected_dimensions_m")

        is_core_mesh = item.get("mesh_load_required") is True or category in CORE_MESH_CATEGORIES or bool(item.get("mesh_url") or item.get("mesh_uri") or item.get("mesh_staged_path"))
        if is_core_mesh:
            core_mesh_items += 1
            if not isinstance(item.get("mesh_contract_category"), str) or not item.get("mesh_contract_category", "").strip():
                errors.append(f"{label} is a required/core mesh item but lacks mesh_contract_category")

        for key in DIMENSION_KEYS:
            if key in item:
                errors.extend(_dimension_errors(label, key, item.get(key)))

        is_robot_or_tool = section in {"robots", "tools"} or category in ROBOT_OR_TOOL_CATEGORIES
        if is_robot_or_tool:
            if "mesh_unit_correction" in item:
                errors.append(f"{label} is a robot/gripper/tool link but contains mesh_unit_correction")
            if category == "robot_link" and item.get("allow_mesh_unit_autoscale") is True:
                errors.append(f"{label} is a robot item but sets allow_mesh_unit_autoscale: true")

    if not table_items:
        errors.append("no table/workbench item found in web scene")
    if not camera_items:
        errors.append("no camera/Realsense item found in web scene")

    summary: Json = {
        "contract_status": "passed" if not errors else "failed",
        "item_count": len(items),
        "table_item_count": len(table_items),
        "camera_item_count": len(camera_items),
        "core_mesh_item_count": core_mesh_items,
        "violation_count": len(errors),
        "violations": errors,
    }
    return summary, errors


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check Workcell Studio web scene visual-bounds contract metadata.")
    parser.add_argument("web_scene", nargs="?", type=Path, default=DEFAULT_WEB_SCENE, help=f"Web scene JSON path (default: {DEFAULT_WEB_SCENE})")
    parser.add_argument("--json", action="store_true", help="Print the concise summary as JSON instead of text.")
    args = parser.parse_args(argv)

    try:
        payload = _load_json(args.web_scene)
        summary, errors = check(payload)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=True))
    else:
        print(f"web_scene: {args.web_scene}")
        print(f"items: {summary['item_count']}")
        print(f"table_items: {summary['table_item_count']}")
        print(f"camera_items: {summary['camera_item_count']}")
        print(f"core_mesh_items: {summary['core_mesh_item_count']}")
        print(f"violations: {summary['violation_count']}")
        for violation in errors:
            print(f"  - {violation}")
        print(f"contract_status: {summary['contract_status']}")

    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
