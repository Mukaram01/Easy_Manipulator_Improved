#!/usr/bin/env python3
"""Validate UR5 visual transform plausibility from a generated Scene3D mesh index.

This is an offline validator: it only reads scene_visual_mesh_index.json and does
not require ROS, RViz, a GUI, screenshots, launch files, or EPD.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

BASE_ALIASES = ("base_link_inertia", "base", "base_link")
REQUIRED_CHAIN = (
    "base",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
)
LINK_ALIASES: dict[str, tuple[str, ...]] = {
    "base": BASE_ALIASES,
    "shoulder_link": ("shoulder_link",),
    "upper_arm_link": ("upper_arm_link",),
    "forearm_link": ("forearm_link",),
    "wrist_1_link": ("wrist_1_link",),
    "wrist_2_link": ("wrist_2_link",),
    "wrist_3_link": ("wrist_3_link",),
}

# Conservative UR5 plausibility thresholds in metres. They are intentionally
# wider than exact UR5 dimensions so this catches transform collapse/explosion
# without rejecting harmless mesh-origin differences.
BASE_ORIGIN_MAX_M = 0.25
ADJACENT_MAX_M = 0.70
ADJACENT_LIMITS_M: dict[tuple[str, str], float] = {
    ("base", "shoulder_link"): 0.25,
    ("shoulder_link", "upper_arm_link"): 0.35,
    ("upper_arm_link", "forearm_link"): 0.55,
    ("forearm_link", "wrist_1_link"): 0.55,
    ("wrist_1_link", "wrist_2_link"): 0.25,
    ("wrist_2_link", "wrist_3_link"): 0.25,
}


def _as_xyz(value: Any) -> list[float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    xyz: list[float] = []
    for component in value:
        if not isinstance(component, (int, float)):
            return None
        f = float(component)
        if not math.isfinite(f):
            return None
        xyz.append(f)
    return xyz


def _pose_xyz(record: dict[str, Any]) -> list[float] | None:
    pose = record.get("pose")
    if isinstance(pose, dict):
        xyz = _as_xyz(pose.get("xyz"))
        if xyz is not None:
            return xyz
    for key in ("xyz", "world_xyz", "translation"):
        xyz = _as_xyz(record.get(key))
        if xyz is not None:
            return xyz
    return None


def _diagnostic_records(data: dict[str, Any]) -> list[dict[str, Any]]:
    diagnostics = data.get("ur5_transform_diagnostics")
    if isinstance(diagnostics, list):
        return [x for x in diagnostics if isinstance(x, dict)]
    if isinstance(diagnostics, dict):
        records: list[dict[str, Any]] = []
        for key, value in diagnostics.items():
            if isinstance(value, dict):
                record = dict(value)
                record.setdefault("link", key)
                records.append(record)
        return records
    return []


def _record_link(record: dict[str, Any]) -> str:
    for key in ("link", "link_name", "name", "id"):
        value = record.get(key)
        if isinstance(value, str):
            return value
    return ""


def _locate_link_poses(data: dict[str, Any]) -> tuple[dict[str, list[float]], list[str]]:
    records = _diagnostic_records(data)
    records.extend(x for x in data.get("visual_items", []) if isinstance(x, dict))
    poses: dict[str, list[float]] = {}
    sources: list[str] = []
    for canonical, aliases in LINK_ALIASES.items():
        for record in records:
            link = _record_link(record)
            if link not in aliases:
                continue
            xyz = _pose_xyz(record)
            if xyz is None:
                # Preserve the missing/non-finite failure for this canonical link.
                continue
            poses[canonical] = xyz
            sources.append(f"{canonical}:{link}")
            break
    return poses, sources


def _distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def validate_index(index_path: Path) -> dict[str, Any]:
    errors: list[str] = []
    checks: list[dict[str, Any]] = []
    try:
        data = json.loads(index_path.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001 - CLI should report parse/path failures clearly.
        return {"ok": False, "index": str(index_path), "checks": [], "errors": [f"Failed to load JSON index: {exc}"]}

    poses, sources = _locate_link_poses(data if isinstance(data, dict) else {})
    for link in REQUIRED_CHAIN:
        ok = link in poses
        checks.append({"check": f"{link} pose present and finite", "ok": ok})
        if not ok:
            aliases = ", ".join(LINK_ALIASES[link])
            errors.append(f"Missing finite UR5 link pose for {link} (accepted names: {aliases})")

    distances: dict[str, float] = {}
    if "base" in poses:
        base_distance = _distance(poses["base"], [0.0, 0.0, 0.0])
        distances["base_to_origin"] = base_distance
        ok = base_distance <= BASE_ORIGIN_MAX_M
        checks.append({"check": "base near world origin", "ok": ok, "distance_m": base_distance, "limit_m": BASE_ORIGIN_MAX_M})
        if not ok:
            errors.append(f"UR5 base is {base_distance:.3f} m from world origin; expected <= {BASE_ORIGIN_MAX_M:.3f} m")

    adjacent_values: list[float] = []
    for parent, child in zip(REQUIRED_CHAIN, REQUIRED_CHAIN[1:]):
        if parent not in poses or child not in poses:
            continue
        distance = _distance(poses[parent], poses[child])
        limit = ADJACENT_LIMITS_M[(parent, child)]
        distances[f"{parent}_to_{child}"] = distance
        adjacent_values.append(distance)
        ok = distance <= limit
        checks.append({"check": f"{child} attached near {parent}", "ok": ok, "distance_m": distance, "limit_m": limit})
        if not ok:
            errors.append(f"UR5 adjacent link distance {parent}->{child} is {distance:.3f} m; expected <= {limit:.3f} m")

    max_adjacent = max(adjacent_values) if adjacent_values else None
    if max_adjacent is not None:
        ok = max_adjacent <= ADJACENT_MAX_M
        checks.append({"check": "maximum adjacent UR5 link distance reasonable", "ok": ok, "distance_m": max_adjacent, "limit_m": ADJACENT_MAX_M})
        if not ok:
            errors.append(f"Maximum adjacent UR5 link distance is {max_adjacent:.3f} m; expected <= {ADJACENT_MAX_M:.3f} m")

    return {
        "ok": not errors,
        "index": str(index_path),
        "required_chain": list(REQUIRED_CHAIN),
        "pose_sources": sources,
        "poses": poses,
        "distances_m": distances,
        "checks": checks,
        "errors": errors,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--index", required=True, type=Path, help="Path to generated/scene_visual_mesh_index.json")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON output")
    args = parser.parse_args(argv)

    report = validate_index(args.index)
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        status = "PASS" if report.get("ok") else "FAIL"
        print(f"UR5 Scene3D transform plausibility: {status}")
        for error in report.get("errors", []):
            print(f"- {error}")
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
