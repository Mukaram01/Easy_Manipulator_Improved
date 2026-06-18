#!/usr/bin/env python3
"""Validate UR5 visual transform plausibility from a generated Scene3D mesh index.

This is an offline validator: it only reads scene_visual_mesh_index.json and does
not require ROS, RViz, a GUI, screenshots, launch files, or EPD.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

BASE_ALIASES = ("base_link_inertia", "base", "base_link")
REQUIRED_CHAIN = (
    "base_link",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
)
LINK_ALIASES: dict[str, tuple[str, ...]] = {
    "base_link": BASE_ALIASES,
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
VISUAL_ADJACENT_MIN_M = 0.025
TOOL_ATTACHMENT_MAX_M = 0.35
ADJACENT_LIMITS_M: dict[tuple[str, str], float] = {
    ("base_link", "shoulder_link"): 0.25,
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


def _pose_xyz_from_keys(
    record: dict[str, Any],
    pose_keys: tuple[str, ...],
    scalar_keys: tuple[str, ...] = (),
) -> list[float] | None:
    for pose_key in pose_keys:
        pose = record.get(pose_key)
        if isinstance(pose, dict):
            xyz = _as_xyz(pose.get("xyz"))
            if xyz is not None:
                return xyz
    for key in scalar_keys:
        xyz = _as_xyz(record.get(key))
        if xyz is not None:
            return xyz
    return None


def _link_frame_xyz(record: dict[str, Any]) -> list[float] | None:
    return _pose_xyz_from_keys(
        record, ("link_world_pose", "world_pose"), ("world_xyz", "xyz", "translation")
    )


def _visual_pose_xyz(record: dict[str, Any]) -> list[float] | None:
    # Renderer adjacency must use the rendered visual center, not only the
    # physical link frame.  baked_world_visual_pose is emitted by newer indexes;
    # pose is the legacy rendered pose with visual_origin already applied.
    return _pose_xyz_from_keys(
        record, ("baked_world_visual_pose", "pose"), ("world_xyz", "xyz", "translation")
    )


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


def _locate_link_poses(
    data: dict[str, Any], *, visual: bool = False
) -> tuple[dict[str, list[float]], list[str]]:
    records = _diagnostic_records(data)
    records.extend(x for x in data.get("visual_items", []) if isinstance(x, dict))
    poses: dict[str, list[float]] = {}
    sources: list[str] = []
    for canonical, aliases in LINK_ALIASES.items():
        for record in records:
            link = _record_link(record)
            if link not in aliases:
                continue
            xyz = _visual_pose_xyz(record) if visual else _link_frame_xyz(record)
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
        return {
            "ok": False,
            "index": str(index_path),
            "checks": [],
            "errors": [f"Failed to load JSON index: {exc}"],
        }

    index_data = data if isinstance(data, dict) else {}
    poses, sources = _locate_link_poses(index_data, visual=False)
    visual_poses, visual_sources = _locate_link_poses(index_data, visual=True)
    for link in REQUIRED_CHAIN:
        ok = link in poses
        checks.append({"check": f"{link} pose present and finite", "ok": ok})
        if not ok:
            aliases = ", ".join(LINK_ALIASES[link])
            errors.append(
                f"Missing finite UR5 link pose for {link} (accepted names: {aliases})"
            )

    distances: dict[str, float] = {}
    if "base_link" in poses:
        base_distance = _distance(poses["base_link"], [0.0, 0.0, 0.0])
        distances["base_to_origin"] = base_distance
        ok = base_distance <= BASE_ORIGIN_MAX_M
        checks.append(
            {
                "check": "base near world origin",
                "ok": ok,
                "distance_m": base_distance,
                "limit_m": BASE_ORIGIN_MAX_M,
            }
        )
        if not ok:
            errors.append(
                f"UR5 base is {base_distance:.3f} m from world origin; expected <= {BASE_ORIGIN_MAX_M:.3f} m"
            )

    adjacent_values: list[float] = []
    for parent, child in zip(REQUIRED_CHAIN, REQUIRED_CHAIN[1:]):
        if parent not in poses or child not in poses:
            continue
        distance = _distance(poses[parent], poses[child])
        limit = ADJACENT_LIMITS_M[(parent, child)]
        distances[f"{parent}_to_{child}"] = distance
        adjacent_values.append(distance)
        ok = distance <= limit
        checks.append(
            {
                "check": f"physical link frame {child} attached near {parent}",
                "ok": ok,
                "distance_m": distance,
                "limit_m": limit,
            }
        )
        if not ok:
            errors.append(
                f"UR5 physical link-frame distance {parent}->{child} is {distance:.3f} m; expected <= {limit:.3f} m"
            )

    visual_distances: dict[str, float] = {}
    for link in REQUIRED_CHAIN:
        ok = link in visual_poses
        checks.append(
            {"check": f"{link} rendered visual pose present and finite", "ok": ok}
        )
        if not ok:
            aliases = ", ".join(LINK_ALIASES[link])
            errors.append(
                f"Missing finite UR5 rendered visual pose for {link} (accepted names: {aliases})"
            )

    for parent, child in zip(REQUIRED_CHAIN, REQUIRED_CHAIN[1:]):
        if parent not in visual_poses or child not in visual_poses:
            continue
        distance = _distance(visual_poses[parent], visual_poses[child])
        limit = ADJACENT_LIMITS_M[(parent, child)]
        key = f"visual_{parent}_to_{child}"
        visual_distances[key] = distance
        distances[key] = distance
        max_ok = distance <= limit
        min_ok = distance >= VISUAL_ADJACENT_MIN_M
        checks.append(
            {
                "check": f"rendered visual {child} near {parent}",
                "ok": max_ok,
                "distance_m": distance,
                "limit_m": limit,
            }
        )
        checks.append(
            {
                "check": f"rendered visual {child} not collapsed onto {parent}",
                "ok": min_ok,
                "distance_m": distance,
                "limit_m": VISUAL_ADJACENT_MIN_M,
            }
        )
        if not max_ok:
            errors.append(
                f"UR5 rendered visual center distance {parent}->{child} is {distance:.3f} m; expected <= {limit:.3f} m"
            )
        if not min_ok:
            errors.append(
                f"UR5 rendered visual centers {parent}->{child} are collapsed at {distance:.3f} m; expected >= {VISUAL_ADJACENT_MIN_M:.3f} m"
            )

    tool_anchor = poses.get("tool0") or poses.get("wrist_3_link")
    tool_records = [
        record
        for record in index_data.get("visual_items", [])
        if isinstance(record, dict)
        and any(
            token
            in " ".join(
                str(record.get(k, "")).lower()
                for k in (
                    "link",
                    "id",
                    "source_path",
                    "package_uri",
                    "resolved_source_path",
                )
            )
            for token in ("robotiq", "gripper", "tool")
        )
    ]
    checks.append(
        {
            "check": "Robotiq/tool visuals present",
            "ok": bool(tool_records),
            "count": len(tool_records),
        }
    )
    if not tool_records:
        errors.append("Missing Robotiq/tool visuals near UR5 wrist/tool attachment")
    if tool_anchor is not None and tool_records:
        tool_distances = [
            _distance(tool_anchor, xyz)
            for record in tool_records
            if (xyz := _visual_pose_xyz(record)) is not None
        ]
        if not tool_distances:
            checks.append({"check": "Robotiq/tool visual poses finite", "ok": False})
            errors.append(
                "Robotiq/tool visuals were found but none had finite rendered visual poses"
            )
        else:
            nearest = min(tool_distances)
            farthest = max(tool_distances)
            distances["nearest_tool_visual_to_wrist_3_link_or_tool0"] = nearest
            distances["farthest_tool_visual_to_wrist_3_link_or_tool0"] = farthest
            ok = farthest <= TOOL_ATTACHMENT_MAX_M
            checks.append(
                {
                    "check": "Robotiq/tool visuals attached near wrist_3_link/tool0",
                    "ok": ok,
                    "nearest_distance_m": nearest,
                    "farthest_distance_m": farthest,
                    "limit_m": TOOL_ATTACHMENT_MAX_M,
                }
            )
            if not ok:
                errors.append(
                    f"Robotiq/tool visual is {farthest:.3f} m from wrist_3_link/tool0; expected all <= {TOOL_ATTACHMENT_MAX_M:.3f} m"
                )

    max_adjacent = max(adjacent_values) if adjacent_values else None
    if max_adjacent is not None:
        ok = max_adjacent <= ADJACENT_MAX_M
        checks.append(
            {
                "check": "maximum adjacent UR5 link distance reasonable",
                "ok": ok,
                "distance_m": max_adjacent,
                "limit_m": ADJACENT_MAX_M,
            }
        )
        if not ok:
            errors.append(
                f"Maximum adjacent UR5 link distance is {max_adjacent:.3f} m; expected <= {ADJACENT_MAX_M:.3f} m"
            )

    return {
        "ok": not errors,
        "index": str(index_path),
        "required_chain": list(REQUIRED_CHAIN),
        "pose_sources": sources,
        "visual_pose_sources": visual_sources,
        "poses": poses,
        "visual_poses": visual_poses,
        "distances_m": distances,
        "checks": checks,
        "errors": errors,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--index",
        required=True,
        type=Path,
        help="Path to generated/scene_visual_mesh_index.json",
    )
    parser.add_argument(
        "--json", action="store_true", help="Print machine-readable JSON output"
    )
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
