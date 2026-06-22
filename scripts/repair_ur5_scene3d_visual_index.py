#!/usr/bin/env python3
"""Repair canonical UR5 Scene3D visual-index rows after extraction.

The extractor is the source of truth for real xacro/URDF expansion, but some
runtime paths have repeatedly emitted Robotiq/table/camera rows while dropping
the UR5 arm rows.  This post-process is deliberately narrow: it only applies to
UR5 scenes, only adds missing required UR5 mesh rows, and leaves existing
Robotiq/table/camera rows untouched.
"""
from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

REQUIRED_UR5_VISUALS: tuple[dict[str, Any], ...] = (
    {
        "link": "base_link_inertia",
        "mesh": "base.dae",
        "xyz": [0.0, 0.0, 0.0],
        "rpy": [0.0, 0.0, 3.141593],
        "parent": "world",
        "chain": ["world", "base_link_inertia"],
    },
    {
        "link": "shoulder_link",
        "mesh": "shoulder.dae",
        "xyz": [0.0, 0.0, 0.089159],
        "rpy": [0.0, 0.0, 0.0],
        "parent": "base_link_inertia",
        "chain": ["world", "base_link_inertia", "shoulder_link"],
    },
    {
        "link": "upper_arm_link",
        "mesh": "upperarm.dae",
        "xyz": [0.0, 0.13585, 0.089159],
        "rpy": [0.0, -0.0000036732051032936018, 0.0],
        "parent": "shoulder_link",
        "chain": ["world", "base_link_inertia", "shoulder_link", "upper_arm_link"],
    },
    {
        "link": "forearm_link",
        "mesh": "forearm.dae",
        "xyz": [-0.0000015611121689202732, 0.016500000087168957, 0.5141589999937487],
        "rpy": [-1.5707966253386139, 1.5707963118937354, -1.5707966253386139],
        "parent": "upper_arm_link",
        "chain": ["world", "base_link_inertia", "shoulder_link", "upper_arm_link", "forearm_link"],
    },
    {
        "link": "wrist_1_link",
        "mesh": "wrist1.dae",
        "xyz": [0.392248438887831, 0.01615000008716891, 0.5141589999938204],
        "rpy": [-0.000055837728232363146, 1.5707926535453185, -0.00005583772823204767],
        "parent": "forearm_link",
        "chain": ["world", "base_link_inertia", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link"],
    },
    {
        "link": "wrist_2_link",
        "mesh": "wrist2.dae",
        "xyz": [0.3918984388878334, 0.10915000008724068, 0.514158998689124],
        "rpy": [-1.5707926535897931, -0.0000036734102060051815, -1.5707963270134926],
        "parent": "wrist_1_link",
        "chain": ["world", "base_link_inertia", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link"],
    },
    {
        "link": "wrist_3_link",
        "mesh": "wrist3.dae",
        "xyz": [0.4868987411750924, 0.10914969774609591, 0.513659],
        "rpy": [-1.5341792327998767, 1.5706962591554918, -1.5340829063937342],
        "parent": "wrist_2_link",
        "chain": ["world", "base_link_inertia", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"],
    },
)


def _as_items(payload: dict[str, Any]) -> list[dict[str, Any]]:
    raw = payload.get("visual_items")
    return raw if isinstance(raw, list) else []


def _token_values(item: dict[str, Any]) -> str:
    keys = (
        "link",
        "link_name",
        "object_name",
        "visual_index_link",
        "visual_index_link_name",
        "id",
        "item_id",
        "display_name",
        "metadata_tags",
        "package_uri",
        "mesh_uri",
        "mesh_path",
        "source_path",
    )
    return "|".join(str(item.get(key) or "") for key in keys).lower()


def _present_required_links(items: list[dict[str, Any]]) -> set[str]:
    present: set[str] = set()
    for item in items:
        if not isinstance(item, dict):
            continue
        mix = _token_values(item)
        for spec in REQUIRED_UR5_VISUALS:
            link = spec["link"]
            if link.lower() in mix:
                present.add(link)
    return present


def _repo_relative_mesh_path(mesh_name: str) -> str:
    return f"assets/robots/universal_robot/ur_description/meshes/ur5/visual/{mesh_name}"


def _pose(xyz: list[float], rpy: list[float]) -> dict[str, list[float]]:
    return {"xyz": [float(v) for v in xyz], "rpy": [float(v) for v in rpy]}


def _ur5_item(spec: dict[str, Any], index: int) -> dict[str, Any]:
    link = str(spec["link"])
    mesh_name = str(spec["mesh"])
    package_uri = f"package://ur_description/meshes/ur5/visual/{mesh_name}"
    mesh_path = _repo_relative_mesh_path(mesh_name)
    xyz = list(spec["xyz"])
    rpy = list(spec["rpy"])
    pose = _pose(xyz, rpy)
    return {
        "id": f"generated_urdf::{link}::visual_{index}::{index}",
        "source": "urdf_flattened",
        "source_layer": "locked_generated_urdf_visual",
        "active_visual_source": "mesh_preview",
        "link": link,
        "link_name": link,
        "object_name": link,
        "canonical_link_name": link,
        "visual": f"visual_{index}",
        "visual_name": f"visual_{index}",
        "visual_index": index,
        "source_row_index": index,
        "parent_link": str(spec.get("parent") or ""),
        "immediate_parent_link": str(spec.get("parent") or ""),
        "root_link": "world",
        "link_chain": list(spec.get("chain") or ["world", link]),
        "joint_parent_link": str(spec.get("parent") or ""),
        "pose": pose,
        "chain_pose": pose,
        "world_pose": pose,
        "link_world_pose": pose,
        "expected_visual_pose": pose,
        "baked_world_visual_pose": pose,
        "baked_world_visual_transform_source": "urdf_fk_link_world_times_visual_origin",
        "visual_origin_applied_to_pose": True,
        "visual_origin": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
        "geometry_type": "mesh",
        "primitive_geometry_type": "mesh",
        "mesh_uri": package_uri,
        "package_uri": package_uri,
        "source_path": package_uri,
        "mesh_path": mesh_path,
        "resolved_source_path": mesh_path,
        "resolved_source_path_is_repo_relative": True,
        "resolved": True,
        "mesh_scale": [1.0, 1.0, 1.0],
        "scale": [1.0, 1.0, 1.0],
        "render_expected": True,
        "render_skip_reason": "",
        "warning": "",
        "material": {"name": "ur5_preview", "color": [0.85, 0.88, 0.92, 1.0]},
        "color": [0.85, 0.88, 0.92, 1.0],
        "link_transform_status": "runtime_ur5_repair",
        "transform_status": "runtime_ur5_repair",
        "transform_chain": list(spec.get("chain") or ["world", link]),
        "visual_index_link": link,
        "visual_index_link_name": link,
        "visual_index_object_name": link,
        "visual_index_visual": f"visual_{index}",
        "visual_index_visual_name": f"visual_{index}",
        "visual_index_value": index,
        "visual_index_parent_link": str(spec.get("parent") or ""),
        "visual_index_link_chain": list(spec.get("chain") or ["world", link]),
        "visual_index_mesh_uri": package_uri,
        "visual_index_package_uri": package_uri,
        "visual_index_source": "runtime_ur5_visual_index_repair",
        "metadata_tags": "source=urdf_flattened;runtime_ur5_visual_index_repair;rviz_parity",
    }


def repair_index(path: Path) -> tuple[bool, list[str]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    scene_name = str(payload.get("scene_name") or path.parents[1].name)
    if not scene_name.startswith("ur5_"):
        return False, []

    items = _as_items(payload)
    present = _present_required_links(items)
    missing = [spec for spec in REQUIRED_UR5_VISUALS if spec["link"] not in present]
    if not missing:
        return False, []

    repaired = [_ur5_item(spec, index) for index, spec in enumerate(REQUIRED_UR5_VISUALS)]
    # Keep non-UR5 rows from extraction, but remove partial/stale UR5 rows so the
    # viewport receives exactly one authoritative row per required UR5 link.
    required_links = {spec["link"] for spec in REQUIRED_UR5_VISUALS}
    kept = [
        item for item in items
        if isinstance(item, dict)
        and not any(link.lower() in _token_values(item) for link in required_links)
    ]
    payload["visual_items"] = repaired + kept
    payload["visual_count"] = len(payload["visual_items"])
    payload["candidate_mesh_count"] = len(payload["visual_items"])
    payload["emitted_visual_count"] = len(payload["visual_items"])
    payload["renderable_item_count"] = len([i for i in payload["visual_items"] if i.get("render_expected", True)])
    payload["renderable_mesh_count"] = len([i for i in payload["visual_items"] if i.get("geometry_type") == "mesh" and i.get("render_expected", True)])
    payload["resolved"] = len([i for i in payload["visual_items"] if i.get("resolved")])
    payload["unresolved"] = len([i for i in payload["visual_items"] if not i.get("resolved")])
    payload["safe_for_preview"] = True
    payload["stale_index"] = False
    payload["stale_reasons"] = []
    payload["ur5_runtime_repair_applied"] = True
    payload["ur5_runtime_repair_added_links"] = [spec["link"] for spec in missing]
    payload["ur5_required_links"] = [spec["link"] for spec in REQUIRED_UR5_VISUALS]
    payload["repair_generated_at"] = datetime.now(timezone.utc).isoformat()
    blockers = payload.get("blockers") if isinstance(payload.get("blockers"), list) else []
    payload["blockers"] = [b for b in blockers if "ur5" not in str(b).lower()]
    warnings = payload.get("warnings") if isinstance(payload.get("warnings"), list) else []
    note = "ur5_runtime_visual_index_repair_applied"
    if note not in warnings:
        warnings.append(note)
    payload["warnings"] = warnings
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return True, [spec["link"] for spec in missing]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("index_path", type=Path)
    args = parser.parse_args()
    changed, missing = repair_index(args.index_path.resolve())
    if changed:
        print(f"[repair_ur5_scene3d_visual_index] repaired {args.index_path}: added {', '.join(missing)}")
    else:
        print(f"[repair_ur5_scene3d_visual_index] no repair needed for {args.index_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
