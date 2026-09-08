#!/usr/bin/env python3
"""Normalize staged COLLADA loader metadata for ROS Z-up Product View parity.

Three.js ColladaLoader converts files declaring ``Z_UP`` into a Y-up scene by
adding a loader-root rotation. Workcell Studio deliberately renders in ROS
coordinates (Z-up), and generated URDF visual poses already contain the exact
URDF visual-origin transform used by RViz. Preserving ColladaLoader's extra
file-level conversion therefore rotates DAE visuals a second time.

Only repository-local *staged browser copies* are modified here. Checked-in
package meshes remain byte-for-byte untouched for RViz/MoveIt and other ROS
consumers.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any, Mapping, MutableMapping

SCENE_SECTIONS = ("robots", "tools", "assets", "sensors", "zones", "frames")
STAGED_ROOT = "build/workcell_studio_web_scene/assets/"
UP_AXIS_RE = re.compile(
    r"(<up_axis(?:\s[^>]*)?>\s*)([XYZ]_UP)(\s*</up_axis>)",
    re.IGNORECASE,
)


def _iter_items(payload: Mapping[str, Any]):
    for section in SCENE_SECTIONS:
        values = payload.get(section)
        if not isinstance(values, list):
            continue
        for item in values:
            if isinstance(item, MutableMapping):
                yield section, item


def _staged_path(item: Mapping[str, Any]) -> str:
    for field in ("mesh_staged_path", "repo_relative_staged_path", "mesh_url", "mesh_uri"):
        value = item.get(field)
        if not isinstance(value, str):
            continue
        text = value.strip().replace("\\", "/").lstrip("/")
        if text.startswith(STAGED_ROOT):
            return text
    return ""


def _inside(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def normalize_staged_collada_axes(
    payload: MutableMapping[str, Any], *, repo_root: Path, stage_assets: bool
) -> MutableMapping[str, Any]:
    """Neutralize Three.js' Z_UP->Y_UP loader rotation on staged DAE copies.

    The raw vertex data are intentionally not transformed. Rewriting the staged
    copy's ``up_axis`` declaration to ``Y_UP`` simply prevents ColladaLoader
    from inserting a scene-root axis conversion. Product View can then apply
    the ROS/URDF visual pose directly, matching RViz.
    """
    repo_root = Path(repo_root).expanduser().resolve()
    asset_root = (repo_root / STAGED_ROOT).resolve()
    staged_collada = 0
    normalized = 0
    records: list[dict[str, str]] = []

    if stage_assets:
        for section, item in _iter_items(payload):
            relative = _staged_path(item)
            if not relative or Path(relative).suffix.lower() != ".dae":
                continue

            target = (repo_root / relative).resolve()
            if not _inside(target, asset_root) or not target.is_file():
                continue

            staged_collada += 1
            try:
                text = target.read_text(encoding="utf-8")
            except (OSError, UnicodeError):
                continue

            match = UP_AXIS_RE.search(text)
            if match is None:
                continue
            original_axis = match.group(2).upper()
            if original_axis != "Z_UP":
                continue

            replacement = f"{match.group(1)}Y_UP{match.group(3)}"
            normalized_text = text[: match.start()] + replacement + text[match.end() :]
            try:
                target.write_text(normalized_text, encoding="utf-8")
            except OSError:
                continue

            item["collada_original_up_axis"] = original_axis
            item["collada_staged_up_axis"] = "Y_UP"
            item["collada_loader_axis_normalized_for_ros_z_up"] = True
            item["collada_axis_normalization_reason"] = (
                "staged_browser_copy_neutralizes_threejs_z_up_to_y_up_root_rotation; "
                "ROS_URDF_visual_pose_remains_authoritative"
            )
            normalized += 1
            records.append(
                {
                    "section": section,
                    "id": str(item.get("id") or item.get("link") or "<unnamed>"),
                    "staged_path": relative,
                    "original_up_axis": original_axis,
                    "staged_up_axis": "Y_UP",
                }
            )

    metadata = payload.setdefault("metadata", {})
    if isinstance(metadata, MutableMapping):
        metadata["collada_ros_axis_normalization"] = {
            "schema_version": "workcell_studio_collada_ros_axis/v1",
            "status": "PASS",
            "stage_assets_requested": bool(stage_assets),
            "staged_collada_count": staged_collada,
            "normalized_z_up_count": normalized,
            "source_meshes_modified": False,
            "records": records,
        }
    return payload
