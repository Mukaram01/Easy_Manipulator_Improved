#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.repair_ur5_scene3d_visual_index import REQUIRED_UR5_LINKS, STABLE_UR5_2F_LINKS, repair_index


def _write_index(tmp_path: Path, payload: dict) -> Path:
    scene_dir = tmp_path / "scenes" / "suction_test" / "generated"
    scene_dir.mkdir(parents=True)
    path = scene_dir / "scene_visual_mesh_index.json"
    path.write_text(json.dumps(payload) + "\n", encoding="utf-8")
    return path


def test_repair_replaces_stale_unrenderable_ur5_rows_and_preserves_non_ur5_rows(tmp_path):
    path = _write_index(
        tmp_path,
        {
            "scene_name": "suction_test",
            "visual_items": [
                {
                    "id": "stale_shoulder",
                    "link": "shoulder_link",
                    "mesh_uri": "package://ur_description/meshes/ur5/visual/shoulder.dae",
                    "resolved": False,
                    "render_expected": True,
                    "warning": "file_not_found",
                },
                {
                    "id": "table",
                    "display_name": "table",
                    "geometry_type": "box",
                    "render_expected": True,
                    "resolved": True,
                },
            ],
            "items": [],
            "blockers": [
                "ur5_final_viewport_links_missing",
                "rendered_ur5_link_count_below_7",
                "keep_non_ur5_blocker",
            ],
        },
    )

    changed, added = repair_index(path)

    assert changed is True
    assert set(added) == set(REQUIRED_UR5_LINKS)
    payload = json.loads(path.read_text(encoding="utf-8"))
    rows = payload["visual_items"]
    assert payload["items"] == rows
    assert any(row.get("id") == "table" for row in rows)
    ur5_rows = [row for row in rows if row.get("robot_model") == "ur5"]
    assert len(ur5_rows) == len(REQUIRED_UR5_LINKS)
    assert {row["link"] for row in ur5_rows} == set(REQUIRED_UR5_LINKS)
    assert all(row["category"] == "robot" for row in ur5_rows)
    assert all(row["role"] == "robot" for row in ur5_rows)
    assert all(row["has_mesh_metadata"] is False for row in ur5_rows)
    assert all(row["geometry_type"] == "box" for row in ur5_rows)
    assert all(row["primitive_geometry_type"] == "box" for row in ur5_rows)
    assert all(row["active_visual_source"] == "primitive_fallback" for row in ur5_rows)
    assert all(row["source_layer"] == "locked_generated_urdf_visual" for row in ur5_rows)
    assert all(row["final_render_identity"] == row["link"] for row in ur5_rows)
    assert all(row["render_expected"] is True for row in ur5_rows)
    assert payload["ur5_runtime_repair_mode"] == "stable_primitive_builder_preview"
    assert "ur5_final_viewport_links_missing" not in payload["blockers"]
    assert "rendered_ur5_link_count_below_7" not in payload["blockers"]
    assert "keep_non_ur5_blocker" in payload["blockers"]


def test_repair_adds_locked_2f_gripper_proxy_for_ur5_2f_scene(tmp_path):
    path = _write_index(
        tmp_path,
        {
            "scene_name": "ur5_2f_test",
            "visual_items": [
                {
                    "id": "seeded_wrist",
                    "link": "wrist_3_link",
                    "mesh_uri": "package://ur_description/meshes/ur5/visual/wrist3.dae",
                    "baked_world_visual_transform_source": "generated_urdf_identity_rviz_parity_seed",
                    "resolved": True,
                    "render_expected": True,
                },
                {
                    "id": "camera",
                    "display_name": "camera",
                    "geometry_type": "box",
                    "category": "camera",
                    "render_expected": True,
                    "resolved": True,
                },
            ],
            "items": [],
            "blockers": [],
        },
    )

    changed, _added = repair_index(path)

    assert changed is True
    payload = json.loads(path.read_text(encoding="utf-8"))
    rows = payload["visual_items"]
    links = {row.get("link") for row in rows}
    assert set(STABLE_UR5_2F_LINKS).issubset(links)
    assert payload["ur5_runtime_repair_added_end_effector_links"] == list(STABLE_UR5_2F_LINKS)
    gripper_rows = [row for row in rows if row.get("link") in STABLE_UR5_2F_LINKS]
    assert len(gripper_rows) == len(STABLE_UR5_2F_LINKS)
    assert all(row["category"] == "end_effector" for row in gripper_rows)
    assert all(row["role"] == "end_effector" for row in gripper_rows)
    assert all(row["preview_locked"] is True for row in gripper_rows)
    assert all(row["active_visual_source"] == "primitive_fallback" for row in gripper_rows)
    assert any(row.get("id") == "camera" for row in rows)


def test_repair_skips_non_ur5_payload(tmp_path):
    path = _write_index(
        tmp_path,
        {
            "scene_name": "generic_scene",
            "visual_items": [
                {"id": "table", "display_name": "table", "geometry_type": "box", "resolved": True}
            ],
        },
    )

    changed, added = repair_index(path)

    assert changed is False
    assert added == []
