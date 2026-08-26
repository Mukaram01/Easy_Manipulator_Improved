from __future__ import annotations

import copy
import shutil
from pathlib import Path
from typing import Any

import yaml

from scripts.workcell_studio_layout_merge import merge

ROOT = Path(__file__).resolve().parents[1]
SOURCE_SCENE = ROOT / "scenes" / "ur5_2f_test"
LAYOUT_RELATIVE_PATH = Path("layout") / "workcell_studio_layout.yaml"
MESH_INDEX_RELATIVE_PATH = Path("generated") / "scene_visual_mesh_index.json"

ROUNDTRIP_IDS = [
    "support_surface_table",
    "pick_zone_main",
    "place_zone_default",
    "target_bin_default",
    "realsense_overhead",
    "safety_zone_keepout",
    "home_pose_safe",
    "object_02",
    "object_01",
    "object_04",
]
ROUNDTRIP_FIELDS = [
    "type",
    "role",
    "category",
    "display_name",
    "dimensions",
    "editable",
    "locked",
    "source_layer",
]


def _load_yaml(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return data if isinstance(data, dict) else {}


def _write_yaml(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


def _items_by_id(layout: dict[str, Any]) -> dict[str, dict[str, Any]]:
    items = layout.get("items")
    assert isinstance(items, list)
    return {str(item["id"]): item for item in items if isinstance(item, dict) and item.get("id")}


def _item_ids(layout: dict[str, Any]) -> list[str]:
    items = layout.get("items")
    assert isinstance(items, list)
    return [str(item.get("id")) for item in items if isinstance(item, dict) and item.get("id")]


def _locked_preview_items_from_mesh_index(scene_dir: Path) -> list[dict[str, Any]]:
    """Model the canvas-side locked generated preview layer without making it editable."""
    index_path = scene_dir / MESH_INDEX_RELATIVE_PATH
    if not index_path.is_file():
        return []

    import json

    mesh_index = json.loads(index_path.read_text(encoding="utf-8"))
    if not mesh_index.get("safe_for_preview"):
        return []

    preview_items: list[dict[str, Any]] = []
    for visual in mesh_index.get("visual_items") or []:
        if not isinstance(visual, dict) or not visual.get("id"):
            continue
        preview_items.append(
            {
                "id": str(visual["id"]),
                "type": str(visual.get("geometry_type") or "generated_visual"),
                "role": "generated_preview",
                "category": "generated_urdf_visual",
                "display_name": str(visual.get("link") or visual["id"]),
                "pose": copy.deepcopy(visual.get("pose") or {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}),
                "dimensions": copy.deepcopy(visual.get("dimensions") or visual.get("mesh_scale") or [1.0, 1.0, 1.0]),
                "editable": False,
                "locked": True,
                "source_layer": "locked_generated_urdf_visual",
                "source": str(MESH_INDEX_RELATIVE_PATH),
            }
        )
    return preview_items


def _simulate_canvas_load(scene_dir: Path) -> list[dict[str, Any]]:
    """Load editable layout items and generated preview items like the builder canvas model."""
    layout = _load_yaml(scene_dir / LAYOUT_RELATIVE_PATH)
    editable_items = [copy.deepcopy(item) for item in layout.get("items") or [] if isinstance(item, dict)]
    # Raw authored YAML may omit the runtime provenance field. The builder
    # canonicalizes every record loaded from this file to editable_layout.
    for item in editable_items:
        item.setdefault("source_layer", "editable_layout")
    return editable_items + _locked_preview_items_from_mesh_index(scene_dir)


def _simulate_builder_save(scene_dir: Path, canvas_items: list[dict[str, Any]]) -> None:
    """Persist only editable layout-layer items back to the authored layout YAML."""
    persisted_items = []
    for item in canvas_items:
        if not isinstance(item, dict):
            continue
        if item.get("locked") is True or item.get("editable") is False:
            continue
        if item.get("source_layer") != "editable_layout":
            continue
        persisted_items.append(copy.deepcopy(item))

    layout_path = scene_dir / LAYOUT_RELATIVE_PATH
    current = _load_yaml(layout_path)
    current["items"] = persisted_items
    _write_yaml(layout_path, current)


def _assert_roundtrip_fields_match(source: dict[str, Any], actual: dict[str, Any]) -> None:
    for field in ROUNDTRIP_FIELDS:
        assert actual.get(field) == source.get(field), f"{actual.get('id')} field {field} changed"
    assert actual.get("pose", {}).get("xyz") == source.get("pose", {}).get("xyz")
    assert actual.get("pose", {}).get("rpy") == source.get("pose", {}).get("rpy")


def test_ur5_2f_workcell_studio_layout_roundtrip_preserves_canonical_items_without_preview_duplication(tmp_path: Path) -> None:
    source_layout = _load_yaml(SOURCE_SCENE / LAYOUT_RELATIVE_PATH)
    for item in source_layout.get("items") or []:
        if isinstance(item, dict):
            item.setdefault("source_layer", "editable_layout")
    source_by_id = _items_by_id(source_layout)
    for expected_id in ROUNDTRIP_IDS:
        assert _item_ids(source_layout).count(expected_id) == 1
        assert expected_id in source_by_id

    scene_copy = tmp_path / "ur5_2f_test"
    shutil.copytree(SOURCE_SCENE, scene_copy)

    source_count = len(_item_ids(source_layout))
    previous_ids = _item_ids(source_layout)
    locked_preview_ids = {item["id"] for item in _locked_preview_items_from_mesh_index(scene_copy)}

    for _round in range(2):
        canvas_items = _simulate_canvas_load(scene_copy)
        assert len(canvas_items) >= source_count

        _simulate_builder_save(scene_copy, canvas_items)
        report = merge(scene_copy)
        assert report["status"] == "READY"

        reloaded_layout = _load_yaml(scene_copy / LAYOUT_RELATIVE_PATH)
        reloaded_ids = _item_ids(reloaded_layout)
        assert reloaded_ids == previous_ids
        assert len(reloaded_ids) == source_count
        assert len(reloaded_ids) == len(set(reloaded_ids))

        reloaded_by_id = _items_by_id(reloaded_layout)
        for expected_id in ROUNDTRIP_IDS:
            assert reloaded_ids.count(expected_id) == 1
            _assert_roundtrip_fields_match(source_by_id[expected_id], reloaded_by_id[expected_id])

        assert locked_preview_ids.isdisjoint(reloaded_ids)
        previous_ids = reloaded_ids
