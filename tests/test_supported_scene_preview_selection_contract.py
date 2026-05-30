from __future__ import annotations

import json
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "scenes" / "supported_scenes.yaml"


@dataclass(frozen=True)
class PreviewSelectionItem:
    id: str
    source_layer: str
    active_visual_source: str
    editable: bool
    locked: bool
    linked_to_editable_layout_state: bool
    pose: dict[str, list[float]]


def _load_yaml(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def _write_yaml(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _enabled_available_scene_entries() -> list[dict[str, Any]]:
    catalog = _load_yaml(CATALOG)
    entries: list[dict[str, Any]] = []
    for entry in catalog.get("scenes") or []:
        if not isinstance(entry, dict):
            continue
        if entry.get("enabled", True) is False:
            continue
        scene_name = str(entry.get("scene_name") or "").strip()
        if not scene_name:
            continue
        scene_path = ROOT / str(entry.get("scene_path") or f"scenes/{scene_name}")
        if not scene_path.is_dir():
            continue
        normalized = dict(entry)
        normalized["scene_name"] = scene_name
        normalized["scene_path_abs"] = scene_path
        entries.append(normalized)
    return entries


def _scene_id(entry: dict[str, Any]) -> str:
    return str(entry["scene_name"])


def _layout_item_is_editable(item: dict[str, Any]) -> bool:
    # Match Workcell Builder's preview contract: authoring/layout items are
    # editable unless explicitly locked or explicitly marked non-editable.
    return bool(item.get("id")) and not bool(item.get("locked", False)) and item.get("editable", True) is not False


def _item_pose(item: dict[str, Any]) -> dict[str, list[float]]:
    pose = item.get("pose") if isinstance(item.get("pose"), dict) else {}
    xyz = pose.get("xyz") if isinstance(pose.get("xyz"), list) else [0.0, 0.0, 0.0]
    rpy = pose.get("rpy") if isinstance(pose.get("rpy"), list) else [0.0, 0.0, 0.0]
    return {"xyz": list(xyz), "rpy": list(rpy)}


def _generated_preview_item_id(item: dict[str, Any], index: int, prefix: str) -> str:
    raw_id = item.get("id") or item.get("name") or item.get("link") or item.get("visual")
    return str(raw_id or f"{prefix}_{index}")


def _generated_visual_source(item: dict[str, Any]) -> str:
    geometry_type = str(item.get("geometry_type") or item.get("mesh_type") or "").strip().lower()
    if geometry_type == "mesh" or item.get("mesh_path") or item.get("source_path"):
        return "mesh_preview"
    return "primitive_fallback"


def _build_preview_selection_contract(scene_dir: Path) -> list[PreviewSelectionItem]:
    preview: list[PreviewSelectionItem] = []

    layout_path = scene_dir / "layout" / "workcell_studio_layout.yaml"
    if layout_path.exists():
        layout = _load_yaml(layout_path)
        for item in layout.get("items") or []:
            if not isinstance(item, dict) or not item.get("id"):
                continue
            editable = _layout_item_is_editable(item)
            preview.append(
                PreviewSelectionItem(
                    id=str(item["id"]),
                    source_layer="editable_layout",
                    active_visual_source="editable_layout",
                    editable=editable,
                    locked=not editable,
                    linked_to_editable_layout_state=editable,
                    pose=_item_pose(item),
                )
            )

    mesh_index_path = scene_dir / "generated" / "scene_visual_mesh_index.json"
    if mesh_index_path.exists():
        mesh_index = _load_json(mesh_index_path)
        generated_items = mesh_index.get("visual_items") or mesh_index.get("items") or []
        for index, item in enumerate(generated_items):
            if not isinstance(item, dict):
                continue
            preview.append(
                PreviewSelectionItem(
                    id=_generated_preview_item_id(item, index, "generated_visual"),
                    source_layer="locked_generated_urdf_visual",
                    active_visual_source=_generated_visual_source(item),
                    editable=False,
                    locked=True,
                    linked_to_editable_layout_state=False,
                    pose=_item_pose(item),
                )
            )

    generated_layout_path = scene_dir / "layout" / "workcell_studio_layout.generated.yaml"
    if generated_layout_path.exists():
        generated_layout = _load_yaml(generated_layout_path)
        for index, item in enumerate(generated_layout.get("items") or []):
            if not isinstance(item, dict):
                continue
            source = str(item.get("source") or item.get("source_layer") or "").strip()
            if source not in {"locked_generated_urdf_visual", "generated_urdf_visual"}:
                continue
            preview.append(
                PreviewSelectionItem(
                    id=_generated_preview_item_id(item, index, "generated_layout"),
                    source_layer="locked_generated_urdf_visual",
                    active_visual_source=_generated_visual_source(item),
                    editable=False,
                    locked=True,
                    linked_to_editable_layout_state=False,
                    pose=_item_pose(item),
                )
            )
    return preview


def _transform_controls_editable(item: PreviewSelectionItem) -> bool:
    if item.locked:
        return False
    if item.source_layer == "editable_layout":
        return item.editable
    if item.source_layer in {"mesh_preview", "primitive_fallback"} or item.active_visual_source in {"mesh_preview", "primitive_fallback"}:
        return item.editable and item.linked_to_editable_layout_state
    return False


def _save_selected_pose(scene_dir: Path, selected: PreviewSelectionItem, xyz: list[float], rpy: list[float]) -> None:
    if not _transform_controls_editable(selected):
        raise PermissionError(f"selection is read-only and cannot write layout YAML: {selected.id}")
    layout_path = scene_dir / "layout" / "workcell_studio_layout.yaml"
    layout = _load_yaml(layout_path)
    for item in layout.get("items") or []:
        if isinstance(item, dict) and str(item.get("id")) == selected.id:
            item["pose"] = {"xyz": xyz, "rpy": rpy}
            _write_yaml(layout_path, layout)
            return
    raise KeyError(f"editable selection not found in layout YAML: {selected.id}")


def _copy_contract_inputs(source_scene: Path, tmp_path: Path) -> Path:
    copied = tmp_path / source_scene.name
    for rel in (
        Path("layout/workcell_studio_layout.yaml"),
        Path("layout/workcell_studio_layout.generated.yaml"),
        Path("generated/scene_visual_mesh_index.json"),
    ):
        source = source_scene / rel
        if source.exists():
            target = copied / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
    return copied


SCENE_ENTRIES = _enabled_available_scene_entries()


@pytest.mark.parametrize("scene_entry", SCENE_ENTRIES, ids=_scene_id)
def test_enabled_supported_scene_preview_selection_contract(scene_entry: dict[str, Any], tmp_path: Path) -> None:
    scene_dir = _copy_contract_inputs(Path(scene_entry["scene_path_abs"]), tmp_path)
    preview = _build_preview_selection_contract(scene_dir)

    editable_items = [item for item in preview if item.source_layer == "editable_layout" and item.editable]
    locked_preview_items = [item for item in preview if item.source_layer == "locked_generated_urdf_visual"]

    assert editable_items or locked_preview_items, f"{scene_entry['scene_name']} has no preview/selection contract items"

    if editable_items:
        editable = editable_items[0]
        assert editable.linked_to_editable_layout_state is True
        assert _transform_controls_editable(editable) is True
        _save_selected_pose(scene_dir, editable, xyz=[0.12, -0.34, 0.56], rpy=[0.01, 0.02, 0.03])
        reloaded = _build_preview_selection_contract(scene_dir)
        updated = next(item for item in reloaded if item.id == editable.id and item.source_layer == "editable_layout")
        assert updated.pose == {"xyz": [0.12, -0.34, 0.56], "rpy": [0.01, 0.02, 0.03]}

    if locked_preview_items:
        locked = locked_preview_items[0]
        layout_path = scene_dir / "layout" / "workcell_studio_layout.yaml"
        before = layout_path.read_text(encoding="utf-8") if layout_path.exists() else ""
        assert locked.locked is True
        assert locked.editable is False
        assert locked.linked_to_editable_layout_state is False
        assert _transform_controls_editable(locked) is False
        with pytest.raises(PermissionError, match="read-only.*cannot write layout YAML"):
            _save_selected_pose(scene_dir, locked, xyz=[9.0, 9.0, 9.0], rpy=[9.0, 9.0, 9.0])
        after = layout_path.read_text(encoding="utf-8") if layout_path.exists() else ""
        assert after == before


def test_supported_scene_contract_covers_enabled_catalog_not_two_scene_smoke_subset() -> None:
    scene_names = {_scene_id(entry) for entry in SCENE_ENTRIES}
    assert scene_names, "supported scene catalog has no enabled available scenes"
    assert len(scene_names) > 2
    assert scene_names - {"ur10_2f_test", "suction_test"}, (
        "preview/selection contract tests must not exercise only ur10_2f_test or suction_test"
    )
