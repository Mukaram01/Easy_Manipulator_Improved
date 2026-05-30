from __future__ import annotations

import json
import shutil
from dataclasses import dataclass
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests" / "fixtures" / "scene3d_selection_roundtrip"


@dataclass(frozen=True)
class PreviewItem:
    id: str
    source_layer: str
    active_visual_source: str
    editable: bool
    locked: bool
    linked_to_editable_layout_state: bool
    pose: dict[str, list[float]]
    source_path: str


def _load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def _write_yaml(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


def _copy_fixture(tmp_path: Path, scene_name: str = "renamed_scene_without_branch") -> Path:
    scene = tmp_path / scene_name
    shutil.copytree(FIXTURE, scene)

    for rel in [
        "environment.yaml",
        "layout/workcell_studio_layout.yaml",
        "cell_definition.yaml",
        "scene_manifest.yaml",
    ]:
        path = scene / rel
        payload = _load_yaml(path)
        if "scene_name" in payload:
            payload["scene_name"] = scene_name
        if payload.get("scene", {}).get("id"):
            payload["scene"]["id"] = scene_name
        _write_yaml(path, payload)

    index_path = scene / "generated" / "scene_visual_mesh_index.json"
    index = json.loads(index_path.read_text(encoding="utf-8"))
    index["scene_name"] = scene_name
    index_path.write_text(json.dumps(index, indent=2) + "\n", encoding="utf-8")
    return scene


def _build_preview_payload(scene: Path) -> list[PreviewItem]:
    layout = _load_yaml(scene / "layout" / "workcell_studio_layout.yaml")
    items: list[PreviewItem] = []
    for item in layout.get("items", []):
        if not isinstance(item, dict):
            continue
        locked = bool(item.get("locked", False))
        editable = bool(item.get("editable", False)) and not locked
        items.append(
            PreviewItem(
                id=item["id"],
                source_layer="editable_layout",
                active_visual_source="editable_layout",
                editable=editable,
                locked=locked,
                linked_to_editable_layout_state=editable,
                pose=item.get("pose", {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}),
                source_path=item.get("source_path") or item.get("mesh", {}).get("path", ""),
            )
        )

    mesh_index = json.loads((scene / "generated" / "scene_visual_mesh_index.json").read_text(encoding="utf-8"))
    for visual in mesh_index.get("visual_items", []):
        if not isinstance(visual, dict):
            continue
        items.append(
            PreviewItem(
                id=visual["id"],
                source_layer="locked_generated_urdf_visual",
                active_visual_source="mesh_preview" if visual.get("geometry_type") == "mesh" else "primitive_fallback",
                editable=False,
                locked=True,
                linked_to_editable_layout_state=False,
                pose=visual.get("pose", {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}),
                source_path=visual.get("source_path", ""),
            )
        )
    return items


def _select(items: list[PreviewItem], item_id: str) -> PreviewItem:
    return next(item for item in items if item.id == item_id)


def _inspector_editable(item: PreviewItem) -> bool:
    if item.locked:
        return False
    if item.source_layer == "editable_layout":
        return item.editable
    if item.source_layer in {"mesh_preview", "primitive_fallback"} or item.active_visual_source in {"mesh_preview", "primitive_fallback"}:
        return item.editable and item.linked_to_editable_layout_state
    return False


def _save_editable_pose(scene: Path, selected: PreviewItem, xyz: list[float], rpy: list[float]) -> None:
    if not _inspector_editable(selected):
        raise PermissionError(f"locked/generated item edit rejected: {selected.id}")
    layout_path = scene / "layout" / "workcell_studio_layout.yaml"
    layout = _load_yaml(layout_path)
    for item in layout.get("items", []):
        if item.get("id") == selected.id:
            item["pose"] = {"xyz": xyz, "rpy": rpy}
            break
    else:
        raise KeyError(selected.id)
    _write_yaml(layout_path, layout)


def test_synthetic_fixture_has_required_authoring_and_preview_files() -> None:
    required = [
        "environment.yaml",
        "layout/workcell_studio_layout.yaml",
        "cell_definition.yaml",
        "scene_manifest.yaml",
        "generated/scene_visual_mesh_index.json",
    ]
    for rel in required:
        assert (FIXTURE / rel).is_file(), rel

    items = (_load_yaml(FIXTURE / "layout" / "workcell_studio_layout.yaml").get("items") or [])
    editable_items = [item for item in items if item.get("editable") is True and item.get("locked") is False]
    mesh_index = json.loads((FIXTURE / "generated" / "scene_visual_mesh_index.json").read_text(encoding="utf-8"))
    locked_visuals = [item for item in mesh_index.get("visual_items", []) if item.get("id")]

    assert [item["id"] for item in editable_items] == ["editable_fixture_plate"]
    assert [item["id"] for item in locked_visuals] == ["generated_locked_robot_visual"]
    assert mesh_index["safe_for_preview"] is True


def test_selection_and_inspector_rules_are_layer_based_without_scene_name_branches(tmp_path: Path) -> None:
    scene = _copy_fixture(tmp_path, "arbitrary_customer_fixture_name")
    preview = _build_preview_payload(scene)

    editable = _select(preview, "editable_fixture_plate")
    locked = _select(preview, "generated_locked_robot_visual")

    assert editable.source_layer == "editable_layout"
    assert editable.linked_to_editable_layout_state is True
    assert _inspector_editable(editable) is True

    assert locked.source_layer == "locked_generated_urdf_visual"
    assert locked.active_visual_source == "mesh_preview"
    assert locked.linked_to_editable_layout_state is False
    assert _inspector_editable(locked) is False


def test_save_and_reload_updates_only_editable_layout_and_preserves_locked_preview(tmp_path: Path) -> None:
    scene = _copy_fixture(tmp_path, "another_non_catalog_scene_name")
    before = _build_preview_payload(scene)
    editable = _select(before, "editable_fixture_plate")
    locked = _select(before, "generated_locked_robot_visual")

    _save_editable_pose(scene, editable, xyz=[0.70, -0.20, 0.11], rpy=[0.1, 0.2, 0.3])
    try:
        _save_editable_pose(scene, locked, xyz=[9, 9, 9], rpy=[9, 9, 9])
    except PermissionError as exc:
        assert "locked/generated item edit rejected" in str(exc)
    else:  # pragma: no cover - this is the regression failure path.
        raise AssertionError("locked generated mesh preview item was editable")

    after = _build_preview_payload(scene)
    reloaded_editable = _select(after, "editable_fixture_plate")
    reloaded_locked = _select(after, "generated_locked_robot_visual")

    assert reloaded_editable.pose == {"xyz": [0.70, -0.20, 0.11], "rpy": [0.1, 0.2, 0.3]}
    assert reloaded_locked.pose == locked.pose
    assert reloaded_locked.locked is True
    assert reloaded_locked.linked_to_editable_layout_state is False
