import json
import shutil
from pathlib import Path

import pytest

from scripts import export_workcell_studio_web_scene as exporter


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "ur5_2f_test"
MESH = ROOT / "assets/end_effectors/robotiq_3f_gripper/robotiq_3f_gripper_description/meshes/robotiq-3f-gripper/collision/GRIPPER_OPEN_PALM.stl"


def _write_overlay(path: Path, items: list[dict], *, revision: int = 7) -> None:
    path.write_text(json.dumps({
        "schema_version": "workcell_studio.authoring_session_overlay.v1",
        "scene_id": "ur5_2f_test",
        "request_identity": {
            "scene_id": "ur5_2f_test", "payload_fingerprint": f"dirty-{revision}",
            "payload_revision": revision, "generation": revision + 1,
        },
        "items": items,
    }), encoding="utf-8")


def _item(item_id: str, x: float, mesh: Path = MESH) -> dict:
    return {
        "id": item_id, "display_name": item_id, "asset_id": "imported_test_asset",
        "type": "imported_test_asset", "category": "imported_test_asset", "role": "asset",
        "source_layer": "editable_layout", "editable": True, "locked": False, "selectable": True,
        "source_mesh_path": str(mesh), "mesh_type": "stl", "mesh_scale": [0.001, 0.001, 0.001],
        "world_pose": {"xyz": [x, 0.2, 0.3], "rpy": [0, 0, 0]},
        "render_owner": "editable_layout", "render_policy": "primary",
    }


def test_mixed_overlay_merges_canonical_asset_and_stages_distinct_imported_objects(tmp_path):
    overlay = tmp_path / "dirty-overlay.json"
    output = ROOT / "build/workcell_studio_web_scene/ur5_2f_test.overlay-test.json"
    staged_root = ROOT / "build/workcell_studio_web_scene/assets/ur5_2f_test"
    imported_dir = ROOT / "build" / "test-authoring-overlay" / "scenes" / "ur5_2f_test" / tmp_path.name
    imported_dir.mkdir(parents=True)
    imported_meshes = [imported_dir / "first.stl", imported_dir / "second.stl"]
    for mesh in imported_meshes:
        shutil.copy2(MESH, mesh)

    canonical = exporter.build_web_scene(SCENE)
    canonical_bin = next(row for row in canonical["assets"] if row.get("type") == "target_bin")
    transient_bin = _item(canonical_bin["id"], 0.8, ROOT / canonical_bin["mesh_path"])
    transient_bin["world_pose"]["rpy"] = [0.1, 0.2, 0.3]
    transient_bin["mesh_scale"] = [9, 9, 9]
    _write_overlay(overlay, [
        transient_bin,
        _item("object_01", 0.1, imported_meshes[0]),
        _item("object_02", 0.4, imported_meshes[1]),
    ])
    try:
        payload = exporter.build_web_scene(
            SCENE, stage_assets=True, output_path=output, authoring_session_overlay=overlay
        )
        expected_ids = {canonical_bin["id"], "object_01", "object_02"}
        objects = {row["id"]: row for row in payload["assets"] if row.get("id") in expected_ids}
        assert set(objects) == expected_ids
        first = objects["object_01"]
        assert first["pose"]["xyz"] == [0.1, 0.2, 0.3]
        assert first["catalog_asset_id"] == "imported_test_asset"
        assert first["mesh_scale"] == [0.001, 0.001, 0.001]
        assert first["category"] == "authored_asset_object"
        assert first["mesh_contract_category"] == "object"
        assert first["readiness_category"] != "robot_arm"
        assert first["render_policy"] == "primary"
        assert first["render_owner"] == "editable_layout"
        for row in objects.values():
            assert row["mesh_staging_status"] == "staged"
            assert not row["mesh_uri"].startswith(("file://", "/"))
            assert (ROOT / row["mesh_uri"]).is_file()
        assert objects["object_02"]["mesh_contract_category"] == "object"
        assert objects["object_02"]["readiness_category"] != "robot_arm"
        assert objects["object_02"]["catalog_asset_id"] == "imported_test_asset"
        assert objects["object_02"]["mesh_scale"] == [0.001, 0.001, 0.001]
        assert first["render_identity"] != objects["object_02"]["render_identity"]

        exported_bin = objects[canonical_bin["id"]]
        assert exported_bin["pose"] == transient_bin["world_pose"]
        for field in ("type", "role", "category", "material", "mesh_scale", "mesh_local_transform", "visual_origin", "provenance"):
            assert exported_bin[field] == canonical_bin[field]
        assert exported_bin["mesh_contract_category"] == "object"
        assert exported_bin["readiness_category"] != "robot_arm"
    finally:
        output.unlink(missing_ok=True)
        shutil.rmtree(staged_root, ignore_errors=True)
        shutil.rmtree(ROOT / "build" / "test-authoring-overlay", ignore_errors=True)


def test_overlay_rejects_unresolvable_imported_source_with_item_diagnostic(tmp_path):
    overlay = tmp_path / "bad-overlay.json"
    bad = _item("object_01", 0.1)
    bad["source_mesh_path"] = "/missing/imported_test_asset.stl"
    _write_overlay(overlay, [bad])
    with pytest.raises(exporter.BlockingExportError, match=r"object_01.*missing/imported_test_asset\.stl"):
        exporter.build_web_scene(SCENE, authoring_session_overlay=overlay)
