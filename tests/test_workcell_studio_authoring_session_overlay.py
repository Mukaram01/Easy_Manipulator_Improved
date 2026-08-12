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


def _item(item_id: str, x: float) -> dict:
    return {
        "id": item_id, "display_name": item_id, "asset_id": "imported_test_asset",
        "type": "imported_test_asset", "category": "imported_test_asset", "role": "asset",
        "source_layer": "editable_layout", "editable": True, "locked": False, "selectable": True,
        "source_mesh_path": str(MESH), "mesh_type": "stl", "mesh_scale": [1, 1, 1],
        "world_pose": {"xyz": [x, 0.2, 0.3], "rpy": [0, 0, 0]},
        "render_owner": "editable_layout", "render_policy": "primary",
    }


def test_dirty_imported_placement_survives_refresh_and_stages_browser_mesh(tmp_path):
    overlay = tmp_path / "dirty-overlay.json"
    output = ROOT / "build/workcell_studio_web_scene/ur5_2f_test.overlay-test.json"
    staged_root = ROOT / "build/workcell_studio_web_scene/assets/ur5_2f_test"
    _write_overlay(overlay, [_item("object_01", 0.1), _item("object_02", 0.4)])
    try:
        payload = exporter.build_web_scene(
            SCENE, stage_assets=True, output_path=output, authoring_session_overlay=overlay
        )
        objects = {row["id"]: row for row in payload["assets"] if row.get("id") in {"object_01", "object_02"}}
        assert set(objects) == {"object_01", "object_02"}
        first = objects["object_01"]
        assert first["pose"]["xyz"] == [0.1, 0.2, 0.3]
        assert first["catalog_asset_id"] == "imported_test_asset"
        assert first["category"] == "authored_asset_object"
        assert first["render_policy"] == "primary"
        assert first["render_owner"] == "editable_layout"
        assert first["mesh_staging_status"] == "staged"
        assert not first["mesh_uri"].startswith(("file://", "/"))
        assert (ROOT / first["mesh_uri"]).is_file()
        assert first["render_identity"] != objects["object_02"]["render_identity"]
    finally:
        output.unlink(missing_ok=True)
        shutil.rmtree(staged_root, ignore_errors=True)


def test_overlay_rejects_unresolvable_imported_source_with_item_diagnostic(tmp_path):
    overlay = tmp_path / "bad-overlay.json"
    bad = _item("object_01", 0.1)
    bad["source_mesh_path"] = "/missing/imported_test_asset.stl"
    _write_overlay(overlay, [bad])
    with pytest.raises(exporter.BlockingExportError, match=r"object_01.*missing/imported_test_asset\.stl"):
        exporter.build_web_scene(SCENE, authoring_session_overlay=overlay)

