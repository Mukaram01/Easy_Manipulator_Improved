import importlib.util
import math
from pathlib import Path
import sys

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "workcell_studio_layout_mesh_preview_node.py"
SPEC = importlib.util.spec_from_file_location("layout_mesh_preview", SCRIPT)
preview = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = preview
SPEC.loader.exec_module(preview)


def mesh_item(item_id="fixture", path="assets/environment/fixture_pkg/meshes/fixture.stl", **mesh):
    return {
        "id": item_id,
        "geometry_type": "mesh",
        "pose": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]},
        "mesh": {"path": path, **mesh},
    }


def test_repository_environment_package_uri_conversion():
    assert preview.resolve_mesh_resource("assets/environment/demo_assets/meshes/foo.stl") == (
        "package://demo_assets/meshes/foo.stl"
    )


def test_workcell_builder_imported_asset_uri_conversion():
    assert preview.resolve_mesh_resource("workcell_builder/workcell_builder/assets/imported/foo.stl") == (
        "package://workcell_builder/assets/imported/foo.stl"
    )


def test_existing_package_uri_is_preserved():
    uri = "package://fixture_description/meshes/foo.stl"
    assert preview.resolve_mesh_resource(uri) == uri


def test_traversal_is_rejected():
    with pytest.raises(preview.LayoutMeshError, match="traversal"):
        preview.resolve_mesh_resource("assets/environment/pkg/../secret.stl")


def test_missing_mesh_path_is_rejected():
    item = mesh_item()
    item["mesh"].pop("path")
    with pytest.raises(preview.LayoutMeshError, match="mesh.path or mesh.uri"):
        preview.parse_layout_meshes({"items": [item]})


def test_scale_is_preserved_and_defaults_only_when_absent():
    scaled, defaulted = preview.parse_layout_meshes({"items": [
        mesh_item("scaled", scale=[0.001, 0.002, 0.003]),
        mesh_item("defaulted"),
    ]})
    assert scaled.scale == pytest.approx((0.001, 0.002, 0.003))
    assert defaulted.scale == (1.0, 1.0, 1.0)


def test_owner_rotation_rotates_mesh_local_offset():
    item = mesh_item(origin_offset=[1, 0, 0])
    item["pose"] = {"xyz": [1, 2, 3], "rpy": [0, 0, math.pi / 2]}
    marker = preview.parse_layout_meshes({"items": [item]})[0]
    assert marker.position == pytest.approx((1, 3, 3))


def test_owner_and_mesh_rpy_quaternions_are_composed():
    item = mesh_item(rpy=[math.pi / 2, 0, 0])
    item["pose"]["rpy"] = [0, 0, math.pi / 2]
    marker = preview.parse_layout_meshes({"items": [item]})[0]
    expected = preview._quaternion_multiply(
        preview._quaternion_from_rpy([0, 0, math.pi / 2]),
        preview._quaternion_from_rpy([math.pi / 2, 0, 0]),
    )
    assert marker.orientation == pytest.approx(expected)


def test_multiple_meshes_have_distinct_deterministic_ids():
    forward = preview.parse_layout_meshes({"items": [mesh_item("zeta"), mesh_item("alpha")]})
    reverse = preview.parse_layout_meshes({"items": [mesh_item("alpha"), mesh_item("zeta")]})
    assert {item.item_id: item.marker_id for item in forward} == {"alpha": 0, "zeta": 1}
    assert {item.item_id: item.marker_id for item in reverse} == {"alpha": 0, "zeta": 1}


def test_non_mesh_items_and_non_mapping_mesh_overlays_are_ignored():
    items = [
        {"id": "pick_zone", "geometry_type": "zone", "mesh": {}},
        {"id": "overlay", "geometry_type": "mesh", "mesh": "diagnostic"},
        mesh_item("physical_fixture"),
    ]
    assert [item.item_id for item in preview.parse_layout_meshes({"items": items})] == ["physical_fixture"]
