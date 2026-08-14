from pathlib import Path

import pytest

from scripts import export_workcell_studio_web_scene as exporter


def _write_bounds_stl(path: Path, dimensions: tuple[float, float, float]) -> None:
    x, y, z = dimensions
    path.write_text(
        "solid bounds\n"
        f"  vertex 0 0 0\n  vertex {x} 0 0\n  vertex 0 {y} 0\n"
        f"  vertex 0 0 {z}\n  vertex {x} {y} {z}\n"
        "endsolid bounds\n",
        encoding="utf-8",
    )


def _imported_item(item_id: str, mesh: Path, catalog_asset_id: str, *, overlay: bool) -> dict:
    return {
        "id": item_id,
        "catalog_asset_id": catalog_asset_id,
        "category": "authored_asset_object",
        "source_kind": "user_authored",
        "source_layer": "editable_layout",
        "authoring_session_overlay": overlay,
        "mesh_uri": str(mesh),
        "resolved_source_path": str(mesh),
        "dimensions": [1, 1, 1],
        "mesh_scale": [0.001, 0.001, 0.001],
        "pose": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]},
    }


def test_imported_stl_bounds_outrank_placeholder_dimensions_per_mesh(tmp_path: Path) -> None:
    first_mesh = tmp_path / "first.stl"
    second_mesh = tmp_path / "second.stl"
    _write_bounds_stl(first_mesh, (101.6129, 75.0, 161.0062))
    _write_bounds_stl(second_mesh, (690.5, 298.5, 1026.5))
    first = _imported_item("object_01", first_mesh, "imported_2068_001_24", overlay=False)
    second = _imported_item("object_02", second_mesh, "imported_02030397", overlay=False)
    payload = {"assets": [first, second]}

    exporter._populate_visual_bounds_item_fields(payload, {})

    assert first["expected_dimensions_m"] == pytest.approx([0.1016129, 0.075, 0.1610062])
    assert first["expected_dimensions_m"] != [0.001, 0.001, 0.001]
    assert second["expected_dimensions_m"] == pytest.approx([0.6905, 0.2985, 1.0265])
    assert first["catalog_asset_id"] == "imported_2068_001_24"
    assert second["catalog_asset_id"] == "imported_02030397"
    for item in (first, second):
        assert item["mesh_scale"] == [0.001, 0.001, 0.001]
        assert item["allow_mesh_unit_autoscale"] is False


def test_persisted_and_overlay_instances_share_measured_physical_bounds(tmp_path: Path) -> None:
    mesh = tmp_path / "shared.stl"
    _write_bounds_stl(mesh, (100, 75, 160))
    persisted = _imported_item("saved_object", mesh, "shared_catalog_asset", overlay=False)
    overlay = _imported_item("dirty_object", mesh, "shared_catalog_asset", overlay=True)
    payload = {"assets": [persisted, overlay]}

    exporter._populate_visual_bounds_item_fields(payload, {})

    expected = pytest.approx([0.1, 0.075, 0.16])
    assert persisted["expected_dimensions_m"] == expected
    assert overlay["expected_dimensions_m"] == expected
    assert persisted["dimensions"] == overlay["dimensions"] == [1, 1, 1]


def test_trustworthy_explicit_dimensions_remain_authoritative(tmp_path: Path) -> None:
    mesh = tmp_path / "oversized.stl"
    _write_bounds_stl(mesh, (1000, 1000, 1000))
    item = _imported_item("explicit_contract", mesh, "catalog_asset", overlay=False)
    item["expected_dimensions_m"] = [0.1, 0.1, 0.1]

    exporter._populate_visual_bounds_item_fields({"assets": [item]}, {})

    assert item["expected_dimensions_m"] == [0.1, 0.1, 0.1]
    assert item["allow_mesh_unit_autoscale"] is False
