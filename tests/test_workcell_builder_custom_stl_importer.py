from pathlib import Path

import pytest

from workcell_builder.workcell_builder.workcell_asset_catalog import CustomStlMetadata
from scripts.render_workcell_builder_metadata import render_metadata


def test_stl_metadata_creation(tmp_path: Path) -> None:
    mesh = tmp_path / "fixture.stl"
    mesh.write_text("solid fixture\nendsolid fixture\n", encoding="utf-8")
    payload = CustomStlMetadata(
        source_path=str(mesh),
        copied_asset_path="assets/custom/fixture.stl",
        display_name="Fixture A",
        category="fixture",
        support_status="supported",
    )
    assert payload.validate() == []
    dumped = payload.to_dict()
    assert dumped["source_path"] == str(mesh)
    assert dumped["category"] == "fixture"


def test_invalid_file_extension_fails(tmp_path: Path) -> None:
    mesh = tmp_path / "fixture.ply"
    mesh.write_text("ply", encoding="utf-8")
    payload = CustomStlMetadata(source_path=str(mesh), display_name="Fixture A")
    errors = payload.validate()
    assert any("extensions" in e for e in errors)


def test_scale_pose_and_collision_mode_saved(tmp_path: Path) -> None:
    mesh = tmp_path / "box.obj"
    mesh.write_text("o box", encoding="utf-8")
    payload = CustomStlMetadata(
        source_path=str(mesh),
        display_name="Box",
        category="object",
        scale=[1.2, 0.8, 0.5],
        xyz=[0.1, 0.2, 0.3],
        rpy=[0.0, 1.57, 0.0],
        collision_mode="visual_only",
    )
    dumped = payload.to_dict()
    assert dumped["scale"] == [1.2, 0.8, 0.5]
    assert dumped["xyz"] == [0.1, 0.2, 0.3]
    assert dumped["rpy"] == [0.0, 1.57, 0.0]
    assert dumped["collision_mode"] == "visual_only"


def test_generated_metadata_includes_imported_stl_asset(tmp_path: Path) -> None:
    mesh = tmp_path / "asset.dae"
    mesh.write_text("<COLLADA/>", encoding="utf-8")
    custom = CustomStlMetadata(source_path=str(mesh), display_name="Env", category="environment_asset").to_dict()
    metadata = render_metadata("UR5", "2f", "d435i", "finger_pinch_basic", custom_stl_assets=[{**custom, "collision_known": False}])
    reasons = metadata["compatibility"]["reasons"]
    assert any("Custom STL assets" in reason for reason in reasons)


def test_visual_only_custom_stl_does_not_break_generation(tmp_path: Path) -> None:
    mesh = tmp_path / "safe.stl"
    mesh.write_text("solid s\nendsolid s\n", encoding="utf-8")
    payload = CustomStlMetadata(source_path=str(mesh), display_name="Safe", collision_mode="visual_only")
    assert payload.validate() == []
    assert payload.to_dict()["collision_mode"] == "visual_only"


def test_mesh_collision_falls_back_with_warn_status(tmp_path: Path) -> None:
    mesh = tmp_path / "machine.stl"
    mesh.write_text("solid m\nendsolid m\n", encoding="utf-8")
    payload = CustomStlMetadata(source_path=str(mesh), display_name="Machine", category="machine", collision_mode="mesh_collision")
    dumped = payload.to_dict()
    assert dumped["collision_mode"] == "bounding_box_collision"
    assert dumped["support_status"] == "warn"
    assert "fallback" in dumped["notes"]
