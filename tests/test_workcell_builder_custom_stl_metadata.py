from workcell_builder.workcell_builder.workcell_asset_catalog import CustomStlMetadata


def test_custom_stl_metadata_roundtrip(tmp_path) -> None:
    path = tmp_path / "custom.stl"
    path.write_text("solid c\nendsolid c\n", encoding="utf-8")
    payload = CustomStlMetadata(
        source_path=str(path),
        display_name="Custom Fixture",
        category="fixture",
        scale=[1.0, 1.0, 1.0],
        xyz=[0.1, 0.2, 0.3],
        rpy=[0.0, 0.0, 1.57],
        collision_mode="visual_only",
        notes="approximate collision",
    )
    loaded = CustomStlMetadata.from_dict(payload.to_dict())
    assert loaded.source_path == payload.source_path
    assert loaded.collision_mode == "visual_only"
