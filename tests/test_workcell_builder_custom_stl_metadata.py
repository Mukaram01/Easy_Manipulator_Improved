from workcell_builder.workcell_builder.workcell_asset_catalog import CustomStlMetadata


def test_custom_stl_metadata_roundtrip() -> None:
    payload = CustomStlMetadata(
        file_path="/tmp/custom.stl",
        display_name="Custom Fixture",
        category="fixture",
        scale=[1.0, 1.0, 1.0],
        xyz=[0.1, 0.2, 0.3],
        rpy=[0.0, 0.0, 1.57],
        collision_enabled=False,
        visual_only=True,
        notes="approximate collision",
    )
    loaded = CustomStlMetadata.from_dict(payload.to_dict())
    assert loaded.file_path == payload.file_path
    assert loaded.visual_only is True
