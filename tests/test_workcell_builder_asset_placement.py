from scripts.workcell_builder_environment_palette import (
    PlacementEditorError,
    apply_quick_placement,
    build_environment_asset,
    create_asset_placement_editor_model,
    export_environment_layout_with_placement,
    validate_asset_placements,
)


def test_xyz_rpy_serialization() -> None:
    asset = build_environment_asset(asset_id="bin_a", asset_type="bin", xyz=[0.2, 0.3, 0.1], rpy=[0.0, 0.1, 0.2], dimensions=[0.2, 0.2, 0.2])
    model = create_asset_placement_editor_model(asset=asset, role="pick_area", scale=1.1)
    assert model["pose"]["xyz"] == [0.2, 0.3, 0.1]
    assert model["pose"]["rpy"] == [0.0, 0.1, 0.2]
    exported = export_environment_layout_with_placement("layout_a", [{**asset, **model}])
    pose = exported["metadata"]["placement_editor"]["asset_poses"]["bin_a"]
    assert pose["xyz"] == [0.2, 0.3, 0.1]
    assert pose["rpy"] == [0.0, 0.1, 0.2]


def test_quick_placement_functions() -> None:
    table = build_environment_asset(asset_id="table", asset_type="table", xyz=[0.5, 0.0, 0.0], rpy=[0, 0, 0], dimensions=[1.2, 0.8, 0.05])
    robot = build_environment_asset(asset_id="robot", asset_type="robot_base", xyz=[0.0, 0.0, 0.0], rpy=[0, 0, 0], dimensions=[0.4, 0.4, 0.2])
    bin_asset = build_environment_asset(asset_id="bin", asset_type="bin", xyz=[0, 0, 0], rpy=[0, 0, 0], dimensions=[0.2, 0.2, 0.15])
    placed = apply_quick_placement(bin_asset, "on_table", [table, robot])
    assert placed["pose"]["xyz"][2] == 0.05
    assert apply_quick_placement(bin_asset, "left_of_robot", [table, robot])["pose"]["xyz"][1] == 0.5
    dup = apply_quick_placement(bin_asset, "duplicate_asset", [table, robot])
    assert dup["id"] == "bin_copy"


def test_invalid_scale_fails() -> None:
    asset = build_environment_asset(asset_id="bad_scale", asset_type="bin", xyz=[0, 0, 0], rpy=[0, 0, 0], dimensions=[0.1, 0.1, 0.1])
    try:
        create_asset_placement_editor_model(asset=asset, scale=0)
        assert False, "expected PlacementEditorError"
    except PlacementEditorError:
        assert True


def test_missing_pick_place_warnings() -> None:
    assets = [
        build_environment_asset(asset_id="table", asset_type="table", xyz=[0.5, 0, 0], rpy=[0, 0, 0], dimensions=[1.2, 0.8, 0.05]),
        build_environment_asset(asset_id="robot", asset_type="robot_base", xyz=[0, 0, 0], rpy=[0, 0, 0], dimensions=[0.4, 0.4, 0.2]),
    ]
    warnings = validate_asset_placements({"assets": assets})
    assert "pick area missing" in warnings
    assert "place target missing" in warnings


def test_generated_environment_layout_includes_asset_poses() -> None:
    bin_asset = build_environment_asset(asset_id="bin_a", asset_type="bin", xyz=[0.4, 0.2, -0.1], rpy=[0, 0, 0], dimensions=[0.2, 0.2, 0.2])
    enriched = {**bin_asset, "role": "visual_only", "scale": 1.0, "visible": True, "collision_enabled": False, "lock_asset": True}
    layout = export_environment_layout_with_placement("layout_with_placement", [enriched])
    assert layout["metadata"]["placement_editor"]["enabled"] is True
    assert "bin_a" in layout["metadata"]["placement_editor"]["asset_poses"]
    assert any("below floor" in w for w in layout["metadata"]["placement_warnings"])
