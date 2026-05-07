from __future__ import annotations

from scripts.validate_environment_layout import validate_layout
from scripts.workcell_builder_environment_palette import build_environment_asset, build_environment_layout


def test_table_bin_camera_conveyor_asset_metadata_created() -> None:
    assets = [
        build_environment_asset(asset_id="table_main", asset_type="table", xyz=[0, 0, 0], rpy=[0, 0, 0], dimensions=[1.2, 0.8, 0.05], mesh_path="assets/table.stl"),
        build_environment_asset(asset_id="bin_red", asset_type="bin", xyz=[0.4, 0.3, 0.1], rpy=[0, 0, 0], dimensions=[0.2, 0.2, 0.15]),
        build_environment_asset(asset_id="cam_overhead", asset_type="camera_mount", xyz=[0.6, 0, 1.2], rpy=[0, 0, 0], perception_enabled=True),
        build_environment_asset(asset_id="conv_preview", asset_type="conveyor_placeholder", xyz=[1.2, 0, 0], rpy=[0, 0, 0], dimensions=[2.0, 0.5, 0.2]),
        build_environment_asset(asset_id="robot_base_1", asset_type="robot_base", xyz=[0, 0, 0], rpy=[0, 0, 0], dimensions=[0.4, 0.4, 0.2]),
        build_environment_asset(asset_id="spawn_main", asset_type="object_spawn_area", xyz=[0.45, 0, 0.08], rpy=[0, 0, 0], dimensions=[0.3, 0.2, 0.1]),
        build_environment_asset(asset_id="fixture_1", asset_type="fixture_placeholder", xyz=[0.2, -0.3, 0.1], rpy=[0, 0, 0], dimensions=[0.1, 0.1, 0.1]),
    ]
    assert assets[2]["camera"]["device_id"] == "intel_realsense_d435i"
    assert assets[3]["preview_status"] == "preview_only"


def test_xyz_rpy_values_are_saved() -> None:
    asset = build_environment_asset(asset_id="bin_blue", asset_type="bin", xyz=[0.1, 0.2, 0.3], rpy=[0.4, 0.5, 0.6], dimensions=[0.2, 0.2, 0.2])
    assert asset["pose"]["xyz"] == [0.1, 0.2, 0.3]
    assert asset["pose"]["rpy"] == [0.4, 0.5, 0.6]


def test_conveyor_marked_preview_warn() -> None:
    layout = build_environment_layout("demo", [
        build_environment_asset(asset_id="conv_preview", asset_type="conveyor_placeholder", xyz=[1, 0, 0], rpy=[0, 0, 0], dimensions=[1, 1, 1])
    ])
    result = validate_layout(layout, path=__import__('pathlib').Path('in-memory.yaml'), parser='yaml', notes=[])
    assert any("placeholder/visual metadata only" in w for w in result.warnings)


def test_generated_environment_metadata_validates() -> None:
    layout = build_environment_layout("valid", [
        build_environment_asset(asset_id="table_main", asset_type="table", xyz=[0, 0, 0], rpy=[0, 0, 0], dimensions=[1, 1, 0.1]),
        build_environment_asset(asset_id="robot_base_1", asset_type="robot_base", xyz=[0, 0, 0], rpy=[0, 0, 0], dimensions=[0.4, 0.4, 0.2]),
        build_environment_asset(asset_id="spawn_main", asset_type="object_spawn_area", xyz=[0.4, 0, 0.1], rpy=[0, 0, 0], dimensions=[0.3, 0.2, 0.1]),
    ])
    result = validate_layout(layout, path=__import__('pathlib').Path('in-memory.yaml'), parser='yaml', notes=[])
    assert result.ok


def test_ur5_table_bins_still_generates() -> None:
    layout = build_environment_layout("ur5_bins", [
        build_environment_asset(asset_id="table_main", asset_type="table", xyz=[0.5, 0, 0], rpy=[0, 0, 0], dimensions=[1.2, 0.8, 0.05]),
        build_environment_asset(asset_id="bin_red", asset_type="bin", xyz=[0.4, 0.3, 0.1], rpy=[0, 0, 0], dimensions=[0.2, 0.2, 0.15]),
        build_environment_asset(asset_id="bin_blue", asset_type="bin", xyz=[0.4, -0.3, 0.1], rpy=[0, 0, 0], dimensions=[0.2, 0.2, 0.15]),
        build_environment_asset(asset_id="robot_base_1", asset_type="robot_base", xyz=[0, 0, 0], rpy=[0, 0, 0], dimensions=[0.4, 0.4, 0.2]),
        build_environment_asset(asset_id="spawn_main", asset_type="object_spawn_area", xyz=[0.45, 0, 0.08], rpy=[0, 0, 0], dimensions=[0.3, 0.2, 0.1]),
    ])
    assert layout["layout_id"] == "ur5_bins"
    assert len(layout["assets"]) >= 5
