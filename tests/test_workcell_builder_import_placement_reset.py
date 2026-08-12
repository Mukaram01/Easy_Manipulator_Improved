from pathlib import Path


MAIN = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
HEADER = Path("workcell_builder/workcell_builder/gui/mainwindow.h").read_text(encoding="utf-8")


def _between(start: str, end: str) -> str:
    offset = MAIN.index(start)
    return MAIN[offset:MAIN.index(end, offset)]


def test_armed_asset_reset_clears_all_native_and_embedded_placement_state() -> None:
    body = _between(
        "void MainWindow::clear_armed_asset_placement()",
        "void MainWindow::reset_armed_asset_transform_to_defaults()",
    )
    for token in (
        "place_asset_armed_ = false",
        "armed_asset_id_.clear()",
        "armed_asset_category_.clear()",
        "armed_asset_display_name_.clear()",
        "armed_asset_source_path_.clear()",
        "armed_asset_default_xy_px_ = QPointF(0.0, 0.0)",
        "armed_asset_x_m_ = 0.0",
        "armed_asset_y_m_ = 0.0",
        "armed_asset_z_m_ = 0.0",
        "armed_asset_roll_rad_ = 0.0",
        "armed_asset_pitch_rad_ = 0.0",
        "armed_asset_yaw_rad_ = 0.0",
        "cancel_embedded_asset_placement()",
    ):
        assert token in body
    assert "void clear_armed_asset_placement();" in HEADER


def test_import_invalidates_old_arm_before_catalog_refresh_without_rearming() -> None:
    body = _between(
        "void MainWindow::import_stl_to_asset_library()",
        "void MainWindow::open_add_asset_dialog()",
    )
    reset = body.index("clear_armed_asset_placement();")
    refresh = body.index("populate_asset_catalog();")
    find_import = body.index("find_asset_catalog_item_by_id(asset_catalog_tree_, asset_id)")
    select_import = body.index("asset_catalog_tree_->setCurrentItem(imported_item)")
    assert reset < refresh < find_import < select_import
    assert "arm_place_asset_mode(" not in body
    assert "place_asset_armed_ = true" not in body


def test_catalog_add_action_is_the_explicit_rearm_path() -> None:
    connection = next(
        line for line in MAIN.splitlines()
        if "connect_button(add_to_canvas_button_" in line
    )
    assert "place_catalog_asset_at_world_position(asset_id" in connection
    placement = _between(
        "bool MainWindow::place_catalog_asset_at_world_position",
        "bool MainWindow::configure_asset_placement_transform",
    )
    assert "if (!arm_place_asset_mode(asset_id)) return false;" in placement

