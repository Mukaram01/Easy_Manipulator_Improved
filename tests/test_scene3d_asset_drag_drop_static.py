from pathlib import Path


def test_scene3d_asset_drag_drop_tokens_exist():
    main_cpp = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    viewport_h = Path("workcell_builder/workcell_builder/gui/scene3d_viewport_widget.h").read_text(encoding="utf-8")
    viewport_cpp = Path("workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp").read_text(encoding="utf-8")

    assert "application/x-workcell-studio-asset" in main_cpp
    assert "application/x-workcell-studio-asset" in viewport_cpp
    assert "QDrag" in main_cpp and "QMimeData" in main_cpp
    assert "disabled_reason" in main_cpp and "Cannot place here" in main_cpp
    drag_payload = main_cpp[main_cpp.index('payload["asset_id"] = e.asset_id'):main_cpp.index("auto * mime = new QMimeData()", main_cpp.index('payload["asset_id"] = e.asset_id'))]
    assert 'payload["asset_id"] = e.asset_id' in drag_payload
    assert "source_path" not in drag_payload
    assert "CatalogRoleAssetId" in main_cpp
    assert "asset_drop_cb" in viewport_h
    assert "dragEnterEvent" in viewport_h and "dropEvent" in viewport_h
    assert "drag_position_to_world_xy" in viewport_cpp
    assert "snap_translation_value(hit.x(), snap_mode)" in viewport_cpp
    assert "Drop to place" in viewport_cpp
    assert "drag_asset_preview_visible_" in viewport_h
    assert "workcell_studio_next_id" in main_cpp
    assert "mark_layout_dirty" in main_cpp
    assert "parse_collada_bytes_for_test" in viewport_cpp
    assert "ext == QStringLiteral(\"dae\")" in viewport_cpp
    assert "unsupported mesh format" in viewport_cpp
    assert "Locked URDF" in viewport_cpp
    assert "Overlays %1" in viewport_cpp
    assert "Items %1 • Mesh %2 • Boxes %3 • Missing %4" in viewport_cpp
    assert "geometry_type" in main_cpp
    assert "resolved_path" in main_cpp
    assert "package_uri" in main_cpp
    assert "missing_mesh_source_path" in main_cpp
    assert "non_mesh_geometry_added" in main_cpp


def test_native_drop_delegates_identity_and_world_position_to_shared_backend_once():
    main_cpp = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    callback = main_cpp.split("scene3d_viewport->asset_drop_cb =", 1)[1].split("};", 1)[0]
    assert callback.count("place_catalog_asset_at_world_position(") == 1
    assert 'payload.value("asset_id").toString()' in callback
    assert "source_path" not in callback


def test_drag_payload_contains_exactly_canonical_identity_b():
    """A1 step 7: no stale A metadata can hitchhike beside B's asset_id."""
    main_cpp = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    drag = main_cpp.split("auto * drag = new QDrag(asset_catalog_tree_);", 1)[0].rsplit(
        "QJsonObject payload;", 1
    )[1]
    assert drag.count('payload["asset_id"] = e.asset_id;') == 1
    assert 'mime->setData(kWorkcellStudioAssetMime' in drag
    assert 'QJsonDocument(payload).toJson(QJsonDocument::Compact)' in drag
    for forbidden in ["source_path", "display_name", "category", "CatalogRoleIndex"]:
        assert forbidden not in drag


def test_native_drag_drop_reaches_arm_and_commit_through_shared_backend():
    """A1 step 8: native drop has no placement implementation of its own."""
    main_cpp = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    callback = main_cpp.split("scene3d_viewport->asset_drop_cb =", 1)[1].split("};", 1)[0]
    backend = main_cpp.split("bool MainWindow::place_catalog_asset_at_world_position", 1)[1].split(
        "bool MainWindow::configure_asset_placement_transform", 1
    )[0]
    assert callback.count("place_catalog_asset_at_world_position(") == 1
    assert "arm_place_asset_mode(" not in callback
    assert "commit_armed_asset_placement(" not in callback
    assert backend.count("arm_place_asset_mode(asset_id)") == 1
    assert backend.count("commit_armed_asset_placement(") == 1
