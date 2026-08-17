from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
PREVIEW = (ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text(encoding="utf-8")
VIEWER = (ROOT / "workcell_studio_web/viewer/viewer.js").read_text(encoding="utf-8")
MIME = "application/x-workcell-studio-catalog-asset"


def function_body(source: str, signature: str, next_signature: str) -> str:
    return source[source.index(signature):source.index(next_signature, source.index(signature))]


def test_drag_payload_is_only_the_canonical_catalog_identity():
    assert f'kWorkcellStudioAssetMime = "{MIME}"' in MAIN
    assert "mime->setData(kWorkcellStudioAssetMime, e.asset_id.toUtf8())" in MAIN
    drag_source = function_body(MAIN, "bool MainWindow::eventFilter", "void MainWindow::update_scene_builder_top_controls_overflow")
    assert "display_name" not in drag_source
    assert "source_path" not in drag_source
    assert "QJsonDocument(payload)" not in drag_source


def test_unavailable_assets_are_rejected_by_existing_catalog_authority():
    assert "CatalogRolePlaceable).toBool() || !e.editable" in MAIN
    assert "match == asset_catalog_entries_.cend() || !match->editable" in MAIN
    assert "!match->disabled_reason.trimmed().isEmpty()" in MAIN


def test_product_view_accepts_only_internal_mime_and_arms_existing_pipeline():
    assert f'QStringLiteral("{MIME}")' in PREVIEW
    assert "mime->formats() != QStringList{kMimeType}" in PREVIEW
    assert "return arm_place_asset_mode(asset_id);" in MAIN
    # Arming creates only the existing transient browser preview; IDs remain in commit.
    arm = function_body(MAIN, "bool MainWindow::arm_place_asset_mode", "void MainWindow::commit_armed_asset_placement")
    assert "arm_embedded_asset_placement" in arm
    assert "workcell_studio_next_id" not in arm


def test_drag_coordinates_are_view_local_browser_client_coordinates():
    assert "const QPoint point = event->pos(); // QWebEngineView-local == browser client coordinates." in PREVIEW
    assert "updatePlacementPointer?.(%1,%2)" in PREVIEW
    assert "commitPlacementPointer?.(%1,%2)" in PREVIEW
    assert "mapToGlobal" not in PREVIEW[PREVIEW.index("class AssetDropWebEngineView"):PREVIEW.index("#endif\n}  // namespace")]


def test_drag_preview_reuses_the_single_viewer_geometry_pipeline():
    update = function_body(VIEWER, "function updatePlacementPointer", "function commitPlacementPointer")
    commit = function_body(VIEWER, "function commitPlacementPointer", "function getPlacementState")
    assert "updatePlacementPreview({ clientX, clientY })" in update
    assert "updatePlacementPointer(clientX, clientY)" in commit
    assert "state.placement.previewRoot" in VIEWER
    for authority in ["placementPointFromViewport", "proposedPlacementPoint", "contactCorrectPlacementPoint", "evaluatePlacementCollision"]:
        assert authority in VIEWER
        assert authority not in PREVIEW


def test_drop_emits_once_only_when_existing_viewer_state_is_valid():
    commit = function_body(VIEWER, "function commitPlacementPointer", "function getPlacementState")
    assert "Boolean(point && state.placement.valid)" in commit
    assert commit.count("pushEditorEvent('placement_requested'") == 1
    assert "repeat: false" in commit
    assert "cancelPlacement();" in commit
    assert "commit_armed_asset_placement" not in PREVIEW[PREVIEW.index("class AssetDropWebEngineView"):PREVIEW.index("#endif\n}  // namespace")]


def test_invalid_collision_leave_and_cancel_clean_the_same_session():
    assert "if (!result.toBool() && drag_cancelled) drag_cancelled();" in PREVIEW
    assert "void dragLeaveEvent(QDragLeaveEvent * event) override" in PREVIEW
    assert "window.__WORKCELL_EDITOR_API_V1__?.cancelPlacement?.()" in PREVIEW
    assert "catalog_asset_drag_cancel_cb = [this]()" in MAIN
    assert "clear_armed_asset_placement();" in MAIN
    assert "result == Qt::IgnoreAction" in MAIN


def test_recent_and_instance_creation_remain_canonical_commit_only():
    drag_code = MAIN[MAIN.index("bool MainWindow::eventFilter"):MAIN.index("void MainWindow::update_scene_builder_top_controls_overflow")]
    preview_code = PREVIEW[PREVIEW.index("class AssetDropWebEngineView"):PREVIEW.index("#endif\n}  // namespace")]
    assert "record_recent_asset" not in drag_code + preview_code
    assert "workcell_studio_next_id" not in drag_code + preview_code
    commit = function_body(MAIN, "void MainWindow::commit_armed_asset_placement", "void MainWindow::place_last_asset_again")
    assert "workcell_studio_next_id" in commit
    assert "record_recent_asset(asset_id)" in commit


def test_normal_button_rotation_repeat_and_escape_pipeline_remains_present():
    for token in [
        "arm_place_asset_mode(asset_id)",
        "event.shiftKey === true",
        "state.placement.yaw",
        "event.code === 'KeyQ'",
        "event.code === 'KeyE'",
        "event.key === 'Escape'",
    ]:
        assert token in MAIN or token in VIEWER
