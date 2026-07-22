from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VIEWER = (ROOT / "workcell_studio_web/viewer/viewer.js").read_text(encoding="utf-8")
INDEX = (ROOT / "workcell_studio_web/viewer/index.html").read_text(encoding="utf-8")
STYLE = (ROOT / "workcell_studio_web/viewer/style.css").read_text(encoding="utf-8")
CPP = (ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text(encoding="utf-8")
HDR = (ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.h").read_text(encoding="utf-8")


def test_browser_editor_api_v1_contract_and_bounded_events():
    assert "window.__WORKCELL_EDITOR_API_V1__" in VIEWER
    for method in ["getState", "selectItem", "clearSelection", "setMode", "setSnap", "undo", "redo", "fitScene", "getEditPatch", "drainEvents"]:
        assert f"{method}:" in VIEWER
    for field in ["ready", "sceneId", "selectedItemId", "selectedItemType", "selectedEditable", "dirty", "dirtyCount", "canUndo", "canRedo", "mode", "error"]:
        assert field in VIEWER
    for event in ["selection_changed", "dirty_changed", "transform_committed", "editor_error"]:
        assert event in VIEWER
    assert "state.editorEvents.length > 100" in VIEWER
    assert "splice(0, state.editorEvents.length - 100)" in VIEWER


def test_browser_api_reuses_existing_editor_functions_and_preserves_locks():
    for token in ["selectObject(String(id || ''))", "refreshGizmoSnap()", "undoPreviewEdit()", "redoPreviewEdit()", "resetView()", "buildEditPatch()"]:
        assert token in VIEWER
    assert "function canEditItem(item)" in VIEWER
    assert "source.includes('generated')" in VIEWER
    assert "item?.locked || item?.editable !== true" in VIEWER
    assert "emitTransformCommitted(rendered)" in VIEWER
    assert "dragging-changed" in VIEWER


def test_embedded_mode_hides_standalone_browser_ui_only_when_requested():
    assert "params.get('embedded') === '1'" in VIEWER
    assert "document.body.classList.add('embedded-mode')" in VIEWER
    assert "body.embedded-mode .topbar" in STYLE
    assert "body.embedded-mode .object-panel" in STYLE
    assert "body.embedded-mode .details-panel" in STYLE
    assert "body.embedded-mode .toolbar" in STYLE
    assert "body.embedded-mode .app-shell" in STYLE
    assert "class=\"topbar\"" in INDEX


def test_qt_loads_embedded_view_and_routes_controls_without_reload():
    assert "&embedded=1" in CPP
    for helper in ["run_embedded_editor_command", "poll_embedded_editor_events", "apply_embedded_editor_state", "embedded_snap_command"]:
        assert helper in CPP and helper in HDR
    for command in ["setMode", "setSnap", "fitScene", "undo()", "redo()", "selectItem"]:
        assert command in CPP
    assert "QTimer::singleShot(200, this, &ScenePreviewWidget::poll_embedded_editor_events)" in CPP
    assert "refresh_embedded_web_product_view();" not in CPP.split('connect(gizmo_mode_selector_', 1)[1].split('connect(mesh_preview_mode_selector_', 1)[0]


def test_qt_compact_toolbar_for_embedded_web3d():
    for member in ["embedded_undo_button_", "embedded_redo_button_", "embedded_fit_button_"]:
        assert member in CPP and member in HDR
    assert "set_visible(view_actions_label_, !embedded_web_active)" in CPP
    assert "set_visible(view_actions_selector_, !embedded_web_active)" in CPP
    assert "set_visible(labels_label_, !embedded_web_active)" in CPP
    assert "set_visible(labels_selector_, !embedded_web_active)" in CPP
    assert "set_visible(mesh_preview_mode_label_, !embedded_web_active)" in CPP
    assert "set_visible(mesh_preview_mode_selector_, !embedded_web_active)" in CPP
    assert "set_visible(interaction_mode_label_, !embedded_web_active)" in CPP
    assert "set_visible(interaction_mode_selector_, !embedded_web_active)" in CPP
    assert "set_visible(overlays_selector_, !embedded_web_active)" in CPP
    assert "set_visible(gizmo_mode_label_, embedded_web_active)" in CPP
    assert "set_visible(gizmo_mode_selector_, embedded_web_active)" in CPP
    assert "set_visible(snap_mode_selector_, embedded_web_active)" in CPP
    assert "Unsaved preview edits: %1" in CPP
    assert "Locked item — select an editable object or area" in CPP


def test_product_view_selection_uses_stable_identity_and_filters_helpers():
    assert "function isNormalSelectableRendered(rendered)" in VIEWER
    assert "item.selectable !== false" in VIEWER
    assert "!isDiagnosticOnlyItem(item) && !isOverlayPolicyItem(item)" in VIEWER
    assert "renderedById(requestedId)" in VIEWER
    assert "missing_render_identity" in VIEWER
    assert "diagnostic_helper_or_non_selectable" in VIEWER
    assert "selectObject(item.id);" in VIEWER
    assert "itemLabel(item)" not in VIEWER.split("function pickObject", 1)[1].split("function beginDirectMoveDrag", 1)[0]


def test_qt_selection_clears_stale_scene_and_missing_id_callbacks():
    assert "selected_preview_item_id_.clear();" in CPP
    assert "scene_context_changed" in CPP
    assert "selection_missing_after_refresh" in CPP
    assert "Preview selection cleared after refresh (id missing):" in CPP
    assert "if (!embedded_web_identity_is_current(identity)) return;" in CPP
    assert "if (id != selected_preview_item_id_)" in CPP
    assert "selection_update_guard_" in (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")


def test_selection_diagnostics_are_exposed_to_qt_bridge():
    assert "function currentSelectionDiagnostics()" in VIEWER
    assert "selectionDiagnostics: currentSelectionDiagnostics()" in VIEWER
    assert "selectionDiagnostics: () => currentSelectionDiagnostics()" in VIEWER
    for field in ["renderIdentity", "sourceLayer", "activeVisualSource", "diagnosticOnly", "helperOrOverlay", "objectPresent"]:
        assert field in VIEWER
