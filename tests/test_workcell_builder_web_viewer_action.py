from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT / "workcell_builder/workcell_builder/gui/scene_select.cpp"
HDR = ROOT / "workcell_builder/workcell_builder/gui/scene_select.h"
README = ROOT / "workcell_studio_web/viewer/README.md"
PATCH_DOC = ROOT / "docs/WORKCELL_STUDIO_WEB_3D_EDIT_PATCH.md"


def test_web_viewer_action_registered_and_wired_to_export_slot():
    cpp = CPP.read_text(encoding="utf-8")
    hdr = HDR.read_text(encoding="utf-8")
    assert "Export & Open Web 3D Viewer" in cpp
    assert "export_open_web_3d_viewer_action" in cpp
    assert "on_export_open_web_3d_viewer_clicked" in cpp
    assert "void on_export_open_web_3d_viewer_clicked();" in hdr
    assert "QDesktopServices::openUrl" in cpp


def test_web_viewer_action_exports_fresh_scene_to_build_before_opening_viewer():
    cpp = CPP.read_text(encoding="utf-8")
    assert '"scripts" / "run_workcell_web3d_visual_acceptance.py"' in cpp
    assert 'repo_root / "build" / "workcell_studio_web_scene"' in cpp
    assert 'scene_id + ".web_scene.json"' in cpp
    assert 'scene_dir_for_current_selection()' in cpp
    assert '"--scene", QString::fromStdString(scene_dir.string())' in cpp
    assert '"--output", QString::fromStdString(output_path.string())' in cpp
    assert '"--port", "8765"' in cpp
    assert 'repo_root / "workcell_studio_web" / "viewer" / "index.html"' in cpp
    assert (
        "Web 3D scene export failed; viewer not opened with stale generated artifacts."
        in cpp
    )
    assert "QProcess::startDetached" in cpp
    assert "python3 -m http.server 8765 --bind 127.0.0.1" in cpp
    assert "http://127.0.0.1:8765/workcell_studio_web/viewer/index.html?scene=" in cpp

    export_success_index = cpp.index('append_success("Exported Web 3D scene JSON: "')
    server_start_index = cpp.index("QProcess::startDetached", export_success_index)
    browser_open_index = cpp.index("QDesktopServices::openUrl(QUrl(viewer_url))", export_success_index)
    assert export_success_index < server_start_index
    assert export_success_index < browser_open_index


def test_user_facing_failures_and_local_server_fallback_are_present():
    cpp = CPP.read_text(encoding="utf-8")
    for text in [
        "No scene selected",
        "Exporter script missing",
        "Scene path missing",
        "Web 3D scene export failed",
        "Viewer file missing",
        "Browser open failed",
        "reused existing server",
        "Local static asset server",
        "Viewer URL opened",
        "Exported web scene JSON",
        "python3 -m http.server 8765 --bind 127.0.0.1",
        "http://127.0.0.1:8765/workcell_studio_web/viewer/index.html?scene=",
    ]:
        assert text in cpp


def test_docs_describe_builder_action_and_safety_boundaries():
    docs = README.read_text(encoding="utf-8") + "\n" + PATCH_DOC.read_text(encoding="utf-8")
    assert "Export & Open Web 3D Viewer" in docs
    assert "build/workcell_studio_web_scene/<scene_id>.web_scene.json" in docs
    assert "does not write under `scenes/`" in docs
    assert "does not apply edit patches" in docs
    assert "RViz/MoveIt" in docs
    assert "python3 -m http.server 8765" in docs


SCENE_PREVIEW_CPP = ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp"
SCENE_PREVIEW_HDR = ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.h"
SCENE_PREVIEW_CMAKE = ROOT / "workcell_builder/workcell_builder/CMakeLists.txt"


def test_embedded_web3d_product_view_prepares_scene_before_loading():
    cpp = SCENE_PREVIEW_CPP.read_text(encoding="utf-8")
    hdr = SCENE_PREVIEW_HDR.read_text(encoding="utf-8")
    assert "QWebEngineView" in cpp
    assert "embeddedWeb3dProductView" in cpp
    assert "WaitingForBrowserReadiness" in hdr
    assert "CompatibilityReady" in hdr
    assert "scripts/ensure_workcell_studio_web_scene_fresh.py" in cpp
    assert '"--scene", selected_scene_dir' in cpp
    assert '"--output", embedded_web_prepare_output_path_' in cpp
    assert '"--stage-assets"' in cpp
    assert "setWorkingDirectory(repo_root)" in cpp
    assert "setProcessEnvironment(QProcessEnvironment::systemEnvironment())" in cpp
    assert "embedded_web_expected_viewer_url_ = QUrl(viewer_url)" in cpp
    assert "embedded_web_view_->load(embedded_web_expected_viewer_url_)" in cpp
    assert "QFileInfo::exists(QDir(embedded_web_repo_root_).filePath(output_path))" in cpp
    assert "old output is rejected even if present" in cpp


def test_embedded_web3d_coalesces_refreshes_and_rejects_stale_completions():
    cpp = SCENE_PREVIEW_CPP.read_text(encoding="utf-8")
    hdr = SCENE_PREVIEW_HDR.read_text(encoding="utf-8")
    assert "pending_embedded_web_identity_" in hdr
    assert "embedded_web_request_generation_" in hdr
    assert "if (embedded_web_prepare_process_ && embedded_web_prepare_process_->state() != QProcess::NotRunning) return;" in cpp
    assert "embedded_web_identity_is_current(identity)" in cpp
    assert "Ignored stale Embedded Product View" in cpp


def test_embedded_web3d_server_is_local_reused_and_controls_are_not_native_noops():
    cpp = SCENE_PREVIEW_CPP.read_text(encoding="utf-8")
    cmake = SCENE_PREVIEW_CMAKE.read_text(encoding="utf-8")
    assert '"--bind", "127.0.0.1"' in cpp
    assert "embedded_web_server_is_usable" in cpp
    assert "reused existing verified server" in cpp
    assert "load_prepared_embedded_web_scene" in cpp
    assert "builderRevision=" in cpp
    assert "build/workcell_studio_web_scene/%1.web_scene.json" in cpp
    assert "hide native Scene3DViewportWidget-only controls" in cpp
    assert "set_visible(view_actions_label_, !embedded_web_active)" in cpp
    assert "Qt5::Network" in cmake


def test_embedded_web3d_header_and_mode_do_not_show_native_banner_text():
    main_cpp = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    preview_cpp = SCENE_PREVIEW_CPP.read_text(encoding="utf-8")
    product_view_block = main_cpp.split("#ifdef WORKCELL_BUILDER_HAS_WEBENGINE", 1)[1].split("top_bar->addWidget(product_view_help_label);", 1)[0]
    webengine_block = product_view_block.split("#else", 1)[0]
    native_fallback_block = product_view_block.split("#else", 1)[1].split("#endif", 1)[0]

    assert "Native Scene3D: lightweight editable layout preview; not guaranteed RViz-equivalent." not in main_cpp
    assert "Web3D Product View" in webengine_block
    assert "Native Scene3D" not in webengine_block
    assert "Native Scene3D compatibility preview" in native_fallback_block
    assert 'mode_selector_->addItems({"Web3D Product View", "2D Layout"})' in preview_cpp


def test_embedded_web3d_hides_native_only_toolbar_controls():
    cpp = SCENE_PREVIEW_CPP.read_text(encoding="utf-8")
    visibility_body = cpp.split("void ScenePreviewWidget::refresh_toolbar_visibility()", 1)[1].split("void ScenePreviewWidget::refresh_mode_and_state()", 1)[0]
    assert "const bool embedded_web_active = (active_native_viewport() == nullptr);" in visibility_body
    for token in [
        "set_visible(mesh_preview_mode_label_, !embedded_web_active)",
        "set_visible(mesh_preview_mode_selector_, !embedded_web_active)",
        "set_visible(gizmo_mode_label_, embedded_web_active)",
        "set_visible(gizmo_mode_selector_, embedded_web_active)",
        "set_visible(labels_label_, !embedded_web_active)",
        "set_visible(labels_selector_, !embedded_web_active)",
        "set_visible(view_actions_label_, !embedded_web_active)",
        "set_visible(view_actions_selector_, !embedded_web_active)",
        "set_visible(interaction_mode_label_, !embedded_web_active)",
        "set_visible(interaction_mode_selector_, !embedded_web_active)",
    ]:
        assert token in visibility_body
