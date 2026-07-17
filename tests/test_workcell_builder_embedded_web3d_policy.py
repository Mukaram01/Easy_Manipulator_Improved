from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CMAKE = (ROOT / "workcell_builder/workcell_builder/CMakeLists.txt").read_text()
PKG = (ROOT / "workcell_builder/workcell_builder/package.xml").read_text()
CPP = (ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text()
MAINWINDOW_CPP = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text()
HDR = (ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.h").read_text()
INSTALL = (ROOT / "scripts/install_system_deps.sh").read_text()


def test_default_cmake_requires_webengine_with_actionable_fallback_message():
    assert 'option(WORKCELL_BUILDER_ALLOW_NATIVE_3D_FALLBACK' in CMAKE
    assert 'OFF)' in CMAKE
    assert 'find_package(Qt5 COMPONENTS Widgets Concurrent Svg OpenGL Network REQUIRED)' in CMAKE
    assert 'find_package(Qt5 COMPONENTS WebEngineWidgets QUIET)' in CMAKE
    assert 'if(NOT Qt5WebEngineWidgets_FOUND AND NOT WORKCELL_BUILDER_ALLOW_NATIVE_3D_FALLBACK)' in CMAKE
    assert 'message(FATAL_ERROR' in CMAKE
    for token in [
        'embedded Web3D Product View is required',
        'Qt5 WebEngineWidgets',
        'sudo apt install qtwebengine5-dev',
        '-DWORKCELL_BUILDER_ALLOW_NATIVE_3D_FALLBACK=ON',
    ]:
        assert token in CMAKE


def test_webengine_compile_link_contract_and_native_opt_in():
    assert 'target_compile_definitions(workcell_builder PRIVATE WORKCELL_BUILDER_HAS_WEBENGINE=1)' in CMAKE
    assert 'target_link_libraries(workcell_builder Qt5::WebEngineWidgets)' in CMAKE
    assert 'elseif(WORKCELL_BUILDER_ALLOW_NATIVE_3D_FALLBACK)' in CMAKE
    assert 'WORKCELL_BUILDER_NATIVE_3D_FALLBACK=1' in CMAKE


def test_ros_executable_install_path_uses_package_libexec():
    install_block = CMAKE.split('install(TARGETS workcell_builder', 1)[1].split(')', 1)[0]
    assert 'DESTINATION lib/${PROJECT_NAME}' in install_block
    assert 'RUNTIME DESTINATION bin' not in install_block


def test_package_and_system_deps_declare_qtwebengine_dev():
    assert '<build_depend>qtwebengine5-dev</build_depend>' in PKG
    assert '<exec_depend>qtwebengine5-dev</exec_depend>' in PKG
    assert 'qtwebengine5-dev' in INSTALL


def test_web3d_is_default_widget_and_native_ingest_is_skipped_when_active():
    assert 'embedded_web_view_ = new QWebEngineView(view3d_container_)' in CPP
    assert 'embedded_web_view_->setObjectName("embeddedWeb3dProductView")' in CPP
    assert 'simple_3d_view_ = embedded_web_view_' in CPP
    assert 'simple_3d_view_ = new Scene3DViewportWidget(view3d_container_)' in CPP
    assert 'Scene3DViewportWidget * compatibility_scene3d_viewport_{ nullptr };' in HDR
    assert 'compatibility_scene3d_viewport_ = new Scene3DViewportWidget(view3d_container_);' in CPP
    assert 'ProductViewBackend product_view_backend_{ ProductViewBackend::NativeScene3D };' in HDR
    assert 'ProductViewBackend { EmbeddedWeb3D, NativeScene3D }' in HDR
    assert 'WORKCELL_BUILDER_PRODUCT_VIEW_BACKEND' in CPP
    assert 'product_view_backend_ = ProductViewBackend::EmbeddedWeb3D' in CPP
    assert 'product_view_backend_ = ProductViewBackend::NativeScene3D' in CPP
    assert 'auto * viewport = active_native_viewport();' in CPP
    assert 'if (viewport) viewport->ingest_preview_items(preview_items_);' in CPP


def test_backend_diagnostics_are_deterministic_and_concise():
    assert 'emit_backend_startup_diagnostic_once' in HDR
    assert 'backend_startup_diagnostic_emitted_' in HDR
    assert 'Workcell Product View backend=embedded_web3d webengine_compiled=true' in CPP
    assert 'Workcell Product View backend=native_compatibility reason=explicit_opt_in' in CPP
    assert 'Workcell Product View backend=native_compatibility reason=webengine_unavailable_fallback' in CPP


def test_embedded_backend_skips_native_scene3d_final_append_and_audit_paths():
    assert 'scene_preview_widget_->is_native_product_view_backend()' in MAINWINDOW_CPP
    assert 'scene_preview_widget_->is_native_product_view_backend() && viewport' in MAINWINDOW_CPP
    assert 'scene_preview_widget_->is_native_product_view_backend() &&\n      has_selected_scene() && selected_scene_name() == QStringLiteral("ur5_2f_test")' in MAINWINDOW_CPP
    assert 'scene_preview_widget_ && scene_preview_widget_->is_native_product_view_backend() &&\n        (scene_name_for_final_append == QStringLiteral("ur5_2f_test") || visual_index_contains_ur5_mesh_rows)' in MAINWINDOW_CPP
    assert 'if (scene_preview_widget_->is_native_product_view_backend()) {\n      append_scene_diagnostic_log_once' in MAINWINDOW_CPP
    for token in [
        'Scene3D skipped duplicate generated visual index row',
        'UR5_FINAL_APPEND',
        'UR5_BAKED_MATRIX_HANDOFF',
        'Scene3D final viewport audit',
        'Scene3D full payload committed',
    ]:
        assert token in MAINWINDOW_CPP


def test_product_view_lifecycle_prepares_before_loading_and_rejects_stale_output():
    prepare_idx = CPP.index('set_embedded_product_view_state(EmbeddedProductViewState::Preparing, scene_id)')
    server_idx = CPP.index('ensure_embedded_web_server_started(embedded_web_repo_root_, identity)')
    load_idx = CPP.index('set_embedded_product_view_state(EmbeddedProductViewState::Loading)', server_idx)
    assert prepare_idx < server_idx < load_idx
    assert 'scripts/ensure_workcell_studio_web_scene_fresh.py' in CPP
    assert '"--scene", selected_scene_dir, "--output", embedded_web_prepare_output_path_, "--stage-assets"' in CPP
    assert 'embedded_web_prepare_command_for_log(selected_scene_dir, embedded_web_prepare_output_path_, force)' in CPP
    assert 'QStringLiteral("scenes/%1").arg(scene)' not in CPP
    assert 'build/workcell_studio_web_scene/%1.web_scene.json' in CPP
    assert 'output_is_fresh' in CPP
    assert 'stale output will not be loaded' in CPP


def test_server_is_loopback_only():
    assert 'http://127.0.0.1:%1/%2' in CPP
    assert '"--bind", "127.0.0.1"' in CPP
    assert 'http://127.0.0.1:%1/workcell_studio_web/viewer/index.html?scene=%2&builderRevision=%3' in CPP


def test_load_finished_true_only_starts_bounded_readiness_polling():
    load_finished = CPP.index('&QWebEngineView::loadFinished')
    handler = CPP[load_finished:CPP.index('simple_3d_view_ = embedded_web_view_', load_finished)]
    assert 'if (!ok)' in handler
    assert 'set_embedded_product_view_state(EmbeddedProductViewState::Failed' in handler
    assert 'set_embedded_product_view_state(EmbeddedProductViewState::Loading' in handler
    assert 'start_embedded_web_readiness_polling' in handler
    assert 'set_embedded_product_view_state(EmbeddedProductViewState::Ready' not in handler


def test_load_completion_and_readiness_are_scoped_to_the_current_navigation():
    load_finished = CPP.index('&QWebEngineView::loadFinished')
    handler = CPP[load_finished:CPP.index('simple_3d_view_ = embedded_web_view_', load_finished)]
    load_scene = CPP[CPP.index('void ScenePreviewWidget::load_prepared_embedded_web_scene'):]
    readiness = CPP[CPP.index('void ScenePreviewWidget::poll_embedded_web_readiness'):CPP.index('void ScenePreviewWidget::load_prepared_embedded_web_scene')]

    assert 'embedded_web_loading_navigation_token_ = ++embedded_web_navigation_token_' in load_scene
    assert 'embedded_web_expected_viewer_url_ = QUrl(viewer_url)' in load_scene
    assert 'embedded_web_view_->load(embedded_web_expected_viewer_url_)' in load_scene
    assert 'navigation_token != embedded_web_navigation_token_' in handler
    assert 'embedded_web_view_->url() != expected_viewer_url' in handler
    assert 'Ignored stale Embedded Product View load completion' in handler
    assert 'embedded_web_view_->setVisible(false);' in handler
    assert 'start_embedded_web_readiness_polling(identity, navigation_token' in handler
    assert 'navigation_token != embedded_web_navigation_token_' in readiness
    assert 'poll_embedded_web_readiness(identity, navigation_token' in readiness


def test_product_view_ready_requires_viewer_status_not_load_finished_true():
    assert 'window.__WORKCELL_VIEWER_STATUS__' in CPP
    assert 'runJavaScript' in CPP
    assert 'viewer_boot_state' in CPP
    assert 'scene_json_loaded' in CPP
    assert 'source_web_scene_file' in CPP
    assert 'robot_preview_lifecycle_state' in CPP
    assert 'identity.scene_id == QStringLiteral("ur5_2f_test")' in CPP
    assert 'boot_state == QStringLiteral("ready") && expected_json_loaded && robot_ready' in CPP
    assert 'startup timed out after 45s' in CPP
    assert 'failed_stage' in CPP
    assert 'fatal_error' in CPP
    assert 'fatal_stack' in CPP



def test_embedded_web_repo_root_resolution_uses_selected_scene_before_fallbacks():
    cpp = CPP
    hdr = HDR
    assert "QString resolve_embedded_web_repo_root(const QString & selected_scene_dir) const;" in hdr
    assert "resolve_embedded_web_repo_root(const QString & selected_scene_dir) const" in cpp
    assert "selected scene directory upward walk" in cpp
    assert "WORKCELL_STUDIO_REPO_ROOT secondary override" in cpp
    assert "fallback application path upward walk" in cpp
    assert cpp.index("selected scene directory upward walk") < cpp.index("WORKCELL_STUDIO_REPO_ROOT secondary override")
    assert cpp.index("WORKCELL_STUDIO_REPO_ROOT secondary override") < cpp.index("fallback application path upward walk")


def test_embedded_web_repo_root_requires_all_markers_and_rejects_partial_roots():
    cpp = CPP
    for marker in [
        "workcell_studio_web/viewer/index.html",
        "scripts/ensure_workcell_studio_web_scene_fresh.py",
        "scenes",
    ]:
        assert marker in cpp
    assert "missing_markers" in cpp
    assert "candidate rejected" in cpp
    assert "missing %3" in cpp
    assert "repo root selected" in cpp


def test_embedded_web_repo_root_resolution_covers_launch_and_symlink_scenarios():
    cpp = CPP
    assert "canonicalFilePath" in cpp  # symlinked scene path resolves to the real repository tree
    assert "QDir::currentPath()" in cpp  # home/workspace/repository launch cwd is fallback only
    assert "QCoreApplication::applicationDirPath()" in cpp
    prepare = cpp[cpp.index('void ScenePreviewWidget::start_embedded_web_prepare'):cpp.index('void ScenePreviewWidget::on_embedded_web_prepare_finished')]
    assert "identity.absolute_scene_dir" in prepare
    assert "selected_scene_info.exists()" in prepare
    assert "scene_directory_matches_id(selected_scene_dir, scene_id)" in prepare
    assert "QDir::currentPath()" not in prepare
    assert "could not find a Workcell Studio repo root with required markers" in cpp
