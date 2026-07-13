from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CMAKE = (ROOT / "workcell_builder/workcell_builder/CMakeLists.txt").read_text()
PKG = (ROOT / "workcell_builder/workcell_builder/package.xml").read_text()
CPP = (ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text()
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
    assert 'auto * viewport = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_);\n  if (viewport) viewport->ingest_preview_items(preview_items_);' in CPP


def test_backend_diagnostics_are_deterministic_and_concise():
    assert 'emit_backend_startup_diagnostic_once' in HDR
    assert 'backend_startup_diagnostic_emitted_' in HDR
    assert 'Workcell Product View backend=embedded_web3d webengine_compiled=true' in CPP
    assert 'Workcell Product View backend=native_compatibility reason=explicit_build_option' in CPP


def test_product_view_lifecycle_prepares_before_loading_and_rejects_stale_output():
    prepare_idx = CPP.index('set_embedded_product_view_state(EmbeddedProductViewState::Preparing, scene)')
    server_idx = CPP.index('ensure_embedded_web_server_started(embedded_web_repo_root_)')
    load_idx = CPP.index('set_embedded_product_view_state(EmbeddedProductViewState::Loading)', server_idx)
    assert prepare_idx < server_idx < load_idx
    assert 'scripts/ensure_workcell_studio_web_scene_fresh.py' in CPP
    assert '"--scene", QStringLiteral("scenes/%1").arg(scene), "--output", embedded_web_prepare_output_path_, "--stage-assets"' in CPP
    assert 'build/workcell_studio_web_scene/%1.web_scene.json' in CPP
    assert 'output_is_fresh' in CPP
    assert 'stale output will not be loaded' in CPP


def test_server_is_loopback_only():
    assert 'socket.connectToHost(QStringLiteral("127.0.0.1"), embedded_web_server_port_)' in CPP
    assert '"--bind", "127.0.0.1"' in CPP
    assert 'http://127.0.0.1:%1/workcell_studio_web/viewer/index.html?scene=%2&builderRevision=%3' in CPP


def test_ready_state_only_from_successful_browser_load():
    load_finished = CPP.index('&QWebEngineView::loadFinished')
    ready = CPP.index('if (ok) set_embedded_product_view_state(EmbeddedProductViewState::Ready', load_finished)
    failed = CPP.index('else set_embedded_product_view_state(EmbeddedProductViewState::Failed', load_finished)
    assert load_finished < ready < failed
    assert 'set_embedded_product_view_state(EmbeddedProductViewState::Ready' not in CPP[:load_finished]
