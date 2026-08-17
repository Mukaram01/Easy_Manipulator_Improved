from pathlib import Path

ROOT = Path(__file__).parents[1]
MAIN = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text()
HEADER = (ROOT / "workcell_builder/workcell_builder/gui/asset_thumbnail_service.h").read_text()
SERVICE = (ROOT / "workcell_builder/workcell_builder/gui/asset_thumbnail_service.cpp").read_text()


def test_canonical_thumbnail_contract_and_bounded_background_worker():
    for status in ["Missing", "Queued", "Rendering", "Ready", "Failed"]:
        assert status in HEADER
    for field in ["asset_id", "image", "source_fingerprint", "generated_at", "error"]:
        assert field in HEADER
    assert "QtConcurrent::run" in SERVICE
    assert "worker_active_" in SERVICE


def test_cache_is_derived_and_source_fingerprinted():
    assert 'QStandardPaths::CacheLocation' in SERVICE
    assert 'asset-thumbnails/v1' in SERVICE
    for token in ["canonicalFilePath", "size()", "lastModified", "mesh_scale", "Sha256"]:
        assert token in SERVICE


def test_cards_and_selected_asset_have_thumbnail_states_without_affecting_placement():
    assert 'setIconSize(QSize(112, 84))' in MAIN
    assert 'assetLibrarySelectedThumbnail' in MAIN
    assert 'Loading preview…' in MAIN
    assert 'Preview unavailable' in MAIN
    validate = MAIN.split("void MainWindow::validate_asset_catalog_selection()", 1)[1].split("QString MainWindow::selected_catalog_item_path", 1)[0]
    assert "AssetThumbnail" not in validate
    assert "CatalogRolePlaceable" in validate


def test_filtering_remains_metadata_only_and_requests_after_filter():
    body = MAIN.split("void MainWindow::on_asset_filter_changed(int)", 1)[1].split("void MainWindow::request_visible_asset_thumbnails", 1)[0]
    assert "resolve_visual_mesh_source_path" not in body
    assert body.index("item->setHidden") < body.index("request_visible_asset_thumbnails()")
    assert "requested < 32" in MAIN


def test_existing_resolver_and_single_mesh_scale_are_used():
    assert "resolve_visual_mesh_source_path" in MAIN
    assert "QVector3D(p.x,p.y,p.z)*request.mesh_scale" in SERVICE
    assert SERVICE.count("*request.mesh_scale") == 1
