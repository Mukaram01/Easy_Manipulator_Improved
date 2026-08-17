from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text()


def test_recents_are_persisted_as_catalog_ids_only_after_successful_commit():
    commit = CPP.index("void MainWindow::commit_armed_asset_placement")
    success = CPP.index('append_studio_log("ghost placement preview committed")', commit)
    record = CPP.index("record_recent_asset(asset_id);", success)
    end = CPP.index("void MainWindow::validate_asset_catalog_selection", record)
    assert success < record < end
    assert 'QSettings().setValue(QStringLiteral("asset_library/recent_catalog_ids")' in CPP
    assert "RoleCatalogAssetId" in CPP[commit:end]


def test_arming_or_cancelling_does_not_record_a_recent_asset():
    arm = CPP.index("bool MainWindow::arm_place_asset_mode")
    commit = CPP.index("void MainWindow::commit_armed_asset_placement", arm)
    assert "record_recent_asset" not in CPP[arm:commit]
    clear = CPP.index("void MainWindow::clear_armed_asset_placement")
    reset = CPP.index("void MainWindow::reset_armed_asset_transform_to_defaults", clear)
    assert "record_recent_asset" not in CPP[clear:reset]


def test_place_again_reuses_the_canonical_arming_pipeline_without_creating_an_instance():
    start = CPP.index("void MainWindow::place_last_asset_again")
    end = CPP.index("QString MainWindow::selected_catalog_item_path", start)
    body = CPP[start:end]
    assert "arm_place_asset_mode(recent_asset_ids_.front())" in body
    assert "commit_armed_asset_placement" not in body
    assert "workcell_studio_next_id" not in body


def test_recent_rows_reuse_catalog_items_and_thumbnail_requests():
    filter_start = CPP.index("void MainWindow::on_asset_filter_changed")
    thumbnail_start = CPP.index("void MainWindow::request_visible_asset_thumbnails", filter_start)
    filter_body = CPP[filter_start:thumbnail_start]
    assert 'selected == QStringLiteral("recent")' in filter_body
    assert "takeTopLevelItem" in filter_body
    assert "new QTreeWidgetItem" not in filter_body
    assert "request_visible_asset_thumbnails();" in filter_body

