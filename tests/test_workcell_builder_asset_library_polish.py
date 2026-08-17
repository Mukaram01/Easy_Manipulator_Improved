from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text()


def test_compact_empty_states_cover_search_recent_and_imported_without_catalog_mutation():
    for message in [
        "No assets match this search.",
        "No recently placed assets yet.",
        "No imported assets yet.<br/>Import an asset to add one.",
    ]:
        assert message in CPP
    filter_body = CPP.split("void MainWindow::on_asset_filter_changed(int)", 1)[1].split(
        "void MainWindow::request_visible_asset_thumbnails", 1
    )[0]
    assert "asset_catalog_entries_.clear()" not in filter_body
    assert "recent_asset_ids_.clear()" not in filter_body


def test_place_again_and_cards_keep_canonical_identity_not_display_text():
    assert "arm_place_asset_mode(recent_asset_ids_.front())" in CPP
    populate = CPP.split("void MainWindow::populate_asset_catalog()", 1)[1].split(
        "void MainWindow::import_stl_to_asset_library", 1
    )[0]
    assert "CatalogRoleAssetId, e.asset_id" in populate
    assert "CatalogRoleAssetId, e.display_name" not in populate


def test_details_use_consistent_presentation_order_and_unknown_marker():
    start = CPP.index('"<b>Name</b>')
    fragment = CPP[start:start + 650]
    labels = ["Name", "Category", "Source", "Format", "Dimensions", "Provenance"]
    assert [fragment.index(label) for label in labels] == sorted(fragment.index(label) for label in labels)
    assert 'QStringLiteral("—")' in fragment
