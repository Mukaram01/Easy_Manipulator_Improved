from pathlib import Path

def test_drag_payload_contains_exactly_canonical_identity_b():
    """A1 step 7: no stale A metadata can hitchhike beside B's asset_id."""
    main_cpp = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    drag = main_cpp.split("auto * drag = new QDrag(asset_catalog_tree_);", 1)[0].rsplit(
        "auto * mime = new QMimeData();", 1
    )[1]
    assert 'mime->setData(kWorkcellStudioAssetMime' in drag
    assert 'e.asset_id.toUtf8()' in drag
    for forbidden in ["source_path", "display_name", "category", "CatalogRoleIndex"]:
        assert forbidden not in drag
