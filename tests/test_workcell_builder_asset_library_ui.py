from pathlib import Path


def test_asset_ui_columns_filters_inspector_export_controls():
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text()
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()
    for token in ['Asset Name', 'Category', 'Readiness', 'Source', 'Path/Package']:
        assert token in cpp
    for token in ['asset_search_filter', 'asset_category_filter', 'asset_readiness_filter', 'show only usable', 'show preview-only', 'show broken']:
        assert token in ui
    for token in ['Inspector diagnostics', 'Refresh Asset Catalog', 'Export Asset Catalog Report']:
        assert token in ui or token in cpp
