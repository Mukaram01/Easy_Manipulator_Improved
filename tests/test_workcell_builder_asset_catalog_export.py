from pathlib import Path


def test_export_report_tokens_exist():
    cpp = Path('workcell_builder/workcell_builder/gui/asset_catalog_model.cpp').read_text()
    for token in ['asset_catalog.json', 'asset_catalog.html', 'discovered_roots', 'assets', 'fake_hardware_first']:
        assert token in cpp
