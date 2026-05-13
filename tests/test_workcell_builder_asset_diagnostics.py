from pathlib import Path


def test_diagnostics_tokens_and_rules_present():
    cpp = Path('workcell_builder/workcell_builder/gui/asset_catalog_model.cpp').read_text()
    for token in ['MISSING_MOVEIT_CONFIG', 'READY_WITH_WARNINGS', 'PREVIEW_ONLY', 'metadata/preview asset', 'MoveIt config not detected']:
        assert token in cpp
    assert 'simple_conveyor' in cpp
