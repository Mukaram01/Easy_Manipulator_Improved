from pathlib import Path


def test_generated_environment_asset_source_present():
    repo = Path(__file__).resolve().parents[1]
    header = repo / "workcell_builder/workcell_builder/include/generated_environment_asset_writer.hpp"
    source = repo / "workcell_builder/workcell_builder/src_generated_environment_asset_writer.cpp"
    assert header.exists()
    assert source.exists()

    txt = source.read_text()
    assert "generated_by: workcell_builder" in txt
    assert "created_from_ui: true" in txt
