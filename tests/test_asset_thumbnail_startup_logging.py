from pathlib import Path

SOURCE = Path(
    "workcell_builder/workcell_builder/gui/asset_thumbnail_service.cpp"
).read_text(encoding="utf-8")


def test_successful_thumbnail_work_does_not_flood_interactive_startup_logs():
    assert "Asset thumbnail cache hit:" not in SOURCE
    assert "Asset thumbnail generated:" not in SOURCE


def test_thumbnail_failures_remain_actionable_warnings():
    assert 'qWarning("Asset thumbnail failed: asset=%s reason=%s"' in SOURCE
    assert "mesh file is unavailable:" in SOURCE
