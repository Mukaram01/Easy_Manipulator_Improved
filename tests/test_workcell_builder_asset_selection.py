from pathlib import Path

from workcell_builder.workcell_builder.workcell_asset_catalog import grouped_selection, load_catalog


def test_selection_groups_items_by_category() -> None:
    payload = load_catalog(Path.cwd())
    groups = grouped_selection(payload)
    assert groups["Robots"]
    assert groups["End Effectors"]
    assert groups["Objects"]
    assert "Custom STL" in groups


def test_placeholder_items_are_marked() -> None:
    payload = load_catalog(Path.cwd())
    placeholders = [i for i in payload["items"] if i["support_status"] == "placeholder"]
    assert placeholders
    assert all(i["runtime_status"] in {"placeholder", "preview_only"} for i in placeholders)
