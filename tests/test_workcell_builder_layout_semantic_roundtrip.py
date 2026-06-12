import re
from pathlib import Path

import yaml


REPO = Path(__file__).resolve().parents[1]
MAINWINDOW_CPP = REPO / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
UR5_LAYOUT = REPO / "scenes/ur5_2f_test/layout/workcell_studio_layout.yaml"


def _role_category_entry_field() -> str:
    src = MAINWINDOW_CPP.read_text(encoding="utf-8")
    match = re.search(
        r"item->setData\(RoleCategory,\s*QString::fromStdString\(entry\.([a-zA-Z_][a-zA-Z0-9_]*)\)\);",
        src,
    )
    assert match, "MainWindow canvas rebuild must set RoleCategory from a canvas model entry field."
    return match.group(1)


def _simulate_canvas_load_save_reload(item: dict, role_category_entry_field: str) -> dict:
    """Mirror the metadata path used by rebuild_digital_twin_canvas() and serialized_editable_canvas_item()."""
    canvas_entry = {
        "id": item.get("id", ""),
        "type": item.get("type", ""),
        "role": item.get("role", ""),
        "category": item.get("category", ""),
    }
    canvas_roles = {
        "RoleId": canvas_entry["id"],
        "RoleType": canvas_entry["type"],
        "RoleRole": canvas_entry["role"],
        "RoleCategory": canvas_entry[role_category_entry_field],
    }

    saved = dict(item)
    saved["id"] = canvas_roles["RoleId"]
    if canvas_roles["RoleCategory"]:
        saved["category"] = canvas_roles["RoleCategory"]
    if canvas_roles["RoleType"]:
        saved["type"] = canvas_roles["RoleType"]
    if canvas_roles["RoleRole"]:
        saved["role"] = canvas_roles["RoleRole"]

    # Simulate the reload path by reading the semantic fields back from saved YAML.
    return {
        "id": saved.get("id"),
        "type": saved.get("type"),
        "role": saved.get("role"),
        "category": saved.get("category"),
    }


def test_ur5_2f_semantic_layout_items_keep_metadata_after_canvas_roundtrip():
    layout = yaml.safe_load(UR5_LAYOUT.read_text(encoding="utf-8"))
    semantic_items = [
        item
        for item in layout["items"]
        if all(item.get(field) for field in ("id", "type", "role", "category"))
    ]
    assert semantic_items, "ur5_2f_test layout must contain semantic editable items."

    role_category_entry_field = _role_category_entry_field()
    assert role_category_entry_field == "category"

    for original in semantic_items:
        reloaded = _simulate_canvas_load_save_reload(original, role_category_entry_field)
        expected = {field: original[field] for field in ("id", "type", "role", "category")}
        assert reloaded == expected
