from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
LIVE_TEST = ROOT / "tests/test_live_authoring_incremental_refresh.py"


def function_span(text: str, signature: str) -> tuple[int, int]:
    start = text.index(signature)
    brace = text.index("{", start)
    depth = 0
    for i in range(brace, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return start, i + 1
    raise SystemExit(f"unterminated function: {signature}")


def patch_function(text: str, signature: str, transform) -> str:
    start, end = function_span(text, signature)
    original = text[start:end]
    patched = transform(original)
    if patched == original:
        raise SystemExit(f"{signature}: patch made no change")
    return text[:start] + patched + text[end:]


def remove_full_refresh(body: str, label: str) -> str:
    needle = "  refresh_scene_builder_left_explorer();\n"
    count = body.count(needle)
    if count != 1:
        raise SystemExit(f"{label}: expected one full-refresh call, found {count}")
    return body.replace(needle, "", 1)


source = MAIN.read_text(encoding="utf-8")


def patch_duplicate(body: str) -> str:
    body = remove_full_refresh(body, "duplicate")
    hierarchy = "  refresh_scene_hierarchy_tree_from_current_items();\n"
    selection = "  apply_scene_selection(new_id, copy.role, false, false);\n"
    if body.count(hierarchy) != 1 or body.count(selection) != 1:
        raise SystemExit("duplicate: expected one hierarchy refresh and one copy selection")

    # Selection must be authoritative before rebuilding the hierarchy rows so the
    # newly-created row is immediately selected. Remove the stale workaround
    # comment that described the old full-refresh ordering.
    stale_comment = "  // Prevent refresh-time browser state from overriding the live duplicate selection.\n"
    body = body.replace(stale_comment, "")
    body = body.replace(hierarchy, "", 1)
    body = body.replace(selection, "", 1)

    anchor = "  mark_layout_dirty(\"Duplicate Selected\");\n"
    if body.count(anchor) != 1:
        raise SystemExit("duplicate: dirty-state anchor not found exactly once")
    replacement = (
        "  // The duplicate already exists in the live authored session and Product View.\n"
        "  // Do not rebuild from the saved scene: preserve unsaved state and selection.\n"
        + selection
        + hierarchy
        + anchor
    )
    return body.replace(anchor, replacement, 1)


source = patch_function(source, "void MainWindow::duplicate_selected_item()", patch_duplicate)
source = patch_function(source, "void MainWindow::delete_selected_item()", lambda b: remove_full_refresh(b, "delete"))
source = patch_function(source, "void MainWindow::undo_layout_edit()", lambda b: remove_full_refresh(b, "undo"))
source = patch_function(source, "void MainWindow::redo_layout_edit()", lambda b: remove_full_refresh(b, "redo"))
MAIN.write_text(source, encoding="utf-8")

LIVE_TEST.write_text(r'''from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")


def function_body(signature: str) -> str:
    start = MAIN.index(signature)
    brace = MAIN.index("{", start)
    depth = 0
    for i in range(brace, len(MAIN)):
        if MAIN[i] == "{":
            depth += 1
        elif MAIN[i] == "}":
            depth -= 1
            if depth == 0:
                return MAIN[start:i + 1]
    raise AssertionError(f"unterminated function: {signature}")


def test_duplicate_keeps_live_session_and_hierarchy_in_sync():
    body = function_body("void MainWindow::duplicate_selected_item()")
    assert "duplicate_authoring_item(target.state.id, copy)" in body
    assert "refresh_scene_builder_left_explorer();" not in body
    assert "apply_scene_selection(new_id, copy.role, false, false);" in body
    assert "refresh_scene_hierarchy_tree_from_current_items();" in body
    assert body.index("apply_scene_selection(new_id, copy.role, false, false);") < body.index(
        "refresh_scene_hierarchy_tree_from_current_items();"
    )


def test_delete_stays_incremental():
    body = function_body("void MainWindow::delete_selected_item()")
    assert "remove_authoring_item(id)" in body
    assert "refresh_scene_hierarchy_tree_from_current_items();" in body
    assert "refresh_scene_builder_left_explorer();" not in body


def test_undo_and_redo_stay_incremental():
    undo = function_body("void MainWindow::undo_layout_edit()")
    redo = function_body("void MainWindow::redo_layout_edit()")
    for body in (undo, redo):
        assert "refresh_scene_hierarchy_tree_from_current_items();" in body
        assert "refresh_scene_builder_left_explorer();" not in body


def test_explicit_full_scene_refresh_is_still_available():
    body = function_body("void MainWindow::refresh_scene_builder_left_explorer()")
    assert "sync_selected_scene_state();" in body
    assert "rebuild_digital_twin_canvas();" in body
    assert "populate_scene_hierarchy();" in body
    assert "populate_asset_catalog();" in body
''', encoding="utf-8")
