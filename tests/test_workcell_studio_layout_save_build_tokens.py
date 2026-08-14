from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
MAINWINDOW_CPP = REPO_ROOT / "workcell_builder" / "workcell_builder" / "gui" / "mainwindow.cpp"
SCENE_PREVIEW_CPP = REPO_ROOT / "workcell_builder" / "workcell_builder" / "gui" / "scene_preview_widget.cpp"


def _extract_function_body(text: str, signature: str) -> str:
    start = text.find(signature)
    assert start != -1, f"Signature not found: {signature}"
    brace_start = text.find("{", start)
    assert brace_start != -1, f"Opening brace not found for: {signature}"
    depth = 0
    for idx in range(brace_start, len(text)):
        ch = text[idx]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return text[brace_start + 1 : idx]
    raise AssertionError(f"Closing brace not found for: {signature}")


def test_minimal_environment_layout_has_no_backup_or_studio_log_calls():
    cpp = MAINWINDOW_CPP.read_text(encoding="utf-8")
    body = _extract_function_body(cpp, "static YAML::Node minimal_environment_layout")
    assert "layout_path" not in body
    assert "append_studio_log" not in body


def test_save_layout_changes_contains_backup_before_write_tokens():
    cpp = MAINWINDOW_CPP.read_text(encoding="utf-8")
    wrapper = _extract_function_body(cpp, "void MainWindow::save_layout_changes()")
    assert "save_native_layout_changes(QJsonObject{})" in wrapper
    body = _extract_function_body(cpp, "bool MainWindow::save_native_layout_changes")
    assert 'layout_backup' in body
    assert '".bak.yaml"' in body
    assert "append_studio_log" in body


def test_no_preview_regressions_for_known_tokens():
    mainwindow_cpp = MAINWINDOW_CPP.read_text(encoding="utf-8")
    scene_preview_cpp = SCENE_PREVIEW_CPP.read_text(encoding="utf-8")
    assert "QPolygonF{" not in scene_preview_cpp
    assert "select_preview_item(item->data(RoleId).toString().trimmed())" in mainwindow_cpp


def test_save_layout_failure_feedback_contract_tokens():
    cpp = MAINWINDOW_CPP.read_text(encoding="utf-8")
    body = _extract_function_body(cpp, "bool MainWindow::save_native_layout_changes")

    required_tokens = [
        "Save Layout failed: Scene3D canvas is not initialized; no file was written.",
        "Save Layout failed: no scene selected; no file was written.",
        "canonical layout path could not be computed for scene_root=%1 selected_scene_index=%2",
        "Save Layout failed: action=Save Layout scene=%1 selected_scene_index=%2 scene_root=%3 target=%4 blocker=%5; no file was written.",
        "Save Layout blocked: Scene3D canvas is not initialized; no file was written.",
        "Save Layout blocked: no scene selected; no file was written.",
        "Save Layout blocked: canonical layout path could not be computed; no file was written.",
        "cannot create required directory",
        "malformed layout YAML at %1 and backup failed (%2)",
        "invalid id '%1' for YAML/package compatibility",
    ]
    for token in required_tokens:
        assert token in body

    assert "if (!digital_twin_scene_) return;" not in body
    assert "if (layout_path.empty()) return;" not in body
