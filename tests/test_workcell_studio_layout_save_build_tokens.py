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
    body = _extract_function_body(cpp, "void MainWindow::save_layout_changes()")
    assert '"environment_layout."' in body
    assert '".bak.yaml"' in body
    assert "append_studio_log" in body


def test_no_preview_regressions_for_known_tokens():
    mainwindow_cpp = MAINWINDOW_CPP.read_text(encoding="utf-8")
    scene_preview_cpp = SCENE_PREVIEW_CPP.read_text(encoding="utf-8")
    assert "QPolygonF{" not in scene_preview_cpp
    assert "select_preview_item(item->" not in mainwindow_cpp
