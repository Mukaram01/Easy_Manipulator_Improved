from pathlib import Path


def test_mainwindow_build_regression_tokens():
    text = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")

    # Avoid broken QString literal form in linked hierarchy label update.
    assert 'QString("\nLinked hierarchy item: %1")' not in text
    assert 'QStringLiteral("\\nLinked hierarchy item: %1")' in text

    assert "Q_UNUSED(int)" not in text

    stale_buttons = [
        "open_asset_folder_button",
        "copy_asset_path_button",
        "import_asset_button",
        "add_existing_stl_button",
        "placeholder_button",
    ]
    for token in stale_buttons:
        assert token not in text

    for action in [
        "open_asset_folder_action",
        "copy_asset_path_action",
        "import_asset_action",
        "add_existing_stl_action",
        "placeholder_action",
    ]:
        assert f"connect({action}, &QAction::triggered" in text

    assert "has_package_files" not in text
    assert "has_launch_file" not in text
    assert "entry.warning)" not in text
    assert "entry.warning;" not in text
    assert "entry.warnings" in text


def test_save_layout_success_refreshes_scene_builder_workflow_after_writes():
    text = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    start = text.index("void MainWindow::save_layout_changes()")
    end = text.index("void MainWindow::create_starter_layout_from_preview()", start)
    body = text[start:end]

    workcell_write = body.index('"Save Layout: wrote editable layout items to %1"')
    environment_write = body.index('"Save Layout: wrote environment metadata to %1"')
    browser_refresh = body.index("refresh_scene_browser_ui();")
    workflow_refresh = body.index("refresh_scene_workflow_rail();", browser_refresh)
    chip_refresh = body.index("refresh_scene_builder_view_chips();", workflow_refresh)

    assert workcell_write < environment_write < browser_refresh < workflow_refresh < chip_refresh
