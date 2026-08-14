from pathlib import Path
import re


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
        assert f"connect_action({action}," in text

    assert "has_package_files" not in text
    assert "has_launch_file" not in text
    assert "entry.warning)" not in text
    assert "entry.warning;" not in text
    assert "entry.warnings" in text


def test_create_editable_layout_recommended_action_handler_tokens():
    header = Path("workcell_builder/workcell_builder/gui/mainwindow.h").read_text(encoding="utf-8")
    source = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")

    enum_match = re.search(
        r"enum class RecommendedWorkflowActionHandler \{(?P<body>.*?)\n  \};",
        header,
        re.DOTALL,
    )
    assert enum_match is not None
    assert "CreateEditableLayoutFromPreview" in enum_match.group("body")

    trigger_match = re.search(
        r"case RecommendedWorkflowActionHandler::CreateEditableLayoutFromPreview:\s*"
        r"create_starter_layout_from_preview\(\);\s*"
        r"return;",
        source,
        re.DOTALL,
    )
    assert trigger_match is not None

    create_editable_action_blocks = re.findall(
        r"add_action\(\s*"
        r"\"create_editable_layout_from_preview\"\s*,\s*"
        r"\"Create editable layout from preview\"(?P<body>.*?)\);",
        source,
        re.DOTALL,
    )
    assert create_editable_action_blocks
    assert all(
        "RecommendedWorkflowActionHandler::CreateEditableLayoutFromPreview" in block
        for block in create_editable_action_blocks
    )


def test_save_layout_success_refreshes_workflow_rail_and_scene_chips_after_writes():
    source = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")

    save_match = re.search(
        r"bool MainWindow::save_native_layout_changes\([^)]*\)\s*\{(?P<body>.*?)\n\}\n\nvoid MainWindow::create_starter_layout_from_preview",
        source,
        re.DOTALL,
    )
    assert save_match is not None
    body = save_match.group("body")

    write_layout_pos = body.find('"layout" / "workcell_studio_layout.yaml"')
    close_layout_pos = body.find("out.commit()", write_layout_pos)
    write_environment_pos = body.find("refresh_scene_builder_left_explorer();")
    close_environment_pos = write_environment_pos
    refresh_browser_pos = body.find("refresh_scene_browser_ui();")
    refresh_workflow_pos = body.find("refresh_scene_workflow_rail();")
    refresh_chips_pos = body.find("refresh_scene_builder_view_chips();")

    assert write_layout_pos != -1
    assert close_layout_pos != -1
    assert write_environment_pos != -1
    assert close_environment_pos != -1
    assert refresh_browser_pos != -1
    assert refresh_workflow_pos != -1
    assert refresh_chips_pos != -1
    assert write_layout_pos < close_layout_pos < write_environment_pos
    assert write_environment_pos <= close_environment_pos < refresh_browser_pos
    assert refresh_browser_pos < refresh_workflow_pos < refresh_chips_pos

def test_save_layout_empty_canvas_preserves_existing_canonical_items():
    source = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    save_match = re.search(
        r"void MainWindow::save_layout_changes\(\)\{(?P<body>.*?)\n\}\n\nvoid MainWindow::create_starter_layout_from_preview",
        source,
        re.DOTALL,
    )
    assert save_match is not None
    body = save_match.group("body")

    assert 'scene_dir / "layout" / "workcell_studio_layout.yaml"' in body
    assert "QSaveFile out(QString::fromStdString(effective_layout_path.string()));" in body
    assert "out.commit()" in body
    assert "Save Layout: no editable items; saved canonical layout metadata to %1." in body
    assert "Use Create editable layout from preview or add an item to persist editable objects." in body
    assert "gi->data(RoleLocked).toBool()" in body
    assert 'source_layer != QStringLiteral("editable_layout")' in body
    assert "empty_layout_marker" not in body
    assert "root = YAML::Node(YAML::NodeType::Map);" not in body[body.find("std::vector<QGraphicsItem *> editable_canvas_items"):]


def test_save_layout_roundtrip_keeps_locked_canonical_item_unchanged_and_updates_editable_by_id():
    source = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    save_match = re.search(
        r"void MainWindow::save_layout_changes\(\)\{(?P<body>.*?)\n\}\n\nvoid MainWindow::create_starter_layout_from_preview",
        source,
        re.DOTALL,
    )
    assert save_match is not None
    body = save_match.group("body")

    canonical_branch = re.search(
        r"if \(saving_workcell_layout\) \{(?P<branch>.*?)\n  \} else if \(saving_placed_assets_layout\)",
        body,
        re.DOTALL,
    )
    assert canonical_branch is not None
    branch = canonical_branch.group("branch")
    assert 'YAML::Node existing_by_id = existing_items_by_id(root["items"]);' in branch
    assert 'YAML::Node existing_items = root["items"];' in branch
    assert 'editable_by_id.contains(id)' in branch
    assert 'serialized_editable_canvas_item(editable_by_id.value(id), existing_item)' in branch
    assert 'updated_placed.push_back(YAML::Clone(existing_item));' in branch
    assert 'if (saved_ids.contains(id)) continue;' in branch
    assert 'root["items"] = updated_placed;' in branch


def test_serialized_editable_canvas_item_preserves_existing_editable_locked_flags():
    source = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    serializer_match = re.search(
        r"static YAML::Node serialized_editable_canvas_item\(QGraphicsItem \* gi, const YAML::Node & existing\)\n\{(?P<body>.*?)\n\}",
        source,
        re.DOTALL,
    )
    assert serializer_match is not None
    serializer = serializer_match.group("body")
    assert "serialize_layout_item(" in serializer
    authority = Path("workcell_builder/workcell_builder/gui/layout_item_serializer.hpp").read_text(encoding="utf-8")
    assert "existing_record ? YAML::Clone(existing)" in authority
    new_record_branch = authority.split("if (!existing_record) {", 1)[1].split(
        "update_layout_item_pose", 1
    )[0]
    assert 'item["editable"] = true;' in new_record_branch
    assert 'item["locked"] = false;' in new_record_branch
