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
        assert f"connect({action}, &QAction::triggered" in text

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
