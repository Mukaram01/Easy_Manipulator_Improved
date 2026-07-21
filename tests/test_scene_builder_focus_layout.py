from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BOOTSTRAP = (
    ROOT
    / "workcell_builder/workcell_builder/gui/scene_builder_focus_layout_bootstrap.hpp"
)
UI_UTILS = (
    ROOT
    / "workcell_builder/workcell_builder/include/workcell_builder_ui_utils.hpp"
)


def test_focus_layout_bootstrap_is_installed_in_workcell_builder():
    ui_utils = UI_UTILS.read_text(encoding="utf-8")
    source = BOOTSTRAP.read_text(encoding="utf-8")

    assert '#include "scene_builder_focus_layout_bootstrap.hpp"' in ui_utils
    assert "Q_COREAPP_STARTUP_FUNCTION(workcellBuilderBootstrapSceneBuilderFocusLayout)" in source
    assert "installSceneBuilderFocusLayout(window)" in source


def test_scene_builder_defaults_to_a_center_first_readable_workspace():
    source = BOOTSTRAP.read_text(encoding="utf-8")

    for token in (
        'QStringLiteral("sceneBuilderWorkspace")',
        'QStringLiteral("sceneBuilderMainSplitter")',
        'QStringLiteral("sceneBuilderLeftPanel")',
        'QStringLiteral("sceneBuilderRightPanel")',
        'QStringLiteral("assetLibrarySearchBox")',
        'QStringLiteral("studioSceneHierarchyTree")',
        "assets_layout->addWidget(asset_card, 1)",
        "workflow_layout->insertWidget(0, workflow_card)",
        "right_panel->setMinimumWidth(360)",
        'button->setText(QStringLiteral("Panels & tools"))',
        'QStringLiteral("scene_builder/focus_layout_version")',
        'QStringLiteral("scene_builder/right_panel_visible"), false',
        "splitter->setSizes({320, center_width, 0})",
    ):
        assert token in source


def test_duplicate_scene_header_actions_and_redundant_labels_are_hidden():
    source = BOOTSTRAP.read_text(encoding="utf-8")

    for object_name in (
        "sceneBuilderHeaderFilesButton",
        "sceneBuilderHeaderSaveLayoutButton",
        "sceneBuilderHeaderRunNextButton",
    ):
        assert object_name in source
    assert "duplicate->hide()" in source
    assert 'text.startsWith(QStringLiteral("Legend:"))' in source
    assert 'text.contains(QStringLiteral("Scene3D Product Preview"))' in source


def test_inspector_is_opened_by_an_intentional_scene_selection():
    source = BOOTSTRAP.read_text(encoding="utf-8")

    assert "QTreeWidget::itemClicked" in source
    assert "ScenePreviewWidget::preview_item_selected" in source
    assert "QApplication::mouseButtons() != Qt::NoButton" in source
    assert 'tabIndexByText(right_tabs, QStringLiteral("Selection"))' in source
    assert "show_right->setChecked(true)" in source


def test_focus_layout_change_does_not_add_motion_or_source_writes():
    combined = "\n".join(
        path.read_text(encoding="utf-8").lower() for path in (BOOTSTRAP, UI_UTILS)
    )
    for forbidden in (
        "ros2 launch",
        "execute_trajectory",
        "move_group",
        "real_hardware_enabled",
        "environment.yaml").write",
        "scene_manifest.yaml").write",
    ):
        assert forbidden not in combined


def test_focus_layout_bootstrap_remains_small_and_layered():
    source = BOOTSTRAP.read_text(encoding="utf-8")
    assert len(source.splitlines()) < 280
    assert "mainwindow.cpp" not in source
