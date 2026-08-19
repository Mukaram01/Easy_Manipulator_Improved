from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "workcell_builder/workcell_builder/gui/main.cpp"
LAYOUT = ROOT / "workcell_builder/workcell_builder/gui/scene_builder_product_layout.hpp"


def text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_product_layout_is_applied_explicitly_from_mainwindow_wrapper():
    main = text(MAIN)
    assert '#include "gui/scene_builder_product_layout.hpp"' in main
    assert "configure_scene_builder_product_layout(this)" in main


def test_product_layout_has_one_primary_scene_toolbar():
    source = text(LAYOUT)
    for token in (
        'QStringLiteral("Select")',
        'QStringLiteral("Move")',
        'QStringLiteral("Rotate")',
        'QStringLiteral("Place Asset")',
        'QStringLiteral("Add Object")',
        'QStringLiteral("Fit")',
        'QStringLiteral("Undo")',
        'QStringLiteral("Redo")',
        'QStringLiteral("Save Layout")',
        'overflow->setText(QStringLiteral("View"))',
        'sceneBuilderEmbeddedPreviewChrome',
        'previewToolbarChip',
    ):
        assert token in source


def test_product_layout_promotes_major_tabs_and_simplifies_hierarchy():
    source = text(LAYOUT)
    for token in (
        'sceneBuilderMajorTabs',
        'QStringLiteral("Scene")',
        'QStringLiteral("Assets")',
        'QStringLiteral("Workflow")',
        'source_tabs->tabBar()->hide()',
        'sceneBuilderHierarchySearch',
        'tree->setHeaderHidden(true)',
        'tree->setColumnHidden(1, true)',
        'tree->setColumnHidden(2, true)',
        'studioSelectedItemCard',
        'selected_card->hide()',
        'sceneBuilderLayersGroup',
        'group->setChecked(false)',
    ):
        assert token in source


def test_product_layout_prioritizes_the_3d_viewport():
    source = text(LAYOUT)
    for token in (
        'left->setMinimumWidth(245)',
        'left->setMaximumWidth(300)',
        'center->setMinimumWidth(760)',
        'right->setMinimumWidth(300)',
        'right->setMaximumWidth(360)',
        'splitter->setStretchFactor(1, 10)',
        'splitter->setSizes({270, 1120, 320})',
    ):
        assert token in source


def test_product_layout_removes_redundant_scene_and_debug_chrome():
    source = text(LAYOUT)
    for token in (
        'sceneBuilderCompactSceneIdentity',
        'text.startsWith(QStringLiteral("Scene Builder:"))',
        'text.startsWith(QStringLiteral("Unsaved Layout Edits:"))',
        'text.startsWith(QStringLiteral("Legend:"))',
        'text.startsWith(QStringLiteral("Scene load:"))',
        'sceneBuilderLatestStatus',
        'digital_twin_minimap',
    ):
        assert token in source


def test_product_layout_keeps_one_contextual_inspector_surface():
    source = text(LAYOUT)
    for token in (
        'QStringLiteral("Selection"), QStringLiteral("Workflow"), QStringLiteral("Readiness")',
        'tabs->setTabText(0, QStringLiteral("Inspector"))',
        'tabs->setTabText(1, QStringLiteral("Task"))',
        'tabs->setTabText(2, QStringLiteral("Checks"))',
        'card->hide()',
        'scene_builder/right_panel_visible',
    ):
        assert token in source


def test_product_layout_does_not_reintroduce_runtime_polish_or_motion_paths():
    source = text(LAYOUT)
    forbidden = (
        "Q_COREAPP_STARTUP_FUNCTION",
        "QTimer",
        "installEventFilter",
        "QApplication::topLevelWidgets",
        "setParent(",
        "QWebEngineView",
        "execute_trajectory",
        "real_hardware_enabled",
        "ros2 launch",
        'environment.yaml").write',
        'scene_manifest.yaml").write',
    )
    for token in forbidden:
        assert token not in source
