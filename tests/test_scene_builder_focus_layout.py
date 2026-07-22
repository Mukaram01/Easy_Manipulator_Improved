from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAINWINDOW = ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
UI_UTILS = ROOT / "workcell_builder/workcell_builder/include/workcell_builder_ui_utils.hpp"
BOOTSTRAP = ROOT / "workcell_builder/workcell_builder/gui/scene_builder_focus_layout_bootstrap.hpp"


def read_main():
    return MAINWINDOW.read_text(encoding="utf-8")


def test_focus_layout_bootstrap_is_retired_from_workcell_builder():
    ui_utils = UI_UTILS.read_text(encoding="utf-8")
    source = read_main()

    assert not BOOTSTRAP.exists()
    assert 'scene_builder_focus_layout_bootstrap.hpp' not in ui_utils
    assert 'Q_COREAPP_STARTUP_FUNCTION(workcellBuilderBootstrapSceneBuilderFocusLayout)' not in source
    assert 'installSceneBuilderFocusLayout(window)' not in source


def test_scene_builder_builds_center_first_readable_workspace_natively():
    source = read_main()

    for token in (
        'scene_shell->setObjectName("sceneBuilderWorkspace")',
        'scene_splitter->setObjectName("sceneBuilderMainSplitter")',
        'left_panel->setObjectName("sceneBuilderLeftPanel")',
        'center_panel->setObjectName("sceneBuilderProductViewPanel")',
        'right_panel->setObjectName("sceneBuilderRightPanel")',
        'left_panel->setMinimumWidth(320)',
        'center_panel->setMinimumWidth(720)',
        'right_panel->setMinimumWidth(360)',
        'scene_splitter->setStretchFactor(1, 8)',
        'scene_splitter->setSizes({320, 1000, 0})',
        'scene_builder/native_layout_version',
        'settings.setValue(QStringLiteral("scene_builder/right_panel_visible"), false)',
    ):
        assert token in source


def test_left_panel_tabs_are_scene_assets_workflow_and_assets_are_native():
    source = read_main()

    assert 'scene_builder_left_tabs_->addTab(scene_tab, "Scene")' in source
    assert 'scene_builder_left_tabs_->addTab(assets_tab, "Assets")' in source
    assert 'scene_builder_left_tabs_->addTab(files_tab, "Workflow")' in source
    assert 'assets_tab_layout->addWidget(catalog_card, 1)' in source
    assert 'Asset Library is available below Scene Hierarchy' not in source


def test_duplicate_scene_header_actions_are_removed_but_actions_remain_accessible():
    source = read_main()

    for object_name in (
        "sceneBuilderHeaderFilesButton",
        "sceneBuilderHeaderSaveLayoutButton",
        "sceneBuilderHeaderRunNextButton",
    ):
        assert object_name not in source
    assert 'scenes_open_button->setText("Scenes")' in source
    assert 'run_next_button->setText("Run Next")' in source
    assert 'register_scene_action("layout.save", "Save Layout"' in source


def test_inspector_is_closed_by_default_and_opens_on_selection():
    source = read_main()

    assert 'right_panel->setVisible(false)' in source
    assert 'scene_builder_right_panel_->setVisible(true)' in source
    assert 'scene_builder_show_right_panel_action_->setChecked(true)' in source
    assert 'scene_builder_inspector_tabs_->setCurrentIndex(selection_tab)' in source
    assert 'connect(scene_preview_widget_, &ScenePreviewWidget::preview_item_selected' in source
    assert 'connect(scene_hierarchy_tree_, &QTreeWidget::itemClicked' in source


def test_no_repeating_layout_timer_or_runtime_reparenting():
    combined = "\n".join(
        path.read_text(encoding="utf-8") for path in (MAINWINDOW, UI_UTILS)
    )
    assert 'QTimer::timeout, qApp' not in combined
    assert 'QApplication::topLevelWidgets()' not in combined
    assert 'setParent(' not in combined
    assert 'addWidget(asset_card, 1)' not in combined
    assert 'insertWidget(0, workflow_card)' not in combined


def test_panels_tools_label_is_well_formed():
    source = read_main()
    assert 'scene_builder_secondary_overflow_button_->setText("Panels & Tools")' in source
    assert 'Panels_tools' not in source
    assert 'Panels & tools' not in source


def test_focus_layout_change_does_not_add_motion_or_source_writes():
    combined = "\n".join(
        path.read_text(encoding="utf-8").lower() for path in (MAINWINDOW, UI_UTILS)
    )
    for forbidden in (
        "execute_trajectory",
        "real_hardware_enabled",
        'environment.yaml").write',
        'scene_manifest.yaml").write',
    ):
        assert forbidden not in combined
