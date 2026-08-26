from pathlib import Path

BASE_HOME = Path(
    "workcell_builder/workcell_builder/include/home_workcells_panel.hpp"
).read_text(encoding="utf-8")
TARGET = Path(
    "workcell_builder/workcell_builder/include/home_workcells_target_shell.hpp"
).read_text(encoding="utf-8")
MAIN = Path(
    "workcell_builder/workcell_builder/gui/main.cpp"
).read_text(encoding="utf-8")
LEGACY_MAIN = Path(
    "workcell_builder/workcell_builder/gui/main_legacy.inc"
).read_text(encoding="utf-8")


def test_home_shell_is_explicit_and_not_runtime_polish():
    assert "configure_target_shell(this, workspace)" in MAIN
    assert '#include "home_workcells_target_shell.hpp"' in MAIN
    for source in (TARGET, BASE_HOME):
        assert "Q_COREAPP_STARTUP_FUNCTION" not in source
        assert "installEventFilter" not in source
        assert "QTimer::singleShot" not in source
        assert "setCellWidget" not in source
        assert "deleteLater" not in source


def test_target_shell_matches_reference_information_architecture():
    for token in [
        'studioTargetSidebar',
        'studioTargetTopbar',
        'WORKCELL',
        'New Cell',
        'Home',
        'Product View',
        'Simulate',
        'Validation',
        'Export',
        'Simulation Mode   FAKE HARDWARE',
        'Real hardware   LOCKED',
        'Your workcells',
        'Total Workcells',
        'Needs Attention',
        'Search workcells',
        'Robot: All',
        'Tool: All',
        'Pinned',
        'Selected Workcell',
        'Generate Package',
        'Ready for fake-hardware simulation',
    ]:
        assert token in TARGET


def test_target_table_is_single_native_table_with_delegate_thumbnails():
    assert "class TargetWorkcellDelegate final" in TARGET
    assert "table->setItemDelegate(new TargetWorkcellDelegate" in TARGET
    assert 'QStringLiteral("Modified")' in TARGET
    assert 'QStringLiteral("★")' in TARGET
    assert 'current_valid_home_preview_path(workspace_root_, raw_id)' in TARGET
    assert 'friendly_workcell_name(raw_id)' in TARGET
    assert "setCellWidget" not in TARGET


def test_selected_workcell_panel_reuses_real_scene_actions_and_file_backed_preview():
    for token in [
        'studioTargetPreview',
        'studioTargetMetaRobot',
        'studioTargetMetaTool',
        'studioTargetMetaTask',
        'studioTargetMetaLaunch',
        'studioTargetMetaUpdated',
        'current_valid_home_preview_path(workspace_root, scene_id)',
        'Generate Scene Package',
        'studioTargetViewValidation',
    ]:
        assert token in TARGET
    assert "embeddedWeb3dProductView" not in TARGET
    assert "scene3dViewportWidget" not in TARGET


def test_inspector_has_only_backed_secondary_actions_and_no_open_cta():
    assert 'Open Scene Builder' not in TARGET
    assert 'studioTargetPrimaryAction' not in TARGET
    for forbidden in [
        'new QPushButton(QStringLiteral("◇  Product View")',
        'new QPushButton(QStringLiteral("♢  Validate")',
        'new QPushButton(QStringLiteral("▷  Simulate")',
    ]:
        assert forbidden not in TARGET
    assert "add_backed_more_action" in TARGET
    assert "proxy->setVisible(backing->isEnabled())" in TARGET
    assert "if (more_menu->actions().isEmpty()) more->hide()" in TARGET
    assert 'source_action(QStringLiteral("Delete Scene"))' in TARGET


def test_pin_column_and_canonical_label_contract():
    assert "table->setColumnWidth(5, 46)" in TARGET
    assert "star.setPixelSize(18)" in TARGET
    assert 'Qt::AlignCenter, pinned ? QStringLiteral("★") : QStringLiteral("☆")' in TARGET
    for token in [
        'lower == QStringLiteral("robotiq_2f")',
        'lower == QStringLiteral("robotiq 2f")',
        'return QStringLiteral("Robotiq 3F")',
        'lower == QStringLiteral("suction")',
        'return QStringLiteral("Single Suction")',
    ]:
        assert token in BASE_HOME


def test_home_uses_canonical_metadata_and_one_composed_filter_sort_pipeline():
    browser = Path(
        "workcell_builder/workcell_builder/src_workcell_studio_scene_browser.cpp"
    ).read_text(encoding="utf-8")
    canvas = Path(
        "workcell_builder/workcell_builder/src_workcell_studio_canvas_model.cpp"
    ).read_text(encoding="utf-8")
    mainwindow = Path(
        "workcell_builder/workcell_builder/gui/mainwindow.cpp"
    ).read_text(encoding="utf-8")

    assert "load_workcell_studio_scene_metadata_summary" in browser
    for token in [
        'snapshot_yaml(snapshot, "environment.yaml"',
        'snapshot_yaml(snapshot, "cell_definition.yaml"',
        'snapshot_yaml(snapshot, "scene_manifest.yaml"',
        'snapshot_yaml(snapshot, "config/task_recipe.yaml"',
        'snapshot_yaml(snapshot, "config/workcell_builder_task_intent.yaml"',
    ]:
        assert token in canvas
    for token in [
        "search_matches && status_matches && robot_matches && tool_matches && pinned_matches",
        "Recently modified",
        "Name A–Z",
        "Pinned first",
        "std::stable_sort",
        "refresh_filter_options",
        "refresh_metrics",
    ]:
        assert token in TARGET
    assert "currentChanged, this" in mainwindow
    assert "refresh_scene_browser_ui();" in mainwindow


def test_modified_time_excludes_generated_and_visual_activity_evidence():
    authored_section = TARGET.split("inline QDateTime scene_last_updated", 1)[1].split(
        "inline QString relative_time", 1
    )[0]
    for token in [
        'environment.yaml',
        'cell_definition.yaml',
        'scene_manifest.yaml',
        'layout/workcell_studio_layout.yaml',
        'config/workcell_builder_task_intent.yaml',
        'config/task_recipe.yaml',
    ]:
        assert token in authored_section
    for forbidden in ["smoke/", "acceptance/", "generated/scene_visual_mesh_index.json"]:
        assert forbidden not in authored_section


def test_fake_hardware_and_smoke_safety_contract_remains_intact():
    assert '--scene3d-smoke' in TARGET
    assert 'FAKE HARDWARE' in TARGET
    assert 'LOCKED' in TARGET
    assert "Scene3DSmokeRunner" in LEGACY_MAIN
    assert "--scene3d-smoke" in LEGACY_MAIN
    assert "StartupDialog startup" in LEGACY_MAIN
    assert '#include "main_legacy.inc"' in MAIN
