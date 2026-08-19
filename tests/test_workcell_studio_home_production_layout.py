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
        'Open in Scene Builder',
        'Generate Package',
        'Simulate · Fake Hardware',
    ]:
        assert token in TARGET


def test_target_table_is_single_native_table_with_delegate_thumbnails():
    assert "class TargetWorkcellDelegate final" in TARGET
    assert "table->setItemDelegate(new TargetWorkcellDelegate" in TARGET
    assert 'QStringLiteral("Updated")' in TARGET
    assert 'QStringLiteral("Pinned")' in TARGET
    assert 'find_preview_path(workspace_root_, raw_id)' in TARGET
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
        'find_preview_path(workspace_root, scene_id)',
        'Plan / Simulate',
        'Generate Scene Package',
    ]:
        assert token in TARGET
    assert "embeddedWeb3dProductView" not in TARGET
    assert "scene3dViewportWidget" not in TARGET


def test_fake_hardware_and_smoke_safety_contract_remains_intact():
    assert '--scene3d-smoke' in TARGET
    assert 'FAKE HARDWARE' in TARGET
    assert 'LOCKED' in TARGET
    assert "Scene3DSmokeRunner" in LEGACY_MAIN
    assert "--scene3d-smoke" in LEGACY_MAIN
    assert "StartupDialog startup" in LEGACY_MAIN
    assert '#include "main_legacy.inc"' in MAIN
