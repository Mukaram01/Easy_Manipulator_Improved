from pathlib import Path

HOME = Path(
    "workcell_builder/workcell_builder/include/home_workcells_panel.hpp"
).read_text(encoding="utf-8")
MAIN = Path(
    "workcell_builder/workcell_builder/gui/main.cpp"
).read_text(encoding="utf-8")
LEGACY_MAIN = Path(
    "workcell_builder/workcell_builder/gui/main_legacy.inc"
).read_text(encoding="utf-8")


def test_home_layout_is_explicit_and_not_a_runtime_polish_guard():
    assert "home_workcells::configure(this, workspace)" in MAIN
    assert "Q_COREAPP_STARTUP_FUNCTION" not in HOME
    assert "installEventFilter" not in HOME
    assert "QTimer::singleShot" not in HOME
    assert "removeCellWidget" not in HOME
    assert "deleteLater" not in HOME


def test_home_layout_focuses_one_workcell_library_and_details_panel():
    for token in [
        'legacy_library_card(window)',
        'legacy->hide()',
        'QStringLiteral("<b>Workcells</b>")',
        'QStringLiteral("Workcell")',
        'QStringLiteral("Tool / Gripper")',
        'QStringLiteral("WORKCELL DETAILS")',
        'studioHomeScenePreview',
        'studioHomeReadinessSummary',
    ]:
        assert token in HOME


def test_home_table_has_one_delegate_and_no_cell_widget_overlays():
    assert "class WorkcellTableDelegate final" in HOME
    assert "table->setItemDelegate(new WorkcellTableDelegate(table))" in HOME
    assert "setCellWidget" not in HOME
    assert "Robotiq 2F-85" in HOME
    assert "Single Suction" in HOME
    assert "Needs Attention" in HOME


def test_home_preview_is_file_backed_and_never_force_loads_hidden_3d():
    for token in [
        "src/easy_manipulation_deployment/scenes",
        "src/scenes",
        "smoke/scene3d_gui_smoke.png",
        "acceptance/scene3d_gui_smoke.png",
        "preview_launch/product_view.png",
        "NO PREVIEW IMAGE",
    ]:
        assert token in HOME
    assert "embeddedWeb3dProductView" not in HOME
    assert "scene3dViewportWidget" not in HOME


def test_existing_entrypoint_is_preserved_without_rewriting_smoke_workflow():
    assert "Scene3DSmokeRunner" in LEGACY_MAIN
    assert "--scene3d-smoke" in LEGACY_MAIN
    assert "StartupDialog startup" in LEGACY_MAIN
    assert '#include "main_legacy.inc"' in MAIN
