from pathlib import Path

HOME_V5 = Path(
    "workcell_builder/workcell_builder/include/workcell_home_polish_v5.hpp"
).read_text(encoding="utf-8")
STARTUP = Path(
    "workcell_builder/workcell_builder/gui/startup_dialog.cpp"
).read_text(encoding="utf-8")


def test_v5_is_quarantined_from_startup_after_workstation_bus_error():
    assert '#include "workcell_home_polish_v3.hpp"' in STARTUP
    assert '#include "workcell_home_polish_v4.hpp"' not in STARTUP
    assert '#include "workcell_home_polish_v5.hpp"' not in STARTUP
    assert '#include "workcell_home_polish_v3.hpp"' in HOME_V5


def test_v5_never_removes_or_deletes_legacy_table_cell_widgets():
    assert 'hideLegacyCellWidgets' in HOME_V5
    assert 'widget->hide()' in HOME_V5
    assert 'removeCellWidget' not in HOME_V5
    assert 'deleteLater' not in HOME_V5
    assert 'Never remove/delete cell widgets here' in HOME_V5


def test_v5_keeps_delegate_rendering_and_canonical_table_data_separate():
    for token in [
        'class HomeTableDelegate final',
        'table->setItemDelegate(new HomeTableDelegate(table))',
        'table->setColumnHidden(kTaskColumn, true)',
        'table->setColumnHidden(kLaunchColumn, true)',
        'QStringLiteral("Workcell")',
        'QStringLiteral("Status")',
        'QStringLiteral("Robot")',
        'QStringLiteral("Tool / Gripper")',
        'QStringLiteral("Updated")',
        'QStringLiteral("Pinned")',
    ]:
        assert token in HOME_V5


def test_v5_preview_is_non_blocking_and_has_explicit_fallback():
    assert 'cachedPreviewPath' in HOME_V5
    assert 'liveCanonicalPreview' in HOME_V5
    assert 'NO PREVIEW GENERATED' in HOME_V5
    assert 'PREVIEW PREPARING' not in HOME_V5
