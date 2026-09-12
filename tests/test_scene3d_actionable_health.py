"""Keep scene health independent of visibility and informational lock state."""
from pathlib import Path

GUI = Path(__file__).resolve().parents[1] / 'workcell_builder/workcell_builder/gui'


def test_health_counts_hidden_issues_without_treating_expected_helpers_as_errors():
    main = (GUI / 'mainwindow.cpp').read_text()
    counts = main.split('int classified_overlay_count = 0;', 1)[1].split(
        'bool editable_layout_yaml_malformed', 1)[0]
    assert 'if (!preview_item_visible_for_active_layers(item)) continue;' not in counts
    assert 'has_actionable_visual_warning(item)' in counts
    gate = main.split('const bool visual_quality_needs_review =', 1)[1].split(';', 1)[0]
    assert 'classified_overlay_count' not in gate
    assert 'scene3d_clean_product_view_' not in gate
    assert 'classified_warning_count > 0' in gate
    assert '!scene3d_counters.visual_quality_warnings.isEmpty()' in gate


def test_lock_reason_stays_metadata_and_temporary_warning_trace_is_removed():
    main = (GUI / 'mainwindow.cpp').read_text()
    assert 'p.lock_reason = base_reason;' in main
    assert 'p.warnings << QStringLiteral("Locked: %1")' not in main
    assert 'Scene3D actionable warning item:' not in main
