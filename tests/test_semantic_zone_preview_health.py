"""Focused preview-health contracts; no generated-scene or runtime execution."""
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
GUI = ROOT / 'workcell_builder/workcell_builder/gui'


def test_semantic_helper_identity_reaches_overlay_classification():
    main = (GUI / 'mainwindow.cpp').read_text()
    preview = (GUI / 'scene_preview_widget.cpp').read_text()
    assert 'preview.semantic_task_zone_helper = item.semantic_task_zone_helper;' in preview
    assert 'if (p.semantic_task_zone_helper) p.active_visual_source = QStringLiteral("semantic_primitive");' in main
    assert 'p.semantic_task_zone_helper = concept == QStringLiteral("pick_zone")' in main
    assert 'const bool is_overlay = item.semantic_task_zone_helper ||' in main
    assert 'if (is_overlay) ++classified_overlay_count;' in main
    # A helper carrying a real mesh warning still enters the warning bucket.
    assert 'has_actionable_visual_warning(item)' in main
    assert 'Scene3D actionable warning item:' not in main
    assert 'p.warnings << QStringLiteral("Locked: %1")' not in main


def test_expected_overlays_alone_leave_preview_done_and_real_warnings_remain():
    main = (GUI / 'mainwindow.cpp').read_text()
    block = main.split('if (preview_has_runtime_content) {', 1)[1].split('if (visual_quality_needs_review', 1)[0]
    assert 'preview_status = SceneWorkflowStepStatus::Done;' in block
    gate = block.split('const bool visual_quality_needs_review =', 1)[1]
    assert 'classified_overlay_count' not in gate
    for blocker in ('editable_layout_yaml_malformed', 'classified_fallback_count > 0',
                    'classified_warning_count > 0', 'classified_diagnostic_count > 0',
                    'has_warnings', '!scene3d_counters.visual_quality_warnings.isEmpty()'):
        assert blocker in gate
    assert '.arg(classified_overlay_count)' in main  # still reviewable in details
    assert 'transform_parity.failed' in block  # parity gate retained


def test_zone_edges_remain_intrinsically_excluded_from_picking():
    source = (ROOT / 'workcell_studio_web/viewer/viewer.js').read_text()
    function = 'function intrinsicallyExcludedPickNode(node)' + source.split(
        'function intrinsicallyExcludedPickNode(node)', 1)[1].split('function passThroughPickNode', 1)[0]
    script = function + """
const assert = require('assert');
for (const name of ['arbitrary_pick_fallback_edges', 'arbitrary_drop_fallback_edges']) {
  assert.strictEqual(intrinsicallyExcludedPickNode({name,visible:true,userData:{}}), true);
}
assert.strictEqual(intrinsicallyExcludedPickNode({name:'physical_mesh',visible:true,userData:{}}), false);
"""
    subprocess.run(['node', '-e', script], check=True, capture_output=True, text=True)
    assert 'edges.name = `${item.id || itemLabel(item)}_fallback_edges`;' in source
    assert 'group.add(edges);' in source


def test_native_helper_layer_precedes_editable_provenance_and_can_reveal_from_hierarchy():
    assembly = (GUI / 'scene3d_candidate_assembly.cpp').read_text()
    predicate = assembly.split('bool include_preview_item_for_scene3d(', 1)[1].split(
        'Scene3DLayerVisibilityDefaults compute_', 1)[0]
    assert predicate.index('if (item.semantic_task_zone_helper)') < predicate.index(
        'if (source_layer == "editable_layout")')
    assert 'enabled_layers.contains(is_warning_or_missing ? "warning" : "overlay")' in predicate
    assert 'out.overlay = false;' in assembly
    main = (GUI / 'mainwindow.cpp').read_text()
    hierarchy = main.split('void MainWindow::on_hierarchy_item_selected', 1)[1].split(
        'void MainWindow::rename_selected_item', 1)[0]
    assert 'all_scene_preview_items_' in hierarchy
    assert 'preview_item->semantic_task_zone_helper' in hierarchy
    assert 'preview_layer_overlays_helpers_box_->setChecked(true)' in hierarchy
    # The existing bridge sends visibility only, not authored removals.
    filters = main.split('void MainWindow::apply_scene3d_preview_layer_filters', 1)[1].split(
        'void MainWindow::', 1)[0]
    assert 'include_preview_item_for_scene3d(p, enabled_layers)' in filters
    assert 'set_live_visible_item_ids(visible_ids)' in filters


def test_hidden_diagnostics_still_count_toward_health_in_clean_view():
    main = (GUI / 'mainwindow.cpp').read_text()
    counts = main.split('int classified_overlay_count = 0;', 1)[1].split(
        'bool editable_layout_yaml_malformed', 1)[0]
    assert 'if (!preview_item_visible_for_active_layers(item)) continue;' not in counts
    assert counts.index('if (is_warning) ++classified_warning_count;') < counts.index(
        'if (preview_item_visible_for_active_layers(item))')
    gate = main.split('const bool visual_quality_needs_review =', 1)[1].split(';', 1)[0]
    assert 'scene3d_clean_product_view_' not in gate
    assert 'classified_warning_count > 0' in gate


def test_existing_zone_presentation_is_translucent_wireframe_and_bridge_only_changes_visibility():
    source = (ROOT / 'workcell_studio_web/viewer/viewer.js').read_text()
    material = source.split('function fallbackMaterialFor(item)', 1)[1].split(
        'function fallbackEdgeMaterialFor', 1)[0]
    assert 'opacity: isZoneFallback ? 0.08' in material
    assert 'wireframe: isZoneFallback' in material
    visibility = source.split('function setVisibleItemIdsFromBridge', 1)[1].split(
        'function ', 1)[0]
    assert 'object3d.visible = visible.has(id)' in visibility
    assert 'dirtyTransforms' not in visibility
    assert 'removeItem' not in visibility
