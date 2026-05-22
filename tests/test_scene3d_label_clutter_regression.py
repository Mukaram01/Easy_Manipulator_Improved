from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PREVIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene_preview_widget.cpp').read_text(encoding='utf-8')
VIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')


def test_default_label_mode_is_selected():
    assert 'labels_selector_->setCurrentText("Selected");' in PREVIEW_CPP
    assert 'case ScenePreviewWidget::LabelMode::Selected: draw_label = selected; break;' in VIEW_CPP


def test_generated_robot_link_labels_suppressed_by_default_and_selected_still_draws():
    assert 'const bool is_urdf_visual = it.locked && !it.editable && it.lock_reason.contains("URDF visual", Qt::CaseInsensitive);' in VIEW_CPP
    assert 'if (is_urdf_visual && !selected && label_mode != ScenePreviewWidget::LabelMode::All) draw_label = false;' in VIEW_CPP
    assert 'case ScenePreviewWidget::LabelMode::Selected: draw_label = selected; break;' in VIEW_CPP


def test_overlap_suppression_path_and_counters_present():
    assert 'apply_label_overlap_offset' in VIEW_CPP
    assert 'LABEL_OVERLAP_SUPPRESS_LOWER_PRIORITY' in VIEW_CPP
    assert 'if (overlaps) continue;' in VIEW_CPP
    assert 'QString("Overlays %1 • Locked URDF %2 • Mode: %3")' in VIEW_CPP
