from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')


def test_translate_gizmo_editability_gate_tokens_present():
    assert 'bool item_is_editable_for_gizmo(const ScenePreviewWidget::PreviewItem & it)' in VIEW_CPP
    assert 'if (it.locked) return false;' in VIEW_CPP
    assert 'return it.editable;' in VIEW_CPP


def test_translate_gizmo_available_for_editable_and_not_for_locked_generated_visuals():
    assert 'if (!item_is_editable_for_gizmo(it)) {' in VIEW_CPP
    assert 'status_message_cb(QStringLiteral("Locked: %1").arg(item_locked_reason(it)));' in VIEW_CPP


def test_gizmo_pick_move_axis_and_drag_handle_tokens_present():
    assert 'pick_gizmo_axis_at_screen' in VIEW_CPP
    assert 'drag_active_handle_ = (axis == "x") ? GizmoHandle::MoveX' in VIEW_CPP
    assert 'if (gizmo_mode == GizmoMode::Move)' in VIEW_CPP


def test_overlay_items_never_show_translate_affordance_rule_documented():
    assert 'bool is_overlay_only_item(const ScenePreviewWidget::PreviewItem & it)' in VIEW_CPP
    assert 'role_text.contains("overlay")' in VIEW_CPP
