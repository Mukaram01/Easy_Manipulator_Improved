from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')


def test_translate_gizmo_editability_gate_is_contract_aware():
    assert 'bool item_is_editable_for_gizmo(const ScenePreviewWidget::PreviewItem & it)' in VIEW_CPP
    assert 'if (it.locked) return false;' in VIEW_CPP
    assert 'if (source_layer == "editable_layout") return it.editable;' in VIEW_CPP
    assert 'source_layer == "primitive_fallback"' in VIEW_CPP
    assert 'visual_source == "mesh_preview"' in VIEW_CPP
    assert 'it.linked_to_editable_layout_state' in VIEW_CPP


def test_translate_gizmo_available_for_editable_and_not_for_locked_generated_visuals():
    assert 'if (!item_is_editable_for_gizmo(it)) {' in VIEW_CPP
    assert 'status_message_cb(QStringLiteral("Locked: %1").arg(item_locked_reason(it)));' in VIEW_CPP


def test_gizmo_pick_move_axis_and_drag_handle_tokens_present():
    assert 'pick_gizmo_axis_at_screen' in VIEW_CPP
    assert 'drag_active_handle_ = (axis == "x") ? GizmoHandle::MoveX' in VIEW_CPP
    assert 'if (gizmo_mode == GizmoMode::Move)' in VIEW_CPP


def test_overlay_helper_and_generated_items_never_show_translate_affordance():
    assert 'overlay_or_helper' in VIEW_CPP
    assert 'generated_robot_visual' in VIEW_CPP
    assert 'return false;' in VIEW_CPP
