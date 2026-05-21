from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')
MAIN_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_drag_preview_updates_item_pose_without_commit_write_on_mousemove():
    assert 'if (dragging_gizmo_ && (e->buttons() & Qt::LeftButton) && !selected_id.isEmpty()) {' in VIEW_CPP
    assert 'it.x = drag_start_pose_.x + snapped' in VIEW_CPP
    assert 'it.y = drag_start_pose_.y + snapped' in VIEW_CPP
    assert 'update();' in VIEW_CPP


def test_drag_commit_happens_on_mouse_release_via_transform_callback():
    assert 'void Scene3DViewportWidget::mouseReleaseEvent(QMouseEvent * e)' in VIEW_CPP
    assert 'if (drag_in_progress_ && !drag_cancelled_ && transform_changed_cb)' in VIEW_CPP
    assert "const bool finite_xyz = std::isfinite(it.x) && std::isfinite(it.y) && std::isfinite(it.z);" in VIEW_CPP
    assert 'const bool bounded_xyz = qAbs(it.x) <= kWorkspaceLimitMeters' in VIEW_CPP
    assert "const bool id_valid = !it.id.trimmed().isEmpty() && !drag_start_pose_.item_id.trimmed().isEmpty() && it.id == selected_id;" in VIEW_CPP
    assert 'transform_changed_cb(it.id, it.x, it.y, it.z, it.roll, it.pitch, it.yaw);' in VIEW_CPP


def test_drag_commit_rejects_invalid_xyz_and_reverts_without_write():
    assert "if (!id_valid || !finite_xyz || !bounded_xyz) {" in VIEW_CPP
    for token in ['it.x = drag_start_pose_.x;', 'it.y = drag_start_pose_.y;', 'it.z = drag_start_pose_.z;']:
        assert token in VIEW_CPP
    assert "Rejected gizmo drag commit for '%1': invalid final XYZ or mismatched drag source; pose reverted." in VIEW_CPP
    assert 'if (status_message_cb) status_message_cb(message);' in VIEW_CPP
    assert 'update();' in VIEW_CPP


def test_drag_commit_mismatch_source_emits_discard_message_without_callback_write():
    assert 'No valid drag source found for commit; change discarded.' in VIEW_CPP
    assert 'if (!committed && status_message_cb) {' in VIEW_CPP


def test_drag_cancel_restores_previous_pose_and_keeps_selection_path_alive():
    for token in ['it.x = drag_start_pose_.x;', 'it.y = drag_start_pose_.y;', 'it.z = drag_start_pose_.z;']:
        assert token in VIEW_CPP
    esc_block = VIEW_CPP.split('if (e->key() == Qt::Key_Escape) {', 1)[1].split('QOpenGLWidget::keyPressEvent(e);', 1)[0]
    assert 'transform_changed_cb(it.id, it.x, it.y, it.z, it.roll, it.pitch, it.yaw);' not in esc_block
    assert 'drag_cancelled_ = true;' in esc_block
    assert 'update();' in esc_block
    assert 'QStringLiteral("Gizmo drag cancelled.")' in esc_block


def test_transform_editing_guard_and_generated_locked_rejection_tokens():
    assert 'Locked/generated item edit rejected' in MAIN_CPP
