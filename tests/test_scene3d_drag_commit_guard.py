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
    assert 'transform_changed_cb(it.id, it.x, it.y, it.z, it.roll, it.pitch, it.yaw);' in VIEW_CPP


def test_drag_cancel_restores_previous_pose_without_commit_callback():
    for token in [
        'it.x = drag_start_pose_.x;',
        'it.y = drag_start_pose_.y;',
        'it.z = drag_start_pose_.z;',
        'it.roll = drag_start_pose_.roll;',
        'it.pitch = drag_start_pose_.pitch;',
        'it.yaw = drag_start_pose_.yaw;',
    ]:
        assert token in VIEW_CPP
    esc_block = VIEW_CPP.split('if (e->key() == Qt::Key_Escape) {', 1)[1].split('QOpenGLWidget::keyPressEvent(e);', 1)[0]
    assert 'transform_changed_cb(it.id, it.x, it.y, it.z, it.roll, it.pitch, it.yaw);' not in esc_block
    assert 'drag_cancelled_ = true;' in esc_block
    assert 'update();' in esc_block
    assert 'QStringLiteral("Gizmo drag cancelled.")' in esc_block


def test_transform_editing_guard_and_generated_locked_rejection_tokens():
    assert 'Locked/generated item edit rejected' in MAIN_CPP
