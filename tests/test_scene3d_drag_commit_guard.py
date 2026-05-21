from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')


class DragCommitFlow:
    def __init__(self):
        self.pose = {"x": 1.0, "y": 2.0, "z": 3.0}
        self.start = dict(self.pose)
        self.dragging = False
        self.drag_cancelled = False
        self.callbacks = []

    def press(self):
        self.dragging = True
        self.drag_cancelled = False
        self.start = dict(self.pose)

    def move(self, dx):
        if self.dragging:
            self.pose["x"] = self.start["x"] + dx

    def cancel(self):
        if not self.dragging:
            return
        self.pose = dict(self.start)
        self.drag_cancelled = True
        self.dragging = False

    def release(self):
        if not self.dragging:
            return False
        self.dragging = False
        if self.drag_cancelled:
            return False
        self.callbacks.append((self.pose["x"], self.pose["y"], self.pose["z"]))
        return True


def test_drag_commit_static_sentinel_mouse_release_event_exists():
    assert 'void Scene3DViewportWidget::mouseReleaseEvent(QMouseEvent * e)' in VIEW_CPP


def test_drag_move_mutates_preview_but_release_is_the_only_save_boundary():
    flow = DragCommitFlow()
    flow.press()

    flow.move(dx=0.75)
    assert flow.pose["x"] == 1.75
    assert flow.callbacks == []

    assert flow.release() is True
    assert len(flow.callbacks) == 1


def test_drag_cancel_reverts_preview_and_never_writes_callback():
    flow = DragCommitFlow()
    flow.press()
    flow.move(dx=-0.8)
    assert flow.pose["x"] == 0.19999999999999996

    flow.cancel()
    assert flow.pose == flow.start
    assert flow.release() is False
    assert flow.callbacks == []
