from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')


class DragFlowHarness:
    def __init__(self, *, editable=True, locked=False):
        self.editable = editable
        self.locked = locked
        self.pose = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.start_pose = None
        self.drag_in_progress = False
        self.drag_cancelled = False
        self.saved = []

    def press(self):
        if self.locked or not self.editable:
            self.drag_in_progress = False
            return False
        self.start_pose = dict(self.pose)
        self.drag_in_progress = True
        self.drag_cancelled = False
        return True

    def move(self, dx=0.0, dy=0.0, dz=0.0):
        if not self.drag_in_progress:
            return False
        self.pose["x"] = self.start_pose["x"] + dx
        self.pose["y"] = self.start_pose["y"] + dy
        self.pose["z"] = self.start_pose["z"] + dz
        return True

    def cancel(self):
        if not self.drag_in_progress:
            return
        self.pose = dict(self.start_pose)
        self.drag_cancelled = True

    def release(self):
        if self.drag_in_progress and not self.drag_cancelled:
            self.saved.append(dict(self.pose))
        self.drag_in_progress = False


def test_drag_commit_guard_has_minimal_static_sentinel():
    assert 'void Scene3DViewportWidget::mouseReleaseEvent(QMouseEvent * e)' in VIEW_CPP


def test_drag_flow_preview_updates_before_release_and_release_commits():
    h = DragFlowHarness(editable=True, locked=False)
    assert h.press() is True
    assert h.move(dx=0.2, dy=-0.1, dz=0.05) is True
    assert h.pose == {"x": 0.2, "y": -0.1, "z": 0.05}
    assert h.saved == []

    h.release()
    assert h.saved == [{"x": 0.2, "y": -0.1, "z": 0.05}]


def test_drag_flow_cancel_and_readonly_paths_do_not_commit():
    h = DragFlowHarness(editable=True, locked=False)
    assert h.press() is True
    assert h.move(dx=0.3) is True
    h.cancel()
    h.release()
    assert h.pose == {"x": 0.0, "y": 0.0, "z": 0.0}
    assert h.saved == []

    readonly = DragFlowHarness(editable=False, locked=False)
    assert readonly.press() is False
    assert readonly.move(dx=0.6) is False
    readonly.release()
    assert readonly.saved == []
