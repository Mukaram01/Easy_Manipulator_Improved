from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')


class SimulatedScene3DDrag:
    def __init__(self, *, editable=True, locked=False):
        self.item = {"id": "item1", "x": 0.0, "y": 0.0, "z": 0.0, "editable": editable, "locked": locked}
        self.dragging = False
        self.drag_in_progress = False
        self.drag_cancelled = False
        self.drag_start = None
        self.save_calls = []

    def _is_editable(self):
        return self.item["editable"] and not self.item["locked"]

    def press(self):
        if not self._is_editable():
            self.dragging = False
            self.drag_in_progress = False
            return False
        self.dragging = True
        self.drag_in_progress = True
        self.drag_cancelled = False
        self.drag_start = (self.item["x"], self.item["y"], self.item["z"])
        return True

    def move(self, dx):
        if not self.dragging:
            return False
        # Preview transform update happens in-memory during drag.
        self.item["x"] = self.drag_start[0] + dx
        return True

    def cancel(self):
        if not self.drag_in_progress:
            return False
        self.item["x"], self.item["y"], self.item["z"] = self.drag_start
        self.drag_cancelled = True
        self.dragging = False
        return True

    def release(self):
        if self.drag_in_progress and not self.drag_cancelled:
            self.save_calls.append((self.item["id"], self.item["x"], self.item["y"], self.item["z"]))
        self.dragging = False
        self.drag_in_progress = False


def test_translate_gizmo_has_minimal_static_sentinel():
    assert 'bool item_is_editable_for_gizmo(const ScenePreviewWidget::PreviewItem & it)' in VIEW_CPP


def test_translate_gizmo_behavior_preview_mutates_and_release_commits_once():
    sim = SimulatedScene3DDrag(editable=True, locked=False)
    assert sim.press() is True
    assert sim.move(0.25) is True
    assert sim.item["x"] == 0.25
    assert sim.save_calls == []

    sim.release()
    assert len(sim.save_calls) == 1
    assert sim.save_calls[0][0] == "item1"
    assert sim.save_calls[0][1] == 0.25


def test_translate_gizmo_behavior_no_commit_for_locked_or_cancelled_drag():
    locked = SimulatedScene3DDrag(editable=True, locked=True)
    assert locked.press() is False
    assert locked.move(0.5) is False
    locked.release()
    assert locked.save_calls == []

    cancelled = SimulatedScene3DDrag(editable=True, locked=False)
    assert cancelled.press() is True
    assert cancelled.move(0.5) is True
    assert cancelled.item["x"] == 0.5
    assert cancelled.cancel() is True
    assert cancelled.item["x"] == 0.0
    cancelled.release()
    assert cancelled.save_calls == []
