from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')


class Scene3DDragSimulator:
    def __init__(self, *, editable=True):
        self.editable = editable
        self.selected_id = "item_1"
        self.pose = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.start_pose = dict(self.pose)
        self.dragging = False
        self.drag_cancelled = False
        self.save_calls = []

    def press(self):
        if not self.editable:
            return False
        self.dragging = True
        self.drag_cancelled = False
        self.start_pose = dict(self.pose)
        return True

    def move(self, dx, dy=0.0, dz=0.0):
        if not self.dragging:
            return
        self.pose["x"] = self.start_pose["x"] + dx
        self.pose["y"] = self.start_pose["y"] + dy
        self.pose["z"] = self.start_pose["z"] + dz

    def cancel(self):
        if not self.dragging:
            return
        self.pose = dict(self.start_pose)
        self.drag_cancelled = True
        self.dragging = False

    def release(self):
        if not self.dragging:
            return False
        self.dragging = False
        if self.drag_cancelled:
            return False
        self.save_calls.append((self.selected_id, self.pose["x"], self.pose["y"], self.pose["z"]))
        return True


def test_translate_gizmo_static_sentinel_function_exists():
    assert 'bool item_is_editable_for_gizmo(const ScenePreviewWidget::PreviewItem & it)' in VIEW_CPP


def test_translate_gizmo_editable_item_press_move_release_behaves_like_single_commit_flow():
    sim = Scene3DDragSimulator(editable=True)

    assert sim.press() is True
    sim.move(dx=0.25)
    assert sim.pose["x"] == 0.25  # preview mutates during drag/move
    assert sim.save_calls == []

    assert sim.release() is True
    assert len(sim.save_calls) == 1


def test_translate_gizmo_read_only_item_never_enters_drag_or_save_flow():
    sim = Scene3DDragSimulator(editable=False)

    assert sim.press() is False
    sim.move(dx=0.5)
    assert sim.pose == {"x": 0.0, "y": 0.0, "z": 0.0}
    assert sim.release() is False
    assert sim.save_calls == []
