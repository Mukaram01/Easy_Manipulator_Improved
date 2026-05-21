from pathlib import Path
import json
import subprocess

ROOT = Path(__file__).resolve().parents[1]


class SaveGuardSimulator:
    def __init__(self, *, editable=True):
        self.editable = editable
        self.pose = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.start = dict(self.pose)
        self.dragging = False
        self.cancelled = False
        self.saved = 0

    def press(self):
        if not self.editable:
            return False
        self.dragging = True
        self.cancelled = False
        self.start = dict(self.pose)
        return True

    def move(self, delta):
        if self.dragging:
            self.pose["x"] = self.start["x"] + delta

    def release(self):
        if not self.dragging:
            return False
        self.dragging = False
        if self.cancelled:
            return False
        self.saved += 1
        return True

    def cancel(self):
        if self.dragging:
            self.pose = dict(self.start)
            self.dragging = False
            self.cancelled = True


def test_feature_regression_static_sentinel_contract_checker_has_status_field(tmp_path):
    out_json = tmp_path / 'contract.json'
    subprocess.run(['python3', str(ROOT / 'scripts' / 'check_scene3d_canvas_contract.py'), '--scene', 'suction_test', '--json', str(out_json)], check=False)
    payload = json.loads(out_json.read_text(encoding='utf-8'))
    assert 'contract_status' in payload['scenes'][0]


def test_feature_regression_behavior_successful_drag_writes_once_cancel_and_read_only_write_never():
    ok = SaveGuardSimulator(editable=True)
    assert ok.press() is True
    ok.move(0.3)
    assert ok.pose["x"] == 0.3
    assert ok.saved == 0
    assert ok.release() is True
    assert ok.saved == 1

    cancelled = SaveGuardSimulator(editable=True)
    cancelled.press()
    cancelled.move(0.9)
    cancelled.cancel()
    assert cancelled.pose["x"] == 0.0
    assert cancelled.release() is False
    assert cancelled.saved == 0

    ro = SaveGuardSimulator(editable=False)
    assert ro.press() is False
    ro.move(1.0)
    assert ro.pose["x"] == 0.0
    assert ro.release() is False
    assert ro.saved == 0
