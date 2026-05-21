from pathlib import Path
import subprocess
import json

ROOT = Path(__file__).resolve().parents[1]
VIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')


class EventFlowContractSim:
    def __init__(self, *, editable=True, locked=False):
        self.item = {"x": 1.0, "y": 2.0, "z": 3.0}
        self.start = None
        self.editable = editable
        self.locked = locked
        self.drag_active = False
        self.cancelled = False
        self.write_count = 0

    def press(self):
        if self.locked or not self.editable:
            return False
        self.start = dict(self.item)
        self.drag_active = True
        self.cancelled = False
        return True

    def move(self, dx):
        if not self.drag_active:
            return False
        self.item["x"] = self.start["x"] + dx
        return True

    def cancel(self):
        if self.drag_active:
            self.item = dict(self.start)
            self.cancelled = True

    def release(self):
        if self.drag_active and not self.cancelled:
            self.write_count += 1
        self.drag_active = False


def test_feature_regression_has_minimal_static_sentinel():
    assert 'if (drag_in_progress_ && !drag_cancelled_ && transform_changed_cb)' in VIEW_CPP


def test_feature_regression_behavior_primary_guardrails():
    ok = EventFlowContractSim(editable=True, locked=False)
    assert ok.press() is True
    assert ok.move(0.75) is True
    assert ok.item["x"] == 1.75
    assert ok.write_count == 0
    ok.release()
    assert ok.write_count == 1

    cancelled = EventFlowContractSim(editable=True, locked=False)
    assert cancelled.press() is True
    assert cancelled.move(-0.4) is True
    cancelled.cancel()
    cancelled.release()
    assert cancelled.item["x"] == 1.0
    assert cancelled.write_count == 0

    readonly = EventFlowContractSim(editable=False, locked=False)
    assert readonly.press() is False
    assert readonly.move(0.2) is False
    readonly.release()
    assert readonly.write_count == 0


def test_checker_has_pass_warn_fail_fields(tmp_path):
    out_json = tmp_path / 'contract.json'
    subprocess.run(['python3', str(ROOT / 'scripts' / 'check_scene3d_canvas_contract.py'), '--scene', 'suction_test', '--json', str(out_json)], check=False)
    payload = json.loads(out_json.read_text(encoding='utf-8'))
    scene = payload['scenes'][0]
    required = ['scene','editable_layout_count','mesh_preview_count','locked_generated_urdf_visual_count','primitive_fallback_count','overlay_count','unsafe_visual_reason_count','unresolved_placeholder_count','contract_status','blockers','suggested_fixes']
    for k in required:
        assert k in scene
    assert scene['contract_status'] in {'PASS','WARN','FAIL'}
