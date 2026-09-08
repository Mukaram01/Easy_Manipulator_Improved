import json
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
VIEWER = ROOT / "workcell_studio_web/viewer/viewer.js"
LIFECYCLE = ROOT / "workcell_studio_web/viewer/viewer_lifecycle_bridge.js"
INDEX = ROOT / "workcell_studio_web/viewer/index.html"
PREVIEW_CPP = ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp"
MAIN_LEGACY = ROOT / "workcell_builder/workcell_builder/gui/main_legacy.inc"


def test_viewer_lifecycle_bridge_is_idempotent_and_available_before_bundle(tmp_path):
    source = LIFECYCLE.read_text(encoding="utf-8")
    index = INDEX.read_text(encoding="utf-8")
    harness = f"""
const calls = [];
const CustomEvent = function(name, init) {{ this.type = name; this.detail = init?.detail; }};
const window = {{
  dispatchEvent: event => calls.push({{ type: event.type, detail: event.detail }}),
}};
{source}
const api = window.__WORKCELL_VIEWER_LIFECYCLE__;
const first = api.disposeScene('test-switch');
const second = api.disposeScene('test-switch-again');
process.stdout.write(JSON.stringify({{ first, second, calls, apiVersion: api.apiVersion }}));
"""
    result = subprocess.run(
        ["node", "-e", harness], cwd=ROOT, check=True, capture_output=True, text=True
    )
    payload = json.loads(result.stdout)
    assert payload["first"] == {
        "disposed": True,
        "already_disposed": False,
        "reason": "test-switch",
    }
    assert payload["second"] == {
        "disposed": True,
        "already_disposed": True,
        "reason": "test-switch-again",
    }
    assert payload["apiVersion"] == "1.0.0"
    assert [event["type"] for event in payload["calls"]] == ["workcell:viewer-dispose"]
    assert index.index("viewer_lifecycle_bridge.js") < index.index("viewer.bundle.js")


def test_qt_waits_for_idempotent_viewer_teardown_before_scene_navigation():
    source = PREVIEW_CPP.read_text(encoding="utf-8")
    start = source.index("void ScenePreviewWidget::load_prepared_embedded_web_scene")
    block = source[start:source.index("void ScenePreviewWidget::verify_embedded_editor_contract", start)]
    assert "__WORKCELL_VIEWER_LIFECYCLE__?.disposeScene" in block
    assert "qt_scene_navigation" in block
    assert block.index("disposeScene") < block.index("embedded_web_view_->load(viewer_url);")
    assert "embedded_web_identity_is_current(identity)" in block
    assert "queued_navigation_token != embedded_web_navigation_token_" in block


def test_viewer_module_remains_free_to_own_scene_render_state():
    source = VIEWER.read_text(encoding="utf-8")
    assert "function clearSceneObjects(" in source
    assert "state.animationId" in source
    assert "state.three" in source


def test_real_gui_switch_smoke_waits_for_web3d_ready_and_matching_scene_identity():
    source = MAIN_LEGACY.read_text(encoding="utf-8")
    for token in [
        "--web3d-scene-sequence",
        "Web3D Product View — ready",
        "preview->preview_context().scene_id == scene",
        "workcell_studio_web3d_scene_switch_smoke/v1",
    ]:
        assert token in source
