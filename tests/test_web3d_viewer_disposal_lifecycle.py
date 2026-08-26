import json
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
VIEWER = ROOT / "workcell_studio_web/viewer/viewer.js"
PREVIEW_CPP = ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp"
MAIN_LEGACY = ROOT / "workcell_builder/workcell_builder/gui/main_legacy.inc"


def test_viewer_disposal_is_idempotent_and_releases_render_resources(tmp_path):
    source = VIEWER.read_text(encoding="utf-8")
    start = source.index("function disposeViewerLifecycle(")
    end = source.index("window.__WORKCELL_VIEWER_LIFECYCLE__", start)
    function_source = source[start:end]
    harness = f"""
const calls = [];
const window = {{ removeEventListener: (name) => calls.push(`window:${{name}}`) }};
const el = {{ canvas: {{ removeEventListener: (name) => calls.push(`canvas:${{name}}`) }} }};
const state = {{ animationId: 42, three: {{
  renderer: {{ renderLists: {{ dispose: () => calls.push('renderLists') }}, dispose: () => calls.push('renderer') }},
  controls: {{ dispose: () => calls.push('controls') }},
  transformControls: {{ detach: () => calls.push('detach'), dispose: () => calls.push('transformControls') }}
}} }};
let viewerLifecycleDisposed = false;
const cancelAnimationFrame = id => calls.push(`cancel:${{id}}`);
const cancelInitialCameraFitRetry = () => calls.push('cancelFit');
const clearSceneObjects = () => calls.push('clearScene');
function resize() {{}} function onEditorKeyDown() {{}}
function onCanvasPointerDown() {{}} function onCanvasPointerMove() {{}}
function onCanvasPointerUp() {{}} function onCanvasPointerCancel() {{}}
function onCanvasContextMenu() {{}}
{function_source}
const first = disposeViewerLifecycle('test-switch');
const second = disposeViewerLifecycle('test-switch-again');
process.stdout.write(JSON.stringify({{ first, second, calls, threeKeys: Object.keys(state.three) }}));
"""
    result = subprocess.run(
        ["node", "-e", harness], cwd=ROOT, check=True, capture_output=True, text=True
    )
    payload = json.loads(result.stdout)
    assert payload["first"] == {
        "disposed": True, "already_disposed": False, "reason": "test-switch"
    }
    assert payload["second"]["already_disposed"] is True
    assert payload["calls"].count("cancel:42") == 1
    for expected in ["clearScene", "detach", "transformControls", "controls", "renderLists", "renderer"]:
        assert payload["calls"].count(expected) == 1
    assert payload["threeKeys"] == []


def test_qt_waits_for_idempotent_viewer_teardown_before_scene_navigation():
    source = PREVIEW_CPP.read_text(encoding="utf-8")
    start = source.index("void ScenePreviewWidget::load_prepared_embedded_web_scene")
    block = source[start:source.index("void ScenePreviewWidget::verify_embedded_editor_contract", start)]
    assert "__WORKCELL_VIEWER_LIFECYCLE__?.disposeScene" in block
    assert "qt_scene_navigation" in block
    assert block.index("disposeScene") < block.index("embedded_web_view_->load(viewer_url);")
    assert "embedded_web_identity_is_current(identity)" in block
    assert "queued_navigation_token != embedded_web_navigation_token_" in block


def test_real_gui_switch_smoke_waits_for_web3d_ready_and_matching_scene_identity():
    source = MAIN_LEGACY.read_text(encoding="utf-8")
    for token in [
        "--web3d-scene-sequence",
        "Web3D Product View — ready",
        "preview->preview_context().scene_id == scene",
        "workcell_studio_web3d_scene_switch_smoke/v1",
    ]:
        assert token in source
