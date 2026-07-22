import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VIEWER = ROOT / "workcell_studio_web/viewer"
PREFLIGHT = VIEWER / "product_view_runtime_preflight.js"
INDEX = VIEWER / "index.html"


def test_preflight_loads_before_the_pinned_viewer_bundle():
    html = INDEX.read_text(encoding="utf-8")
    preflight = '<script src="./product_view_runtime_preflight.js"></script>'
    bundle = '<script type="module" src="./dist/viewer.bundle.js"></script>'
    assert preflight in html
    assert bundle in html
    assert html.index(preflight) < html.index(bundle)


def test_preflight_does_not_patch_network_and_forces_light_clear_color():
    source = PREFLIGHT.read_text(encoding="utf-8")
    node_script = f"""
const vm = require('vm');
const source = {json.dumps(source)};
class FakeXhr {{
  open(method, url) {{ this.method = method; this.url = String(url); }}
}}
class FakeGl {{
  constructor(id = 'scene-canvas') {{ this.canvas = {{ id }}; this.last = null; }}
  clearColor(r, g, b, a) {{ this.last = [r, g, b, a]; }}
}}
const originalFetch = async input => {{ return {{ ok: true, input }}; }};
const originalOpen = FakeXhr.prototype.open;
const location = new URL('http://127.0.0.1:8765/index.html?embedded=1');
const styleValues = {{}};
const window = {{
  location,
  fetch: originalFetch,
  XMLHttpRequest: FakeXhr,
  WebGLRenderingContext: FakeGl,
  WebGL2RenderingContext: FakeGl,
}};
const context = {{
  window,
  document: {{ documentElement: {{ style: {{ setProperty: (key, value) => styleValues[key] = value }} }} }},
  URL,
  URLSearchParams,
  Symbol,
  console,
}};
vm.runInNewContext(source, context, {{ filename: 'product_view_runtime_preflight.js' }});
const api = window.__WORKCELL_PRODUCT_VIEW_RUNTIME_PREFLIGHT_V1__;
if (!api?.enabled || api.scope !== 'scene-canvas-light-background') throw new Error('preflight not ready');
if (window.fetch !== originalFetch) throw new Error('fetch was patched');
if (FakeXhr.prototype.open !== originalOpen) throw new Error('XMLHttpRequest.open was patched');
if ('rewritePackageRootUrl' in api) throw new Error('network mesh rewrite API is still exposed');
const gl = new FakeGl();
gl.clearColor(0.01, 0.02, 0.03, 1);
const expectedClear = [0xee / 0xff, 0xf1 / 0xff, 0xf4 / 0xff, 1];
if (gl.last.some((value, index) => Math.abs(value - expectedClear[index]) > 1e-12)) {{
  throw new Error('scene canvas background remained dark');
}}
const other = new FakeGl('other-canvas');
other.clearColor(0.1, 0.2, 0.3, 0.4);
if (JSON.stringify(other.last) !== JSON.stringify([0.1, 0.2, 0.3, 0.4])) {{
  throw new Error('non-viewer canvas was modified');
}}
if (styleValues['--workcell-product-view-background'] !== '#eef1f4') {{
  throw new Error('CSS background fallback missing');
}}
"""
    result = subprocess.run(
        ["node", "-e", node_script],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_preflight_stays_scoped_to_visual_asset_fetch_and_rendering_only():
    source = PREFLIGHT.read_text(encoding="utf-8").lower()
    for forbidden in (
        "ros2 launch",
        "execute_trajectory",
        "move_group",
        "real_hardware_enabled",
        "yaml",
        "writefile",
        "localstorage",
    ):
        assert forbidden not in source
