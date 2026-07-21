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


def test_preflight_rewrites_bare_ros_package_meshes_and_forces_light_clear_color():
    source = PREFLIGHT.read_text(encoding="utf-8")
    node_script = f"""
const vm = require('vm');
const source = {json.dumps(source)};
class FakeRequest {{
  constructor(url, init) {{ this.url = String(url); this.init = init; }}
}}
class FakeXhr {{
  open(method, url) {{ this.method = method; this.url = String(url); }}
}}
class FakeGl {{
  constructor(id = 'scene-canvas') {{ this.canvas = {{ id }}; this.last = null; }}
  clearColor(r, g, b, a) {{ this.last = [r, g, b, a]; }}
}}
const location = new URL(
  'http://127.0.0.1:8765/index.html?embedded=1&scene=' +
  encodeURIComponent('build/workcell_studio_web_scene/ur5_2f_test.web_scene.json')
);
const styleValues = {{}};
const calls = [];
const window = {{
  location,
  fetch: async input => {{ calls.push(input instanceof FakeRequest ? input.url : String(input)); return {{ ok: true }}; }},
  XMLHttpRequest: FakeXhr,
  WebGLRenderingContext: FakeGl,
  WebGL2RenderingContext: FakeGl,
}};
const context = {{
  window,
  document: {{ documentElement: {{ style: {{ setProperty: (key, value) => styleValues[key] = value }} }} }},
  URL,
  URLSearchParams,
  Request: FakeRequest,
  Symbol,
  console,
}};
vm.runInNewContext(source, context, {{ filename: 'product_view_runtime_preflight.js' }});
const api = window.__WORKCELL_PRODUCT_VIEW_RUNTIME_PREFLIGHT_V1__;
if (!api?.enabled || api.getSceneId() !== 'ur5_2f_test') throw new Error('preflight not ready');
const bare = 'http://127.0.0.1:8765/robotiq_85_description/meshes/visual/robotiq_85_base_link.dae';
const expected = 'http://127.0.0.1:8765/build/workcell_studio_web_scene/assets/ur5_2f_test/robotiq_85_description/meshes/visual/robotiq_85_base_link.dae';
if (api.rewritePackageRootUrl(bare) !== expected) throw new Error('package URL not staged');
const ordinary = 'http://127.0.0.1:8765/workcell_studio_web/viewer/style.css';
if (api.rewritePackageRootUrl(ordinary) !== ordinary) throw new Error('ordinary URL changed');
(async () => {{
  await window.fetch(bare);
  if (calls[0] !== expected) throw new Error('fetch did not use staged URL');
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
}})().catch(error => {{ console.error(error); process.exit(1); }});
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
