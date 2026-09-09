from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VIEWER = ROOT / "workcell_studio_web" / "viewer"


def test_product_view_installs_lifecycle_bridge_before_versioned_bundle():
    index = (VIEWER / "index.html").read_text(encoding="utf-8")
    lifecycle = (VIEWER / "viewer_lifecycle_bridge.js").read_text(encoding="utf-8")

    assert '<script src="./viewer_lifecycle_bridge.js"></script>' in index
    assert index.index("viewer_lifecycle_bridge.js") < index.index("viewer.bundle.js")
    assert "__WORKCELL_VIEWER_LIFECYCLE__" in lifecycle
    assert "disposeScene" in lifecycle
    assert "already_disposed: true" in lifecycle
    assert "already_disposed: false" in lifecycle
    assert "workcell:viewer-dispose" in lifecycle


def test_bridge_and_production_bundle_cleanup_and_repeated_navigation():
    """Load the actual HTML/bootstrap/bundle together in a real WebGL browser."""
    import functools
    import hashlib
    import http.server
    import json
    import threading
    from urllib.parse import urlparse
    from playwright.sync_api import sync_playwright

    class QuietHandler(http.server.SimpleHTTPRequestHandler):
        def log_message(self, *args):
            pass

    server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), functools.partial(QuietHandler, directory=str(ROOT)))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    build = hashlib.sha256((VIEWER / 'dist/viewer.bundle.js').read_bytes()).hexdigest()
    url = f'http://127.0.0.1:{server.server_port}/workcell_studio_web/viewer/index.html?viewerBuild={build}&scene=build/workcell_studio_web_scene/fixture.web_scene.json'
    fixture = {'schema_version':'workcell_studio_web_scene/v1', 'scene_id':'ur5_2f_test', 'assets':[
        {'id':'bin', 'type':'target_bin', 'geometry_type':'box', 'dimensions':[.2,.2,.2],
         'editable':True, 'locked':False, 'source_layer':'editable_layout',
         'pose':{'xyz':[.45,.22,.1],'rpy':[0,0,0]}}]}
    try:
        with sync_playwright() as p:
            browser = p.chromium.launch(headless=True, args=["--enable-unsafe-swiftshader"])
            page = browser.new_page()
            errors = []
            page.on('pageerror', lambda error: errors.append(str(error)))
            page.route(lambda url: urlparse(url).path.endswith('/fixture.web_scene.json'), lambda route: route.fulfill(json=fixture))
            page.add_init_script('''
              window.cleanupEvents = 0;
              window.cancelledFrames = 0;
              window.addEventListener('workcell:viewer-dispose', () => window.cleanupEvents++);
              const cancel = window.cancelAnimationFrame;
              window.cancelAnimationFrame = id => { window.cancelledFrames++; cancel(id); };
            ''')
            for reason in ('qt_scene_navigation', 'pagehide', 'beforeunload'):
                page.goto(url)
                try:
                    page.wait_for_function("window.__WORKCELL_EDITOR_API_V1__?.getState().ready", timeout=10000)
                except Exception:
                    raise AssertionError(page.evaluate("({status:window.__WORKCELL_VIEWER_STATUS__,text:document.body.innerText})"))
                # Qt serializes object keys in a different order from JavaScript.
                # Consecutive saves must compare numeric values, not JSON text.
                for x in (.46, .47):
                    page.evaluate("x => window.__WORKCELL_EDITOR_API_V1__.setItemPose('bin',x,.22,.1,0,0,0)", x)
                    patch = page.evaluate('window.__WORKCELL_EDITOR_API_V1__.getEditPatch()')
                    assert len(patch['edits']) == 1
                    patch = json.loads(json.dumps(patch, sort_keys=True))
                    result = page.evaluate('patch => window.__WORKCELL_EDITOR_API_V1__.rebasePersistedPatch(patch)', patch)
                    assert result['ok'] and result['dirtyCount'] == 0
                if reason == 'qt_scene_navigation':
                    result = page.evaluate("window.__WORKCELL_VIEWER_LIFECYCLE__.disposeScene('qt_scene_navigation')")
                    assert result['disposed'] and not result['already_disposed']
                else:
                    page.evaluate('reason => window.dispatchEvent(new Event(reason))', reason)
                second = page.evaluate("window.__WORKCELL_VIEWER_LIFECYCLE__.disposeScene('repeat')")
                assert second['already_disposed']
                assert page.evaluate('window.cleanupEvents') == 1
                assert page.evaluate('window.cancelledFrames') >= 1
            assert not errors
            browser.close()
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
