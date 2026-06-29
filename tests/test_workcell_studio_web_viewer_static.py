from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VIEWER = ROOT / "workcell_studio_web" / "viewer"


def test_static_viewer_files_exist():
    assert (VIEWER / "index.html").is_file()
    assert (VIEWER / "viewer.js").is_file()
    assert (VIEWER / "style.css").is_file()
    assert (VIEWER / "README.md").is_file()


def test_index_references_static_assets():
    index = (VIEWER / "index.html").read_text(encoding="utf-8")
    assert 'href="style.css"' in index
    assert 'src="viewer.js"' in index
    assert 'id="scene-file"' in index
    assert 'web_scene.json' in index


def test_viewer_js_schema_and_inspector_hooks():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    assert "SUPPORTED_SCHEMA_VERSION" in js
    assert "workcell_studio_web_scene/v1" in js
    assert "schema_version" in js
    assert "populateInspector" in js
    assert "mesh_uri" in js
    assert "primitive" in js
    assert "editable" in js
    assert "locked" in js


def test_viewer_records_render_status_for_inspector():
    js = (VIEWER / "viewer.js").read_text(encoding="utf-8")
    assert "setRenderInfo" in js
    for token in [
        "render_status",
        "fallback_reason",
        "mesh_loaded",
        "primitive_fallback",
        "box_fallback",
        "mesh_failed",
    ]:
        assert token in js
    assert "child.userData.renderInfo" in js
    assert "rendered.renderInfo" in js
