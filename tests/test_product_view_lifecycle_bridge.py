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
