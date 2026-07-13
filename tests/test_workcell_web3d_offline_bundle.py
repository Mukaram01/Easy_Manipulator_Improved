import json
import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
VIEWER = ROOT / "workcell_studio_web" / "viewer"
RUNTIME_FILES = [
    VIEWER / "index.html",
    VIEWER / "style.css",
    VIEWER / "dist" / "viewer.bundle.js",
    VIEWER / "workcell_runtime_marker.json",
]
CDN_RE = re.compile(r"https?://[^\s\"']*(?:unpkg|jsdelivr|cdnjs|cdn)[^\s\"']*", re.I)
BARE_THREE_URDF_IMPORT_RE = re.compile(
    r"\bimport\s*(?:\([^)]*['\"](?:three(?:/[^'\"]*)?|urdf-loader)['\"]\)|[^;]*?from\s*['\"](?:three(?:/[^'\"]*)?|urdf-loader)['\"]|['\"](?:three(?:/[^'\"]*)?|urdf-loader)['\"])",
    re.M,
)


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_runtime_viewer_uses_committed_bundle_not_importmap():
    html = _text(VIEWER / "index.html")
    assert 'type="importmap"' not in html
    assert "type='importmap'" not in html
    assert (VIEWER / "dist" / "viewer.bundle.js").is_file()
    assert 'src="./dist/viewer.bundle.js"' in html or 'src="dist/viewer.bundle.js"' in html or "src='./dist/viewer.bundle.js'" in html or "src='dist/viewer.bundle.js'" in html


@pytest.mark.parametrize("path", RUNTIME_FILES, ids=lambda p: p.relative_to(ROOT).as_posix())
def test_runtime_viewer_files_do_not_reference_cdn_urls(path):
    assert path.is_file(), f"runtime viewer file is missing: {path.relative_to(ROOT)}"
    text = _text(path)
    assert "unpkg" not in text.lower()
    assert "jsdelivr" not in text.lower()
    assert not CDN_RE.search(text)


@pytest.mark.parametrize("path", [VIEWER / "index.html", VIEWER / "dist" / "viewer.bundle.js"], ids=lambda p: p.relative_to(ROOT).as_posix())
def test_delivered_runtime_files_have_no_unresolved_bare_three_or_urdf_imports(path):
    assert not BARE_THREE_URDF_IMPORT_RE.search(_text(path))


def test_dependency_lockfile_and_runtime_versions_are_pinned_exactly():
    lockfile = VIEWER / "package-lock.json"
    assert lockfile.is_file()
    package = json.loads((VIEWER / "package.json").read_text(encoding="utf-8"))
    lock = json.loads(lockfile.read_text(encoding="utf-8"))
    deps = package["dependencies"]
    assert deps["three"] == "0.160.0"
    assert deps["urdf-loader"] == "0.13.0"
    assert deps["esbuild"] == "0.20.2"
    assert deps["@esbuild/linux-x64"] == "0.20.2"
    assert lock["packages"][""]["dependencies"]["three"] == "0.160.0"
    assert lock["packages"][""]["dependencies"]["urdf-loader"] == "0.13.0"
    assert lock["packages"]["node_modules/three"]["version"] == "0.160.0"
    assert lock["packages"]["node_modules/urdf-loader"]["version"] == "0.13.0"
    assert lock["packages"]["node_modules/esbuild"]["version"] == "0.20.2"


def test_stale_bundle_guard_runs_when_node_dependencies_are_available_or_is_wired():
    package = json.loads((VIEWER / "package.json").read_text(encoding="utf-8"))
    assert package["scripts"]["check:stale-bundle"] == "node scripts/check_stale_bundle.mjs"
    assert (VIEWER / "scripts" / "check_stale_bundle.mjs").is_file()
    if shutil.which("node") and (VIEWER / "node_modules" / "esbuild").exists():
        result = subprocess.run(
            ["npm", "run", "check:stale-bundle"],
            cwd=VIEWER,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        assert result.returncode == 0, result.stdout + result.stderr
    else:
        workflow_text = "\n".join(path.read_text(encoding="utf-8") for path in (ROOT / ".github" / "workflows").glob("*.yml"))
        readme = (VIEWER / "README.md").read_text(encoding="utf-8")
        assert "check:stale-bundle" in workflow_text or "npm run check:stale-bundle" in readme
