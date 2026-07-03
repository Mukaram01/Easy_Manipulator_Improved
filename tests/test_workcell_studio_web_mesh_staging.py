import importlib.util
import json
import subprocess
import sys
from pathlib import Path
from urllib.parse import urljoin, urlparse

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
EXPORTER = REPO_ROOT / "scripts" / "export_workcell_studio_web_scene.py"
VIEWER_JS = REPO_ROOT / "workcell_studio_web" / "viewer" / "viewer.js"

spec = importlib.util.spec_from_file_location("export_workcell_studio_web_scene", EXPORTER)
exporter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(exporter)


def _write_yaml(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def _write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, sort_keys=True), encoding="utf-8")


def _mesh(path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("solid dummy\nendsolid dummy\n", encoding="utf-8")
    return path


def _scene(tmp_path: Path, scene_id: str, visual_items: list[dict]) -> Path:
    scene = tmp_path / scene_id
    (scene / "generated").mkdir(parents=True)
    _write_yaml(scene / "environment.yaml", {"scene": {"id": scene_id, "name": scene_id}})
    _write_json(scene / "generated" / "scene_visual_mesh_index.json", {"visual_items": visual_items})
    return scene


def _package(prefix: Path, package: str, rel: str, contents: str = "solid pkg\nendsolid pkg\n") -> Path:
    mesh = prefix / "share" / package / rel
    mesh.parent.mkdir(parents=True, exist_ok=True)
    mesh.write_text(contents, encoding="utf-8")
    return mesh


def _export_with_prefix(monkeypatch, scene: Path, output: Path, prefix: Path | None = None, *, stage_assets: bool = True) -> dict:
    if prefix is not None:
        monkeypatch.setenv("AMENT_PREFIX_PATH", str(prefix))
    else:
        monkeypatch.delenv("AMENT_PREFIX_PATH", raising=False)
    payload = exporter.build_web_scene(scene, stage_assets=stage_assets, output_path=output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    return payload


def _all_renderable_items(payload: dict) -> list[dict]:
    items: list[dict] = []
    for section in ("robots", "tools", "assets", "sensors", "zones"):
        items.extend(item for item in payload.get(section, []) if isinstance(item, dict))
    return items


def _item(payload: dict, item_id: str) -> dict:
    return next(item for item in _all_renderable_items(payload) if item.get("id") == item_id)


def test_package_mesh_uri_is_staged_and_preserves_original_uri(monkeypatch, tmp_path):
    prefix = tmp_path / "ament"
    _package(prefix, "demo_meshes", "meshes/visual/cube.stl")
    package_uri = "package://demo_meshes/meshes/visual/cube.stl"
    scene = _scene(tmp_path, "stage_pkg_scene", [{"id": "cube", "category": "asset", "mesh_uri": package_uri}])

    payload = _export_with_prefix(monkeypatch, scene, tmp_path / "out" / "scene.web_scene.json", prefix)
    staged = _item(payload, "cube")

    assert staged["mesh_staging_status"] == "staged"
    assert staged["mesh_uri"].startswith("build/workcell_studio_web_scene/assets/stage_pkg_scene/")
    assert "package://" not in staged["mesh_uri"]
    assert staged["original_mesh_uri"] == package_uri
    assert (REPO_ROOT / staged["mesh_uri"]).is_file()


def test_missing_package_source_reports_clear_unresolved_diagnostic(monkeypatch, tmp_path):
    package_uri = "package://missing_pkg/meshes/ghost.dae"
    scene = _scene(tmp_path, "missing_pkg_scene", [{"id": "ghost", "category": "asset", "mesh_uri": package_uri}])

    payload = _export_with_prefix(monkeypatch, scene, tmp_path / "out" / "scene.web_scene.json", tmp_path / "ament")
    ghost = _item(payload, "ghost")

    assert ghost["mesh_staging_status"] == "resolve_failed"
    assert ghost["mesh_uri"] == package_uri
    assert ghost["original_mesh_uri"] == package_uri
    assert "Could not resolve package mesh URI" in ghost["mesh_resolve_warning"]
    assert "missing_pkg" in ghost["mesh_resolve_warning"]


def test_staged_paths_remain_under_scene_asset_root_and_duplicate_names_do_not_collide(monkeypatch, tmp_path):
    prefix = tmp_path / "ament"
    _package(prefix, "pkg_a", "meshes/shared/part.stl", "solid a\nendsolid a\n")
    _package(prefix, "pkg_b", "meshes/other/part.stl", "solid b\nendsolid b\n")
    scene = _scene(
        tmp_path,
        "collision_scene",
        [
            {"id": "a", "category": "asset", "mesh_uri": "package://pkg_a/meshes/shared/part.stl"},
            {"id": "b", "category": "asset", "mesh_uri": "package://pkg_b/meshes/other/part.stl"},
        ],
    )

    payload = _export_with_prefix(monkeypatch, scene, tmp_path / "out" / "scene.web_scene.json", prefix)
    uris = {_item(payload, item_id)["mesh_uri"] for item_id in ("a", "b")}

    assert len(uris) == 2
    for uri in uris:
        assert uri.startswith("build/workcell_studio_web_scene/assets/collision_scene/")
        staged_path = (REPO_ROOT / uri).resolve()
        asset_root = (REPO_ROOT / "build" / "workcell_studio_web_scene" / "assets" / "collision_scene").resolve()
        assert staged_path.relative_to(asset_root)
        assert staged_path.is_file()


def test_unsafe_mesh_paths_are_rejected(monkeypatch, tmp_path):
    unsafe_abs = _mesh(tmp_path / "outside_allowed_roots" / "outside.stl").resolve()
    scene = _scene(
        tmp_path,
        "unsafe_scene",
        [
            {"id": "traversal", "category": "asset", "mesh_uri": "../escape.stl"},
            {"id": "absolute", "category": "asset", "mesh_uri": str(unsafe_abs)},
            {"id": "scheme", "category": "asset", "mesh_uri": "http://example.test/mesh.stl"},
            {"id": "package_traversal", "category": "asset", "mesh_uri": "package://pkg/../escape.stl"},
        ],
    )

    payload = _export_with_prefix(monkeypatch, scene, tmp_path / "out" / "scene.web_scene.json", tmp_path / "ament")

    assert _item(payload, "traversal")["mesh_staging_status"] == "unsafe_path"
    assert _item(payload, "absolute")["mesh_staging_status"] == "unsafe_path"
    assert _item(payload, "scheme")["mesh_staging_status"] == "unsupported_scheme"
    assert _item(payload, "package_traversal")["mesh_staging_status"] == "unsafe_path"
    assert "outside allowed roots" in _item(payload, "absolute")["mesh_resolve_warning"]


def test_viewer_uri_policy_allows_staged_assets_and_rejects_traversal():
    js = VIEWER_JS.read_text(encoding="utf-8")

    assert "build/workcell_studio_web_scene/assets/" in js
    assert "allowedRoots.some(root => pathOnly.startsWith(root))" in js
    assert "repoRootRelativeUrl(uri)" in js
    assert "fetch(repoRootRelativeUrl(sceneUrl)" in js
    assert "part === '..'" in js
    assert "mesh_uri path traversal rejected" in js


def test_staged_ur5_mesh_uri_resolves_from_builder_http_launch_context(tmp_path):
    scene = REPO_ROOT / "scenes" / "ur5_2f_test"
    output = REPO_ROOT / "build" / "workcell_studio_web_scene" / "ur5_2f_test.web_scene.json"

    subprocess.run(
        [sys.executable, str(EXPORTER), "--scene", str(scene), "--output", str(output), "--stage-assets"],
        cwd=REPO_ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    payload = json.loads(output.read_text(encoding="utf-8"))
    staged_items = [
        item for item in _all_renderable_items(payload)
        if str(item.get("mesh_uri") or "").startswith("build/workcell_studio_web_scene/assets/ur5_2f_test/")
    ]

    assert staged_items, "Expected ur5_2f_test export to include staged mesh URIs."
    viewer_url = "http://127.0.0.1:8765/workcell_studio_web/viewer/index.html?scene=build%2Fworkcell_studio_web_scene%2Fur5_2f_test.web_scene.json"
    for item in staged_items[:10]:
        mesh_uri = item["mesh_uri"]
        browser_request = urljoin("http://127.0.0.1:8765/", mesh_uri)
        parsed = urlparse(browser_request)
        assert parsed.netloc == "127.0.0.1:8765"
        assert parsed.path.startswith("/build/workcell_studio_web_scene/assets/ur5_2f_test/")
        assert (REPO_ROOT / parsed.path.lstrip("/")).is_file(), f"{mesh_uri} from {viewer_url} should target an existing staged file"


def _fallback_causes(payload: dict) -> list[str]:
    causes: list[str] = []
    for item in _all_renderable_items(payload):
        uri = str(item.get("mesh_uri") or item.get("package_uri") or "")
        status = str(item.get("mesh_staging_status") or "")
        warning = str(item.get("mesh_resolve_warning") or "")
        if uri.startswith("package://") or status in {"resolve_failed", "unsupported_scheme", "unsafe_path"} or warning:
            causes.append(f"{item.get('id')}:{status}:{uri}:{warning}")
    return causes


def test_ur5_2f_export_with_asset_staging_reduces_unresolved_package_fallback_causes(tmp_path):
    scene = REPO_ROOT / "scenes" / "ur5_2f_test"
    unstaged = subprocess.run(
        [sys.executable, str(EXPORTER), "--scene", str(scene), "--output", str(tmp_path / "unstaged.json"), "--no-stage-assets"],
        cwd=REPO_ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    staged = subprocess.run(
        [sys.executable, str(EXPORTER), "--scene", str(scene), "--output", str(tmp_path / "staged.json"), "--stage-assets"],
        cwd=REPO_ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    assert unstaged.returncode == 0
    assert staged.returncode == 0
    unstaged_payload = json.loads((tmp_path / "unstaged.json").read_text(encoding="utf-8"))
    staged_payload = json.loads((tmp_path / "staged.json").read_text(encoding="utf-8"))

    assert len(_fallback_causes(staged_payload)) < len(_fallback_causes(unstaged_payload))
    assert any("package://" in cause for cause in _fallback_causes(unstaged_payload))
