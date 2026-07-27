import importlib.util
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path
from urllib.parse import urljoin, urlparse

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
EXPORTER = REPO_ROOT / "scripts" / "export_workcell_studio_web_scene.py"
FRESHENER = REPO_ROOT / "scripts" / "ensure_workcell_studio_web_scene_fresh.py"
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


def test_nested_layout_target_bin_mesh_is_the_authoritative_physical_render(monkeypatch, tmp_path):
    scene = tmp_path / "nested_target_bin_scene"
    _write_yaml(
        scene / "layout" / "workcell_studio_layout.yaml",
        {
            "items": [
                {
                    "id": "target_bin_default",
                    "type": "target_bin",
                    "role": "target_bin",
                    "category": "place_zone",
                    "pose": {"xyz": [0.55, -0.28, 0.19], "rpy": [0.0, 0.0, 0.0]},
                    "dimensions": [0.35, 0.25, 0.18],
                    "geometry_type": "mesh",
                    "mesh": {
                        "path": "assets/environment/sorting_bin_description/meshes/sorting_bin.stl",
                        "scale": [0.001, 0.001, 0.001],
                        "rpy": [0.0, 0.0, 0.0],
                        "origin_offset": [0.0, 0.0, 0.0],
                    },
                },
                {
                    "id": "place_zone_default",
                    "type": "place_zone",
                    "role": "place_zone",
                    "category": "work_surface",
                    "dimensions": [0.35, 0.30, 0.01],
                },
            ]
        },
    )
    _write_yaml(
        scene / "environment.yaml",
        {
            "scene": {"id": scene.name, "name": scene.name},
            "environment": {
                "assets": [
                    {
                        "id": "target_bin_default",
                        "type": "target_bin",
                        "role": "target_bin",
                        "category": "place_zone",
                        "layout_item_ref": "target_bin_default",
                        "support_surface_ref": "support_surface_table",
                        "task_zone_ref": "default_drop_zone",
                    }
                ]
            },
        },
    )
    staged_root = REPO_ROOT / "build" / "workcell_studio_web_scene" / "assets" / scene.name

    try:
        payload = _export_with_prefix(monkeypatch, scene, tmp_path / "out" / "scene.web_scene.json")
        matching = [item for item in _all_renderable_items(payload) if item.get("id") == "target_bin_default"]

        assert len(matching) == 1
        target_bin = matching[0]
        assert target_bin["source_kind"] == "user_authored"
        assert target_bin["render_policy"] == "primary"
        assert target_bin["render_owner"] == "editable_layout"
        assert target_bin["mesh_staging_status"] == "staged"
        assert target_bin["mesh_scale"] == [0.001, 0.001, 0.001]
        assert target_bin["mesh_local_transform"] == {
            "xyz": [0.0, 0.0, 0.0],
            "rpy": [0.0, 0.0, 0.0],
            "scale": [0.001, 0.001, 0.001],
        }
        assert target_bin["visual_origin"] == {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}
        assert target_bin["mesh_uri"] == target_bin["mesh_url"]
        assert target_bin["mesh_uri"].endswith("sorting_bin.stl")
        assert target_bin["support_surface_ref"] == "support_surface_table"
        assert target_bin["task_zone_ref"] == "default_drop_zone"
        assert target_bin["mesh_contract_category"] == "object"
        assert payload["metadata"]["mesh_contract"]["mesh_contract_status"] == "passed"
        assert any(item.get("id") == "place_zone_default" for item in payload["zones"])
        assert (REPO_ROOT / target_bin["mesh_uri"]).is_file()
    finally:
        shutil.rmtree(staged_root, ignore_errors=True)


def test_ur5_target_bin_calibrated_mesh_transform_and_bounds_survive_export():
    scene = REPO_ROOT / "scenes" / "ur5_2f_test"
    payload = json.loads(json.dumps(exporter.build_web_scene(scene)))
    target_bin = _item(payload, "target_bin_default")
    place_zone = _item(payload, "place_zone_default")

    assert target_bin["transform_group"] == "default_drop_destination"
    assert place_zone["transform_group"] == "default_drop_destination"

    world_pose = {"xyz": [0.55, -0.28, 0.20], "rpy": [0.0, 0.0, 0.0]}
    mesh_transform = {
        "xyz": [-0.1738994366, 0.0, -0.10],
        "rpy": [1.57079632679, 0.0, 1.57079632679],
        "scale": [0.001, 0.001, 0.001],
    }
    assert target_bin["pose"] == world_pose
    assert target_bin["mesh_local_transform"] == mesh_transform
    assert target_bin["pose"] is not target_bin["mesh_local_transform"]
    assert target_bin["dimensions"] == [0.3522011268, 0.2133871002, 0.20]

    # Apply the exported XYZ fixed-axis RPY transform to all STL bound corners.
    stl_bounds_mm = (
        (-106.69355, 106.69355),
        (0.0, 200.0),
        (-2.2011268, 350.0),
    )
    roll, pitch, yaw = mesh_transform["rpy"]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    transformed = []
    for x in stl_bounds_mm[0]:
        for y in stl_bounds_mm[1]:
            for z in stl_bounds_mm[2]:
                scaled = tuple(value * scale for value, scale in zip((x, y, z), mesh_transform["scale"]))
                rotated = tuple(sum(rotation[row][column] * scaled[column] for column in range(3)) for row in range(3))
                transformed.append(
                    tuple(world_pose["xyz"][axis] + mesh_transform["xyz"][axis] + rotated[axis] for axis in range(3))
                )
    bounds = tuple((min(point[axis] for point in transformed), max(point[axis] for point in transformed)) for axis in range(3))

    expected = ((0.3738994366, 0.7261005634), (-0.38669355, -0.17330645), (0.10, 0.30))
    for actual_axis, expected_axis in zip(bounds, expected):
        assert all(math.isclose(actual, wanted, abs_tol=1e-9) for actual, wanted in zip(actual_axis, expected_axis))
    assert math.isclose(sum(bounds[0]) / 2, 0.55, abs_tol=1e-9)
    assert math.isclose(sum(bounds[1]) / 2, -0.28, abs_tol=1e-9)
    assert math.isclose(bounds[2][0], 0.10, abs_tol=1e-9)


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
    assert "STAGED_MESH_ROOTS.some(root => pathOnly.startsWith(root))" in js
    assert "repoRootRelativeUrl(uri)" in js
    assert "fetch(repoRootRelativeUrl(sceneUrl)" in js
    assert "part === '..'" in js
    assert "mesh_uri path traversal rejected" in js


def test_staged_mesh_uri_resolves_from_builder_opened_viewer_base_url():
    mesh_uri = "build/workcell_studio_web_scene/assets/ur5_2f_test/ur_description/meshes/ur5/visual/base.dae"
    staged_mesh = REPO_ROOT / mesh_uri
    staged_mesh.parent.mkdir(parents=True, exist_ok=True)
    had_existing_mesh = staged_mesh.exists()
    if not had_existing_mesh:
        staged_mesh.write_text("<COLLADA><asset/></COLLADA>\n", encoding="utf-8")

    try:
        staged_item = {
            "id": "ur5_base_link",
            "category": "robot",
            "mesh_uri": mesh_uri,
            "mesh_staging_status": "staged",
        }
        viewer_url = "http://127.0.0.1:8765/workcell_studio_web/viewer/index.html?scene=build%2Fworkcell_studio_web_scene%2Fur5_2f_test.web_scene.json"

        browser_default_request = urljoin(viewer_url, staged_item["mesh_uri"])
        default_parsed = urlparse(browser_default_request)
        assert default_parsed.path.startswith("/workcell_studio_web/viewer/build/"), (
            "This fixture must model the browser's default relative-URL resolution from the "
            "Builder-opened viewer location; without a repo-root rewrite it would fetch from "
            "workcell_studio_web/viewer/build/..."
        )

        viewer_js = VIEWER_JS.read_text(encoding="utf-8")
        scene_select_cpp = (REPO_ROOT / "workcell_builder" / "workcell_builder" / "gui" / "scene_select.cpp").read_text(encoding="utf-8")
        assert "function repoRootRelativeUrl(uri)" in viewer_js
        assert "new URL(uri, base).href" in viewer_js
        assert "repoRootRelativeUrl(uri)" in viewer_js
        assert 'QString::fromStdString(repo_root.string())' in scene_select_cpp
        assert "python3 -m http.server 8765 --bind 127.0.0.1" in scene_select_cpp

        implemented_request = urljoin("http://127.0.0.1:8765/", staged_item["mesh_uri"])
        implemented_parsed = urlparse(implemented_request)
        assert implemented_parsed.netloc == "127.0.0.1:8765"
        assert implemented_parsed.path == "/" + mesh_uri
        assert not implemented_parsed.path.startswith("/workcell_studio_web/viewer/build/")
        assert implemented_parsed.path.startswith("/build/workcell_studio_web_scene/assets/ur5_2f_test/")
        assert (REPO_ROOT / implemented_parsed.path.lstrip("/")).is_file(), (
            f"{staged_item['mesh_uri']} from {viewer_url} must be fetchable from the "
            "repository-root static server."
        )
    finally:
        if not had_existing_mesh:
            staged_mesh.unlink(missing_ok=True)



def test_required_core_mesh_contract_records_full_staging_fields(monkeypatch, tmp_path):
    prefix = tmp_path / "ament"
    _package(prefix, "demo_robot", "meshes/visual/base.dae", "<COLLADA></COLLADA>\n")
    mesh_uri = "package://demo_robot/meshes/visual/base.dae"
    scene = _scene(
        tmp_path,
        "contract_scene",
        [{
            "id": "robot_base",
            "category": "robot_static_mesh_visual",
            "role": "robot",
            "link": "base_link",
            "geometry_type": "mesh",
            "mesh_uri": mesh_uri,
            "source_path": mesh_uri,
            "render_expected": True,
            "active_visual_source": "mesh_preview",
        }],
    )

    payload = _export_with_prefix(monkeypatch, scene, tmp_path / "out" / "scene.web_scene.json", prefix)
    item = _item(payload, "robot_base")

    assert item["original_mesh_uri"] == mesh_uri
    assert item["original_package_uri"] == mesh_uri
    assert item["original_source_path"] == mesh_uri
    assert item["resolved_source_path"].endswith("share/demo_robot/meshes/visual/base.dae") or item["resolved_source_path"].endswith("demo_robot/meshes/visual/base.dae")
    assert item["mesh_staged_path"] == item["mesh_uri"]
    assert item["mesh_url"] == item["mesh_uri"]
    assert item["mesh_format"] == "dae"
    assert item["mesh_load_required"] is True
    assert item["core_mesh_category"] == "robot_arm_link"
    contract = payload["metadata"]["mesh_contract"]
    assert contract == {
        "required_mesh_count": 1,
        "staged_mesh_count": 1,
        "missing_required_meshes": [],
        "fallback_primitive_count": 0,
        "core_mesh_failures": [],
        "mesh_contract_status": "passed",
    }


def test_stage_assets_reports_required_core_mesh_failures(monkeypatch, tmp_path):
    mesh_uri = "package://missing_robot/meshes/visual/base.dae"
    scene = _scene(
        tmp_path,
        "contract_failure_scene",
        [{
            "id": "missing_robot_base",
            "category": "robot_static_mesh_visual",
            "role": "robot",
            "geometry_type": "mesh",
            "mesh_uri": mesh_uri,
            "render_expected": True,
        }],
    )

    payload = _export_with_prefix(monkeypatch, scene, tmp_path / "out" / "scene.web_scene.json", tmp_path / "ament")
    item = _item(payload, "missing_robot_base")

    assert item["mesh_load_required"] is True
    assert item["mesh_staging_status"] == "resolve_failed"
    assert item["mesh_url"] is None
    contract = payload["metadata"]["mesh_contract"]
    assert contract["required_mesh_count"] == 1
    assert contract["staged_mesh_count"] == 0
    assert contract["mesh_contract_status"] == "failed"
    assert contract["missing_required_meshes"] == contract["core_mesh_failures"]
    assert contract["core_mesh_failures"][0]["id"] == "missing_robot_base"
    assert contract["core_mesh_failures"][0]["category"] == "robot_arm_link"

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
    subprocess.run(
        [sys.executable, str(FRESHENER), "--scene", str(scene), "--output", str(tmp_path / "fresh.web_scene.json"), "--stage-assets", "--force"],
        cwd=REPO_ROOT,
        check=True,
    )
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


def _repo_discovered_package(root: Path, package: str, rel: str, contents: str = "solid pkg\nendsolid pkg\n") -> Path:
    pkg_dir = root / "assets" / "end_effectors" / "nested" / package
    (pkg_dir).mkdir(parents=True, exist_ok=True)
    (pkg_dir / "package.xml").write_text(
        f"<package format=\"3\"><name>{package}</name><version>0.0.0</version><description>test</description><maintainer email=\"test@example.com\">Test</maintainer><license>Apache-2.0</license></package>\n",
        encoding="utf-8",
    )
    mesh = pkg_dir / rel
    mesh.parent.mkdir(parents=True, exist_ok=True)
    mesh.write_text(contents, encoding="utf-8")
    return mesh


def test_nested_repo_package_mesh_uri_uses_package_discovery(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    mesh = _repo_discovered_package(tmp_path, "nested_tool_description", "meshes/visual/finger.dae", "<COLLADA></COLLADA>\n")
    package_uri = "package://nested_tool_description/meshes/visual/finger.dae"
    scene = _scene(tmp_path, "nested_package_scene", [{"id": "finger", "category": "tool", "mesh_uri": package_uri}])

    payload = _export_with_prefix(monkeypatch, scene, tmp_path / "out" / "scene.web_scene.json", None)
    item = _item(payload, "finger")

    assert item["mesh_staging_status"] == "staged"
    assert item["resolved_source_path"] == str(mesh.relative_to(tmp_path)).replace("/", "/")
    assert item["original_package_uri"] == package_uri
    assert item["mesh_uri"] == "build/workcell_studio_web_scene/assets/nested_package_scene/nested_tool_description/meshes/visual/finger.dae"
    assert (tmp_path / item["mesh_uri"]).read_text(encoding="utf-8") == "<COLLADA></COLLADA>\n"


def test_package_uri_with_url_encoded_spaces_is_staged(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    mesh = _repo_discovered_package(tmp_path, "space_pkg", "meshes/visual/gripper jaw.dae", "<COLLADA></COLLADA>\n")
    scene = _scene(tmp_path, "space_scene", [{"id": "jaw", "category": "tool", "mesh_uri": "package://space_pkg/meshes/visual/gripper%20jaw.dae"}])

    payload = _export_with_prefix(monkeypatch, scene, tmp_path / "out" / "scene.web_scene.json", None)
    item = _item(payload, "jaw")

    assert item["mesh_staging_status"] == "staged"
    assert item["resolved_source_path"] == str(mesh.relative_to(tmp_path)).replace("/", "/")
    assert item["mesh_uri"].endswith("/space_pkg/meshes/visual/gripper jaw.dae")
    assert (tmp_path / item["mesh_uri"]).is_file()


def test_package_uri_accepts_uppercase_and_lowercase_dae_suffixes(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    _repo_discovered_package(tmp_path, "case_pkg", "meshes/visual/upper.DAE", "<COLLADA>upper</COLLADA>\n")
    _repo_discovered_package(tmp_path, "case_pkg", "meshes/visual/lower.dae", "<COLLADA>lower</COLLADA>\n")
    scene = _scene(
        tmp_path,
        "case_scene",
        [
            {"id": "upper", "category": "asset", "mesh_uri": "package://case_pkg/meshes/visual/upper.DAE"},
            {"id": "lower", "category": "asset", "mesh_uri": "package://case_pkg/meshes/visual/lower.dae"},
        ],
    )

    payload = _export_with_prefix(monkeypatch, scene, tmp_path / "out" / "scene.web_scene.json", None)

    assert _item(payload, "upper")["mesh_staging_status"] == "staged"
    assert _item(payload, "upper")["mesh_format"] == "dae"
    assert _item(payload, "upper")["mesh_uri"].endswith("upper.DAE")
    assert _item(payload, "lower")["mesh_staging_status"] == "staged"
    assert _item(payload, "lower")["mesh_format"] == "dae"
    assert _item(payload, "lower")["mesh_uri"].endswith("lower.dae")


def test_unknown_package_and_package_traversal_keep_explicit_failure_statuses(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    scene = _scene(
        tmp_path,
        "package_failure_scene",
        [
            {"id": "unknown", "category": "asset", "mesh_uri": "package://unknown_pkg/meshes/missing.dae"},
            {"id": "traversal", "category": "asset", "mesh_uri": "package://unknown_pkg/../secret.dae"},
        ],
    )

    payload = _export_with_prefix(monkeypatch, scene, tmp_path / "out" / "scene.web_scene.json", None)

    assert _item(payload, "unknown")["mesh_staging_status"] == "resolve_failed"
    assert "Could not resolve package mesh URI" in _item(payload, "unknown")["mesh_resolve_warning"]
    assert _item(payload, "traversal")["mesh_staging_status"] == "unsafe_path"
    assert _item(payload, "traversal")["mesh_uri"] == "package://unknown_pkg/../secret.dae"


def test_staged_url_maps_to_intended_file_under_scene_asset_root(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    mesh = _repo_discovered_package(tmp_path, "map_pkg", "meshes/visual/part.dae", "<COLLADA>mapped</COLLADA>\n")
    scene = _scene(tmp_path, "mapped_scene", [{"id": "mapped", "category": "asset", "mesh_uri": "package://map_pkg/meshes/visual/part.dae"}])

    payload = _export_with_prefix(monkeypatch, scene, tmp_path / "out" / "scene.web_scene.json", None)
    item = _item(payload, "mapped")
    staged = (tmp_path / item["mesh_url"]).resolve()
    asset_root = (tmp_path / "build" / "workcell_studio_web_scene" / "assets" / "mapped_scene").resolve()

    assert item["mesh_staging_status"] == "staged"
    assert item["mesh_staged_path"] == item["mesh_url"] == item["mesh_uri"]
    assert staged.relative_to(asset_root)
    assert staged.read_text(encoding="utf-8") == mesh.read_text(encoding="utf-8")


def test_known_package_missing_mesh_file_reports_missing_file(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    _repo_discovered_package(tmp_path, "known_missing_pkg", "meshes/visual/existing.dae", "<COLLADA></COLLADA>\n")
    scene = _scene(tmp_path, "known_missing_scene", [{"id": "missing", "category": "asset", "mesh_uri": "package://known_missing_pkg/meshes/visual/missing.dae"}])

    payload = _export_with_prefix(monkeypatch, scene, tmp_path / "out" / "scene.web_scene.json", None)
    item = _item(payload, "missing")

    assert item["mesh_staging_status"] == "missing_file"
    assert "Package mesh file does not exist" in item["mesh_resolve_warning"]
    assert item["mesh_uri"] == "package://known_missing_pkg/meshes/visual/missing.dae"


def _expanded_robot_scene(tmp_path: Path, scene_id: str, mesh_uris: list[tuple[str, str]]) -> Path:
    scene = _scene(tmp_path, scene_id, [])
    links = ['world', 'base_link', 'tool0', 'gripper_base_link']
    link_blocks: dict[str, list[str]] = {}
    for link, uri in mesh_uris:
        kind = 'visual'
        if '|' in link:
            link, kind = link.split('|', 1)
        link_blocks.setdefault(link, []).append(
            f'    <{kind}>\n      <origin xyz="0 0 0" rpy="0 0 0"/>\n      <geometry><mesh filename="{uri}" scale="1 1 1"/></geometry>\n    </{kind}>'
        )
    visuals = [f'  <link name="{link}">\n' + '\n'.join(blocks) + '\n  </link>' for link, blocks in link_blocks.items()]
    for link in links:
        if link not in link_blocks:
            visuals.append(f'  <link name="{link}"/>')
    joints = '''  <joint name="world_to_base" type="fixed"><parent link="world"/><child link="base_link"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>\n  <joint name="base_to_tool" type="fixed"><parent link="base_link"/><child link="tool0"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>\n  <joint name="tool_to_gripper" type="fixed"><parent link="tool0"/><child link="gripper_base_link"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>'''
    (scene / 'generated' / 'expanded_scene_preview.urdf').write_text('<robot name="fixture">\n' + '\n'.join(visuals) + '\n' + joints + '\n</robot>\n', encoding='utf-8')
    _write_json(scene / 'generated' / 'scene_visual_mesh_index.json', {
        'source_expanded_urdf_path': str(scene / 'generated' / 'expanded_scene_preview.urdf'),
        'visual_items': [],
    })
    return scene


def test_expanded_urdf_meshes_use_canonical_root_relative_urls(monkeypatch, tmp_path):
    prefix = tmp_path / 'ament'
    _package(prefix, 'generic_tool_description', 'meshes/visual/tool.dae', '<COLLADA/>')
    scene = _expanded_robot_scene(
        tmp_path,
        'canonical_scene',
        [('base_link', 'package://generic_tool_description/meshes/visual/tool.dae')],
    )

    payload = _export_with_prefix(monkeypatch, scene, tmp_path / 'out' / 'scene.web_scene.json', prefix)
    urdf_path = REPO_ROOT / payload['robot_preview']['urdf_url']
    root = exporter.ET.fromstring(urdf_path.read_text(encoding='utf-8'))
    filenames = [mesh.get('filename') for mesh in root.findall('.//mesh')]

    assert filenames == ['/build/workcell_studio_web_scene/assets/canonical_scene/generic_tool_description/meshes/visual/tool.dae']
    for filename in filenames:
        assert 'package://' not in filename
        assert '..' not in filename
        assert (REPO_ROOT / filename.lstrip('/')).is_file()
        resolved = urljoin('http://127.0.0.1:8765/build/workcell_studio_web_scene/canonical_scene.expanded.urdf', filename)
        assert resolved.startswith('http://127.0.0.1:8765/build/workcell_studio_web_scene/assets/canonical_scene/')
        assert '/build/workcell_studio_web_scene/build/' not in resolved
    diagnostics = payload['metadata']['expanded_urdf_staging']
    assert diagnostics['expanded_urdf_mesh_reference_count'] == 1
    assert diagnostics['expanded_urdf_staged_mesh_count'] == 1
    assert diagnostics['expanded_urdf_unresolved_mesh_count'] == 0
    assert diagnostics['expanded_urdf_package_count'] == 1
    assert diagnostics['expanded_urdf_uses_canonical_root_relative_urls'] is True


def test_expanded_urdf_visual_and_collision_meshes_are_canonicalized_once(monkeypatch, tmp_path):
    prefix = tmp_path / 'ament'
    _package(prefix, 'generic_robot_description', 'meshes/visual/base.dae', '<COLLADA>visual</COLLADA>')
    _package(prefix, 'generic_robot_description', 'meshes/collision/base.stl', 'solid collision\nendsolid collision\n')
    scene = _expanded_robot_scene(
        tmp_path,
        'visual_collision_scene',
        [
            ('base_link', 'package://generic_robot_description/meshes/visual/base.dae'),
            ('base_link|collision', 'package://generic_robot_description/meshes/collision/base.stl'),
            ('tool0|collision', 'package://generic_robot_description/meshes/collision/base.stl'),
        ],
    )

    payload = _export_with_prefix(monkeypatch, scene, tmp_path / 'out' / 'scene.web_scene.json', prefix)
    urdf_path = REPO_ROOT / payload['robot_preview']['urdf_url']
    root = exporter.ET.fromstring(urdf_path.read_text(encoding='utf-8'))
    filenames = [mesh.get('filename') for mesh in root.findall('.//mesh')]

    assert filenames == [
        '/build/workcell_studio_web_scene/assets/visual_collision_scene/generic_robot_description/meshes/visual/base.dae',
        '/build/workcell_studio_web_scene/assets/visual_collision_scene/generic_robot_description/meshes/collision/base.stl',
        '/build/workcell_studio_web_scene/assets/visual_collision_scene/generic_robot_description/meshes/collision/base.stl',
    ]
    assert all('package://' not in filename and 'file://' not in filename for filename in filenames)
    diagnostics = payload['metadata']['expanded_urdf_staging']
    assert diagnostics['expanded_urdf_mesh_reference_count'] == 3
    assert diagnostics['expanded_urdf_visual_mesh_reference_count'] == 1
    assert diagnostics['expanded_urdf_collision_mesh_reference_count'] == 2
    assert diagnostics['expanded_urdf_visual_meshes_staged'] == 1
    assert diagnostics['expanded_urdf_collision_meshes_staged'] == 1
    assert diagnostics['expanded_urdf_deduplicated_reference_count'] == 1
    assert diagnostics['expanded_urdf_unresolved_references'] == []
    readiness = payload['robot_preview']
    assert readiness['expected_visual_links'] == ['base_link']
    assert readiness['expected_tool_visual_links'] == []


def test_expanded_urdf_unresolved_collision_mesh_remains_blocking(monkeypatch, tmp_path):
    monkeypatch.setenv('AMENT_PREFIX_PATH', str(tmp_path / 'ament'))
    scene = _expanded_robot_scene(
        tmp_path,
        'missing_collision_expanded_scene',
        [('base_link|collision', 'package://missing_robot/meshes/collision/base.stl')],
    )

    try:
        exporter.build_web_scene(scene, stage_assets=True, output_path=tmp_path / 'out.json')
    except exporter.BlockingExportError as exc:
        message = str(exc)
    else:  # pragma: no cover
        raise AssertionError('missing required expanded URDF collision mesh did not fail')

    assert 'package://missing_robot/meshes/collision/base.stl' in message
    assert 'collision_index=0' in message
    assert 'Could not resolve package mesh URI' in message


def test_expanded_urdf_missing_required_mesh_fails_clearly(monkeypatch, tmp_path):
    monkeypatch.setenv('AMENT_PREFIX_PATH', str(tmp_path / 'ament'))
    scene = _expanded_robot_scene(
        tmp_path,
        'missing_expanded_scene',
        [('base_link', 'package://missing_robot/meshes/visual/base.dae')],
    )

    try:
        exporter.build_web_scene(scene, stage_assets=True, output_path=tmp_path / 'out.json')
    except exporter.BlockingExportError as exc:
        message = str(exc)
    else:  # pragma: no cover
        raise AssertionError('missing required expanded URDF mesh did not fail')

    assert 'package://missing_robot/meshes/visual/base.dae' in message
    assert 'Could not resolve package mesh URI' in message
    staged = REPO_ROOT / 'build' / 'workcell_studio_web_scene' / 'missing_expanded_scene.expanded.urdf'
    assert not staged.exists() or 'package://missing_robot' not in staged.read_text(encoding='utf-8')


def test_expanded_urdf_rejects_unsafe_and_non_package_mesh_uris(monkeypatch, tmp_path):
    monkeypatch.setenv('AMENT_PREFIX_PATH', str(tmp_path / 'ament'))
    bad_uris = [
        'package://bad_pkg/../escape.dae',
        'package://bad_pkg/meshes/%2e%2e/escape.dae',
        'https://example.test/mesh.dae',
        'file:///tmp/mesh.dae',
        '/tmp/mesh.dae',
    ]
    for index, uri in enumerate(bad_uris):
        scene = _expanded_robot_scene(tmp_path, f'bad_expanded_{index}', [('base_link', uri)])
        try:
            exporter.build_web_scene(scene, stage_assets=True, output_path=tmp_path / f'out_{index}.json')
        except exporter.BlockingExportError as exc:
            message = str(exc)
        else:  # pragma: no cover
            raise AssertionError(f'unsafe URI was accepted: {uri}')
        assert uri in message
        assert ('Invalid or unsafe package URI' in message) or ('must be a package:// URI' in message)


def test_real_nested_robotiq_85_package_uri_stages_from_repo_assets(monkeypatch, tmp_path):
    monkeypatch.chdir(REPO_ROOT)
    package_uri = "package://robotiq_85_description/meshes/visual/robotiq_85_base_link.dae"
    scene = _scene(tmp_path, "real_nested_2f_scene", [{"id": "robotiq_85_base", "category": "tool", "mesh_uri": package_uri}])

    payload = _export_with_prefix(monkeypatch, scene, tmp_path / "out" / "scene.web_scene.json", None)
    item = _item(payload, "robotiq_85_base")

    assert item["mesh_staging_status"] == "staged"
    assert item["resolved_source_path"] == "assets/end_effectors/robotiq_85_gripper/robotiq_85_description/meshes/visual/robotiq_85_base_link.dae"
    assert item["mesh_uri"] == "build/workcell_studio_web_scene/assets/real_nested_2f_scene/robotiq_85_description/meshes/visual/robotiq_85_base_link.dae"
    assert (REPO_ROOT / item["mesh_uri"]).is_file()


def test_real_nested_robotiq_3f_package_uri_stages_from_repo_assets(monkeypatch, tmp_path):
    monkeypatch.chdir(REPO_ROOT)
    package_uri = "package://robotiq_3f_gripper_description/meshes/robotiq-3f-gripper_articulated/visual/palm.dae"
    scene = _scene(tmp_path, "real_nested_3f_scene", [{"id": "robotiq_3f_palm", "category": "tool", "mesh_uri": package_uri}])

    payload = _export_with_prefix(monkeypatch, scene, tmp_path / "out" / "scene.web_scene.json", None)
    item = _item(payload, "robotiq_3f_palm")

    assert item["mesh_staging_status"] == "staged"
    assert item["resolved_source_path"] == "assets/end_effectors/robotiq_3f_gripper/robotiq_3f_gripper_description/meshes/robotiq-3f-gripper_articulated/visual/palm.dae"
    assert item["mesh_uri"] == "build/workcell_studio_web_scene/assets/real_nested_3f_scene/robotiq_3f_gripper_description/meshes/robotiq-3f-gripper_articulated/visual/palm.dae"
    assert (REPO_ROOT / item["mesh_uri"]).is_file()


def test_package_resolver_augments_incomplete_map_and_waits_for_existing_mesh(tmp_path):
    stale = tmp_path / "stale" / "share" / "aug_pkg"
    stale.mkdir(parents=True)
    (stale / "package.xml").write_text("<package><name>aug_pkg</name></package>\n", encoding="utf-8")
    repo_mesh = _repo_discovered_package(tmp_path, "aug_pkg", "meshes/visual/ok.dae", "<COLLADA>repo</COLLADA>\n")

    resolved, package, dest_rel, warning, checked = exporter.resolve_package_mesh_uri(
        "package://aug_pkg/meshes/visual/ok.dae",
        repo_root=tmp_path,
        package_map={"aug_pkg": stale},
        supported_suffixes=exporter.SUPPORTED_MESH_SUFFIXES,
    )

    assert resolved == repo_mesh.resolve()
    assert package == "aug_pkg"
    assert dest_rel == Path("aug_pkg/meshes/visual/ok.dae")
    assert warning is None
    assert checked[0] == repo_mesh.parents[2].resolve()


def test_expanded_urdf_staging_uses_repository_root_when_cwd_is_scene(monkeypatch):
    scene = REPO_ROOT / "scenes" / "ur5_2f_test"
    source = scene / "expanded_root_regression.urdf"
    source.write_text(
        '<robot name="root_regression"><link name="world"/><link name="base_link"><visual><geometry><mesh filename="package://robotiq_85_description/meshes/visual/robotiq_85_base_link.dae"/></geometry></visual></link><joint name="world_to_base" type="fixed"><parent link="world"/><child link="base_link"/></joint></robot>\n',
        encoding="utf-8",
    )
    try:
        monkeypatch.chdir(scene)
        payload = {"scene": {"id": "ur5_2f_test"}, "_visual_mesh_index_source": {"source_expanded_urdf_path": str(source)}}

        exporter._stage_expanded_robot_urdf(payload, scene, scene / "tmp.web_scene.json", [])
    finally:
        source.unlink(missing_ok=True)

    preview = payload["robot_preview"]
    assert preview["mode"] == "expanded_urdf_loader"
    assert preview["urdf_url"].startswith("build/workcell_studio_web_scene/")
    diagnostics = payload["metadata"]["expanded_urdf_staging"]
    assert diagnostics["expanded_urdf_unresolved_mesh_count"] == 0
    assert (REPO_ROOT / "build" / "workcell_studio_web_scene" / "assets" / "ur5_2f_test" / "robotiq_85_description" / "meshes" / "visual" / "robotiq_85_base_link.dae").is_file()
