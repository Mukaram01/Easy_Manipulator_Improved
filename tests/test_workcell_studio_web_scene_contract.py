import json
import math
import subprocess
import sys
from pathlib import Path

import yaml
from jsonschema import Draft202012Validator


REPO_ROOT = Path(__file__).resolve().parents[1]
EXPORTER = REPO_ROOT / "scripts" / "export_workcell_studio_web_scene.py"
SCHEMA = REPO_ROOT / "schemas" / "workcell_studio_web_scene_v1.schema.json"


def _write_yaml(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def _write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, sort_keys=True), encoding="utf-8")


def _tiny_scene_fixture(tmp_path: Path) -> Path:
    scene = tmp_path / "tiny_scene"
    (scene / "layout").mkdir(parents=True)
    (scene / "generated").mkdir()

    _write_yaml(
        scene / "environment.yaml",
        {
            "scene": {"id": "tiny_scene", "name": "Tiny Scene"},
            "environment": {
                "assets": [
                    {
                        "id": "workbench",
                        "type": "table",
                        "role": "support_surface",
                        "pose_xyz": [0.0, 0.0, 0.0],
                        "dimensions": [1.0, 0.7, 0.05],
                    }
                ],
                "sensors": [
                    {
                        "id": "inspection_camera",
                        "category": "camera",
                        "role": "camera",
                        "pose_xyz": [0.4, -0.2, 1.1],
                    }
                ],
            },
        },
    )
    _write_yaml(
        scene / "layout" / "workcell_studio_layout.yaml",
        {
            "schema_version": "workcell_studio_layout/v1",
            "items": [
                {
                    "id": "layout_bin",
                    "type": "target_bin",
                    "category": "bin",
                    "pose_xyz": [0.45, 0.25, 0.08],
                    "mesh_path": str(scene / "meshes" / "bin.stl"),
                }
            ],
        },
    )
    _write_json(
        scene / "generated" / "scene_visual_mesh_index.json",
        {
            "visual_items": [
                {
                    "id": "ur5_visual",
                    "category": "robot",
                    "role": "robot",
                    "mesh_path": str(scene / "meshes" / "ur5.dae"),
                },
                {
                    "id": "robotiq_2f_visual",
                    "category": "tool",
                    "role": "gripper",
                    "mesh_path": str(scene / "meshes" / "robotiq_2f.dae"),
                },
            ]
        },
    )
    return scene


def _export(scene: Path, output: Path) -> dict:
    subprocess.run(
        [sys.executable, str(EXPORTER), "--scene", str(scene), "--output", str(output)],
        cwd=REPO_ROOT,
        check=True,
    )
    return json.loads(output.read_text(encoding="utf-8"))


def _export_persisted_ur5_2f_web_scene() -> tuple[Path, dict]:
    output = REPO_ROOT / "build" / "workcell_studio_web_scene" / "ur5_2f_test.web_scene.json"
    output.unlink(missing_ok=True)
    subprocess.run(
        [
            sys.executable,
            str(EXPORTER),
            "--scene",
            str(REPO_ROOT / "scenes" / "ur5_2f_test"),
            "--output",
            str(output),
        ],
        cwd=REPO_ROOT,
        check=True,
    )
    assert output.exists()
    return output, json.loads(output.read_text(encoding="utf-8"))


def test_web_scene_schema_file_exists_and_is_valid_json():
    assert SCHEMA.exists()
    with SCHEMA.open(encoding="utf-8") as fh:
        schema = json.load(fh)
    assert schema["$id"].endswith("/workcell_studio_web_scene_v1.schema.json")
    assert schema["properties"]["schema_version"]["const"] == "workcell_studio_web_scene/v1"
    Draft202012Validator.check_schema(schema)


def test_tiny_scene_exports_generated_and_authored_asset_contract(tmp_path):
    scene = _tiny_scene_fixture(tmp_path)
    out1 = tmp_path / "out" / "scene.web_scene.json"
    out2 = tmp_path / "out" / "scene.again.web_scene.json"

    payload1 = _export(scene, out1)
    payload2 = _export(scene, out2)

    assert out1.read_bytes() == out2.read_bytes()
    assert payload1 == payload2
    schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
    Draft202012Validator(schema).validate(payload1)
    assert payload1["schema_version"] == "workcell_studio_web_scene/v1"
    assert payload1["scene"]["units"] == {"distance": "metre", "angle": "radian"}
    assert payload1["scene"]["coordinate_system"]["convention"] == "ros_world_z_up"
    assert payload1["inputs"]["environment"]["present"] is True
    assert payload1["inputs"]["layout"]["present"] is True
    assert payload1["inputs"]["visual_mesh_index"]["present"] is True

    summary = payload1["viewer_summary"]
    assert summary["renderable_count"] == sum(len(payload1[bucket]) for bucket in ("robots", "tools", "assets", "sensors", "zones"))
    assert summary["mesh_backed_count"] >= 0
    assert summary["fallback_count"] >= 1
    assert summary["missing_or_failed_mesh_count"] >= 1
    assert summary["scene_bounds"]["min"][0] <= 0.0
    assert summary["scene_bounds"]["max"][2] >= 1.1
    assert summary["required_item_status"]["robot"]["present"] is True
    assert summary["required_item_status"]["tool"]["present"] is True
    assert summary["required_item_status"]["table"]["present"] is True
    assert summary["required_item_status"]["camera"]["present"] is True

    robot = next(item for item in payload1["robots"] if item["id"] == "ur5_visual")
    tool = next(item for item in payload1["tools"] if item["id"] == "robotiq_2f_visual")
    for generated in (robot, tool):
        assert generated["source_kind"] == "generated_preview"
        assert generated["locked"] is True
        assert generated["editable"] is False
        assert generated["provenance"]["mesh_path"] == "generated/scene_visual_mesh_index.json"

    layout_asset = next(item for item in payload1["assets"] if item["id"] == "layout_bin")
    environment_asset = next(item for item in payload1["assets"] if item["id"] == "workbench")
    environment_sensor = next(item for item in payload1["sensors"] if item["id"] == "inspection_camera")
    for authored in (layout_asset, environment_asset, environment_sensor):
        assert authored["source_kind"] == "user_authored"
        assert authored["editable"] is True
        assert authored["locked"] is False

    actions = {action["id"]: action for action in payload1["backend_actions"]}
    assert actions["validate"]["request_kind"] == "backend_request"
    assert actions["generate_scene_package"]["enabled"] is True
    assert actions["plan_simulate"]["enabled"] is False
    assert "real hardware execution is not exposed" in actions["plan_simulate"]["safety_note"]


def test_missing_optional_inputs_warn_in_output_json_without_crashing(tmp_path):
    scene = tmp_path / "missing_optional_inputs_scene"
    scene.mkdir()

    payload = _export(scene, tmp_path / "out" / "scene.web_scene.json")

    schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
    Draft202012Validator(schema).validate(payload)
    assert payload["schema_version"] == "workcell_studio_web_scene/v1"
    assert payload["viewer_summary"]["renderable_count"] == 0
    assert payload["viewer_summary"]["scene_bounds"] == {"min": [0.0, 0.0, 0.0], "max": [0.0, 0.0, 0.0], "source_count": 0, "sources": []}
    missing = [warning for warning in payload["warnings"] if warning["code"] == "optional_file_missing"]
    assert {warning["source"] for warning in missing} == {
        "scene_manifest.yaml",
        "cell_definition.yaml",
        "environment.yaml",
        "layout/workcell_studio_layout.yaml",
        "generated/scene_visual_mesh_index.json",
    }
    assert all(payload["inputs"][name]["present"] is False for name in payload["inputs"])


def test_ur5_2f_web_scene_stages_required_product_meshes_without_required_fallbacks():
    artifact, payload = _export_persisted_ur5_2f_web_scene()
    assert artifact == REPO_ROOT / "build" / "workcell_studio_web_scene" / "ur5_2f_test.web_scene.json"
    required = {
        "ur5": [item for item in payload["robots"] if "ur_description/meshes/ur5/visual" in str(item.get("original_mesh_uri") or "")],
        "robotiq": [item for item in payload["tools"] if "robotiq_85_description/meshes/visual" in str(item.get("original_mesh_uri") or "")],
        "table": [item for item in payload["assets"] if "workbench_description/meshes/visual/table.stl" in str(item.get("original_mesh_uri") or "")],
        "camera": [item for item in payload["sensors"] if "realsense2_description/meshes/d435.dae" in str(item.get("original_mesh_uri") or "")],
    }
    assert len(required["ur5"]) >= 7
    required_major_ur5_links = {
        "base_link_inertia",
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    }
    ur5_by_required_link = {item.get("link"): item for item in required["ur5"] if item.get("link") in required_major_ur5_links}
    assert set(ur5_by_required_link) == required_major_ur5_links
    for link, item in ur5_by_required_link.items():
        assert item.get("active_visual_source") == "mesh_preview", link
        assert item.get("category") == "robot_static_mesh_visual", link
        assert item.get("role") == "robot", link
        assert "package://ur_description/meshes/ur5/visual/" in str(item.get("original_mesh_uri") or ""), link
        assert item.get("mesh_staging_status") == "staged", link
        assert item.get("primitive_geometry_type") in (None, "mesh"), link
        assert "${mesh}" not in json.dumps(item, sort_keys=True), link
    assert required["robotiq"]
    assert required["table"]
    assert required["camera"]

    renderable_items = [
        item
        for bucket in ("robots", "tools", "assets", "sensors", "zones")
        for item in payload[bucket]
        if item.get("render_expected", True) is not False
    ]
    for item in renderable_items:
        for field in ("mesh_uri", "mesh_path", "source_path", "package_uri", "resolved_source_path"):
            value = item.get(field)
            assert value != "${mesh}", (item.get("id"), field)
            assert not (isinstance(value, str) and "${" in value and "}" in value), (item.get("id"), field, value)
    assert any(
        warning.get("code") == "unresolved_placeholder_visual_suppressed"
        and warning.get("source") == "generated/scene_visual_mesh_index.json"
        for warning in payload["warnings"]
    )

    summary = payload["viewer_summary"]
    assert summary["renderable_count"] >= 20
    assert summary["mesh_backed_count"] >= len(required["ur5"]) + len(required["robotiq"]) + len(required["table"]) + len(required["camera"])
    assert summary["missing_or_failed_mesh_count"] == 0
    assert summary["scene_bounds"]["source_count"] == summary["renderable_count"]
    for axis in range(3):
        assert summary["scene_bounds"]["min"][axis] <= summary["scene_bounds"]["max"][axis]
    for category in ("robot", "tool", "table", "camera"):
        assert summary["required_item_status"][category]["present"] is True
        assert summary["required_item_status"][category]["status"] in {"mesh_backed", "missing_or_failed_mesh"}
        assert summary["required_item_status"][category]["item_ids"]
    assert any(item["id"] == "safety_zone_keepout" for item in payload["zones"])
    assert "safety_zone_keepout" not in summary["required_item_status"]["robot"]["item_ids"]

    required_identity_expectations = {
        "ur5": ("robots", "robot", "ur5", ("category", "role", "id", "source_kind", "source_layer", "active_visual_source", "link", "original_mesh_uri", "mesh_uri")),
        "robotiq": ("tools", "tool", "robotiq", ("id", "source_kind", "link", "original_mesh_uri", "mesh_uri")),
        "table": ("assets", "table", "workbench", ("id", "source_kind", "link", "original_mesh_uri", "mesh_uri")),
        "camera": ("sensors", "camera", "realsense", ("id", "source_kind", "link", "original_mesh_uri", "mesh_uri")),
    }
    for family, (_bucket, required_category, identity_token, identity_fields) in required_identity_expectations.items():
        assert summary["required_item_status"][required_category]["status"] == "mesh_backed", family
        for item in required[family]:
            identity_text = " ".join(str(item.get(field, "")).lower() for field in identity_fields)
            assert identity_token in identity_text, item["id"]
            assert item["id"] in summary["required_item_status"][required_category]["item_ids"]
            for field in identity_fields:
                assert field in item, f"{item['id']} should export identity field {field}"

    for group in required.values():
        for item in group:
            assert item["mesh_staging_status"] == "staged"
            assert summary["required_item_status"][
                "robot" if item in required["ur5"] else
                "tool" if item in required["robotiq"] else
                "table" if item in required["table"] else
                "camera"
            ]["status"] != "primitive_fallback"
            assert item.get("primitive_geometry_type") in (None, "mesh")
            if "active_visual_source" in item:
                assert item["active_visual_source"] == "mesh_preview"
            assert item["mesh_uri"].startswith("build/workcell_studio_web_scene/assets/ur5_2f_test/")
            assert not item["mesh_uri"].startswith(("package://", "file://", "/"))
            assert "://" not in item["mesh_uri"]
            assert ".." not in Path(item["mesh_uri"]).parts
            assert item.get("mesh_resolve_warning") in (None, "")

    required_ids = {item["id"] for group in required.values() for item in group}
    fallback_warnings = [
        warning for warning in payload["warnings"]
        if warning.get("code") in {"mesh_primitive_fallback", "mesh_stage_failed"}
    ]
    assert not [warning for warning in fallback_warnings if warning.get("source") in required_ids]


def test_web_scene_export_status_summary_counts_mesh_fallback_and_missing_items(tmp_path):
    scene = _tiny_scene_fixture(tmp_path)
    (scene / "meshes").mkdir()
    (scene / "meshes" / "ur5.dae").write_text("<COLLADA></COLLADA>\n", encoding="utf-8")
    # Leave robotiq_2f.dae and layout_bin mesh absent so the summary must report
    # missing/failed meshes while authored table/camera remain primitive fallbacks.

    payload = _export(scene, tmp_path / "out" / "summary.web_scene.json")
    summary = payload["viewer_summary"]
    renderable = [
        item
        for bucket in ("robots", "tools", "assets", "sensors", "zones")
        for item in payload[bucket]
    ]

    assert summary["renderable_count"] == len(renderable)
    assert summary["mesh_backed_count"] == 1
    assert summary["fallback_count"] == 2
    assert summary["missing_or_failed_mesh_count"] == 2
    assert (
        summary["mesh_backed_count"]
        + summary["fallback_count"]
        + summary["missing_or_failed_mesh_count"]
    ) == summary["renderable_count"]
    robot = next(item for item in payload["robots"] if item["id"] == "ur5_visual")
    tool = next(item for item in payload["tools"] if item["id"] == "robotiq_2f_visual")
    assert robot["mesh_staging_status"] == "staged"
    assert tool["mesh_staging_status"] == "resolve_failed"
    assert summary["required_item_status"]["robot"]["present"] is True
    assert summary["required_item_status"]["tool"]["status"] == "missing_or_failed_mesh"
    assert summary["required_item_status"]["table"]["status"] == "primitive_fallback"
    assert summary["required_item_status"]["camera"]["status"] == "primitive_fallback"


def _items_with_original_mesh(payload: dict, bucket: str, token: str) -> list[dict]:
    return [
        item for item in payload[bucket]
        if token in str(item.get("original_mesh_uri") or item.get("mesh_uri") or "")
    ]


def _final_pose(item: dict) -> dict:
    pose = item.get("final_transform") or item.get("world_from_visual") or item.get("pose") or item.get("world_pose")
    assert isinstance(pose, dict), f"{item.get('id')} missing final/world pose"
    return pose


def _finite_vector(values: object, *, length: int, label: str) -> list[float]:
    assert isinstance(values, list), f"{label} must be a list"
    assert len(values) == length, f"{label} must have {length} values"
    out = [float(v) for v in values]
    assert all(math.isfinite(v) for v in out), f"{label} must contain finite values"
    return out


def _xyz(item: dict) -> list[float]:
    return _finite_vector(_final_pose(item).get("xyz"), length=3, label=f"{item.get('id')}.xyz")


def _assert_pose_is_finite(item: dict) -> None:
    pose = _final_pose(item)
    _finite_vector(pose.get("xyz"), length=3, label=f"{item.get('id')}.xyz")
    if "rpy" in pose:
        _finite_vector(pose.get("rpy"), length=3, label=f"{item.get('id')}.rpy")
    if "quaternion" in pose:
        _finite_vector(pose.get("quaternion"), length=4, label=f"{item.get('id')}.quaternion")
    quat = item.get("baked_world_visual_quaternion")
    if isinstance(quat, dict):
        assert all(math.isfinite(float(quat[k])) for k in ("w", "x", "y", "z"))


def _distance(a: dict, b: dict) -> float:
    ax, ay, az = _xyz(a)
    bx, by, bz = _xyz(b)
    return math.dist((ax, ay, az), (bx, by, bz))


def _assert_browser_safe_staged_mesh(item: dict) -> None:
    uri = item.get("mesh_uri")
    assert isinstance(uri, str) and uri
    assert uri.startswith("build/workcell_studio_web_scene/assets/ur5_2f_test/")
    assert not uri.startswith(("/", "file://", "package://"))
    assert "://" not in uri
    assert ".." not in Path(uri).parts
    assert item.get("mesh_staging_status") == "staged"
    assert item.get("mesh_resolve_warning") in (None, "")


def test_ur5_2f_web_scene_export_transform_parity_for_product_meshes(tmp_path):
    payload = _export(REPO_ROOT / "scenes" / "ur5_2f_test", tmp_path / "out" / "ur5_2f_test.parity.web_scene.json")

    ur5_by_link = {
        item.get("link"): item
        for item in _items_with_original_mesh(payload, "robots", "ur_description/meshes/ur5/visual")
    }
    required_ur5_links = (
        "base_link_inertia",
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    )
    for link in required_ur5_links:
        assert link in ur5_by_link
        _assert_pose_is_finite(ur5_by_link[link])
        _assert_browser_safe_staged_mesh(ur5_by_link[link])
        assert ur5_by_link[link].get("primitive_geometry_type") in (None, "mesh")

    adjacent_pairs = (
        ("base_link_inertia", "shoulder_link"),
        ("shoulder_link", "upper_arm_link"),
        ("upper_arm_link", "forearm_link"),
        ("forearm_link", "wrist_1_link"),
        ("wrist_1_link", "wrist_2_link"),
        ("wrist_2_link", "wrist_3_link"),
    )
    for parent, child in adjacent_pairs:
        dist = _distance(ur5_by_link[parent], ur5_by_link[child])
        assert 0.03 <= dist <= 0.75, f"{parent}->{child} implausible distance {dist}"

    assert any(any(abs(v) > 0.05 for v in _xyz(item)) for item in ur5_by_link.values())

    gripper_items = _items_with_original_mesh(payload, "tools", "robotiq_85_description/meshes/visual")
    assert gripper_items
    wrist = ur5_by_link["wrist_3_link"]
    for item in gripper_items:
        _assert_pose_is_finite(item)
        _assert_browser_safe_staged_mesh(item)
        assert item.get("primitive_geometry_type") in (None, "mesh")
        assert _distance(wrist, item) <= 0.35

    tables = _items_with_original_mesh(payload, "assets", "workbench_description/meshes/visual/table.stl")
    assert tables
    for table in tables:
        _assert_pose_is_finite(table)
        _assert_browser_safe_staged_mesh(table)
        scale = _finite_vector(table.get("scale") or table.get("mesh_scale"), length=3, label=f"{table.get('id')}.scale")
        assert scale == [0.001, 0.001, 0.001]
        assert all(0.0001 <= value <= 10.0 for value in scale)
        assert table.get("primitive_geometry_type") in (None, "mesh")

    cameras = _items_with_original_mesh(payload, "sensors", "realsense2_description/meshes/d435.dae")
    assert cameras
    for camera in cameras:
        _assert_pose_is_finite(camera)
        _assert_browser_safe_staged_mesh(camera)
        x, y, z = _xyz(camera)
        assert -1.5 <= x <= 1.5
        assert -1.5 <= y <= 1.5
        assert 0.2 <= z <= 2.0
        assert camera.get("primitive_geometry_type") in (None, "mesh")

    required_ids = {item["id"] for item in [*ur5_by_link.values(), *gripper_items, *tables, *cameras]}
    fallback_warnings = [
        warning for warning in payload["warnings"]
        if warning.get("code") in {"mesh_primitive_fallback", "mesh_stage_failed"}
    ]
    assert not [warning for warning in fallback_warnings if warning.get("source") in required_ids]
