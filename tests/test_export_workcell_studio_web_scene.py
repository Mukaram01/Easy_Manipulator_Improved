import json
import subprocess
import sys
from pathlib import Path

import yaml


SCRIPT = Path("scripts/export_workcell_studio_web_scene.py")
ROOT = Path(__file__).resolve().parents[1]
EXTRACT_INDEX_SCRIPT = ROOT / "scripts" / "extract_scene_urdf_visual_mesh_index.py"


def test_export_web_scene_contract_and_determinism(tmp_path):
    scene = tmp_path / "scene"
    (scene / "layout").mkdir(parents=True)
    (scene / "generated").mkdir()
    (scene / "scene_manifest.yaml").write_text(
        yaml.safe_dump({"scene": {"name": "demo"}, "robot": {"model": "ur5"}}, sort_keys=False),
        encoding="utf-8",
    )
    (scene / "cell_definition.yaml").write_text(
        yaml.safe_dump({"cell": {"id": "demo"}, "robot": {"model": "ur5", "planning_group": "manipulator"}}, sort_keys=False),
        encoding="utf-8",
    )
    (scene / "environment.yaml").write_text(
        yaml.safe_dump(
            {"environment": {"assets": [{"id": "bin", "type": "target_bin", "pose_xyz": [1, 2, 3]}], "sensors": [{"id": "realsense_camera", "role": "camera", "category": "camera", "dimensions": [0.08, 0.08, 0.06], "pose_xyz": [0.4, 0.0, 0.8], "mesh_uri": "meshes/realsense.stl"}]}},
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (scene / "layout/workcell_studio_layout.yaml").write_text(
        yaml.safe_dump(
            {"schema_version": "workcell_studio_layout/v1", "items": [{"id": "table", "role": "support_surface", "dimensions": [1.2, 0.8, 0.08], "mesh_path": str(scene / "meshes/table.stl")}]},
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (scene / "generated/scene_visual_mesh_index.json").write_text(
        json.dumps(
            {
                "visual_items": [
                    {
                        "id": "robot_mesh",
                        "category": "robot",
                        "role": "robot",
                        "mesh_path": str(scene / "meshes/robot.dae"),
                        "pose": {"xyz": [9, 9, 9], "rpy": [0, 0, 0]},
                        "world_pose": {"xyz": [8, 8, 8], "rpy": [0, 0, 0]},
                        "link_world_pose": {"xyz": [1, 2, 3], "rpy": [0.1, 0.2, 0.3]},
                        "visual_origin": {"xyz": [0.1, 0.2, 0.3], "rpy": [0, 0, 1.57]},
                        "baked_world_visual_pose": {"xyz": [1.1, 2.2, 3.3], "rpy": [0.1, 0.2, 1.87]},
                        "baked_world_visual_matrix": [[1, 0, 0, 1.1], [0, 1, 0, 2.2], [0, 0, 1, 3.3], [0, 0, 0, 1]],
                        "baked_world_visual_quaternion": [0, 0, 0, 1],
                        "mesh_scale": [1, 1, 1],
                    },
                    {
                        "id": "robotiq_finger",
                        "category": "gripper",
                        "role": "tool",
                        "link": "robotiq_85_left_finger_link",
                        "mesh_path": str(scene / "meshes/robotiq_finger.dae"),
                        "expected_visual_pose": {"xyz": [0.4, 0.5, 0.6], "rpy": [0.7, 0.8, 0.9]},
                        "link_world_pose": {"xyz": [0.1, 0.2, 0.3], "rpy": [0.0, 0.0, 0.0]},
                    },
                    {"id": "camera_mesh", "category": "camera", "role": "camera", "package_uri": "package://camera/mesh.dae"},
                ]
            }
        ),
        encoding="utf-8",
    )

    out1 = tmp_path / "build/scene.web_scene.json"
    out2 = tmp_path / "build/scene_2.web_scene.json"
    cmd1 = [sys.executable, str(SCRIPT), "--scene", str(scene), "--output", str(out1)]
    cmd2 = [sys.executable, str(SCRIPT), "--scene", str(scene), "--output", str(out2)]
    subprocess.run(cmd1, check=True)
    subprocess.run(cmd2, check=True)

    assert out1.read_text(encoding="utf-8") == out2.read_text(encoding="utf-8")
    payload = json.loads(out1.read_text(encoding="utf-8"))
    assert payload["schema_version"] == "workcell_studio_web_scene/v1"
    assert payload["metadata"]["visual_bounds_contract"]["status"] == "failed"
    assert payload["metadata"]["visual_bounds_contract"]["camera_framing_blockers"]
    assert payload["robots"][-1]["id"] == "robot_mesh"
    assert payload["robots"][-1]["locked"] is True
    assert payload["robots"][-1]["editable"] is False
    table = next(item for item in payload["assets"] if item["id"] == "table")
    assert table["locked"] is False
    assert table["editable"] is True
    assert table["mesh_path"] == "meshes/table.stl"
    assert table["expected_dimensions_m"] == [1.2, 0.8, 0.08]
    assert table["mesh_contract_category"] == "table"
    assert table["support_surface_kind"] == "support_surface"
    assert table["top_surface_z_m"] == 0.04
    assert table["support_surface_height_m"] == 0.04
    assert table["expected_support_footprint_m"] == [1.2, 0.8]
    camera = next(item for item in payload["sensors"] if item["id"] == "realsense_camera")
    assert camera["expected_dimensions_m"] == [0.08, 0.08, 0.06]
    assert camera["mesh_contract_category"] == "camera"
    mesh_backed_items = [
        item
        for section in ("robots", "tools", "assets", "sensors", "zones")
        for item in payload[section]
        if item.get("mesh_uri") or item.get("mesh_path") or item.get("package_uri") or item.get("mesh_url")
    ]
    assert mesh_backed_items
    assert all(item.get("mesh_contract_category") for item in mesh_backed_items)
    robot_or_generated_link_items = [
        item
        for section in ("robots", "tools")
        for item in payload[section]
        if item.get("source_kind") == "generated_preview" or item.get("link") or item.get("category") == "robot"
    ]
    assert robot_or_generated_link_items
    assert all(item.get("allow_mesh_unit_autoscale") is not True for item in robot_or_generated_link_items)
    assert payload["robots"][-1]["mesh_path"] == "meshes/robot.dae"
    assert payload["robots"][-1]["final_transform"] == {"xyz": [1.1, 2.2, 3.3], "rpy": [0.1, 0.2, 1.87]}
    assert payload["robots"][-1]["world_from_visual"] == payload["robots"][-1]["final_transform"]
    assert payload["robots"][-1]["transform_source"] == "baked_world_visual_pose"
    assert payload["robots"][-1]["workcell_web_render_pose_mode"] == "baked_visible_world_pose"
    assert payload["robots"][-1]["visual_origin_application"] == "baked_into_web_preview_pose"
    assert payload["robots"][-1]["link_world_pose"] == {"xyz": [1, 2, 3], "rpy": [0.1, 0.2, 0.3]}
    assert payload["robots"][-1]["original_link_world_pose"] == payload["robots"][-1]["link_world_pose"]
    assert payload["robots"][-1]["visual_origin"] == {"xyz": [0.1, 0.2, 0.3], "rpy": [0, 0, 1.57]}
    assert payload["robots"][-1]["mesh_scale"] == [1, 1, 1]
    assert payload["robots"][-1]["baked_world_visual_matrix"] == [[1, 0, 0, 1.1], [0, 1, 0, 2.2], [0, 0, 1, 3.3], [0, 0, 0, 1]]
    assert payload["robots"][-1]["baked_world_visual_quaternion"] == [0, 0, 0, 1]
    tool_mesh = next(item for item in payload["tools"] if item["id"] == "robotiq_finger")
    assert tool_mesh["final_transform"] == {"xyz": [0.4, 0.5, 0.6], "rpy": [0.7, 0.8, 0.9]}
    assert tool_mesh["workcell_web_render_pose_mode"] == "baked_visible_world_pose"
    assert tool_mesh["visual_origin_application"] == "baked_into_web_preview_pose"
    assert tool_mesh["original_link_world_pose"] == {"xyz": [0.1, 0.2, 0.3], "rpy": [0.0, 0.0, 0.0]}
    assert tool_mesh["link_world_pose"] == {"xyz": [0.1, 0.2, 0.3], "rpy": [0.0, 0.0, 0.0]}
    assert tool_mesh["provenance"]["workcell_web_render_pose_mode"] == "generated/scene_visual_mesh_index.json"
    assert tool_mesh["provenance"]["visual_origin_application"] == "generated/scene_visual_mesh_index.json"
    assert tool_mesh["provenance"]["original_link_world_pose"] == "generated/scene_visual_mesh_index.json"
    camera_mesh = next(item for item in payload["sensors"] if item["id"] == "camera_mesh")
    assert camera_mesh["package_uri"] == "package://camera/mesh.dae"
    assert payload["robots"][-1]["provenance"]["mesh_path"] == "generated/scene_visual_mesh_index.json"
    assert payload["robots"][-1]["provenance"]["final_transform"] == "generated/scene_visual_mesh_index.json"


def test_export_web_scene_warns_for_missing_optional_inputs(tmp_path):
    scene = tmp_path / "scene"
    scene.mkdir()
    out = tmp_path / "build/scene.web_scene.json"

    subprocess.run([sys.executable, str(SCRIPT), "--scene", str(scene), "--output", str(out)], check=True)

    payload = json.loads(out.read_text(encoding="utf-8"))
    missing = [w for w in payload["warnings"] if w["code"] == "optional_file_missing"]
    assert {w["source"] for w in missing} == {
        "scene_manifest.yaml",
        "cell_definition.yaml",
        "environment.yaml",
        "layout/workcell_studio_layout.yaml",
        "generated/scene_visual_mesh_index.json",
    }


def test_export_visual_bounds_contract_ignores_helper_zones_outside_workspace(tmp_path):
    scene = tmp_path / "scene"
    (scene / "layout").mkdir(parents=True)
    (scene / "generated").mkdir()
    (scene / "scene_manifest.yaml").write_text(
        yaml.safe_dump(
            {
                "scene": {
                    "name": "helper_zone_bounds_test",
                    "expected_workspace_bounds_m": {"min": [-1.0, -1.0, 0.0], "max": [1.0, 1.0, 1.8]},
                }
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (scene / "cell_definition.yaml").write_text("{}", encoding="utf-8")
    (scene / "environment.yaml").write_text(
        yaml.safe_dump(
            {
                "environment": {
                    "zones": [
                        {
                            "id": "far_pick_zone",
                            "role": "pick_zone",
                            "category": "safety_zone",
                            "dimensions": [0.2, 0.2, 0.01],
                            "pose_xyz": [50.0, 50.0, 0.1],
                        }
                    ]
                }
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (scene / "layout/workcell_studio_layout.yaml").write_text(
        yaml.safe_dump(
            {
                "items": [
                    {
                        "id": "workbench",
                        "role": "support_surface",
                        "dimensions": [1.0, 0.6, 0.08],
                        "pose_xyz": [0.0, 0.0, 0.04],
                    },
                    {
                        "id": "camera_fov_overlay",
                        "role": "camera_fov",
                        "category": "overlay",
                        "dimensions": [0.3, 0.3, 0.3],
                        "pose_xyz": [-50.0, -50.0, 0.2],
                    },
                ]
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (scene / "generated/scene_visual_mesh_index.json").write_text(json.dumps({"visual_items": []}), encoding="utf-8")

    out = tmp_path / "build/scene.web_scene.json"
    subprocess.run([sys.executable, str(SCRIPT), "--scene", str(scene), "--output", str(out)], check=True)

    payload = json.loads(out.read_text(encoding="utf-8"))

    exported_ids = {
        item["id"]
        for section in ("robots", "tools", "assets", "sensors", "zones")
        for item in payload[section]
    }
    assert {"camera_fov_overlay", "far_pick_zone"}.issubset(exported_ids)
    assert any(item["id"] == "far_pick_zone" for item in payload["zones"])
    assert payload["metadata"]["visual_bounds_contract"]["status"] == "passed"
    assert payload["metadata"]["visual_bounds_contract"]["camera_framing_blockers"] == []
    assert all("far_pick_zone" not in source for source in payload["metadata"]["visual_bounds_contract"]["scene_bounds_m"]["sources"])
    assert all("camera_fov_overlay" not in source for source in payload["metadata"]["visual_bounds_contract"]["scene_bounds_m"]["sources"])


def test_export_fails_oversized_physical_camera_bounds_but_excludes_camera_helpers(tmp_path):
    scene = tmp_path / "scene"
    (scene / "layout").mkdir(parents=True)
    (scene / "generated").mkdir()
    (scene / "scene_manifest.yaml").write_text(
        yaml.safe_dump(
            {
                "scene": {
                    "name": "camera_bounds_regression",
                    "expected_workspace_bounds_m": {"min": [-1.0, -1.0, 0.0], "max": [1.0, 1.0, 1.8]},
                }
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (scene / "cell_definition.yaml").write_text("{}", encoding="utf-8")
    (scene / "environment.yaml").write_text(
        yaml.safe_dump(
            {
                "environment": {
                    "sensors": [
                        {
                            "id": "realsense_d435",
                            "role": "camera",
                            "category": "camera",
                            "dimensions": [0.08, 0.08, 0.06],
                        }
                    ]
                }
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (scene / "layout/workcell_studio_layout.yaml").write_text(yaml.safe_dump({"items": []}), encoding="utf-8")
    (scene / "generated/scene_visual_mesh_index.json").write_text(
        json.dumps(
            {
                "visual_items": [
                    {
                        "id": "generated_urdf::camera_link::visual_0",
                        "category": "camera",
                        "role": "camera",
                        "link": "camera_link",
                        "mesh_uri": "package://realsense2_description/meshes/d435.dae",
                        "pose": {"xyz": [0.0, 0.0, 0.6], "rpy": [0.0, 0.0, 0.0]},
                        "mesh_bounds": {"min": [-50.0, -50.0, -50.0], "max": [50.0, 50.0, 50.0]},
                    },
                    {
                        "id": "camera_fov_overlay",
                        "category": "camera",
                        "role": "camera_fov",
                        "source_layer": "overlay",
                        "active_visual_source": "camera_fov",
                        "pose": {"xyz": [0.0, 0.0, 0.6], "rpy": [0.0, 0.0, 0.0]},
                        "dimensions": [100.0, 100.0, 100.0],
                    },
                ]
            }
        ),
        encoding="utf-8",
    )

    out = tmp_path / "build/scene.web_scene.json"
    subprocess.run([sys.executable, str(SCRIPT), "--scene", str(scene), "--output", str(out)], check=True)

    payload = json.loads(out.read_text(encoding="utf-8"))
    contract = payload["metadata"]["visual_bounds_contract"]
    assert contract["status"] == "failed"
    assert any(
        blocker["id"] == "generated_urdf::camera_link::visual_0"
        and blocker["reason"] == "oversized_item_can_break_camera_framing"
        for blocker in contract["camera_framing_blockers"]
    )
    assert all("camera_fov_overlay" not in source for source in contract["scene_bounds_m"]["sources"])
    assert any(item["id"] == "camera_fov_overlay" for item in payload["zones"])


def test_export_carries_authored_fixture_dimensions_to_generated_table_camera_meshes(tmp_path):
    scene = tmp_path / "scene"
    (scene / "layout").mkdir(parents=True)
    (scene / "generated").mkdir()
    (scene / "scene_manifest.yaml").write_text(yaml.safe_dump({"scene": {"name": "generated_fixture_dims"}}), encoding="utf-8")
    (scene / "cell_definition.yaml").write_text("{}", encoding="utf-8")
    (scene / "layout/workcell_studio_layout.yaml").write_text(yaml.safe_dump({"items": []}), encoding="utf-8")
    (scene / "environment.yaml").write_text(
        yaml.safe_dump(
            {
                "environment": {
                    "support_surfaces": [
                        {
                            "id": "support_surface_table",
                            "type": "table",
                            "role": "support_surface",
                            "category": "work_surface",
                            "dimensions": [1.2, 0.8, 0.08],
                        }
                    ],
                    "assets": [
                        {
                            "id": "realsense_overhead",
                            "type": "realsense",
                            "role": "camera",
                            "category": "camera",
                            "dimensions": [0.08, 0.08, 0.06],
                        }
                    ],
                }
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (scene / "generated/scene_visual_mesh_index.json").write_text(
        json.dumps(
            {
                "visual_items": [
                    {
                        "id": "urdf_visual_table",
                        "link": "table_",
                        "object_name": "table_",
                        "mesh_uri": "package://workbench_description/meshes/visual/table.stl",
                        "mesh_scale": [0.001, 0.001, 0.001],
                        "pose": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]},
                    },
                    {
                        "id": "urdf_visual_camera",
                        "link": "camera_link",
                        "object_name": "camera_link",
                        "mesh_uri": "package://realsense2_description/meshes/d435.dae",
                        "pose": {"xyz": [0.35, 0, 0.85], "rpy": [0, 1.5708, 0]},
                    },
                ]
            }
        ),
        encoding="utf-8",
    )

    out = tmp_path / "build/scene.web_scene.json"
    subprocess.run([sys.executable, str(SCRIPT), "--scene", str(scene), "--output", str(out)], check=True)

    payload = json.loads(out.read_text(encoding="utf-8"))
    table = next(item for item in payload["assets"] if item["id"] == "urdf_visual_table")
    camera = next(item for item in payload["sensors"] if item["id"] == "urdf_visual_camera")
    assert table["mesh_contract_category"] == "table"
    assert table["support_surface_kind"] == "workbench_body"
    assert 1.1 < table["expected_dimensions_m"][0] < 1.3
    assert 0.7 < table["expected_dimensions_m"][1] < 0.9
    assert 0.8 < table["expected_dimensions_m"][2] < 1.0
    assert 1.1 < table["expected_support_footprint_m"][0] < 1.3
    assert 0.7 < table["expected_support_footprint_m"][1] < 0.9
    assert isinstance(table["top_surface_z_m"], float)
    assert isinstance(table["support_surface_height_m"], float)
    assert table["original_mesh_uri"] == "package://workbench_description/meshes/visual/table.stl"
    assert table["mesh_staging_status"] == "staged"
    assert table["mesh_staged_path"] == table["mesh_uri"]
    assert camera["mesh_contract_category"] == "camera"
    assert camera["expected_dimensions_m"] == [0.08, 0.08, 0.06]


def test_export_web_scene_stages_safe_mesh_assets(tmp_path):
    scene = tmp_path / "scene"
    mesh_dir = scene / "meshes"
    (scene / "layout").mkdir(parents=True)
    (scene / "generated").mkdir()
    mesh_dir.mkdir()
    (mesh_dir / "table.stl").write_text("solid table\nendsolid table\n", encoding="utf-8")
    (scene / "scene_manifest.yaml").write_text(yaml.safe_dump({"scene": {"name": "staging_test"}}), encoding="utf-8")
    (scene / "cell_definition.yaml").write_text("{}", encoding="utf-8")
    (scene / "environment.yaml").write_text("{}", encoding="utf-8")
    (scene / "layout/workcell_studio_layout.yaml").write_text(
        yaml.safe_dump({"items": [{"id": "table", "mesh_uri": "meshes/table.stl"}]}),
        encoding="utf-8",
    )
    (scene / "generated/scene_visual_mesh_index.json").write_text(json.dumps({"visual_items": []}), encoding="utf-8")

    out = tmp_path / "build/scene.web_scene.json"
    subprocess.run([sys.executable, str(SCRIPT), "--scene", str(scene), "--output", str(out)], check=True)

    payload = json.loads(out.read_text(encoding="utf-8"))
    table = next(item for item in payload["assets"] if item["id"] == "table")
    assert table["original_mesh_uri"] == "meshes/table.stl"
    assert table["mesh_staging_status"] == "staged"
    assert table["mesh_resolve_warning"] is None
    assert table["mesh_uri"] == "build/workcell_studio_web_scene/assets/staging_test/external_" + table["mesh_uri"].split("/external_", 1)[1]
    assert Path(table["mesh_staged_path"]).is_file()


def test_export_web_scene_rejects_unsafe_or_unsupported_mesh_assets(tmp_path):
    scene = tmp_path / "scene"
    (scene / "layout").mkdir(parents=True)
    (scene / "generated").mkdir()
    (scene / "scene_manifest.yaml").write_text(yaml.safe_dump({"scene": {"name": "unsafe_test"}}), encoding="utf-8")
    (scene / "cell_definition.yaml").write_text("{}", encoding="utf-8")
    (scene / "environment.yaml").write_text("{}", encoding="utf-8")
    (scene / "layout/workcell_studio_layout.yaml").write_text(
        yaml.safe_dump(
            {
                "items": [
                    {"id": "traversal", "mesh_uri": "../secret.stl"},
                    {"id": "remote", "mesh_uri": "https://example.invalid/mesh.stl"},
                    {"id": "texture", "mesh_uri": "meshes/texture.png"},
                ]
            }
        ),
        encoding="utf-8",
    )
    (scene / "generated/scene_visual_mesh_index.json").write_text(json.dumps({"visual_items": []}), encoding="utf-8")

    out = tmp_path / "build/scene.web_scene.json"
    subprocess.run([sys.executable, str(SCRIPT), "--scene", str(scene), "--output", str(out)], check=True)

    payload = json.loads(out.read_text(encoding="utf-8"))
    by_id = {item["id"]: item for item in payload["assets"]}
    assert by_id["traversal"]["mesh_staging_status"] == "unsafe_path"
    assert "Unsafe relative mesh path" in by_id["traversal"]["mesh_resolve_warning"]
    assert by_id["remote"]["mesh_staging_status"] == "unsupported_scheme"
    assert "Unsupported mesh URI scheme" in by_id["remote"]["mesh_resolve_warning"]
    assert by_id["texture"]["mesh_staging_status"] == "unsupported_format"
    assert "Unsupported mesh format" in by_id["texture"]["mesh_resolve_warning"]


def test_web_viewer_prefers_canonical_final_transform_without_visual_origin_recomposition():
    viewer = Path("workcell_studio_web/viewer/viewer.js").read_text(encoding="utf-8")
    pose_block = viewer.split("function poseOf(item)", 1)[1].split("function scaleOf", 1)[0]
    assert "item.final_transform || item.world_from_visual || item.baked_world_visual_pose" in pose_block
    assert "visual_origin" not in pose_block


def test_robotiq_fallback_uses_tool0_frame_not_wrist_visual_pose(tmp_path):
    scene = tmp_path / "scene"
    (scene / "layout").mkdir(parents=True)
    (scene / "generated").mkdir()
    (scene / "scene_manifest.yaml").write_text(
        yaml.safe_dump(
            {
                "scene": {"name": "tool0_frame_regression"},
                "end_effector": {"id": "robotiq_85", "model": "robotiq_85"},
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (scene / "cell_definition.yaml").write_text("{}", encoding="utf-8")
    (scene / "environment.yaml").write_text("{}", encoding="utf-8")
    (scene / "layout/workcell_studio_layout.yaml").write_text(yaml.safe_dump({"items": []}), encoding="utf-8")
    (scene / "generated/scene_visual_mesh_index.json").write_text(
        json.dumps(
            {
                "visual_items": [
                    {
                        "id": "generated_urdf::wrist_3_link::visual_0",
                        "category": "robot",
                        "role": "robot",
                        "link": "wrist_3_link",
                        "mesh_uri": "package://ur_description/meshes/ur5/visual/wrist3.dae",
                        "link_world_pose": {"xyz": [0.4, 0.2, 0.6], "rpy": [0.0, 0.0, 0.0]},
                        "visual_origin": {"xyz": [0.3, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
                        "baked_world_visual_pose": {"xyz": [0.7, 0.2, 0.6], "rpy": [0.0, 0.0, 0.0]},
                    },
                    {
                        "id": "generated_urdf::tool0::frame",
                        "category": "tool",
                        "role": "frame_anchor",
                        "link": "tool0",
                        "render_expected": False,
                        "geometry_type": "frame",
                        "link_world_pose": {"xyz": [0.4, 0.2, 0.6], "rpy": [0.0, 0.0, 0.0]},
                    },
                ]
            }
        ),
        encoding="utf-8",
    )

    out = tmp_path / "build/scene.web_scene.json"
    subprocess.run([sys.executable, str(SCRIPT), "--scene", str(scene), "--output", str(out)], check=True)

    payload = json.loads(out.read_text(encoding="utf-8"))
    gripper_base = next(item for item in payload["tools"] if item["link"] == "gripper_base_link")
    assert gripper_base["parent_link"] == "tool0"
    assert gripper_base["joint_parent_link"] == "tool0"
    assert gripper_base["link_world_pose"]["xyz"] == [0.4, 0.2, 0.6]
    assert gripper_base["final_transform"]["xyz"] == [0.4, 0.2, 0.6]
    assert gripper_base["final_transform"]["xyz"] != [0.7, 0.2, 0.6]


def test_gripper_parent_fallback_ignores_wrist_visual_origin_when_tool0_missing(tmp_path):
    scene = tmp_path / "scene"
    (scene / "layout").mkdir(parents=True)
    (scene / "generated").mkdir()
    (scene / "scene_manifest.yaml").write_text(
        yaml.safe_dump(
            {
                "scene": {"name": "missing_tool0_frame_regression"},
                "end_effector": {"id": "robotiq_85", "model": "robotiq_85"},
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (scene / "cell_definition.yaml").write_text("{}", encoding="utf-8")
    (scene / "environment.yaml").write_text("{}", encoding="utf-8")
    (scene / "layout/workcell_studio_layout.yaml").write_text(yaml.safe_dump({"items": []}), encoding="utf-8")
    (scene / "generated/scene_visual_mesh_index.json").write_text(
        json.dumps(
            {
                "visual_items": [
                    {
                        "id": "generated_urdf::wrist_3_link::visual_0",
                        "category": "robot",
                        "role": "robot",
                        "link": "wrist_3_link",
                        "mesh_uri": "package://ur_description/meshes/ur5/visual/wrist3.dae",
                        "link_world_pose": {"xyz": [0.4, 0.2, 0.6], "rpy": [0.0, 0.0, 0.0]},
                        "visual_origin": {"xyz": [0.3, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
                        "baked_world_visual_pose": {"xyz": [0.7, 0.2, 0.6], "rpy": [0.0, 0.0, 0.0]},
                    },
                    {
                        "id": "generated_urdf::gripper_base_link::visual_0",
                        "category": "tool",
                        "role": "gripper",
                        "link": "gripper_base_link",
                        "parent_link": "tool0",
                        "joint_parent_link": "tool0",
                        "mesh_uri": "package://robotiq_85_description/meshes/visual/robotiq_85_base_link.dae",
                        "final_transform": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
                        "transform_chain": ["wrist_3_link", "tool0", "gripper_base_link"],
                    },
                ]
            }
        ),
        encoding="utf-8",
    )

    out = tmp_path / "build/scene.web_scene.json"
    subprocess.run([sys.executable, str(SCRIPT), "--scene", str(scene), "--output", str(out)], check=True)

    payload = json.loads(out.read_text(encoding="utf-8"))
    wrist = next(item for item in payload["robots"] if item["link"] == "wrist_3_link")
    gripper_base = next(item for item in payload["tools"] if item["link"] == "gripper_base_link")
    warning_codes = {warning["code"] for warning in payload["warnings"]}

    assert wrist["visual_origin"] == {"xyz": [0.3, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}
    assert gripper_base["final_transform"]["xyz"] == [0.4, 0.2, 0.6]
    assert gripper_base["final_transform"]["xyz"] != wrist["baked_world_visual_pose"]["xyz"]
    assert "tool0_frame_missing_or_collapsed_using_wrist_3_link_fallback" in warning_codes


def test_ur5_2f_web_scene_preserves_tool0_transform_anchor_metadata(tmp_path):
    extract = subprocess.run(
        [
            sys.executable,
            str(EXTRACT_INDEX_SCRIPT),
            "--scene",
            "ur5_2f_test",
            "--allow-xacro-lite-fallback",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert extract.returncode == 0, extract.stdout + extract.stderr

    out = tmp_path / "ur5_2f_test.web_scene.json"
    subprocess.run(
        [
            sys.executable,
            str(ROOT / SCRIPT),
            "--scene",
            str(ROOT / "scenes" / "ur5_2f_test"),
            "--output",
            str(out),
            "--no-stage-assets",
        ],
        cwd=ROOT,
        check=True,
    )

    payload = json.loads(out.read_text(encoding="utf-8"))
    anchors = [
        item
        for item in payload.get("frames", [])
        if item.get("link") == "tool0" and item.get("role") == "transform_anchor"
    ]
    assert anchors, "expected exported Web3D scene to preserve the non-rendered tool0 frame anchor"

    anchor = anchors[0]
    assert anchor["id"] == "urdf_frame_anchor_tool0"
    assert anchor["type"] == "transform_anchor"
    assert anchor["render_expected"] is False
    assert anchor["mesh_available"] is False
    assert anchor["mesh_load_required"] is False
    assert "mesh_uri" not in anchor
    assert anchor["parent_link"]
    assert anchor["joint_parent_link"] == anchor["parent_link"]
    assert isinstance(anchor["joint_origin"], dict)
    assert isinstance(anchor["link_world_pose"], dict)
    assert anchor["frame_world_pose"] == anchor["link_world_pose"]
    assert anchor["world_pose"] == anchor["link_world_pose"]
    assert anchor["pose"] == anchor["link_world_pose"]
    assert anchor["source_layer"]
    assert anchor["active_visual_source"]

    items_by_link = {
        item.get("link") or item.get("link_name") or item.get("frame"): item
        for section in ("robots", "tools", "frames")
        for item in payload.get(section, [])
    }
    wrist = items_by_link["wrist_3_link"]
    gripper = items_by_link["gripper_base_link"]
    assert anchor["parent_link"] == "wrist_3_link"
    assert anchor["meshless_frame"] is True
    assert anchor["assembly_group"] == wrist["assembly_group"] == gripper["assembly_group"]
    assert anchor["robot_instance_id"] == wrist["robot_instance_id"] == gripper["robot_instance_id"]
    assert gripper["parent_link"] == "tool0"
    assert gripper["mesh_uri"]
    assert gripper["parent_to_child_pose"]["xyz"] == [0.0, 0.0, 0.0]
    assert gripper["parent_to_child_pose_source"] == "web_export_parent_world_inverse_times_child_world"
    for link, item in items_by_link.items():
        if str(link).startswith("gripper_"):
            assert item["assembly_group"] == wrist["assembly_group"]
            assert item["robot_instance_id"] == wrist["robot_instance_id"]
            assert item.get("parent_to_child_pose"), f"{link} must carry explicit parent-to-child local pose"
