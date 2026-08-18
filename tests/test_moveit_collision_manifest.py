from __future__ import annotations

import ast
import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest
import yaml


ROOT = Path(__file__).parents[1]
SCRIPT = ROOT / "scripts" / "generate_moveit_collision_manifest.py"
SPEC = importlib.util.spec_from_file_location("generate_moveit_collision_manifest", SCRIPT)
assert SPEC and SPEC.loader
module = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


def _layout(items):
    return {
        "schema_version": "workcell_studio_layout/v1",
        "scene_name": "fixture",
        "items": items,
    }


def _item(item_id, role, geometry="box"):
    item = {
        "id": item_id,
        "type": role,
        "role": role,
        "geometry_type": geometry,
        "pose": {"xyz": [0.1, 0.2, 0.3], "rpy": [0.0, 0.0, 0.5]},
        "dimensions": [0.4, 0.5, 0.6],
    }
    if geometry == "mesh":
        item["mesh"] = {"path": "assets/imported/part.stl", "scale": [0.001, 0.001, 0.001]}
    return item


def test_manifest_separates_physical_collision_truth_from_semantic_markers():
    manifest = module.build_manifest(
        _layout([
            _item("table", "support_surface"),
            _item("part", "asset", "mesh"),
            _item("pick", "pick_zone"),
            _item("place", "place_zone"),
            _item("home", "home_pose", "sphere"),
        ]),
        scene_name="fixture",
        source_path="layout/workcell_studio_layout.yaml",
        source_sha256="abc",
    )

    assert [obj["source_item_id"] for obj in manifest["objects"]] == ["part", "table"]
    assert [obj["id"] for obj in manifest["objects"]] == ["workcell::part", "workcell::table"]
    assert manifest["objects"][0]["collision_geometry"] == {
        "type": "box",
        "dimensions_m": [0.4, 0.5, 0.6],
        "fidelity": "box_proxy",
        "review_required": True,
        "bounds_source": "authored_dimensions",
    }
    assert {row["id"] for row in manifest["excluded_items"]} == {"pick", "place", "home"}
    assert manifest["truth_boundary"]["collision_and_planning_truth"] == "MoveIt PlanningScene"
    assert manifest["truth_boundary"]["viewer_aabb_feedback"] == "advisory_only"
    assert module.validate_manifest(manifest) == []


def test_manifest_rejects_duplicate_ids_and_invalid_physical_dimensions():
    with pytest.raises(module.CollisionManifestError, match="duplicate layout item id"):
        module.build_manifest(
            _layout([_item("same", "asset"), _item("same", "asset")]),
            scene_name="fixture", source_path="layout/workcell_studio_layout.yaml", source_sha256="abc",
        )
    invalid = _item("collapsed", "asset")
    invalid["dimensions"] = [0.4, 0.0, 0.6]
    with pytest.raises(module.CollisionManifestError, match="greater than zero"):
        module.build_manifest(
            _layout([invalid]), scene_name="fixture",
            source_path="layout/workcell_studio_layout.yaml", source_sha256="abc",
        )


def test_collision_can_be_explicitly_disabled():
    disabled = _item("camera", "camera")
    disabled["collision"] = {"enabled": False}
    manifest = module.build_manifest(
        _layout([disabled, _item("table", "support_surface")]),
        scene_name="fixture", source_path="layout/workcell_studio_layout.yaml", source_sha256="abc",
    )
    assert [obj["source_item_id"] for obj in manifest["objects"]] == ["table"]
    assert manifest["excluded_items"][0]["reason"] == "collision explicitly disabled"


def test_north_star_collision_manifest_is_current_and_valid():
    scene = ROOT / "scenes" / "ur5_2f_test"
    output = scene / "config" / "moveit_collision_objects.yaml"
    completed = subprocess.run(
        [sys.executable, str(SCRIPT), "--layout", str(scene / "layout" / "workcell_studio_layout.yaml"),
         "--output", str(output), "--scene-name", "ur5_2f_test", "--check", "--json"],
        cwd=ROOT, text=True, capture_output=True,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr
    data = yaml.safe_load(output.read_text(encoding="utf-8"))
    assert data["summary"]["collision_object_count"] >= 3
    assert data["summary"]["box_proxy_count"] >= 1
    assert data["safety"]["publishes_robot_motion"] is False


def test_north_star_launch_wires_guarded_planning_scene_loader():
    launch_path = ROOT / "scenes" / "ur5_2f_test" / "launch" / "demo.launch.py"
    tree = ast.parse(launch_path.read_text(encoding="utf-8"))
    text = launch_path.read_text(encoding="utf-8")
    assert "COLLISION_MANIFEST_REL_PATH = \"config/moveit_collision_objects.yaml\"" in text
    assert 'executable="workcell_studio_planning_scene_node.py"' in text
    assert '"/apply_planning_scene"' in text
    assert "planning_scene_loader" in {node.id for node in ast.walk(tree) if isinstance(node, ast.Name)}
    cmake = (ROOT / "workcell_builder" / "workcell_builder" / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "../../scripts/workcell_studio_planning_scene_node.py" in cmake
    assert "../../scripts/generate_moveit_collision_manifest.py" in cmake
    package = (ROOT / "scenes" / "ur5_2f_test" / "package.xml").read_text(encoding="utf-8")
    for dependency in ("moveit_msgs", "shape_msgs", "geometry_msgs", "rclpy"):
        assert f"<exec_depend>{dependency}</exec_depend>" in package
