import ast
import math
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "ur5_2f_test"
LAUNCH = SCENE / "launch" / "demo.launch.py"
XACRO = SCENE / "urdf" / "scene.urdf.xacro"
LAYOUT = SCENE / "layout" / "workcell_studio_layout.yaml"


def _layout_helpers():
    """Load pure layout helpers without requiring a ROS installation."""
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"))
    wanted = {
        "CANONICAL_LAYOUT_REL_PATH",
        "CANONICAL_LAYOUT_SCHEMA",
        "REQUIRED_AUTHORED_POSE_IDS",
    }
    nodes = [
        node
        for node in tree.body
        if (
            isinstance(node, ast.Assign)
            and any(
                target.id in wanted
                for target in node.targets
                if isinstance(target, ast.Name)
            )
        )
        or (
            isinstance(node, ast.FunctionDef)
            and node.name
            in {"_format_xacro_vector", "load_canonical_layout_poses"}
        )
    ]
    namespace = {"os": __import__("os"), "math": math, "yaml": yaml, "scene_pkg": "ur5_2f_test"}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(LAUNCH), "exec"), namespace)
    return namespace


def test_launch_maps_required_canonical_layout_world_poses_to_xacro():
    source = LAUNCH.read_text(encoding="utf-8")
    xacro = XACRO.read_text(encoding="utf-8")

    assert 'CANONICAL_LAYOUT_REL_PATH = "layout/workcell_studio_layout.yaml"' in source
    assert '"support_surface_table"' in source
    assert '"realsense_overhead"' in source
    for mapping in (
        '"table_world_xyz": _format_xacro_vector(table_pose["xyz"])',
        '"table_world_rpy": _format_xacro_vector(table_pose["rpy"])',
        '"camera_world_xyz": _format_xacro_vector(camera_pose["xyz"])',
        '"camera_world_rpy": _format_xacro_vector(camera_pose["rpy"])',
    ):
        assert mapping in source

    assert '<origin xyz="$(arg table_world_xyz)" rpy="$(arg table_world_rpy)"/>' in xacro
    assert '<xacro:sensor_d435i parent="$(arg world_frame)"' in xacro
    assert '<origin xyz="$(arg camera_world_xyz)" rpy="$(arg camera_world_rpy)"/>' in xacro
    assert 'parent="table_"' not in xacro
    assert 'xyz="-0.58 0.12 0.655"' not in xacro


def test_changed_temp_layout_changes_xacro_pose_mappings_without_rewriting_yaml(tmp_path):
    helpers = _layout_helpers()
    copied_layout = tmp_path / "layout.yaml"
    canonical_before = LAYOUT.read_bytes()
    document = yaml.safe_load(LAYOUT.read_text(encoding="utf-8"))
    by_id = {item["id"]: item for item in document["items"]}
    by_id["support_surface_table"]["pose"]["xyz"][0] += 0.20
    by_id["realsense_overhead"]["pose"]["xyz"][1] += 0.15
    copied_layout.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    before = copied_layout.read_bytes()

    poses = helpers["load_canonical_layout_poses"](layout_path=copied_layout)
    mappings = {
        "table_world_xyz": helpers["_format_xacro_vector"](poses["support_surface_table"]["xyz"]),
        "table_world_rpy": helpers["_format_xacro_vector"](poses["support_surface_table"]["rpy"]),
        "camera_world_xyz": helpers["_format_xacro_vector"](poses["realsense_overhead"]["xyz"]),
        "camera_world_rpy": helpers["_format_xacro_vector"](poses["realsense_overhead"]["rpy"]),
    }

    assert mappings["table_world_xyz"] == "0.75 0 0.059999999999999998"
    assert mappings["camera_world_xyz"] == "0.34999999999999998 0.14999999999999999 0.84999999999999998"
    assert copied_layout.read_bytes() == before
    assert LAYOUT.read_bytes() == canonical_before


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (lambda doc: doc.update(schema_version="wrong/v1"), "must use schema_version"),
        (
            lambda doc: doc.__setitem__(
                "items", [item for item in doc["items"] if item["id"] != "realsense_overhead"]
            ),
            "realsense_overhead'; found 0",
        ),
        (
            lambda doc: next(
                item for item in doc["items"] if item["id"] == "support_surface_table"
            )["pose"].update(xyz=[0, math.inf, 0]),
            "support_surface_table' pose.xyz",
        ),
    ],
)
def test_malformed_required_layout_pose_fails_actionably(tmp_path, mutate, message):
    helpers = _layout_helpers()
    document = yaml.safe_load(LAYOUT.read_text(encoding="utf-8"))
    mutate(document)
    layout = tmp_path / "layout.yaml"
    layout.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")

    with pytest.raises(RuntimeError, match=message):
        helpers["load_canonical_layout_poses"](layout_path=layout)


def test_safe_launch_defaults_remain_locked():
    source = LAUNCH.read_text(encoding="utf-8")
    assert '"use_fake_hardware",\n            default_value="true"' in source
    assert '"allow_trajectory_execution": False' in source
    assert '"moveit_manage_controllers": False' in source
    assert "no pose fallback was used" in source
