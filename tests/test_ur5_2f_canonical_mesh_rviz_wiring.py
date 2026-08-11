import ast
import importlib.util
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
BUILDER = ROOT / "workcell_builder" / "workcell_builder"
SCENE = ROOT / "scenes" / "ur5_2f_test"
LAUNCH = SCENE / "launch" / "demo.launch.py"
LAYOUT = SCENE / "layout" / "workcell_studio_layout.yaml"


def _dependencies(package_xml):
    root = ET.parse(package_xml).getroot()
    return {
        element.text
        for element in root
        if element.tag in {"depend", "exec_depend"}
    }


def _publisher_helper():
    script = ROOT / "scripts" / "workcell_studio_layout_mesh_preview_node.py"
    spec = importlib.util.spec_from_file_location("canonical_mesh_preview", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_workcell_builder_installs_publisher_and_declares_runtime_dependencies():
    cmake = (BUILDER / "CMakeLists.txt").read_text(encoding="utf-8")
    runtime_install = re.search(
        r"install\(PROGRAMS\s+(.*?)DESTINATION lib/\$\{PROJECT_NAME\}\)",
        cmake,
        re.DOTALL,
    )
    assert runtime_install is not None
    assert "../../scripts/workcell_studio_layout_mesh_preview_node.py" in runtime_install.group(1)
    assert {"rclpy", "visualization_msgs", "python3-yaml"} <= _dependencies(
        BUILDER / "package.xml"
    )


def test_scene_declares_mesh_preview_runtime_dependencies():
    assert {"workcell_builder", "sorting_bin_description"} <= _dependencies(
        SCENE / "package.xml"
    )


def test_launch_wires_positional_canonical_layout_mesh_preview():
    source = LAUNCH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "Node"
        and any(
            keyword.arg == "executable"
            and isinstance(keyword.value, ast.Constant)
            and keyword.value.value == "workcell_studio_layout_mesh_preview_node.py"
            for keyword in node.keywords
        )
    ]
    assert len(calls) == 1
    keywords = {keyword.arg: keyword.value for keyword in calls[0].keywords}
    assert ast.literal_eval(keywords["package"]) == "workcell_builder"
    arguments = keywords["arguments"]
    assert isinstance(arguments, ast.List)
    assert isinstance(arguments.elts[0], ast.Name)
    assert arguments.elts[0].id == "canonical_layout_path"
    assert all(not (isinstance(item, ast.Constant) and item.value == "--layout") for item in arguments.elts)
    assert 'CANONICAL_LAYOUT_REL_PATH = "layout/workcell_studio_layout.yaml"' in source
    assert "get_package_share_directory(scene_pkg),\n        CANONICAL_LAYOUT_REL_PATH," in source
    assert '"--frame-id",\n            world_frame' in source
    assert 'f"/{scene_pkg}/canonical_mesh_markers"' in source
    assert re.search(r"return \[.*?canonical_mesh_preview,.*?\]", source, re.DOTALL)


def test_rviz_enables_transient_local_reliable_mesh_marker_display():
    config = yaml.safe_load((SCENE / "launch" / "demo.rviz").read_text(encoding="utf-8"))
    displays = config["Visualization Manager"]["Displays"]
    matches = [item for item in displays if item.get("Name") == "Workcell Imported Meshes"]
    assert len(matches) == 1
    display = matches[0]
    assert display["Class"] == "rviz_default_plugins/MarkerArray"
    assert display["Enabled"] is True
    assert display["Topic"] == {
        "Depth": 1,
        "Durability Policy": "Transient Local",
        "History Policy": "Keep Last",
        "Reliability Policy": "Reliable",
        "Value": "/ur5_2f_test/canonical_mesh_markers",
    }


def test_target_bin_uses_canonical_mesh_publisher_resolution():
    document = yaml.safe_load(LAYOUT.read_text(encoding="utf-8"))
    target_bin = next(item for item in document["items"] if item["id"] == "target_bin_default")
    assert target_bin["geometry_type"] == "mesh"
    specs = _publisher_helper().parse_layout_meshes(document)
    published_bin = next(spec for spec in specs if spec.item_id == "target_bin_default")
    assert published_bin.mesh_resource == (
        "package://sorting_bin_description/meshes/sorting_bin.stl"
    )


def test_existing_canonical_xacro_mapping_and_safety_guards_remain():
    source = LAUNCH.read_text(encoding="utf-8")
    for mapping in (
        '"table_world_xyz": _format_xacro_vector(table_pose["xyz"])',
        '"table_world_rpy": _format_xacro_vector(table_pose["rpy"])',
        '"camera_world_xyz": _format_xacro_vector(camera_pose["xyz"])',
        '"camera_world_rpy": _format_xacro_vector(camera_pose["rpy"])',
    ):
        assert mapping in source
    assert '"use_fake_hardware",\n            default_value="true"' in source
    assert '"allow_trajectory_execution": False' in source
    assert '"moveit_manage_controllers": False' in source
