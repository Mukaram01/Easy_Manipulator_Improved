from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
RVIZ = ROOT / "scenes" / "ur5_2f_test" / "launch" / "demo.rviz"


def test_rviz_defaults_to_authored_meshes_without_collision_proxy_occlusion():
    config = yaml.safe_load(RVIZ.read_text(encoding="utf-8"))
    displays = config["Visualization Manager"]["Displays"]

    planning_scene = next(item for item in displays if item.get("Name") == "PlanningScene")
    authored_meshes = next(item for item in displays if item.get("Name") == "Workcell Imported Meshes")

    assert planning_scene["Enabled"] is True
    assert planning_scene["Scene Geometry"]["Show Scene Geometry"] is False

    assert authored_meshes["Class"] == "rviz_default_plugins/MarkerArray"
    assert authored_meshes["Enabled"] is True
    assert authored_meshes["Value"] is True
    assert authored_meshes["Topic"]["Value"] == "/ur5_2f_test/canonical_mesh_markers"
