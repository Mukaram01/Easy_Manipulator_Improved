from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_object_placement_manager_ui_strings_exist():
    txt = (
        (ROOT / "workcell_builder/workcell_builder/gui/scene_select.cpp").read_text(encoding="utf-8")
        + (ROOT / "workcell_builder/workcell_builder/gui/addobject.cpp").read_text(encoding="utf-8")
        + (ROOT / "workcell_builder/workcell_builder/gui/object_placement_dialog.cpp").read_text(encoding="utf-8")
        + (ROOT / "workcell_builder/workcell_builder/include/yaml_parser/generate_yaml.h").read_text(encoding="utf-8")
    )
    for needle in [
        "Object Placement Manager",
        "Placed Objects",
        "Add Asset Object",
        "Import STL to Asset Library",
        "Duplicate Object",
        "Remove Object",
        "Edit Pose",
        "Refresh Preview",
        "Open RViz STL Preview",
        "Open Interactive RViz Preview",
        "Import RViz Pose Feedback",
        "Save Placed Objects to Scene YAML",
        "Placed object changes are pending",
        "Generate Files",
        "environment.yaml",
        "Apply Valid Updates",
        "Proposed XYZ/RPY",
        "safe_for_robot_motion",
        "placed_objects_feedback.yaml",
        "Add Camera",
        "Edit Camera Pose",
        "Open Camera Frustum Preview",
        "Save Cameras to Scene YAML",
        "camera_placements",
        "camera frustum",
        "Create Pick Zone",
        "Create Place Zone",
        "Edit Zone Pose",
        "Edit Zone Size",
        "Save Task Zones to Scene YAML",
        "Open Task Zone Preview",
        "task_zones",
        "robot_mount",
        "tool_attachment",
    ]:
        assert needle in txt


def test_negative_malformed_placed_object_and_legacy_scene_do_not_crash_markers_present():
    launch_py = (ROOT / "workcell_builder/workcell_builder/templates/ros2/launch/demo.launch.py").read_text(encoding="utf-8")
    assert 'if not isinstance(placed_objects, list) or not placed_objects:' in launch_py
    assert 'if isinstance(obj, dict):' in launch_py
    assert 'continue' in launch_py
