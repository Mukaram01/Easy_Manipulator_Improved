import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "perceived_object_grasp_execute.py"
SPEC = importlib.util.spec_from_file_location("perceived_object_grasp_execute", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def parameter(value):
    return SimpleNamespace(bool_value=value)


def component(class_type):
    return SimpleNamespace(class_type=class_type)


def box(object_id, dimensions):
    return {"id": object_id, "shape": "BOX", "frame_id": "world",
            "dimensions": dimensions, "pose": [0, 0, 0, 0, 0, 0, 1]}


def test_graspable_selection_rejects_oversize_lower_id_and_preserves_live_id():
    selected = MODULE.select_graspable_box([
        box("3", [0.40, 0.27, 0.05]), box("4", [0.11, 0.06, 0.05])])
    assert selected["id"] == "4"


def test_supported_object_proxy_penetration_is_corrected_minimally():
    mouse = box("6", [0.11, 0.06, 0.047])
    mouse["pose"][2] = 0.312
    suitcase = box("3", [0.40, 0.27, 0.046])
    suitcase["pose"][2] = 0.286
    correction, support_id = MODULE.support_penetration_correction(mouse, [suitcase, mouse])
    assert support_id == "3"
    assert correction == pytest.approx(0.0215)


def test_canonical_place_target_is_loaded_from_existing_layout(tmp_path):
    layout = tmp_path / "layout"
    layout.mkdir()
    (layout / "workcell_studio_layout.yaml").write_text(
        "items:\n- id: target_bin_default\n  pose:\n    xyz: [0.25, 0.45, 0.20]\n",
        encoding="utf-8")
    target = MODULE.load_canonical_place_target(tmp_path)
    assert target == {"id": "default_drop_zone", "target_id": "target_bin_default",
                      "frame_id": "world", "pose_xyz": [0.25, 0.45, 0.20]}


def test_translated_pose_preserves_orientation_and_input():
    from geometry_msgs.msg import PoseStamped

    original = PoseStamped()
    original.pose.position.x = 1.0
    original.pose.orientation.w = 1.0
    translated = MODULE.translated_pose(original, dx=0.2, dy=-0.3, dz=0.4)
    assert [translated.pose.position.x, translated.pose.position.y,
            translated.pose.position.z] == pytest.approx([1.2, -0.3, 0.4])
    assert translated.pose.orientation.w == 1.0
    assert original.pose.position.x == 1.0


def test_candidate_three_is_prioritized_without_dropping_existing_candidates():
    indices = MODULE.candidate_indices(16)
    assert indices[0] == 3
    assert sorted(indices) == list(range(16))


def test_fake_hardware_guard_requires_moveit_flag_and_mock_component():
    evidence = MODULE.fake_hardware_evidence(
        [parameter(True)], [component("mock_components/GenericSystem")])
    assert evidence["real_hardware"] is False


@pytest.mark.parametrize("params,components", [
    ([parameter(False)], [component("mock_components/GenericSystem")]),
    ([parameter(True)], [component("ur_robot_driver/URPositionHardwareInterface")]),
    ([parameter(True)], []),
])
def test_fake_hardware_guard_fails_closed(params, components):
    with pytest.raises(RuntimeError, match="execution rejected"):
        MODULE.fake_hardware_evidence(params, components)


def test_attachment_preserves_id_and_atomically_removes_world_object():
    from moveit_msgs.msg import CollisionObject

    original = CollisionObject()
    original.id = "17"
    original.header.frame_id = "world"
    diff = MODULE.attachment_diff(original, "ee_palm", ["ee_palm", "tool0"])
    assert diff.world.collision_objects[0].id == "17"
    assert diff.world.collision_objects[0].operation == CollisionObject.REMOVE
    attached = diff.robot_state.attached_collision_objects[0]
    assert attached.object.id == "17"
    assert attached.object.operation == CollisionObject.ADD
    assert attached.link_name == "ee_palm"


def test_attachment_tolerates_live_lifecycle_removing_world_object_first():
    from moveit_msgs.msg import CollisionObject

    original = CollisionObject()
    original.id = "17"
    original.header.frame_id = "world"
    diff = MODULE.attachment_diff(original, "ee_palm", ["ee_palm"], remove_world=False)
    assert diff.world.collision_objects == []
    assert diff.robot_state.attached_collision_objects[0].object.id == "17"


def test_attachment_status_requires_removed_world_and_matching_link():
    attached = SimpleNamespace(object=SimpleNamespace(id="1"), link_name="ee_palm")
    scene = SimpleNamespace(world=SimpleNamespace(collision_objects=[]),
                            robot_state=SimpleNamespace(attached_collision_objects=[attached]))
    assert MODULE.attachment_status(scene, "1", "ee_palm")["valid"] is True
    scene.world.collision_objects.append(SimpleNamespace(id="1"))
    assert MODULE.attachment_status(scene, "1", "ee_palm")["valid"] is False


def test_failure_cleanup_removes_exact_attachment():
    from moveit_msgs.msg import CollisionObject

    cleanup = MODULE.detachment_cleanup_diff("23", "ee_palm")
    item = cleanup.robot_state.attached_collision_objects[0]
    assert item.object.id == "23"
    assert item.object.operation == CollisionObject.REMOVE
    assert item.link_name == "ee_palm"


def test_place_detachment_preserves_id_and_restores_one_world_object():
    from moveit_msgs.msg import CollisionObject

    original = CollisionObject()
    original.id = "6"
    original.pose.orientation.w = 1.0
    diff = MODULE.place_detachment_diff(original, "ee_palm", [0.25, 0.45, 0.20])
    detached = diff.robot_state.attached_collision_objects[0]
    placed = diff.world.collision_objects[0]
    assert detached.object.id == placed.id == "6"
    assert detached.object.operation == CollisionObject.REMOVE
    assert placed.operation == CollisionObject.ADD
    assert [placed.pose.position.x, placed.pose.position.y, placed.pose.position.z] == [0.25, 0.45, 0.20]
