import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "perceived_object_grasp_plan.py"
SPEC = importlib.util.spec_from_file_location("perceived_object_grasp_plan", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def box(object_id="2", x=0.4):
    return {"id": object_id, "shape": "BOX", "frame_id": "world",
            "dimensions": [0.08, 0.06, 0.12],
            "pose": [x, 0.1, 0.2, 0.0, 0.0, 0.0, 1.0]}


def test_selection_is_deterministic_and_preserves_perceived_identity():
    selected = MODULE.select_perceived_box([box("2"), box("1", 0.3)])
    target = MODULE.build_grasp_target(selected)
    assert target["perceived_object_id"] == "1"
    assert target["target_pose"] == [0.3, 0.1, 0.2, 0.0, 0.0, 0.0, 1.0]
    assert target["target_dimensions"] == [0.08, 0.06, 0.12]


def test_candidates_are_derived_from_box_pose_and_dimensions():
    target = MODULE.build_grasp_target(box("1", 0.37))
    candidates = MODULE.generate_box_grasp_candidates(target, clearance=0.1)
    assert len(candidates) == 8
    assert all(candidate[:2] == [0.37, 0.1] for candidate in candidates)
    assert all(candidate[2] == pytest.approx(0.36) for candidate in candidates)


@pytest.mark.parametrize("bad", [
    {"id": "1", "shape": "BOX", "frame_id": "world", "dimensions": [], "pose": [0] * 7},
    {"id": "1", "shape": "BOX", "frame_id": "world", "dimensions": [1, 1, 0], "pose": [0] * 6 + [1]},
    {"id": "1", "shape": "SPHERE", "frame_id": "world", "dimensions": [1, 1, 1], "pose": [0] * 6 + [1]},
])
def test_missing_or_invalid_box_geometry_is_rejected(bad):
    with pytest.raises(ValueError):
        MODULE.select_perceived_box([bad])


def test_execution_guard_fails_closed():
    guard = MODULE.ExecutionGuard()
    with pytest.raises(RuntimeError, match="forbidden"):
        guard.forbid_execution()
    assert guard.execution_attempted is True


def test_tool_goal_keeps_the_authored_clearance_at_the_grasp_frame():
    contract = MODULE.load_grasp_contract(SCRIPT.parents[1] / "scenes/ur5_2f_test")
    target = MODULE.build_grasp_target(box())
    grasp = MODULE.generate_box_grasp_candidates(target, clearance=0.12)[0]
    tool = MODULE.tool_pose_for_grasp(grasp, contract)
    assert tool[2] == pytest.approx(grasp[2] + 0.14)
    assert MODULE.compose_pose(tool, contract["tcp_pose"]) == pytest.approx(grasp)


def test_tcp_conversion_handles_lateral_offsets_and_rotated_mounts():
    grasp = [0.4, -0.2, 0.6] + MODULE.quaternion_from_rpy([0.1, 0.3, -0.6])
    contract = {"tcp_pose": [0.03, -0.02, 0.14] + MODULE.quaternion_from_rpy([0.3, -0.2, 0.5])}
    tool = MODULE.tool_pose_for_grasp(grasp, contract)
    assert MODULE.compose_pose(tool, contract["tcp_pose"]) == pytest.approx(grasp)


def test_package_name_resolves_same_contract_as_source_directory(monkeypatch):
    from ament_index_python import packages
    directory = SCRIPT.parents[1] / "scenes/ur5_2f_test"
    monkeypatch.setattr(packages, "get_package_share_directory", lambda name: str(directory))
    assert MODULE.load_grasp_contract("installed_scene") == MODULE.load_grasp_contract(directory)


def test_missing_handoff_does_not_silently_remove_tcp_offset(tmp_path):
    with pytest.raises(FileNotFoundError):
        MODULE.load_grasp_contract(tmp_path)


def test_invalid_tcp_handoff_is_rejected(tmp_path):
    (tmp_path / "cell_definition.yaml").write_text("end_effector: {tcp_pose_xyz: [0, 0, .nan]}\n")
    with pytest.raises(ValueError, match="tcp_pose_xyz"):
        MODULE.load_grasp_contract(tmp_path)
