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
