from __future__ import annotations
import json, subprocess, tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def _run(cmd: list[str]):
    return subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True, check=False)


def test_create_cell_help() -> None:
    r = _run(["python3", "scripts/workcell_studio.py", "create-cell", "--help"])
    assert r.returncode == 0


def test_create_ur5_finger_cell() -> None:
    with tempfile.TemporaryDirectory() as d:
        out = Path(d) / "cell"
        r = _run(["python3", "scripts/workcell_studio.py", "create-cell", "--cell-id", "ur5_2f", "--robot", "ur5", "--end-effector", "robotiq_2f", "--sensor", "intel_realsense_d435i", "--task", "task_magnetic_pick_place", "--grasp-strategy", "finger_pinch_basic", "--output-dir", str(out), "--validate", "--force"])
        assert r.returncode == 0
        s = json.loads((out / "create_cell_summary.json").read_text())
        assert (out / "cell_definition.yaml").exists()
        assert s["safety_status"]["fake_hardware_default"] is True
        assert s["safety_status"]["motion_started"] is False
        assert s["safety_status"]["ros_launch_started"] is False


def test_incompatible_combo_handling() -> None:
    with tempfile.TemporaryDirectory() as d:
        out = Path(d) / "bad"
        bad = _run(["python3", "scripts/workcell_studio.py", "create-cell", "--cell-id", "bad", "--robot", "generic_delta_900", "--end-effector", "robotiq_2f_85", "--sensor", "intel_realsense_d435i", "--task", "task_magnetic_pick_place", "--grasp-strategy", "suction_top_basic", "--output-dir", str(out), "--force"])
        assert bad.returncode != 0
        ok = _run(["python3", "scripts/workcell_studio.py", "create-cell", "--cell-id", "bad2", "--robot", "generic_delta_900", "--end-effector", "robotiq_2f_85", "--sensor", "intel_realsense_d435i", "--task", "task_magnetic_pick_place", "--grasp-strategy", "suction_top_basic", "--output-dir", str(out), "--allow-incompatible", "--preview", "--force"])
        assert ok.returncode == 0
        s = json.loads((out / "create_cell_summary.json").read_text())
        assert s["runtime_mode"] in {"preview_only", "blocked"}
        assert s["blockers"]
