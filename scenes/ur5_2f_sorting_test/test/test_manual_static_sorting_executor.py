#!/usr/bin/env python3
"""Validate manual_static_sorting_executor safety-gated behavior."""

import argparse
import importlib.util
from pathlib import Path
import sys
from unittest.mock import patch


def main() -> int:
    scene_root = Path(__file__).resolve().parents[1]
    script_path = scene_root / "scripts" / "manual_static_sorting_executor.py"
    spec = importlib.util.spec_from_file_location("m", script_path)
    m = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(m)

    payload_path = Path("/tmp/fake_runtime_payload.json")

    class R:
        def __init__(self, code=0):
            self.returncode = code
            self.stdout = ""
            self.stderr = ""

    def fake_run(cmd):
        if "generate_static_sorting_runtime_bridge_payload" in " ".join(cmd):
            payload_path.write_text("{}", encoding="utf-8")
        return R(0)

    base_args = dict(json=True, output=None, prepare_output=None, skip_send_dry_run_validation=False, runtime_payload=None,
                     payload_output=payload_path, ros_interface="service", service_name="grasp_requests", topic_name="grasp_tasks", targets=None)

    with patch.object(m, "_run_subprocess", side_effect=fake_run), patch.object(m, "_find_replay_script", return_value=Path("/tmp/replay.py")):
        args = argparse.Namespace(**base_args, require_active_runtime=False, manual_enable_execution=False, execute=False, confirm_runtime_send=False)
        rep, _ = m.build_report(args)
        assert rep["result"]["status"] == "dry_run_only"
        assert rep["execution_attempted"] is False and rep["robot_motion_requested"] is False

        args = argparse.Namespace(**base_args, require_active_runtime=False, manual_enable_execution=False, execute=True, confirm_runtime_send=False)
        rep, rc = m.build_report(args)
        assert rc != 0 and rep["result"]["status"] == "blocked"

        args = argparse.Namespace(**base_args, require_active_runtime=False, manual_enable_execution=True, execute=False, confirm_runtime_send=False)
        rep, rc = m.build_report(args)
        assert rc == 0 and rep["result"]["status"] == "ready_for_manual_runtime"

        args = argparse.Namespace(**base_args, require_active_runtime=False, manual_enable_execution=True, execute=True, confirm_runtime_send=False)
        rep, rc = m.build_report(args)
        assert rc != 0 and rep["result"]["status"] == "blocked"

    commands = []
    def fake_run_send(cmd):
        commands.append(cmd)
        return R(0)

    with patch.object(m, "_run_subprocess", side_effect=fake_run_send), patch.object(m, "_find_replay_script", return_value=Path("/tmp/replay.py")), patch.object(
        m, "_runtime_checks", return_value=({"checked": True, "grasp_execution_node": "present", "joint_states": "present", "controller_manager": "present", "ur5_arm_controller": "active", "grasp_requests_service": "present", "grasp_tasks_topic": "present"}, [])
    ):
        args = argparse.Namespace(**base_args, require_active_runtime=True, manual_enable_execution=True, execute=True, confirm_runtime_send=True)
        rep, _ = m.build_report(args)
        assert rep["result"]["status"] == "sent_to_runtime"
        assert rep["execution_attempted"] is True and rep["robot_motion_requested"] is True
        assert any("--dry-run" in c for c in commands)
        assert any("--dry-run" not in c and "replay.py" in " ".join(c) for c in commands)

    return 0


if __name__ == "__main__":
    sys.exit(main())
