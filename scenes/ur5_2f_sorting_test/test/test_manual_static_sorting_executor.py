#!/usr/bin/env python3
"""Validate manual_static_sorting_executor safety-gated behavior."""

import argparse
import importlib.util
import json
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
        def __init__(self, code=0, out="", err=""):
            self.returncode = code
            self.stdout = out
            self.stderr = err

    def fake_run(cmd):
        if "generate_static_sorting_runtime_bridge_payload" in " ".join(cmd):
            payload = {"summary": {"target_filter_applied": False}, "grasp_task": {"grasp_targets": [{"object_id": "item_red"}]}}
            payload_path.write_text(json.dumps(payload), encoding="utf-8")
        return R(0, "PASS")

    base_args = dict(json=True, output=None, prepare_output=None, skip_send_dry_run_validation=False, runtime_payload=None,
                     payload_output=payload_path, ros_interface="service", service_name="grasp_requests", topic_name="grasp_tasks", targets=None)

    with patch.object(m, "_run_subprocess", side_effect=fake_run), patch.object(m, "_find_replay_script", return_value=Path("/tmp/replay.py")):
        args = argparse.Namespace(**base_args, require_active_runtime=False, manual_enable_execution=False, execute=False, confirm_runtime_send=False)
        rep, _ = m.build_report(args)
        assert rep["result"]["status"] == "dry_run_only"
        assert rep["runtime_send"]["executed"] is False

    calls = []
    def fake_run_success(cmd):
        calls.append(cmd)
        if "generate_static_sorting_runtime_bridge_payload" in " ".join(cmd):
            payload_path.write_text(json.dumps({"summary": {"target_filter_applied": False}, "grasp_task": {"grasp_targets": [{"object_id": "item_red"}]}}), encoding="utf-8")
            return R(0, "")
        if "--dry-run" in cmd:
            return R(0, "PASS: dry-run")
        return R(0, "runtime_send_diagnostic: service response: success=True")

    with patch.object(m, "_run_subprocess", side_effect=fake_run_success), patch.object(m, "_find_replay_script", return_value=Path("/tmp/replay.py")), patch.object(
        m, "_runtime_checks", return_value=({"checked": True, "grasp_execution_node": "present", "joint_states": "present", "controller_manager": "present", "ur5_arm_controller": "active", "grasp_requests_service": "present", "grasp_tasks_topic": "present"}, [])
    ):
        args = argparse.Namespace(**base_args, require_active_runtime=True, manual_enable_execution=True, execute=True, confirm_runtime_send=True)
        rep, rc = m.build_report(args)
        assert rc == 0
        assert rep["result"]["status"] == "runtime_send_succeeded"
        assert rep["runtime_send"]["exit_code"] == 0

    long_err = "x" * 1800 + " timed out"
    def fake_run_fail(cmd):
        if "generate_static_sorting_runtime_bridge_payload" in " ".join(cmd):
            payload_path.write_text(json.dumps({"summary": {"target_filter_applied": False}, "grasp_task": {"grasp_targets": [{"object_id": "item_red"}]}}), encoding="utf-8")
            return R(0, "")
        if "--dry-run" in cmd:
            return R(0, "PASS: dry-run")
        return R(1, "", long_err)

    with patch.object(m, "_run_subprocess", side_effect=fake_run_fail), patch.object(m, "_find_replay_script", return_value=Path("/tmp/replay.py")), patch.object(
        m, "_runtime_checks", return_value=({"checked": True, "grasp_execution_node": "present", "joint_states": "present", "controller_manager": "present", "ur5_arm_controller": "active", "grasp_requests_service": "present", "grasp_tasks_topic": "present"}, [])
    ):
        args = argparse.Namespace(**base_args, require_active_runtime=True, manual_enable_execution=True, execute=True, confirm_runtime_send=True)
        rep, rc = m.build_report(args)
        assert rc == 2
        assert rep["result"]["status"] == "runtime_send_failed"
        assert rep["runtime_send"]["classified_failure_reason"] == "service_timeout"
        assert len(rep["runtime_send"]["stderr_tail"]) <= 1200

    return 0


if __name__ == "__main__":
    sys.exit(main())
