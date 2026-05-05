#!/usr/bin/env python3
"""Tests for static_sorting_runtime_smoke_test helper safety and observability."""

import argparse
import importlib.util
import json
from pathlib import Path
import sys
from unittest.mock import patch


def main() -> int:
    scene_root = Path(__file__).resolve().parents[1]
    script_path = scene_root / "scripts" / "static_sorting_runtime_smoke_test.py"
    spec = importlib.util.spec_from_file_location("smoke", script_path)
    smoke = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(smoke)

    payload_path = Path("/tmp/ur5_2f_sorting_runtime_bridge_payload.json")

    class R:
        def __init__(self, stdout: str, code: int = 0):
            self.returncode = code
            self.stdout = stdout
            self.stderr = ""

    def fake_manual_executor(cmd):
        selected = [cmd[i + 1] for i, c in enumerate(cmd[:-1]) if c == "--target"] or ["item_red"]
        payload_path.write_text(json.dumps({"grasp_task": {"grasp_targets": [{"object_id": t, "destination_id": "bin_a", "destination_pose": {"frame_id": "world"}} for t in selected]}}), encoding="utf-8")
        report = {
            "payload": {"path": str(payload_path)},
            "payload_validation": {"dry_run_replay_status": "pass"},
            "runtime_checks": {"checked": "--require-active-runtime" in cmd},
            "execution_attempted": "--manual-enable-execution" in cmd,
            "robot_motion_requested": "--manual-enable-execution" in cmd,
            "runtime_send": {"executed": "--manual-enable-execution" in cmd, "exit_code": 0},
            "result": {"status": "runtime_send_succeeded" if "--manual-enable-execution" in cmd else "dry_run_only"},
        }
        return R(json.dumps(report))

    with patch.object(smoke, "_run_subprocess", side_effect=fake_manual_executor):
        args = argparse.Namespace(target="item_red", all=False, allow_batch_runtime_send=False, payload_output=payload_path, json=True, print_commands=False, require_active_runtime=False, execute=False, confirm_runtime_send=False)
        rep, rc = smoke.build_report(args)
        assert rc == 0
        assert rep["runtime_send"]["executed"] is False

        runtime_args = argparse.Namespace(target="item_red", all=False, allow_batch_runtime_send=False, payload_output=payload_path, json=True, print_commands=False, require_active_runtime=True, execute=True, confirm_runtime_send=True)
        rep, _ = smoke.build_report(runtime_args)
        assert rep["final_send_command_executed"] is True
        assert rep["runtime_send"]["executed"] is True
        assert rep["manual_executor_result"]["status"] == "runtime_send_succeeded"

    return 0


if __name__ == "__main__":
    sys.exit(main())
