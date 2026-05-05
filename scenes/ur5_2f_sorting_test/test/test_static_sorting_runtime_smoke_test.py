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

    class R:
        def __init__(self, stdout: str, code: int = 0):
            self.returncode = code
            self.stdout = stdout
            self.stderr = ""

    payload_path = Path("/tmp/ur5_2f_sorting_runtime_bridge_payload.json")

    def fake_manual_executor(cmd):
        selected = []
        i = 0
        while i < len(cmd):
            if cmd[i] == "--target":
                selected.append(cmd[i + 1])
                i += 2
            else:
                i += 1
        if not selected:
            selected = ["item_red"]
        payload = {
            "grasp_task": {"grasp_targets": [{"object_id": t, "destination_id": "bin_a", "destination_pose": {"frame_id": "world"}} for t in selected]}
        }
        payload_path.write_text(json.dumps(payload), encoding="utf-8")
        report = {
            "payload": {"path": str(payload_path)},
            "payload_validation": {"dry_run_replay_status": "pass"},
            "runtime_checks": {"checked": "--require-active-runtime" in cmd},
            "execution_attempted": "--manual-enable-execution" in cmd,
            "robot_motion_requested": "--manual-enable-execution" in cmd,
            "result": {"status": "dry_run_only"},
        }
        return R(json.dumps(report))

    with patch.object(smoke, "_run_subprocess", side_effect=fake_manual_executor):
        args = argparse.Namespace(target="item_red", all=False, allow_batch_runtime_send=False, payload_output=payload_path, json=True, print_commands=False, require_active_runtime=False, execute=False, confirm_runtime_send=False)
        rep, rc = smoke.build_report(args)
        assert rc == 0
        assert rep["robot_motion_requested"] is False

        one_args = argparse.Namespace(target="item_red", all=False, allow_batch_runtime_send=False, payload_output=payload_path, json=True, print_commands=False, require_active_runtime=False, execute=False, confirm_runtime_send=False)
        rep, _ = smoke.build_report(one_args)
        assert rep["target_count"] == 1
        assert rep["selected_target_ids"] == ["item_red"]

        all_args = argparse.Namespace(target=None, all=True, allow_batch_runtime_send=False, payload_output=payload_path, json=True, print_commands=False, require_active_runtime=False, execute=False, confirm_runtime_send=False)
        rep, _ = smoke.build_report(all_args)
        assert rep["target_count"] == 3
        assert rep["recommended_sequential_order"] == ["item_red", "item_blue", "item_green"]

        cmd_args = argparse.Namespace(target="item_red", all=False, allow_batch_runtime_send=False, payload_output=payload_path, json=True, print_commands=True, require_active_runtime=False, execute=False, confirm_runtime_send=False)
        rep, _ = smoke.build_report(cmd_args)
        assert "explicit_release_pose_source:=bridge_payload" in rep["launch_command"]
        assert "explicit_release_pose_bridge_payload_path:=/tmp/ur5_2f_sorting_runtime_bridge_payload.json" in rep["launch_command"]
        assert "ros2 run ur5_2f_sorting_test manual_static_sorting_executor --target item_red --require-active-runtime --manual-enable-execution --execute --confirm-runtime-send --json" == rep["execution_command"]

        guarded_args = argparse.Namespace(target="item_red", all=False, allow_batch_runtime_send=False, payload_output=payload_path, json=True, print_commands=False, require_active_runtime=False, execute=True, confirm_runtime_send=True)
        rep, _ = smoke.build_report(guarded_args)
        assert rep["final_send_command_executed"] is False
        assert rep["robot_motion_requested"] is False

    return 0


if __name__ == "__main__":
    sys.exit(main())
