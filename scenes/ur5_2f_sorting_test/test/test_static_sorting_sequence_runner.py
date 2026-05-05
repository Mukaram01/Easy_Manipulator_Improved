#!/usr/bin/env python3
"""Tests for static_sorting_sequence_runner."""

import argparse
import importlib.util
from pathlib import Path
import sys
from unittest.mock import patch


def main() -> int:
    scene_root = Path(__file__).resolve().parents[1]
    script_path = scene_root / "scripts" / "static_sorting_sequence_runner.py"
    spec = importlib.util.spec_from_file_location("seq", script_path)
    seq = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(seq)

    payload_dir = Path("/tmp/ur5_2f_sorting_sequence_tests")

    def ns(**overrides):
        base = dict(json=True, print_commands=False, target=None, all_targets=False, payload_output_dir=payload_dir,
                    require_active_runtime=False, execute=False, confirm_runtime_send=False, allow_batch_runtime_send=False)
        base.update(overrides)
        return argparse.Namespace(**base)

    calls = []
    def fake_build_report(ns):
        calls.append(ns)
        return ({"result": {"status": "dry_run_only"}, "runtime_send": {"executed": False}}, 0)

    with patch.object(seq.manual_executor, "build_report", side_effect=fake_build_report):
        rep, rc = seq.build_report(ns())
        assert rc == 0
        assert rep["result"]["status"] == "dry_run_only"
        assert rep["safety"]["robot_motion_requested"] is False

    calls.clear()
    with patch.object(seq.manual_executor, "build_report", side_effect=fake_build_report):
        rep, _ = seq.build_report(ns(target="item_red"))
        assert rep["selected_targets"] == ["item_red"]
        assert len(rep["sequence"]) == 1

    calls.clear()
    with patch.object(seq.manual_executor, "build_report", side_effect=fake_build_report):
        rep, _ = seq.build_report(ns(all_targets=True))
        assert rep["selected_targets"] == ["item_red", "item_blue", "item_green"]

    with patch.object(seq.manual_executor, "build_report", side_effect=fake_build_report):
        rep, rc = seq.build_report(ns(all_targets=True, execute=True, confirm_runtime_send=True))
        assert rc == 2
        assert rep["result"]["status"] == "blocked"

    calls.clear()
    with patch.object(seq.manual_executor, "build_report", side_effect=fake_build_report):
        rep, rc = seq.build_report(ns(target="item_red", execute=True, confirm_runtime_send=True))
        assert rc == 0
        assert rep["result"]["status"] == "sequence_runtime_send_succeeded"
        assert calls[0].manual_enable_execution is True

    calls.clear()
    with patch.object(seq.manual_executor, "build_report", side_effect=fake_build_report):
        rep, rc = seq.build_report(ns(all_targets=True, execute=True, confirm_runtime_send=True, allow_batch_runtime_send=True))
        assert rc == 0
        assert len(calls) == 3
        assert all(c.manual_enable_execution for c in calls)
        assert rep["result"]["status"] == "sequence_runtime_send_succeeded"

    idx = 0
    def fail_second(ns):
        nonlocal idx
        idx += 1
        if idx == 2:
            return ({"result": {"status": "runtime_send_failed"}, "runtime_send": {"executed": True}}, 2)
        return ({"result": {"status": "runtime_send_succeeded"}, "runtime_send": {"executed": True}}, 0)

    with patch.object(seq.manual_executor, "build_report", side_effect=fail_second):
        rep, rc = seq.build_report(ns(all_targets=True, execute=True, confirm_runtime_send=True, allow_batch_runtime_send=True))
        assert rc == 2
        assert rep["result"]["status"] == "sequence_runtime_send_failed"
        assert rep["result"]["failed_target"] == "item_blue"

    assert "safety" in rep and "result" in rep and "sequence" in rep
    return 0


if __name__ == "__main__":
    sys.exit(main())
