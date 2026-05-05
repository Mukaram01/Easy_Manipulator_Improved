#!/usr/bin/env python3
import importlib.util
from pathlib import Path


def main() -> int:
    script = Path(__file__).resolve().parents[3] / "scripts" / "replay_emd_bridge_payload.py"
    spec = importlib.util.spec_from_file_location("r", script)
    r = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(r)

    msg = r._format_runtime_response(True, "ok", "task_1", 3)
    assert "success=True" in msg and "task_id='task_1'" in msg and "grasp_targets=3" in msg
    assert r._count_grasp_targets({"grasp_task": {"grasp_targets": [1, 2]}}) == 2
    assert r._count_grasp_targets({}) == 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
