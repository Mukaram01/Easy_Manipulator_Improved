#!/usr/bin/env python3
from __future__ import annotations
import json, os, subprocess
from pathlib import Path

def main() -> int:
    exe = os.environ.get("WORKCELL_BUILDER_EXE", "workcell_builder")
    env = dict(os.environ)
    env.setdefault("QT_QPA_PLATFORM", "offscreen")
    cmd = [exe, "--self-test-gui"]
    run = subprocess.run(cmd, env=env, capture_output=True, text=True)
    report = Path('/tmp/workcell_builder_gui_acceptance_report.json')
    if run.returncode != 0:
        print(run.stdout)
        print(run.stderr)
        return run.returncode
    if not report.exists():
        raise SystemExit("missing acceptance report json")
    payload = json.loads(report.read_text())
    if not payload.get('pass', False):
        raise SystemExit(f"acceptance failed: {payload.get('missing_items', [])}")
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
