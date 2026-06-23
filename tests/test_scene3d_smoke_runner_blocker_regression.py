#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path


def test_scene3d_smoke_runner_reports_no_physical_items_instead_of_json_unreadable(tmp_path):
    repo = Path(__file__).resolve().parents[1]
    fake_builder = tmp_path / "fake_workcell_builder_no_physical.py"
    fake_builder.write_text(
        "#!/usr/bin/env python3\n"
        "import json, pathlib, sys\n"
        "out = pathlib.Path(sys.argv[sys.argv.index('--smoke-output') + 1])\n"
        "out.parent.mkdir(parents=True, exist_ok=True)\n"
        "out.write_text(json.dumps({\n"
        "    'schema': 'workcell_studio_scene3d_gui_smoke/v1',\n"
        "    'status': 'PASS',\n"
        "    'counters': {'rendered_count': 1},\n"
        "    'render_debug_counters': {'physical_mesh_items_rendered': 0, 'primitive_fallback_items_rendered': 0},\n"
        "}) + '\\n', encoding='utf-8')\n",
        encoding="utf-8",
    )
    fake_builder.chmod(0o755)

    out = tmp_path / "smoke.json"
    cmd = [
        sys.executable,
        str(repo / "scripts/run_workcell_builder_scene3d_gui_smoke.py"),
        "--repo-root",
        str(repo),
        "--workspace-root",
        str(tmp_path),
        "--scene",
        "ur5_2f_test",
        "--output",
        str(out),
        "--executable",
        str(fake_builder),
        "--timeout-sec",
        "2",
    ]
    env = os.environ.copy()
    env["ROS_DISTRO"] = "humble"

    proc = subprocess.run(cmd, text=True, capture_output=True, env=env)

    assert proc.returncode != 0
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["status"] == "FAIL"
    assert payload["wrapper_status"] == "FAIL"
    assert "scene_rendered_no_physical_items" in payload["blockers"]
    assert not any(str(blocker).startswith("app_smoke_json_unreadable") for blocker in payload["blockers"])
    assert "APP_JSON_UNREADABLE" not in proc.stdout
