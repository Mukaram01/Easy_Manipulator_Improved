from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path


def test_golden_builder_readiness_demo_end_to_end(tmp_path: Path) -> None:
    scene = tmp_path / "scene"
    shutil.copytree("scenes/ur5_2f_test", scene)
    out = tmp_path / "golden_pack"
    run = subprocess.run([
        sys.executable,
        "scripts/run_golden_builder_readiness_demo.py",
        "--scene-package", str(scene),
        "--output-dir", str(out),
        "--force",
        "--json",
    ], capture_output=True, text=True, check=False)
    assert run.returncode == 0

    manifest_path = out / "readiness_pack_manifest.json"
    assert manifest_path.exists()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    assert Path(manifest["artifacts"]["cell_definition"]).exists()
    assert Path(manifest["artifacts"]["environment_layout"]).exists()
    assert Path(manifest["artifacts"]["builder_task_intent"]).exists()
    assert Path(manifest["artifacts"]["task_flow_summary"]).exists()
    assert Path(manifest["artifacts"]["readiness_dashboard"]).exists()

    assert manifest["results"]["task_intent_status"] == "PASS"
    assert manifest["results"].get("classification") != "physical_scene_only"
    assert manifest["safety"]["real_hardware_enabled"] is False
    assert manifest["safety"]["motion_command_sent"] is False
    assert manifest["safety"]["runtime_execution_called"] is False

    dashboard = Path(manifest["artifacts"]["readiness_dashboard"]).read_text(encoding="utf-8")
    assert "Task flow: pick → grasp → place → release" in dashboard
    assert "Offline/fake-hardware readiness review only" in dashboard
