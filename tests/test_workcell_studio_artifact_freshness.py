import json
import subprocess
import sys
import time
from pathlib import Path


def test_layout_newer_than_merge_marks_stale(tmp_path: Path):
    scene = tmp_path / "ur5_robotiq_pick_place"
    subprocess.run([sys.executable, "scripts/run_workcell_studio_golden_flow.py", "--scene-dir", str(scene)], check=True)
    layout = scene / "layout" / "workcell_studio_layout.yaml"
    time.sleep(1)
    layout.write_text(layout.read_text(encoding="utf-8") + "\n# touch\n", encoding="utf-8")
    subprocess.run([sys.executable, "scripts/validate_workcell_studio_generated_scene.py", str(scene), "--json"], check=False)
    acceptance = json.loads((scene / "acceptance" / "generated_scene_acceptance.json").read_text(encoding="utf-8"))
    assert acceptance["layout_stale"] is True
