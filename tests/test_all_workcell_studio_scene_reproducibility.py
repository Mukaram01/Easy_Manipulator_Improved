from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


def test_all_scene_reproducibility_validator_runs_and_emits_json_report():
    repo_root = Path(__file__).resolve().parents[1]
    script = repo_root / "scripts" / "validate_all_workcell_studio_scenes.py"
    report = repo_root / "build" / "workcell_studio_scene_reproducibility_report.json"

    if report.exists():
        report.unlink()

    proc = subprocess.run([sys.executable, str(script)], cwd=repo_root, capture_output=True, text=True)

    assert proc.returncode == 0, proc.stdout + "\n" + proc.stderr
    assert report.exists(), "Expected JSON report was not created"

    payload = json.loads(report.read_text(encoding="utf-8"))
    assert isinstance(payload.get("scene_count"), int)
    assert isinstance(payload.get("scenes"), list)

    status_counts = payload.get("status_counts")
    assert isinstance(status_counts, dict)
    assert status_counts.get("blocked") == 0

    required_yaml_files = {
        "scene_manifest.yaml",
        "environment.yaml",
        "cell_definition.yaml",
        "layout/workcell_studio_layout.yaml",
    }

    for scene in payload["scenes"]:
        assert scene.get("status") in {"ready", "warning"}

        yaml_parse = scene.get("yaml_parse")
        assert isinstance(yaml_parse, dict)
        for yaml_name in required_yaml_files:
            assert yaml_parse.get(yaml_name) == "ok"

    assert "Traceback" not in (proc.stdout + proc.stderr)
