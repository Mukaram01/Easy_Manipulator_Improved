from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import yaml


def test_all_scene_reproducibility_validator_runs_and_emits_json_report():
    repo_root = Path(__file__).resolve().parents[1]
    script = repo_root / "scripts" / "validate_all_workcell_studio_scenes.py"
    report = repo_root / "build" / "workcell_studio" / "all_scene_reproducibility_report.json"

    proc = subprocess.run([sys.executable, str(script)], cwd=repo_root, capture_output=True, text=True)

    assert report.exists(), proc.stdout + "\n" + proc.stderr
    payload = json.loads(report.read_text(encoding="utf-8"))
    assert isinstance(payload.get("scene_count"), int)
    assert isinstance(payload.get("scenes"), list)
    assert payload.get("status_counts", {}).keys() >= {"PASS", "WARN", "FAIL", "SKIP"}

    assert "Traceback" not in (proc.stdout + proc.stderr)


def test_report_scene_set_matches_supported_scene_catalog():
    repo_root = Path(__file__).resolve().parents[1]
    script = repo_root / "scripts" / "validate_all_workcell_studio_scenes.py"
    catalog_path = repo_root / "scenes" / "supported_scenes.yaml"
    report = repo_root / "build" / "workcell_studio" / "all_scene_reproducibility_report.json"

    subprocess.run([sys.executable, str(script)], cwd=repo_root, check=True, capture_output=True, text=True)
    payload = json.loads(report.read_text(encoding="utf-8"))
    catalog = yaml.safe_load(catalog_path.read_text(encoding="utf-8"))

    expected = {
        entry["scene_name"]: entry
        for entry in catalog["scenes"]
        if entry.get("enabled", True)
    }
    actual = {scene["scene_name"]: scene for scene in payload["scenes"]}

    assert payload["supported_scene_catalog"].endswith("scenes/supported_scenes.yaml")
    assert set(actual) == set(expected)
    for scene_name, entry in expected.items():
        scene = actual[scene_name]
        assert scene["scene_name"] == entry["scene_name"]
        if entry.get("known_blocker"):
            assert any(entry["known_blocker"] in blocker for blocker in scene.get("blockers", []))


def test_validator_has_no_suction_only_special_case():
    repo_root = Path(__file__).resolve().parents[1]
    script_text = (repo_root / "scripts" / "validate_all_workcell_studio_scenes.py").read_text(encoding="utf-8")
    assert "== \"suction_test\"" not in script_text
    assert "!= \"suction_test\"" not in script_text
