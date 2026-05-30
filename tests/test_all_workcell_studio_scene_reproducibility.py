from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import yaml


def _enabled_catalog_entries(repo_root: Path) -> list[dict]:
    catalog = yaml.safe_load((repo_root / "scenes" / "supported_scenes.yaml").read_text(encoding="utf-8"))
    entries = catalog.get("scenes", [])
    return [entry for entry in entries if entry.get("enabled", True)]


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
    report = repo_root / "build" / "workcell_studio" / "all_scene_reproducibility_report.json"

    subprocess.run([sys.executable, str(script)], cwd=repo_root, check=True, capture_output=True, text=True)
    payload = json.loads(report.read_text(encoding="utf-8"))

    expected_entries = _enabled_catalog_entries(repo_root)
    expected_names = {entry["scene_name"] for entry in expected_entries}
    actual_by_name = {entry.get("scene_name"): entry for entry in payload.get("scenes", [])}

    assert set(actual_by_name) == expected_names
    for catalog_entry in expected_entries:
        report_entry = actual_by_name[catalog_entry["scene_name"]]
        assert report_entry["support_level"] == catalog_entry["support_level"]
        assert report_entry["catalog_status"] == catalog_entry["status"]
        assert report_entry["known_blocker"] == catalog_entry["known_blocker"]


def test_validator_has_no_catalog_scene_name_special_cases():
    repo_root = Path(__file__).resolve().parents[1]
    script_text = (repo_root / "scripts" / "validate_all_workcell_studio_scenes.py").read_text(encoding="utf-8")

    for entry in _enabled_catalog_entries(repo_root):
        name = entry["scene_name"]
        assert f'== "{name}"' not in script_text
        assert f"== '{name}'" not in script_text
        assert f'!= "{name}"' not in script_text
        assert f"!= '{name}'" not in script_text
