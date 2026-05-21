from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


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


def test_known_scene_set_matches_expected_contract():
    repo_root = Path(__file__).resolve().parents[1]
    script_text = (repo_root / "scripts" / "validate_all_workcell_studio_scenes.py").read_text(encoding="utf-8")
    for name in [
        "suction_test",
        "ur10_2f_test",
        "ur3_suction_test",
        "ur5_2f_builder_pick_place_demo",
        "ur5_2f_sorting_test",
        "ur5_2f_test",
        "ur5_3f_test",
        "ur5_airpick4_test",
    ]:
        assert name in script_text


def test_validator_has_no_suction_only_special_case():
    repo_root = Path(__file__).resolve().parents[1]
    script_text = (repo_root / "scripts" / "validate_all_workcell_studio_scenes.py").read_text(encoding="utf-8")
    assert "== \"suction_test\"" not in script_text
    assert "!= \"suction_test\"" not in script_text
