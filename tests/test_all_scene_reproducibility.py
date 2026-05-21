from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

KNOWN_SCENES = {
    "suction_test",
    "ur5_2f_test",
    "ur5_3f_test",
    "ur5_2f_builder_pick_place_demo",
    "ur5_2f_sorting_test",
    "ur3_suction_test",
    "ur10_2f_test",
    "ur5_airpick4_test",
}


def test_validator_emits_per_scene_report_and_contract_keys():
    repo_root = Path(__file__).resolve().parents[1]
    script = repo_root / "scripts" / "validate_all_workcell_studio_scenes.py"
    report = repo_root / "build" / "workcell_studio" / "all_scene_reproducibility_report.json"

    if report.exists():
        report.unlink()

    proc = subprocess.run([sys.executable, str(script)], cwd=repo_root, capture_output=True, text=True)

    assert report.exists(), proc.stdout + "\n" + proc.stderr
    payload = json.loads(report.read_text(encoding="utf-8"))
    scenes = payload.get("scenes")
    assert isinstance(scenes, list) and scenes

    names = {s.get("scene_name") for s in scenes}
    assert KNOWN_SCENES.issubset(names)

    expected_keys = {
        "scene_name",
        "status",
        "files",
        "optional_files",
        "generated_mesh_index_present",
        "mesh_index_regeneration_status",
        "mesh_index_renderable_items",
        "preview_readiness_status",
        "generated_artifacts_present",
        "fake_hardware_smoke_command_available",
        "fake_hardware_smoke_command",
        "blockers",
        "warnings",
    }
    for scene in scenes:
        assert expected_keys.issubset(scene.keys())
        assert scene["status"] in {"PASS", "WARN", "FAIL", "SKIP"}
        assert isinstance(scene["blockers"], list)
        assert isinstance(scene["warnings"], list)

    assert "ModuleNotFoundError: No module named 'yaml'" not in (proc.stdout + proc.stderr)
