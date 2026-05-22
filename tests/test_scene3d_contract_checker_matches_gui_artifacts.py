import json
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCENES = ("ur5_2f_test", "ur5_2f_sorting_test")


@pytest.mark.parametrize("scene_name", SCENES)
def test_contract_checker_consistency_with_runtime_artifact_counts(scene_name, tmp_path):
    runtime_json = tmp_path / f"{scene_name}_runtime.json"
    checker_json = tmp_path / f"{scene_name}_checker.json"

    subprocess.run(["python3", str(ROOT / "scripts" / "validate_scene3d_runtime_acceptance.py"), "--scene", scene_name, "--json", str(runtime_json)], check=False)
    subprocess.run(["python3", str(ROOT / "scripts" / "check_scene3d_canvas_contract.py"), "--scene", scene_name, "--json", str(checker_json)], check=False)

    runtime_scene = json.loads(runtime_json.read_text(encoding="utf-8"))["scenes"][0]
    checker_scene = json.loads(checker_json.read_text(encoding="utf-8"))["scenes"][0]

    assert runtime_scene["counts"]["editable_layout_count"] >= checker_scene["editable_layout_count"]
    assert runtime_scene["visibility_contract"]["visible_after_default_filters"] >= checker_scene["visible_after_filters_count"]
    assert isinstance(checker_scene["contract_status"], str)
