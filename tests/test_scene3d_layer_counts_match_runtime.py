import json
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCENES = ("ur5_2f_test", "ur5_2f_sorting_test")


@pytest.mark.parametrize("scene_name", SCENES)
def test_preview_capable_fixture_scenes_have_non_zero_runtime_layers(scene_name, tmp_path):
    out_json = tmp_path / f"{scene_name}_runtime_acceptance.json"
    subprocess.run(
        [
            "python3",
            str(ROOT / "scripts" / "validate_scene3d_runtime_acceptance.py"),
            "--scene",
            scene_name,
            "--json",
            str(out_json),
        ],
        check=False,
    )
    scene = json.loads(out_json.read_text(encoding="utf-8"))["scenes"][0]
    layers = scene["layers"]
    counts = scene["counts"]

    assert counts["editable_layout_count"] > 0
    assert layers["editable_layout_visible"] > 0
    assert scene["visibility_contract"]["visible_after_default_filters"] > 0
