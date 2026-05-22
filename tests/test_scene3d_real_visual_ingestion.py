import json
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCENES = ("ur5_2f_test", "ur5_2f_sorting_test")


@pytest.fixture(params=SCENES)
def scene_name(request):
    return request.param


def _run_runtime_acceptance(scene_name: str, tmp_path: Path) -> dict:
    out_json = tmp_path / f"{scene_name}_runtime_acceptance.json"
    out_md = tmp_path / f"{scene_name}_runtime_acceptance.md"
    subprocess.run(
        [
            "python3",
            str(ROOT / "scripts" / "validate_scene3d_runtime_acceptance.py"),
            "--scene",
            scene_name,
            "--json",
            str(out_json),
            "--markdown",
            str(out_md),
        ],
        check=False,
    )
    return json.loads(out_json.read_text(encoding="utf-8"))


def test_fixture_scenes_ingestion_preserves_previewable_rows(scene_name, tmp_path):
    payload = _run_runtime_acceptance(scene_name, tmp_path)
    scene = payload["scenes"][0]
    counts = scene["counts"]
    visibility = scene["visibility_contract"]

    assert counts["editable_layout_count"] > 0
    assert visibility["visible_after_default_filters"] > 0
    assert visibility["input_items_count"] >= visibility["visible_after_default_filters"]


def test_unresolved_mesh_visuals_are_fallback_not_dropped(scene_name, tmp_path):
    payload = _run_runtime_acceptance(scene_name, tmp_path)
    scene = payload["scenes"][0]
    blockers = "\n".join(scene.get("blockers", [])).lower()

    # Guard against regressions where unresolved mesh visuals are silently dropped.
    assert "dropped" not in blockers
    assert scene["counts"]["editable_layout_count"] >= 1
