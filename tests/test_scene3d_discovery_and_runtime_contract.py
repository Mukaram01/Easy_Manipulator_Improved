import json
from pathlib import Path

from scripts.scene3d_scene_discovery import discover_scene3d_scenes
import scripts.validate_scene3d_runtime_acceptance as runtime


def _mk_scene(root: Path, name: str, files: list[str]):
    d = root / name
    d.mkdir(parents=True, exist_ok=True)
    for rel in files:
        p = d / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text("x", encoding="utf-8")


def test_discovery_ignores_hidden_and_helper_folders(tmp_path: Path):
    scenes = tmp_path / "scenes"
    scenes.mkdir()
    (scenes / ".workcell_studio_trash").mkdir()
    (scenes / "generated").mkdir()
    _mk_scene(scenes, "demo_scene", ["package.xml", "scene_manifest.yaml"])

    found = {x["scene"]: x for x in discover_scene3d_scenes(scenes)}
    assert found[".workcell_studio_trash"]["status"] == "IGNORED_NON_SCENE"
    assert found["generated"]["status"] == "IGNORED_NON_SCENE"
    assert found["demo_scene"]["status"] == "PASS"


def test_runtime_counter_reads_top_and_nested():
    assert runtime._runtime_counter({"rendered_count": 3}, "rendered_count") == 3
    assert runtime._runtime_counter({"counters": {"rendered_count": 4}}, "rendered_count") == 4
    assert runtime._runtime_counter({"counters": {"hierarchy_rows": 5}}, "hierarchy_rows_count") == 5


def test_all_scenes_excludes_ignored_non_scene(tmp_path: Path):
    scenes = tmp_path / "scenes"
    scenes.mkdir()
    (scenes / "task").mkdir()
    _mk_scene(scenes, "real_scene", ["package.xml", "scene_manifest.yaml"])
    out = [d["scene"] for d in discover_scene3d_scenes(scenes) if d["status"] != "IGNORED_NON_SCENE"]
    assert out == ["real_scene"]
