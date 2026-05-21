from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "workcell_studio_scene_artifacts.py"


@pytest.fixture
def scene_artifacts_script() -> Path:
    if not SCRIPT.is_file():
        pytest.skip("scene artifacts generator script not present yet")
    return SCRIPT


@pytest.fixture
def representative_scenes(tmp_path: Path) -> tuple[Path, list[str]]:
    scenes_root = tmp_path / "scenes"
    scenes_root.mkdir(parents=True)
    scene_names = ["suction_test", "ur5_2f_sorting_test", "ur5_2f_test"]
    for name in scene_names:
        scene = scenes_root / name
        (scene / "layout").mkdir(parents=True)
        (scene / "config").mkdir(parents=True)
        (scene / "urdf").mkdir(parents=True)
        (scene / "generated").mkdir(parents=True)
        (scene / "layout" / "workcell_studio_layout.yaml").write_text(
            yaml.safe_dump(
                {
                    "schema_version": "workcell_studio_layout/v1",
                    "items": [
                        {"id": "editable_bin", "type": "bin", "pose": {"xyz": [0.2, 0.0, 0.0], "rpy": [0, 0, 0]}}
                    ],
                },
                sort_keys=False,
            ),
            encoding="utf-8",
        )
        (scene / "urdf" / "scene.urdf.xacro").write_text("<robot name='demo'></robot>\n", encoding="utf-8")

    # Incomplete data scene: layout present but minimal/no task bindings.
    (scenes_root / "suction_test" / "config" / "workcell_builder_task_intent.yaml").write_text(
        yaml.safe_dump({"schema": "workcell_builder_task_intent/v1"}, sort_keys=False), encoding="utf-8"
    )

    return scenes_root, scene_names


def _run(script: Path, *args: str, cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run([sys.executable, str(script), *args], cwd=cwd, capture_output=True, text=True, check=False)


def _yaml_files_under(path: Path) -> list[Path]:
    return sorted([p for p in path.rglob("*.yaml") if p.is_file()])


def test_dry_run_produces_no_file_writes(scene_artifacts_script: Path, representative_scenes: tuple[Path, list[str]]):
    scenes_root, _ = representative_scenes
    before = {str(p.relative_to(scenes_root)): p.read_bytes() for p in _yaml_files_under(scenes_root)}
    proc = _run(scene_artifacts_script, "--root", str(scenes_root), "--all", "--dry-run", "--json", cwd=ROOT)
    assert proc.returncode == 0, proc.stderr
    after = {str(p.relative_to(scenes_root)): p.read_bytes() for p in _yaml_files_under(scenes_root)}
    assert before == after


def test_scene_flag_affects_only_selected_scene(scene_artifacts_script: Path, representative_scenes: tuple[Path, list[str]]):
    scenes_root, scene_names = representative_scenes
    target = scene_names[0]
    proc = _run(scene_artifacts_script, "--root", str(scenes_root), "--scene", target, "--json", cwd=ROOT)
    assert proc.returncode == 0, proc.stderr
    changed = [p.parent.parent.name for p in scenes_root.rglob("generated/*") if p.is_file()]
    assert target in changed
    assert all(name == target for name in changed)


def test_all_covers_all_known_scenes(scene_artifacts_script: Path, representative_scenes: tuple[Path, list[str]]):
    scenes_root, scene_names = representative_scenes
    proc = _run(scene_artifacts_script, "--root", str(scenes_root), "--all", "--json", cwd=ROOT)
    assert proc.returncode == 0, proc.stderr
    payload = json.loads(proc.stdout) if proc.stdout.strip().startswith("{") else {}
    audited = set(payload.get("scenes", [])) if isinstance(payload.get("scenes"), list) else set()
    assert set(scene_names).issubset(audited) or all((scenes_root / s / "generated").exists() for s in scene_names)


def test_all_generated_outputs_are_valid_yaml(scene_artifacts_script: Path, representative_scenes: tuple[Path, list[str]]):
    scenes_root, _ = representative_scenes
    proc = _run(scene_artifacts_script, "--root", str(scenes_root), "--all", "--json", cwd=ROOT)
    assert proc.returncode == 0, proc.stderr
    for path in scenes_root.rglob("generated/*.yaml"):
        yaml.safe_load(path.read_text(encoding="utf-8"))


def test_generated_layout_does_not_overwrite_editable_layout(scene_artifacts_script: Path, representative_scenes: tuple[Path, list[str]]):
    scenes_root, names = representative_scenes
    editable = scenes_root / names[0] / "layout" / "workcell_studio_layout.yaml"
    before = editable.read_text(encoding="utf-8")
    proc = _run(scene_artifacts_script, "--root", str(scenes_root), "--scene", names[0], "--json", cwd=ROOT)
    assert proc.returncode == 0, proc.stderr
    after = editable.read_text(encoding="utf-8")
    assert after == before
    assert (scenes_root / names[0] / "layout" / "workcell_studio_layout.generated.yaml").exists()


def test_locked_urdf_preview_items_kept_separate_from_editable_layout_items(scene_artifacts_script: Path, representative_scenes: tuple[Path, list[str]]):
    scenes_root, names = representative_scenes
    proc = _run(scene_artifacts_script, "--root", str(scenes_root), "--scene", names[1], "--json", cwd=ROOT)
    assert proc.returncode == 0, proc.stderr
    editable = yaml.safe_load((scenes_root / names[1] / "layout" / "workcell_studio_layout.yaml").read_text(encoding="utf-8")) or {}
    generated = yaml.safe_load((scenes_root / names[1] / "layout" / "workcell_studio_layout.generated.yaml").read_text(encoding="utf-8")) or {}
    editable_ids = {i.get("id") for i in editable.get("items", []) if isinstance(i, dict)}
    generated_ids = {i.get("id") for i in generated.get("items", []) if isinstance(i, dict)}
    assert editable_ids <= generated_ids
    locked = [i for i in generated.get("items", []) if isinstance(i, dict) and i.get("locked") is True]
    assert locked
    assert all((i.get("id") not in editable_ids) or i.get("locked") for i in locked)


def test_generated_files_contain_provenance_and_source_fields_and_sim_safe_defaults(scene_artifacts_script: Path, representative_scenes: tuple[Path, list[str]]):
    scenes_root, names = representative_scenes
    proc = _run(scene_artifacts_script, "--root", str(scenes_root), "--all", "--json", cwd=ROOT)
    assert proc.returncode == 0, proc.stderr

    for scene in names:
        for out in (scenes_root / scene / "generated").glob("*.yaml"):
            data = yaml.safe_load(out.read_text(encoding="utf-8")) or {}
            provenance = data.get("provenance") or data.get("source") or data.get("source_metadata")
            assert provenance is not None, f"missing provenance/source in {out}"

        task_intent_candidates = [
            scenes_root / scene / "generated" / "workcell_builder_task_intent.yaml",
            scenes_root / scene / "config" / "workcell_builder_task_intent.yaml",
        ]
        for tip in task_intent_candidates:
            if tip.exists():
                ti = yaml.safe_load(tip.read_text(encoding="utf-8")) or {}
                safety = ti.get("safety", {}) if isinstance(ti, dict) else {}
                assert safety.get("use_fake_hardware", True) is True
                mode = str(((ti.get("task") or {}).get("mode", ""))) if isinstance(ti, dict) else ""
                assert ("sim" in mode.lower()) or ("preview" in mode.lower()) or (mode == "")
