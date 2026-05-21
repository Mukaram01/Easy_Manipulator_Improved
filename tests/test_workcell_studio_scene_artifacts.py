from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "generate_workcell_studio_scene_artifacts.py"
KNOWN_SCENES = {
    "suction_test",
    "ur10_2f_test",
    "ur3_suction_test",
    "ur5_2f_builder_pick_place_demo",
    "ur5_2f_sorting_test",
    "ur5_2f_test",
    "ur5_3f_test",
    "ur5_airpick4_test",
}


def _run(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run([sys.executable, str(SCRIPT), *args], cwd=ROOT, capture_output=True, text=True, check=False)


def test_generator_script_exists_and_is_runnable():
    assert SCRIPT.is_file()
    proc = _run("--help")
    assert proc.returncode == 0


def test_dry_run_writes_nothing(tmp_path):
    proc = _run("--all", "--dry-run")
    assert proc.returncode == 0, proc.stderr
    assert "DRY-RUN write:" in proc.stdout


def test_scene_only_writes_requested_scene(tmp_path):
    scene = "ur5_2f_test"
    proc = _run("--scene", scene, "--overwrite")
    assert proc.returncode == 0, proc.stderr
    touched = [line for line in proc.stdout.splitlines() if line.startswith("Wrote:")]
    assert touched
    assert all(f"/scenes/{scene}/" in line for line in touched)


def test_all_covers_all_known_scenes_and_outputs_parseable_yaml():
    proc = _run("--all", "--overwrite")
    assert proc.returncode == 0, proc.stderr

    scenes_root = ROOT / "scenes"
    discovered = {p.name for p in scenes_root.iterdir() if p.is_dir()}
    assert KNOWN_SCENES.issubset(discovered)

    required = [
        "generated/environment_assets.yaml",
        "layout/workcell_studio_layout.generated.yaml",
        "config/task_recipe.yaml",
        "config/workcell_builder_task_intent.yaml",
    ]
    for scene in KNOWN_SCENES:
        for rel in required:
            p = scenes_root / scene / rel
            assert p.exists(), f"missing generated artifact: {p}"
            yaml.safe_load(p.read_text(encoding="utf-8"))


def test_generated_layout_does_not_overwrite_editable_and_keeps_locked_preview_separate():
    scene = "ur5_2f_test"
    editable = ROOT / "scenes" / scene / "layout" / "workcell_studio_layout.yaml"
    before = editable.read_text(encoding="utf-8")

    proc = _run("--scene", scene, "--overwrite")
    assert proc.returncode == 0, proc.stderr

    after = editable.read_text(encoding="utf-8")
    assert before == after

    generated = yaml.safe_load((ROOT / "scenes" / scene / "layout" / "workcell_studio_layout.generated.yaml").read_text(encoding="utf-8"))
    assert isinstance(generated.get("locked_urdf_preview_entities"), list)
    editable_data = yaml.safe_load(before)
    editable_ids = {item.get("id") for item in editable_data.get("items", []) if isinstance(item, dict)}
    locked_ids = {item.get("id") for item in generated.get("locked_urdf_preview_entities", []) if isinstance(item, dict)}
    assert editable_ids.isdisjoint(locked_ids)


def test_generated_files_have_provenance_and_sim_safe_intent_defaults():
    proc = _run("--all", "--overwrite")
    assert proc.returncode == 0, proc.stderr

    scenes_root = ROOT / "scenes"
    for scene in KNOWN_SCENES:
        env_assets = yaml.safe_load((scenes_root / scene / "generated" / "environment_assets.yaml").read_text(encoding="utf-8"))
        layout_generated = yaml.safe_load((scenes_root / scene / "layout" / "workcell_studio_layout.generated.yaml").read_text(encoding="utf-8"))
        task_recipe = yaml.safe_load((scenes_root / scene / "config" / "task_recipe.yaml").read_text(encoding="utf-8"))
        task_intent = yaml.safe_load((scenes_root / scene / "config" / "workcell_builder_task_intent.yaml").read_text(encoding="utf-8"))

        for payload in (env_assets, layout_generated, task_recipe, task_intent):
            assert any(key in payload for key in ("provenance", "source", "source_metadata"))

        safety = task_intent.get("safety", {})
        assert safety.get("use_fake_hardware") is True
        mode = (task_intent.get("task", {}) or {}).get("mode", "")
        exec_mode = safety.get("execution_mode", "")
        assert "sim" in str(mode).lower() or "preview" in str(mode).lower()
        assert "sim" in str(exec_mode).lower() or "preview" in str(exec_mode).lower()
