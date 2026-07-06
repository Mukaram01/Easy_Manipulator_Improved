import json
import shutil
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
FRESHENER = REPO_ROOT / "scripts" / "ensure_workcell_studio_web_scene_fresh.py"
CANONICAL_SCENE = REPO_ROOT / "scenes" / "ur5_2f_test"


def _copy_scene_without_generated(tmp_path: Path) -> Path:
    scene = tmp_path / "ur5_2f_test_fresh_copy"
    shutil.copytree(
        CANONICAL_SCENE,
        scene,
        ignore=shutil.ignore_patterns("generated", "__pycache__"),
    )
    generated = scene / "generated"
    if generated.exists():
        shutil.rmtree(generated)
    return scene


def test_freshener_creates_mesh_index_and_web_scene_for_scene_without_generated_dir(tmp_path):
    scene = _copy_scene_without_generated(tmp_path)
    output = tmp_path / "exports" / f"{scene.name}.web_scene.json"

    assert not (scene / "generated").exists()
    assert output.is_relative_to(tmp_path)

    result = subprocess.run(
        [
            sys.executable,
            str(FRESHENER),
            "--scene",
            str(scene),
            "--output",
            str(output),
            "--stage-assets",
            "--force",
        ],
        cwd=REPO_ROOT,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    mesh_index = scene / "generated" / "scene_visual_mesh_index.json"
    assert mesh_index.is_file()
    assert output.is_file()

    mesh_payload = json.loads(mesh_index.read_text(encoding="utf-8"))
    web_payload = json.loads(output.read_text(encoding="utf-8"))
    assert mesh_payload.get("extractor_version")
    assert mesh_payload.get("visual_items")
    assert web_payload.get("schema_version") == "workcell_studio_web_scene/v1"
    assert output.parent == tmp_path / "exports"
    assert not output.is_relative_to(REPO_ROOT / "scenes")
