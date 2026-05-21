import subprocess
from pathlib import Path

from scripts.scene_root_resolver import resolve_scene_root
from scripts import check_scene3d_canvas_contract as canvas_contract
from scripts import validate_scene3d_runtime_acceptance as runtime_acceptance

ROOT = Path(__file__).resolve().parents[1]


def test_both_scripts_resolve_ur5_2f_test_from_same_root():
    shared_root = resolve_scene_root(ROOT)
    canvas_path = canvas_contract.SCENES_ROOT / "ur5_2f_test"
    runtime_path = runtime_acceptance.scene_dir("ur5_2f_test")

    assert canvas_contract.SCENES_ROOT == runtime_acceptance.SCENES_ROOT == shared_root
    assert canvas_path == runtime_path
    assert canvas_path.exists()


def test_missing_scene_error_message_is_explicit_for_both_scripts(tmp_path):
    missing_scene = "scene_does_not_exist_for_contract_test"

    canvas_proc = subprocess.run(
        ["python3", str(ROOT / "scripts" / "check_scene3d_canvas_contract.py"), "--scene", missing_scene],
        capture_output=True,
        text=True,
    )
    assert canvas_proc.returncode != 0
    assert "requested scene path is missing:" in canvas_proc.stderr

    runtime_proc = subprocess.run(
        ["python3", str(ROOT / "scripts" / "validate_scene3d_runtime_acceptance.py"), "--scene", missing_scene, "--json", str(tmp_path / "r.json"), "--markdown", str(tmp_path / "r.md")],
        capture_output=True,
        text=True,
    )
    assert runtime_proc.returncode != 0
    assert "requested scene path is missing:" in runtime_proc.stderr
