from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.audit_workcell_cleanup_candidates import (  # noqa: E402
    audit_paths,
    build_arg_parser,
    classify_cleanup_candidate,
)


def _touch(path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("demo\n", encoding="utf-8")
    return path


def test_audit_workcell_cleanup_candidates_classifies_generated_artifacts(tmp_path: Path) -> None:
    files = {
        "python_bytecode": _touch(tmp_path / "pkg/__pycache__/mod.cpython-310.pyc"),
        "pytest_cache": _touch(tmp_path / ".pytest_cache/CACHEDIR.TAG"),
        "log": _touch(tmp_path / "build/workcell_studio/example.stdout.log"),
        "smoke": _touch(tmp_path / "build/workcell_studio/scene3d_gui_smoke_ur5_2f_test.json"),
        "backup": _touch(tmp_path / "notes.bak"),
    }

    classifications = {name: classify_cleanup_candidate(path, tmp_path) for name, path in files.items()}

    assert classifications["python_bytecode"]["category"] == "python_cache"
    assert classifications["python_bytecode"]["confidence"] == "high"
    assert classifications["python_bytecode"]["safe_to_delete_now"] is True
    assert classifications["pytest_cache"]["category"] == "python_cache"
    assert classifications["pytest_cache"]["confidence"] == "high"
    assert classifications["pytest_cache"]["safe_to_delete_now"] is True
    assert classifications["log"]["category"] == "generated_logs"
    assert classifications["smoke"]["category"] == "generated_smoke_outputs"
    assert classifications["backup"]["category"] == "temp_backup_files"


def test_audit_workcell_cleanup_candidates_protects_scene_runtime_assets(tmp_path: Path) -> None:
    protected_paths = [
        _touch(tmp_path / "scenes/demo/scene_manifest.yaml"),
        _touch(tmp_path / "scenes/demo/launch/demo.launch.py"),
        _touch(tmp_path / "scenes/demo/urdf/scene.urdf.xacro"),
        _touch(tmp_path / "scenes/demo/meshes/workpiece.stl"),
    ]

    classifications = audit_paths(protected_paths, tmp_path)

    assert classifications
    assert all(result["safe_to_delete_now"] is False for result in classifications)
    assert all(result["category"] == "protected_scene_runtime_asset" for result in classifications)


def test_audit_workcell_cleanup_candidates_has_no_delete_or_apply_cli_option() -> None:
    parser = build_arg_parser()
    option_strings = {
        option
        for action in parser._actions
        for option in action.option_strings
    }

    assert "--delete" not in option_strings
    assert "--apply" not in option_strings
