from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import export_workcell_studio_web_scene as exporter  # noqa: E402


def test_ur5_2f_overlay_recovers_repo_asset_from_workspace_scene_alias(tmp_path):
    repo_root = tmp_path / "easy_manipulation_deployment"
    scene_dir = repo_root / "scenes" / "ur5_2f_test"
    scene_dir.mkdir(parents=True)

    repo_mesh = (
        repo_root
        / "assets"
        / "environment"
        / "sorting_bin_description"
        / "meshes"
        / "sorting_bin.stl"
    )
    repo_mesh.parent.mkdir(parents=True)
    repo_mesh.write_bytes(b"solid sorting_bin\nendsolid sorting_bin\n")

    stale_alias_mesh = (
        tmp_path
        / "workcell_ws"
        / "src"
        / "scenes"
        / "ur5_2f_test"
        / "assets"
        / "environment"
        / "sorting_bin_description"
        / "meshes"
        / "sorting_bin.stl"
    )
    assert not stale_alias_mesh.exists()

    resolved, _source_root, _dest_rel, warning = exporter._resolve_local_mesh_uri(
        str(stale_alias_mesh), scene_dir, repo_root
    )

    assert warning is None
    assert resolved == repo_mesh.resolve()


def test_overlay_resolver_keeps_genuine_missing_absolute_mesh_blocking(tmp_path):
    repo_root = tmp_path / "easy_manipulation_deployment"
    scene_dir = repo_root / "scenes" / "ur5_2f_test"
    scene_dir.mkdir(parents=True)
    missing_mesh = tmp_path / "outside" / "missing.stl"

    resolved, _source_root, _dest_rel, warning = exporter._resolve_local_mesh_uri(
        str(missing_mesh), scene_dir, repo_root
    )

    assert resolved is None
    assert warning
