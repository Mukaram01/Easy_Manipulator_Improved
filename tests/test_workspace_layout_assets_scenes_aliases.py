import os
import shlex
import subprocess
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
VERIFY_SCRIPT = REPO_ROOT / "scripts" / "verify_workspace_discovery.sh"


def _write_package_xml(path: Path, name: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        f"<package format='3'><name>{name}</name><version>0.0.0</version>"
        "<description>test</description>"
        "<maintainer email='test@example.com'>Test</maintainer>"
        "<license>Apache-2.0</license></package>\n",
        encoding="utf-8",
    )


def _make_colcon_stub(bin_dir: Path, rows: list[tuple[str, Path]]) -> None:
    lines = [
        "#!/usr/bin/env bash",
        "set -euo pipefail",
        "if [[ ${1:-} != 'list' ]]; then exit 2; fi",
    ]
    lines.extend(
        f"printf '%s %s\\n' {shlex.quote(name)} {shlex.quote(str(path))}"
        for name, path in rows
    )
    colcon = bin_dir / "colcon"
    colcon.write_text("\n".join(lines) + "\n", encoding="utf-8")
    colcon.chmod(0o755)


def _run_verify(
    tmp_path: Path, rows: list[tuple[str, Path]]
) -> subprocess.CompletedProcess[str]:
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    _make_colcon_stub(bin_dir, rows)
    env = os.environ.copy()
    env["WORKSPACE_ROOT"] = str(tmp_path / "ws")
    env["PATH"] = f"{bin_dir}{os.pathsep}{env.get('PATH', '')}"
    return subprocess.run(
        [str(VERIFY_SCRIPT)],
        cwd=REPO_ROOT,
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )


def _create_canonical_repo(ws: Path) -> Path:
    repo = ws / "src" / "easy_manipulation_deployment"
    (repo / "assets").mkdir(parents=True)
    _write_package_xml(repo / "scenes" / "ur5_2f_test" / "package.xml", "ur5_2f_test")
    _write_package_xml(repo / "workcell_builder" / "package.xml", "workcell_builder")
    return repo


def test_fix_workspace_layout_uses_assets_and_scenes_aliases():
    text = Path('scripts/fix_workspace_layout.sh').read_text(encoding='utf-8')
    assert 'ensure_workspace_alias "assets"' in text
    assert 'ensure_workspace_alias "scenes"' in text
    assert 'Removing legacy asset package symlink:' in text
    assert 'Duplicate package discovery detected' in text
    assert 'Workspace layout summary' in text
    assert 'src/assets ->' in text
    assert 'src/scenes ->' in text
    assert 'Exposed ${#EXPOSED_PACKAGES[@]}' not in text


def test_fix_workspace_layout_no_per_package_asset_symlink_creation():
    text = Path('scripts/fix_workspace_layout.sh').read_text(encoding='utf-8')
    assert 'find "$REPO_DIR/assets" -name package.xml' in text  # still used for discovery checks
    assert 'Linking repository package' not in text


def test_verify_workspace_discovery_accepts_canonical_layout_without_aliases(tmp_path):
    ws = tmp_path / "ws"
    repo = _create_canonical_repo(ws)
    result = _run_verify(
        tmp_path,
        [
            ("easy_manipulation_deployment", repo),
            ("workcell_builder", repo / "workcell_builder"),
        ],
    )

    assert result.returncode == 0, result.stderr
    output = result.stdout + result.stderr
    assert "Missing workspace alias" not in output
    assert str(ws / "src" / "assets") not in output
    assert str(ws / "src" / "scenes") not in output
    assert not (ws / "src" / "assets").exists()
    assert not (ws / "src" / "scenes").exists()


def test_verify_workspace_discovery_rejects_missing_canonical_content(tmp_path):
    ws = tmp_path / "ws"
    repo = ws / "src" / "easy_manipulation_deployment"
    repo.mkdir(parents=True)
    _write_package_xml(repo / "workcell_builder" / "package.xml", "workcell_builder")
    result = _run_verify(
        tmp_path,
        [
            ("easy_manipulation_deployment", repo),
            ("workcell_builder", repo / "workcell_builder"),
        ],
    )

    assert result.returncode != 0
    assert "Missing canonical layout/content" in result.stderr
    assert str(repo / "assets") in result.stderr or str(repo / "scenes") in result.stderr


def test_verify_workspace_discovery_rejects_duplicate_packages(tmp_path):
    ws = tmp_path / "ws"
    repo = _create_canonical_repo(ws)
    duplicate_repo = ws / "src" / "duplicate_easy_manipulation_deployment"
    duplicate_repo.mkdir(parents=True)
    result = _run_verify(
        tmp_path,
        [
            ("easy_manipulation_deployment", repo),
            ("easy_manipulation_deployment", duplicate_repo),
            ("workcell_builder", repo / "workcell_builder"),
        ],
    )

    assert result.returncode != 0
    assert "Duplicate package discovered" in result.stderr


def test_verify_workspace_discovery_static_markers_present():
    text = Path('scripts/verify_workspace_discovery.sh').read_text(encoding='utf-8')
    assert 'CANONICAL_REPO="$SRC_DIR/easy_manipulation_deployment"' in text
    assert 'canonical repository layout at $CANONICAL_REPO' in text
    assert 'legacy workspace aliases at $SRC_DIR/assets and $SRC_DIR/scenes' in text
    assert 'Missing workspace layout: expected canonical repo at $CANONICAL_REPO' in text
    assert 'Duplicate package discovered' in text
    assert 'required=(easy_manipulation_deployment workcell_builder)' in text


def test_curated_asset_tokens_present():
    txt = Path('workcell_builder/workcell_builder/config/asset_profiles/environment_assets.json').read_text(encoding='utf-8')
    assert "table_small" in txt and "pick_box" in txt


def test_portable_bundle_markers_present():
    blob = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    for m in ['Export Scene Bundle','Import Scene Bundle','Portable Scene Bundle','Bundle Validation Status','Imported Scene Ready','Exported Scene Archive']:
        assert m in blob
