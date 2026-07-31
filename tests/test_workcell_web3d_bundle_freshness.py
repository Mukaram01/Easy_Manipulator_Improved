import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT / "scripts/ensure_workcell_studio_web_bundle_fresh.py"
CMAKE = ROOT / "workcell_builder/workcell_builder/CMakeLists.txt"


def _fake_tools(tmp_path: Path, *, stale: bool = False) -> tuple[Path, Path]:
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    (bin_dir / "node").write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    log = tmp_path / "npm.log"
    state = tmp_path / "stale"
    if stale:
        state.touch()
    npm = bin_dir / "npm"
    npm.write_text(
        "#!/bin/sh\n"
        'echo "$*" >> "$NPM_LOG"\n'
        'if [ "$1 $2" = "run check:stale-bundle" ] && [ -f "$STALE_STATE" ]; then exit 1; fi\n'
        'if [ "$1 $2" = "run build:web3d" ]; then /bin/rm -f "$STALE_STATE"; fi\n'
        "exit 0\n",
        encoding="utf-8",
    )
    npm.chmod(0o755)
    (bin_dir / "node").chmod(0o755)
    return bin_dir, log


def _run(tmp_path: Path, *, stale: bool = False) -> tuple[subprocess.CompletedProcess[str], list[str]]:
    viewer = tmp_path / "viewer"
    viewer.mkdir()
    bin_dir, log = _fake_tools(tmp_path, stale=stale)
    env = {**os.environ, "PATH": str(bin_dir), "NPM_LOG": str(log), "STALE_STATE": str(tmp_path / "stale")}
    result = subprocess.run(
        [sys.executable, str(HELPER), "--viewer-root", str(viewer)],
        text=True,
        capture_output=True,
        env=env,
        check=False,
    )
    return result, log.read_text(encoding="utf-8").splitlines()


def test_current_bundle_is_a_no_op(tmp_path):
    result, calls = _run(tmp_path)
    assert result.returncode == 0, result.stderr
    assert calls == ["ls --ignore-scripts --depth=0", "run check:stale-bundle"]
    assert "already current" in result.stdout


def test_stale_bundle_is_built_and_verified(tmp_path):
    result, calls = _run(tmp_path, stale=True)
    assert result.returncode == 0, result.stderr
    assert calls == [
        "ls --ignore-scripts --depth=0",
        "run check:stale-bundle",
        "run build:web3d",
        "run check:stale-bundle",
    ]


def test_missing_npm_has_actionable_error(tmp_path):
    viewer = tmp_path / "viewer"
    viewer.mkdir()
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    node = bin_dir / "node"
    node.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    node.chmod(0o755)
    result = subprocess.run(
        [sys.executable, str(HELPER), "--viewer-root", str(viewer)],
        text=True,
        capture_output=True,
        env={**os.environ, "PATH": str(bin_dir)},
        check=False,
    )
    assert result.returncode == 1
    assert "'npm' was not found on PATH" in result.stderr
    assert "Install Node.js and npm" in result.stderr


def test_cmake_wires_freshness_without_tracking_node_modules():
    cmake = CMAKE.read_text(encoding="utf-8")
    assert "add_custom_target(ensure_workcell_web3d_bundle_fresh" in cmake
    assert "add_dependencies(workcell_builder ensure_workcell_web3d_bundle_fresh)" in cmake
    target_block = cmake.split("add_custom_command(", 1)[1].split("add_custom_target", 1)[0]
    assert "node_modules" not in target_block
    assert "workcell_web3d_bundle_fresh.stamp" in cmake
