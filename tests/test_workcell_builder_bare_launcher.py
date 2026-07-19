from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[1]
CMAKE = ROOT / "workcell_builder" / "workcell_builder" / "CMakeLists.txt"
LAUNCHER = ROOT / "workcell_builder" / "workcell_builder" / "scripts" / "workcell_builder"


def test_cmake_installs_bare_launcher_into_prefix_bin() -> None:
    cmake = CMAKE.read_text(encoding="utf-8")
    assert "install(PROGRAMS\n  scripts/workcell_builder\n  DESTINATION bin)" in cmake
    assert "DESTINATION lib/${PROJECT_NAME}" in cmake


def test_launcher_source_is_executable_for_symlink_install() -> None:
    assert LAUNCHER.stat().st_mode & 0o111


def test_launcher_resolves_package_binary_and_forwards_arguments() -> None:
    launcher = LAUNCHER.read_text(encoding="utf-8")
    assert "${prefix_dir}/lib/workcell_builder/workcell_builder" in launcher
    assert 'exec "${binary}" "$@"' in launcher
    assert "ros2 run" not in launcher
    assert "/home/user" not in launcher
    assert "/home/ubuntu" not in launcher
    assert "workcell_ws" not in launcher


def test_launcher_executes_sibling_installed_binary(tmp_path: Path) -> None:
    prefix = tmp_path / "install" / "workcell_builder"
    bin_dir = prefix / "bin"
    app_dir = prefix / "lib" / "workcell_builder"
    bin_dir.mkdir(parents=True)
    app_dir.mkdir(parents=True)

    installed_launcher = bin_dir / "workcell_builder"
    shutil.copyfile(LAUNCHER, installed_launcher)
    installed_launcher.chmod(0o755)

    installed_app = app_dir / "workcell_builder"
    installed_app.write_text(
        "#!/usr/bin/env bash\nprintf '%s\\n' \"$@\"\n",
        encoding="utf-8",
    )
    installed_app.chmod(0o755)

    result = subprocess.run(
        [str(installed_launcher), "first", "two words", "--flag=value"],
        check=False,
        capture_output=True,
        text=True,
        env={**os.environ, "PATH": os.environ.get("PATH", "")},
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == ["first", "two words", "--flag=value"]


def test_launcher_reports_missing_installed_binary(tmp_path: Path) -> None:
    bin_dir = tmp_path / "prefix" / "bin"
    bin_dir.mkdir(parents=True)
    installed_launcher = bin_dir / "workcell_builder"
    shutil.copyfile(LAUNCHER, installed_launcher)
    installed_launcher.chmod(0o755)

    result = subprocess.run(
        [str(installed_launcher)],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 127
    assert "missing or not executable" in result.stderr
    assert "Rebuild the workcell_builder package" in result.stderr
