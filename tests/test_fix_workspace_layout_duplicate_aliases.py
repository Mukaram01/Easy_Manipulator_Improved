import os
import shutil
import subprocess
from pathlib import Path


def write_pkg(path: Path, name: str) -> None:
    path.mkdir(parents=True, exist_ok=True)
    (path / "package.xml").write_text(
        f"""<package format=\"3\"><name>{name}</name><version>0.0.0</version><description>test</description><maintainer email=\"a@b.c\">t</maintainer><license>MIT</license></package>\n""",
        encoding="utf-8",
    )


def make_repo(tmp_path: Path) -> Path:
    repo = tmp_path / "repo"
    repo.mkdir()
    shutil.copytree("scripts", repo / "scripts")
    write_pkg(repo, "easy_manipulation_deployment")
    write_pkg(repo / "assets" / "ur_description", "ur_description")
    write_pkg(repo / "scenes" / "ur5_2f_test", "ur5_2f_test")
    return repo


def run_layout(repo: Path, workspace: Path) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env["WORKSPACE_ROOT"] = str(workspace)
    env["HOME"] = str(workspace.parent / "home")
    return subprocess.run(
        ["bash", str(repo / "scripts" / "fix_workspace_layout.sh"), "--layout-only"],
        cwd=repo,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )


def test_repo_linked_under_src_removes_redundant_aliases(tmp_path: Path) -> None:
    repo = make_repo(tmp_path)
    ws = tmp_path / "ws"
    src = ws / "src"
    src.mkdir(parents=True)
    (src / "easy_manipulation_deployment").symlink_to(repo, target_is_directory=True)
    (src / "assets").symlink_to(repo / "assets", target_is_directory=True)
    (src / "scenes").symlink_to(repo / "scenes", target_is_directory=True)

    result = run_layout(repo, ws)

    assert not (src / "assets").exists() and not (src / "assets").is_symlink()
    assert not (src / "scenes").exists() and not (src / "scenes").is_symlink()
    assert "assets alias: skipped" in result.stdout
    assert "duplicate package check: OK" in result.stdout


def test_repo_outside_src_still_creates_aliases(tmp_path: Path) -> None:
    repo = make_repo(tmp_path)
    ws = tmp_path / "ws"
    (ws / "src").mkdir(parents=True)

    run_layout(repo, ws)

    assert (ws / "src" / "assets").resolve() == (repo / "assets").resolve()
    assert (ws / "src" / "scenes").resolve() == (repo / "scenes").resolve()


def test_real_directories_are_never_deleted(tmp_path: Path) -> None:
    repo = make_repo(tmp_path)
    ws = tmp_path / "ws"
    src = ws / "src"
    src.mkdir(parents=True)
    (src / "easy_manipulation_deployment").symlink_to(repo, target_is_directory=True)
    real_assets = src / "assets"
    real_assets.mkdir()
    marker = real_assets / "keep.txt"
    marker.write_text("keep", encoding="utf-8")

    run_layout(repo, ws)

    assert marker.read_text(encoding="utf-8") == "keep"
    assert real_assets.is_dir() and not real_assets.is_symlink()


def test_repeated_execution_is_idempotent(tmp_path: Path) -> None:
    repo = make_repo(tmp_path)
    ws = tmp_path / "ws"
    src = ws / "src"
    src.mkdir(parents=True)
    (src / "easy_manipulation_deployment").symlink_to(repo, target_is_directory=True)
    (src / "assets").symlink_to(repo / "assets", target_is_directory=True)
    (src / "scenes").symlink_to(repo / "scenes", target_is_directory=True)

    first = run_layout(repo, ws)
    second = run_layout(repo, ws)

    assert "duplicate package check: OK" in first.stdout
    assert "duplicate package check: OK" in second.stdout
    assert not (src / "assets").exists() and not (src / "assets").is_symlink()
    assert not (src / "scenes").exists() and not (src / "scenes").is_symlink()
