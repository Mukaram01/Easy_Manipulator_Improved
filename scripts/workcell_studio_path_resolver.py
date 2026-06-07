from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path
from typing import Any

REPO_MARKERS = ("scripts", "workcell_builder", "scenes")
_LAST: dict[str, Any] = {}


def _looks_like_repo_root(path: Path) -> bool:
    return all((path / marker).exists() for marker in REPO_MARKERS)


def resolve_repo_root(start: Path | None = None, explicit_repo_root: Path | str | None = None) -> Path:
    if explicit_repo_root:
        repo = Path(explicit_repo_root).expanduser().resolve()
        if not _looks_like_repo_root(repo):
            raise FileNotFoundError(f"explicit repo root is not valid: {repo}")
        return repo
    origin = (start or Path.cwd()).resolve()
    scan_from = origin if origin.is_dir() else origin.parent
    for candidate in (scan_from, *scan_from.parents):
        if _looks_like_repo_root(candidate):
            return candidate
    fallback = Path(__file__).resolve().parents[1]
    if _looks_like_repo_root(fallback):
        return fallback
    raise FileNotFoundError("could not resolve repository root")


def resolve_workspace_root(repo_root: Path, explicit_workspace_root: Path | str | None = None) -> Path | None:
    if explicit_workspace_root:
        ws = Path(explicit_workspace_root).expanduser().resolve()
        return ws
    # infer <workspace>/src/easy_manipulation_deployment
    if repo_root.parent.name == "src":
        return repo_root.parent.parent
    for candidate in (repo_root.parent, *repo_root.parent.parents):
        if (candidate / "src").is_dir() and repo_root.is_relative_to(candidate / "src"):
            return candidate
    return None


def resolve_install_setup(workspace_root: Path | None) -> Path | None:
    if workspace_root is None:
        return None
    setup = workspace_root / "install" / "setup.bash"
    return setup if setup.exists() else None


def workcell_builder_executable_candidates(workspace_root: Path | None) -> list[Path]:
    candidates: list[Path] = []

    def add_candidate(candidate: Path | str | None) -> None:
        if not candidate:
            return
        candidate_path = Path(candidate).expanduser()
        if candidate_path not in candidates:
            candidates.append(candidate_path)

    add_candidate(os.environ.get("WORKCELL_BUILDER_EXECUTABLE"))

    if workspace_root:
        add_candidate(
            workspace_root
            / "install"
            / "workcell_builder"
            / "lib"
            / "workcell_builder"
            / "workcell_builder"
        )
        add_candidate(workspace_root / "install" / "workcell_builder" / "bin" / "workcell_builder")
        add_candidate(workspace_root / "install" / "bin" / "workcell_builder")

    add_candidate(shutil.which("workcell_builder"))

    ros2 = shutil.which("ros2")
    if ros2:
        try:
            result = subprocess.run(
                [ros2, "pkg", "prefix", "workcell_builder"],
                check=False,
                capture_output=True,
                text=True,
                timeout=5,
            )
        except (OSError, subprocess.SubprocessError):
            result = None
        if result and result.returncode == 0:
            prefix_text = result.stdout.strip().splitlines()
            if prefix_text:
                prefix = Path(prefix_text[0]).expanduser()
                add_candidate(prefix / "lib" / "workcell_builder" / "workcell_builder")
                add_candidate(prefix / "bin" / "workcell_builder")
                add_candidate(prefix / ".." / "bin" / "workcell_builder")

    return candidates


def _record_executable_search(searched: list[Path], resolved: Path | None) -> None:
    _LAST["searched_executable_paths"] = [str(p) for p in searched]
    _LAST["workcell_builder_executable_candidates"] = [
        {
            "path": str(p),
            "exists": p.exists(),
            "is_file": p.is_file(),
            "executable": os.access(p, os.X_OK),
            "selected": resolved is not None and p == resolved,
        }
        for p in searched
    ]
    _LAST["resolved_workcell_builder_executable"] = str(resolved) if resolved else None


def resolve_workcell_builder_executable(workspace_root: Path | None, explicit_executable: Path | str | None = None) -> Path | None:
    searched: list[Path] = []
    if explicit_executable:
        exe = Path(explicit_executable).expanduser().resolve()
        searched.append(exe)
        resolved = exe if exe.exists() and os.access(exe, os.X_OK) else None
        _record_executable_search(searched, resolved)
        return resolved

    _LAST.pop("explicit_executable", None)
    _LAST.pop("explicit_executable_exists", None)
    _LAST.pop("explicit_executable_is_executable", None)
    searched.extend(workcell_builder_executable_candidates(workspace_root))
    for p in searched:
        if p.exists() and os.access(p, os.X_OK):
            _record_executable_search(searched, p)
            return p
    _record_executable_search(searched, None)
    return None


def resolve_scenes_root(repo_root: Path) -> Path:
    return repo_root / "scenes"


def describe_resolution() -> dict[str, Any]:
    return dict(_LAST)
