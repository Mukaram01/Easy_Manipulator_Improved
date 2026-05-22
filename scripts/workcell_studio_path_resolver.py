from __future__ import annotations

import os
import shutil
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


def resolve_workcell_builder_executable(workspace_root: Path | None, explicit_executable: Path | str | None = None) -> Path | None:
    searched: list[Path] = []
    if explicit_executable:
        exe = Path(explicit_executable).expanduser().resolve()
        searched.append(exe)
        _LAST["searched_executable_paths"] = [str(p) for p in searched]
        return exe if exe.exists() and os.access(exe, os.X_OK) else None

    path_hit = shutil.which("workcell_builder")
    if path_hit:
        p = Path(path_hit)
        searched.append(p)
        if p.exists() and os.access(p, os.X_OK):
            _LAST["searched_executable_paths"] = [str(x) for x in searched]
            return p

    if workspace_root:
        searched.extend([
            workspace_root / "install" / "workcell_builder" / "lib" / "workcell_builder" / "workcell_builder",
            workspace_root / "install" / "workcell_builder" / "bin" / "workcell_builder",
            workspace_root / "install" / "bin" / "workcell_builder",
        ])
    _LAST["searched_executable_paths"] = [str(p) for p in searched]
    for p in searched:
        if p.exists() and os.access(p, os.X_OK):
            return p
    return None


def resolve_scenes_root(repo_root: Path) -> Path:
    return repo_root / "scenes"


def describe_resolution() -> dict[str, Any]:
    return dict(_LAST)
