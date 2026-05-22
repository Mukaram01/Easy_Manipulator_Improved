from __future__ import annotations

import os
import shutil
from pathlib import Path
from typing import Optional

REPO_MARKERS = ("scripts", "workcell_builder", "scenes", "assets", "package.xml")


def _looks_like_repo_root(path: Path) -> bool:
    return all((path / marker).exists() for marker in REPO_MARKERS)


def resolve_repo_root(start: Path | None = None) -> Path:
    """Resolve repository root by walking upward from start (or this file)."""
    origin = (start or Path(__file__).resolve()).resolve()
    scan_from = origin if origin.is_dir() else origin.parent

    for candidate in (scan_from, *scan_from.parents):
        if _looks_like_repo_root(candidate):
            return candidate

    # Conservative fallback for script-in-repo execution.
    return Path(__file__).resolve().parents[1]


def resolve_workspace_root(repo_root: Path) -> Optional[Path]:
    """Find a colcon-style workspace root without assuming workspace name."""
    env_candidates = [
        os.environ.get("WORKCELL_WS"),
        os.environ.get("WORKSPACE"),
        os.environ.get("COLCON_CURRENT_PREFIX"),
    ]
    for raw in env_candidates:
        if not raw:
            continue
        p = Path(raw).expanduser().resolve()
        root = p if (p / "src").is_dir() else p.parent
        if (root / "src").is_dir():
            return root

    cwd = Path.cwd().resolve()
    for candidate in (cwd, *cwd.parents):
        if (candidate / "src").is_dir():
            return candidate

    for candidate in (repo_root.parent, *repo_root.parent.parents):
        if (candidate / "src").is_dir():
            return candidate

    return None


def resolve_install_setup(workspace_root: Path | None, repo_root: Path) -> Optional[Path]:
    candidates = []
    if workspace_root is not None:
        candidates.append(workspace_root / "install" / "setup.bash")
    candidates += [
        repo_root / "install" / "setup.bash",
        repo_root.parent / "install" / "setup.bash",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def resolve_workcell_builder_executable(workspace_root: Path | None, repo_root: Path) -> Optional[Path]:
    """Resolve workcell_builder executable and emit searched-path diagnostics."""
    candidates: list[Path] = []
    if workspace_root is not None:
        candidates.extend(
            [
                workspace_root / "install" / "workcell_builder" / "lib" / "workcell_builder" / "workcell_builder",
                workspace_root / "build" / "workcell_builder" / "workcell_builder",
            ]
        )

    candidates.extend(
        [
            repo_root / "install" / "workcell_builder" / "lib" / "workcell_builder" / "workcell_builder",
            repo_root / "build" / "workcell_builder" / "workcell_builder",
        ]
    )

    system_exe = shutil.which("workcell_builder")
    if system_exe:
        candidates.append(Path(system_exe))

    for candidate in candidates:
        if candidate.exists() and os.access(candidate, os.X_OK):
            return candidate

    search_log = "\n".join(f"- {p}" for p in candidates) if candidates else "- <none>"
    print("workcell_builder executable not found. searched:\n" + search_log)
    return None


def resolve_scenes_root(repo_root: Path) -> Path:
    canonical = repo_root / "scenes"
    legacy = repo_root / "workcell_builder" / "scenes"
    if canonical.exists():
        return canonical
    if legacy.exists():
        return legacy
    return canonical
