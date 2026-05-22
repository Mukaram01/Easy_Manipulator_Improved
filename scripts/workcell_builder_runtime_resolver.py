#!/usr/bin/env python3
from __future__ import annotations

import os
import shutil
from pathlib import Path


def _uniq(paths: list[Path]) -> list[Path]:
    out: list[Path] = []
    seen: set[str] = set()
    for path in paths:
        key = str(path)
        if key in seen:
            continue
        seen.add(key)
        out.append(path)
    return out


def resolve_runtime_paths(
    *,
    script_path: Path,
    repo_root_arg: Path | None,
    workspace_root_arg: Path | None,
    workcell_builder_executable_arg: str | None,
) -> dict[str, object]:
    repo_root = (repo_root_arg or script_path.resolve().parents[1]).resolve()
    workspace_root = (workspace_root_arg or repo_root).resolve()

    searched_setup_paths = _uniq([
        workspace_root / "install/setup.bash",
        repo_root / "install/setup.bash",
    ])
    setup_path = next((p for p in searched_setup_paths if p.is_file()), None)

    searched_executable_paths = _uniq([
        Path(workcell_builder_executable_arg).expanduser() if workcell_builder_executable_arg else workspace_root / "install/workcell_builder/lib/workcell_builder/workcell_builder",
        repo_root / "install/workcell_builder/lib/workcell_builder/workcell_builder",
    ])

    executable_path: Path | None = None
    executable_source = "searched_paths"
    for candidate in searched_executable_paths:
        if candidate.is_file():
            executable_path = candidate.resolve()
            break

    if executable_path is None and workcell_builder_executable_arg:
        which_match = shutil.which(workcell_builder_executable_arg)
        if which_match:
            executable_path = Path(which_match).resolve()
            executable_source = "path_lookup"

    if executable_path is None and not workcell_builder_executable_arg:
        which_match = shutil.which("workcell_builder")
        if which_match:
            executable_path = Path(which_match).resolve()
            executable_source = "path_lookup"

    return {
        "repo_root": repo_root,
        "workspace_root": workspace_root,
        "setup_path": setup_path,
        "executable_path": executable_path,
        "executable_source": executable_source,
        "searched_setup_paths": [str(p) for p in searched_setup_paths],
        "searched_executable_paths": [str(p) for p in searched_executable_paths],
        "ci": str(os.environ.get("CI", "")).strip().lower() in {"1", "true", "yes", "on"},
    }
