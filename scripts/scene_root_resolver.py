from __future__ import annotations

from pathlib import Path

from scripts.workcell_studio_path_resolver import resolve_scenes_root


def resolve_scene_root(repo_root: Path) -> Path:
    """Backward-compatible scene root resolver."""
    return resolve_scenes_root(repo_root)
