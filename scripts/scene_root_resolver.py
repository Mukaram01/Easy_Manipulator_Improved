from __future__ import annotations

from pathlib import Path


def resolve_scene_root(repo_root: Path) -> Path:
    """Resolve the canonical scene root for script utilities.

    Priority:
    1) <repo>/scenes
    2) <repo>/workcell_builder/scenes (legacy fallback) only when canonical does not exist.
    """

    canonical = repo_root / "scenes"
    legacy = repo_root / "workcell_builder" / "scenes"
    if canonical.exists():
        return canonical
    if legacy.exists():
        return legacy
    return canonical
