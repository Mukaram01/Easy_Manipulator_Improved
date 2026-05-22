from __future__ import annotations

from pathlib import Path

try:
    from scripts.workcell_studio_path_resolver import resolve_scenes_root
except ModuleNotFoundError:
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from workcell_studio_path_resolver import resolve_scenes_root


def resolve_scene_root(repo_root: Path) -> Path:
    """Backward-compatible scene root resolver."""
    return resolve_scenes_root(repo_root)
