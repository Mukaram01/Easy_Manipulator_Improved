#!/usr/bin/env python3
"""Portable Workcell Studio web-scene exporter entry point.

The implementation remains in ``export_workcell_studio_web_scene_impl.py``.
This wrapper preserves the public Python/CLI API and applies the portable visual
artifact contract to every returned web-scene payload before it is written.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

import export_workcell_studio_web_scene_impl as _impl
import workcell_studio_visual_artifact_portability as _portability

_EXPORTED_NAMES = {
    name for name in _impl.__dict__ if not name.startswith("__") and name != "main"
}
for _name in _EXPORTED_NAMES:
    globals()[_name] = getattr(_impl, _name)

_ORIGINAL_BUILD_WEB_SCENE = _impl.build_web_scene
_ORIGINAL_MAIN = _impl.main
_ORIGINAL_RESOLVE_LOCAL_MESH_URI = _impl._resolve_local_mesh_uri


def _resolve_local_mesh_uri(uri: str, scene_dir: Path, repo_root: Path):
    """Resolve mesh paths while tolerating the workspace ``src/scenes`` alias.

    Workcell Studio's editable layout can legitimately reference repository
    assets such as ``assets/environment/...``.  After a generate/simulate
    refresh, an unsaved authoring-session item may carry the same asset as a
    stale absolute path rooted below the workspace scene alias, for example::

        <workspace>/src/scenes/ur5_2f_test/assets/environment/...

    That absolute leaf does not exist because repository assets live at
    ``<repo>/assets``.  Re-run the canonical resolver with the portable
    ``assets/...`` suffix before declaring the authoring overlay invalid.  The
    canonical resolver still enforces allowed roots and supported mesh types.
    """
    resolved = _ORIGINAL_RESOLVE_LOCAL_MESH_URI(uri, Path(scene_dir), Path(repo_root))
    if resolved[0] is not None:
        return resolved

    raw = Path(str(uri))
    if not raw.is_absolute():
        return resolved

    parts = raw.parts
    asset_indexes = [index for index, part in enumerate(parts) if part == "assets"]
    for index in reversed(asset_indexes):
        portable = Path(*parts[index:]).as_posix()
        recovered = _ORIGINAL_RESOLVE_LOCAL_MESH_URI(
            portable, Path(scene_dir), Path(repo_root)
        )
        if recovered[0] is not None:
            return recovered

    return resolved


def _sync_impl_globals() -> None:
    """Keep monkeypatching and legacy imports compatible with the old module."""
    for name in _EXPORTED_NAMES:
        if name in globals() and name != "build_web_scene":
            setattr(_impl, name, globals()[name])


def build_web_scene(
    scene_dir: Path,
    *,
    stage_assets: bool = False,
    output_path: Optional[Path] = None,
    allow_incomplete_preview: bool = False,
    authoring_session_overlay: Optional[Path] = None,
):
    _sync_impl_globals()
    payload = _ORIGINAL_BUILD_WEB_SCENE(
        scene_dir,
        stage_assets=stage_assets,
        output_path=output_path,
        allow_incomplete_preview=allow_incomplete_preview,
        authoring_session_overlay=authoring_session_overlay,
    )
    _portability.normalize_web_scene_payload(
        payload,
        scene_dir=Path(scene_dir),
        output_path=output_path,
        stage_assets=stage_assets,
    )
    return payload


def main(argv=None) -> int:
    _sync_impl_globals()
    previous = _impl.build_web_scene
    _impl.build_web_scene = build_web_scene
    try:
        return int(_ORIGINAL_MAIN(argv))
    finally:
        _impl.build_web_scene = previous


if __name__ == "__main__":
    raise SystemExit(main())
