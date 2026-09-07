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
    """Resolve mesh paths while tolerating stale scene-local asset aliases.

    Workcell Studio's editable layout can legitimately reference repository
    assets such as ``assets/environment/...`` or ``package://...`` URIs.  An
    authoring-session overlay may contain a stale absolute alias below the scene
    directory, for example::

        <repo>/scenes/ur5_2f_test/assets/environment/...
        <repo>/scenes/ur5_2f_test/assets/realsense2_description/...

    Repository assets actually live below ``<repo>/assets`` and package assets
    can be nested inside category directories.  Recover only an unambiguous
    existing repository asset before declaring the overlay invalid.  The
    canonical resolver still enforces allowed roots and supported mesh types.
    """
    scene_dir = Path(scene_dir)
    repo_root = Path(repo_root)
    resolved = _ORIGINAL_RESOLVE_LOCAL_MESH_URI(uri, scene_dir, repo_root)
    if resolved[0] is not None:
        return resolved

    raw = Path(str(uri))
    if not raw.is_absolute():
        return resolved

    parts = raw.parts
    asset_indexes = [index for index, part in enumerate(parts) if part == "assets"]
    for index in reversed(asset_indexes):
        portable_path = Path(*parts[index:])
        portable = portable_path.as_posix()
        recovered = _ORIGINAL_RESOLVE_LOCAL_MESH_URI(portable, scene_dir, repo_root)
        if recovered[0] is not None:
            return recovered

        # package:// URIs can be normalized by the live authoring overlay into
        # ``.../scene/assets/<package>/<tail>`` while the checked-in package is
        # actually nested below a repository category, e.g.
        # ``assets/environment/realsense2_description/<tail>``.  Resolve this
        # only when exactly one matching package asset exists; never guess when
        # repository contents are ambiguous.
        rel_parts = portable_path.parts
        if len(rel_parts) < 3 or rel_parts[0] != "assets":
            continue
        package_name = rel_parts[1]
        package_tail = Path(*rel_parts[2:])
        assets_root = repo_root / "assets"
        candidates = []
        if assets_root.is_dir():
            for package_dir in assets_root.rglob(package_name):
                if not package_dir.is_dir() or package_dir.name != package_name:
                    continue
                candidate = package_dir / package_tail
                if candidate.is_file():
                    candidates.append(candidate.resolve())
        unique_candidates = sorted(set(candidates))
        if len(unique_candidates) == 1:
            recovered = _ORIGINAL_RESOLVE_LOCAL_MESH_URI(
                str(unique_candidates[0]), scene_dir, repo_root
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
