#!/usr/bin/env python3
from __future__ import annotations
from dataclasses import dataclass, asdict
from pathlib import Path
import os

MESH_EXTS = {'.stl','.dae','.obj','.mesh'}

@dataclass
class MeshResolution:
    original_uri: str
    resolved_path: str
    exists: bool
    extension: str
    source_kind: str
    message: str = ''

    def to_dict(self):
        return asdict(self)


def discover_package_map(repo_root: Path, extra_roots: list[Path] | None = None) -> dict[str, Path]:
    roots = [repo_root, repo_root / 'assets', repo_root / 'scenes', repo_root / 'workcell_builder/workcell_builder/assets', Path('/opt/ros/humble/share')]
    if extra_roots:
        roots.extend(extra_roots)
    out: dict[str, Path] = {}
    for root in roots:
        if not root.exists():
            continue
        for pkg in root.rglob('package.xml'):
            out.setdefault(pkg.parent.name, pkg.parent)
    return out


def resolve_mesh_uri(uri: str, *, repo_root: Path, scene_dir: Path | None = None, package_map: dict[str, Path] | None = None, asset_catalog: set[str] | None = None) -> MeshResolution:
    original = (uri or '').strip().strip('"\'')
    ext = Path(original).suffix.lower()
    if not original:
        return MeshResolution(uri, '', False, ext, 'unresolved', 'empty mesh URI/path')

    def found(path: Path, kind: str):
        return MeshResolution(uri, str(path.resolve()), path.exists(), Path(path).suffix.lower(), kind, '' if path.exists() else f'missing {kind} path')

    if original.startswith('file://'):
        p = Path(original[7:])
        return found(p, 'file_uri')

    if original.startswith('package://') and '/' in original[10:]:
        pkg, rel = original[10:].split('/', 1)
        if package_map and pkg in package_map:
            p = package_map[pkg] / rel
            if p.exists():
                return found(p, 'package_uri')
        ros_share = Path('/opt/ros/humble/share') / pkg / rel
        if ros_share.exists():
            return found(ros_share, 'ros_share')
        return MeshResolution(uri, '', False, ext, 'unresolved', f'unresolved package URI: {original}')

    p = Path(original)
    if p.is_absolute():
        return found(p, 'absolute')

    candidates = []
    if scene_dir:
        candidates += [scene_dir / original, scene_dir / 'urdf' / original]
    candidates += [repo_root / original, repo_root / 'assets' / original, repo_root / 'workcell_builder/workcell_builder/assets' / original]
    for c in candidates:
        if c.exists():
            kind = 'relative_scene' if scene_dir and str(c).startswith(str(scene_dir)) else 'relative_repo'
            if '/assets/' in str(c):
                kind = 'asset_catalog'
            return found(c, kind)

    if asset_catalog:
        base = Path(original).name
        for x in asset_catalog:
            if Path(x).name == base:
                c = repo_root / x
                if c.exists():
                    return found(c, 'asset_catalog')

    return MeshResolution(uri, '', False, ext, 'unresolved', f'could not resolve mesh path: {original}')
