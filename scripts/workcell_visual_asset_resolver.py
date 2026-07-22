#!/usr/bin/env python3
from __future__ import annotations
from dataclasses import dataclass, asdict
from pathlib import Path
import os
import xml.etree.ElementTree as ET

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


def _read_package_xml_name(package_xml: Path) -> str | None:
    try:
        name = ET.parse(package_xml).getroot().findtext('name')
    except ET.ParseError:
        return None
    name = (name or '').strip()
    return name or None


def _package_discovery_roots(repo_root: Path, extra_roots: list[Path] | None = None) -> list[Path]:
    roots = [
        repo_root,
        repo_root / 'assets',
        repo_root / 'scenes',
        repo_root / 'workcell_builder/workcell_builder/assets',
    ]
    if extra_roots:
        roots.extend(extra_roots)
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if prefix:
            roots.append(Path(prefix) / 'share')
    roots.append(Path('/opt/ros/humble/share'))

    out: list[Path] = []
    seen: set[str] = set()
    for root in roots:
        try:
            resolved = root.resolve()
        except OSError:
            resolved = root
        key = str(resolved)
        if key not in seen:
            seen.add(key)
            out.append(resolved)
    return out


def discover_package_map(repo_root: Path, extra_roots: list[Path] | None = None) -> dict[str, Path]:
    """Discover ROS package names by package.xml identity, including nested assets.

    The returned map is keyed by the package name declared inside package.xml,
    not by the directory basename.  That keeps package:// URI resolution aligned
    with ROS package identity and allows repository assets to live below nested
    roots such as assets/end_effectors/<vendor>/<description_package>.
    """
    out: dict[str, Path] = {}
    for root in _package_discovery_roots(repo_root, extra_roots):
        if not root.exists():
            continue
        for pkg_xml in root.rglob('package.xml'):
            pkg_dir = pkg_xml.parent.resolve()
            package_name = _read_package_xml_name(pkg_xml) or pkg_dir.name
            out.setdefault(package_name, pkg_dir)
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
        package_map = package_map or discover_package_map(repo_root)
        if pkg in package_map:
            p = package_map[pkg] / rel
            if p.exists():
                return found(p, 'package_uri')
            return MeshResolution(uri, str(p.resolve()), False, ext, 'package_uri', f'missing package mesh file: {original}')
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
