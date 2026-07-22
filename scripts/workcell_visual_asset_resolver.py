#!/usr/bin/env python3
from __future__ import annotations
from dataclasses import dataclass, asdict
from urllib.parse import unquote, urlparse
from pathlib import Path
import os
import xml.etree.ElementTree as ET

MESH_EXTS = {'.stl','.dae','.obj','.mesh'}
SUPPORTED_BROWSER_MESH_EXTS = {'.stl', '.dae', '.obj'}

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



def _merge_package_candidates(package: str, repo_root: Path, extra_roots: list[Path] | None = None, package_map: dict[str, Path] | None = None) -> list[Path]:
    """Return candidate package roots in Product View's authoritative order.

    Candidates are accepted only when their package.xml declares ``package``.
    A provided map is augmented with discovered roots instead of replacing
    discovery, so an incomplete non-empty map cannot mask repository assets.
    """
    roots: list[Path] = []
    seen: set[str] = set()

    def add(candidate: Path | None) -> None:
        if candidate is None:
            return
        try:
            resolved = candidate.resolve()
        except OSError:
            resolved = candidate
        if not resolved.exists():
            return
        pkg_xml = resolved / 'package.xml'
        if pkg_xml.is_file():
            identity = _read_package_xml_name(pkg_xml)
            if identity != package:
                return
        elif resolved.name != package:
            return
        key = str(resolved)
        if key not in seen:
            seen.add(key)
            roots.append(resolved)

    # 1. Repository asset packages should win over stale installed packages.
    repo_asset_roots = [repo_root / 'assets', repo_root / 'workcell_builder/workcell_builder/assets']
    for root in repo_asset_roots:
        if root.exists():
            add(root / package)
            for pkg_xml in root.rglob('package.xml'):
                add(pkg_xml.parent)

    # 2. Explicit/source-workspace roots, including any map passed by callers.
    if package_map:
        add(package_map.get(package))
    for root in ([repo_root, repo_root / 'scenes'] + list(extra_roots or [])):
        if root.exists():
            for pkg_xml in root.rglob('package.xml'):
                add(pkg_xml.parent)

    # 3. AMENT_PREFIX_PATH share.
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if prefix:
            add(Path(prefix) / 'share' / package)

    # 4. ROS Humble share.
    add(Path('/opt/ros/humble/share') / package)
    return roots


def resolve_package_mesh_uri(uri: str, *, repo_root: Path, package_map: dict[str, Path] | None = None, extra_roots: list[Path] | None = None, supported_suffixes: set[str] | None = None) -> tuple[Path | None, str, Path | None, str | None, list[Path]]:
    parsed = urlparse(uri)
    package = parsed.netloc
    rel = Path(unquote(parsed.path).lstrip('/'))
    parts = rel.parts
    if parsed.scheme != 'package' or not package or rel.is_absolute() or not parts or any(part in ('', '.', '..') for part in parts):
        return None, package or 'package', None, f'Invalid or unsafe package URI: {uri}', []
    suffixes = supported_suffixes or SUPPORTED_BROWSER_MESH_EXTS
    if rel.suffix.lower() not in suffixes:
        return None, package, None, f'Unsupported mesh format for {uri}; supported formats are .stl, .dae, and .obj.', []
    candidates = _merge_package_candidates(package, repo_root.resolve(), extra_roots, package_map)
    checked: list[Path] = []
    for package_dir in candidates:
        candidate = (package_dir / rel).resolve()
        checked.append(package_dir)
        try:
            candidate.relative_to(package_dir.resolve())
        except ValueError:
            return None, package, None, f'Invalid or unsafe package URI: {uri}', checked
        if candidate.is_file():
            return candidate, package, Path(package, *parts), None, checked
    checked_text = ', '.join(str(p) for p in checked) or '<none>'
    detail = f'package={package}; repository_root={repo_root.resolve()}; candidate_package_roots_checked=[{checked_text}]'
    if checked:
        return None, package, None, f'Package mesh file does not exist: {uri} ({detail})', checked
    return None, package, None, f'Could not resolve package mesh URI: {uri} ({detail})', checked

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
        resolved, _pkg, _stage_rel, warning, _checked = resolve_package_mesh_uri(original, repo_root=repo_root, package_map=package_map)
        if resolved is not None:
            return found(resolved, 'package_uri')
        return MeshResolution(uri, '', False, ext, 'unresolved', warning or f'unresolved package URI: {original}')

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
