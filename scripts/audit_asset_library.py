#!/usr/bin/env python3
"""Headless inventory of the Workcell Studio asset library.

Reproduces the Builder's Asset Library discovery (gui/asset_catalog_model.cpp:
recursive crawl of the asset roots, one entry per ROS package) and inspects
every mesh file under the asset roots:

* geometry: triangle count, degenerate faces, bounding box, likely units,
  boundary (open) edges, signed volume, stored-normal vs winding agreement;
* COLLADA: <unit meter>, up axis, materials, texture files;
* references: scenes, catalogs, robot/tool descriptions, scripts, tests,
  docs and evidence that name the file or its package:// URI;
* provenance: package.xml licence and maintainer, LICENSE/README files.

It never opens a window and never modifies files. Output: JSON (and an
optional Markdown summary).
"""

from __future__ import annotations

import argparse
import json
import os
import hashlib
import re
import struct
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import numpy as np

MESH_EXT = {".stl", ".dae", ".obj", ".glb", ".gltf"}
DESC_EXT = {".urdf", ".xacro"}
TEXT_EXT = {".yaml", ".yml", ".json", ".xacro", ".urdf", ".py", ".cpp", ".hpp", ".h", ".inc", ".js",
            ".md", ".launch", ".xml", ".srdf", ".txt", ".cmake", ".sdf", ".world"}
SKIP_DIRS = {".git", "build", "install", "log", "node_modules", "__pycache__", ".pytest_cache", "dist"}


# --------------------------------------------------------------------------- GUI discovery port

def _has_component(path: Path, name: str) -> bool:
    return any(part.lower() == name for part in path.parts)


def _package_dir(file: Path, root: Path) -> Path | None:
    current = file.parent
    while True:
        if (current / "package.xml").exists():
            return current
        if current == root or len(str(current)) < len(str(root)) or current.parent == current:
            return None
        current = current.parent


def _description_dir(file: Path, root: Path) -> Path | None:
    current = file.parent
    while current != root and current.parent != current:
        if current.name.endswith("_description") and len(current.name) > 12:
            return current
        current = current.parent
    return None


def withheld_reason(package_dir: Path | None) -> str | None:
    """Reason a package withholds itself from the library (package.xml marker), else None."""
    if package_dir is None or not (package_dir / "package.xml").exists():
        return None
    m = re.search(r'<workcell_asset_library\s+status="(replace|remove)"(?:\s+reason="([^"]*)")?',
                  (package_dir / "package.xml").read_text(errors="replace"))
    return (m.group(2) or m.group(1)) if m else None


def _placeholder(path: Path) -> bool:
    text = str(path).lower()
    return "/workcell_builder/workcell_builder/assets/environment/" in text or "placeholder" in text


def _file_rank(path: Path, root: Path, repo_root: Path) -> int:
    root_text = str(root).lower()
    rank = 80 if "/workcell_builder/workcell_builder/assets" in root_text else (
        0 if root_text.startswith(str(repo_root / "assets").lower()) else 20)
    ext = path.suffix.lower()
    if _placeholder(path):
        rank += 200
    if ext in DESC_EXT:
        rank += 60
    if ext in MESH_EXT:
        rank += 20
        if _has_component(path, "visual"):
            rank -= 15
    if ext == ".dae":
        rank -= 5
    if path.name.lower() == "d435.dae":
        rank -= 20
    return rank


def gui_catalog(repo_root: Path, withheld: dict | None = None) -> list[dict]:
    """Entries the Builder's Asset Library lists (workspace == repo checkout)."""
    roots = []
    for root in (repo_root / "assets", repo_root / "workcell_builder" / "workcell_builder" / "assets" / "environment"):
        if root.exists() and root.resolve() not in roots:
            roots.append(root.resolve())
    candidates = []
    seen = set()
    for root in roots:
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames.sort()
            for name in sorted(filenames):
                path = Path(dirpath) / name
                ext = path.suffix.lower()
                if ext not in MESH_EXT and ext not in DESC_EXT:
                    continue
                if _has_component(path, "collision") or any(
                        _has_component(path, c) for c in ("tests", "test", "rviz", "launch")):
                    continue
                key = str(path.resolve())
                if key in seen:
                    continue
                seen.add(key)
                pkg = _package_dir(path, root)
                if pkg is not None and "_moveit_config" in pkg.name.lower():
                    continue
                reason = withheld_reason(pkg)
                if reason is not None:
                    if withheld is not None:
                        withheld[pkg.name] = reason
                    continue
                group = pkg or _description_dir(path, root)
                candidates.append({
                    "rank": _file_rank(path, root, repo_root),
                    "package": str(group) if group else "",
                    "id": pkg.name if pkg else path.stem,
                    "path": str(path),
                    "placeholder": _placeholder(path),
                })
    candidates.sort(key=lambda c: (c["rank"], c["path"]))
    package_dirs = {}
    for c in candidates:  # best-ranked directory of each package name, as the Builder keeps
        if c["package"]:
            package_dirs.setdefault(Path(c["package"]).name, Path(c["package"]))
    emitted, geometry, out = set(), set(), []
    for c in candidates:
        if c["package"] and c["package"] in emitted:
            continue
        emitted.add(c["package"])
        # Byte-identical geometry from two packages is one product (as the Builder lists it).
        key = geometry_content_key(Path(c["path"]), package_dirs)
        if key is not None and key in geometry:
            continue
        if key is not None:
            geometry.add(key)
        out.append(c)
    return out


def geometry_content_key(path: Path, package_dirs: dict):
    """Hash of an entry's visual mesh, or of the first package:// mesh its description references."""
    mesh = path
    if path.suffix.lower() in (".xacro", ".urdf"):
        m = re.search(r'filename="package://([^/"]+)/([^"]+)"', path.read_text(errors="replace"))
        if not m or m.group(1) not in package_dirs:
            return None
        mesh = package_dirs[m.group(1)] / m.group(2)
    try:
        return hashlib.sha256(mesh.read_bytes()).hexdigest()
    except OSError:
        return None


# --------------------------------------------------------------------------- mesh parsing

def _stl_triangles(path: Path):
    data = path.read_bytes()
    notes = []
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if 84 + 50 * count == len(data):
            if data[:5].lower() == b"solid":
                notes.append("binary STL whose header starts with 'solid' (ASCII-detection hazard)")
            rec = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]),
                                count=count, offset=84)
            return rec["v"].astype(np.float64), rec["n"].astype(np.float64), "binary", notes
    text = data.decode("latin-1")
    if not text.lstrip().lower().startswith("solid"):
        raise ValueError("not a valid binary or ASCII STL")
    verts = re.findall(r"vertex\s+(\S+)\s+(\S+)\s+(\S+)", text)
    norms = re.findall(r"facet\s+normal\s+(\S+)\s+(\S+)\s+(\S+)", text)
    v = np.array(verts, dtype=np.float64)
    if len(v) % 3:
        raise ValueError("ASCII STL vertex count is not a multiple of 3")
    return v.reshape(-1, 3, 3), np.array(norms, dtype=np.float64).reshape(-1, 3), "ascii", notes


def _dae_triangles(path: Path):
    """Triangles of every <triangles>/<polylist> (polygons fan-triangulated), scaled by <unit meter>."""
    tree = ET.parse(path)
    root = tree.getroot()
    ns = root.tag.split("}")[0] + "}" if root.tag.startswith("{") else ""
    info = {}
    unit = root.find(f"{ns}asset/{ns}unit")
    meter = float(unit.get("meter", "1")) if unit is not None else 1.0
    up = root.find(f"{ns}asset/{ns}up_axis")
    info["unit_meter"] = meter
    info["up_axis"] = up.text.strip() if up is not None and up.text else "Y_UP"
    info["materials"] = len(root.findall(f".//{ns}library_materials/{ns}material"))
    images = [i.text.strip() for i in root.findall(f".//{ns}library_images/{ns}image/{ns}init_from") if i.text]
    info["textures"] = images
    info["missing_textures"] = [i for i in images if not (path.parent / i.replace("file://", "")).exists()]
    tris = []
    poly_non_tri = 0
    for mesh in root.iter(f"{ns}mesh"):
        sources = {}
        for src in mesh.findall(f"{ns}source"):
            arr = src.find(f"{ns}float_array")
            if arr is not None and arr.text:
                sources[src.get("id")] = np.array(arr.text.split(), dtype=np.float64)
        vert_map = {}
        for vtx in mesh.findall(f"{ns}vertices"):
            for inp in vtx.findall(f"{ns}input"):
                if inp.get("semantic") == "POSITION":
                    vert_map[vtx.get("id")] = inp.get("source", "").lstrip("#")
        for prim in list(mesh.findall(f"{ns}triangles")) + list(mesh.findall(f"{ns}polylist")):
            inputs = prim.findall(f"{ns}input")
            stride = max(int(i.get("offset", 0)) for i in inputs) + 1
            pos_off, pos_src = None, None
            for i in inputs:
                if i.get("semantic") == "VERTEX":
                    pos_off = int(i.get("offset", 0))
                    pos_src = vert_map.get(i.get("source", "").lstrip("#"))
            p = prim.find(f"{ns}p")
            if pos_off is None or pos_src not in sources or p is None or not p.text:
                continue
            positions = sources[pos_src].reshape(-1, 3)
            idx = np.array(p.text.split(), dtype=np.int64).reshape(-1, stride)[:, pos_off]
            if prim.tag.endswith("polylist"):
                vc = prim.find(f"{ns}vcount")
                counts = np.array(vc.text.split(), dtype=np.int64) if vc is not None and vc.text else np.array([])
                cursor = 0
                for n in counts:
                    poly = idx[cursor:cursor + n]
                    cursor += n
                    if n != 3:
                        poly_non_tri += 1
                    for k in range(1, n - 1):
                        tris.append(positions[[poly[0], poly[k], poly[k + 1]]])
            else:
                if len(idx) % 3 == 0:
                    tris.extend(positions[idx.reshape(-1, 3)])
    info["non_triangle_polygons"] = poly_non_tri
    arr = np.array(tris, dtype=np.float64).reshape(-1, 3, 3) * meter
    return arr, info


def _obj_triangles(path: Path):
    verts, tris = [], []
    for line in path.read_text(errors="replace").splitlines():
        if line.startswith("v "):
            verts.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("f "):
            ids = [int(tok.split("/")[0]) for tok in line.split()[1:]]
            ids = [i - 1 if i > 0 else len(verts) + i for i in ids]
            for k in range(1, len(ids) - 1):
                tris.append([ids[0], ids[k], ids[k + 1]])
    v = np.array(verts, dtype=np.float64)
    return v[np.array(tris, dtype=np.int64)] if tris else np.zeros((0, 3, 3))


def analyse_triangles(tris: np.ndarray, stored_normals: np.ndarray | None = None) -> dict:
    out = {"triangles": int(len(tris))}
    if len(tris) == 0:
        out["problems"] = ["no triangles"]
        return out
    problems = []
    if not np.isfinite(tris).all():
        problems.append("non-finite vertex coordinates")
        tris = np.nan_to_num(tris)
    lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    ext = hi - lo
    out["bbox_min"] = lo.round(6).tolist()
    out["extent"] = ext.round(6).tolist()
    cross = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    area2 = np.linalg.norm(cross, axis=1)
    scale = max(float(ext.max()), 1e-12)
    degenerate = int((area2 < 1e-10 * scale * scale).sum())
    out["degenerate_triangles"] = degenerate
    if degenerate > 0.05 * len(tris):
        problems.append(f"{degenerate} degenerate (zero-area) triangles ({degenerate / len(tris):.0%})")
    if stored_normals is not None and len(stored_normals) == len(tris):
        n = np.linalg.norm(stored_normals, axis=1)
        valid = (n > 1e-9) & (area2 > 0)
        if valid.any():
            dot = (cross[valid] / area2[valid, None] * (stored_normals[valid] / n[valid, None])).sum(1)
            out["stored_normal_disagreement"] = round(float((dot < 0).mean()), 4)
            if out["stored_normal_disagreement"] > 0.5:
                problems.append("stored facet normals oppose triangle winding (inverted winding)")
    # Topology on welded vertices.
    q = np.round(tris.reshape(-1, 3) / (scale * 1e-7)).astype(np.int64)
    _, inv = np.unique(q, axis=0, return_inverse=True)
    faces = inv.reshape(-1, 3)
    edges = np.sort(np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), axis=1)
    _, counts = np.unique(edges, axis=0, return_counts=True)
    out["boundary_edges"] = int((counts == 1).sum())
    out["non_manifold_edges"] = int((counts > 2).sum())
    closed = out["boundary_edges"] == 0
    out["closed"] = bool(closed)
    volume = float(np.einsum("ij,ij->i", tris[:, 0], np.cross(tris[:, 1], tris[:, 2])).sum() / 6.0)
    out["signed_volume"] = volume
    if closed and volume < 0:
        problems.append("closed mesh with negative volume (inside-out winding)")
    if ext.max() > 20.0:
        out["unit_guess"] = "millimetres (extent > 20 m as metres)"
    elif ext.max() < 0.005:
        out["unit_guess"] = "suspiciously small (< 5 mm)"
    else:
        out["unit_guess"] = "metres"
    if len(tris) < 4:
        problems.append(f"not a solid model ({len(tris)} triangle{'s' if len(tris) != 1 else ''}: stub geometry)")
    if np.count_nonzero(ext < 1e-9) >= 1:
        problems.append("flat mesh (zero extent on an axis)")
    out["problems"] = problems
    return out


def inspect_mesh(path: Path) -> dict:
    ext = path.suffix.lower()
    rec = {"format": ext[1:].upper(), "bytes": path.stat().st_size}
    try:
        if ext == ".stl":
            tris, normals, kind, notes = _stl_triangles(path)
            rec["stl_encoding"] = kind
            rec.update(analyse_triangles(tris, normals))
            rec["problems"] = notes + rec["problems"]
        elif ext == ".dae":
            tris, info = _dae_triangles(path)
            rec.update(info)
            rec.update(analyse_triangles(tris))
            if info["missing_textures"]:
                rec["problems"].append(f"missing textures: {info['missing_textures']}")
            if info["materials"] == 0:
                rec["problems"].append("no materials")
        elif ext == ".obj":
            rec.update(analyse_triangles(_obj_triangles(path)))
        else:
            rec["problems"] = ["format not inspected (glTF)"]
    except Exception as exc:  # noqa: BLE001 - report, never crash the inventory
        rec["problems"] = [f"unreadable: {exc}"]
    return rec


# --------------------------------------------------------------------------- references & provenance

def text_corpus(repo_root: Path, exclude: Path) -> dict[str, str]:
    corpus = {}
    for dirpath, dirnames, filenames in os.walk(repo_root):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        for name in filenames:
            path = Path(dirpath) / name
            if path.suffix.lower() not in TEXT_EXT or path.stat().st_size > 4_000_000:
                continue
            corpus[str(path.relative_to(repo_root))] = path.read_text(errors="replace")
    return corpus


def references(rel: str, package: str, pkg_rel_file: str, corpus: dict[str, str], own_prefix: str) -> list[str]:
    needles = {rel}
    if package and pkg_rel_file:
        needles.add(f"package://{package}/{pkg_rel_file}")
        needles.add(f"$(find {package})/{pkg_rel_file}")
    hits = []
    for doc, text in corpus.items():
        if doc.startswith(own_prefix) and doc.endswith((".xacro", ".urdf")) is False:
            continue
        if any(n in text for n in needles):
            hits.append(doc)
    return sorted(hits)


def provenance(package_dir: Path | None) -> dict:
    if package_dir is None:
        return {"package_xml": False}
    info = {"package_xml": (package_dir / "package.xml").exists()}
    if info["package_xml"]:
        text = (package_dir / "package.xml").read_text(errors="replace")
        lic = re.findall(r"<license>(.*?)</license>", text)
        info["license"] = lic[0].strip() if lic else ""
        desc = re.findall(r"<description>(.*?)</description>", text, re.S)
        info["description"] = " ".join(desc[0].split())[:160] if desc else ""
    info["license_file"] = any((package_dir / n).exists() for n in ("LICENSE", "LICENSE.txt", "COPYING"))
    info["readme"] = any((package_dir / n).exists() for n in ("README.md", "README.rst", "README"))
    return info


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    ap.add_argument("--json", type=Path, required=True)
    args = ap.parse_args(argv)
    repo = args.repo_root.resolve()
    roots = [repo / "assets", repo / "workcell_builder" / "workcell_builder" / "assets", repo / "workcell_studio_catalog" / "meshes"]
    corpus = text_corpus(repo, repo / "assets")
    withheld: dict[str, str] = {}
    gui = gui_catalog(repo, withheld)
    gui_paths = {c["path"]: c for c in gui}
    files = []
    for root in roots:
        if not root.exists():
            continue
        for path in sorted(root.rglob("*")):
            if not path.is_file() or path.suffix.lower() not in MESH_EXT:
                continue
            rel = str(path.relative_to(repo))
            pkg = _package_dir(path, root)
            pkg_rel_file = str(path.relative_to(pkg)) if pkg else ""
            own_prefix = str(pkg.relative_to(repo)) + "/" if pkg else rel
            rec = {"path": rel, "package": pkg.name if pkg else "", "role": "collision" if _has_component(path, "collision") else "visual",
                   "in_gui_catalog": str(path) in gui_paths}
            rec.update(inspect_mesh(path))
            rec["referenced_by"] = references(rel, pkg.name if pkg else "", pkg_rel_file, corpus, own_prefix)
            files.append(rec)
    packages = {}
    for root in roots:
        for xml in root.rglob("package.xml"):
            packages[str(xml.parent.relative_to(repo))] = provenance(xml.parent)
    report = {"repo_root": str(repo), "gui_catalog": [
        {k: (str(Path(v).relative_to(repo)) if k in ("path", "package") and v else v) for k, v in c.items()} for c in gui],
        "withheld_packages": withheld, "mesh_files": files, "packages": packages}
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps(report, indent=1, sort_keys=True))
    bad = [f for f in files if f.get("problems")]
    print(f"gui entries={len(gui)} withheld packages={len(withheld)} mesh files={len(files)} "
          f"with problems={len(bad)} packages={len(packages)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
