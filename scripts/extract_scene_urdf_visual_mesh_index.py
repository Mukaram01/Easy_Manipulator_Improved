#!/usr/bin/env python3
from __future__ import annotations
import json, os, re, shutil, subprocess
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]
SCENES_ROOT = ROOT / "scenes"
MESH_RE = re.compile(r"<mesh[^>]*filename=[\"']([^\"']+)[\"'][^>]*/?>")
INCLUDE_RE = re.compile(r"<xacro:include[^>]*filename=[\"']([^\"']+)[\"'][^>]*/?>")
ARG_RE = re.compile(r"<xacro:arg[^>]*name=[\"']([^\"']+)[\"'][^>]*default=[\"']([^\"']+)[\"']")


def read_yaml(path: Path):
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8")) if path.exists() else None
    except Exception:
        return None


def parse_vec(text: str | None, n: int, default: float = 0.0):
    vals = [default] * n
    if not text:
        return vals
    for i, p in enumerate(re.split(r"\s+", text.strip())[:n]):
        try: vals[i] = float(p)
        except Exception: pass
    return vals


def cat(link: str):
    low = link.lower()
    if any(t in low for t in ["ur", "robot", "base", "arm"]): return "robot_visual"
    if any(t in low for t in ["gripper", "tool", "ee", "suction", "hand"]): return "gripper_visual"
    if any(t in low for t in ["table", "conveyor", "camera", "fixture", "zone", "cell", "env"]): return "environment_visual"
    if "object" in low or "bin" in low: return "object_visual"
    return "unknown_visual"


def identity_tf():
    return [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


def matmul4(a, b):
    return [[sum(a[r][k] * b[k][c] for k in range(4)) for c in range(4)] for r in range(4)]


def tf_from_xyz_rpy(xyz, rpy):
    x, y, z = (xyz + [0.0, 0.0, 0.0])[:3]
    rr, pp, yy = (rpy + [0.0, 0.0, 0.0])[:3]
    cr, sr = math.cos(rr), math.sin(rr)
    cp, sp = math.cos(pp), math.sin(pp)
    cy, sy = math.cos(yy), math.sin(yy)
    rot = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    out = identity_tf()
    for r in range(3):
        for c in range(3):
            out[r][c] = rot[r][c]
    out[0][3], out[1][3], out[2][3] = x, y, z
    return out


def parse_urdf_graph(root: ET.Element):
    links = {}
    joints = []
    for link in root.findall('.//link'):
        lname = link.attrib.get('name', 'unknown_link')
        links.setdefault(lname, {'inbound_joints': [], 'outbound_joints': []})
    for joint in root.findall('.//joint'):
        name = joint.attrib.get('name', '')
        jtype = joint.attrib.get('type', 'fixed')
        parent = (joint.find('parent').attrib.get('link', '') if joint.find('parent') is not None else '')
        child = (joint.find('child').attrib.get('link', '') if joint.find('child') is not None else '')
        origin = joint.find('origin')
        xyz = parse_vec(origin.attrib.get('xyz') if origin is not None else None, 3)
        rpy = parse_vec(origin.attrib.get('rpy') if origin is not None else None, 3)
        axis_tag = joint.find('axis')
        axis = parse_vec(axis_tag.attrib.get('xyz') if axis_tag is not None else None, 3)
        j = {'name': name, 'type': jtype, 'parent': parent, 'child': child, 'origin': {'xyz': xyz, 'rpy': rpy}, 'axis': axis}
        joints.append(j)
        links.setdefault(parent, {'inbound_joints': [], 'outbound_joints': []})
        links.setdefault(child, {'inbound_joints': [], 'outbound_joints': []})
        links[parent]['outbound_joints'].append(j)
        links[child]['inbound_joints'].append(j)
    roots = sorted([lname for lname, meta in links.items() if not meta['inbound_joints']])
    return links, joints, roots


def compute_link_world_tfs(link_graph, roots):
    world = {}
    status = {k: 'local_only' for k in link_graph.keys()}
    warnings = []

    def dfs(link, cur_tf, stack):
        world[link] = cur_tf
        status[link] = 'resolved'
        for j in link_graph.get(link, {}).get('outbound_joints', []):
            child = j.get('child', '')
            if not child:
                continue
            if child in stack:
                status[child] = 'partial'
                warnings.append(f"cyclic joint graph detected: {' -> '.join(stack + [child])}")
                continue
            if child in world:
                continue
            # static/default view: joint value = 0 for revolute/continuous/prismatic, so origin-only tf
            child_tf = matmul4(cur_tf, tf_from_xyz_rpy(j['origin']['xyz'], j['origin']['rpy']))
            dfs(child, child_tf, stack + [child])

    if not roots and link_graph:
        warnings.append('no root links detected; graph may be cyclic or disconnected')
    for root in roots:
        dfs(root, identity_tf(), [root])

    for link in link_graph.keys():
        if link in world:
            continue
        inbound = link_graph[link].get('inbound_joints', [])
        if inbound:
            status[link] = 'partial'
            warnings.append(f"link disconnected from root propagation: {link}")
        else:
            world[link] = identity_tf()
            status[link] = 'local_only'
    return world, status, warnings


def discover_package_map(scene_dir: Path):
    package_map = {}
    search_roots = [ROOT, ROOT / "assets", ROOT / "scenes", scene_dir]
    ws_src = ROOT.parent / "src"
    if ws_src.exists(): search_roots.append(ws_src)
    for prefix in [p for p in os.environ.get("AMENT_PREFIX_PATH", "").split(":") if p]:
        share = Path(prefix) / "share"
        if share.exists():
            for d in share.iterdir():
                if d.is_dir():
                    package_map.setdefault(d.name, d)

    for root in search_roots:
        if not root.exists():
            continue
        for pkg_xml in root.rglob("package.xml"):
            pkg_dir = pkg_xml.parent
            try:
                xml = ET.fromstring(pkg_xml.read_text(encoding="utf-8", errors="ignore"))
                name = (xml.findtext("name") or pkg_dir.name).strip()
            except Exception:
                name = pkg_dir.name
            package_map.setdefault(name, pkg_dir)
            package_map.setdefault(pkg_dir.name, pkg_dir)
    return package_map


def apply_subs(text: str, args: dict[str, str]):
    out = text
    out = re.sub(r"\$\(arg\s+([^\)]+)\)", lambda m: args.get(m.group(1).strip(), ""), out)
    out = re.sub(r"\$\{([^\}]+)\}", lambda m: args.get(m.group(1).strip(), ""), out)
    return out


def resolve_uri(uri: str, scene_dir: Path, package_map: dict[str, Path], args: dict[str, str]):
    uri = apply_subs(uri.strip(), args)
    uri = uri.strip('"\'')
    if uri.startswith("file://"):
        uri = uri[len("file://"):]
    m = re.match(r"\$\(find\s+([^\)]+)\)(?:/(.*))?$", uri)
    if m:
        pkg, rel = m.group(1).strip(), (m.group(2) or "")
        if pkg in package_map:
            p = package_map[pkg] / rel
            if p.exists(): return str(p.resolve()), uri
    if uri.startswith("package://"):
        rest = uri[len("package://"):]
        if "/" in rest:
            pkg, rel = rest.split("/", 1)
            if pkg in package_map:
                p = package_map[pkg] / rel
                if p.exists(): return str(p.resolve()), uri
    p = Path(uri)
    if p.is_absolute() and p.exists(): return str(p.resolve()), uri
    for c in [scene_dir / uri, scene_dir / "urdf" / uri, ROOT / uri, ROOT / "assets" / uri]:
        if c.exists(): return str(c.resolve()), uri
    return "", uri


def gather_text_with_includes(path: Path, package_map: dict[str, Path], args: dict[str, str], seen: set[Path], warnings: list[str]):
    if path in seen or not path.exists():
        return ""
    seen.add(path)
    text = path.read_text(encoding="utf-8", errors="ignore")
    for n, d in ARG_RE.findall(text):
        args.setdefault(n.strip(), apply_subs(d.strip(), args))
    chunks = [text]
    for inc in INCLUDE_RE.findall(text):
        resolved, _ = resolve_uri(inc, path.parent, package_map, args)
        if not resolved:
            warnings.append(f"unresolved xacro include: {inc}")
            continue
        inc_p = Path(resolved)
        if inc_p.suffix.lower() not in {".xacro", ".urdf", ".xml"}:
            continue
        chunks.append(gather_text_with_includes(inc_p, package_map, args, seen, warnings))
    return "\n".join(chunks)


def expand_xacro(path: Path):
    if shutil.which("xacro") is None:
        return None, "best_effort", ["xacro executable unavailable; using best_effort parsing"]
    try:
        out = subprocess.run(["xacro", str(path)], check=True, capture_output=True, text=True, timeout=45)
        return out.stdout, "xacro_expanded", []
    except Exception as exc:
        return None, "best_effort", [f"xacro expansion failed: {exc}"]


def extract(xml_text: str, scene_dir: Path, package_map: dict[str, Path], args: dict[str, str]):
    items, warnings = [], []
    try:
        root = ET.fromstring(xml_text)
        idx = 0
        link_graph, joints, roots = parse_urdf_graph(root)
        link_world_tfs, link_status, gw = compute_link_world_tfs(link_graph, roots)
        warnings.extend(gw)
        for link in root.findall('.//link'):
            lname = link.attrib.get('name', 'unknown_link')
            for visual in link.findall('visual'):
                mesh = visual.find('geometry/mesh')
                if mesh is None: continue
                package_uri = mesh.attrib.get('filename', '').strip()
                if not package_uri: continue
                src, normalized = resolve_uri(package_uri, scene_dir, package_map, args)
                warn = "" if src else f"unresolved mesh path: {normalized}"
                origin = visual.find('origin')
                visual_tf = tf_from_xyz_rpy(parse_vec(origin.attrib.get('xyz') if origin is not None else None, 3), parse_vec(origin.attrib.get('rpy') if origin is not None else None, 3))
                link_tf = link_world_tfs.get(lname, identity_tf())
                world_visual_tf = matmul4(link_tf, visual_tf)
                items.append({"id": f"urdf_visual_{idx}_{lname}", "link": lname, "visual": visual.attrib.get('name', ''),
                              "source_path": src, "package_uri": normalized,
                              "pose": {"xyz": parse_vec(origin.attrib.get('xyz') if origin is not None else None, 3), "rpy": parse_vec(origin.attrib.get('rpy') if origin is not None else None, 3)},
                              "world_visual_tf": world_visual_tf,
                              "link_status": link_status.get(lname, 'local_only'),
                              "scale": parse_vec(mesh.attrib.get('scale'), 3, 1.0), "material": {}, "category": cat(lname), "resolved": bool(src), "warning": warn})
                if warn: warnings.append(warn)
                idx += 1
        if items:
            return items, warnings
    except Exception:
        pass
    for idx, m in enumerate(MESH_RE.finditer(xml_text)):
        src, normalized = resolve_uri(m.group(1), scene_dir, package_map, args)
        warn = "" if src else f"unresolved mesh path: {normalized}"
        items.append({"id": f"urdf_visual_regex_{idx}", "link": "unknown_link", "visual": "", "source_path": src, "package_uri": normalized,
                      "pose": {"xyz": [0,0,0], "rpy": [0,0,0]}, "scale": [1,1,1], "material": {}, "category": "unknown_visual", "resolved": bool(src), "warning": warn})
        if warn: warnings.append(warn)
    return items, warnings


def main():
    scenes = sorted([p for p in SCENES_ROOT.iterdir() if p.is_dir()])
    report = {"scene_count": 0, "visual_count": 0, "resolved": 0, "unresolved": 0, "scenes": []}
    for scene_dir in scenes:
        manifest = read_yaml(scene_dir / "scene_manifest.yaml") or {}
        urdf_rel = ((manifest.get("files") or {}).get("urdf_xacro") or "urdf/scene.urdf.xacro")
        urdf_path = scene_dir / urdf_rel
        warnings, items, mode = [], [], "best_effort"
        package_map = discover_package_map(scene_dir)
        if urdf_path.exists():
            expanded, mode, w = expand_xacro(urdf_path); warnings.extend(w)
            args = {"scene_dir": str(scene_dir)}
            if expanded:
                text = expanded
            else:
                text = gather_text_with_includes(urdf_path, package_map, args, set(), warnings)
                mode = "best_effort_recursive"
            items, xw = extract(text, scene_dir, package_map, args)
            warnings.extend(xw)
        else:
            warnings.append(f"missing urdf_xacro: {urdf_path}")
        if not items:
            items.append({"id":"urdf_visual_unresolved_0","link":"unknown_link","visual":"","source_path":"","package_uri":"","pose":{"xyz":[0,0,0],"rpy":[0,0,0]},"scale":[1,1,1],"material":{},"category":"unknown_visual","resolved":False,"warning":"no mesh references discovered in URDF/Xacro parse"})

        out = scene_dir / "generated" / "scene_visual_mesh_index.json"
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps({"scene_name": scene_dir.name, "source_urdf_xacro_path": str(urdf_path), "extraction_mode": mode, "visual_items": items, "warnings": warnings}, indent=2), encoding="utf-8")
        report["scene_count"] += 1; report["visual_count"] += len(items)
        report["resolved"] += sum(1 for i in items if i.get("resolved")); report["unresolved"] += sum(1 for i in items if not i.get("resolved"))
        report["scenes"].append({"scene": scene_dir.name, "visual_count": len(items), "warnings": warnings, "output": str(out)})

    build = ROOT / "build"; build.mkdir(exist_ok=True)
    rpt = build / "workcell_studio_urdf_visual_mesh_index_report.json"
    rpt.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"scanned={report['scene_count']} visuals={report['visual_count']} resolved={report['resolved']} unresolved={report['unresolved']}")
    print(rpt)
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
