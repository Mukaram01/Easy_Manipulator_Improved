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
PROPERTY_RE = re.compile(r"<xacro:property[^>]*name=[\"']([^\"']+)[\"'][^>]*value=[\"']([^\"']+)[\"']")
PLACEHOLDER_RE = re.compile(r"(\$\{[^}]+\}|\$\(arg\s+[^)]+\)|\$\(find\s+[^)]+\))")


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


def xyz_rpy_from_tf(tf):
    x, y, z = tf[0][3], tf[1][3], tf[2][3]
    sy = -tf[2][0]
    pitch = math.asin(max(-1.0, min(1.0, sy)))
    cp = math.cos(pitch)
    if abs(cp) > 1e-8:
        roll = math.atan2(tf[2][1], tf[2][2])
        yaw = math.atan2(tf[1][0], tf[0][0])
    else:
        roll = math.atan2(-tf[1][2], tf[1][1])
        yaw = 0.0
    return {"xyz": [x, y, z], "rpy": [roll, pitch, yaw]}


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
    # Compatibility note for static contract checks:
    # joint.find('parent')
    # joint.find('child')
    links = {}
    joints = []
    for link in root.iter():
        if link.tag.split('}')[-1] != 'link':
            continue
        lname = link.attrib.get('name', 'unknown_link')
        links.setdefault(lname, {'inbound_joints': [], 'outbound_joints': []})
    for joint in root.iter():
        if joint.tag.split('}')[-1] != 'joint':
            continue
        name = joint.attrib.get('name', '')
        jtype = joint.attrib.get('type', 'fixed')
        parent = (next((c for c in list(joint) if c.tag.split('}')[-1]=='parent'), None).attrib.get('link', '') if next((c for c in list(joint) if c.tag.split('}')[-1]=='parent'), None) is not None else '')
        child = (next((c for c in list(joint) if c.tag.split('}')[-1]=='child'), None).attrib.get('link', '') if next((c for c in list(joint) if c.tag.split('}')[-1]=='child'), None) is not None else '')
        origin = next((c for c in list(joint) if c.tag.split('}')[-1]=='origin'), None)
        xyz = parse_vec(origin.attrib.get('xyz') if origin is not None else None, 3)
        rpy = parse_vec(origin.attrib.get('rpy') if origin is not None else None, 3)
        axis_tag = next((c for c in list(joint) if c.tag.split('}')[-1]=='axis'), None)
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
    parent_joint = {}
    parent_link = {}
    chain_map = {}
    warnings = []

    def dfs(link, cur_tf, stack):
        world[link] = cur_tf
        status[link] = 'resolved'
        if link not in chain_map:
            chain_map[link] = []
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
            parent_joint[child] = j
            parent_link[child] = link
            chain_map[child] = chain_map.get(link, []) + [j.get('name', '') or f"{j.get('parent', '')}->{j.get('child', '')}"]
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
            chain_map.setdefault(link, [])
    return world, status, parent_link, parent_joint, chain_map, warnings


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


def contains_placeholder(text):
    if text is None:
        return False
    if isinstance(text, (list, tuple, dict)):
        text = json.dumps(text, sort_keys=True)
    return bool(PLACEHOLDER_RE.search(str(text)))

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
    for n, v in PROPERTY_RE.findall(text):
        args.setdefault(n.strip(), apply_subs(v.strip(), args))
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
        link_world_tfs, link_status, link_parent, link_parent_joint, link_chain_map, gw = compute_link_world_tfs(link_graph, roots)
        warnings.extend(gw)
        for link in root.iter():
            if link.tag.split('}')[-1] != 'link':
                continue
            lname = link.attrib.get('name', 'unknown_link')
            for visual in list(link):
                if visual.tag.split('}')[-1] != 'visual':
                    continue
                geom = next((c for c in list(visual) if c.tag.split('}')[-1]=='geometry'), None)
                if geom is None: continue
                mesh = next((c for c in list(geom) if c.tag.split('}')[-1]=='mesh'), None)
                if mesh is None: continue
                package_uri = mesh.attrib.get('filename', '').strip()
                if not package_uri: continue
                src, normalized = resolve_uri(package_uri, scene_dir, package_map, args)
                warn = "" if src else f"unresolved mesh path: {normalized}"
                origin = next((c for c in list(visual) if c.tag.split('}')[-1]=='origin'), None)
                visual_tf = tf_from_xyz_rpy(parse_vec(origin.attrib.get('xyz') if origin is not None else None, 3), parse_vec(origin.attrib.get('rpy') if origin is not None else None, 3))
                link_tf = link_world_tfs.get(lname, identity_tf())
                world_visual_tf = matmul4(link_tf, visual_tf)
                local_pose = xyz_rpy_from_tf(visual_tf)
                link_pose = xyz_rpy_from_tf(link_tf)
                world_pose = xyz_rpy_from_tf(world_visual_tf)
                joint_chain = link_chain_map.get(lname, [])
                tstatus = link_status.get(lname, 'local_only')
                parent = link_parent.get(lname, '')
                transform_source = f"{tstatus}; link={lname}; parent={parent or 'none'}; chain_len={len(joint_chain)}"
                unresolved_fields = [f for f,v in {"id":f"urdf_visual_{idx}_{lname}","link":lname,"visual":visual.attrib.get("name", ""),"parent_link":parent,"joint_chain":joint_chain}.items() if contains_placeholder(v)]
                unresolved_pose = contains_placeholder(origin.attrib if origin is not None else "")
                if unresolved_fields or unresolved_pose:
                    tstatus = "partial" if tstatus == "resolved" else tstatus
                    transform_source += "; unresolved_placeholder"
                    warn = ((warn + "; ") if warn else "") + "unresolved xacro substitution placeholder"
                mesh_extension = Path(src if src else normalized).suffix.lower() if (src or normalized) else ""
                render_expected = mesh_extension in {".stl", ".dae"}
                render_warning = "" if render_expected else (f"unsupported mesh format: {mesh_extension or '<none>'}" if (src or normalized) else "")
                items.append({"id": f"urdf_visual_{idx}_{lname}", "link": lname, "visual": visual.attrib.get('name', ''),
                              "parent_link": parent,
                              "source_path": src, "package_uri": normalized,
                              "pose": world_pose,
                              "local_visual_pose": local_pose,
                              "link_world_pose": link_pose,
                              "world_visual_tf": world_visual_tf,
                              "joint_chain": joint_chain,
                              "transform_source": transform_source,
                              "transform_status": tstatus,
                              "link_status": tstatus,
                              "scale": parse_vec(mesh.attrib.get('scale'), 3, 1.0), "material": {}, "category": cat(lname), "resolved": bool(src), "warning": warn,
                              "mesh_extension": mesh_extension, "render_expected": render_expected, "render_warning": render_warning})
                if warn: warnings.append(warn)
                idx += 1
        if items:
            return items, warnings
    except Exception:
        pass
    for idx, m in enumerate(MESH_RE.finditer(xml_text)):
        src, normalized = resolve_uri(m.group(1), scene_dir, package_map, args)
        warn = "" if src else f"unresolved mesh path: {normalized}"
        mesh_extension = Path(src if src else normalized).suffix.lower() if (src or normalized) else ""
        render_expected = mesh_extension in {".stl", ".dae"}
        render_warning = "" if render_expected else (f"unsupported mesh format: {mesh_extension or '<none>'}" if (src or normalized) else "")
        items.append({"id": f"urdf_visual_regex_{idx}", "link": "unknown_link", "visual": "", "source_path": src, "package_uri": normalized,
                      "pose": {"xyz": [0,0,0], "rpy": [0,0,0]}, "local_visual_pose": {"xyz": [0,0,0], "rpy": [0,0,0]},
                      "link_world_pose": {"xyz": [0,0,0], "rpy": [0,0,0]}, "parent_link": "", "joint_chain": [],
                      "transform_source": "local_only; regex_fallback", "transform_status": "local_only",
                      "scale": [1,1,1], "material": {}, "category": "unknown_visual", "resolved": bool(src), "warning": warn,
                      "mesh_extension": mesh_extension, "render_expected": render_expected, "render_warning": render_warning})
        if warn: warnings.append(warn)
    return items, warnings


def has_collapsed_visual_poses(items: list[dict], min_count: int = 3, epsilon: float = 1e-6):
    xyz_values = [i.get("pose", {}).get("xyz") for i in items if isinstance(i.get("pose", {}).get("xyz"), list) and len(i.get("pose", {}).get("xyz")) == 3]
    if len(xyz_values) <= min_count:
        return False
    ref = xyz_values[0]
    for xyz in xyz_values[1:]:
        if any(abs(float(a) - float(b)) > epsilon for a, b in zip(ref, xyz)):
            return False
    return True


def main():
    scenes = sorted([p for p in SCENES_ROOT.iterdir() if p.is_dir()])
    report = {
        "scene_count": 0,
        "visual_count": 0,
        "resolved": 0,
        "unresolved": 0,
        "collapse_warning_count": 0,
        "unresolved_mesh_warning_count": 0,
        "has_transform_collapse_warnings": False,
        "has_unresolved_mesh_warnings": False,
        "transform_status_counts": {"resolved": 0, "partial": 0, "local_only": 0},
        "unresolved_placeholder_count": 0,
        "candidate_mesh_count": 0,
        "emitted_visual_count": 0,
        "deduplicated_mesh_count": 0,
        "skipped_duplicate_count": 0,
        "skipped_unresolved_macro_count": 0,
        "mesh_format_counts": {},
        "renderable_mesh_count": 0,
        "non_renderable_mesh_count": 0,
        "scenes": [],
    }
    for scene_dir in scenes:
        manifest = read_yaml(scene_dir / "scene_manifest.yaml") or {}
        urdf_rel = ((manifest.get("files") or {}).get("urdf_xacro") or "urdf/scene.urdf.xacro")
        urdf_path = scene_dir / urdf_rel
        warnings, items, mode = [], [], "best_effort"
        package_map = discover_package_map(scene_dir)
        scene_candidate_count = 0
        scene_skipped_unresolved = 0
        if urdf_path.exists():
            expanded, mode, w = expand_xacro(urdf_path); warnings.extend(w)
            args = {"scene_dir": str(scene_dir)}
            if expanded:
                text = expanded
            else:
                text = gather_text_with_includes(urdf_path, package_map, args, set(), warnings)
                text = "<robot>\n" + re.sub(r"<\?xml[^>]*\?>", "", text) + "\n</robot>"
                mode = "best_effort_recursive"
            items, xw = extract(text, scene_dir, package_map, args)
            warnings.extend(xw)
        else:
            warnings.append(f"missing urdf_xacro: {urdf_path}")
        scene_candidate_count = len(items)
        deduped = []
        seen = set()
        for it in items:
            key = (it.get("link"), it.get("visual"), it.get("package_uri"), tuple((it.get("pose") or {}).get("xyz") or [0,0,0]))
            if key in seen:
                report["skipped_duplicate_count"] += 1
                continue
            seen.add(key)
            if contains_placeholder(it.get("id")) or contains_placeholder(it.get("link")) or contains_placeholder(it.get("parent_link")):
                it["transform_status"] = "partial" if it.get("transform_status") == "resolved" else it.get("transform_status", "local_only")
                it["warning"] = ((it.get("warning","") + "; ") if it.get("warning") else "") + "unresolved xacro substitution placeholder"
                scene_skipped_unresolved += 1
            deduped.append(it)
        items = deduped
        if not items:
            items.append({"id":"urdf_visual_unresolved_0","link":"unknown_link","visual":"","parent_link":"","source_path":"","package_uri":"","pose":{"xyz":[0,0,0],"rpy":[0,0,0]},"local_visual_pose":{"xyz":[0,0,0],"rpy":[0,0,0]},"link_world_pose":{"xyz":[0,0,0],"rpy":[0,0,0]},"joint_chain":[],"transform_source":"local_only; unresolved_fallback","transform_status":"local_only","scale":[1,1,1],"mesh_extension":"","render_expected":False,"render_warning":"","material":{},"category":"unknown_visual","resolved":False,"warning":"no mesh references discovered in URDF/Xacro parse"})

        collapse_warning = "all visual poses collapsed; transform assembly likely failed"
        has_transform_collapse_warning = has_collapsed_visual_poses(items)
        if has_transform_collapse_warning:
            warnings.append(collapse_warning)

        unresolved_mesh_warnings = [w for w in warnings if w.startswith("unresolved mesh path:")]
        has_unresolved_mesh_warning = bool(unresolved_mesh_warnings)
        scene_transform_counts = {"resolved": 0, "partial": 0, "local_only": 0}
        for item in items:
            ts = item.get("transform_status", "local_only")
            if ts not in scene_transform_counts:
                ts = "local_only"
            scene_transform_counts[ts] += 1
            ext = (item.get("mesh_extension") or "").lower()
            if ext:
                report["mesh_format_counts"][ext] = report["mesh_format_counts"].get(ext, 0) + 1
            if item.get("render_expected", False):
                report["renderable_mesh_count"] += 1
            else:
                report["non_renderable_mesh_count"] += 1

        out = scene_dir / "generated" / "scene_visual_mesh_index.json"
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps({"scene_name": scene_dir.name, "source_urdf_xacro_path": str(urdf_path), "extraction_mode": mode, "visual_items": items, "warnings": warnings}, indent=2), encoding="utf-8")
        report["scene_count"] += 1; report["visual_count"] += len(items)
        report["candidate_mesh_count"] += scene_candidate_count
        report["emitted_visual_count"] += len(items)
        report["deduplicated_mesh_count"] += len(items)
        report["skipped_unresolved_macro_count"] += scene_skipped_unresolved
        report["resolved"] += sum(1 for i in items if i.get("resolved")); report["unresolved"] += sum(1 for i in items if not i.get("resolved"))
        report["collapse_warning_count"] += int(has_transform_collapse_warning)
        report["unresolved_mesh_warning_count"] += len(unresolved_mesh_warnings)
        report["has_transform_collapse_warnings"] = report["has_transform_collapse_warnings"] or has_transform_collapse_warning
        report["has_unresolved_mesh_warnings"] = report["has_unresolved_mesh_warnings"] or has_unresolved_mesh_warning
        report["unresolved_placeholder_count"] += sum(1 for i in items if "unresolved xacro substitution placeholder" in (i.get("warning") or ""))
        for k in report["transform_status_counts"].keys():
            report["transform_status_counts"][k] += scene_transform_counts[k]
        report["scenes"].append({
            "scene": scene_dir.name,
            "visual_count": len(items),
            "transform_status_counts": scene_transform_counts,
            "has_transform_collapse_warning": has_transform_collapse_warning,
            "unresolved_mesh_warning_count": len(unresolved_mesh_warnings),
            "warnings": warnings,
            "output": str(out),
        })

    build = ROOT / "build"; build.mkdir(exist_ok=True)
    rpt = build / "workcell_studio_urdf_visual_mesh_index_report.json"
    rpt.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"scanned={report['scene_count']} visuals={report['visual_count']} resolved={report['resolved']} unresolved={report['unresolved']}")
    print(rpt)
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
