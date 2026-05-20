#!/usr/bin/env python3
from __future__ import annotations
import json, os, re, shutil, subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]
SCENES_ROOT = ROOT / "scenes"


def read_yaml(path: Path):
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8")) if path.exists() else None
    except Exception:
        return None


def parse_vec(text: str | None, n: int, default: float = 0.0):
    vals = [default] * n
    if not text:
        return vals
    parts = re.split(r"\s+", text.strip())
    for i in range(min(n, len(parts))):
        try: vals[i] = float(parts[i])
        except Exception: pass
    return vals


def cat(link: str):
    low = link.lower()
    if any(t in low for t in ["ur", "robot", "base", "arm"]): return "robot_visual"
    if any(t in low for t in ["gripper", "tool", "ee", "suction", "hand"]): return "gripper_visual"
    if any(t in low for t in ["table", "conveyor", "camera", "fixture", "zone", "cell", "env"]): return "environment_visual"
    if "object" in low or "bin" in low: return "object_visual"
    return "unknown_visual"


def collect_package_roots(scene_dir: Path):
    roots = [scene_dir, ROOT, ROOT / "assets"]
    ws_src = ROOT.parent / "src"
    if ws_src.exists(): roots.append(ws_src)
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(":"):
        if not prefix: continue
        p = Path(prefix) / "share"
        if p.exists(): roots.append(p)
    return roots


def resolve_uri(uri: str, scene_dir: Path, roots):
    p = Path(uri)
    if uri.startswith("package://"):
        rest = uri[len("package://"):]
        if "/" in rest:
            pkg, rel = rest.split("/", 1)
            for r in roots:
                c = r / pkg / rel
                if c.exists(): return str(c.resolve())
    elif p.is_absolute() and p.exists():
        return str(p)
    else:
        cands = [scene_dir / uri, scene_dir / "assets" / uri, ROOT / uri]
        for c in cands:
            if c.exists(): return str(c.resolve())
    return ""


def expand_xacro(path: Path):
    if shutil.which("xacro") is None:
        return None, "best_effort", ["xacro executable unavailable; using best_effort parsing"]
    try:
        out = subprocess.run(["xacro", str(path)], check=True, capture_output=True, text=True, timeout=30)
        return out.stdout, "xacro_expanded", []
    except Exception as exc:
        return None, "best_effort", [f"xacro expansion failed: {exc}"]


def extract(xml_text: str, scene_name: str, scene_dir: Path, roots):
    items, warnings = [], []
    try:
        root = ET.fromstring(xml_text)
    except Exception as exc:
        warnings.append(f"xml parse failed: {exc}")
        for idx, m in enumerate(re.finditer(r"<mesh[^>]*filename=[\"']([^\"']+)[\"'][^>]*/?>", xml_text)):
            uri = m.group(1)
            src = resolve_uri(uri, scene_dir, roots)
            warn = "" if src else f"unresolved mesh path: {uri}"
            items.append({"id": f"urdf_visual_regex_{idx}", "link": "unknown_link", "visual": "", "source_path": src, "package_uri": uri,
                          "pose": {"xyz": [0.0,0.0,0.0], "rpy": [0.0,0.0,0.0]}, "scale": [1.0,1.0,1.0], "material": {},
                          "category": "unknown_visual", "resolved": bool(src), "warning": warn})
            if warn: warnings.append(warn)
        return items, warnings
    idx = 0
    for link in root.findall('.//link'):
        lname = link.attrib.get('name', 'unknown_link')
        for visual in link.findall('visual'):
            geo = visual.find('geometry')
            if geo is None: continue
            mesh = geo.find('mesh')
            if mesh is None: continue
            package_uri = mesh.attrib.get('filename', '').strip()
            if not package_uri: continue
            origin = visual.find('origin')
            xyz = parse_vec(origin.attrib.get('xyz') if origin is not None else None, 3, 0.0)
            rpy = parse_vec(origin.attrib.get('rpy') if origin is not None else None, 3, 0.0)
            scale = parse_vec(mesh.attrib.get('scale'), 3, 1.0)
            material = visual.find('material')
            color = None
            if material is not None and material.find('color') is not None:
                color = material.find('color').attrib.get('rgba')
            src = resolve_uri(package_uri, scene_dir, roots)
            warn = "" if src else f"unresolved mesh path: {package_uri}"
            items.append({
                "id": f"urdf_visual_{idx}_{lname}", "link": lname, "visual": visual.attrib.get('name', ''),
                "source_path": src, "package_uri": package_uri,
                "pose": {"xyz": xyz, "rpy": rpy}, "scale": scale,
                "material": {"color": color} if color else {}, "category": cat(lname),
                "resolved": bool(src), "warning": warn,
            })
            if warn: warnings.append(warn)
            idx += 1
    return items, warnings


def main():
    scenes = sorted([p for p in SCENES_ROOT.iterdir() if p.is_dir()])
    report = {"scene_count": 0, "visual_count": 0, "resolved": 0, "unresolved": 0, "scenes": []}
    for scene_dir in scenes:
        manifest = read_yaml(scene_dir / "scene_manifest.yaml") or {}
        urdf_rel = ((manifest.get("files") or {}).get("urdf_xacro") or "urdf/scene.urdf.xacro")
        urdf_path = scene_dir / urdf_rel
        mode = "best_effort"
        warnings = []
        items = []
        if urdf_path.exists():
            expanded, mode, w = expand_xacro(urdf_path)
            warnings.extend(w)
            text = expanded if expanded else urdf_path.read_text(encoding="utf-8", errors="ignore")
            roots = collect_package_roots(scene_dir)
            items, xw = extract(text, scene_dir.name, scene_dir, roots)
            warnings.extend(xw)
            if not items:
                stls = list(scene_dir.glob("assets/**/*.stl"))[:50]
                for i, stl in enumerate(stls):
                    items.append({"id": f"urdf_visual_asset_{i}", "link": stl.stem, "visual": "", "source_path": str(stl.resolve()),
                                  "package_uri": str(stl), "pose": {"xyz": [0,0,0], "rpy": [0,0,0]}, "scale": [1,1,1],
                                  "material": {}, "category": cat(stl.stem), "resolved": True, "warning": ""})
                if stls:
                    warnings.append("best_effort fallback used scene assets because URDF parsing found no mesh tags")
                if not items:
                    items.append({"id":"urdf_visual_unresolved_0","link":"unknown_link","visual":"","source_path":"","package_uri":"","pose":{"xyz":[0,0,0],"rpy":[0,0,0]},"scale":[1,1,1],"material":{},"category":"unknown_visual","resolved":False,"warning":"no mesh references discovered in URDF/Xacro best-effort parse"})
        else:
            warnings.append(f"missing urdf_xacro: {urdf_path}")
        gen = {"scene_name": scene_dir.name, "source_urdf_xacro_path": str(urdf_path), "extraction_mode": mode, "visual_items": items, "warnings": warnings}
        out = scene_dir / "generated" / "scene_visual_mesh_index.json"
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(gen, indent=2), encoding="utf-8")
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
