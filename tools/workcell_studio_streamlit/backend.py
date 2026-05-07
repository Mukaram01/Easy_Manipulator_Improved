from __future__ import annotations

import json
import subprocess
import sys
from html import escape
from pathlib import Path
from typing import Any

try:
    import yaml
except Exception:  # pragma: no cover
    yaml = None


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _load_yaml_file(path: Path) -> dict[str, Any]:
    if yaml is not None:
        loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
        return loaded if isinstance(loaded, dict) else {}
    script_path = repo_root() / "scripts"
    if str(script_path) not in sys.path:
        sys.path.insert(0, str(script_path))
    import validate_cell_definition as yaml_support

    loaded, _, _ = yaml_support.load_yaml(path)
    return loaded if isinstance(loaded, dict) else {}


def _catalog_entries(root: Path, group: str, key: str) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    directory = root / "catalog" / "capabilities" / group
    for path in sorted(directory.glob("*.yaml")):
        payload = _load_yaml_file(path)
        data = payload.get(key, {}) if isinstance(payload.get(key), dict) else {}
        out.append(
            {
                "file": str(path),
                "group": group,
                "id": data.get("id"),
                "label": data.get("label") or data.get("name"),
                "family": data.get("family") or data.get("strategy"),
                "runtime_supported": bool(data.get("runtime_supported", True)),
                "preview_only": bool(data.get("preview_only", False)),
                "compatible_families": data.get("compatible_tool_families")
                or data.get("supported_task_families")
                or [],
                "notes": data.get("limitations_warnings") or [],
                "raw": data,
            }
        )
    return out


def load_capability_catalog(root: Path | None = None) -> dict[str, list[dict[str, Any]]]:
    rr = root or repo_root()
    return {
        "robots": _catalog_entries(rr, "robots", "robot"),
        "end_effectors": _catalog_entries(rr, "end_effectors", "end_effector"),
        "sensors": _catalog_entries(rr, "sensors", "sensor"),
        "environment_assets": _catalog_entries(rr, "environment_assets", "asset"),
        "tasks": _catalog_entries(rr, "tasks", "task"),
    }


def load_grasp_strategy_catalog(root: Path | None = None) -> list[dict[str, Any]]:
    rr = root or repo_root()
    out: list[dict[str, Any]] = []
    for path in sorted((rr / "catalog" / "grasp_strategies").glob("*.yaml")):
        payload = _load_yaml_file(path)
        data = payload.get("grasp_strategy", {}) if isinstance(payload.get("grasp_strategy"), dict) else {}
        out.append(
            {
                "file": str(path),
                "id": data.get("id"),
                "label": data.get("label") or data.get("name"),
                "type": data.get("strategy"),
                "compatible_families": data.get("compatible_tool_families") or [],
                "runtime_supported": bool(data.get("runtime_supported", False)),
                "preview_only": True,
                "notes": data.get("limitations_warnings") or [],
                "raw": data,
            }
        )
    return out


def run_command(cmd: list[str], cwd: Path | None = None, timeout: int = 120) -> dict[str, Any]:
    proc = subprocess.run(
        cmd,
        cwd=str(cwd) if cwd else None,
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )
    return {
        "ok": proc.returncode == 0,
        "returncode": proc.returncode,
        "stdout": proc.stdout,
        "stderr": proc.stderr,
        "command": cmd,
    }


def _parse_json_output(run_result: dict[str, Any]) -> dict[str, Any]:
    out = dict(run_result)
    text = (run_result.get("stdout") or "").strip()
    if text:
        try:
            out["json"] = json.loads(text)
        except json.JSONDecodeError:
            out["json"] = None
    else:
        out["json"] = None
    return out


def validate_cell_definition(cell_definition_path: str | Path, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    path = Path(cell_definition_path)
    if not path.exists():
        return {"ok": False, "error": f"Missing cell_definition file: {path}"}
    cmd = [sys.executable, str(rr / "scripts" / "validate_cell_definition.py"), str(path), "--json"]
    return _parse_json_output(run_command(cmd, cwd=rr))


def validate_environment_layout(layout_path: str | Path, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    path = Path(layout_path)
    if not path.exists():
        return {"ok": False, "error": f"Missing environment layout file: {path}"}
    cmd = [sys.executable, str(rr / "scripts" / "validate_environment_layout.py"), str(path), "--json"]
    return _parse_json_output(run_command(cmd, cwd=rr))


def import_builder_scene(scene_package: str | Path, output_dir: str | Path, project_name: str, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    scene_path = Path(scene_package)
    out_dir = Path(output_dir)
    if not scene_path.exists() or not scene_path.is_dir():
        return {"ok": False, "error": f"Scene package path does not exist or is not a directory: {scene_path}"}
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable,
        str(rr / "scripts" / "workcell_studio.py"),
        "import-builder-scene",
        "--scene-package",
        str(scene_path),
        "--output-dir",
        str(out_dir),
        "--project-name",
        project_name,
        "--validate",
        "--generate-project",
    ]
    result = run_command(cmd, cwd=rr, timeout=300)
    summary = load_import_summary(out_dir)
    result["summary"] = summary
    return result


def load_import_summary(output_dir: str | Path) -> dict[str, Any]:
    out_dir = Path(output_dir)
    summary_json = out_dir / "workcell_studio_import_summary.json"
    summary_md = out_dir / "workcell_studio_import_summary.md"
    payload: dict[str, Any] = {
        "summary_json_path": str(summary_json) if summary_json.exists() else "",
        "summary_markdown_path": str(summary_md) if summary_md.exists() else "",
    }
    if summary_json.exists():
        payload["summary"] = json.loads(summary_json.read_text(encoding="utf-8"))
    else:
        payload["summary"] = {}
    return payload


def load_demo_catalog(root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    catalog = rr / "catalog" / "workcell_studio_demos.yaml"
    if not catalog.exists():
        return {"catalog_path": str(catalog), "demos": []}
    payload = _load_yaml_file(catalog)
    demos = payload.get("demos") if isinstance(payload.get("demos"), list) else []
    return {"catalog_path": str(catalog), "demos": demos}


def list_demos(root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    cmd = [sys.executable, str(rr / "scripts" / "workcell_studio.py"), "list-demos", "--json"]
    return _parse_json_output(run_command(cmd, cwd=rr))


def generate_demo_bundle(output_dir: str | Path, demo_id: str | None = None, all_demos: bool = False, force: bool = True, continue_on_error: bool = False, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    cmd = [sys.executable, str(rr / "scripts" / "workcell_studio.py"), "generate-demo-bundle", "--output-dir", str(output_dir)]
    if all_demos:
        cmd.append("--all")
    elif demo_id:
        cmd.extend(["--demo-id", demo_id])
    if force:
        cmd.append("--force")
    if continue_on_error:
        cmd.append("--continue-on-error")
    return _parse_json_output(run_command(cmd, cwd=rr, timeout=600))


def load_demo_bundle_summary(bundle_dir: str | Path) -> dict[str, Any]:
    b = Path(bundle_dir)
    js = b / "demo_bundle_summary.json"
    md = b / "demo_bundle_summary.md"
    out: dict[str, Any] = {
        "summary_json_path": str(js) if js.exists() else "",
        "summary_markdown_path": str(md) if md.exists() else "",
        "summary": {},
        "markdown": md.read_text(encoding="utf-8") if md.exists() else "",
    }
    if js.exists():
        out["summary"] = json.loads(js.read_text(encoding="utf-8"))
    return out


def generate_static_preview(cell_definition_path: str | Path, output_dir: str | Path, title: str, environment_layout_path: str | Path | None = None, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    cmd = [sys.executable, str(rr / "scripts" / "generate_workcell_static_preview.py"), "--cell-definition", str(cell_definition_path), "--output-dir", str(output_dir), "--title", title, "--json"]
    if environment_layout_path:
        cmd.extend(["--environment-layout", str(environment_layout_path)])
    return _parse_json_output(run_command(cmd, cwd=rr))


def load_static_preview_summary(preview_dir: str | Path) -> dict[str, Any]:
    p = Path(preview_dir)
    js = p / "static_preview_summary.json"
    return json.loads(js.read_text(encoding="utf-8")) if js.exists() else {}


def read_preview_html(preview_dir: str | Path) -> str:
    html_path = Path(preview_dir) / "static_preview.html"
    return html_path.read_text(encoding="utf-8") if html_path.exists() else ""


def convert_builder_task_intent_to_task_recipe(task_intent_path: str | Path, output_path: str | Path, scene_package: str | Path | None = None, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    cmd = [sys.executable, str(rr / "scripts" / "convert_builder_task_intent_to_task_recipe.py"), "--task-intent", str(task_intent_path), "--output", str(output_path), "--validate", "--json"]
    if scene_package:
        cmd.extend(["--scene-package", str(scene_package)])
    return _parse_json_output(run_command(cmd, cwd=rr))


def load_generated_task_recipe(path: str | Path) -> dict[str, Any]:
    p = Path(path)
    if not p.exists():
        return {}
    return _load_yaml_file(p)


def summarize_task_recipe(path: str | Path) -> dict[str, Any]:
    payload = load_generated_task_recipe(path)

def generate_plan_preview_request(task_recipe_path: str | Path, output_path: str | Path, cell_definition_path: str | Path | None = None, environment_layout_path: str | Path | None = None, allow_incomplete: bool = False, validate: bool = True, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    cmd = [sys.executable, str(rr / "scripts" / "generate_offline_plan_preview_request.py"), "--task-recipe", str(task_recipe_path), "--output", str(output_path), "--json"]
    if cell_definition_path:
        cmd.extend(["--cell-definition", str(cell_definition_path)])
    if environment_layout_path:
        cmd.extend(["--environment-layout", str(environment_layout_path)])
    if allow_incomplete:
        cmd.append("--allow-incomplete")
    if validate:
        cmd.append("--validate")
    return _parse_json_output(run_command(cmd, cwd=rr))


def validate_plan_preview_request(request_path: str | Path, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    cmd = [sys.executable, str(rr / "scripts" / "validate_offline_plan_preview_request.py"), str(request_path), "--json"]
    return _parse_json_output(run_command(cmd, cwd=rr))


def load_plan_preview_request(path: str | Path) -> dict[str, Any]:
    p = Path(path)
    return _load_yaml_file(p) if p.exists() else {}


def summarize_plan_preview_request(path: str | Path) -> dict[str, Any]:
    payload = load_plan_preview_request(path)
    req = payload.get("request", {}) if isinstance(payload.get("request"), dict) else {}
    return {
        "schema": payload.get("schema"),
        "pick_source": (req.get("pick") or {}).get("source_id"),
        "place_target": (req.get("place") or {}).get("target_id"),
        "grasp_strategy": (req.get("tool") or {}).get("grasp_strategy"),
        "waypoint_count": len(req.get("waypoints") or []),
        "safety": payload.get("safety", {}),
    }
    bti = payload.get("builder_task_intent", {}) if isinstance(payload.get("builder_task_intent"), dict) else {}
    task = payload.get("task", {}) if isinstance(payload.get("task"), dict) else {}
    return {
        "path": str(path),
        "task_id": task.get("id"),
        "task_type": task.get("type"),
        "pick_source": ((bti.get("pick", {}) or {}).get("source", {}) or {}).get("id"),
        "place_target": ((bti.get("place", {}) or {}).get("target", {}) or {}).get("id"),
        "grasp_strategy": (payload.get("grasp", {}) if isinstance(payload.get("grasp"), dict) else {}).get("strategy_ref"),
        "release_strategy": bti.get("release_strategy"),
        "routing_rules": len(task.get("rules", []) if isinstance(task.get("rules"), list) else []),
        "safety": bti.get("safety", {}),
    }


def resolve_catalog_choices(root: Path | None = None) -> dict[str, list[dict[str, str]]]:
    caps = load_capability_catalog(root)
    grasps = load_grasp_strategy_catalog(root)
    return {
        "robots": [{"id": x.get("id"), "label": x.get("label") or x.get("id")} for x in caps.get("robots", []) if x.get("id")],
        "end_effectors": [{"id": x.get("id"), "label": x.get("label") or x.get("id")} for x in caps.get("end_effectors", []) if x.get("id")],
        "sensors": [{"id": x.get("id"), "label": x.get("label") or x.get("id")} for x in caps.get("sensors", []) if x.get("id")],
        "tasks": [{"id": x.get("id"), "label": x.get("label") or x.get("id")} for x in caps.get("tasks", []) if x.get("id")],
        "grasp_strategies": [{"id": x.get("id"), "label": x.get("label") or x.get("id")} for x in grasps if x.get("id")],
    }


def create_cell(cell_id: str, robot: str, end_effector: str, sensor: str, task: str, grasp_strategy: str, output_dir: str | Path, validate: bool = True, preview: bool = True, generate_bundle: bool = False, allow_incompatible: bool = False, force: bool = True, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    cmd = [sys.executable, str(rr / "scripts" / "workcell_studio.py"), "create-cell", "--cell-id", cell_id, "--robot", robot, "--end-effector", end_effector, "--sensor", sensor, "--task", task, "--grasp-strategy", grasp_strategy, "--output-dir", str(output_dir)]
    if validate:
        cmd.append("--validate")
    if preview:
        cmd.append("--preview")
    if generate_bundle:
        cmd.append("--generate-bundle")
    if allow_incompatible:
        cmd.append("--allow-incompatible")
    if force:
        cmd.append("--force")
    result = _parse_json_output(run_command(cmd, cwd=rr, timeout=600))
    result["summary"] = load_create_cell_summary(output_dir)
    return result


def load_create_cell_summary(output_dir: str | Path) -> dict[str, Any]:
    out_dir = Path(output_dir)
    js = out_dir / "create_cell_summary.json"
    md = out_dir / "create_cell_summary.md"
    return {
        "summary_json_path": str(js) if js.exists() else "",
        "summary_markdown_path": str(md) if md.exists() else "",
        "summary": json.loads(js.read_text(encoding="utf-8")) if js.exists() else {},
        "markdown": md.read_text(encoding="utf-8") if md.exists() else "",
    }




def list_builder_scene_authoring_targets(scene_package: str | Path, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    cmd=[sys.executable, str(rr/"scripts"/"list_builder_scene_authoring_targets.py"), "--scene-package", str(scene_package), "--json"]
    return _parse_json_output(run_command(cmd, cwd=rr))

def create_or_update_builder_task_intent(scene_package: str | Path, task_id: str, task_type: str, pick_source: str, place_target: str, grasp_strategy: str, output_path: str | Path | None = None, approach_axis: str = "z_down", approach_distance_m: float = 0.1, retreat_axis: str = "z_up", retreat_distance_m: float = 0.1, release_strategy: str = "tool_release", object_class: str = "any", object_color: str = "any", validate: bool = True, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    cmd=[sys.executable, str(rr/"scripts"/"create_or_update_builder_task_intent.py"), "--scene-package", str(scene_package), "--task-id", task_id, "--task-type", task_type, "--pick-source", pick_source, "--place-target", place_target, "--grasp-strategy", grasp_strategy, "--approach-axis", approach_axis, "--approach-distance-m", str(approach_distance_m), "--retreat-axis", retreat_axis, "--retreat-distance-m", str(retreat_distance_m), "--release-strategy", release_strategy, "--object-class", object_class, "--object-color", object_color, "--json"]
    if output_path: cmd += ["--output", str(output_path)]
    if validate: cmd.append("--validate")
    return _parse_json_output(run_command(cmd, cwd=rr))

def create_or_update_environment_target(environment_layout_path: str | Path, target_id: str, target_type: str, label: str, frame: str, xyz: list[float], rpy: list[float], size: list[float], output_path: str | Path | None = None, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    out = output_path or environment_layout_path
    cmd=[sys.executable, str(rr/"scripts"/"create_or_update_environment_target.py"), "--environment-layout", str(environment_layout_path), "--target-id", target_id, "--target-type", target_type, "--label", label, "--frame", frame, "--xyz", *[str(x) for x in xyz], "--rpy", *[str(x) for x in rpy], "--size", *[str(x) for x in size], "--output", str(out), "--json"]
    return _parse_json_output(run_command(cmd, cwd=rr))

def load_environment_targets(environment_layout_path: str | Path) -> list[dict[str, Any]]:
    payload = load_environment_layout(environment_layout_path)
    return list_environment_targets(payload)


def load_environment_layout(path: str | Path) -> dict[str, Any]:
    p = Path(path)
    if not p.exists():
        return {}
    return _load_yaml_file(p)


def save_environment_layout(path: str | Path, data: dict[str, Any]) -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    if yaml is not None:
        p.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    else:
        p.write_text(_to_yaml(data) + "\n", encoding="utf-8")


def list_environment_targets(layout_or_path: dict[str, Any] | str | Path) -> list[dict[str, Any]]:
    payload = layout_or_path if isinstance(layout_or_path, dict) else load_environment_layout(layout_or_path)
    zones = payload.get("zones") if isinstance(payload, dict) else []
    return [z for z in (zones or []) if isinstance(z, dict) and z.get("id")]


def compute_scene_bounds(targets: list[dict[str, Any]]) -> dict[str, float]:
    if not targets:
        return {"min_x": -0.5, "max_x": 0.5, "min_y": -0.4, "max_y": 0.4}
    min_x, max_x, min_y, max_y = 9e9, -9e9, 9e9, -9e9
    for t in targets:
        pose = t.get("pose", {}) if isinstance(t.get("pose"), dict) else {}
        xyz = pose.get("xyz") if isinstance(pose.get("xyz"), list) else [0.0, 0.0, 0.0]
        size = t.get("size") if isinstance(t.get("size"), list) else [0.2, 0.2, 0.1]
        x = float(xyz[0]) if len(xyz) > 0 else 0.0
        y = float(xyz[1]) if len(xyz) > 1 else 0.0
        sx = max(float(size[0]) if len(size) > 0 else 0.2, 0.01)
        sy = max(float(size[1]) if len(size) > 1 else 0.2, 0.01)
        min_x = min(min_x, x - sx / 2)
        max_x = max(max_x, x + sx / 2)
        min_y = min(min_y, y - sy / 2)
        max_y = max(max_y, y + sy / 2)
    pad = 0.15
    return {"min_x": min_x - pad, "max_x": max_x + pad, "min_y": min_y - pad, "max_y": max_y + pad}


def render_topdown_targets_svg(targets: list[dict[str, Any]], width: int = 700, height: int = 500, scale_m_to_px: int = 500) -> str:
    safe_targets = sorted(targets, key=lambda t: str(t.get("id", "")))
    bounds = compute_scene_bounds(safe_targets)
    world_w = max(bounds["max_x"] - bounds["min_x"], width / scale_m_to_px)
    world_h = max(bounds["max_y"] - bounds["min_y"], height / scale_m_to_px)

    def to_px(x: float, y: float) -> tuple[float, float]:
        px = ((x - bounds["min_x"]) / world_w) * width
        py = height - (((y - bounds["min_y"]) / world_h) * height)
        return px, py

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect x="0" y="0" width="100%" height="100%" fill="#f8fafc"/>',
    ]
    for gx in range(0, width + 1, 50):
        lines.append(f'<line x1="{gx}" y1="0" x2="{gx}" y2="{height}" stroke="#e2e8f0" stroke-width="1"/>')
    for gy in range(0, height + 1, 50):
        lines.append(f'<line x1="0" y1="{gy}" x2="{width}" y2="{gy}" stroke="#e2e8f0" stroke-width="1"/>')
    origin_x, origin_y = to_px(0.0, 0.0)
    lines.append(f'<line x1="0" y1="{origin_y:.2f}" x2="{width}" y2="{origin_y:.2f}" stroke="#64748b" stroke-width="1.5"/>')
    lines.append(f'<line x1="{origin_x:.2f}" y1="0" x2="{origin_x:.2f}" y2="{height}" stroke="#64748b" stroke-width="1.5"/>')
    lines.append(f'<text x="{min(width-60, origin_x+4):.2f}" y="{max(14, origin_y-4):.2f}" font-size="11" fill="#334155">world (0,0)</text>')

    if not safe_targets:
        lines.append('<text x="18" y="28" font-size="14" fill="#475569">No targets discovered yet.</text>')
    for t in safe_targets:
        tid = escape(str(t.get("id", "")))
        ttype = str(t.get("type", ""))
        label = escape(str(t.get("label") or tid))
        pose = t.get("pose", {}) if isinstance(t.get("pose"), dict) else {}
        xyz = pose.get("xyz") if isinstance(pose.get("xyz"), list) else [0.0, 0.0, 0.0]
        x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2] if len(xyz) > 2 else 0.0)
        size = t.get("size") if isinstance(t.get("size"), list) else [0.2, 0.2, 0.1]
        sx = max(float(size[0]), 0.01); sy = max(float(size[1]), 0.01)
        cx, cy = to_px(x, y)
        rw = (sx / world_w) * width
        rh = (sy / world_h) * height
        rx, ry = cx - rw/2, cy - rh/2
        color = "#16a34a" if ttype == "pick_zone" else "#2563eb" if ttype in {"place_target", "bin"} else "#7c3aed"
        lines.append(f'<rect x="{rx:.2f}" y="{ry:.2f}" width="{rw:.2f}" height="{rh:.2f}" fill="{color}" fill-opacity="0.20" stroke="{color}" stroke-width="2"/>')
        lines.append(f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="3" fill="{color}"/>')
        lines.append(f'<text x="{(rx+4):.2f}" y="{max(12, ry-6):.2f}" font-size="11" fill="#0f172a">{label} ({tid}) [{escape(ttype)}] xyz=({x:.2f},{y:.2f},{z:.2f})</text>')
    lines.append("</svg>")
    return "\n".join(lines)

def summarize_environment_targets(environment_layout_path: str | Path) -> dict[str, Any]:
    zones = load_environment_targets(environment_layout_path)
    return {"pick_sources":[z.get("id") for z in zones if str(z.get("type","")).lower() in {"pick","pick_zone"}], "place_targets":[z.get("id") for z in zones if str(z.get("type","")).lower() in {"place","place_target","bin"}], "count":len(zones)}
def default_builder_task_intent(scene_package: str = "") -> dict[str, Any]:
    return {"schema":"workcell_builder_task_intent/v1","scene_package":scene_package,"task":{"id":"default_builder_task","type":"pick_place","mode":"offline_preview"},"pick":{"source":{"type":"zone","id":"pick_zone_main"},"object_filter":{"class_id":"any","color":"any"}},"grasp":{"strategy_ref":"suction_top_basic","approach_axis":"z_down","approach_distance_m":0.1,"retreat_axis":"z_up","retreat_distance_m":0.1},"place":{"target":{"type":"destination","id":"bin_main"},"release_strategy":"tool_release","place_offset_xyz":[0.0,0.0,0.05]},"routing":{"rules":[]},"safety":{"metadata_only":True,"runtime_io_applied":False,"motion_started":False,"ros_launch_started":False}}

def find_builder_task_intent(scene_package: str | Path) -> str:
    sp=Path(scene_package)
    for rel in ["generated/workcell_builder_task_intent.yaml","workcell_builder_task_intent.yaml"]:
        p=sp/rel
        if p.exists(): return str(p)
    return ""

def load_builder_task_intent(path: str | Path) -> dict[str, Any]:
    p=Path(path)
    if not p.exists(): return {}
    return _load_yaml_file(p)

def _yaml_scalar(v: Any) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    if v is None:
        return "null"
    if isinstance(v, (int, float)):
        return str(v)
    return json.dumps(str(v))

def _to_yaml(value: Any, indent: int = 0) -> str:
    sp = " " * indent
    if isinstance(value, dict):
        lines = []
        for k, v in value.items():
            if isinstance(v, (dict, list)):
                lines.append(f"{sp}{k}:")
                lines.append(_to_yaml(v, indent + 2))
            else:
                lines.append(f"{sp}{k}: {_yaml_scalar(v)}")
        return "\n".join(lines)
    if isinstance(value, list):
        lines = []
        for x in value:
            if isinstance(x, (dict, list)):
                lines.append(f"{sp}-")
                lines.append(_to_yaml(x, indent + 2))
            else:
                lines.append(f"{sp}- {_yaml_scalar(x)}")
        return "\n".join(lines)
    return f"{sp}{_yaml_scalar(value)}"

def save_builder_task_intent(path: str | Path, payload: dict[str, Any]) -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    if yaml is not None:
        p.write_text(yaml.safe_dump(payload, sort_keys=False), encoding='utf-8')
    else:
        p.write_text(_to_yaml(payload) + "\n", encoding='utf-8')

def validate_builder_task_intent(task_intent_path: str | Path, scene_package: str | Path | None = None, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    cmd=[sys.executable, str(rr/"scripts"/"validate_builder_task_intent.py"), str(task_intent_path), "--json"]
    if scene_package: cmd += ["--scene-package", str(scene_package)]
    return _parse_json_output(run_command(cmd, cwd=rr))


def summarize_task_flow(task_intent_path: str | Path | None = None, task_recipe_path: str | Path | None = None, scene_package: str | Path | None = None, environment_layout_path: str | Path | None = None, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    cmd = [sys.executable, str(rr / "scripts" / "summarize_task_flow.py"), "--json"]
    if task_intent_path: cmd += ["--task-intent", str(task_intent_path)]
    if task_recipe_path: cmd += ["--task-recipe", str(task_recipe_path)]
    if scene_package: cmd += ["--scene-package", str(scene_package)]
    if environment_layout_path: cmd += ["--environment-layout", str(environment_layout_path)]
    return _parse_json_output(run_command(cmd, cwd=rr))

def load_task_flow_summary(path: str | Path) -> dict[str, Any]:
    p = Path(path)
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else {}

def generate_static_preview_with_task_flow(cell_definition_path: str | Path, output_dir: str | Path, title: str, task_intent_path: str | Path | None = None, task_recipe_path: str | Path | None = None, environment_layout_path: str | Path | None = None, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    cmd = [sys.executable, str(rr / "scripts" / "generate_workcell_static_preview.py"), "--cell-definition", str(cell_definition_path), "--output-dir", str(output_dir), "--title", title, "--json"]
    if environment_layout_path: cmd += ["--environment-layout", str(environment_layout_path)]
    if task_intent_path: cmd += ["--task-intent", str(task_intent_path)]
    if task_recipe_path: cmd += ["--task-recipe", str(task_recipe_path)]
    return _parse_json_output(run_command(cmd, cwd=rr))


def prepare_rviz_plan_preview(scene_package: str | Path, plan_preview_request: str | Path, output_dir: str | Path, allow_missing_launch: bool = True, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    cmd = [sys.executable, str(rr / "scripts" / "workcell_studio.py"), "prepare-rviz-plan-preview", "--scene-package", str(scene_package), "--plan-preview-request", str(plan_preview_request), "--output-dir", str(output_dir), "--json"]
    if allow_missing_launch:
        cmd.append("--allow-missing-launch")
    return _parse_json_output(run_command(cmd, cwd=rr))


def validate_rviz_plan_preview_session(session_path: str | Path, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    cmd = [sys.executable, str(rr / "scripts" / "workcell_studio.py"), "validate-rviz-plan-preview", "--session", str(session_path), "--json"]
    return _parse_json_output(run_command(cmd, cwd=rr))


def load_rviz_plan_preview_session(output_dir: str | Path) -> dict[str, Any]:
    d = Path(output_dir)
    p = d / "rviz_moveit_plan_preview_session.json"
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else {}


def read_suggested_commands(output_dir: str | Path) -> str:
    p = Path(output_dir) / "suggested_commands.sh"
    return p.read_text(encoding="utf-8") if p.exists() else ""

def smoke_launch_preview(session_path: str | Path, output_dir: str | Path, execute: bool = False, timeout_s: int = 20, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    cmd = [sys.executable, str(rr / "scripts" / "workcell_studio.py"), "smoke-launch-preview", "--session", str(session_path), "--output-dir", str(output_dir), "--timeout-s", str(timeout_s), "--json"]
    cmd.append("--execute" if execute else "--dry-run")
    return _parse_json_output(run_command(cmd, cwd=rr, timeout=max(120, timeout_s + 30)))

def validate_smoke_launch_report(report_path: str | Path, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    cmd = [sys.executable, str(rr / "scripts" / "workcell_studio.py"), "validate-smoke-launch-report", "--report", str(report_path), "--json"]
    return _parse_json_output(run_command(cmd, cwd=rr))

def load_smoke_launch_report(output_dir: str | Path) -> dict[str, Any]:
    p = Path(output_dir) / "fake_hardware_smoke_launch_report.json"
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else {}

def read_smoke_launch_logs(output_dir: str | Path, max_chars: int = 4000) -> dict[str, str]:
    d = Path(output_dir)
    out = (d / "captured_stdout.log").read_text(encoding="utf-8") if (d / "captured_stdout.log").exists() else ""
    err = (d / "captured_stderr.log").read_text(encoding="utf-8") if (d / "captured_stderr.log").exists() else ""
    return {"stdout": out[-max_chars:], "stderr": err[-max_chars:]}


def check_planning_scene_readiness(scene_package: str | Path, output_dir: str | Path, cell_definition: str | Path | None = None, task_recipe: str | Path | None = None, plan_preview_request: str | Path | None = None, plan_preview_session: str | Path | None = None, smoke_report: str | Path | None = None, strict: bool = False, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    cmd = [sys.executable, str(rr / "scripts" / "workcell_studio.py"), "check-planning-scene-readiness", "--scene-package", str(scene_package), "--output-dir", str(output_dir), "--json"]
    if cell_definition: cmd += ["--cell-definition", str(cell_definition)]
    if task_recipe: cmd += ["--task-recipe", str(task_recipe)]
    if plan_preview_request: cmd += ["--plan-preview-request", str(plan_preview_request)]
    if plan_preview_session: cmd += ["--plan-preview-session", str(plan_preview_session)]
    if smoke_report: cmd += ["--smoke-report", str(smoke_report)]
    if strict: cmd.append("--strict")
    return _parse_json_output(run_command(cmd, cwd=rr, timeout=300))

def validate_planning_scene_readiness_report(report: str | Path, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    cmd = [sys.executable, str(rr / "scripts" / "workcell_studio.py"), "validate-planning-scene-readiness", "--report", str(report), "--json"]
    return _parse_json_output(run_command(cmd, cwd=rr))

def load_planning_scene_readiness_report(output_dir: str | Path) -> dict[str, Any]:
    d = Path(output_dir)
    js = d / "planning_scene_readiness_report.json"
    md = d / "planning_scene_readiness_report.md"
    return {"report_json_path": str(js) if js.exists() else "", "report_markdown_path": str(md) if md.exists() else "", "report": json.loads(js.read_text(encoding="utf-8")) if js.exists() else {}}

def generate_readiness_pack(scene_package: str | Path, output_dir: str | Path, project_name: str, validate: bool = True, prepare_rviz_preview: bool = False, smoke_dry_run: bool = True, strict: bool = False, continue_on_error: bool = False, smoke_execute: bool = False, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    cmd = [sys.executable, str(rr / "scripts" / "workcell_studio.py"), "generate-readiness-pack", "--scene-package", str(scene_package), "--output-dir", str(output_dir), "--project-name", project_name, "--force", "--json"]
    if validate: cmd.append("--validate")
    if prepare_rviz_preview: cmd.append("--prepare-rviz-preview")
    if smoke_dry_run: cmd.append("--smoke-dry-run")
    if strict: cmd.append("--strict")
    if continue_on_error: cmd.append("--continue-on-error")
    if smoke_execute: cmd.append("--smoke-execute")
    return _parse_json_output(run_command(cmd, cwd=rr, timeout=600))

def validate_readiness_pack(manifest: str | Path, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    return _parse_json_output(run_command([sys.executable, str(rr / "scripts" / "workcell_studio.py"), "validate-readiness-pack", "--manifest", str(manifest), "--json"], cwd=rr))

def load_readiness_pack_manifest(output_dir: str | Path) -> dict[str, Any]:
    p = Path(output_dir) / "readiness_pack_manifest.json"
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else {}

def read_readiness_pack_summary(output_dir: str | Path) -> str:
    p = Path(output_dir) / "readiness_pack_summary.md"
    return p.read_text(encoding="utf-8") if p.exists() else ""

def read_readiness_pack_next_commands(output_dir: str | Path) -> str:
    p = Path(output_dir) / "next_commands.md"
    return p.read_text(encoding="utf-8") if p.exists() else ""


def dashboard_path_from_manifest(manifest: dict[str, Any]) -> str:
    arts = manifest.get("artifacts", {}) if isinstance(manifest.get("artifacts"), dict) else {}
    return str(arts.get("readiness_dashboard", ""))

def generate_readiness_dashboard(manifest: str | Path, output: str | Path, root: Path | None = None) -> dict[str, Any]:
    rr = root or repo_root()
    cmd = [sys.executable, str(rr / "scripts" / "workcell_studio.py"), "generate-readiness-dashboard", "--manifest", str(manifest), "--output", str(output), "--json"]
    return _parse_json_output(run_command(cmd, cwd=rr))

def read_readiness_dashboard(path: str | Path, max_chars: int = 12000) -> str:
    p = Path(path)
    if not p.exists():
        return ""
    text = p.read_text(encoding="utf-8")
    return text[:max_chars]
