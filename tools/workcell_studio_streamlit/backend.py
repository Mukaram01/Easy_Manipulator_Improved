from __future__ import annotations

import json
import subprocess
import sys
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
