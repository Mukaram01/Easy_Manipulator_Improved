#!/usr/bin/env python3
"""Generate an offline static HTML dashboard for a workcell project manifest."""

from __future__ import annotations

import argparse
import hashlib
import html
import json
import os
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


@dataclass
class DashboardResult:
    status: str
    output_path: Path
    warnings: list[str]
    errors: list[str]
    project_dir: Path
    manifest_path: Path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while True:
            chunk = handle.read(65536)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def _escape(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, (dict, list)):
        text = json.dumps(value, ensure_ascii=False)
    else:
        text = str(value)
    return html.escape(text)


def _status_rank(value: str) -> int:
    order = {"PASS": 0, "WARN": 1, "SKIP": 1, "UNKNOWN": 1, "FAIL": 2}
    return order.get((value or "UNKNOWN").upper(), 1)


def _overall_status(statuses: dict[str, Any]) -> str:
    if not isinstance(statuses, dict) or not statuses:
        return "UNKNOWN"
    resolved = [str(v).upper() for v in statuses.values()]
    if any(v == "FAIL" for v in resolved):
        return "FAIL"
    if any(v in {"WARN", "SKIP", "UNKNOWN"} for v in resolved):
        return "WARN"
    if all(v == "PASS" for v in resolved):
        return "PASS"
    return "UNKNOWN"


def _resolve_link(project_dir: Path, output_dir: Path, rel: str) -> tuple[str, bool]:
    rel_path = Path(rel)
    if rel_path.is_absolute():
        return "#", False
    target = (project_dir / rel_path).resolve()
    exists = target.exists()
    href = Path(os.path.relpath(target, output_dir.resolve()))
    return _escape(href.as_posix()), exists


def _rows_from_artifacts(manifest: dict[str, Any], project_dir: Path, output_dir: Path) -> list[dict[str, str]]:
    artifacts = manifest.get("artifacts", {})
    checksums = manifest.get("checksums", {})
    rows: list[dict[str, str]] = []
    if isinstance(artifacts, dict):
        for key, rel in sorted(artifacts.items()):
            rel_text = str(rel)
            href, exists = _resolve_link(project_dir, output_dir, rel_text)
            rows.append(
                {
                    "name": _escape(key),
                    "path": _escape(rel_text),
                    "href": href,
                    "exists": "YES" if exists else "NO",
                    "checksum": _escape(checksums.get(key, "")) if isinstance(checksums, dict) else "",
                }
            )
    return rows


def _render_status_badge(status: str) -> str:
    text = (status or "UNKNOWN").upper()
    css = "unknown"
    if text == "PASS":
        css = "pass"
    elif text in {"WARN", "SKIP", "UNKNOWN"}:
        css = "warn"
    elif text == "FAIL":
        css = "fail"
    return f'<span class="badge {css}">{_escape(text)}</span>'


def _load_next_steps(project_dir: Path, manifest: dict[str, Any]) -> str:
    artifacts = manifest.get("artifacts", {})
    candidates = []
    if isinstance(artifacts, dict) and artifacts.get("next_commands"):
        candidates.append(project_dir / str(artifacts["next_commands"]))
    candidates.append(project_dir / "next_commands.md")
    for path in candidates:
        if path.is_file():
            return path.read_text(encoding="utf-8")
    return "No next command file found."


def _render_html(manifest: dict[str, Any], project_dir: Path, manifest_path: Path, output_path: Path, warnings: list[str]) -> str:
    output_dir = output_path.parent
    statuses = manifest.get("statuses", {}) if isinstance(manifest.get("statuses"), dict) else {}
    overall = _overall_status(statuses)

    cell = manifest.get("cell", {}) if isinstance(manifest.get("cell"), dict) else {}
    robot = manifest.get("robot", {}) if isinstance(manifest.get("robot"), dict) else {}
    ee = manifest.get("end_effector", {}) if isinstance(manifest.get("end_effector"), dict) else {}
    env = manifest.get("environment", {}) if isinstance(manifest.get("environment"), dict) else {}
    task = manifest.get("task", {}) if isinstance(manifest.get("task"), dict) else {}
    task_recipe = manifest.get("task_recipe", {}) if isinstance(manifest.get("task_recipe"), dict) else {}

    artifact_rows = _rows_from_artifacts(manifest, project_dir, output_dir)

    reports: list[str] = []
    artifacts = manifest.get("artifacts", {}) if isinstance(manifest.get("artifacts"), dict) else {}
    for name, rel in sorted(artifacts.items()):
        if "report" in name or "summary" in name or str(rel).startswith("reports/"):
            href, exists = _resolve_link(project_dir, output_dir, str(rel))
            marker = "✓" if exists else "✗"
            reports.append(f"<li>{_escape(name)} ({marker}) - <a href=\"{href}\">{_escape(rel)}</a></li>")

    next_steps = _escape(_load_next_steps(project_dir, manifest))

    status_rows = "".join(
        f"<tr><td>{_escape(name)}</td><td>{_render_status_badge(str(value))}</td></tr>" for name, value in sorted(statuses.items())
    ) or "<tr><td colspan='2'>No status entries</td></tr>"

    artifact_table = "".join(
        "<tr>"
        f"<td>{row['name']}</td>"
        f"<td><a href=\"{row['href']}\">{row['path']}</a></td>"
        f"<td>{row['exists']}</td>"
        f"<td><code>{row['checksum']}</code></td>"
        "</tr>"
        for row in artifact_rows
    ) or "<tr><td colspan='4'>No artifacts listed in manifest.</td></tr>"

    warning_list = "".join(f"<li>{_escape(item)}</li>" for item in warnings) or "<li>(none)</li>"
    report_list = "".join(reports) or "<li>No report artifacts discovered.</li>"

    return f"""<!doctype html>
<html lang=\"en\">
<head>
<meta charset=\"utf-8\">
<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">
<title>Workcell Project Dashboard</title>
<style>
body {{ font-family: Arial, sans-serif; margin: 1.2rem; color: #1f2937; background: #fafafa; }}
h1, h2 {{ margin-top: 1.5rem; }}
.card {{ background: #fff; border: 1px solid #ddd; border-radius: 8px; padding: 0.9rem; margin-bottom: 1rem; }}
.grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); gap: 0.8rem; }}
small {{ color: #555; }}
.badge {{ border-radius: 999px; padding: 0.15rem 0.6rem; font-size: 0.8rem; color: white; font-weight: bold; }}
.badge.pass {{ background: #15803d; }}
.badge.warn {{ background: #b45309; }}
.badge.fail {{ background: #b91c1c; }}
.badge.unknown {{ background: #374151; }}
table {{ width: 100%; border-collapse: collapse; }}
th, td {{ border: 1px solid #d1d5db; padding: 0.45rem; text-align: left; vertical-align: top; }}
code, pre {{ background: #f3f4f6; border: 1px solid #e5e7eb; border-radius: 4px; padding: 0.1rem 0.2rem; }}
pre {{ padding: 0.6rem; overflow-x: auto; white-space: pre-wrap; }}
ul {{ margin-top: 0.2rem; }}
</style>
</head>
<body>
<h1>Workcell Project Dashboard</h1>
<p><small>Read-only offline project summary generated at {_escape(datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'))}.</small></p>

<section class=\"card\">
<h2>Project Summary</h2>
<div class=\"grid\">
<div><strong>Project name:</strong> {_escape(manifest.get('cell_name') or cell.get('name') or manifest.get('cell_id') or 'UNKNOWN')}</div>
<div><strong>Package name:</strong> {_escape(manifest.get('generated_package_name', 'UNKNOWN'))}</div>
<div><strong>Schema version:</strong> {_escape(manifest.get('schema_version', 'UNKNOWN'))}</div>
<div><strong>Generated timestamp:</strong> {_escape(manifest.get('generated_at_utc', 'UNKNOWN'))}</div>
<div><strong>Overall status:</strong> {_render_status_badge(overall)}</div>
<div><strong>Source cell definition:</strong> {_escape(manifest.get('source_path', 'UNKNOWN'))}</div>
</div>
<p><small>Manifest: {_escape(str(manifest_path))}</small></p>
</section>

<section class=\"card\">
<h2>Cell Overview</h2>
<ul>
<li>Robot model: {_escape(robot.get('model', manifest.get('robot_model', 'UNKNOWN')))}</li>
<li>End effector / gripper: {_escape(ee.get('id', manifest.get('end_effector_id', 'UNKNOWN')))}</li>
<li>Planning group: {_escape(robot.get('planning_group', manifest.get('planning_group', 'UNKNOWN')))}</li>
<li>Frames: base={_escape(robot.get('base_frame', manifest.get('base_frame', 'UNKNOWN')))}, tool={_escape(robot.get('ee_link', manifest.get('tool_frame', 'UNKNOWN')))}, grasp={_escape(ee.get('grasp_frame', manifest.get('grasp_frame', 'UNKNOWN')))}</li>
</ul>
</section>

<section class=\"card\">
<h2>Environment Overview</h2>
<ul>
<li>Support/table objects: {_escape(env.get('supports') or manifest.get('support_objects') or 'UNKNOWN')}</li>
<li>Camera/sensors: {_escape(manifest.get('camera') or env.get('sensors') or 'UNKNOWN')}</li>
<li>Scene/generated package path: {_escape(artifacts.get('generated_package', 'UNKNOWN'))}</li>
</ul>
</section>

<section class=\"card\">
<h2>Task Overview</h2>
<ul>
<li>Task type: {_escape(task.get('type', manifest.get('task_type', 'UNKNOWN')))}</li>
<li>Destinations: {_escape(task.get('destinations', 'UNKNOWN'))}</li>
<li>Decision rules: {_escape(task.get('decision_rules', manifest.get('task_rules', 'UNKNOWN')))}</li>
<li>Task recipe mode/type: {_escape(task_recipe.get('mode', 'none'))} / {_escape(task_recipe.get('type', 'UNKNOWN'))}</li>
<li>Task recipe fallback: {_escape(task_recipe.get('fallback_present', 'UNKNOWN'))}</li>
<li>Task recipe preview: <a href=\"{_resolve_link(project_dir, output_dir, str(artifacts.get('task_preview', '#')))[0]}\">{_escape(artifacts.get('task_preview', 'UNKNOWN'))}</a></li>
<li>Task execution plan: <a href=\"{_resolve_link(project_dir, output_dir, str(artifacts.get('execution_plan_md', '#')))[0]}\">{_escape(artifacts.get('execution_plan_md', 'UNKNOWN'))}</a></li>
</ul>
</section>

<section class=\"card\">
<h2>Validation and Commissioning Status</h2>
<table>
<thead><tr><th>Step</th><th>Status</th></tr></thead>
<tbody>{status_rows}</tbody>
</table>
<h3>Generated reports</h3>
<ul>{report_list}</ul>
<h3>Warnings</h3>
<ul>{warning_list}</ul>
</section>

<section class=\"card\">
<h2>Generated Artifacts</h2>
<table>
<thead><tr><th>Artifact</th><th>Relative path</th><th>Exists</th><th>Checksum</th></tr></thead>
<tbody>{artifact_table}</tbody>
</table>
</section>

<section class=\"card\">
<h2>Operator Next Steps</h2>
<ol>
<li>Review cell definition</li>
<li>Review scene manifest</li>
<li>Review task recipe</li>
<li>Review commissioning bundle</li>
<li>Run offline validation</li>
<li>Only then test runtime launch</li>
</ol>
<pre>{next_steps}</pre>
</section>

<section class=\"card\">
<h2>Notes / Limitations</h2>
<ul>
<li>The dashboard is a read-only offline project summary. It does not replace the existing GUI and does not control the robot.</li>
<li>This page is static and offline (no external JavaScript/CSS/CDN dependencies).</li>
<li>No runtime control, planner control, or ROS launch action is performed here.</li>
</ul>
</section>

</body>
</html>
"""


def generate_dashboard(project_dir: Path, manifest_path: Path, output_path: Path) -> DashboardResult:
    warnings: list[str] = []
    errors: list[str] = []

    if not manifest_path.is_file():
        errors.append(f"Missing manifest file: {manifest_path}")
        return DashboardResult("FAIL", output_path, warnings, errors, project_dir, manifest_path)

    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except Exception as exc:
        errors.append(f"Invalid manifest JSON: {exc}")
        return DashboardResult("FAIL", output_path, warnings, errors, project_dir, manifest_path)

    for field in ("schema_version", "artifacts", "statuses"):
        if field not in manifest:
            warnings.append(f"Manifest missing optional dashboard field: {field}")

    html_text = _render_html(manifest, project_dir, manifest_path, output_path, warnings + list(manifest.get("warnings", [])))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(html_text, encoding="utf-8")

    status = "FAIL" if errors else ("WARN" if warnings else "PASS")
    return DashboardResult(status, output_path, warnings, errors, project_dir, manifest_path)


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-dir", type=Path, help="Generated workcell project directory containing project_manifest.json")
    parser.add_argument("--manifest", type=Path, help="Direct path to a project_manifest.json file")
    parser.add_argument("--output", type=Path, help="Output HTML path (default: <project-dir>/dashboard/index.html)")
    parser.add_argument("--json", action="store_true", help="Print machine-readable summary")
    parser.add_argument("--strict", action="store_true", help="Treat warnings as failures")
    parser.add_argument("--quiet", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    if not args.project_dir and not args.manifest:
        print("FAIL: provide --project-dir or --manifest")
        return 2

    if args.project_dir:
        project_dir = args.project_dir.resolve()
        manifest_path = args.manifest.resolve() if args.manifest else project_dir / "project_manifest.json"
    else:
        manifest_path = args.manifest.resolve()
        project_dir = manifest_path.parent

    output_path = args.output.resolve() if args.output else (project_dir / "dashboard" / "index.html")

    result = generate_dashboard(project_dir, manifest_path, output_path)
    summary = {
        "status": result.status,
        "project_dir": str(result.project_dir),
        "manifest_path": str(result.manifest_path),
        "dashboard_path": str(result.output_path),
        "dashboard_checksum": _sha256(result.output_path) if result.output_path.is_file() else "",
        "warnings": result.warnings,
        "errors": result.errors,
    }

    if not args.quiet:
        print(f"Workcell dashboard: {result.status}")
        print(f"manifest: {result.manifest_path}")
        print(f"output: {result.output_path}")
        if result.warnings:
            print("warnings:")
            for note in result.warnings:
                print(f" - {note}")
        if result.errors:
            print("errors:")
            for note in result.errors:
                print(f" - {note}")

    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=True))

    if result.errors:
        return 1
    if args.strict and result.warnings:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
