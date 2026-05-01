#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, subprocess, sys
from pathlib import Path


def _load_summary(workcell: Path) -> dict:
    path = workcell / "generated" / "generated_workcell_summary.json"
    if not path.exists():
        raise SystemExit(f"FAIL: missing generated summary: {path}")
    return json.loads(path.read_text(encoding='utf-8'))


def build_command(summary: dict, output_dir: Path, dry_run: bool, no_replay: bool, gated: bool, as_json: bool) -> list[str]:
    cmd = [sys.executable, str(Path(__file__).resolve().parent / 'run_generated_cell_cycle.py'), '--scene-package', summary['scene_package'], '--task-recipe', summary['task_recipe_path'], '--detected-objects', summary['detected_objects_example_path'], '--output-dir', str(output_dir), '--min-objects', '1', '--once']
    if dry_run:
        cmd.append('--dry-run')
    if no_replay:
        cmd.append('--no-replay')
    if gated:
        cmd += ['--require-preflight', '--write-task-flow-preview']
    if as_json:
        cmd.append('--json')
    return cmd


def build_preview_command(workcell: Path, marker_mode: bool, as_json: bool, task_flow_preview: Path | None = None, show_task_flow: bool = False) -> list[str]:
    script = Path(__file__).resolve().parent / 'preview_generated_workcell_bundle.py'
    cmd = [sys.executable, str(script), '--workcell', str(workcell)]
    if marker_mode:
        cmd.append('--publish-markers')
    if as_json:
        cmd.append('--json')
    return cmd


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description='Run generated workcell bundle gated dry-run')
    p.add_argument('--workcell', type=Path, required=True)
    p.add_argument('--output-dir', type=Path)
    p.add_argument('--gated-dry-run', action='store_true')
    p.add_argument('--dry-run', action='store_true', default=True)
    p.add_argument('--no-replay', action='store_true', default=True)
    p.add_argument('--json', action='store_true')
    p.add_argument('--preview-only', action='store_true')
    p.add_argument('--preview-markers', action='store_true')
    p.add_argument('--preview-task-flow', action='store_true')
    args = p.parse_args(argv)
    if args.preview_only or args.preview_markers:
        cmd = build_preview_command(args.workcell, args.preview_markers, args.json or args.preview_only)
        proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
        print(proc.stdout if proc.stdout else proc.stderr)
        return proc.returncode

    summary = _load_summary(args.workcell)
    detected = Path(summary.get('detected_objects_example_path', ''))
    if not detected.exists():
        raise SystemExit(f"FAIL: missing detected objects example: {detected}")
    out = args.output_dir or (args.workcell / 'generated' / 'bundle_run')
    out.mkdir(parents=True, exist_ok=True)
    cmd = build_command(summary, out, args.dry_run, args.no_replay, args.gated_dry_run, args.json)
    proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
    payload = {'status': 'FAIL', 'stdout': proc.stdout, 'stderr': proc.stderr, 'command': cmd}
    if args.json and proc.stdout.strip():
        try:
            payload = json.loads(proc.stdout)
        except Exception:
            pass
    (out / 'bundle_run_report.json').write_text(json.dumps(payload, indent=2, sort_keys=True)+'\n', encoding='utf-8')
    if args.preview_task_flow and args.gated_dry_run:
        tf_path = out / 'task_flow_preview.json'
        preview_cmd = build_preview_command(args.workcell, args.preview_markers, True, task_flow_preview=tf_path, show_task_flow=True)
        subprocess.run(preview_cmd, capture_output=True, text=True, check=False)
    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(proc.stdout if proc.stdout else proc.stderr)
    return proc.returncode


if __name__ == '__main__':
    raise SystemExit(main())
