#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, subprocess, os
from pathlib import Path

import sys

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from scripts.workcell_studio_script_bootstrap import ensure_repo_root_on_sys_path

ensure_repo_root_on_sys_path(__file__)

from scripts.workcell_studio_path_resolver import resolve_repo_root, resolve_workspace_root, resolve_install_setup, resolve_workcell_builder_executable, resolve_scenes_root, describe_resolution


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo-root")
    ap.add_argument("--workspace-root")
    ap.add_argument("--workcell-builder-executable", "--executable", dest="executable")
    ap.add_argument("--scenes", nargs="+", default=["ur5_2f_test", "ur5_2f_sorting_test"])
    ap.add_argument("--include-gui-smoke", action="store_true")
    ap.add_argument("--require-gui-smoke", action="store_true")
    ap.add_argument("--include-readiness", action="store_true")
    ap.add_argument("--build", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--output-dir", default="build/workcell_studio/local_validation")
    args = ap.parse_args()

    repo_root = resolve_repo_root(explicit_repo_root=args.repo_root)
    workspace_root = resolve_workspace_root(repo_root, args.workspace_root)
    setup = resolve_install_setup(workspace_root)
    exe = resolve_workcell_builder_executable(workspace_root, args.executable)
    scenes_root = resolve_scenes_root(repo_root)
    output_dir = (Path(args.output_dir) if Path(args.output_dir).is_absolute() else (repo_root / args.output_dir)).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"resolved repo_root={repo_root}\nworkspace_root={workspace_root}\ninstall_setup={setup}\nexecutable={exe}\nscenes_root={scenes_root}\noutput_dir={output_dir}")
    blockers=[]; warnings=[]; commands=[]; per_scene=[]; smoke_json=[]; screenshots=[]

    def _tail(text: str | None, lines: int = 40) -> str:
        if not text:
            return ""
        chunks = text.strip().splitlines()
        return "\n".join(chunks[-lines:])

    def _record_command(name: str, cmd: list[str], cwd: Path) -> int:
        proc = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
        commands.append({
            "name": name,
            "status": "PASS" if proc.returncode == 0 else "FAIL",
            "returncode": proc.returncode,
            "command": " ".join(cmd),
            "stdout_tail": _tail(proc.stdout),
            "stderr_tail": _tail(proc.stderr),
        })
        return proc.returncode

    if args.build and workspace_root:
        build_cmd = ["colcon","build","--symlink-install","--packages-select","workcell_builder"]
        build_rc = _record_command("build_workcell_builder", build_cmd, workspace_root)
        if build_rc != 0:
            blockers.append(f"build failed returncode={build_rc}")

    if args.include_gui_smoke:
        for s in args.scenes:
            sj = output_dir / f"scene3d_gui_smoke_{s}.json"; sp = output_dir / f"scene3d_gui_smoke_{s}.png"
            smoke_json.append(str(sj)); screenshots.append(str(sp))
            if exe is None:
                msg = "missing workcell_builder executable"
                if args.require_gui_smoke and not (args.dry_run or os.getenv("CI")):
                    blockers.append(msg)
                else:
                    warnings.append(msg)
                continue
            cmd=["python3", str(repo_root / "scripts/run_workcell_builder_scene3d_gui_smoke.py"), "--repo-root", str(repo_root), "--workspace-root", str(workspace_root or repo_root), "--scene", s, "--output", str(sj), "--screenshot", str(sp), "--executable", str(exe)]
            rc = _record_command(f"gui_smoke_{s}", cmd, repo_root)
            per_scene.append({"scene":s,"returncode":rc,"json":str(sj),"screenshot":str(sp)})
            if rc!=0:
                blockers.append(f"GUI smoke failed for {s}")
                blockers.append(f"returncode={rc}")
                smoke_payload = None
                if sj.exists():
                    try:
                        smoke_payload = json.loads(sj.read_text(encoding="utf-8"))
                    except Exception as exc:
                        blockers.append(f"invalid smoke JSON for {s}: {exc}")
                else:
                    blockers.append("smoke JSON missing")
                if isinstance(smoke_payload, dict):
                    for b in smoke_payload.get("blockers", []):
                        blockers.append(f"gui_smoke[{s}] blocker: {b}")
                    if not smoke_payload.get("screenshot_available", False):
                        blockers.append(f"gui_smoke[{s}] screenshot missing")
                    blockers.append(f"gui_smoke[{s}] json={sj}")
                elif not sp.exists():
                    blockers.append("screenshot missing")

    if args.include_readiness:
        cmd=["python3", str(repo_root / "scripts/run_workcell_studio_scene_readiness_gate.py"), "--repo-root", str(repo_root)]
        if workspace_root: cmd += ["--workspace-root", str(workspace_root)]
        readiness_rc = _record_command("scene_readiness_gate", cmd, repo_root)
        if readiness_rc != 0:
            blockers.append(f"readiness gate failed returncode={readiness_rc}")

    status = "FAIL" if blockers else ("WARN" if warnings else "PASS")
    if status == "FAIL" and not blockers:
        blockers.append("local_validation_failed_without_blockers")
        for c in commands:
            if c["returncode"] != 0:
                blockers.append(f"{c['name']} returncode={c['returncode']}")

    report={"schema":"workcell_studio_local_validation/v1","repo_root":str(repo_root),"workspace_root":str(workspace_root) if workspace_root else None,"install_setup_path":str(setup) if setup else None,"workcell_builder_executable":str(exe) if exe else None,"scenes_root":str(scenes_root),"output_dir":str(output_dir),"commands_run":[c["command"] for c in commands],"command_results":commands,"per_scene_results":per_scene,"smoke_json_paths":smoke_json,"screenshot_paths":screenshots,"blockers":blockers,"warnings":warnings,"resolution":describe_resolution(),"status":status}
    j=output_dir/"workcell_studio_local_validation.json"; m=output_dir/"workcell_studio_local_validation.md"
    j.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    md = [
        "# Workcell Studio Local Validation",
        "",
        f"## Status",
        "",
        f"**{status}**",
        "",
        "## Resolved paths",
        "",
        f"- repo_root: `{repo_root}`",
        f"- workspace_root: `{workspace_root}`",
        f"- install_setup: `{setup}`",
        f"- executable: `{exe}`",
        f"- scenes_root: `{scenes_root}`",
        f"- output_dir: `{output_dir}`",
        "",
        "## Top-level blockers",
        "",
    ]
    md.extend([f"- {b}" for b in blockers] if blockers else ["- none"])
    md.extend(["", "## Top-level warnings", ""])
    md.extend([f"- {w}" for w in warnings] if warnings else ["- none"])
    md.extend(["", "## Commands run", "", "| Name | Status | Return code | Command |", "|---|---:|---:|---|"])
    for c in commands:
        md.append(f"| {c['name']} | {c['status']} | {c['returncode']} | `{c['command']}` |")
    for c in commands:
        if c["returncode"] != 0:
            md.extend(["", f"### Failed command: {c['name']}", "", "```text", "stderr tail:", c["stderr_tail"] or "<empty>", "", "stdout tail:", c["stdout_tail"] or "<empty>", "```"])
    md.extend(["", "## Per-scene results", ""])
    if per_scene:
        for r in per_scene:
            md.append(f"- scene={r['scene']} returncode={r['returncode']} json=`{r['json']}` screenshot=`{r['screenshot']}`")
    else:
        md.append("- none")
    md.extend(["", "## Artifact paths", "", f"- json report: `{j}`", f"- markdown report: `{m}`"])
    if smoke_json:
        md.append(f"- smoke JSON: {', '.join(f'`{p}`' for p in smoke_json)}")
    if screenshots:
        md.append(f"- screenshots: {', '.join(f'`{p}`' for p in screenshots)}")
    md.extend(["", "## Next fix", ""])
    if blockers:
        md.append(f"Address blockers first: {blockers[0]}")
    elif warnings:
        md.append(f"Resolve warnings: {warnings[0]}")
    else:
        md.append("No immediate fix required.")
    m.write_text("\n".join(md) + "\n", encoding="utf-8")
    return 1 if status=="FAIL" else 0

if __name__=="__main__":
    raise SystemExit(main())
