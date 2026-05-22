#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, subprocess, os
from pathlib import Path
try:
    from scripts.workcell_studio_path_resolver import resolve_repo_root, resolve_workspace_root, resolve_install_setup, resolve_workcell_builder_executable, resolve_scenes_root, describe_resolution
except ModuleNotFoundError:
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from workcell_studio_path_resolver import resolve_repo_root, resolve_workspace_root, resolve_install_setup, resolve_workcell_builder_executable, resolve_scenes_root, describe_resolution


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

    if args.build and workspace_root:
        commands.append(["colcon","build","--symlink-install","--packages-select","workcell_builder"])

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
            commands.append(cmd)
            rc=subprocess.run(cmd, cwd=repo_root, capture_output=True, text=True).returncode
            per_scene.append({"scene":s,"returncode":rc,"json":str(sj),"screenshot":str(sp)})
            if rc!=0: blockers.append(f"GUI smoke failed for {s}")

    if args.include_readiness:
        cmd=["python3", str(repo_root / "scripts/run_workcell_studio_scene_readiness_gate.py"), "--repo-root", str(repo_root)]
        if workspace_root: cmd += ["--workspace-root", str(workspace_root)]
        commands.append(cmd)

    status = "FAIL" if blockers else ("WARN" if warnings else "PASS")
    report={"schema":"workcell_studio_local_validation/v1","repo_root":str(repo_root),"workspace_root":str(workspace_root) if workspace_root else None,"install_setup_path":str(setup) if setup else None,"workcell_builder_executable":str(exe) if exe else None,"scenes_root":str(scenes_root),"output_dir":str(output_dir),"commands_run":[" ".join(c) for c in commands],"per_scene_results":per_scene,"smoke_json_paths":smoke_json,"screenshot_paths":screenshots,"blockers":blockers,"warnings":warnings,"resolution":describe_resolution(),"status":status}
    j=output_dir/"workcell_studio_local_validation.json"; m=output_dir/"workcell_studio_local_validation.md"
    j.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    m.write_text(f"# Workcell Studio Local Validation\n\nStatus: **{status}**\n", encoding="utf-8")
    return 1 if status=="FAIL" else 0

if __name__=="__main__":
    raise SystemExit(main())
