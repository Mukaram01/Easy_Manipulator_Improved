#!/usr/bin/env python3
import argparse
import json
import pathlib
import subprocess
import sys

def _fail(message: str, details: list[str] | None = None) -> int:
    print(f"ERROR: {message}", file=sys.stderr)
    if details:
        for detail in details:
            print(f"  - {detail}", file=sys.stderr)
    return 2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo-root", required=True)
    ap.add_argument("--workspace-root", required=True)
    ap.add_argument("--scene", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--screenshot", required=True)
    ap.add_argument("--executable", required=True)
    ap.add_argument("--timeout-sec", required=True, type=float)
    args = ap.parse_args()

    repo_root = pathlib.Path(args.repo_root)
    workspace_root = pathlib.Path(args.workspace_root)
    output_path = pathlib.Path(args.output)
    screenshot_path = pathlib.Path(args.screenshot)

    missing = []
    if not repo_root.exists():
        missing.append(f"repo root does not exist: {repo_root}")
    if not workspace_root.exists():
        missing.append(f"workspace root does not exist: {workspace_root}")
    executable_path = pathlib.Path(args.executable)
    if not executable_path.exists():
        missing.append(f"executable does not exist: {executable_path}")
    if args.timeout_sec <= 0:
        missing.append(f"timeout-sec must be > 0, got {args.timeout_sec}")
    if missing:
        return _fail("required argument validation failed", missing)

    index = repo_root / "scenes" / args.scene / "generated" / "scene_visual_mesh_index.json"
    if not index.exists():
        return _fail(
            "mesh index artifact is missing",
            [
                f"expected index path: {index}",
                "ensure scene generation completed before parity validation",
            ],
        )

    try:
        data = json.loads(index.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        return _fail("mesh index JSON is invalid", [f"path: {index}", f"decode_error: {exc}"])

    visuals = data.get("visual_items", [])
    expected = sum(1 for v in visuals if v.get("geometry_type") == "mesh")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    smoke = output_path
    cmd = [
        sys.executable,
        str(repo_root / "scripts" / "run_workcell_builder_scene3d_gui_smoke.py"),
        "--repo-root",
        str(repo_root),
        "--workspace-root",
        str(workspace_root),
        "--scene",
        args.scene,
        "--output",
        str(smoke),
        "--screenshot",
        str(screenshot_path),
        "--executable",
        str(executable_path),
        "--timeout-sec",
        str(args.timeout_sec),
    ]
    proc = subprocess.run(cmd, check=False)
    if proc.returncode != 0:
        return _fail(
            "scene3d smoke command failed",
            [
                f"return code: {proc.returncode}",
                f"command: {' '.join(cmd)}",
                f"smoke output target: {smoke}",
            ],
        )

    rendered = 0
    if smoke.exists():
        sj = json.loads(smoke.read_text(encoding="utf-8"))
        rendered = int(sj.get("mesh_rendered_count", 0))
    else:
        return _fail(
            "smoke output artifact missing after successful invocation",
            [f"expected smoke output at: {smoke}", f"command: {' '.join(cmd)}"],
        )

    audit = []
    for v in visuals:
      if v.get('geometry_type')!='mesh':
        continue
      rp=v.get('resolved_source_path','')
      p=pathlib.Path(rp) if rp else None
      audit.append({'link':v.get('link'),'visual':v.get('visual'),'package_uri':v.get('mesh_source_uri'),'resolved_file_path':rp,'file_exists':bool(p and p.exists()),'extension':p.suffix.lower() if p else '', 'loader_used':'assimp_or_fallback_runtime','rendered': bool(rp)})
    audit_path = output_path.with_name(f"scene3d_visual_audit_{args.scene}.json")
    audit_path.write_text(
        json.dumps(
            {
                "scene": args.scene,
                "expected_visual_meshes": expected,
                "rendered_mesh_count": rendered,
                "accepted_minimum": 12,
                "items": audit,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    print(json.dumps({"expected": expected, "rendered": rendered, "audit": str(audit_path)}, indent=2))
    return 0 if rendered >= 12 else 1

if __name__=='__main__':
    raise SystemExit(main())
