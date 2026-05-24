#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, subprocess, sys
from pathlib import Path

DEFAULT_REPO_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = DEFAULT_REPO_ROOT


def _run(cmd: list[str], cwd: Path) -> tuple[int, str, str]:
    p = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, check=False)
    return p.returncode, p.stdout, p.stderr


def _tail(text: str, lines: int = 20) -> str:
    return "\n".join(text.splitlines()[-lines:])


def _json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene-name", default="scene3d_generated_canvas_acceptance")
    ap.add_argument("--output-dir", default="build/workcell_studio/local_validation")
    ap.add_argument("--repo-root", default=str(DEFAULT_REPO_ROOT))
    ap.add_argument("--workspace-root", default=None)
    ap.add_argument("--workcell-builder-executable", "--executable", dest="executable", default=None)
    ap.add_argument("--timeout-sec", type=float, default=30)
    args = ap.parse_args()

    repo_root = Path(args.repo_root).resolve()
    out_dir = (repo_root / args.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    generated_root = out_dir / "generated_scenes"
    generated_root.mkdir(parents=True, exist_ok=True)

    acceptance_json = out_dir / f"{args.scene_name}_generation.json"
    generation_cmd = [
        sys.executable,
        str(repo_root / "scripts" / "generate_scratch_cell_acceptance.py"),
        "--scene-name",
        args.scene_name,
        "--output-root",
        str(generated_root),
        "--json-out",
        str(acceptance_json),
    ]
    rc, so, se = _run(generation_cmd, repo_root)
    generation_summary = (se.strip().splitlines() or so.strip().splitlines() or ["generation command failed"])[-1]
    if rc != 0:
        print(
            json.dumps(
                {
                    "status": "FAIL",
                    "step": "generation",
                    "command": " ".join(generation_cmd),
                    "failure_summary": generation_summary,
                    "stdout_tail": _tail(so),
                    "stderr_tail": _tail(se),
                },
                indent=2,
            )
        )
        return rc

    generation_payload = _json(acceptance_json)
    scene_dir = Path(generation_payload["scene_dir"]).resolve()

    required_outputs = [
        "scene_manifest.yaml",
        "environment_layout.yaml",
        "config/scene3d_mesh_index.json",
        "urdf/scene.urdf.xacro",
        "launch/demo.launch.py",
        "package.xml",
        "CMakeLists.txt",
    ]
    missing_outputs = [r for r in required_outputs if not (scene_dir / r).exists()]

    smoke_json = out_dir / f"scene3d_gui_smoke_{args.scene_name}.json"
    smoke_png = out_dir / f"scene3d_gui_smoke_{args.scene_name}.png"
    smoke_cmd = [
        sys.executable,
        str(repo_root / "scripts" / "run_workcell_builder_scene3d_gui_smoke.py"),
        "--repo-root",
        str(repo_root),
        "--workspace-root",
        str(Path(args.workspace_root).resolve() if args.workspace_root else repo_root),
        "--scene",
        args.scene_name,
        "--output",
        str(smoke_json),
        "--screenshot",
        str(smoke_png),
    ]
    if args.executable:
        smoke_cmd.extend(["--executable", str(Path(args.executable).resolve())])
    smoke_cmd.extend(["--timeout-sec", str(args.timeout_sec)])
    smoke_rc, smoke_so, smoke_se = _run(smoke_cmd, repo_root)
    smoke_summary = (smoke_se.strip().splitlines() or smoke_so.strip().splitlines() or ["gui smoke command failed"])[-1]

    runtime_json = out_dir / f"scene3d_runtime_acceptance_{args.scene_name}.json"
    runtime_md = out_dir / f"scene3d_runtime_acceptance_{args.scene_name}.md"
    runtime_cmd = [
        sys.executable,
        str(repo_root / "scripts" / "validate_scene3d_runtime_acceptance.py"),
        "--scene",
        args.scene_name,
        "--repo-root",
        str(repo_root),
        "--output-dir",
        str(out_dir),
        "--json",
        str(runtime_json),
        "--markdown",
        str(runtime_md),
        "--smoke-json",
        str(smoke_json),
    ]
    if args.workspace_root:
        runtime_cmd.extend(["--workspace-root", str(Path(args.workspace_root).resolve())])
    if args.executable:
        runtime_cmd.extend(["--workcell-builder-executable", str(Path(args.executable).resolve())])
    runtime_rc, runtime_so, runtime_se = _run(runtime_cmd, repo_root)
    runtime_summary = (runtime_se.strip().splitlines() or runtime_so.strip().splitlines() or ["runtime acceptance failed"])[-1]

    smoke_payload = _json(smoke_json) if smoke_json.exists() else {}
    counters = smoke_payload.get("counters", {}) if isinstance(smoke_payload, dict) else {}
    def _count(key: str, nested_alias: str | None = None) -> int:
        if isinstance(smoke_payload, dict) and key in smoke_payload:
            return int(smoke_payload.get(key) or 0)
        if isinstance(counters, dict):
            return int(counters.get(nested_alias or key) or 0)
        return 0
    key_counts = {
        "visible": _count("visible_count"),
        "rendered": _count("rendered_count"),
        "selectable": _count("selectable_count"),
        "hierarchy_rows": _count("hierarchy_rows_count", "hierarchy_rows"),
    }

    blockers: list[str] = []
    failing_step = ""
    failing_command = ""
    if missing_outputs:
        blockers.append(f"missing generated outputs: {', '.join(missing_outputs)}")
        if not failing_step:
            failing_step = "output_validation"
            failing_command = "generated output existence checks"
    if smoke_rc != 0:
        blockers.append("gui smoke command failed")
        if not failing_step:
            failing_step = "gui_smoke"
            failing_command = " ".join(smoke_cmd)
    if not smoke_png.exists() or smoke_png.stat().st_size <= 0:
        blockers.append("gui smoke screenshot missing or empty")
        if not failing_step:
            failing_step = "gui_smoke"
            failing_command = " ".join(smoke_cmd)
    if runtime_rc != 0:
        blockers.append("runtime acceptance validation failed")
        if not failing_step:
            failing_step = "runtime_acceptance"
            failing_command = " ".join(runtime_cmd)
    for k, v in key_counts.items():
        if v <= 0:
            blockers.append(f"runtime counter not > 0: {k}={v}")
            if not failing_step:
                failing_step = "runtime_counters"
                failing_command = "scene3d runtime counter checks"

    artifact = {
        "schema": "scene3d_generated_canvas_acceptance/v1",
        "scene": args.scene_name,
        "scene_dir": str(scene_dir),
        "status": "PASS" if not blockers else "FAIL",
        "failing_step": failing_step if blockers else "",
        "failing_command": failing_command if blockers else "",
        "commands": {
            "generation": " ".join(generation_cmd),
            "gui_smoke": " ".join(smoke_cmd),
            "runtime_acceptance": " ".join(runtime_cmd),
        },
        "generation": generation_payload,
        "required_outputs": required_outputs,
        "missing_outputs": missing_outputs,
        "gui_smoke": {
            "returncode": smoke_rc,
            "json": str(smoke_json),
            "screenshot": str(smoke_png),
            "screenshot_size": smoke_png.stat().st_size if smoke_png.exists() else 0,
            "stdout_tail": "\n".join(smoke_so.splitlines()[-20:]),
            "stderr_tail": "\n".join(smoke_se.splitlines()[-20:]),
            "failure_summary": smoke_summary,
        },
        "runtime_acceptance": {"returncode": runtime_rc, "json": str(runtime_json), "markdown": str(runtime_md), "stdout_tail": "\n".join(runtime_so.splitlines()[-20:]), "stderr_tail": "\n".join(runtime_se.splitlines()[-20:]), "failure_summary": runtime_summary},
        "key_counts": key_counts,
        "blockers": blockers,
    }
    out_json = out_dir / f"{args.scene_name}_acceptance.json"
    out_md = out_dir / f"{args.scene_name}_acceptance.md"
    out_json.write_text(json.dumps(artifact, indent=2) + "\n", encoding="utf-8")
    out_md.write_text(
        "\n".join([
            "# Scene3D Generated Canvas Acceptance",
            "",
            f"- scene: `{args.scene_name}`",
            f"- status: **{artifact['status']}**",
            f"- scene_dir: `{scene_dir}`",
            "",
            "## Missing outputs",
            *([f"- {m}" for m in missing_outputs] or ["- none"]),
            "",
            "## Key runtime counts",
            *[f"- {k}: {v}" for k, v in key_counts.items()],
            "",
            "## Blockers",
            *([f"- {b}" for b in blockers] or ["- none"]),
            "",
            "## Failure context",
            f"- failing_step: `{artifact['failing_step'] or 'none'}`",
            f"- failing_command: `{artifact['failing_command'] or 'none'}`",
            "- detailed diagnostics: `generation.generator_failure`, `gui_smoke.{returncode,stdout_tail,stderr_tail}`, `runtime_acceptance.{returncode,stdout_tail,stderr_tail}`",
            "",
            "## Artifacts",
            f"- `{out_json}`",
            f"- `{runtime_json}`",
            f"- `{smoke_json}`",
            f"- `{smoke_png}`",
        ]) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"status": artifact["status"], "artifact_json": str(out_json), "artifact_md": str(out_md)}, indent=2))
    return 0 if not blockers else 1


if __name__ == "__main__":
    raise SystemExit(main())
