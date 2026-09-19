#!/usr/bin/env python3
"""Run the remaining Stage-A Fortress gates with fresh physics sessions.

This runner never targets real hardware. Each motion gate gets a new isolated
Fortress session, fresh physical observations, fresh Resolve, Generate, and
nine-stage revalidation. It stops at the first failed gate and never replays an
old trajectory.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml

SOURCE_WORLD_SHA256 = "39c2aafb62a01af49663f21b734534843d0d4e4e034a164da2eadb03a761f60e"
QUALIFIED_CAPABILITY_SHA256 = "9f750e46a438d4b415afb07d3d3b77ee66f636fd3beedb3ec8b3e5d90d8d0489"
STAGES = [
    "PREPLAN_APPROACH", "PREPLAN_GRASP", "PREPLAN_CLOSE_GRIPPER",
    "PREPLAN_LIFT", "PREPLAN_TRANSFER", "PREPLAN_PLACE",
    "PREPLAN_OPEN_GRIPPER", "PREPLAN_RETREAT", "PREPLAN_HOME",
]
GATES = ("resolve", "telemetry", "stationary", "contact-release", "full-cycle")
EXPECTED_RESULTS = {
    "telemetry": "MOTION_TELEMETRY_PASS",
    "stationary": "STATIONARY_RETENTION_PASS",
    "contact-release": "CONTACT_RELEASE_PASS",
    "full-cycle": "PASS",
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def group_alive(pgid: int) -> bool:
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / "stat").read_text().rsplit(")", 1)[1].split()
            if fields[0] != "Z" and int(fields[2]) == pgid:
                return True
        except (OSError, ValueError, IndexError):
            pass
    return False


def stop_owned(process: subprocess.Popen | None) -> dict:
    if process is None:
        return {"started": False, "clean": True, "escalation": None}
    escalation = None
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
    deadline = time.monotonic() + 10
    while group_alive(process.pid) and time.monotonic() < deadline:
        time.sleep(.1)
    if group_alive(process.pid):
        escalation = "SIGTERM"
        os.killpg(process.pid, signal.SIGTERM)
        deadline = time.monotonic() + 5
        while group_alive(process.pid) and time.monotonic() < deadline:
            time.sleep(.1)
    if group_alive(process.pid):
        escalation = "SIGKILL"
        os.killpg(process.pid, signal.SIGKILL)
        deadline = time.monotonic() + 3
        while group_alive(process.pid) and time.monotonic() < deadline:
            time.sleep(.05)
    try:
        rc = process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        rc = None
    return {"started": True, "clean": not group_alive(process.pid),
            "returncode": rc, "escalation": escalation}


def run(command, *, env, cwd, log: Path, timeout: float, check=True) -> subprocess.CompletedProcess:
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("w") as stream:
        completed = subprocess.run(
            [str(x) for x in command], cwd=cwd, env=env,
            stdout=stream, stderr=subprocess.STDOUT, timeout=timeout, check=False)
    if check and completed.returncode:
        raise RuntimeError(f"command failed ({completed.returncode}): {' '.join(map(str, command))}; log={log}")
    return completed


def wait_file(path: Path, process: subprocess.Popen, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if path.is_file():
            return
        if process.poll() is not None:
            raise RuntimeError(f"owned launch exited before {path.name} appeared")
        time.sleep(.1)
    raise TimeoutError(f"timed out waiting for {path}")


def assert_domain_free(env: dict, cwd: Path) -> None:
    probe = subprocess.run(
        ["ros2", "node", "list"], cwd=cwd, env=env,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        timeout=10, check=False)
    nodes = [line.strip() for line in probe.stdout.splitlines() if line.strip().startswith("/")]
    if probe.returncode or nodes:
        raise RuntimeError(f"ROS domain {env['ROS_DOMAIN_ID']} is not isolated: {nodes or probe.stdout.strip()}")


def assert_plan(summary: dict, *, require_resolved: bool) -> None:
    if (summary.get("result") != "PLAN_ONLY" or
            summary.get("full_cycle_prevalidated") is not True or
            summary.get("execution_attempted") is not False):
        raise RuntimeError("current authoritative plan-only cycle did not pass")
    plans = summary.get("plan_metadata", [])
    if [item.get("stage") for item in plans] != STAGES:
        raise RuntimeError("current plan does not contain the complete nine-stage cycle")
    if any(item.get("success") is not True or item.get("moveit_code") != 1
           or item.get("points", 0) < 2 for item in plans):
        raise RuntimeError("one or more current cycle stages are not executable plans")
    resolution = summary.get("task_intent_resolution", {})
    if require_resolved and resolution.get("readiness_status") not in ("READY", "WARNING"):
        raise RuntimeError("fresh Resolve did not produce a consumable task handoff")


def assert_gate(gate: str, summary: dict) -> None:
    expected = EXPECTED_RESULTS[gate]
    if summary.get("result") != expected:
        raise RuntimeError(f"{gate} gate returned {summary.get('result')!r}, expected {expected!r}")
    if summary.get("full_cycle_prevalidated") is not True:
        raise RuntimeError(f"{gate} gate did not revalidate the complete cycle")
    if gate == "telemetry":
        metrics = summary.get("motion_telemetry", {})
        if metrics.get("max_fresh_age_ms", 1e9) >= 250 or metrics.get("max_delivery_ms", 1e9) >= 250:
            raise RuntimeError("motion telemetry does not satisfy the unchanged 250 ms guard")
    elif gate == "stationary":
        retention = summary.get("stationary_retention", {})
        if retention.get("duration_sim_ns", 0) < 1_000_000_000:
            raise RuntimeError("stationary physical retention is shorter than one second")
        if set(retention.get("required_contact_links", [])) - set(retention.get("contact_links", [])):
            raise RuntimeError("stationary retention lacks opposing required contacts")
    elif gate == "contact-release":
        if summary.get("verified_lift_clearance_m", 0) < .01 or not summary.get("release_evidence", {}).get("settled"):
            raise RuntimeError("contact-release did not prove lift and physical settling")
    elif gate == "full-cycle":
        final = summary.get("full_cycle_physical_acceptance", {})
        if not summary.get("full_cycle_execution_success") or not final.get("final_collision_valid"):
            raise RuntimeError("full physical cycle final acceptance is incomplete")
        if final.get("attached_ids"):
            raise RuntimeError("full physical cycle left a planning-scene attachment")


def preflight(args, repo: Path, workspace: Path) -> dict:
    source_world = args.source_world.resolve()
    scene = args.scene.resolve()
    generator = args.generator.resolve()
    if not source_world.is_file() or sha256(source_world) != SOURCE_WORLD_SHA256:
        raise RuntimeError("source world is not the frozen pristine Stage-A world")
    text = source_world.read_text(errors="ignore").lower()
    if any(token in text for token in (
            "workcell::simulatormeasurements", "workcell_measurements",
            "gz_ros2_control", "ign_ros2_control", "attach", "magnet", "suction")):
        raise RuntimeError("source world already contains runtime/control/attachment instrumentation")
    if not scene.is_dir() or not generator.is_file():
        raise RuntimeError("prepared Stage-A scene/generator is missing")
    intent_path = scene / "config/workcell_builder_task_intent.yaml"
    intent = yaml.safe_load(intent_path.read_text()) if intent_path.is_file() else {}
    safety = (intent or {}).get("safety", {})
    if ((intent or {}).get("schema") != "workcell_builder_task_intent/v2" or
            safety.get("execution_backend") != "simulator" or
            safety.get("require_fake_hardware") is not False or
            safety.get("real_hardware_enabled") is not False):
        raise RuntimeError("prepared scene is not the explicit simulator-locked TaskIntent v2 commissioning scene")
    manifest_path = workspace / "install/workcell_builder/share/workcell_builder/commission_execute_build.json"
    if not manifest_path.is_file():
        raise RuntimeError("commissioning capability build manifest is missing")
    manifest = json.loads(manifest_path.read_text())
    library = Path(manifest["library"])
    if (manifest.get("sha256") != QUALIFIED_CAPABILITY_SHA256 or
            not library.is_file() or sha256(library) != QUALIFIED_CAPABILITY_SHA256):
        raise RuntimeError("loaded commissioning capability is not the qualified final build")
    return {
        "source_world": str(source_world), "source_world_sha256": SOURCE_WORLD_SHA256,
        "scene": str(scene), "generator": str(generator), "generator_sha256": sha256(generator),
        "capability_sha256": QUALIFIED_CAPABILITY_SHA256,
        "repo_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip(),
        "repo_dirty": bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=repo, text=True).strip()),
    }


def executor_command(repo: Path, scene: Path, receipt: Path, observations: Path,
                     summary: Path, planning_time: float, *, resolve=False, gate=None,
                     evidence: Path | None = None) -> list[str]:
    cmd = [
        sys.executable, str(repo / "scripts/perceived_object_grasp_execute.py"),
        "--scene-package", str(scene), "--backend", "simulator",
        "--simulator-receipt", str(receipt), "--detections", str(observations),
        "--timeout", "300", "--segment-planning-time", str(planning_time),
        "--summary-output", str(summary),
    ]
    if resolve:
        cmd.append("--resolve-task")
    if gate is not None:
        cmd += ["--start", "--simulator-commission", gate]
        if gate == "full-cycle":
            if evidence is None:
                raise ValueError("full-cycle gate requires prerequisite evidence")
            cmd += ["--commission-evidence", str(evidence)]
    return cmd


def one_session(args, repo: Path, gate: str, index: int, prior: dict[str, Path]) -> Path:
    session = args.output / f"{index:02d}-{gate}"
    if session.exists():
        raise RuntimeError(f"refusing to reuse evidence directory {session}")
    session.mkdir(parents=True)
    runtime = session / "runtime"
    receipt = runtime / "receipt.json"
    observations = session / "parts.yaml"
    domain = args.base_domain + index - 1
    if not 1 <= domain <= 232:
        raise RuntimeError("ROS domain range exhausted")
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), IGN_PARTITION=f"{args.partition_prefix}_{gate}",
               ROS_LOCALHOST_ONLY="1", ROS2CLI_DISABLE_DAEMON="1", PYTHONUNBUFFERED="1")
    report = {"gate": gate, "domain": domain, "partition": env["IGN_PARTITION"], "status": "BLOCKED"}
    launch = None
    try:
        assert_domain_free(env, repo)
        run([sys.executable, args.generator], env=env, cwd=repo,
            log=session / "generate-before.log", timeout=180)
        launch_log = (session / "launch.log").open("w")
        launch_cmd = [
            "ros2", "launch", "ur5_2f_test", "demo.launch.py",
            "execution_backend:=simulator", "simulator_commissioning:=true",
            f"simulator_world:={args.source_world}", f"simulator_output:={runtime}",
            "use_sim_time:=true", "launch_rviz:=false",
            "use_fake_hardware:=true", "allow_trajectory_execution:=true",
        ]
        launch = subprocess.Popen(launch_cmd, cwd=repo, env=env, stdout=launch_log,
                                  stderr=subprocess.STDOUT, start_new_session=True)
        wait_file(receipt, launch, 90)
        run([sys.executable, repo / "scripts/simulator_observations.py",
             "--receipt", receipt, "--output", observations, "--class-id", args.class_id],
            env=env, cwd=repo, log=session / "observations.log", timeout=90)
        resolve_summary = session / "resolve/summary.json"
        run(executor_command(repo, args.scene, receipt, observations, resolve_summary,
                             args.segment_planning_time, resolve=True),
            env=env, cwd=repo, log=session / "resolve/executor.log", timeout=420)
        resolved = json.loads(resolve_summary.read_text())
        assert_plan(resolved, require_resolved=True)

        run([sys.executable, args.generator], env=env, cwd=repo,
            log=session / "generate-after-resolve.log", timeout=180)
        plan_summary = session / "revalidate/summary.json"
        run(executor_command(repo, args.scene, receipt, observations, plan_summary,
                             args.segment_planning_time),
            env=env, cwd=repo, log=session / "revalidate/executor.log", timeout=420)
        planned = json.loads(plan_summary.read_text())
        assert_plan(planned, require_resolved=True)
        report.update(selected_object_id=planned.get("selected_object_id"),
                      selected_grasp_index=planned.get("selected_grasp_index"),
                      resolution_sha256=planned.get("resolution_sha256"),
                      no_motion_revalidation="PASS")
        if gate == "resolve":
            report["status"] = "PASS"
            return plan_summary

        evidence = None
        if gate == "full-cycle":
            cancellation = args.cancellation_summary.resolve()
            required = ["telemetry", "stationary", "contact-release"]
            if not cancellation.is_file() or any(name not in prior for name in required):
                raise RuntimeError("full-cycle prerequisite evidence is incomplete")
            records = [json.loads(cancellation.read_text())]
            records += [json.loads(prior[name].read_text()) for name in required]
            evidence = session / "commission-evidence.json"
            evidence.write_text(json.dumps(records, indent=2) + "\n")

        gate_summary = session / f"{gate}/summary.json"
        run(executor_command(repo, args.scene, receipt, observations, gate_summary,
                             args.segment_planning_time, gate=gate, evidence=evidence),
            env=env, cwd=repo, log=session / f"{gate}/executor.log", timeout=600)
        result = json.loads(gate_summary.read_text())
        assert_gate(gate, result)
        report.update(status="PASS", result=result["result"],
                      selected_object_id=result.get("selected_object_id"),
                      selected_grasp_index=result.get("selected_grasp_index"))
        return gate_summary
    except Exception as exc:
        report["failure"] = str(exc)
        raise
    finally:
        report["shutdown"] = stop_owned(launch)
        if "launch_log" in locals():
            launch_log.close()
        (session / "session-report.json").write_text(json.dumps(report, indent=2) + "\n")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-world", type=Path, default=Path("/tmp/stage-a01/full/world.sdf"))
    parser.add_argument("--scene", type=Path, default=Path("/home/user/workcell_ws/a05-evidence-20260918/final-build-cancellation/ur5_2f_test"))
    parser.add_argument("--generator", type=Path, default=Path("/home/user/workcell_ws/a05-evidence-20260918/final-build-cancellation/generate.py"))
    parser.add_argument("--cancellation-summary", type=Path, default=Path("/home/user/workcell_ws/a05-evidence-20260918/final-build-cancellation/cancellation/summary.json"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--base-domain", type=int, default=201)
    parser.add_argument("--partition-prefix", default="stage_a1_finish")
    parser.add_argument("--class-id", default="part")
    parser.add_argument("--segment-planning-time", type=float, default=3.0)
    parser.add_argument("--through", choices=GATES, default="full-cycle")
    args = parser.parse_args(argv)

    repo = Path(__file__).resolve().parents[1]
    workspace = repo.parent.parent
    args.output = args.output.resolve()
    if args.output.exists():
        parser.error("choose a new output directory")
    if not 0 < args.segment_planning_time <= 10:
        parser.error("segment planning time must be in (0,10]")
    args.output.mkdir(parents=True)
    overall = {"status": "BLOCKED", "gates": {}, "preflight": None}
    try:
        overall["preflight"] = preflight(args, repo, workspace)
        prior: dict[str, Path] = {}
        selected = GATES[:GATES.index(args.through)+1]
        for index, gate in enumerate(selected, 1):
            path = one_session(args, repo, gate, index, prior)
            prior[gate] = path
            overall["gates"][gate] = {
                "status": "PASS", "summary": str(path.relative_to(args.output))}
            overall["status"] = "PASS" if gate == args.through else "IN_PROGRESS"
        if args.through == "full-cycle":
            final = json.loads(prior["full-cycle"].read_text())
            if final.get("result") != "PASS" or not final.get("full_cycle_execution_success"):
                raise RuntimeError("full-cycle summary is not physically accepted")
        return 0
    except Exception as exc:
        overall["failure"] = str(exc)
        return 1
    finally:
        overall["finished_wall_ns"] = time.time_ns()
        (args.output / "stage-a1-final-report.json").write_text(json.dumps(overall, indent=2) + "\n")
        print(json.dumps(overall, indent=2))


if __name__ == "__main__":
    raise SystemExit(main())
