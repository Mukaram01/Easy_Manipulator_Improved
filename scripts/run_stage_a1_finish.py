#!/usr/bin/env python3
"""Finish Stage-A Fortress commissioning with fresh, fail-closed sessions.

The runner is portable across the user's two ROS workstations. It prepares a
fresh simulator-locked TaskIntent scene from the tracked canonical UR5+2F scene,
rebuilds/tests the opt-in commissioning capability, re-qualifies cancellation
on the current binary, and then advances only through measured gates that pass.
It never targets real hardware and never replays an old trajectory.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import re
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import yaml

SOURCE_WORLD_GIT_BLOB="b687b5bf3c48e4a731d87c86f99e6c3696b21599"
STAGES=[
    "PREPLAN_APPROACH","PREPLAN_GRASP","PREPLAN_CLOSE_GRIPPER",
    "PREPLAN_LIFT","PREPLAN_TRANSFER","PREPLAN_PLACE",
    "PREPLAN_OPEN_GRIPPER","PREPLAN_RETREAT","PREPLAN_HOME",
]
GATES=("resolve","cancel","telemetry","stationary","contact-release","full-cycle")
EXPECTED_RESULTS={
    "cancel":"CANCELLATION_TRIAL_PASS",
    "telemetry":"MOTION_TELEMETRY_PASS",
    "stationary":"STATIONARY_RETENTION_PASS",
    "contact-release":"CONTACT_RELEASE_PASS",
    "full-cycle":"PASS",
}


def sha256(path:Path)->str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def group_alive(pgid:int)->bool:
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():continue
        try:
            fields=(entry/"stat").read_text().rsplit(")",1)[1].split()
            if fields[0]!="Z" and int(fields[2])==pgid:return True
        except (OSError,ValueError,IndexError):pass
    return False


def launch_child_exits(log,*,cleanup_offset=None,sent_signals=()):
    """Classify exits against the owned cleanup boundary, preserving launch order."""
    if log is None:return []
    raw=Path(log).read_bytes();text=raw.decode(errors="replace")
    boundary=len(text) if cleanup_offset is None else len(raw[:cleanup_offset].decode(errors="replace"))
    pattern=re.compile(r"\[(?P<name>[^]\n]+)\]: process (?:has died \[pid (?P<pid>\d+), exit code (?P<rc>-?\d+),|has finished cleanly \[pid (?P<clean_pid>\d+)\])")
    signal_pattern=re.compile(r"\[(?P<name>[^]\n]+)\]: sending signal '(?P<signal>SIGINT|SIGTERM)' to process")
    launch_signals=list(signal_pattern.finditer(text))
    children=[]
    for match in pattern.finditer(text):
        rc=int(match['rc'] or 0);before=match.start()<boundary
        try:sig=signal.Signals(-rc).name if rc<0 else None
        except ValueError:sig=f"SIGNAL_{-rc}"
        # These helpers deliberately exit after controller setup or verified
        # scene application. Their nonzero/signal exits remain failures.
        transient=bool(re.fullmatch(
            r"(?:spawner(?:\.py)?|workcell_studio_planning_scene_node\.py)-\d+",match['name']))
        child_signals={event['signal'] for event in launch_signals
            if event['name']==match['name'] and boundary<=event.start()<match.start()}
        requested={signal.Signals(item).name for item in sent_signals}|child_signals
        expected=(rc==0 and (transient or (not before and bool(requested)))) or (
            not before and sig in ("SIGINT","SIGTERM") and sig in requested)
        children.append(dict(name=match['name'],pid=int(match['pid'] or match['clean_pid']),
            returncode=rc,signal=sig,before_cleanup=before,expected=expected))
    return children


def stop_owned(process,log=None):
    if process is None:return {"started":False,"clean":True,"escalation":None}
    # Snapshot before signalling: a previously dead launch/long-lived child is
    # never excused by the SIGINT that the runner sends later.
    cleanup_offset=Path(log).stat().st_size if log is not None else 0
    alive_before=process.poll() is None
    escalation=None;sent_signals=[]
    def send(sig):
        try:os.killpg(process.pid,sig)
        except ProcessLookupError:return
        sent_signals.append(sig)
    if alive_before:send(signal.SIGINT)
    deadline=time.monotonic()+10
    while group_alive(process.pid) and time.monotonic()<deadline:time.sleep(.1)
    if group_alive(process.pid):
        escalation="SIGTERM";send(signal.SIGTERM);deadline=time.monotonic()+5
        while group_alive(process.pid) and time.monotonic()<deadline:time.sleep(.1)
    if group_alive(process.pid):
        escalation="SIGKILL";send(signal.SIGKILL);deadline=time.monotonic()+3
        while group_alive(process.pid) and time.monotonic()<deadline:time.sleep(.05)
    try:rc=process.wait(timeout=3)
    except subprocess.TimeoutExpired:rc=None
    children=launch_child_exits(log,cleanup_offset=cleanup_offset,sent_signals=sent_signals)
    crash_evidence=[]
    if log is not None:
        crash_evidence=[line for line in Path(log).read_text(errors="replace").splitlines()
            if re.search(r"segmentation fault|core dumped|\bSIGSEGV\b|\bSIGABRT\b",line,re.IGNORECASE)]
    remaining=group_alive(process.pid)
    root_expected=alive_before and bool(sent_signals) and (rc==0 or
        (rc in (-signal.SIGINT,-signal.SIGTERM) and -rc in sent_signals))
    return {"started":True,"clean":not remaining and root_expected and not crash_evidence
            and escalation!="SIGKILL" and all(child["expected"] for child in children),
            "returncode":rc,"escalation":escalation,"remaining_owned_processes":remaining,
            "root_alive_before_cleanup":alive_before,"cleanup_log_offset":cleanup_offset,
            "sent_signals":[signal.Signals(item).name for item in sent_signals],
            "children":children,"crash_evidence":crash_evidence}


def run(command,*,env,cwd,log,timeout,check=True):
    log=Path(log);log.parent.mkdir(parents=True,exist_ok=True)
    with log.open("w") as stream:
        completed=subprocess.run([str(x) for x in command],cwd=cwd,env=env,
            stdout=stream,stderr=subprocess.STDOUT,timeout=timeout,check=False)
    if check and completed.returncode:
        try:
            lines=log.read_text(errors="replace").splitlines()
            tail="\n".join(lines[-40:])
        except OSError:
            tail="<log unavailable>"
        raise RuntimeError(
            f"command failed ({completed.returncode}): {' '.join(map(str,command))}; log={log}\n"
            f"--- log tail ---\n{tail}\n--- end log tail ---")
    return completed


def wait_file(path,process,timeout,log=None):
    def log_tail():
        if log is None:return ""
        try:
            lines=Path(log).read_text(errors="replace").splitlines()
            return "\n--- launch log tail ---\n"+"\n".join(lines[-60:])+"\n--- end launch log tail ---"
        except OSError:
            return "\n--- launch log unavailable ---"
    path=Path(path);failure=path.parent/'startup-failure.json'
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if path.is_file():return
        if failure.is_file():
            try:payload=json.loads(failure.read_text())
            except Exception:payload={'message':failure.read_text(errors='replace')}
            raise RuntimeError('simulator startup failed: '+str(payload.get('message',payload))+log_tail())
        if process.poll() is not None:
            raise RuntimeError(f"owned launch exited before {path.name} appeared (rc={process.returncode})"+log_tail())
        time.sleep(.1)
    raise TimeoutError(f"timed out waiting for {path}"+log_tail())


def assert_domain_free(env,cwd):
    probe=subprocess.run(["ros2","node","list"],cwd=cwd,env=env,stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,text=True,timeout=10,check=False)
    nodes=[line.strip() for line in probe.stdout.splitlines() if line.strip().startswith("/")]
    if probe.returncode or nodes:
        raise RuntimeError(f"ROS domain {env['ROS_DOMAIN_ID']} is not isolated: {nodes or probe.stdout.strip()}")


def git_blob(path:Path,repo:Path)->str:
    return subprocess.check_output(["git","hash-object",str(path)],cwd=repo,text=True).strip()


def candidate_worlds(workspace:Path):
    repo=Path(__file__).resolve().parents[1]
    roots=[repo/"scenes/ur5_2f_test/worlds/stage_a0.sdf",
           Path("/tmp/stage-a01/full/world.sdf"),
           workspace/"a05-evidence-20260918/runtime/world.sdf"]
    for home in Path("/home").glob("*"):
        roots.append(home/"workcell_ws/a05-evidence-20260918/runtime/world.sdf")
    seen=set()
    for path in roots:
        try:key=str(path.resolve())
        except OSError:key=str(path)
        if key in seen:continue
        seen.add(key);yield path


def discover_source_world(explicit:Path|None,workspace:Path)->Path:
    repo=Path(__file__).resolve().parents[1]
    candidates=[explicit] if explicit is not None else list(candidate_worlds(workspace))
    for path in candidates:
        if path is None or not path.is_file():continue
        if git_blob(path,repo)!=SOURCE_WORLD_GIT_BLOB:continue
        text=path.read_text(errors="ignore").lower()
        if any(token in text for token in ("workcell::simulatormeasurements","workcell_measurements",
                "gz_ros2_control","ign_ros2_control","attach","magnet","suction")):continue
        return path.resolve()
    checked=", ".join(str(p) for p in candidates if p is not None)
    raise RuntimeError("pristine Stage-A world not found on this workstation; checked: "+checked)


def prepare_scene(repo:Path,destination:Path,class_id:str)->Path:
    """Materialize the tracked canonical cell into the exact Stage-A task policy."""
    source=repo/"scenes/ur5_2f_test"
    shutil.copytree(source,destination)
    # export_scene's existing validator resolves the strategy catalog relative
    # to the scene parent; provide the tracked catalog there rather than
    # weakening validation or teaching commissioning a second lookup rule.
    catalog_parent=destination.parent/"catalog"
    shutil.copytree(repo/"catalog/grasp_strategies",catalog_parent/"grasp_strategies")
    sys.path.insert(0,str(repo/"scripts"))
    from task_intent_v2 import migrate_v1
    from export_builder_scene_to_cell_definition import export_scene
    from generate_workcell_from_cell_definition import generate_package

    environment_doc=yaml.safe_load((destination/"environment.yaml").read_text())
    environment=environment_doc.get("environment",environment_doc)
    old=yaml.safe_load((destination/"config/workcell_builder_task_intent.yaml").read_text())
    intent=migrate_v1(old,environment)
    intent["scene_package"]=str(destination)
    selection=intent["pick"]["selection"]
    selection["object_filter"].update(class_id=class_id,min_confidence=None,max_age_seconds=180.0)

    catalog=yaml.safe_load((repo/"catalog/grasp_strategies/finger_pinch_basic.yaml").read_text())["grasp_strategy"]
    grasp=intent["pick"]["grasp"]
    grasp.update(policy="PREFERRED",required_capability="two_finger_parallel",
                 strategy_ref="finger_pinch_basic",
                 approach={"axis":catalog["approach_axis"],"distance_m":catalog["approach_distance_m"]},
                 orientation={"mode":catalog["orientation_mode"],"allowed_roll_deg":[0],
                              "allowed_yaw_deg":catalog["allowed_yaw_angles_deg"],
                              "tolerance_rad":[0.0,0.0,0.0]},
                 tcp_offset_xyz_m=catalog.get("tool_frame_offset_xyz",[0.0,0.0,0.0]),
                 tcp_offset_rpy_rad=catalog.get("tool_frame_offset_rpy",[0.0,0.0,0.0]),
                 contact={"required":True,"min_quality":0.0},
                 aperture={"min_m":0.0,"max_m":0.085},
                 lift={"axis":"z_up","distance_m":catalog["retreat_distance_m"]})
    intent["safety"]={"execution_mode":"simulation_preview","execution_backend":"simulator",
                      "require_fake_hardware":False,"real_hardware_enabled":False,
                      "preview_policy":"diagnostic_if_unresolved"}
    (destination/"config/workcell_builder_task_intent.yaml").write_text(
        yaml.safe_dump(intent,sort_keys=False))

    export_scene(destination,destination,validate=True)
    rc=generate_package(destination/"cell_definition.yaml",destination.parent,destination.name,
                        False,False,existing_package_dir=destination)
    if rc:raise RuntimeError(f"initial Stage-A package generation failed ({rc})")
    return destination


def generate_scene(scene:Path):
    from export_builder_scene_to_cell_definition import export_scene
    from generate_workcell_from_cell_definition import generate_package
    export_scene(scene,scene,validate=True)
    rc=generate_package(scene/"cell_definition.yaml",scene.parent,scene.name,
                        False,False,existing_package_dir=scene)
    if rc:raise RuntimeError(f"Stage-A package generation failed ({rc})")


def build_commissioning(repo:Path,workspace:Path,output:Path,env:dict)->dict:
    from simulator_backend import active_moveit_overlay
    overlay=active_moveit_overlay()
    build_log=output/"commissioning-build.log"
    run([repo/"scripts/build_commissioning_capability.sh"],env=env,cwd=repo,log=build_log,timeout=600)
    manifest_path=workspace/"install/workcell_builder/share/workcell_builder/commission_execute_build.json"
    if not manifest_path.is_file():raise RuntimeError("commissioning build manifest missing after build")
    manifest=json.loads(manifest_path.read_text())
    if manifest.get("moveit_overlay",{}).get("library",{}).get("sha256")!=overlay["library"]["sha256"]:
        raise RuntimeError("commissioning build did not bind the qualified MoveIt overlay")
    library=Path(manifest["library"])
    if not library.is_file() or sha256(library)!=manifest.get("sha256"):
        raise RuntimeError("commissioning library does not match its build manifest")
    test_env=dict(env,ROS_DOMAIN_ID="200",ROS_LOCALHOST_ONLY="1",ROS2CLI_DISABLE_DAEMON="1")
    run([workspace/"build/workcell_builder/workcell_execute_action_test"],env=test_env,cwd=repo,
        log=output/"commissioning-action-tests.log",timeout=120)
    run([workspace/"build/workcell_builder/workcell_support_contact_test"],env=test_env,cwd=repo,
        log=output/"support-cartesian-adapter-tests.log",timeout=120)
    telemetry=Path(manifest.get("telemetry_library",""))
    if not telemetry.is_file() or sha256(telemetry)!=manifest.get("telemetry_sha256"):
        raise RuntimeError("commissioning telemetry library does not match its build manifest")
    installed_telemetry=workspace/"install/workcell_builder/lib/libworkcell_simulator_measurements.so"
    if not installed_telemetry.is_file() or installed_telemetry.resolve()!=telemetry.resolve():
        raise RuntimeError("commissioning telemetry library is not registered in the workspace overlay")
    support=Path(manifest.get("support_library",""))
    if not support.is_file() or sha256(support)!=manifest.get("support_sha256"):
        raise RuntimeError("support/Cartesian planning library does not match its build manifest")
    installed_support=workspace/"install/workcell_builder/lib/libworkcell_support_contact.so"
    if not installed_support.is_file() or installed_support.resolve()!=support.resolve():
        raise RuntimeError("support/Cartesian planning library is not registered in the workspace overlay")
    return {"moveit_overlay":overlay,"sha256":manifest["sha256"],"moveit_version":manifest.get("moveit_version"),
            "library":str(library),"telemetry_sha256":manifest["telemetry_sha256"],
            "telemetry_library":str(telemetry),"support_sha256":manifest["support_sha256"],
            "support_library":str(support)}


def assert_plan(summary,*,require_resolved):
    if summary.get("result")!="PLAN_ONLY" or summary.get("full_cycle_prevalidated") is not True or summary.get("execution_attempted") is not False:
        raise RuntimeError("current authoritative plan-only cycle did not pass")
    plans=summary.get("plan_metadata",[])
    if [item.get("stage") for item in plans]!=STAGES:
        raise RuntimeError("current plan does not contain the complete nine-stage cycle")
    if any(item.get("success") is not True or item.get("moveit_code")!=1 or item.get("points",0)<2 for item in plans):
        raise RuntimeError("one or more current cycle stages are not executable plans")
    resolution=summary.get("task_intent_resolution",{})
    if require_resolved and resolution.get("readiness_status") not in ("READY","WARNING"):
        raise RuntimeError("fresh Resolve did not produce a consumable task handoff")


def assert_gate(gate,summary,capability_sha,overlay_sha):
    expected=EXPECTED_RESULTS[gate]
    if summary.get("result")!=expected:raise RuntimeError(f"{gate} gate returned {summary.get('result')!r}, expected {expected!r}")
    if summary.get("full_cycle_prevalidated") is not True:raise RuntimeError(f"{gate} gate did not revalidate the complete cycle")
    if summary.get("commissioning_capability",{}).get("sha256")!=capability_sha:
        raise RuntimeError(f"{gate} used a different commissioning capability binary")
    if summary.get("commissioning_capability",{}).get("moveit_overlay",{}).get("library",{}).get("sha256")!=overlay_sha:
        raise RuntimeError(f"{gate} used a different MoveIt teardown library")
    if gate=="cancel":
        if not summary.get("cancellation_confirmed") or not summary.get("motion_stop_verified"):
            raise RuntimeError("fresh cancellation qualification is incomplete")
    elif gate=="telemetry":
        metrics=summary.get("motion_telemetry",{})
        if metrics.get("max_fresh_age_ms",1e9)>=250 or metrics.get("max_delivery_ms",1e9)>=250:
            raise RuntimeError("motion telemetry does not satisfy the unchanged 250 ms guard")
    elif gate=="stationary":
        retention=summary.get("stationary_retention",{})
        if retention.get("duration_sim_ns",0)<1_000_000_000:
            raise RuntimeError("stationary physical retention is shorter than one second")
        if set(retention.get("required_contact_links",[]))-set(retention.get("contact_links",[])):
            raise RuntimeError("stationary retention lacks opposing required contacts")
    elif gate=="contact-release":
        if summary.get("verified_lift_clearance_m",0)<.01 or not summary.get("release_evidence",{}).get("settled"):
            raise RuntimeError("contact-release did not prove lift and physical settling")
    elif gate=="full-cycle":
        final=summary.get("full_cycle_physical_acceptance",{})
        if not summary.get("full_cycle_execution_success") or not final.get("final_collision_valid") or final.get("attached_ids"):
            raise RuntimeError("full physical cycle final acceptance is incomplete")


def executor_command(repo,scene,receipt,observations,summary,planning_time,*,resolve=False,gate=None,evidence=None):
    cmd=[sys.executable,str(repo/"scripts/perceived_object_grasp_execute.py"),
         "--scene-package",str(scene),"--backend","simulator","--simulator-receipt",str(receipt),
         "--detections",str(observations),"--timeout","300","--segment-planning-time",str(planning_time),
         "--summary-output",str(summary)]
    if resolve:cmd.append("--resolve-task")
    if gate is not None:
        cmd+=["--start","--simulator-commission",gate]
        if gate=="full-cycle":
            if evidence is None:raise ValueError("full-cycle gate requires prerequisite evidence")
            cmd+=["--commission-evidence",str(evidence)]
    return cmd


def verify_session_moveit(domain,partition,expected_sha):
    from simulator_backend import live_moveit_overlay
    return live_moveit_overlay(domain,partition,expected_sha)


def one_session(args,repo,source_world,capability_sha,gate,index,prior):
    session=args.output/f"{index:02d}-{gate}"
    if session.exists():raise RuntimeError(f"refusing to reuse evidence directory {session}")
    session.mkdir(parents=True)
    scene=prepare_scene(repo,session/"ur5_2f_test",args.class_id)
    runtime=session/"runtime";receipt=runtime/"receipt.json";observations=session/"parts.yaml"
    domain=args.base_domain+index-1
    if not 1<=domain<=232:raise RuntimeError("ROS domain range exhausted")
    env=dict(os.environ,ROS_DOMAIN_ID=str(domain),IGN_PARTITION=f"{args.partition_prefix}_{gate}",
             ROS_LOCALHOST_ONLY="1",ROS2CLI_DISABLE_DAEMON="1",PYTHONUNBUFFERED="1")
    report={"gate":gate,"domain":domain,"partition":env["IGN_PARTITION"],"status":"BLOCKED"}
    launch=None;launch_log=None
    try:
        assert_domain_free(env,repo)
        generate_scene(scene)
        launch_log=(session/"launch.log").open("w")
        launch_cmd=["ros2","launch","ur5_2f_test","demo.launch.py",
            "execution_backend:=simulator","simulator_commissioning:=true",
            f"simulator_world:={source_world}",f"simulator_output:={runtime}",
            "use_sim_time:=true","launch_rviz:=false","use_fake_hardware:=true",
            "allow_trajectory_execution:=true"]
        launch=subprocess.Popen(launch_cmd,cwd=repo,env=env,stdout=launch_log,stderr=subprocess.STDOUT,start_new_session=True)
        wait_file(receipt,launch,90,session/"launch.log")
        run([sys.executable,repo/"scripts/simulator_observations.py","--receipt",receipt,
             "--output",observations,"--class-id",args.class_id],
            env=env,cwd=repo,log=session/"observations.log",timeout=90)

        def refresh_observations(label):
            run([sys.executable,repo/"scripts/simulator_observations.py","--receipt",receipt,
                 "--output",observations,"--class-id",args.class_id,
                 "--refresh-from",observations],
                env=env,cwd=repo,log=session/f"{label}-observations.log",timeout=90)

        report["moveit_overlay"]=verify_session_moveit(domain,env["IGN_PARTITION"],args.moveit_overlay_sha256)
        resolve_summary=session/"resolve/summary.json"
        run(executor_command(repo,scene,receipt,observations,resolve_summary,args.segment_planning_time,resolve=True),
            env=env,cwd=repo,log=session/"resolve/executor.log",timeout=420)
        resolved=json.loads(resolve_summary.read_text());assert_plan(resolved,require_resolved=True)

        # The search may be long. Re-measure the same geometry and renew only
        # timestamps before consuming the saved resolution.
        refresh_observations("revalidate")
        generate_scene(scene)
        plan_summary=session/"revalidate/summary.json"
        run(executor_command(repo,scene,receipt,observations,plan_summary,args.segment_planning_time),
            env=env,cwd=repo,log=session/"revalidate/executor.log",timeout=420)
        planned=json.loads(plan_summary.read_text());assert_plan(planned,require_resolved=True)
        report.update(selected_object_id=planned.get("selected_object_id"),
            selected_grasp_index=planned.get("selected_grasp_index"),
            resolution_sha256=planned.get("resolution_sha256"),no_motion_revalidation="PASS")
        if gate=="resolve":report["status"]="PASS";return plan_summary

        # Motion never relies on the planning-search timestamp. Prove the same
        # physical geometry is still present immediately before the gate.
        refresh_observations(gate)
        evidence=None
        if gate=="full-cycle":
            required=["cancel","telemetry","stationary","contact-release"]
            if any(name not in prior for name in required):raise RuntimeError("full-cycle prerequisite evidence is incomplete")
            records=[json.loads(prior[name].read_text()) for name in required]
            evidence=session/"commission-evidence.json";evidence.write_text(json.dumps(records,indent=2)+"\n")
        gate_summary=session/f"{gate}/summary.json"
        run(executor_command(repo,scene,receipt,observations,gate_summary,args.segment_planning_time,gate=gate,evidence=evidence),
            env=env,cwd=repo,log=session/f"{gate}/executor.log",timeout=600)
        result=json.loads(gate_summary.read_text());assert_gate(gate,result,capability_sha,args.moveit_overlay_sha256)
        report.update(status="PASS",result=result["result"],selected_object_id=result.get("selected_object_id"),
                      selected_grasp_index=result.get("selected_grasp_index"))
        return gate_summary
    except Exception as exc:
        report["failure"]=str(exc);raise
    finally:
        report["shutdown"]=stop_owned(launch,session/"launch.log" if launch_log is not None else None)
        if launch_log is not None:launch_log.close()
        shutdown_failure=None
        if not report["shutdown"]["clean"]:
            shutdown_failure="owned shutdown failed: "+json.dumps(report["shutdown"],sort_keys=True)
            report["status"]="BLOCKED"
            report["shutdown_failure"]=shutdown_failure
        (session/"session-report.json").write_text(json.dumps(report,indent=2)+"\n")
        if shutdown_failure is not None and "failure" not in report:
            raise RuntimeError(shutdown_failure)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-world",type=Path)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--base-domain",type=int,default=201)
    parser.add_argument("--partition-prefix",default="stage_a1_finish")
    parser.add_argument("--class-id",default="part")
    parser.add_argument("--segment-planning-time",type=float,default=3.0)
    parser.add_argument("--through",choices=GATES,default="full-cycle")
    args=parser.parse_args(argv)

    repo=Path(__file__).resolve().parents[1];workspace=repo.parent.parent
    args.output=args.output.resolve()
    if args.output.exists():parser.error("choose a new output directory")
    if not 0<args.segment_planning_time<=10:parser.error("segment planning time must be in (0,10]")
    args.output.mkdir(parents=True)
    overall={"status":"BLOCKED","gates":{},"preflight":None}
    try:
        source_world=discover_source_world(args.source_world,workspace)
        build=build_commissioning(repo,workspace,args.output,dict(os.environ))
        args.moveit_overlay_sha256=build["moveit_overlay"]["library"]["sha256"]
        overall["preflight"]={"repo_head":subprocess.check_output(["git","rev-parse","HEAD"],cwd=repo,text=True).strip(),
            "repo_dirty":bool(subprocess.check_output(["git","status","--porcelain"],cwd=repo,text=True).strip()),
            "source_world":str(source_world),"source_world_git_blob":SOURCE_WORLD_GIT_BLOB,
            "source_world_sha256":sha256(source_world),"commissioning_build":build}
        prior={}
        selected=GATES[:GATES.index(args.through)+1]
        for index,gate in enumerate(selected,1):
            try:
                path=one_session(args,repo,source_world,build["sha256"],gate,index,prior)
            except Exception as exc:
                failed={"status":"FAILED","failure":str(exc)}
                session_report=args.output/f"{index:02d}-{gate}"/"session-report.json"
                if session_report.is_file():
                    failed["session_report"]=str(session_report.relative_to(args.output))
                    recorded=json.loads(session_report.read_text())
                    for key in ("shutdown","shutdown_failure","moveit_overlay"):
                        if key in recorded:failed[key]=recorded[key]
                overall["gates"][gate]=failed
                raise
            prior[gate]=path;overall["gates"][gate]={"status":"PASS","summary":str(path.relative_to(args.output))}
            overall["status"]="PASS" if gate==args.through else "IN_PROGRESS"
        return 0
    except Exception as exc:
        overall["status"]="BLOCKED";overall["failure"]=str(exc);return 1
    finally:
        overall["finished_wall_ns"]=time.time_ns()
        (args.output/"stage-a1-final-report.json").write_text(json.dumps(overall,indent=2)+"\n")
        print(json.dumps(overall,indent=2))


if __name__=="__main__":raise SystemExit(main())
