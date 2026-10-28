#!/usr/bin/env python3
"""Finish Stage-A Fortress commissioning with fresh, fail-closed sessions.

The runner is portable across the user's two ROS workstations. It prepares a
fresh simulator-locked TaskIntent scene from the tracked canonical UR5+2F scene,
rebuilds/tests the opt-in commissioning capability, re-qualifies cancellation
on the current binary, and then advances only through measured gates that pass.
It never targets real hardware and never replays an old trajectory.

Modes (--mode):
  full      the six-gate acceptance sequence (default; --through stops early).
            Only a full run through full-cycle can report Stage-A acceptance.
  targeted  one gate (--gate) in its own fresh, owned session with the same
            preflight, isolation, resolve, revalidation, gate checks and owned
            cleanup as in a full run. Diagnostic only: never acceptance.
  fast      no ROS launch: the commissioning build with its C++ unit tests and
            the focused Stage-A Python tests. Diagnostic only: never acceptance.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import re
from pathlib import Path
import shutil
import signal
import select
import subprocess
import sys
import time

import yaml

SOURCE_WORLD_GIT_BLOB="b533d6de2b454939f0ebedbcfb3aed5ccb16051b"
# Separately pinned worlds. "pile" is the dense Stage-A benchmark; "sparse" is
# the single-workpiece commissioning world used to qualify physical execution
# apart from pile accessibility. Both use the same cell and gates.
SOURCE_WORLDS={
    "pile":("stage_a0.sdf",SOURCE_WORLD_GIT_BLOB),
    "sparse":("stage_a0_sparse.sdf","73484f0b45b6239843720fb3370c0754bf02ebd3"),
}
STAGES=[
    "PREPLAN_APPROACH","PREPLAN_GRASP","PREPLAN_CLOSE_GRIPPER",
    "PREPLAN_LIFT","PREPLAN_TRANSFER","PREPLAN_PLACE",
    "PREPLAN_OPEN_GRIPPER","PREPLAN_RETREAT","PREPLAN_HOME",
]
GATES=("resolve","cancel","telemetry","stationary","contact-release","full-cycle")
RUN_SCOPES={"full":"STAGE_A_ACCEPTANCE_SEQUENCE","targeted":"DIAGNOSTIC_NOT_STAGE_A_ACCEPTANCE",
            "fast":"DIAGNOSTIC_NOT_STAGE_A_ACCEPTANCE","functional":"FUNCTIONAL_VALIDATION_NOT_STAGE_A_ACCEPTANCE"}
# Run settings whose silent defaults change what the evidence means.
INVOCATION_SETTINGS=("world_profile","search_timeout","segment_planning_time","motion_policy")
# First-reply service retries (perceived_object_grasp_execute.SERVICE_RETRY_ALLOWANCE_S):
# 11 clients x 2 retries x 10 s may precede or interleave the search in every session.
SERVICE_RETRY_ALLOWANCE_S=220
# Execution gates: the executor's owned waits (carried steps up to 300 s each, fixed 120 s
# steps, release, holds, service retries) may total ~2 ks after the search; the last-resort
# subprocess kill must come after the executor's own cancellation, never before it.
EXECUTION_GATE_MARGIN_S=2100
MODES=("full","targeted","fast","functional")
# full-cycle needs passing cancel, telemetry, stationary and contact-release
# evidence on the same binaries (executor --commission-evidence); the runner only
# ever builds that file from the sessions of the same full run, so it cannot run alone.
TARGETABLE_GATES=GATES[:-1]
REPORT_NAMES={"full":"stage-a1-final-report.json","targeted":"stage-a1-targeted-report.json",
              "functional":"stage-a1-functional-report.json",
              "fast":"stage-a1-fast-report.json"}
# Focused Stage-A Python suites: the suites of the runner, the executor and every
# local module they import (checked by a test), plus the recovery/runtime suites
# that exercise the executor's closures. None launches ROS or Gazebo.
FAST_TEST_FILES=(
    "tests/test_bridge_shutdown_overlay.py",
    "tests/test_closure_clearance.py","tests/test_destination_binding.py",
    "tests/test_end_effector_hold.py","tests/test_simulation_collision.py","tests/test_simulator_world.py","tests/test_scene_package_dir_launch.py","tests/test_r4_1b_dynamic_runtime.py",
    "tests/test_perception_replay.py","tests/test_live_perception_session.py",
    "tests/test_end_effector_selection.py",
    "tests/test_tool_interface.py","tests/test_transport_policy.py",
    "tests/test_extraction_runtime.py","tests/test_full_cycle_preplanner.py",
    "tests/test_grasp_capture.py","tests/test_grasp_recovery_boundary.py",
    "tests/test_grasp_recovery_runtime.py","tests/test_grasp_strategy_candidates.py",
    "tests/test_perceived_object_grasp_execute.py","tests/test_perceived_object_grasp_plan.py",
    "tests/test_physical_destination.py","tests/test_pile_extraction.py",
    "tests/test_retention_cancel_fault.py","tests/test_runtime_pick_inputs.py",
    "tests/test_simulator_backend.py","tests/test_simulator_box_motion.py",
    "tests/test_simulator_execution.py","tests/test_stage_a1_finish_runner.py",
    "tests/test_task_intent_resolver.py","tests/test_task_intent_v2.py",
    "tests/test_transactional_pick_cycle.py",
)
FAST_TEST_TIMEOUT_S=600
# Runtime-only selections the launch consumes. Unit fixtures build their own mock
# worlds; a test that needs one of these sets it itself (monkeypatch).
# The qualified physics-root path itself is exercised only by runtime sessions.
FAST_TEST_ENV_EXCLUDED=("WORKCELL_SIMULATOR_PHYSICS_ROOT","WORKCELL_PLANNING_TRACE_DIR")
# Opt-in executor diagnostic (simulator_execution.GilHoldWatch); bounds match its own check.
GIL_WATCHDOG_ENV="WORKCELL_GIL_WATCHDOG_MS"
# Opt-in native GIL-holder probe (scripts/gil_probe.c, simulator_execution.GilProbe).
GIL_PROBE_LIB_ENV="WORKCELL_GIL_PROBE_LIB"
GIL_PROBE_MS_ENV="WORKCELL_GIL_PROBE_MS"
GIL_PROBE_NATIVE_PC_ENV="WORKCELL_GIL_PROBE_NATIVE_PC"
# Trial 26: the faulthandler watchdog's own re-arm held the GIL for 400 ms and failed a lift.
# It may observe the plan-only resolve gate only, never a gate that moves the robot.
GIL_WATCHDOG_GATES=("resolve",)
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


def owned_handles(pid):
    """Bind live ancestry before cleanup can orphan a native simulator child."""
    handles={};identities=[];errors=[];root=None
    def exited(fd):return bool(select.select([fd],[],[],0)[0])
    try:
        root=os.pidfd_open(pid)
        fields=Path(f"/proc/{pid}/stat").read_text().rsplit(")",1)[1].split()
        if exited(root):return {},[],['supervisor exited before ownership capture']
        if int(fields[1])!=os.getpid():return {},[],['supervisor is not a child of this caller']
        handles[pid]=root;root=None;identities.append(dict(pid=pid,start_ticks=fields[19]))
        pending=[pid]
        while pending:
            parent=pending.pop();parent_fd=handles[parent]
            for path in Path('/proc').iterdir():
                if not path.name.isdigit() or int(path.name) in handles:continue
                fd=None
                try:
                    before=(path/'stat').read_text().rsplit(')',1)[1].split()
                    if before[0]=='Z' or int(before[1])!=parent:continue
                    fd=os.pidfd_open(int(path.name))
                    after=(path/'stat').read_text().rsplit(')',1)[1].split()
                    # The parent handle must still name the live parent *after*
                    # the child's identity and current PPID have been read.
                    if exited(parent_fd):
                        errors.append('parent exited during ownership capture');break
                    if after[0]=='Z' or after[19]!=before[19] or int(after[1])!=parent:continue
                    child=int(path.name);handles[child]=fd;fd=None;pending.append(child)
                    identities.append(dict(pid=child,start_ticks=after[19]))
                except (FileNotFoundError,ProcessLookupError):continue
                except OSError as exc:errors.append(str(exc))
                finally:
                    if fd is not None:os.close(fd)
            if exited(parent_fd):errors.append('parent exited during ownership capture')
    except (FileNotFoundError,ProcessLookupError):errors.append('supervisor missing before ownership capture')
    except OSError as exc:errors.append(str(exc))
    finally:
        if root is not None:os.close(root)
    return handles,identities,errors


def stop_owned(process,log=None):
    if process is None:return {"started":False,"clean":True,"escalation":None}
    # Snapshot before signalling: a previously dead launch/long-lived child is
    # never excused by the SIGINT that the runner sends later.
    cleanup_offset=Path(log).stat().st_size if log is not None else 0
    alive_before=process.poll() is None
    escalation=None;sent_signals=[]
    pidfds,owned,ownership_errors=owned_handles(process.pid) if alive_before else ({},[],[])
    try:
        def running_owned():
            return [pid for pid,fd in pidfds.items() if not select.select([fd],[],[],0)[0]]
        def alive():return bool(running_owned())
        def send(sig,*,supervisor_only=False):
            targets=[process.pid] if supervisor_only else running_owned()
            delivered=False
            for pid in targets:
                if pid not in pidfds:continue
                try:signal.pidfd_send_signal(pidfds[pid],sig);delivered=True
                except ProcessLookupError:pass
            if delivered:sent_signals.append(sig)
        # ROS launch owns child signalling/reaping. Interrupt its supervisor once;
        # broadcasting first makes launch send a second SIGINT and can race its
        # asyncio child watcher into losing a fast child's real exit status.
        if alive_before:send(signal.SIGINT,supervisor_only=True)
        deadline=time.monotonic()+10
        while alive() and time.monotonic()<deadline:time.sleep(.1)
        if alive():
            escalation="SIGTERM";send(signal.SIGTERM);deadline=time.monotonic()+5
            while alive() and time.monotonic()<deadline:time.sleep(.1)
        if alive():
            escalation="SIGKILL";send(signal.SIGKILL);deadline=time.monotonic()+3
            while alive() and time.monotonic()<deadline:time.sleep(.05)
        try:rc=process.wait(timeout=3)
        except subprocess.TimeoutExpired:rc=None
        children=launch_child_exits(log,cleanup_offset=cleanup_offset,sent_signals=sent_signals)
        crash_evidence=[];launch_escalation_evidence=[]
        if log is not None:
            lines=Path(log).read_text(errors="replace").splitlines()
            crash_evidence=[line for line in lines
                if re.search(r"segmentation fault|core dumped|\bSIGSEGV\b|\bSIGABRT\b",line,re.IGNORECASE)]
            # ROS launch may force a nested child to terminate while the supervisor
            # and child eventually return zero. That is still forced cleanup.
            launch_escalation_evidence=[line for line in lines
                if re.search(r"failed to terminate.*escalating to ['\"]?SIG(?:TERM|KILL)",line)]
        survivors=running_owned();remaining=alive()
        root_expected=alive_before and bool(sent_signals) and (rc==0 or
            (rc in (-signal.SIGINT,-signal.SIGTERM) and -rc in sent_signals))
        report={"started":True,"clean":not ownership_errors and not remaining and root_expected and not crash_evidence
                and escalation is None and not launch_escalation_evidence and all(child["expected"] for child in children),
                "returncode":rc,"escalation":escalation,"remaining_owned_processes":remaining,
                "root_alive_before_cleanup":alive_before,"cleanup_log_offset":cleanup_offset,
                "sent_signals":[signal.Signals(item).name for item in sent_signals],
                "children":children,"crash_evidence":crash_evidence,
                "launch_escalation_evidence":launch_escalation_evidence,
                "owned_processes_before_cleanup":owned,"surviving_owned_processes":survivors,
                "ownership_errors":ownership_errors}
        return report

    finally:
        for fd in pidfds.values():os.close(fd)


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


CONTROLLER_STATES_PROBE='''import json,rclpy
from controller_manager_msgs.srv import ListControllers
rclpy.init();node=rclpy.create_node('stage_a1_controller_readiness');states={}
try:
    client=node.create_client(ListControllers,'/controller_manager/list_controllers')
    if client.wait_for_service(timeout_sec=5.):
        future=client.call_async(ListControllers.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=5.)
        if future.done() and future.result() is not None:states={c.name:c.state for c in future.result().controller}
finally:
    node.destroy_node();rclpy.shutdown()
print(json.dumps(states))
'''


def wait_controllers_active(receipt,process,env,cwd,timeout):
    """The receipt appears at spawn, before the controller spawners finish; the executor's
    identity preflight then rejects the session (controllers not active). Wait, bounded, for
    the receipt's expected controllers to be active; the executor still verifies identity."""
    expected=json.loads(Path(receipt).read_text()).get("expected_controllers") or []
    if not expected:raise RuntimeError("receipt names no expected controllers")
    deadline=time.monotonic()+timeout;states={}
    while time.monotonic()<deadline:
        if process.poll() is not None:
            raise RuntimeError(f"owned launch exited before controllers became active (rc={process.returncode})")
        probe=subprocess.run([sys.executable,"-c",CONTROLLER_STATES_PROBE],cwd=cwd,env=env,stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,text=True,timeout=30,check=False)
        try:states=json.loads(probe.stdout.strip().splitlines()[-1]) if probe.returncode==0 else {}
        except (ValueError,IndexError):states={}
        if all(states.get(name)=="active" for name in expected):return states
        time.sleep(.5)
    raise TimeoutError(f"controllers not active after {timeout:.0f} s: {states}")


def assert_domain_free(env,cwd):
    probe=subprocess.run(["ros2","node","list","--no-daemon"],cwd=cwd,env=env,stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,text=True,timeout=10,check=False)
    nodes=[line.strip() for line in probe.stdout.splitlines() if line.strip().startswith("/")]
    if probe.returncode or nodes:
        raise RuntimeError(f"ROS domain {env['ROS_DOMAIN_ID']} is not isolated: {nodes or probe.stdout.strip()}")


def git_blob(path:Path,repo:Path)->str:
    return subprocess.check_output(["git","hash-object",str(path)],cwd=repo,text=True).strip()


def candidate_worlds(workspace:Path,profile:str="pile"):
    repo=Path(__file__).resolve().parents[1]
    if profile!="pile":
        yield repo/"scenes/ur5_2f_test/worlds"/SOURCE_WORLDS[profile][0]
        return
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


def discover_source_world(explicit:Path|None,workspace:Path,profile:str="pile")->Path:
    repo=Path(__file__).resolve().parents[1]
    expected=SOURCE_WORLD_GIT_BLOB if profile=="pile" else SOURCE_WORLDS[profile][1]
    candidates=[explicit] if explicit is not None else list(candidate_worlds(workspace,profile))
    for path in candidates:
        if path is None or not path.is_file():continue
        if git_blob(path,repo)!=expected:continue
        text=path.read_text(errors="ignore").lower()
        if any(token in text for token in ("workcell::simulatormeasurements","workcell_measurements",
                "gz_ros2_control","ign_ros2_control","attach","magnet","suction")):continue
        return path.resolve()
    checked=", ".join(str(p) for p in candidates if p is not None)
    raise RuntimeError("pristine Stage-A world not found on this workstation; checked: "+checked)


def prepare_scene(repo:Path,destination:Path,class_id:str,motion_policy:dict|None=None)->Path:
    """Materialize the tracked canonical cell into the exact Stage-A task policy.

    ``motion_policy`` (opt-in) is authored into the copy's environment.yaml, the
    same place a cell author writes it; the tracked scene is never modified.
    """
    source=repo/"scenes/ur5_2f_test"
    shutil.copytree(source,destination)
    if motion_policy is not None:
        environment_path=destination/"environment.yaml"
        authored=yaml.safe_load(environment_path.read_text())
        authored["motion_policy"]=motion_policy
        environment_path.write_text(yaml.safe_dump(authored,sort_keys=False))
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


def stream(args,stage,**fields):
    """--stream-status: one JSON line per real session stage (live qualified physics, never replay)."""
    if getattr(args,"stream_status",False):
        print(json.dumps(dict(stage=stage,source="qualified_physics_simulator",live=True,**fields)),flush=True)


class OperatorCancel(BaseException):
    """SIGTERM/SIGINT to the runner (e.g. the Workcell Builder's Cancel): unwind through every owned cleanup."""


def cancel_on_signal(signum,frame):
    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,signal.SIG_IGN)   # cleanup is never re-interrupted
    raise OperatorCancel(signal.Signals(signum).name)


CYCLE_RESULT_FIELDS=("result","failure","failed_stage","selected_object_id","selected_grasp_index","execution_attempted",
                     "full_cycle_plan_success","full_cycle_execution_success","shutdown_clean","cancellation_confirmed","recovery_required",
                     "perception","rejected_objects")


def cycle_result(overall,output:Path)->dict:
    """The run's outcome for the operator: gate statuses plus the executed gate's own measured summary."""
    gates=overall.get("gates") or {}
    summary={}
    if gates:
        name,last=list(gates.items())[-1]
        path=output/last["summary"] if last.get("summary") else output/f"{GATES.index(name)+1:02d}-{name}"/name/"summary.json"
        if path.is_file():
            executed=json.loads(path.read_text())
            summary={key:executed[key] for key in CYCLE_RESULT_FIELDS if key in executed}
            final=(executed.get("release_evidence") or {}).get("final_pose")
            if final is not None:summary["final_object_pose"]=final
    cancelled=overall.get("status")=="CANCELLED"
    if cancelled:
        # Cancellation is confirmed only by the clean owned shutdown of every session that ran.
        summary.update(cancellation_confirmed=bool(gates) and all((gate.get("shutdown") or {}).get("clean") is True
                                                                  for gate in gates.values()),
                       recovery_required=False)   # nothing outlives a torn-down simulator session
    return dict(summary,mode="physics_simulation",live=True,status=overall.get("status"),
                result=summary.get("result") if overall.get("status")=="PASS" else "CANCELLED" if cancelled else "FAIL",
                failure=overall.get("failure") or summary.get("failure"),
                gates={name:gate.get("status") for name,gate in gates.items()},
                authored_cell=(overall.get("authored_cell") or {}).get("package"),
                stage_a_acceptance=overall.get("stage_a_acceptance"),evidence=str(output))


def launch_checkout_guard(repo:Path)->str:
    """The installed scene launch (and the backend it imports) must be this checkout's, never another workspace's."""
    from ament_index_python.packages import get_package_share_directory,PackageNotFoundError
    try:launch=Path(get_package_share_directory("ur5_2f_test"))/"launch/demo.launch.py"
    except PackageNotFoundError as exc:raise RuntimeError("SCENE_PACKAGE_UNINSTALLED: build ur5_2f_test in this workspace") from exc
    resolved=launch.resolve()
    if not resolved.is_relative_to(repo.resolve()):
        raise RuntimeError(f"SCENE_PACKAGE_FROM_OTHER_CHECKOUT: the installed launch {resolved} is not this checkout's "
                           f"({repo}); build ur5_2f_test with --symlink-install in this workspace")
    return str(resolved)


def authored_cell(package:Path)->dict:
    """A Workcell Builder-saved cell bound for the qualified simulator, read without modification."""
    package=Path(package).resolve()
    for rel in ("environment.yaml","cell_definition.yaml","config/workcell_builder_task_intent.yaml"):
        if not (package/rel).is_file():
            raise RuntimeError(f"AUTHORED_CELL_INCOMPLETE: {package} has no {rel}; save and Generate the cell in Workcell Builder")
    intent=yaml.safe_load((package/"config/workcell_builder_task_intent.yaml").read_text())
    if not isinstance(intent,dict) or intent.get("schema")!="workcell_builder_task_intent/v2":
        raise RuntimeError("AUTHORED_TASK_NOT_V2: save the task in the Workcell Builder Task Editor")
    safety=intent.get("safety") or {}
    if (safety.get("execution_backend")!="simulator" or safety.get("require_fake_hardware") is not False or
            safety.get("real_hardware_enabled") is not False):
        raise RuntimeError("AUTHORED_TASK_NOT_SIMULATOR: the saved task does not select the qualified physics "
                           "simulation (Task Editor > Run > Execution backend), so it may not move in the simulator")
    from task_intent_v2 import freshness_source_type
    perception=None
    if freshness_source_type(intent)=="replayed_perception_in_simulation":
        # Stage-B: the saved Task input is a replayed EPD session; each session detects from its recording again.
        replay=Path(intent["perception"].get("replay_path") or "")
        replay=replay if replay.is_absolute() else package/replay
        if not replay.is_file():
            raise RuntimeError(f"PERCEPTION_REPLAY_MISSING: {replay}; record and detect the cell camera session first")
        snapshot=yaml.safe_load(replay.read_text()) or {}
        from perception_replay import resolve_recording
        perception=dict(replay=str(replay),recording=str(resolve_recording((snapshot.get("source") or {}).get("recording"),package)),
                        target_class=intent["pick"]["selection"]["object_filter"].get("class_id"))
    return dict(package=str(package),name=package.name,intent_sha256=sha256(package/"config/workcell_builder_task_intent.yaml"),
                perception=perception)


def prepare_authored_scene(repo:Path,package:Path,destination:Path)->Path:
    """Copy the saved cell verbatim; the session never rewrites its task, scene or tool."""
    shutil.copytree(package,destination,ignore=shutil.ignore_patterns("build","install","log"))
    # The session regenerates its own copy; the saved cell itself may be read-only and stays untouched.
    for path in [destination,*destination.rglob("*")]:
        if not path.is_symlink():path.chmod(path.stat().st_mode|0o200)
    catalog_parent=destination.parent/"catalog"
    if not catalog_parent.exists():
        shutil.copytree(repo/"catalog/grasp_strategies",catalog_parent/"grasp_strategies")
    return destination


def build_commissioning(repo:Path,workspace:Path,output:Path,env:dict)->dict:
    from simulator_backend import active_moveit_overlay,moveit_overlay_identity
    overlay=active_moveit_overlay()
    build_log=output/"commissioning-build.log"
    run([repo/"scripts/build_commissioning_capability.sh"],env=env,cwd=repo,log=build_log,timeout=600)
    manifest_path=workspace/"install/workcell_builder/share/workcell_builder/commission_execute_build.json"
    if not manifest_path.is_file():raise RuntimeError("commissioning build manifest missing after build")
    manifest=json.loads(manifest_path.read_text())
    if moveit_overlay_identity(manifest.get("moveit_overlay",{}))!=moveit_overlay_identity(overlay):
        raise RuntimeError("commissioning build did not bind the qualified MoveIt overlay")
    library=Path(manifest["library"])
    if not library.is_file() or sha256(library)!=manifest.get("sha256"):
        raise RuntimeError("commissioning library does not match its build manifest")
    test_env=dict(env,ROS_DOMAIN_ID="200",ROS_LOCALHOST_ONLY="1",ROS2CLI_DISABLE_DAEMON="1")
    # Unit fixtures must not enter the optional live planning evidence directory.
    test_env.pop("WORKCELL_PLANNING_TRACE_DIR",None)
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


def assert_release_evidence(gate,summary,retention):
    from simulator_backend import PHYSICS_POSE_METHOD
    evidence=summary.get('release_evidence') or {}
    goal=evidence.get('goal') or {}
    observed=evidence.get('separation') or {}
    source=observed.get('pose_source') or {}
    detached=evidence.get('detachment') or {}
    binding=retention.get('binding') or {}
    if (evidence.get('run_id')!=retention.get('run_id') or
        evidence.get('target')!=summary.get('selected_object_id') or
        evidence.get('binding')!=binding or
        goal!=summary.get('owned_open_goal') or
        goal.get('run_id')!=retention.get('run_id') or
        goal.get('target')!=summary.get('selected_object_id') or
        goal.get('resolution_sha256')!=summary.get('resolution_sha256') or
        goal.get('execution_attempt')!=binding.get('execution_attempt') or
        goal.get('selected_grasp_index')!=summary.get('selected_grasp_index') or
        goal.get('stage')!=('COMMISSION_RELEASE' if gate=='contact-release' else 'EXECUTE_OPEN_GRIPPER') or
        not goal.get('uuid') or goal.get('accepted') is not True or
        goal.get('terminal_status')!=4 or goal.get('moveit_code')!=1 or
        not isinstance(goal.get('wall_ns'),int) or not isinstance(goal.get('terminal_wall_ns'),int) or
        type(goal.get('monotonic_raw_ns')) is not int or type(goal.get('terminal_monotonic_raw_ns')) is not int or
        goal['terminal_monotonic_raw_ns']<goal['monotonic_raw_ns'] or
        observed.get('run_id')!=retention.get('run_id') or
        observed.get('target')!=summary.get('selected_object_id') or
        observed.get('required_finger_contacts_absent') is not True or
        source.get('method')!=PHYSICS_POSE_METHOD or source.get('frame')!='world' or
        source.get('read_only') is not True or
        source.get('query_iteration')!=observed.get('iteration') or
        source.get('query_sim_ns')!=observed.get('sim_ns') or
        observed.get('relative_translation_m',0)<=.000001 or
        not isinstance(observed.get('wall_ns'),int) or
        type(observed.get('monotonic_raw_ns')) is not int or observed['monotonic_raw_ns']<=goal['monotonic_raw_ns'] or
        not isinstance(observed.get('sim_ns'),int) or
        evidence.get('transition_sim_ns',-1)<observed['sim_ns'] or
        evidence.get('sim_ns',-1)<evidence.get('transition_sim_ns',0) or
        evidence.get('state')!='RELEASE_CONFIRMED' or evidence.get('settled') is not True or
        detached.get('held') is not False or detached.get('attached_ids') or
        detached.get('acm_restored') is not True or
        detached.get('measured_geometry_matches') is not True):
        raise RuntimeError('physical release evidence is missing or bound to another attempt')


def assert_gate(gate,summary,capability_sha,overlay_identity,*,functional=False):
    from simulator_backend import moveit_overlay_identity
    expected=EXPECTED_RESULTS[gate]
    accepted=(expected,"FUNCTIONAL_PASS") if functional and gate=="full-cycle" else (expected,)
    if summary.get("result") not in accepted:raise RuntimeError(f"{gate} gate returned {summary.get('result')!r}, expected {expected!r}")
    if summary.get("full_cycle_prevalidated") is not True:raise RuntimeError(f"{gate} gate did not revalidate the complete cycle")
    if summary.get("commissioning_capability",{}).get("sha256")!=capability_sha:
        raise RuntimeError(f"{gate} used a different commissioning capability binary")
    if moveit_overlay_identity(summary.get("commissioning_capability",{}).get("moveit_overlay",{}))!=overlay_identity:
        raise RuntimeError(f"{gate} used a different MoveIt teardown dependency")
    if gate in ("stationary","contact-release","full-cycle"):
        retention=summary.get("stationary_retention",{})
        certificate=summary.get("pile_contact_certification",{})
        binding=retention.get("binding",{})
        begin=summary.get("stationary_hold_start",{})
        end=summary.get("stationary_hold_end",{})
        required=set(retention.get("required_contact_links",[]))
        if (not retention.get("run_id") or
            any(record.get("run_id")!=retention["run_id"] for record in (certificate,begin,end)) or
            retention.get("target")!=summary.get("selected_object_id") or
            certificate.get("target")!=retention.get("target") or
            not binding.get("close_goal_uuid") or binding!=certificate.get("binding") or
            binding.get("resolution_sha256")!=summary.get("resolution_sha256") or
            binding.get("close_goal_terminal_wall_ns")!=summary.get("close_terminal_wall_ns") or
            type(binding.get("close_goal_terminal_monotonic_raw_ns")) is not int or
            binding["close_goal_terminal_monotonic_raw_ns"]!=summary.get("close_terminal_monotonic_raw_ns") or
            retention.get("duration_sim_ns",0)<1_000_000_000 or
            retention.get("end_sim_ns",-1)-retention.get("start_sim_ns",0)!=retention.get("duration_sim_ns") or
            not retention.get("start_sim_ns",-1)<=begin.get("sim_ns",-2)<=retention.get("end_sim_ns",-1)<=end.get("sim_ns",-2) or
            len(required)<2 or not required.issubset(retention.get("contact_links",[]))):
            raise RuntimeError("current-grasp retention is missing, incomplete or bound to another attempt")
    if gate in ("contact-release","full-cycle"):
        assert_release_evidence(gate,summary,summary.get("stationary_retention",{}))
    if gate=="cancel":
        if not summary.get("cancellation_confirmed") or not summary.get("motion_stop_verified"):
            raise RuntimeError("fresh cancellation qualification is incomplete")
    elif gate=="telemetry":
        metrics=summary.get("motion_telemetry",{})
        if metrics.get("max_fresh_age_ms",1e9)>=250 or metrics.get("max_delivery_ms",1e9)>=250:
            raise RuntimeError("motion telemetry does not satisfy the unchanged 250 ms guard")
    elif gate=="contact-release":
        if summary.get("verified_lift_clearance_m",0)<.01 or not summary.get("release_evidence",{}).get("settled"):
            raise RuntimeError("contact-release did not prove lift and physical settling")
    elif gate=="full-cycle":
        final=summary.get("full_cycle_physical_acceptance",{})
        if not summary.get("full_cycle_execution_success") or not final.get("final_collision_valid") or final.get("attached_ids"):
            raise RuntimeError("full physical cycle final acceptance is incomplete")


# Functional validation classifies a failed cycle so safety-critical stops are never
# confused with task-quality deviations. Strict Stage-A acceptance is unaffected.
FAILURE_CATEGORIES=(
    ("safety_critical",("unpermitted physical contact","certificate","COLLISION","in collision","TELEMETRY","telemetry",
                        "CANCEL","cancellation","IDENTITY","identity","BINDING","OVERLAY","SUPPORT_CONTACT",
                        "JOINT_LIMIT","penetration","stale measurement")),
    ("task_quality",("retention lost: contact/slip","placement","containment","DESTINATION")),
    ("functional",("motion lacks","SEARCH_BUDGET","CANDIDATE","PREPLAN","IK","planning","did not separate",
                   "home mismatch","not open","did not settle","GRASP_HOLD","closure","dropped","missing_contacts")),
)


def classify_failure(text,summary=None):
    """safety_critical / functional / task_quality / unclassified for one failure text."""
    text=text or ""
    details=((summary or {}).get("grasp_recovery") or {}).get("details") or {}
    if "retention lost" in text and details.get("missing_contacts"):return "functional"      # a physical drop
    for category,markers in FAILURE_CATEGORIES:
        if any(marker in text for marker in markers):return category
    return "unclassified"


def functional_report(summary,failure=None):
    """Functional success, strict compliance, retention and placement, reported separately."""
    summary=summary or {}
    result=summary.get("result")
    retention=summary.get("retention_findings") or {}
    acceptance=summary.get("full_cycle_physical_acceptance") or {}
    # The executor's own failure text classifies better than the runner's command wrapper.
    cause=summary.get("failure") if isinstance(summary.get("failure"),str) and summary.get("failure") else failure
    return dict(functional_success=result in ("PASS","FUNCTIONAL_PASS"),
                strict_benchmark_compliant=result=="PASS",
                strict_retention_compliant=summary.get("strict_retention_compliant"),
                retention_peak=retention.get("peak"),retention_findings=retention.get("findings"),
                placement=acceptance.get("placement"),
                executed_stages=[r.get("stage") for r in summary.get("execution_results",[])],
                failed_stage=summary.get("failed_stage"),retention_loss=((summary.get("grasp_recovery") or {}).get("details")),
                failure=failure,failure_cause=cause,failure_category=classify_failure(cause,summary) if cause else None)


def prior_evidence(directory,capability_sha):
    """Passing prerequisite summaries of a completed run, for a diagnostic targeted full-cycle.

    The executor re-proves the capability and overlay binaries of every record; this
    only refuses an incomplete or failed run early. Reused evidence never makes the
    targeted session Stage-A acceptance.
    """
    directory=Path(directory)
    prior={}
    for gate in ("cancel","telemetry","stationary","contact-release"):
        path=directory/f"{GATES.index(gate)+1:02d}-{gate}"/gate/"summary.json"
        if not path.is_file():raise RuntimeError(f"prior evidence lacks the {gate} summary: {path}")
        summary=json.loads(path.read_text())
        if summary.get("result")!=EXPECTED_RESULTS[gate]:
            raise RuntimeError(f"prior {gate} evidence did not pass ({summary.get('result')!r})")
        if summary.get("commissioning_capability",{}).get("sha256")!=capability_sha:
            raise RuntimeError(f"prior {gate} evidence was produced by a different commissioning capability binary")
        prior[gate]=path
    return prior


def executor_command(repo,scene,receipt,observations,summary,planning_time,*,resolve=False,gate=None,evidence=None,search_timeout=300.,
                     retention_policy=None):
    cmd=[sys.executable,str(repo/"scripts/perceived_object_grasp_execute.py"),
         "--scene-package",str(scene),"--backend","simulator","--simulator-receipt",str(receipt),
         "--detections",str(observations),"--timeout",str(search_timeout),"--segment-planning-time",str(planning_time),
         "--summary-output",str(summary)]
    if resolve:cmd.append("--resolve-task")
    if gate is not None:
        cmd+=["--start","--simulator-commission",gate]
        if gate=="full-cycle":
            if evidence is None:raise ValueError("full-cycle gate requires prerequisite evidence")
            cmd+=["--commission-evidence",str(evidence)]
            if retention_policy is not None:cmd+=["--retention-policy",retention_policy]
    return cmd


def verify_session_moveit(domain,partition,expected_identity):
    from simulator_backend import live_moveit_overlay
    return live_moveit_overlay(domain,partition,expected_identity)


def verify_session_bridge(domain,partition,expected_identity):
    from simulator_backend import live_bridge_overlay
    return live_bridge_overlay(domain,partition,expected_identity)


def one_session(args,repo,source_world,capability_sha,gate,index,prior):
    session=args.output/f"{index:02d}-{gate}"
    if session.exists():raise RuntimeError(f"refusing to reuse evidence directory {session}")
    session.mkdir(parents=True)
    authored=getattr(args,"authored_cell",None)
    if authored:
        scene=prepare_authored_scene(repo,Path(authored["package"]),session/authored["name"])
        # Each authored object keeps its own class (an unclassified object never matches a class filter).
        classes=["--class-map",json.dumps({o["object_id"]:o.get("class_id") or "unclassified" for o in authored["world"]["objects"]})]
        perception=authored.get("perception")
        if perception is not None:
            # Stage-B detect: this session's own deterministic EPD replay of the cell's recording; classes come
            # only from it (bound to the measured objects by simulator_observations --perception).
            from perception_replay import PerceptionBlocked,validate_replay
            detected=session/"perception/perception_replay.yaml"
            stream(args,"PERCEIVING",gate=gate,motion=False)
            run([sys.executable,repo/"scripts/perception_replay.py","detect","--recording",perception["recording"],
                 "--output",detected,"--workspace",perception["workspace"],"--cell",authored["package"]],
                env=dict(os.environ),cwd=repo,log=session/"perception/detect.log",timeout=600)
            try:validate_replay(yaml.safe_load(detected.read_text()),authored["package"],
                                physics_world_sha256=authored["world"]["world_sha256"])
            except PerceptionBlocked as exc:raise RuntimeError(str(exc)) from exc
            classes=["--perception",str(detected),*(["--target-class",perception["target_class"]] if perception["target_class"] else [])]
    else:
        policy=getattr(args,"motion_policy_document",None)
        scene=(prepare_scene(repo,session/"ur5_2f_test",args.class_id) if policy is None else
               prepare_scene(repo,session/"ur5_2f_test",args.class_id,policy))
        classes=["--class-id",args.class_id]
    runtime=session/"runtime";receipt=runtime/"receipt.json";observations=session/"parts.yaml"
    domain=args.base_domain+index-1
    if not 1<=domain<=232:raise RuntimeError("ROS domain range exhausted")
    env=dict(os.environ,ROS_DOMAIN_ID=str(domain),IGN_PARTITION=f"{args.partition_prefix}_{gate}",
             ROS_LOCALHOST_ONLY="1",ROS2CLI_DISABLE_DAEMON="1",PYTHONUNBUFFERED="1")
    report={"gate":gate,"domain":domain,"partition":env["IGN_PARTITION"],"status":"BLOCKED",
            "mode":getattr(args,"mode","full"),
            "scope":RUN_SCOPES.get(getattr(args,"mode","full"),"DIAGNOSTIC_NOT_STAGE_A_ACCEPTANCE")}
    launch=None;launch_log=None
    try:
        assert_domain_free(env,repo)
        generate_scene(scene)
        launch_log=(session/"launch.log").open("w")
        # An authored cell runs this revision's qualified launch on its own generated files.
        launch_target=[str(repo/"scenes/ur5_2f_test/launch/demo.launch.py")] if authored else ["ur5_2f_test","demo.launch.py"]
        launch_cmd=["ros2","launch","--noninteractive",*launch_target,
            "execution_backend:=simulator","simulator_commissioning:=true",
            f"simulator_world:={source_world}",f"simulator_output:={runtime}",
            "use_sim_time:=true","launch_rviz:=false","use_fake_hardware:=true",
            "allow_trajectory_execution:=true",*([f"scene_package_dir:={scene}"] if authored else [])]
        launch=subprocess.Popen(launch_cmd,cwd=repo,env=env,stdout=launch_log,stderr=subprocess.STDOUT,start_new_session=True)
        stream(args,"SIMULATOR_LAUNCH",gate=gate)
        wait_file(receipt,launch,90,session/"launch.log")
        report["controllers_ready"]=wait_controllers_active(receipt,launch,env,repo,60)
        stream(args,"SIMULATOR_READY",gate=gate)
        run([sys.executable,repo/"scripts/simulator_observations.py","--receipt",receipt,
             "--output",observations,*classes],
            env=env,cwd=repo,log=session/"observations.log",timeout=90)

        def refresh_observations(label):
            run([sys.executable,repo/"scripts/simulator_observations.py","--receipt",receipt,
                 "--output",observations,*classes,
                 "--refresh-from",observations],
                env=env,cwd=repo,log=session/f"{label}-observations.log",timeout=90)

        report["moveit_overlay"]=verify_session_moveit(domain,env["IGN_PARTITION"],args.moveit_overlay_identity)
        report["bridge_overlay"]=verify_session_bridge(domain,env["IGN_PARTITION"],args.bridge_overlay_identity)
        resolve_summary=session/"resolve/summary.json"
        stream(args,"RESOLVE_TASK",gate=gate,motion=False)
        run(executor_command(repo,scene,receipt,observations,resolve_summary,args.segment_planning_time,
                resolve=True,search_timeout=args.search_timeout),
            env=env,cwd=repo,log=session/"resolve/executor.log",timeout=args.search_timeout+120+SERVICE_RETRY_ALLOWANCE_S)
        resolved=json.loads(resolve_summary.read_text());assert_plan(resolved,require_resolved=True)

        # The search may be long. Re-measure the same geometry and renew only
        # timestamps before consuming the saved resolution.
        refresh_observations("revalidate")
        generate_scene(scene)
        plan_summary=session/"revalidate/summary.json"
        stream(args,"REVALIDATE_PLAN",gate=gate,motion=False)
        run(executor_command(repo,scene,receipt,observations,plan_summary,args.segment_planning_time,
                search_timeout=args.search_timeout),
            env=env,cwd=repo,log=session/"revalidate/executor.log",timeout=args.search_timeout+120+SERVICE_RETRY_ALLOWANCE_S)
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
        stream(args,"EXECUTE_"+gate.upper().replace("-","_"),gate=gate,motion=True)
        functional=getattr(args,"mode","full")=="functional"
        run(executor_command(repo,scene,receipt,observations,gate_summary,args.segment_planning_time,
                gate=gate,evidence=evidence,search_timeout=args.search_timeout,
                retention_policy="record" if functional else None),
            env=env,cwd=repo,log=session/f"{gate}/executor.log",timeout=args.search_timeout+EXECUTION_GATE_MARGIN_S)
        result=json.loads(gate_summary.read_text());assert_gate(gate,result,capability_sha,args.moveit_overlay_identity,functional=functional)
        report.update(status="PASS",result=result["result"],selected_object_id=result.get("selected_object_id"),
                      selected_grasp_index=result.get("selected_grasp_index"))
        return gate_summary
    except Exception as exc:
        report["failure"]=str(exc);raise
    finally:
        stream(args,"SIMULATOR_SHUTDOWN",gate=gate)
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


def build_gil_probe(repo:Path,output:Path)->dict:
    """Compile the opt-in GIL probe for this interpreter into the evidence directory."""
    import sysconfig
    source=repo/"scripts/gil_probe.c";library=output/"gil_probe.so"
    command=["cc","-O2","-Wall","-Wextra","-Werror","-shared","-fPIC","-I"+sysconfig.get_paths()["include"],
             "-o",str(library),str(source)]
    run(command,env=dict(os.environ),cwd=repo,log=output/"gil-probe-build.log",timeout=120)
    compiler=subprocess.run(["cc","--version"],capture_output=True,text=True,timeout=30).stdout.splitlines()[:1]
    return {"source":str(source),"source_sha256":sha256(source),"library":str(library),
            "library_sha256":sha256(library),"command":command,"compiler":compiler[0] if compiler else None,
            "python":sys.version.split()[0]}


def run_fast_tests(repo:Path,output:Path,env:dict)->dict:
    """Run the focused Stage-A Python suites; a failure or a missing suite is FAILED."""
    missing=[name for name in FAST_TEST_FILES if not (repo/name).is_file()]
    if missing:raise RuntimeError("focused Stage-A test files missing: "+", ".join(missing))
    log=output/"fast-tests.log";started=time.monotonic()
    test_env={key:value for key,value in env.items() if key not in FAST_TEST_ENV_EXCLUDED}
    test_env.update(ROS_DOMAIN_ID="200",ROS_LOCALHOST_ONLY="1",ROS2CLI_DISABLE_DAEMON="1")
    completed=run([sys.executable,"-m","pytest","-q","-p","no:cacheprovider",*FAST_TEST_FILES],
        env=test_env,cwd=repo,log=log,timeout=FAST_TEST_TIMEOUT_S,check=False)
    lines=[line for line in log.read_text(errors="replace").splitlines() if line.strip()]
    # pytest's final summary ("N passed[, M failed] in S"); later output is ignored.
    summary=next((line for line in reversed(lines) if re.search(r"\b(passed|failed|errors?|no tests ran)\b.* in [\d.]+s",line)),"")
    counts={key:int(value) for value,key in re.findall(r"(\d+) (passed|failed|errors?|skipped)",summary)}
    result={"status":"PASS" if completed.returncode==0 and counts.get("passed",0)>0 and not counts.get("failed")
                and not counts.get("error") and not counts.get("errors") else "FAILED",
            "returncode":completed.returncode,"counts":counts,"files":list(FAST_TEST_FILES),
            "env_excluded":sorted(key for key in FAST_TEST_ENV_EXCLUDED if key in env),
            "log":str(log.relative_to(output)),"duration_s":round(time.monotonic()-started,1)}
    if result["status"]!="PASS":raise RuntimeError(f"focused Stage-A tests failed: {counts or lines[-1:] }; log={log}")
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-world",type=Path)
    parser.add_argument("--scene-package",type=Path,
        help="a Workcell Builder-saved cell (package directory); its saved task and derived physics world are used unchanged")
    parser.add_argument("--stream-status",action="store_true",
        help="print one JSON line per session stage and a final cycle_result for the Workcell Builder")
    parser.add_argument("--world-profile",choices=sorted(SOURCE_WORLDS),default="pile",
                        help="pinned physics world: dense pile benchmark (default) or sparse single-workpiece commissioning")
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--base-domain",type=int,default=201)
    parser.add_argument("--partition-prefix",default="stage_a1_finish")
    parser.add_argument("--class-id",default="part")
    parser.add_argument("--segment-planning-time",type=float,default=3.0)
    parser.add_argument("--motion-policy",type=Path,
        help="opt-in YAML motion policy (e.g. a transport_policy/v1 'transport' block) authored into the session's "
             "scene copy; validated before any session starts")
    parser.add_argument("--search-timeout",type=float,default=300.,
        help="Bounded candidate search time in seconds, including continuous collision certification")
    parser.add_argument("--through",choices=GATES,
        help="full mode only: last gate of the sequence (default full-cycle)")
    parser.add_argument("--mode",choices=MODES,default="full",
        help="full: six-gate acceptance; targeted: one gate in its own session; fast: no ROS launch; "
             "functional: one full cycle with recorded (never relaxed) retention limits, reported beside strict compliance")
    parser.add_argument("--gate",choices=GATES,help="targeted mode only: the single gate to run")
    parser.add_argument("--prior-evidence",type=Path,
        help="targeted --gate full-cycle only: completed run whose passing cancel, telemetry, stationary and "
             "contact-release summaries are reused for a diagnostic transfer trial (never Stage-A acceptance)")
    parser.add_argument("--gil-watchdog-ms",type=float,
        help="opt-in executor diagnostic: dump every thread's stack when one holds the GIL this long")
    parser.add_argument("--gil-probe-ms",type=float,
        help="opt-in executor diagnostic: native probe naming the thread that keeps a contended GIL this long")
    parser.add_argument("--gil-probe-native-pc",action="store_true",
        help="with --gil-probe-ms: also sample the holder's native PC (signal to the holder during a long hold only)")
    argv=list(sys.argv[1:] if argv is None else argv)
    args=parser.parse_args(argv)
    # A wrapper that drops --world-profile/--search-timeout silently gets the
    # pile benchmark and 300 s (LENLEG 2026-10-23); record what was asked for.
    unset=object()
    probe=copy.deepcopy(parser);probe.set_defaults(**{key:unset for key in INVOCATION_SETTINGS})
    explicit=vars(probe.parse_args(argv))
    invocation={"argv":argv,"defaulted":sorted(key for key in INVOCATION_SETTINGS if explicit[key] is unset),
                "effective":{key:(str(getattr(args,key)) if isinstance(getattr(args,key),Path) else getattr(args,key))
                             for key in INVOCATION_SETTINGS}}

    repo=Path(__file__).resolve().parents[1];workspace=repo.parent.parent
    args.output=args.output.resolve()
    if args.output.exists():parser.error("choose a new output directory")
    if not 0<args.segment_planning_time<=10:parser.error("segment planning time must be in (0,10]")
    args.motion_policy_document=None
    if args.motion_policy is not None:
        if args.scene_package is not None:
            parser.error("a saved cell owns its motion policy; edit it in the cell and Generate again instead of --motion-policy")
        sys.path.insert(0,str(repo/"scripts"))
        from transport_policy import TransportPolicyError,parse_transport_policy
        try:
            document=yaml.safe_load(args.motion_policy.read_text())
            if not isinstance(document,dict) or set(document)!={"transport"}:
                raise TransportPolicyError("a motion policy file holds exactly one 'transport' block")
            parse_transport_policy(document["transport"])
        except (OSError,yaml.YAMLError,TransportPolicyError) as exc:
            parser.error(f"invalid --motion-policy: {exc}")
        args.motion_policy_document=document
    if not math.isfinite(args.search_timeout) or args.search_timeout<=0:
        parser.error("search timeout must be finite and positive")
    if args.mode=="full":
        if args.gate is not None:parser.error("--gate selects a targeted run; use --mode targeted")
        if args.prior_evidence is not None:parser.error("--prior-evidence applies to a targeted full-cycle only")
        args.through=args.through or "full-cycle"
    else:
        if args.through is not None:parser.error("--through applies to the full sequence only")
        if args.mode=="fast" and args.gate is not None:parser.error("fast mode runs no gate")
        if args.mode=="targeted":
            if args.gate is None:parser.error("targeted mode requires --gate")
            if args.gate not in TARGETABLE_GATES and args.prior_evidence is None:
                parser.error("full-cycle consumes the cancel, telemetry, stationary and contact-release "
                             "evidence of the same run and cannot run alone; use --mode full, or reuse a completed "
                             "run's evidence with --prior-evidence for a diagnostic transfer trial")
        if args.mode=="functional":
            if args.gate is not None:parser.error("functional mode always runs the full cycle")
            if args.prior_evidence is None:
                parser.error("functional mode reuses a completed run's cancel, telemetry, stationary and contact-release "
                             "evidence; give --prior-evidence")
        elif args.prior_evidence is not None and (args.mode!="targeted" or args.gate!="full-cycle"):
            parser.error("--prior-evidence applies to a targeted full-cycle or functional run only")
    if args.gil_watchdog_ms is not None:
        if args.mode=="fast":parser.error("the GIL watchdog observes executor sessions; fast mode runs none")
        if args.mode!="targeted" or args.gate not in GIL_WATCHDOG_GATES:
            parser.error("the GIL watchdog is itself a GIL-hold source (Trial 26) and never runs during motion; "
                         "it may observe --mode targeted --gate resolve only")
        if not math.isfinite(args.gil_watchdog_ms) or not 50<=args.gil_watchdog_ms<=1000:
            parser.error("GIL watchdog threshold must be within [50, 1000] ms")
        os.environ[GIL_WATCHDOG_ENV]=str(args.gil_watchdog_ms)
    else:os.environ.pop(GIL_WATCHDOG_ENV,None)
    if args.gil_probe_ms is not None:
        if args.mode=="fast":parser.error("the GIL probe observes executor sessions; fast mode runs none")
        if not math.isfinite(args.gil_probe_ms) or not 20<=args.gil_probe_ms<=1000:
            parser.error("GIL probe threshold must be within [20, 1000] ms")
    elif args.gil_probe_native_pc:parser.error("--gil-probe-native-pc needs --gil-probe-ms")
    for key in (GIL_PROBE_LIB_ENV,GIL_PROBE_MS_ENV,GIL_PROBE_NATIVE_PC_ENV):os.environ.pop(key,None)
    args.output.mkdir(parents=True)
    started_wall_ns=time.time_ns();started=time.monotonic()
    overall={"status":"BLOCKED","mode":args.mode,"stage_a_acceptance":False,
             "scope":RUN_SCOPES[args.mode],
             "gates":{},"preflight":None,"started_wall_ns":started_wall_ns,
             "search_timeout_s":args.search_timeout,"segment_planning_time_s":args.segment_planning_time,
             "invocation":invocation}
    print("stage-a1 settings: "+", ".join(f"{key}={value}{' (default)' if key in invocation['defaulted'] else ''}"
                                          for key,value in invocation["effective"].items()),file=sys.stderr,flush=True)
    if args.mode=="full":overall["through"]=args.through
    if args.mode=="targeted":
        # Only a cancel gate in this run qualifies cancellation on the current binary.
        overall.update(gate=args.gate,cancellation_qualified_this_run=False)
    if args.gil_watchdog_ms is not None:overall["gil_watchdog_ms"]=args.gil_watchdog_ms
    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,cancel_on_signal)
    try:
        if args.gil_probe_ms is not None:
            overall["gil_probe"]=dict(build_gil_probe(repo,args.output),threshold_ms=args.gil_probe_ms,
                                      native_pc=args.gil_probe_native_pc)
            os.environ[GIL_PROBE_LIB_ENV]=overall["gil_probe"]["library"];os.environ[GIL_PROBE_MS_ENV]=str(args.gil_probe_ms)
            if args.gil_probe_native_pc:os.environ[GIL_PROBE_NATIVE_PC_ENV]="1"
        args.authored_cell=None
        if args.scene_package is not None:
            if args.source_world is not None:raise RuntimeError("--scene-package derives its world; do not pass --source-world")
            args.authored_cell=authored_cell(args.scene_package)
            from simulator_world import derive_world,engine_equivalent,TEMPLATE
            if git_blob(TEMPLATE,repo)!=SOURCE_WORLDS["sparse"][1]:raise RuntimeError("qualified world template changed")
            world_text,world_report=derive_world(args.authored_cell["package"])
            world_report["engine_equivalent_to_qualified_template"]=engine_equivalent(world_text)
            (args.output/"authored-world").mkdir()
            source_world=args.output/"authored-world/world.sdf";source_world.write_text(world_text)
            (args.output/"authored-world/derivation.json").write_text(json.dumps(world_report,indent=2)+"\n")
            args.authored_cell["world"]=world_report;overall["authored_cell"]=dict(args.authored_cell)
            if args.authored_cell["perception"] is not None:
                from perception_replay import PerceptionBlocked,validate_replay
                try:validate_replay(yaml.safe_load(Path(args.authored_cell["perception"]["replay"]).read_text()),
                                    args.authored_cell["package"],physics_world_sha256=world_report["world_sha256"])
                except PerceptionBlocked as exc:raise RuntimeError(str(exc)) from exc
                # Every session detects again with the registered EPD runtime: refuse now, not after the build.
                from live_perception_session import PerceptionUnavailable,load_runtime
                try:load_runtime(workspace)
                except PerceptionUnavailable as exc:raise RuntimeError(f"PERCEPTION_RUNTIME_UNAVAILABLE: {exc}") from exc
                args.authored_cell["perception"]["workspace"]=str(workspace)
        elif args.mode!="fast":source_world=discover_source_world(args.source_world,workspace,args.world_profile)
        launch_checkout_guard(repo)
        from simulator_backend import active_bridge_overlay,bridge_overlay_identity,moveit_overlay_identity
        bridge=active_bridge_overlay()
        args.bridge_overlay_identity=bridge_overlay_identity(bridge)
        build=build_commissioning(repo,workspace,args.output,dict(os.environ))
        args.moveit_overlay_identity=moveit_overlay_identity(build["moveit_overlay"])
        overall["preflight"]={"repo_head":subprocess.check_output(["git","rev-parse","HEAD"],cwd=repo,text=True).strip(),
            "repo_dirty":bool(subprocess.check_output(["git","status","--porcelain"],cwd=repo,text=True).strip()),
            "commissioning_build":build,"bridge_overlay":bridge,
            "duration_s":round(time.monotonic()-started,1)}
        if args.mode=="fast":
            overall["fast_tests"]=run_fast_tests(repo,args.output,dict(os.environ))
            overall["status"]="PASS";return 0
        if args.authored_cell:
            overall["preflight"].update(source_world=str(source_world),world_profile="authored",
                source_world_template_git_blob=SOURCE_WORLDS["sparse"][1],source_world_sha256=sha256(source_world))
        else:
            overall["preflight"].update(source_world=str(source_world),world_profile=args.world_profile,
                source_world_git_blob=SOURCE_WORLDS[args.world_profile][1],source_world_sha256=sha256(source_world))
        prior={}
        if args.prior_evidence is not None:
            prior=prior_evidence(args.prior_evidence,build["sha256"])
            overall["prior_evidence"]={gate:str(path) for gate,path in prior.items()}
        # A targeted gate keeps its full-run session index, so it gets the same
        # ROS domain and partition as in the sequence and its own fresh session.
        selected=[(GATES.index(args.gate)+1,args.gate)] if args.mode=="targeted" else \
            [(len(GATES),"full-cycle")] if args.mode=="functional" else \
            list(enumerate(GATES[:GATES.index(args.through)+1],1))
        last=selected[-1][1]
        for index,gate in selected:
            began=time.monotonic()
            try:
                path=one_session(args,repo,source_world,build["sha256"],gate,index,prior)
            except BaseException as exc:
                failed={"status":"FAILED" if isinstance(exc,Exception) else "INTERRUPTED",
                        "failure":str(exc) if isinstance(exc,Exception) else f"interrupted: {type(exc).__name__}",
                        "duration_s":round(time.monotonic()-began,1)}
                session_report=args.output/f"{index:02d}-{gate}"/"session-report.json"
                if session_report.is_file():
                    failed["session_report"]=str(session_report.relative_to(args.output))
                    recorded=json.loads(session_report.read_text())
                    for key in ("shutdown","shutdown_failure","moveit_overlay","bridge_overlay"):
                        if key in recorded:failed[key]=recorded[key]
                overall["gates"][gate]=failed
                if args.mode=="functional":
                    partial=args.output/f"{index:02d}-{gate}"/gate/"summary.json"
                    overall["functional"]=functional_report(json.loads(partial.read_text()) if partial.is_file() else None,
                                                            failure=failed["failure"])
                raise
            if args.mode=="targeted" and gate=="cancel":overall["cancellation_qualified_this_run"]=True
            if args.mode=="functional":overall["functional"]=functional_report(json.loads(path.read_text()))
            prior[gate]=path;overall["gates"][gate]={"status":"PASS","summary":str(path.relative_to(args.output)),
                                                     "duration_s":round(time.monotonic()-began,1),
                                                     "end_effector_hold":(json.loads(path.read_text()).get("end_effector_hold") or {}).get("profile")}
            overall["status"]="PASS" if gate==last else "IN_PROGRESS"
        # Stage-A acceptance is the reference cell's qualification; an authored cell reports its own cycle.
        overall["stage_a_acceptance"]=(not args.authored_cell and args.mode=="full" and args.through=="full-cycle"
                                       and overall["status"]=="PASS")
        return 0
    except Exception as exc:
        overall["status"]="BLOCKED";overall["failure"]=str(exc);return 1
    except OperatorCancel as exc:
        overall["status"]="CANCELLED";overall["failure"]=f"cancelled by the operator ({exc})";return 3
    except BaseException as exc:
        overall["status"]="BLOCKED";overall["failure"]=f"interrupted: {type(exc).__name__}";raise
    finally:
        overall["finished_wall_ns"]=time.time_ns();overall["duration_s"]=round(time.monotonic()-started,1)
        (args.output/REPORT_NAMES[args.mode]).write_text(json.dumps(overall,indent=2)+"\n")
        if getattr(args,"stream_status",False):
            print(json.dumps({"event":"cycle_result","summary":cycle_result(overall,args.output)}),flush=True)
        print(json.dumps(overall,indent=2))


if __name__=="__main__":raise SystemExit(main())
