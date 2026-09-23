import importlib.util
import json
from pathlib import Path

import pytest

SCRIPT=Path(__file__).parents[1]/"scripts/run_stage_a1_finish.py"
SPEC=importlib.util.spec_from_file_location("run_stage_a1_finish",SCRIPT)
MODULE=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(MODULE)


def test_gate_sequence_requalifies_cancellation_before_motion_acceptance():
    assert MODULE.GATES==("resolve","cancel","telemetry","stationary","contact-release","full-cycle")


def test_plan_gate_requires_complete_nine_stage_plan():
    summary={"result":"PLAN_ONLY","full_cycle_prevalidated":True,"execution_attempted":False,
        "task_intent_resolution":{"readiness_status":"READY"},
        "plan_metadata":[{"stage":stage,"success":True,"moveit_code":1,"points":2}
                         for stage in MODULE.STAGES]}
    MODULE.assert_plan(summary,require_resolved=True)
    bad=json.loads(json.dumps(summary));bad["plan_metadata"].pop()
    with pytest.raises(RuntimeError,match="nine-stage"):MODULE.assert_plan(bad,require_resolved=True)


@pytest.mark.parametrize("gate,result",[
    ("cancel","CANCELLATION_TRIAL_PASS"),
    ("telemetry","MOTION_TELEMETRY_PASS"),
    ("stationary","STATIONARY_RETENTION_PASS"),
    ("contact-release","CONTACT_RELEASE_PASS"),
    ("full-cycle","PASS"),
])
def test_gate_results_bind_to_same_current_capability(gate,result):
    sha="current-build"
    summary={"result":result,"full_cycle_prevalidated":True,
             "commissioning_capability":{"sha256":sha,"moveit_overlay":{"library":{"sha256":"patched-tem"},"move_group":{"executable":{"sha256":"patched-move-group"}}}}}
    if gate=="cancel":
        summary.update(cancellation_confirmed=True,motion_stop_verified=True)
    elif gate=="telemetry":
        summary["motion_telemetry"]={"max_fresh_age_ms":10.,"max_delivery_ms":9.}
    elif gate=="stationary":
        summary["stationary_retention"]={"duration_sim_ns":1_100_000_000,
            "required_contact_links":["left","right"],"contact_links":["left","right"]}
    elif gate=="contact-release":
        summary.update(verified_lift_clearance_m=.02,release_evidence={"settled":True})
    else:
        summary.update(full_cycle_execution_success=True,
            full_cycle_physical_acceptance={"final_collision_valid":True,"attached_ids":[]})
    MODULE.assert_gate(gate,summary,sha,("patched-tem","patched-move-group"))
    with pytest.raises(RuntimeError,match="different commissioning"):
        MODULE.assert_gate(gate,summary,"other-build",("patched-tem","patched-move-group"))
    with pytest.raises(RuntimeError,match="different MoveIt"):
        MODULE.assert_gate(gate,summary,sha,("stale-tem","patched-move-group"))
    with pytest.raises(RuntimeError,match="different MoveIt"):
        MODULE.assert_gate(gate,summary,sha,("patched-tem","stale-move-group"))
    del summary["commissioning_capability"]["moveit_overlay"]
    with pytest.raises(RuntimeError,match="different MoveIt"):
        MODULE.assert_gate(gate,summary,sha,("patched-tem","patched-move-group"))


def test_executor_full_cycle_requires_explicit_evidence(tmp_path):
    cmd=MODULE.executor_command(Path("/repo"),Path("/scene"),Path("/receipt"),
        Path("/parts"),Path("/summary"),3.0,gate="full-cycle",evidence=tmp_path/"evidence.json")
    assert "--start" in cmd
    assert cmd[cmd.index("--simulator-commission")+1]=="full-cycle"
    assert cmd[cmd.index("--commission-evidence")+1]==str(tmp_path/"evidence.json")


def test_source_world_discovery_accepts_only_frozen_git_blob(tmp_path,monkeypatch):
    world=tmp_path/"world.sdf";world.write_text("<sdf/>")
    repo=Path(__file__).parents[1]
    monkeypatch.setattr(MODULE,"SOURCE_WORLD_GIT_BLOB",MODULE.git_blob(world,repo))
    assert MODULE.discover_source_world(world,tmp_path)==world.resolve()
    world.write_text("<sdf><plugin name='workcell::SimulatorMeasurements'/></sdf>")
    monkeypatch.setattr(MODULE,"SOURCE_WORLD_GIT_BLOB",MODULE.git_blob(world,repo))
    with pytest.raises(RuntimeError,match="not found"):
        MODULE.discover_source_world(world,tmp_path)


def test_tracked_stage_a0_world_is_portable_and_has_ten_dynamic_parts():
    import xml.etree.ElementTree as ET
    world_path=Path(__file__).parents[1]/"scenes/ur5_2f_test/worlds/stage_a0.sdf"
    assert MODULE.git_blob(world_path,Path(__file__).parents[1])==MODULE.SOURCE_WORLD_GIT_BLOB
    root=ET.parse(world_path).getroot()
    world=root.find("world")
    assert world is not None and world.get("name")=="a0"
    dynamic=[m for m in world.findall("model") if m.findtext("static","false").lower()!="true"]
    assert [m.get("name") for m in dynamic]==[f"part_{i:02d}" for i in range(10)]
    for model in dynamic:
        collisions=model.findall("link/collision")
        assert len(collisions)==1
        assert collisions[0].find("geometry/box/size") is not None
        assert model.find("link/pose") is None
        assert collisions[0].find("pose") is None


def test_runner_failure_includes_log_tail(tmp_path):
    log=tmp_path/"failure.log"
    env=dict(__import__("os").environ)
    with pytest.raises(RuntimeError) as exc:
        MODULE.run(["bash","-lc","printf 'first\\nsecond\\n' && exit 7"],
            env=env,cwd=tmp_path,log=log,timeout=10)
    text=str(exc.value)
    assert "command failed (7)" in text
    assert "--- log tail ---" in text
    assert "second" in text


def test_launch_wait_failure_surfaces_launch_log_tail(tmp_path):
    log=tmp_path/"launch.log";log.write_text("alpha\nbeta\n")
    class Dead:
        returncode=9
        def poll(self): return 9
    with pytest.raises(RuntimeError) as exc:
        MODULE.wait_file(tmp_path/"receipt.json",Dead(),.1,log)
    text=str(exc.value)
    assert "rc=9" in text
    assert "beta" in text
    assert "launch log tail" in text


def test_commissioning_build_registers_measurement_library():
    script=(Path(__file__).parents[1]/"scripts/build_commissioning_capability.sh").read_text()
    assert "lib/libworkcell_simulator_measurements.so" in script
    assert "telemetry_sha256" in script


def test_wait_file_surfaces_structured_simulator_startup_failure(tmp_path):
    receipt=tmp_path/'runtime'/'receipt.json';receipt.parent.mkdir()
    (receipt.parent/'startup-failure.json').write_text(json.dumps(
        {'type':'RuntimeError','message':'robot spawn failed'}))
    class Alive:
        returncode=None
        def poll(self): return None
    with pytest.raises(RuntimeError,match='robot spawn failed'):
        MODULE.wait_file(receipt,Alive(),1.0)


def test_tracked_stage_a0_world_explicitly_loads_fortress_user_commands():
    import xml.etree.ElementTree as ET
    world_path=Path(__file__).parents[1]/"scenes/ur5_2f_test/worlds/stage_a0.sdf"
    root=ET.parse(world_path).getroot()
    plugins={(p.get("filename"),p.get("name")) for p in root.findall("world/plugin")}
    assert ("ignition-gazebo-physics-system","ignition::gazebo::systems::Physics") in plugins
    assert ("ignition-gazebo-user-commands-system","ignition::gazebo::systems::UserCommands") in plugins
    assert ("ignition-gazebo-scene-broadcaster-system","ignition::gazebo::systems::SceneBroadcaster") in plugins


def test_runner_refreshes_bound_physical_observation_before_revalidate_and_motion():
    source=SCRIPT.read_text()
    assert 'refresh_observations("revalidate")' in source
    assert 'refresh_observations(gate)' in source
    assert '"--refresh-from",observations' in source

@pytest.mark.parametrize('rc,clean', [(0, False),(-2, False),(-15, False),(-11, False),(7, False),(-9, False)])
def test_owned_shutdown_reports_abnormal_root_exit(tmp_path, rc, clean):
    import subprocess, sys
    code = 'import os, signal; ' + (f'os.kill(os.getpid(), {-rc})' if rc < 0 else f'raise SystemExit({rc})')
    process=subprocess.Popen([sys.executable,'-c',code],start_new_session=True)
    process.wait(timeout=5)
    report=MODULE.stop_owned(process)
    assert report['returncode']==rc
    assert report['clean'] is clean


def test_owned_shutdown_reports_child_segfault_even_when_launch_exits_zero(tmp_path):
    import subprocess, sys
    log=tmp_path/'launch.log'
    log.write_text("[INFO] [worker-1]: process started with pid [123]\n"
        "[ERROR] [worker-1]: process has died [pid 123, exit code -11, cmd 'worker'].\n")
    process=subprocess.Popen([sys.executable,'-c','pass'],start_new_session=True)
    process.wait(timeout=5)
    report=MODULE.stop_owned(process,log)
    assert report['clean'] is False
    assert report['children'][0]['signal']=='SIGSEGV'
    assert report['children'][0]['expected'] is False


@pytest.mark.parametrize('rc', [-2, -15])
def test_owned_shutdown_rejects_child_signals_before_cleanup(tmp_path,rc):
    import subprocess, sys
    log=tmp_path/'launch.log'
    log.write_text(f"[ERROR] [worker-1]: process has died [pid 123, exit code {rc}, cmd 'worker'].\n")
    process=subprocess.Popen([sys.executable,'-c','pass'],start_new_session=True)
    process.wait(timeout=5)
    assert MODULE.stop_owned(process,log)['clean'] is False


def test_main_failure_after_prior_gate_does_not_leave_in_progress(tmp_path,monkeypatch):
    monkeypatch.setattr(MODULE,'discover_source_world',lambda *args: SCRIPT)
    monkeypatch.setattr(MODULE,'build_commissioning',lambda *args: {'sha256':'test','moveit_overlay':{'library':{'sha256':'test-tem'}}})
    def session(args,repo,world,sha,gate,index,prior):
        if index==2:raise RuntimeError('owned shutdown failed: SIGSEGV')
        return args.output/'summary.json'
    monkeypatch.setattr(MODULE,'one_session',session)
    output=tmp_path/'evidence'
    assert MODULE.main(['--output',str(output),'--through','cancel'])==1
    assert json.loads((output/'stage-a1-final-report.json').read_text())['status']=='BLOCKED'


def test_session_rejects_passing_plan_when_owned_child_crashes(tmp_path,monkeypatch):
    from types import SimpleNamespace
    import subprocess,sys
    args=SimpleNamespace(output=tmp_path,base_domain=201,partition_prefix='test',
                         class_id='part',segment_planning_time=3.,moveit_overlay_identity=('test-tem','test-exe'))
    monkeypatch.setattr(MODULE,'prepare_scene',lambda repo,path,class_id:path)
    monkeypatch.setattr(MODULE,'assert_domain_free',lambda *args:None)
    monkeypatch.setattr(MODULE,'verify_session_moveit',lambda *args:{'library':{'sha256':'test-tem'}})
    monkeypatch.setattr(MODULE,'generate_scene',lambda *args:None)
    monkeypatch.setattr(MODULE,'wait_file',lambda *args:None)
    monkeypatch.setattr(MODULE,'assert_plan',lambda *args,**kwargs:None)
    real_popen=subprocess.Popen
    def launch(*args,**kwargs):
        process=real_popen([sys.executable,'-c',
            'print("[ERROR] [move_group-5]: process has died [pid 123, exit code -11, cmd \\\'move_group\\\'].")'],
            stdout=kwargs['stdout'],stderr=kwargs['stderr'],start_new_session=True)
        process.wait(timeout=5)
        return process
    monkeypatch.setattr(MODULE.subprocess,'Popen',launch)
    def run(command,**kwargs):
        if '--summary-output' in command:
            output=Path(command[command.index('--summary-output')+1]);output.parent.mkdir(parents=True)
            output.write_text('{}')
    monkeypatch.setattr(MODULE,'run',run)
    with pytest.raises(RuntimeError,match='owned shutdown failed'):
        MODULE.one_session(args,tmp_path,tmp_path/'world.sdf','test','resolve',1,{})
    report=json.loads((tmp_path/'01-resolve/session-report.json').read_text())
    assert report['status']=='BLOCKED'
    assert report['shutdown']['clean'] is False
    assert report['shutdown']['children'][0]['signal']=='SIGSEGV'


@pytest.mark.parametrize('name', ['move_group-5','python3-3','robot_state_publisher-1','parameter_bridge-4','static_transform_publisher-2'])
def test_long_lived_clean_exit_before_cleanup_is_abnormal(tmp_path,name):
    log=tmp_path/'launch.log'
    log.write_text(f"[INFO] [{name}]: process has finished cleanly [pid 123]\n")
    exits=MODULE.launch_child_exits(log)
    assert exits[0]['expected'] is False


@pytest.mark.parametrize('rc', [0,-2,-15,-11])
def test_child_exit_classification_requires_cleanup_and_matching_signal(tmp_path,rc):
    import signal
    log=tmp_path/'launch.log'
    prefix="[INFO] [spawner-6]: process has finished cleanly [pid 122]\n"
    log.write_text(prefix+"[INFO] [move_group-5]: sending signal 'SIGTERM' to process[move_group-5]\n"
        +f"[ERROR] [move_group-5]: process has died [pid 123, exit code {rc}, cmd 'move_group'].\n")
    exits=MODULE.launch_child_exits(log,cleanup_offset=len(prefix),sent_signals=[signal.SIGINT])
    assert exits[0]['expected'] is True
    assert exits[0]['before_cleanup'] is True
    assert exits[1]['expected'] is (rc in (0,-2,-15))
    assert exits[1]['before_cleanup'] is False


def test_unsent_child_signal_is_not_expected_after_cleanup(tmp_path):
    import signal
    log=tmp_path/'launch.log'
    log.write_text("[ERROR] [move_group-5]: process has died [pid 123, exit code -15, cmd 'move_group'].\n")
    assert MODULE.launch_child_exits(log,cleanup_offset=0,sent_signals=[signal.SIGINT])[0]['expected'] is False


@pytest.mark.parametrize('handler,rootcode', [('lambda *args:exit(0)',0),('signal.SIG_DFL',-2)])
def test_owned_signal_cleanup_accepts_running_launch_but_retains_crash_text(tmp_path,handler,rootcode):
    import subprocess,sys
    for crash in (False,True):
        log=tmp_path/f'launch-{crash}.log';ready=tmp_path/f'ready-{crash}'
        with log.open('w') as stream:
            code=f"import signal,time,pathlib; signal.signal(signal.SIGINT,{handler}); "
            if crash:code+="print('Segmentation fault (core dumped)',flush=True); "
            code+=f"pathlib.Path({str(ready)!r}).touch(); time.sleep(30)"
            process=subprocess.Popen([sys.executable,'-c',code],stdout=stream,start_new_session=True)
            MODULE.wait_file(ready,process,5)
            report=MODULE.stop_owned(process,log)
        assert report['clean'] is (not crash)
        assert bool(report['crash_evidence']) is crash
        assert report['root_alive_before_cleanup'] is True
        assert report['returncode']==rootcode


def test_final_report_includes_failed_gate_shutdown(tmp_path,monkeypatch):
    monkeypatch.setattr(MODULE,'discover_source_world',lambda *args: SCRIPT)
    monkeypatch.setattr(MODULE,'build_commissioning',lambda *args: {'sha256':'test','moveit_overlay':{'library':{'sha256':'test-tem'}}})
    shutdown={'clean':False,'children':[{'name':'move_group-5','returncode':-11}]}
    def session(args,repo,world,sha,gate,index,prior):
        report=args.output/f'{index:02d}-{gate}'/'session-report.json';report.parent.mkdir()
        report.write_text(json.dumps({'status':'BLOCKED','shutdown':shutdown,'shutdown_failure':'SIGSEGV'}))
        raise RuntimeError('owned shutdown failed: SIGSEGV')
    monkeypatch.setattr(MODULE,'one_session',session)
    output=tmp_path/'evidence'
    assert MODULE.main(['--output',str(output),'--through','resolve'])==1
    gate=json.loads((output/'stage-a1-final-report.json').read_text())['gates']['resolve']
    assert gate['status']=='FAILED'
    assert gate['shutdown']==shutdown
    assert gate['shutdown_failure']=='SIGSEGV'


@pytest.mark.parametrize('rc,expected', [(0,True),(1,False),(-2,False),(-15,False),(-11,False)])
def test_one_shot_scene_loader_only_accepts_successful_early_exit(tmp_path,rc,expected):
    log=tmp_path/'launch.log'
    log.write_text(f"[ERROR] [workcell_studio_planning_scene_node.py-6]: process has died [pid 20385, exit code {rc}, cmd 'loader'].\n")
    child=MODULE.launch_child_exits(log)[0]
    assert child['before_cleanup'] is True
    assert child['expected'] is expected


def test_real_resolve_log_accepts_completed_scene_loader_and_owned_cleanup():
    # Exact exit/signal excerpt from the fresh 20260923-101850 resolve session.
    # The loader applies/verifies the scene then exits; critical nodes stay up
    # until the recorded runner SIGINT boundary.
    import signal
    log=Path(__file__).parent/'fixtures/stage_a1_resolve_shutdown.log'
    cleanup_offset=log.read_bytes().index(b'[WARNING] [launch]: user interrupted')
    children=MODULE.launch_child_exits(log,cleanup_offset=cleanup_offset,sent_signals=[signal.SIGINT])
    assert len(children)==10
    assert all(child['expected'] for child in children)
    loader=next(child for child in children if child['name']=='workcell_studio_planning_scene_node.py-6')
    assert loader['before_cleanup'] is True
    assert loader['returncode']==0
    move_group=next(child for child in children if child['name']=='move_group-5')
    assert move_group['before_cleanup'] is False
    assert move_group['signal']=='SIGINT'
