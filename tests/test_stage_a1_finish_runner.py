import importlib.util
import json
from pathlib import Path

import pytest

SCRIPT=Path(__file__).parents[1]/"scripts/run_stage_a1_finish.py"
SPEC=importlib.util.spec_from_file_location("run_stage_a1_finish",SCRIPT)
MODULE=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(MODULE)


@pytest.fixture
def bridge_preflight(monkeypatch):
    import simulator_backend
    manifest={'executable':{'sha256':'test-bridge-exe'},'library':{'sha256':'test-bridge-lib'}}
    monkeypatch.setattr(simulator_backend,'active_bridge_overlay',lambda:manifest,raising=False)
    monkeypatch.setattr(simulator_backend,'bridge_overlay_identity',
        lambda data:(data['executable']['sha256'],data['library']['sha256']),raising=False)
    return manifest


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


def current_retention_summary():
    binding={"close_goal_uuid":"current-close","close_goal_terminal_wall_ns":100,
             "resolution_sha256":"resolution"}
    return dict(selected_object_id="runtime::part",resolution_sha256="resolution",
        close_terminal_wall_ns=100,
        pile_contact_certification={"run_id":"current-run","target":"runtime::part","binding":binding},
        stationary_hold_start={"run_id":"current-run","sim_ns":0},
        stationary_hold_end={"run_id":"current-run","sim_ns":1_100_000_000},
        stationary_retention={"run_id":"current-run","target":"runtime::part","binding":dict(binding),
            "duration_sim_ns":1_100_000_000,"start_sim_ns":0,"end_sim_ns":1_100_000_000,
            "required_contact_links":["left","right"],"contact_links":["left","right"]})


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
    if gate in ("stationary","contact-release","full-cycle"):
        summary.update(current_retention_summary())
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


def test_main_failure_after_prior_gate_does_not_leave_in_progress(tmp_path,monkeypatch,bridge_preflight):
    monkeypatch.setattr(MODULE,'discover_source_world',lambda *args: SCRIPT)
    monkeypatch.setattr(MODULE,'build_commissioning',lambda *args: {'sha256':'test','moveit_overlay':{'library':{'sha256':'test-tem'}}})
    def session(args,repo,world,sha,gate,index,prior):
        if index==2:raise RuntimeError('owned shutdown failed: SIGSEGV')
        return args.output/'summary.json'
    monkeypatch.setattr(MODULE,'one_session',session)
    output=tmp_path/'evidence'
    assert MODULE.main(['--output',str(output),'--through','cancel'])==1
    assert json.loads((output/'stage-a1-final-report.json').read_text())['status']=='BLOCKED'


def test_bridge_preflight_rejection_prevents_build_and_launch(tmp_path,monkeypatch,bridge_preflight):
    import simulator_backend
    monkeypatch.setattr(MODULE,'discover_source_world',lambda *args:SCRIPT)
    def reject():
        raise RuntimeError('BRIDGE_OVERLAY_REJECTED: library hash differs')
    monkeypatch.setattr(simulator_backend,'active_bridge_overlay',reject)
    def forbidden(*args):
        pytest.fail('unqualified bridge must block before build or launch')
    monkeypatch.setattr(MODULE,'build_commissioning',forbidden)
    monkeypatch.setattr(MODULE,'one_session',forbidden)
    output=tmp_path/'evidence'
    assert MODULE.main(['--output',str(output),'--through','resolve'])==1
    report=json.loads((output/'stage-a1-final-report.json').read_text())
    assert report['status']=='BLOCKED'
    assert report['failure']=='BRIDGE_OVERLAY_REJECTED: library hash differs'
    assert report['gates']=={}


def test_session_bridge_mismatch_blocks_resolve_and_records_cleanup(tmp_path,monkeypatch):
    from types import SimpleNamespace
    import simulator_backend
    args=SimpleNamespace(output=tmp_path,base_domain=201,partition_prefix='test',
        class_id='part',segment_planning_time=3.,moveit_overlay_identity=('tem','move-group'),
        bridge_overlay_identity=('qualified-exe','qualified-library'))
    monkeypatch.setattr(MODULE,'prepare_scene',lambda repo,path,class_id:path)
    monkeypatch.setattr(MODULE,'assert_domain_free',lambda *args:None)
    monkeypatch.setattr(MODULE,'generate_scene',lambda *args:None)
    monkeypatch.setattr(MODULE,'wait_file',lambda *args:None)
    monkeypatch.setattr(MODULE,'verify_session_moveit',lambda *args:{'verified':True})
    monkeypatch.setattr(MODULE.subprocess,'Popen',lambda *args,**kwargs:object())
    shutdown={'clean':True,'owned_cleanup':True}
    monkeypatch.setattr(MODULE,'stop_owned',lambda *args:shutdown)
    def bridge(domain,partition,expected_identity):
        assert (domain,partition,expected_identity)==(201,'test_resolve',('qualified-exe','qualified-library'))
        raise RuntimeError('BRIDGE_OVERLAY_REJECTED: dependency differs from runner preflight')
    monkeypatch.setattr(simulator_backend,'live_bridge_overlay',bridge,raising=False)
    def run(command,**kwargs):
        assert '--summary-output' not in command,'Resolve must not run after bridge mismatch'
    monkeypatch.setattr(MODULE,'run',run)
    with pytest.raises(RuntimeError,match='BRIDGE_OVERLAY_REJECTED'):
        MODULE.one_session(args,tmp_path,tmp_path/'world.sdf','test','resolve',1,{})
    report=json.loads((tmp_path/'01-resolve/session-report.json').read_text())
    assert report['status']=='BLOCKED'
    assert report['shutdown']==shutdown
    assert report['failure']=='BRIDGE_OVERLAY_REJECTED: dependency differs from runner preflight'


def test_session_rejects_passing_plan_when_owned_child_crashes(tmp_path,monkeypatch,bridge_preflight):
    from types import SimpleNamespace
    import subprocess,sys
    args=SimpleNamespace(output=tmp_path,base_domain=201,partition_prefix='test',
                         class_id='part',segment_planning_time=3.,moveit_overlay_identity=('test-tem','test-exe'),
                         bridge_overlay_identity=('test-bridge-exe','test-bridge-lib'))
    monkeypatch.setattr(MODULE,'prepare_scene',lambda repo,path,class_id:path)
    monkeypatch.setattr(MODULE,'assert_domain_free',lambda *args:None)
    monkeypatch.setattr(MODULE,'verify_session_moveit',lambda *args:{'library':{'sha256':'test-tem'}})
    monkeypatch.setattr(MODULE,'verify_session_bridge',lambda *args:bridge_preflight,raising=False)
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
    assert report['bridge_overlay']==bridge_preflight


@pytest.mark.parametrize('name', ['move_group-5','python3-3','robot_state_publisher-1','parameter_bridge-4','static_transform_publisher-2'])
def test_long_lived_clean_exit_before_cleanup_is_abnormal(tmp_path,name):
    log=tmp_path/'launch.log'
    log.write_text(f"[INFO] [{name}]: process has finished cleanly [pid 123]\n")
    exits=MODULE.launch_child_exits(log)
    assert exits[0]['expected'] is False


@pytest.mark.parametrize('rc', [0,-2,-15,-11,255])
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


def test_final_report_includes_failed_gate_shutdown(tmp_path,monkeypatch,bridge_preflight):
    monkeypatch.setattr(MODULE,'discover_source_world',lambda *args: SCRIPT)
    monkeypatch.setattr(MODULE,'build_commissioning',lambda *args: {'sha256':'test','moveit_overlay':{'library':{'sha256':'test-tem'}}})
    shutdown={'clean':False,'children':[{'name':'move_group-5','returncode':-11}]}
    def session(args,repo,world,sha,gate,index,prior):
        assert args.bridge_overlay_identity==('test-bridge-exe','test-bridge-lib')
        report=args.output/f'{index:02d}-{gate}'/'session-report.json';report.parent.mkdir()
        report.write_text(json.dumps({'status':'BLOCKED','shutdown':shutdown,'shutdown_failure':'SIGSEGV',
                                     'bridge_overlay':bridge_preflight}))
        raise RuntimeError('owned shutdown failed: SIGSEGV')
    monkeypatch.setattr(MODULE,'one_session',session)
    output=tmp_path/'evidence'
    assert MODULE.main(['--output',str(output),'--through','resolve'])==1
    final=json.loads((output/'stage-a1-final-report.json').read_text())
    assert final['preflight']['bridge_overlay']==bridge_preflight
    gate=final['gates']['resolve']
    assert gate['status']=='FAILED'
    assert gate['bridge_overlay']==bridge_preflight
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


def test_owned_launch_forwards_interrupt_once_to_child(tmp_path):
    import subprocess,sys
    child=tmp_path/'child.py';ready=tmp_path/'ready';child_ready=tmp_path/'child-ready';result=tmp_path/'signals'
    child.write_text('''import signal,time,pathlib,sys
hits=0;first=None
def stop(*args):
 global hits,first
 hits+=1
 if first is None:first=time.monotonic()
signal.signal(signal.SIGINT,stop)
pathlib.Path(sys.argv[1]).touch()
while first is None or time.monotonic()-first<.3:time.sleep(.005)
pathlib.Path(sys.argv[2]).write_text(str(hits))
''')
    launch=tmp_path/'launch.py'
    launch.write_text('''import signal,subprocess,pathlib,time,sys
child=subprocess.Popen([sys.executable,sys.argv[1],sys.argv[2],sys.argv[3]])
def stop(*args):child.send_signal(signal.SIGINT)
signal.signal(signal.SIGINT,stop)
while not pathlib.Path(sys.argv[2]).exists():time.sleep(.005)
pathlib.Path(sys.argv[4]).touch()
raise SystemExit(child.wait())
''')
    process=subprocess.Popen([sys.executable,str(launch),str(child),str(child_ready),str(result),str(ready)],start_new_session=True)
    MODULE.wait_file(ready,process,5)
    report=MODULE.stop_owned(process)
    assert report['clean'] is True
    assert report['remaining_owned_processes'] is False
    assert result.read_text()=='1'


@pytest.mark.parametrize('gate',['stationary','contact-release','full-cycle'])
@pytest.mark.parametrize('invalid',['missing','other_run','other_close','other_resolution','other_target','too_short','wrong_duration','missing_finger'])
def test_lifting_gates_reject_retention_from_another_attempt(gate,invalid):
    summary=current_retention_summary()
    summary.update(result=MODULE.EXPECTED_RESULTS[gate],full_cycle_prevalidated=True,
        commissioning_capability={'sha256':'build','moveit_overlay':{'library':{'sha256':'tem'},'move_group':{'executable':{'sha256':'group'}}}},
        verified_lift_clearance_m=.02,release_evidence={'settled':True},full_cycle_execution_success=True,
        full_cycle_physical_acceptance={'final_collision_valid':True,'attached_ids':[]})
    retention=summary['stationary_retention']
    if invalid=='missing':del summary['stationary_retention']
    elif invalid=='other_run':retention['run_id']='old-run'
    elif invalid=='other_close':retention['binding']['close_goal_uuid']='old-close'
    elif invalid=='other_resolution':retention['binding']['resolution_sha256']='old-resolution'
    elif invalid=='other_target':retention['target']='runtime::another'
    elif invalid=='too_short':retention['duration_sim_ns']=999_000_000
    elif invalid=='wrong_duration':retention['start_sim_ns']=1
    elif invalid=='missing_finger':retention['contact_links']=['left']
    with pytest.raises(RuntimeError,match='retention'):
        MODULE.assert_gate(gate,summary,'build',('tem','group'))
