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


@pytest.mark.parametrize('returncode,output,isolated',[
    (0,'',True),
    (0,'/foreign_node\n',False),
    (2,'discovery failed\n',False),
])
def test_domain_probe_never_spawns_daemon_and_keeps_isolation_guard(
        monkeypatch,tmp_path,returncode,output,isolated):
    env={'ROS_DOMAIN_ID':'226','ROS2CLI_DISABLE_DAEMON':'1'}
    calls=[]
    def probe(command,**kwargs):
        calls.append((command,kwargs))
        return MODULE.subprocess.CompletedProcess(command,returncode,stdout=output)
    monkeypatch.setattr(MODULE.subprocess,'run',probe)
    if isolated:
        MODULE.assert_domain_free(env,tmp_path)
    else:
        with pytest.raises(RuntimeError,match='ROS domain 226 is not isolated'):
            MODULE.assert_domain_free(env,tmp_path)
    command,kwargs=calls.pop()
    assert calls==[]
    assert command==['ros2','node','list','--no-daemon']
    assert kwargs['env'] is env and kwargs['cwd']==tmp_path
    assert kwargs['timeout']==10


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
             "commissioning_capability":{"sha256":sha,"moveit_overlay":{"library":{"sha256":"patched-tem"},"move_group":{"executable":{"sha256":"patched-move-group"},"capabilities":{"sha256":"patched-capabilities"}}}}}
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
    if gate in ("contact-release","full-cycle"):
        summary.update(current_release_summary(gate))
    elif gate=="stationary":
        summary.update(current_retention_summary())
    MODULE.assert_gate(gate,summary,sha,("patched-tem","patched-move-group","patched-capabilities"))
    with pytest.raises(RuntimeError,match="different commissioning"):
        MODULE.assert_gate(gate,summary,"other-build",("patched-tem","patched-move-group","patched-capabilities"))
    with pytest.raises(RuntimeError,match="different MoveIt"):
        MODULE.assert_gate(gate,summary,sha,("stale-tem","patched-move-group"))
    with pytest.raises(RuntimeError,match="different MoveIt"):
        MODULE.assert_gate(gate,summary,sha,("patched-tem","stale-move-group"))
    del summary["commissioning_capability"]["moveit_overlay"]
    with pytest.raises(RuntimeError,match="different MoveIt"):
        MODULE.assert_gate(gate,summary,sha,("patched-tem","patched-move-group","patched-capabilities"))


def test_executor_full_cycle_requires_explicit_evidence(tmp_path):
    cmd=MODULE.executor_command(Path("/repo"),Path("/scene"),Path("/receipt"),
        Path("/parts"),Path("/summary"),3.0,gate="full-cycle",evidence=tmp_path/"evidence.json")
    assert "--start" in cmd
    assert cmd[cmd.index("--simulator-commission")+1]=="full-cycle"
    assert cmd[cmd.index("--commission-evidence")+1]==str(tmp_path/"evidence.json")


def test_executor_search_budget_is_explicit_and_keeps_default():
    paths=[Path('/repo'),Path('/scene'),Path('/receipt'),Path('/parts'),Path('/summary')]
    default=MODULE.executor_command(*paths,3.,resolve=True)
    extended=MODULE.executor_command(*paths,3.,resolve=True,search_timeout=900.)
    assert float(default[default.index('--timeout')+1])==300.
    assert float(extended[extended.index('--timeout')+1])==900.
    assert extended[extended.index('--segment-planning-time')+1]=='3.0'
    assert '--start' not in extended


@pytest.mark.parametrize('budget',['0','-1','nan','inf'])
def test_runner_rejects_unbounded_or_nonpositive_search_budget(tmp_path,budget):
    output=tmp_path/'evidence'
    with pytest.raises(SystemExit) as exc:
        MODULE.main(['--output',str(output),'--search-timeout',budget])
    assert exc.value.code==2
    assert not output.exists()


def test_source_world_discovery_accepts_only_frozen_git_blob(tmp_path,monkeypatch):
    world=tmp_path/"world.sdf";world.write_text("<sdf/>")
    repo=Path(__file__).parents[1]
    monkeypatch.setattr(MODULE,"SOURCE_WORLD_GIT_BLOB",MODULE.git_blob(world,repo))
    assert MODULE.discover_source_world(world,tmp_path)==world.resolve()
    world.write_text("<sdf><plugin name='workcell::SimulatorMeasurements'/></sdf>")
    monkeypatch.setattr(MODULE,"SOURCE_WORLD_GIT_BLOB",MODULE.git_blob(world,repo))
    with pytest.raises(RuntimeError,match="not found"):
        MODULE.discover_source_world(world,tmp_path)


def test_sparse_commissioning_world_is_pinned_and_keeps_one_unchanged_pile_part():
    import xml.etree.ElementTree as ET
    repo=Path(__file__).parents[1]
    worlds=repo/"scenes/ur5_2f_test/worlds"
    name,blob=MODULE.SOURCE_WORLDS["sparse"]
    assert MODULE.git_blob(worlds/name,repo)==blob
    assert MODULE.discover_source_world(None,repo,"sparse")==(worlds/name).resolve()
    sparse=ET.parse(worlds/name).getroot().find("world")
    pile=ET.parse(worlds/"stage_a0.sdf").getroot().find("world")
    dynamic=[m for m in sparse.findall("model") if m.findtext("static","false").lower()!="true"]
    assert [m.get("name") for m in dynamic]==["part_04"]
    def text(element):
        element.tail=None
        return ET.tostring(element)
    original=[m for m in pile.findall("model") if m.get("name")=="part_04"][0]
    assert text(dynamic[0])==text(original)
    static=lambda w:[text(m) for m in w.findall("model") if m.findtext("static","false").lower()=="true"]
    assert static(sparse)==static(pile)
    assert text(sparse.find("physics"))==text(pile.find("physics"))


def test_world_profile_default_remains_pile_benchmark():
    assert MODULE.SOURCE_WORLDS["pile"]==("stage_a0.sdf",MODULE.SOURCE_WORLD_GIT_BLOB)


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
        class_id='part',segment_planning_time=3.,search_timeout=300.,moveit_overlay_identity=('tem','move-group'),
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
                         class_id='part',segment_planning_time=3.,search_timeout=300.,moveit_overlay_identity=('test-tem','test-exe'),
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
        commissioning_capability={'sha256':'build','moveit_overlay':{'library':{'sha256':'tem'},'move_group':{'executable':{'sha256':'group'},'capabilities':{'sha256':'capabilities'}}}},
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
        MODULE.assert_gate(gate,summary,'build',('tem','group','capabilities'))


def current_release_summary(gate):
    retained=current_retention_summary()
    goal=dict(uuid='a'*32,accepted=True,
        stage='COMMISSION_RELEASE' if gate=='contact-release' else 'EXECUTE_OPEN_GRIPPER',
        wall_ns=200,terminal_wall_ns=500,terminal_status=4,moveit_code=1,
        run_id='current-run',target='runtime::part',resolution_sha256='resolution',
        execution_attempt='attempt',selected_grasp_index=1)
    retained['pile_contact_certification']['binding']['execution_attempt']='attempt'
    retained['pile_contact_certification']['binding']['selected_grasp_index']=1
    retained['stationary_retention']['binding']['execution_attempt']='attempt'
    retained['stationary_retention']['binding']['selected_grasp_index']=1
    retained['selected_grasp_index']=1
    retained['owned_open_goal']=dict(goal)
    retained['release_evidence']=dict(run_id='current-run',target='runtime::part',
        binding=dict(retained['stationary_retention']['binding']),goal=dict(goal),
        separation=dict(run_id='current-run',target='runtime::part',iteration=10,sim_ns=10,
                        wall_ns=300,gripper_position_rad=.2,relative_translation_m=.0001,
                        required_finger_contacts_absent=True,
                        pose_source=dict(method='physics_link_frame_data_at_offset',frame='world',
                            read_only=True,query_iteration=10,query_sim_ns=10)),
        detachment=dict(held=False,attached_ids=[],acm_restored=True,measured_geometry_matches=True),
        transition_iteration=11,transition_sim_ns=11,state='RELEASE_CONFIRMED',
        settled=True,sim_ns=1_000_000_011)
    return retained


@pytest.mark.parametrize('gate,result', [('contact-release','CONTACT_RELEASE_PASS'),('full-cycle','PASS')])
@pytest.mark.parametrize('bad',['missing','other_run','other_target','other_goal','other_resolution',
                                 'other_attempt','wrong_stage','no_terminal','no_separation','stale_separation',
                                 'attachment','not_settled','missing_source'])
def test_gate_rejects_release_evidence_not_owned_by_current_attempt(gate,result,bad):
    summary=dict(result=result,full_cycle_prevalidated=True,
        commissioning_capability={'sha256':'build','moveit_overlay':{
            'library':{'sha256':'tem'},'move_group':{'executable':{'sha256':'group'},'capabilities':{'sha256':'capabilities'}}}},
        verified_lift_clearance_m=.02,full_cycle_execution_success=True,
        full_cycle_physical_acceptance={'final_collision_valid':True,'attached_ids':[]})
    summary.update(current_release_summary(gate))
    evidence=summary['release_evidence']
    if bad=='missing':summary.pop('release_evidence')
    elif bad=='other_run':evidence['run_id']='other-run'
    elif bad=='other_target':evidence['target']='runtime::other'
    elif bad=='other_goal':evidence['goal']['uuid']='b'*32
    elif bad=='other_resolution':evidence['binding']['resolution_sha256']='other'
    elif bad=='other_attempt':evidence['binding']['execution_attempt']='other'
    elif bad=='wrong_stage':evidence['goal']['stage']='EXECUTE_TRANSFER'
    elif bad=='no_terminal':evidence['goal']['terminal_status']=5
    elif bad=='no_separation':evidence['separation']['required_finger_contacts_absent']=False
    elif bad=='stale_separation':evidence['separation']['wall_ns']=100
    elif bad=='attachment':evidence['detachment']['attached_ids']=['runtime::part']
    elif bad=='missing_source':evidence['separation'].pop('pose_source')
    else:evidence['settled']=False
    with pytest.raises(RuntimeError,match='release'):
        MODULE.assert_gate(gate,summary,'build',('tem','group','capabilities'))


def test_cleanup_rejects_launch_escalation_even_when_child_exits_zero(tmp_path,monkeypatch):
    """Actual parent-GDB diagnostic log escaped the forced-cleanup gate."""
    import subprocess,sys
    ready=tmp_path/'ready';log=tmp_path/'launch.log'
    forced="[ERROR] [gdb-2]: process[gdb-2] failed to terminate '5' seconds after receiving 'SIGINT', escalating to 'SIGTERM'"
    code=f"""import signal,time,pathlib
def stop(*args):
 print({forced!r},flush=True)
 print('[INFO] [gdb-2]: process has finished cleanly [pid 456]',flush=True)
 exit(0)
signal.signal(signal.SIGINT,stop)
pathlib.Path({str(ready)!r}).touch()
time.sleep(30)
"""
    with log.open('w') as stream:
        process=subprocess.Popen([sys.executable,'-c',code],stdout=stream,start_new_session=True)
        MODULE.wait_file(ready,process,5);report=MODULE.stop_owned(process,log)
    assert report['returncode']==0 and report['remaining_owned_processes'] is False
    assert report['ownership_errors']==[]
    assert report['clean'] is False
    assert report['launch_escalation_evidence']==[forced]


def test_cleanup_rejects_runner_sigterm_escalation(tmp_path,monkeypatch):
    import subprocess,sys,time
    ready=tmp_path/'ready'
    code=f"import signal,time,pathlib; signal.signal(signal.SIGINT,signal.SIG_IGN); signal.signal(signal.SIGTERM,lambda *a:exit(0)); pathlib.Path({str(ready)!r}).touch(); time.sleep(30)"
    process=subprocess.Popen([sys.executable,'-c',code],start_new_session=True)
    MODULE.wait_file(ready,process,5);real_clock=time.monotonic
    monkeypatch.setattr(MODULE.time,'monotonic',lambda:real_clock()*100.)
    report=MODULE.stop_owned(process)
    assert report['returncode']==0 and report['escalation']=='SIGTERM'
    assert report['remaining_owned_processes'] is False
    assert report['ownership_errors']==[] and report['clean'] is False


def test_missing_supervisor_is_not_recorded_as_signalled(monkeypatch):
    from types import SimpleNamespace
    process=SimpleNamespace(pid=999999999,poll=lambda:None,wait=lambda timeout:0)
    report=MODULE.stop_owned(process)
    assert report['sent_signals']==[] and report['clean'] is False


@pytest.mark.parametrize('forward', [False,True])
def test_cleanup_tracks_owned_child_in_another_process_group(tmp_path,monkeypatch,forward):
    """A simulator child may outlive its supervisor in a separate group."""
    import os,signal,subprocess,sys,time
    ready=tmp_path/'ready';child_ready=tmp_path/'child-ready';log=tmp_path/'launch.log'
    child=f"import signal,time,pathlib; signal.signal(signal.SIGINT,lambda *a:exit(0)); pathlib.Path({str(child_ready)!r}).touch(); time.sleep(30)"
    parent=f"""import signal,subprocess,sys,time,pathlib
child=subprocess.Popen([sys.executable,'-c',{child!r}],start_new_session=True)
def stop(*args):
 if {forward!r}:child.send_signal(signal.SIGINT)
 exit(0)
signal.signal(signal.SIGINT,stop)
pathlib.Path({str(ready)!r}).write_text(str(child.pid))
time.sleep(30)
"""
    with log.open('w') as stream:
        process=subprocess.Popen([sys.executable,'-c',parent],stdout=stream,stderr=stream,start_new_session=True)
        MODULE.wait_file(ready,process,5);MODULE.wait_file(child_ready,process,5)
        child_pid=int(ready.read_text());child_fd=os.pidfd_open(child_pid);real_clock=time.monotonic
        monkeypatch.setattr(MODULE.time,'monotonic',lambda:real_clock()*100.)
        try:
            report=MODULE.stop_owned(process,log)
            assert report['clean'] is forward
            assert report['remaining_owned_processes'] is False
            assert child_pid in [p['pid'] for p in report['owned_processes_before_cleanup']]
        finally:
            try:signal.pidfd_send_signal(child_fd,signal.SIGINT)
            except ProcessLookupError:pass
            finally:os.close(child_fd)
            if process.poll() is None:process.send_signal(signal.SIGINT)
            process.wait(timeout=3)


def test_execution_gate_timeout_outlasts_the_executors_maximal_owned_waits():
    import importlib.util
    spec=importlib.util.spec_from_file_location("executor_under_test",SCRIPT.parent/"perceived_object_grasp_execute.py")
    executor=importlib.util.module_from_spec(spec);spec.loader.exec_module(executor)
    source=SCRIPT.read_text()
    assert "timeout=args.search_timeout+EXECUTION_GATE_MARGIN_S" in source
    margin=int(source.split("EXECUTION_GATE_MARGIN_S=")[1].split()[0])
    # The runner's last-resort kill must come after the executor's own owned cancellation.
    assert margin>=executor.MAX_OWNED_EXECUTION_S
    # Resolve/revalidate sessions carry the same first-reply service-retry allowance as the executor.
    allowance=int(source.split("SERVICE_RETRY_ALLOWANCE_S=")[1].split()[0])
    assert allowance==executor.SERVICE_RETRY_ALLOWANCE_S
    assert source.count("timeout=args.search_timeout+120+SERVICE_RETRY_ALLOWANCE_S)")==2


# --- execution modes: full, targeted, fast -----------------------------------------------------

@pytest.fixture
def stubbed_preflight(monkeypatch,bridge_preflight):
    monkeypatch.setattr(MODULE,'discover_source_world',lambda *args: SCRIPT)
    calls=[]
    def build(*args):
        calls.append('build');return {'sha256':'test','moveit_overlay':{'library':{'sha256':'test-tem'}}}
    monkeypatch.setattr(MODULE,'build_commissioning',build)
    import simulator_backend
    monkeypatch.setattr(simulator_backend,'moveit_overlay_identity',lambda data:data,raising=False)
    return calls


def record_sessions(monkeypatch,fail_at=None,interrupt_at=None):
    seen=[]
    def session(args,repo,world,sha,gate,index,prior):
        seen.append((gate,index,dict(prior)))
        directory=args.output/f'{index:02d}-{gate}';directory.mkdir()
        (directory/'session-report.json').write_text(json.dumps({'gate':gate,'status':'BLOCKED','shutdown':{'clean':True}}))
        if gate==fail_at:raise RuntimeError(f'{gate} gate returned FAIL')
        if gate==interrupt_at:raise KeyboardInterrupt()
        path=directory/'summary.json';path.write_text('{}');return path
    monkeypatch.setattr(MODULE,'one_session',session)
    return seen


@pytest.mark.parametrize('gate',['resolve','cancel','telemetry','stationary','contact-release'])
def test_targeted_mode_runs_exactly_the_selected_gate_in_its_own_session(tmp_path,monkeypatch,stubbed_preflight,gate):
    seen=record_sessions(monkeypatch)
    output=tmp_path/'evidence'
    assert MODULE.main(['--output',str(output),'--mode','targeted','--gate',gate])==0
    index=MODULE.GATES.index(gate)+1
    assert seen==[(gate,index,{})]                       # same session index (domain, partition) as in the sequence
    assert stubbed_preflight==['build']                  # the same preflight build and C++ tests run first
    report=json.loads((output/'stage-a1-targeted-report.json').read_text())
    assert report['status']=='PASS' and report['mode']=='targeted' and report['gate']==gate
    assert report['stage_a_acceptance'] is False and report['scope']=='DIAGNOSTIC_NOT_STAGE_A_ACCEPTANCE'
    assert report['cancellation_qualified_this_run'] is (gate=='cancel')
    assert list(report['gates'])==[gate] and report['gates'][gate]['status']=='PASS'
    assert not (output/'stage-a1-final-report.json').exists()      # never written where acceptance is read
    assert report['duration_s']>=0 and report['gates'][gate]['duration_s']>=0


def test_targeted_full_cycle_is_refused_because_it_needs_same_run_prerequisite_evidence(tmp_path,monkeypatch,stubbed_preflight,capsys):
    seen=record_sessions(monkeypatch)
    with pytest.raises(SystemExit):
        MODULE.main(['--output',str(tmp_path/'evidence'),'--mode','targeted','--gate','full-cycle'])
    assert 'cannot run alone' in capsys.readouterr().err
    assert seen==[] and stubbed_preflight==[] and not (tmp_path/'evidence').exists()


@pytest.mark.parametrize('argv,message',[
    (['--mode','targeted'],'targeted mode requires --gate'),
    (['--gate','telemetry'],'use --mode targeted'),
    (['--mode','targeted','--gate','telemetry','--through','telemetry'],'full sequence only'),
    (['--mode','fast','--gate','telemetry'],'fast mode runs no gate'),
    (['--mode','fast','--through','resolve'],'full sequence only'),
    (['--mode','fast','--gil-watchdog-ms','100'],'fast mode runs none'),
    (['--mode','targeted','--gate','resolve','--gil-watchdog-ms','10'],'within [50, 1000] ms'),
    # Trial 26: the watchdog itself held the GIL for 400 ms; it never observes a gate that moves the robot
    *[(['--mode','targeted','--gate',gate,'--gil-watchdog-ms','100'],'never runs during motion')
      for gate in ('cancel','telemetry','stationary','contact-release')],
    (['--gil-watchdog-ms','100'],'never runs during motion'),
    (['--mode','targeted','--gate','resolve','--gil-probe-native-pc'],'needs --gil-probe-ms'),
])
def test_mode_arguments_that_could_blur_scope_are_rejected_before_anything_runs(tmp_path,monkeypatch,stubbed_preflight,capsys,argv,message):
    seen=record_sessions(monkeypatch)
    with pytest.raises(SystemExit):
        MODULE.main(['--output',str(tmp_path/'evidence'),*argv])
    assert message in capsys.readouterr().err
    assert seen==[] and stubbed_preflight==[] and not (tmp_path/'evidence').exists()


def test_targeted_gate_failure_is_reported_failed_and_blocked_with_its_session_shutdown(tmp_path,monkeypatch,stubbed_preflight):
    record_sessions(monkeypatch,fail_at='stationary')
    output=tmp_path/'evidence'
    assert MODULE.main(['--output',str(output),'--mode','targeted','--gate','stationary'])==1
    report=json.loads((output/'stage-a1-targeted-report.json').read_text())
    assert report['status']=='BLOCKED' and report['stage_a_acceptance'] is False
    gate=report['gates']['stationary']
    assert gate['status']=='FAILED' and 'stationary gate returned FAIL' in gate['failure']
    assert gate['session_report']=='04-stationary/session-report.json' and gate['shutdown']=={'clean':True}


def test_failed_preflight_blocks_targeted_run_before_any_session(tmp_path,monkeypatch,bridge_preflight):
    monkeypatch.setattr(MODULE,'discover_source_world',lambda *args: SCRIPT)
    def build(*args):raise RuntimeError('commissioning library does not match its build manifest')
    monkeypatch.setattr(MODULE,'build_commissioning',build)
    seen=record_sessions(monkeypatch)
    output=tmp_path/'evidence'
    assert MODULE.main(['--output',str(output),'--mode','targeted','--gate','telemetry'])==1
    report=json.loads((output/'stage-a1-targeted-report.json').read_text())
    assert report['status']=='BLOCKED' and report['gates']=={} and seen==[]
    assert report['failure']=='commissioning library does not match its build manifest'


def test_operator_interrupt_records_the_gate_and_blocked_report_then_propagates(tmp_path,monkeypatch,stubbed_preflight):
    record_sessions(monkeypatch,interrupt_at='telemetry')
    output=tmp_path/'evidence'
    with pytest.raises(KeyboardInterrupt):
        MODULE.main(['--output',str(output),'--mode','targeted','--gate','telemetry'])
    report=json.loads((output/'stage-a1-targeted-report.json').read_text())
    assert report['status']=='BLOCKED' and report['failure']=='interrupted: KeyboardInterrupt'
    assert report['gates']['telemetry']['status']=='INTERRUPTED'
    assert report['gates']['telemetry']['failure']=='interrupted: KeyboardInterrupt'


def test_full_mode_keeps_the_cumulative_sequence_and_only_a_complete_pass_is_acceptance(tmp_path,monkeypatch,stubbed_preflight):
    seen=record_sessions(monkeypatch)
    output=tmp_path/'full'
    assert MODULE.main(['--output',str(output)])==0
    assert [(gate,index) for gate,index,_ in seen]==list(zip(MODULE.GATES,range(1,7)))
    assert set(seen[-1][2])=={'resolve','cancel','telemetry','stationary','contact-release'}   # prerequisites flow to full-cycle
    report=json.loads((output/'stage-a1-final-report.json').read_text())
    assert report['status']=='PASS' and report['mode']=='full' and report['through']=='full-cycle'
    assert report['stage_a_acceptance'] is True and report['scope']=='STAGE_A_ACCEPTANCE_SEQUENCE'
    partial=tmp_path/'partial';seen.clear()
    assert MODULE.main(['--output',str(partial),'--through','stationary'])==0
    report=json.loads((partial/'stage-a1-final-report.json').read_text())
    assert report['status']=='PASS' and report['stage_a_acceptance'] is False   # a passing prefix is not acceptance
    failed=tmp_path/'failed';record_sessions(monkeypatch,fail_at='full-cycle')
    assert MODULE.main(['--output',str(failed)])==1
    assert json.loads((failed/'stage-a1-final-report.json').read_text())['stage_a_acceptance'] is False


def test_report_records_argv_and_which_run_settings_fell_back_to_defaults(tmp_path,monkeypatch,stubbed_preflight,capsys):
    # LENLEG 2026-10-23: a wrapper dropped --world-profile sparse --search-timeout 900
    # and the evidence did not say the pile/300 s run came from defaults.
    record_sessions(monkeypatch,fail_at='resolve')
    output=tmp_path/'defaulted'
    assert MODULE.main(['--output',str(output),'--mode','full','--through','full-cycle'])==1
    report=json.loads((output/'stage-a1-final-report.json').read_text())
    assert report['invocation']['argv']==['--output',str(output),'--mode','full','--through','full-cycle']
    assert report['invocation']['defaulted']==['search_timeout','segment_planning_time','world_profile']
    assert report['invocation']['effective']=={'world_profile':'pile','search_timeout':300.,'segment_planning_time':3.}
    assert 'world_profile=pile (default)' in capsys.readouterr().err
    explicit=tmp_path/'explicit';record_sessions(monkeypatch)
    argv=['--output',str(explicit),'--world-profile','sparse','--search-timeout=900','--through','resolve']
    assert MODULE.main(argv)==0
    report=json.loads((explicit/'stage-a1-final-report.json').read_text())
    assert report['invocation']['argv']==argv and report['invocation']['defaulted']==['segment_planning_time']
    assert report['invocation']['effective']['world_profile']=='sparse' and report['search_timeout_s']==900.
    assert 'world_profile=sparse,' in capsys.readouterr().err


def test_explicit_value_equal_to_the_default_is_not_reported_as_defaulted(tmp_path,monkeypatch,stubbed_preflight):
    record_sessions(monkeypatch)
    output=tmp_path/'evidence'
    assert MODULE.main(['--output',str(output),'--world-profile','pile','--search-time','300',
                        '--segment-planning-time','3','--through','resolve'])==0   # abbreviation accepted by argparse
    assert json.loads((output/'stage-a1-final-report.json').read_text())['invocation']['defaulted']==[]


def test_fast_mode_runs_build_and_focused_tests_without_any_session_or_world(tmp_path,monkeypatch,stubbed_preflight):
    seen=record_sessions(monkeypatch)
    def no_world(*args):pytest.fail('fast mode must not resolve a simulator world')
    monkeypatch.setattr(MODULE,'discover_source_world',no_world)
    calls=[]
    def fake_run(command,*,env,cwd,log,timeout,check=True):
        assert 'WORKCELL_SIMULATOR_PHYSICS_ROOT' not in env                        # unit suites run hermetically
        calls.append((command,env.get('ROS_DOMAIN_ID'),timeout,check))
        Path(log).write_text('........\n724 passed in 27.76s\n')
        return type('Completed',(),{'returncode':0})()
    monkeypatch.setattr(MODULE,'run',fake_run)
    monkeypatch.setenv('WORKCELL_SIMULATOR_PHYSICS_ROOT','/qualified/physics')   # runtime-only selection
    output=tmp_path/'evidence'
    assert MODULE.main(['--output',str(output),'--mode','fast'])==0
    assert seen==[] and stubbed_preflight==['build']
    (command,domain,timeout,check),=calls
    assert command[1:5]==['-m','pytest','-q','-p'] and tuple(command[6:])==MODULE.FAST_TEST_FILES
    assert domain=='200' and timeout==MODULE.FAST_TEST_TIMEOUT_S and check is False
    report=json.loads((output/'stage-a1-fast-report.json').read_text())
    assert report['status']=='PASS' and report['mode']=='fast' and report['stage_a_acceptance'] is False
    assert report['fast_tests']['counts']=={'passed':724} and report['gates']=={}
    assert report['fast_tests']['env_excluded']==['WORKCELL_SIMULATOR_PHYSICS_ROOT']
    assert not (output/'stage-a1-final-report.json').exists()


@pytest.mark.parametrize('tail,returncode',[('3 failed, 721 passed in 27s',1),('1 error in 9.6s',2),('',0),('no tests ran in 0.01s',5),
                                             ('5 passed in 1.0s',1),('collected 0 items',0)])
def test_fast_mode_reports_failed_tests_as_blocked(tmp_path,monkeypatch,stubbed_preflight,tail,returncode):
    def fake_run(command,*,env,cwd,log,timeout,check=True):
        Path(log).write_text(tail+'\n');return type('Completed',(),{'returncode':returncode})()
    monkeypatch.setattr(MODULE,'run',fake_run)
    output=tmp_path/'evidence'
    assert MODULE.main(['--output',str(output),'--mode','fast'])==1
    report=json.loads((output/'stage-a1-fast-report.json').read_text())
    assert report['status']=='BLOCKED' and 'focused Stage-A tests failed' in report['failure']


def test_fast_mode_reads_the_pytest_summary_even_when_output_follows_it(tmp_path,monkeypatch,stubbed_preflight):
    def fake_run(command,*,env,cwd,log,timeout,check=True):
        Path(log).write_text('....\n754 passed in 34.1s\nException ignored in atexit callback\n')
        return type('Completed',(),{'returncode':0})()
    monkeypatch.setattr(MODULE,'run',fake_run)
    output=tmp_path/'evidence'
    assert MODULE.main(['--output',str(output),'--mode','fast'])==0
    assert json.loads((output/'stage-a1-fast-report.json').read_text())['fast_tests']['counts']=={'passed':754}


def test_fast_test_list_covers_every_suite_of_the_modules_the_runner_and_executor_import():
    import ast
    repo=SCRIPT.parents[1]
    assert all((repo/name).is_file() for name in MODULE.FAST_TEST_FILES)
    imported=set()
    for script in ('run_stage_a1_finish','perceived_object_grasp_execute','simulator_execution'):
        imported.add(script)
        for node in ast.walk(ast.parse((repo/'scripts'/f'{script}.py').read_text())):
            names=[a.name for a in node.names] if isinstance(node,ast.Import) else [node.module] if isinstance(node,ast.ImportFrom) and node.module else []
            imported.update(name.split('.')[0] for name in names)
    suites={f'tests/test_{name}.py' for name in imported if (repo/'scripts'/f'{name}.py').is_file() and (repo/'tests'/f'test_{name}.py').is_file()}
    assert suites and suites<=set(MODULE.FAST_TEST_FILES), sorted(suites-set(MODULE.FAST_TEST_FILES))
    assert 'tests/test_grasp_capture.py' in suites and 'tests/test_task_intent_v2.py' in suites
    # suites not named after a module: the runner's own and the preflight's bridge identity
    assert {'tests/test_stage_a1_finish_runner.py','tests/test_bridge_shutdown_overlay.py'}<=set(MODULE.FAST_TEST_FILES)


def test_gil_watchdog_flag_reaches_executor_sessions_only_when_requested(tmp_path,monkeypatch,stubbed_preflight):
    import os
    for key in (MODULE.GIL_PROBE_LIB_ENV,MODULE.GIL_PROBE_MS_ENV,MODULE.GIL_PROBE_NATIVE_PC_ENV,MODULE.GIL_WATCHDOG_ENV):
        monkeypatch.setenv(key,'');monkeypatch.delenv(key)            # recorded, so what main() sets is undone after the test
    observed=[]
    def session(args,repo,world,sha,gate,index,prior):
        observed.append(os.environ.get(MODULE.GIL_WATCHDOG_ENV))
        path=args.output/'summary.json';path.write_text('{}');return path
    monkeypatch.setattr(MODULE,'one_session',session)
    monkeypatch.setenv(MODULE.GIL_WATCHDOG_ENV,'999')                # an inherited value is never used implicitly
    assert MODULE.main(['--output',str(tmp_path/'a'),'--mode','targeted','--gate','resolve'])==0
    assert MODULE.main(['--output',str(tmp_path/'b'),'--mode','targeted','--gate','resolve','--gil-watchdog-ms','100'])==0
    assert observed==[None,'100.0']
    assert json.loads((tmp_path/'b'/'stage-a1-targeted-report.json').read_text())['gil_watchdog_ms']==100.0


def test_session_reports_carry_the_run_mode_and_scope(tmp_path,monkeypatch):
    import types
    source=SCRIPT.read_text()
    assert '"mode":getattr(args,"mode","full"),' in source
    assert '"scope":"STAGE_A_ACCEPTANCE_SEQUENCE" if getattr(args,"mode","full")=="full" else "DIAGNOSTIC_NOT_STAGE_A_ACCEPTANCE"}' in source
    # the real one_session writes them even when its very first step fails
    monkeypatch.setattr(MODULE,'prepare_scene',lambda repo,destination,class_id:destination)
    def busy(env,cwd):raise RuntimeError('ROS domain 203 is not isolated')
    monkeypatch.setattr(MODULE,'assert_domain_free',busy)
    args=types.SimpleNamespace(output=tmp_path,class_id='part',base_domain=201,partition_prefix='p',mode='targeted')
    with pytest.raises(RuntimeError,match='not isolated'):
        MODULE.one_session(args,SCRIPT.parents[1],SCRIPT,'sha','telemetry',3,{})
    report=json.loads((tmp_path/'03-telemetry'/'session-report.json').read_text())
    assert report['mode']=='targeted' and report['scope']=='DIAGNOSTIC_NOT_STAGE_A_ACCEPTANCE'
    assert report['status']=='BLOCKED' and report['domain']==203


def test_gil_probe_flag_builds_the_probe_into_the_evidence_and_reaches_sessions(tmp_path,monkeypatch,stubbed_preflight):
    import os,shutil
    for key in (MODULE.GIL_PROBE_LIB_ENV,MODULE.GIL_PROBE_MS_ENV,MODULE.GIL_PROBE_NATIVE_PC_ENV,MODULE.GIL_WATCHDOG_ENV):
        monkeypatch.setenv(key,'');monkeypatch.delenv(key)            # recorded, so what main() sets is undone after the test
    if shutil.which('cc') is None:pytest.skip('no C compiler')
    observed=[]
    def session(args,repo,world,sha,gate,index,prior):
        observed.append((os.environ.get(MODULE.GIL_PROBE_LIB_ENV),os.environ.get(MODULE.GIL_PROBE_MS_ENV)))
        path=args.output/'summary.json';path.write_text('{}');return path
    monkeypatch.setattr(MODULE,'one_session',session)
    monkeypatch.setenv(MODULE.GIL_PROBE_MS_ENV,'999')                    # never inherited implicitly
    assert MODULE.main(['--output',str(tmp_path/'a'),'--mode','targeted','--gate','resolve'])==0
    assert MODULE.main(['--output',str(tmp_path/'b'),'--mode','targeted','--gate','resolve','--gil-probe-ms','80'])==0
    report=json.loads((tmp_path/'b'/'stage-a1-targeted-report.json').read_text())
    probe=report['gil_probe']
    assert observed==[(None,None),(probe['library'],'80.0')]
    assert Path(probe['library'])==tmp_path/'b'/'gil_probe.so' and Path(probe['library']).is_file()
    assert probe['threshold_ms']==80 and len(probe['source_sha256'])==64 and len(probe['library_sha256'])==64
    assert probe['command'][:2]==['cc','-O2'] and '-Werror' in probe['command'] and probe['compiler']
    assert probe['native_pc'] is False


def test_gil_probe_native_pc_is_a_separate_opt_in_that_reaches_sessions(tmp_path,monkeypatch,stubbed_preflight):
    import os,shutil
    for key in (MODULE.GIL_PROBE_LIB_ENV,MODULE.GIL_PROBE_MS_ENV,MODULE.GIL_PROBE_NATIVE_PC_ENV,MODULE.GIL_WATCHDOG_ENV):
        monkeypatch.setenv(key,'');monkeypatch.delenv(key)            # recorded, so what main() sets is undone after the test
    if shutil.which('cc') is None:pytest.skip('no C compiler')
    observed=[]
    def session(args,repo,world,sha,gate,index,prior):
        observed.append(os.environ.get(MODULE.GIL_PROBE_NATIVE_PC_ENV))
        path=args.output/'summary.json';path.write_text('{}');return path
    monkeypatch.setattr(MODULE,'one_session',session)
    monkeypatch.setenv(MODULE.GIL_PROBE_NATIVE_PC_ENV,'1')               # never inherited implicitly
    assert MODULE.main(['--output',str(tmp_path/'a'),'--mode','targeted','--gate','resolve','--gil-probe-ms','80'])==0
    assert MODULE.main(['--output',str(tmp_path/'b'),'--mode','targeted','--gate','resolve','--gil-probe-ms','80',
                        '--gil-probe-native-pc'])==0
    assert observed==[None,'1']
    assert json.loads((tmp_path/'b'/'stage-a1-targeted-report.json').read_text())['gil_probe']['native_pc'] is True


@pytest.mark.parametrize('argv,message',[
    (['--mode','fast','--gil-probe-ms','80'],'fast mode runs none'),
    (['--mode','targeted','--gate','resolve','--gil-probe-ms','5'],'within [20, 1000] ms'),
])
def test_gil_probe_arguments_are_bounded(tmp_path,monkeypatch,stubbed_preflight,capsys,argv,message):
    with pytest.raises(SystemExit):MODULE.main(['--output',str(tmp_path/'e'),*argv])
    assert message in capsys.readouterr().err and not (tmp_path/'e').exists()


def test_failed_gil_probe_build_blocks_before_any_session(tmp_path,monkeypatch,stubbed_preflight):
    seen=record_sessions(monkeypatch)
    def broken(repo,output):raise RuntimeError('command failed (1): cc ... gil_probe.c')
    monkeypatch.setattr(MODULE,'build_gil_probe',broken)
    output=tmp_path/'evidence'
    assert MODULE.main(['--output',str(output),'--mode','targeted','--gate','telemetry','--gil-probe-ms','80'])==1
    report=json.loads((output/'stage-a1-targeted-report.json').read_text())
    assert report['status']=='BLOCKED' and 'gil_probe.c' in report['failure'] and seen==[] and stubbed_preflight==[]
