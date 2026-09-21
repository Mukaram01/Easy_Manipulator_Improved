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
             "commissioning_capability":{"sha256":sha}}
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
    MODULE.assert_gate(gate,summary,sha)
    with pytest.raises(RuntimeError,match="different commissioning"):
        MODULE.assert_gate(gate,summary,"other-build")


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
