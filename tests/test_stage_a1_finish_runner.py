import importlib.util
import json
from pathlib import Path

import pytest

SCRIPT=Path(__file__).parents[1]/"scripts/run_stage_a1_finish.py"
SPEC=importlib.util.spec_from_file_location("run_stage_a1_finish",SCRIPT)
MODULE=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(MODULE)


def test_plan_gate_requires_complete_nine_stage_plan():
    summary={"result":"PLAN_ONLY","full_cycle_prevalidated":True,"execution_attempted":False,
        "task_intent_resolution":{"readiness_status":"READY"},
        "plan_metadata":[{"stage":stage,"success":True,"moveit_code":1,"points":2}
                         for stage in MODULE.STAGES]}
    MODULE.assert_plan(summary,require_resolved=True)
    bad=json.loads(json.dumps(summary));bad["plan_metadata"].pop()
    with pytest.raises(RuntimeError,match="nine-stage"):MODULE.assert_plan(bad,require_resolved=True)


@pytest.mark.parametrize("gate,result",[
    ("telemetry","MOTION_TELEMETRY_PASS"),
    ("stationary","STATIONARY_RETENTION_PASS"),
    ("contact-release","CONTACT_RELEASE_PASS"),
    ("full-cycle","PASS"),
])
def test_gate_results_are_fail_closed(gate,result):
    summary={"result":result,"full_cycle_prevalidated":True}
    if gate=="telemetry":
        summary["motion_telemetry"]={"max_fresh_age_ms":10.,"max_delivery_ms":9.}
    elif gate=="stationary":
        summary["stationary_retention"]={"duration_sim_ns":1_100_000_000,
            "required_contact_links":["left","right"],"contact_links":["left","right"]}
    elif gate=="contact-release":
        summary.update(verified_lift_clearance_m=.02,release_evidence={"settled":True})
    else:
        summary.update(full_cycle_execution_success=True,
            full_cycle_physical_acceptance={"final_collision_valid":True,"attached_ids":[]})
    MODULE.assert_gate(gate,summary)
    summary["result"]="FAIL"
    with pytest.raises(RuntimeError):MODULE.assert_gate(gate,summary)


def test_executor_full_cycle_requires_explicit_evidence(tmp_path):
    cmd=MODULE.executor_command(Path("/repo"),Path("/scene"),Path("/receipt"),
        Path("/parts"),Path("/summary"),3.0,gate="full-cycle",evidence=tmp_path/"evidence.json")
    assert "--start" in cmd
    assert cmd[cmd.index("--simulator-commission")+1]=="full-cycle"
    assert cmd[cmd.index("--commission-evidence")+1]==str(tmp_path/"evidence.json")


def test_runner_constants_pin_frozen_world_and_qualified_capability():
    assert MODULE.SOURCE_WORLD_SHA256=="39c2aafb62a01af49663f21b734534843d0d4e4e034a164da2eadb03a761f60e"
    assert MODULE.QUALIFIED_CAPABILITY_SHA256=="9f750e46a438d4b415afb07d3d3b77ee66f636fd3beedb3ec8b3e5d90d8d0489"
