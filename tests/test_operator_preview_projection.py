"""Operator HMI preview projection: read-only, contract-derived, truthful blockers."""
import copy
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from operator_preview_projection import project  # noqa: E402
from task_intent_authoring import load_authoring  # noqa: E402

SCENE = ROOT / "scenes" / "ur5_2f_test"


def _tree_digest(root: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(p for p in root.rglob("*") if p.is_file()):
        digest.update(str(path.relative_to(root)).encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def _saved_intent():
    return copy.deepcopy(load_authoring(SCENE)["task_intent"])


@pytest.fixture
def scene_copy(tmp_path):
    target = tmp_path / "ur5_2f_test"
    shutil.copytree(SCENE, target)
    return target


def test_saved_projection_reports_the_saved_contract_and_writes_nothing():
    before = _tree_digest(SCENE)
    view = project(SCENE, evaluate_input=True)
    assert _tree_digest(SCENE) == before
    assert view["schema"] == "workcell_operator_preview/v1" and view["read_only"] is True
    assert view["revision"]["kind"] == "saved"
    saved = _saved_intent()["pick"]["selection"]["object_filter"]["class_id"]
    assert view["task"]["target"]["class_id"] == saved
    assert view["task"]["not_targeted"]["explicit_list"] is None  # v2 has no ignore list
    assert view["task"]["destinations"] and view["task"]["resolved_destination"]
    assert view["validation"]["status"] == "PASS"
    assert view["input"]["kind"] == "replay"
    assert view["execution"] == {**view["execution"], "backend": "UNAVAILABLE", "motion_authorised": False}
    assert view["approval"]["status"] == "NOT_APPROVED"
    decisions = {o["class_id"]: o for o in view["observations"]["objects"]}
    assert decisions[saved]["accepted"] is True
    others = [o for c, o in decisions.items() if c != saved]
    assert others and all(o["reason_code"] == "CLASS_NOT_TARGETED" for o in others)
    codes = [event["code"] for event in view["events"]]
    assert "EXECUTION_BACKEND_UNAVAILABLE" in codes and "OBJECT_ACCEPTED" in codes


def test_draft_is_labelled_compared_with_saved_and_drives_selection():
    intent = _saved_intent()
    saved = intent["pick"]["selection"]["object_filter"]["class_id"]
    target = "cup" if saved != "cup" else "bottle"
    intent["pick"]["selection"]["object_filter"]["class_id"] = target
    view = project(SCENE, intent, evaluate_input=True)
    assert view["revision"]["kind"] == "draft"
    assert view["revision"]["draft_differs_from_saved"] is True
    assert view["approval"]["draft_note"]
    assert view["task"]["target"]["class_id"] == target
    accepted = [o["class_id"] for o in view["observations"]["objects"] if o["accepted"]]
    assert accepted == [target]


def test_unchanged_draft_is_reported_identical_to_saved():
    view = project(SCENE, _saved_intent())
    assert view["revision"]["kind"] == "draft"
    assert view["revision"]["draft_differs_from_saved"] is False


def test_missing_task_input_blocks_the_replay_check_honestly():
    intent = _saved_intent()
    intent["perception"] = {"mode": "replayed_snapshot", "replay_path": "config/missing_replay.yaml"}
    view = project(SCENE, intent, evaluate_input=True)
    assert view["input"]["kind"] == "unavailable"
    assert view["input"]["blocker"].startswith("TASK_INPUT_UNAVAILABLE")
    assert view["observations"] is None
    assert "PERCEPTION_INPUT_UNAVAILABLE" in [event["code"] for event in view["events"]]


def test_missing_scene_and_missing_task_are_reported_not_invented(tmp_path, scene_copy):
    view = project(tmp_path / "no_such_scene")
    assert view["errors"] and view["errors"][0].startswith("SCENE_MISSING")
    assert view["task"] is None

    # Without a saved task the Builder's own adapter cannot propose one for this
    # scene; the projection reports that instead of showing an invented task.
    (scene_copy / "config/workcell_builder_task_intent.yaml").unlink()
    unsaved = project(scene_copy)
    assert unsaved["revision"]["intent_file_exists"] is False
    assert unsaved["task"] is None
    assert unsaved["errors"][0].startswith("TASK_INTENT_UNREADABLE")


def test_bundle_approval_is_read_from_the_existing_contract(scene_copy):
    generated = scene_copy / "generated"
    generated.mkdir(exist_ok=True)
    (generated / "generated_workcell_summary.json").write_text(json.dumps(
        {"approval": {"status": "approved", "approved_by": "line-lead", "approved_at": "2026-10-23T10:00:00Z"}}))
    view = project(scene_copy)
    assert view["approval"]["status"] == "APPROVED" and view["approval"]["approved_by"] == "line-lead"
    assert view["generated_bundle"]["dry_run_available"] is True
    # An unsaved draft is never presented as the approved configuration.
    assert project(scene_copy, _saved_intent())["approval"]["draft_note"]


def test_cli_accepts_a_draft_on_stdin():
    intent = _saved_intent()
    result = subprocess.run(
        [sys.executable, str(SCRIPTS / "operator_preview_projection.py"), "--scene", str(SCENE), "--draft-stdin"],
        input=json.dumps(intent), capture_output=True, text=True, check=True)
    view = json.loads(result.stdout)
    assert view["revision"]["kind"] == "draft"
    bad = subprocess.run(
        [sys.executable, str(SCRIPTS / "operator_preview_projection.py"), "--scene", str(SCENE), "--draft-stdin"],
        input="{not json", capture_output=True, text=True)
    assert bad.returncode == 2 and "DRAFT_INVALID" in bad.stdout


def _session_sorting_draft():
    """A class-routed task observed by its own physics sorting session (G4): simulated RGB-D camera, live EPD."""
    draft = _saved_intent()
    draft["perception"] = {"mode": "live_epd", "replay_path": "", "camera_source": "simulated_rgbd"}
    draft["safety"] = dict(draft.get("safety") or {}, execution_backend="simulator", require_fake_hardware=False,
                           real_hardware_enabled=False)
    draft["pick"]["selection"].update(source_type="perception", source_ref="detected_objects/v1")
    draft["pick"]["selection"]["object_filter"]["class_id"] = ""
    draft["routing"] = {"mode": "class", "unrouted_policy": "leave_in_place", "max_attempts_per_object": 1,
                        "rules": [{"id": "cups", "when": {"attribute": "class", "equals": "cup"},
                                   "destination": draft["place"]["target"]["region_ref"], "enabled": True}]}
    return draft


def test_a_session_observed_task_has_per_cycle_input_and_resolution_not_a_missing_replay():
    view = project(SCENE, _session_sorting_draft())
    assert view["input"] == {"kind": "session", "path": None, "blocker": None, "producer": "simulator_render"}
    assert view["resolution"]["status"] == "PER_CYCLE"
    assert view["cell"]["perception"]["mode"] == "live_epd"
    assert view["cell"]["perception"]["camera_source"] == "simulated_rgbd"
    codes = {event["code"] for event in view["events"]}
    assert {"INPUT_SESSION_RENDERED", "ROUTING_SELECTS"} <= codes
    assert not codes & {"PERCEPTION_INPUT_UNAVAILABLE", "TARGET_CLASS_UNSET"}
    assert view["execution"]["motion_authorised"] is False


def test_tool_and_camera_fall_back_to_the_environment_document(scene_copy):
    manifest = yaml.safe_load((scene_copy / "scene_manifest.yaml").read_text())
    manifest.pop("camera", None)
    manifest["end_effector"] = {"type": "finger"}
    (scene_copy / "scene_manifest.yaml").write_text(yaml.safe_dump(manifest))
    document = yaml.safe_load((scene_copy / "environment.yaml").read_text())
    document["camera"] = {"enabled": True, "camera_id": "cell_camera"}
    document["end_effector"] = {"id": "robotiq_85_gripper", "type": "finger"}
    (scene_copy / "environment.yaml").write_text(yaml.safe_dump(document))
    cell = project(scene_copy)["cell"]
    assert cell["camera"] == {"id": "cell_camera", "enabled": True}
    assert cell["end_effector"]["model"] == "robotiq_85_gripper" and cell["end_effector"]["type"] == "finger"
