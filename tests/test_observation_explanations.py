"""explain_observations() states why each object is or is not selected, using
exactly the rules select_observations() applies."""
import copy
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from task_intent_resolver import explain_observations, observation_rejection, select_observations  # noqa: E402
from test_task_intent_resolver import environment, observations, valid_intent  # noqa: E402

NOW = 100.0


def _intent(**object_filter):
    intent = valid_intent()
    intent["pick"]["selection"]["object_filter"] = {"class_id": "bottle", "max_age_seconds": 5.0, **object_filter}
    return intent


def _variants():
    base = observations()[0]
    cup = dict(copy.deepcopy(base), id="epd::cup-1", class_id="cup")
    stale = dict(copy.deepcopy(base), id="epd::bottle-stale", timestamp=NOW - 60.0)
    far = dict(copy.deepcopy(base), id="epd::bottle-far", pose=[5.0, 5.0, 0.1, 0.0, 0.0, 0.0, 1.0])
    camera = dict(copy.deepcopy(base), id="epd::bottle-camera", frame_id="camera")
    return [base, cup, stale, far, camera]


def test_each_rejection_carries_its_actual_reason():
    decisions = {d["id"]: d for d in explain_observations(_intent(), environment(), _variants(), NOW)}
    assert decisions["epd::bottle-1"]["accepted"] is True
    assert decisions["epd::bottle-1"]["reason_code"] == "SELECTED"
    assert decisions["epd::cup-1"]["reason_code"] == "CLASS_NOT_TARGETED"
    assert decisions["epd::bottle-stale"]["reason_code"] == "OBSERVATION_STALE"
    assert decisions["epd::bottle-far"]["reason_code"] == "OUTSIDE_PICK_ZONE"
    assert decisions["epd::bottle-camera"]["reason_code"] == "FRAME_NOT_WORLD"


def test_explanations_agree_with_selection():
    intent, env, objects = _intent(), environment(), _variants()
    selected = {item["id"] for item in select_observations(intent, env, objects, NOW)}
    explained = {d["id"] for d in explain_observations(intent, env, objects, NOW) if d["accepted"]}
    assert explained == selected == {"epd::bottle-1"}


def test_confidence_minimum_reason():
    base = observations()[0]
    assert observation_rejection(base, {"min_confidence": 0.95}, NOW)[0] == "CONFIDENCE_BELOW_MINIMUM"
    assert observation_rejection(dict(base, confidence=None), {"min_confidence": 0.5}, NOW)[0] == \
        "CONFIDENCE_BELOW_MINIMUM"
    assert observation_rejection(base, {"min_confidence": 0.5}, NOW) is None
