"""R2.4A: class-to-destination routing in the existing TaskIntent v2 resolver (no new planner)."""
from __future__ import annotations

import copy

import pytest

from scripts import simulator_world
from scripts.task_intent_resolver import (
    consume_resolution,
    explain_observations,
    placement_fit_blocker,
    resolution_hash,
    resolve_task_intent,
    resolved_recipe,
)
from scripts.task_intent_v2 import normalized_intent_hash
from tests.test_task_intent_resolver import cell, environment, pass_cycle, valid_intent


def routed_intent(rules=None):
    intent = valid_intent()
    intent["pick"]["selection"]["object_filter"]["class_id"] = ""
    intent["place"]["target"] = {"asset_ref": "bin_a", "region_ref": "drop_a"}
    intent["routing"] = {"mode": "class", "rules": rules if rules is not None else [
        {"id": "toy_a_to_bin_a", "when": {"attribute": "class", "equals": "toy_a"}, "destination": "drop_a"},
        {"id": "toy_b_to_bin_b", "when": {"attribute": "class", "equals": "toy_b"}, "destination": "drop_b"},
    ]}
    return intent


def two_bins():
    """The resolver fixture's bin twice: bin_a at y=+2 and bin_b at y=-2, each with its own region."""
    env = environment()
    bin_a, drop_a = env["assets"][0], env["task_zones"][1]
    bin_a["id"], drop_a["id"], drop_a["target_ref"] = "bin_a", "drop_a", "bin_a"
    bin_b, drop_b = copy.deepcopy(bin_a), copy.deepcopy(drop_a)
    bin_b["id"], drop_b["id"], drop_b["target_ref"] = "bin_b", "drop_b", "bin_b"
    bin_b["pose_xyz"] = [1.0, -2.0, 3.0]
    drop_b["pose_xyz"] = [1.05, -2.0, 3.10]
    env["assets"].append(bin_b)
    env["task_zones"].append(drop_b)
    return env


def observation(object_id, class_id, x=0.4, dimensions=(0.06, 0.05, 0.10)):
    return {"id": object_id, "class_id": class_id, "confidence": 0.95, "timestamp": 99.0, "frame_id": "world",
            "shape": "BOX", "pose": [x, -0.1, dimensions[2] / 2, 0.0, 0.0, 0.0, 1.0], "dimensions": list(dimensions)}


def recording(evaluated):
    def evaluator(request):
        evaluated.append((request["observation"]["id"], request["destination"]["id"]))
        return pass_cycle(request)
    return evaluator


def resolve(intent, observations, evaluator=pass_cycle, env=None, **kwargs):
    return resolve_task_intent(intent, env or two_bins(), cell(), observations, evaluator, now=100.0, **kwargs)


def test_toy_a_routes_to_bin_a_and_toy_b_to_bin_b_with_the_same_saved_task():
    intent = routed_intent()
    a = resolve(intent, [observation("epd::toy-a", "toy_a")])
    b = resolve(intent, [observation("epd::toy-b", "toy_b")])
    for result, rule, region, asset, y in ((a, "toy_a_to_bin_a", "drop_a", "bin_a", 2.0),
                                           (b, "toy_b_to_bin_b", "drop_b", "bin_b", -2.0)):
        assert result["readiness_status"] == "READY", result["readiness"]
        place = result["place_resolution"]
        assert place["route"]["rule_id"] == rule and place["route"]["region_ref"] == region
        assert place["route"]["asset_ref"] == asset == place["destination"]["target_id"]
        assert place["destination"]["id"] == region and place["world_pose"]["xyz_m"] == [1.05, y, 3.10]
        assert [r["region_ref"] for r in place["routes"]] == ["drop_a", "drop_b"]
    assert a["normalized_intent_sha256"] == b["normalized_intent_sha256"] == normalized_intent_hash(intent)
    assert a["resolution_sha256"] != b["resolution_sha256"]                 # the route is part of the identity


def test_each_object_is_planned_only_to_its_own_class_destination():
    evaluated = []
    failing_a = lambda request: ({"success": False, "reason_code": "NO_FEASIBLE_CYCLE", "checks": []}
                                 if request["destination"]["id"] == "drop_a" else pass_cycle(request))
    def evaluator(request):
        evaluated.append((request["observation"]["id"], request["destination"]["id"]))
        return failing_a(request)
    result = resolve(routed_intent(), [observation("epd::toy-a", "toy_a", x=0.3),
                                       observation("epd::toy-b", "toy_b", x=0.5)], evaluator)
    # toy_a's bin fails: toy_b is then picked into ITS bin; toy_a is never tried in bin_b.
    assert result["readiness_status"] == "READY"
    assert result["grasp_resolution"]["selected_object_id"] == "epd::toy-b"
    assert result["place_resolution"]["route"]["region_ref"] == "drop_b"
    assert {pair for pair in evaluated} <= {("epd::toy-a", "drop_a"), ("epd::toy-b", "drop_b")}
    assert ("epd::toy-a", "drop_a") in evaluated


def test_an_unrouted_class_is_never_picked_and_is_explained():
    evaluated = []
    intent = routed_intent()
    objects = [observation("epd::unknown", "toy_c"), observation("epd::toy-a", "toy_a", x=0.5)]
    result = resolve(intent, objects, recording(evaluated))
    assert result["grasp_resolution"]["selected_object_id"] == "epd::toy-a"
    assert {oid for oid, _ in evaluated} == {"epd::toy-a"}
    decisions = {d["id"]: d for d in explain_observations(intent, two_bins(), objects, 100.0)}
    assert decisions["epd::unknown"]["reason_code"] == "CLASS_NOT_ROUTED" and not decisions["epd::unknown"]["accepted"]
    alone = resolve(intent, [observation("epd::unknown", "toy_c")])
    assert alone["readiness"]["primary_code"] == "NO_ELIGIBLE_OBSERVATIONS"


@pytest.mark.parametrize("rules, code", [
    ([{"id": "a", "when": {"attribute": "class", "equals": "toy_a"}, "destination": "drop_a"},
      {"id": "b", "when": {"attribute": "class", "equals": "toy_a"}, "destination": "drop_b"}], "ROUTING_CLASS_AMBIGUOUS"),
    ([{"id": "a", "when": {"attribute": "class", "equals": "toy_a"}, "destination": ""}], "ROUTING_DESTINATION_MISSING"),
    ([{"id": "a", "when": {"attribute": "class", "equals": "toy_a"}, "destination": "drop_a"},
      {"id": "rest", "when": {"default": True}, "destination": "drop_b"}], "ROUTING_CONDITION_UNSUPPORTED"),
    ([{"id": "a", "when": {"attribute": "colour", "equals": "red"}, "destination": "drop_a"}], "ROUTING_CONDITION_UNSUPPORTED"),
    ([], "ROUTING_RULES_REQUIRED"),
    ([{"id": "b", "when": {"attribute": "class", "equals": "toy_b"}, "destination": "drop_b"}], "ROUTING_PRIMARY_DESTINATION"),
])
def test_ambiguous_missing_or_unsupported_rules_block(rules, code):
    result = resolve(routed_intent(rules), [observation("epd::toy-a", "toy_a")])
    assert result["readiness"]["primary_code"] == "INTENT_INVALID"
    assert result["diagnostics"][0]["code"] == code


def test_routing_conflicts_with_other_selection_authorities_block():
    filtered = routed_intent()
    filtered["pick"]["selection"]["object_filter"]["class_id"] = "toy_a"
    excluded = routed_intent()
    excluded["pick"]["selection"]["object_filter"]["exclude_class_ids"] = ["toy_b"]
    exact = routed_intent()
    exact["place"]["placement"].update(policy="PREFERRED", requested_local_pose={"xyz_m": [0.05, 0, 0.1], "rpy_rad": [0, 0, 0]})
    for intent, code in ((filtered, "ROUTING_CLASS_FILTER_CONFLICT"), (excluded, "ROUTING_CLASS_EXCLUDED"),
                         (exact, "ROUTING_PLACEMENT_POLICY")):
        assert resolve(intent, [observation("epd::toy-a", "toy_a")])["diagnostics"][0]["code"] == code


def test_a_destination_must_be_physically_true():
    missing = routed_intent()
    missing["routing"]["rules"][1]["destination"] = "drop_nowhere"
    result = resolve(missing, [observation("epd::toy-a", "toy_a")])
    assert result["readiness"]["primary_code"] == "TASK_ROUTE_DESTINATION_INVALID"
    assert "toy_b_to_bin_b" in result["readiness"]["reason"]
    env = two_bins()
    env["assets"][1]["collision"] = {"enabled": False}
    result = resolve(routed_intent(), [observation("epd::toy-a", "toy_a")], env=env)
    assert result["readiness"]["primary_code"] == "TASK_ROUTE_DESTINATION_INVALID"
    assert "collision" in result["readiness"]["reason"]
    env = two_bins()
    for block in (env["task_zones"][2], env["task_zones"][2]["placement_local"]):
        block["dimensions"] = [0.6, 0.1, 0.24]                                    # larger than bin_b's interior
    result = resolve(routed_intent(), [observation("epd::toy-a", "toy_a")], env=env)
    assert result["readiness"]["primary_code"] == "TASK_ROUTE_DESTINATION_INVALID"
    assert "outside target usable placement" in result["readiness"]["reason"]


def test_an_object_that_does_not_fit_its_own_bin_is_not_sent_to_another():
    env = two_bins()
    for block in (env["task_zones"][2], env["task_zones"][2]["placement_local"]):
        block["dimensions"] = [0.20, 0.10, 0.08]                                  # bin_b region too low for 0.10 m
    evaluated = []
    result = resolve(routed_intent(), [observation("epd::toy-b", "toy_b")], recording(evaluated), env=env)
    assert result["readiness"]["primary_code"] == "PLACE_OBJECT_DOES_NOT_FIT"
    assert evaluated == []
    blocker = placement_fit_blocker(routed_intent(), env, [observation("epd::toy-b", "toy_b")])
    assert blocker.startswith("PLACE_OBJECT_DOES_NOT_FIT: epd::toy-b cannot be placed at drop_b")


def test_revalidation_keeps_the_resolved_route_and_rejects_a_substituted_bin():
    intent, env, objects = routed_intent(), two_bins(), [observation("epd::toy-b", "toy_b")]
    resolved = resolve(intent, objects, env=env)
    checked = resolve(intent, objects, env=env, resolved=resolved)
    assert checked["readiness_status"] == "READY"
    assert checked["place_resolution"]["route"] == resolved["place_resolution"]["route"]
    # A handoff that names bin_a for the toy_b object (well-formed identity) is refused, not executed.
    swapped = copy.deepcopy(resolved)
    a = resolve(intent, [observation("epd::toy-a", "toy_a")], env=env)["place_resolution"]
    swapped["place_resolution"].update(route=a["route"], destination=a["destination"], world_pose=a["world_pose"])
    swapped["resolution_sha256"] = resolution_hash(swapped)
    consume_resolution(swapped, intent, env, cell())                 # identity is intact ...
    result = resolve(intent, objects, env=env, resolved=swapped)
    assert result["readiness"]["primary_code"] == "TASK_ROUTE_CHANGED"   # ... but the route is not the class's


def test_direct_tasks_are_unchanged_and_other_rules_stay_unsupported():
    direct = resolve_task_intent(valid_intent(), environment(), cell(),
                                 [observation("epd::bottle", "bottle")], pass_cycle, now=100.0)
    assert direct["readiness_status"] == "READY" and "routes" not in direct["place_resolution"]
    assert "route" not in direct["place_resolution"]
    unsupported = valid_intent()
    unsupported["routing"]["rules"][0]["destination"] = "elsewhere"
    result = resolve_task_intent(unsupported, environment(), cell(), [observation("epd::bottle", "bottle")],
                                 pass_cycle, now=100.0)
    assert result["readiness"]["primary_code"] == "TASK_ROUTING_UNSUPPORTED"


def test_the_recipe_projection_lists_every_routed_destination():
    intent = routed_intent()
    result = resolve(intent, [observation("epd::toy-a", "toy_a")])
    recipe = resolved_recipe(intent, result)
    assert [d["id"] for d in recipe["destinations"]] == ["drop_a", "drop_b"]
    assert {rule["destination"] for rule in recipe["decision_rules"]} == {"drop_a", "drop_b"}
    direct = resolve_task_intent(valid_intent(), environment(), cell(), [observation("epd::b", "bottle")],
                                 pass_cycle, now=100.0)
    assert [d["id"] for d in resolved_recipe(valid_intent(), direct)["destinations"]] == ["drop"]


def test_physics_stays_blocked_for_a_routed_task(monkeypatch, tmp_path):
    monkeypatch.setattr(simulator_world, "_cell", lambda scene: ({}, {}, {}, routed_intent(), {}))
    with pytest.raises(simulator_world.SimulatorWorldError, match="TASK_ROUTING_PHYSICS_UNQUALIFIED"):
        simulator_world.derive_world(tmp_path)
