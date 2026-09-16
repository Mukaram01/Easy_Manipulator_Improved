"""Resolve authored task intent using physical truth and complete-cycle evidence."""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import sys

import yaml

# The shared planning helpers also support direct execution from scripts/.
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from grasp_strategy_candidates import generate_strategy_candidates
from physical_destination import (
    RequestedLocalPoseError,
    resolve_destination,
    resolve_local_destination,
)
from task_intent_v2 import (
    TOOL_CAPABILITY_MAP,
    normalized_intent_hash,
    parse_validate_normalize,
)


STRATEGY_ORDER = ("top_2f", "side_grip_basic", "finger_pinch_basic")


def _finite(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def _vector(value, size):
    return (isinstance(value, (list, tuple)) and len(value) == size
            and all(_finite(item) for item in value))


def _eligible_observations(observations, filters, now):
    eligible = []
    for observation in observations:
        if not isinstance(observation, dict):
            continue
        confidence = observation.get("confidence")
        timestamp = observation.get("timestamp")
        pose = observation.get("pose")
        dimensions = observation.get("dimensions")
        if (not isinstance(observation.get("id"), str) or not observation["id"]
                or observation.get("frame_id") != "world"
                or observation.get("shape") != "BOX"
                or not _finite(confidence) or not 0 <= confidence <= 1
                or confidence < filters.get("min_confidence", 0.0)
                or not _finite(timestamp) or timestamp > now
                or now - timestamp > filters.get("max_age_seconds", math.inf)
                or not _vector(pose, 7) or math.hypot(*pose[3:]) < 1e-9
                or not _vector(dimensions, 3) or any(v <= 0 for v in dimensions)):
            continue
        if any(filters.get(key) is not None and observation.get(key) != filters[key]
               for key in ("class_id", "color")):
            continue
        eligible.append(copy.deepcopy(observation))
    return sorted(eligible, key=lambda item: (-item["confidence"], item["id"]))


def _readiness(result, status, code, reason):
    result["readiness_status"] = status
    result["readiness"].update(primary_code=code, reason=reason)
    return result


def resolve_task_intent(intent, environment, cell, observations, cycle_evaluator, *, now) -> dict:
    """Resolve deterministically; only the evaluator establishes cycle feasibility.

    Each evaluator request contains the normalized intent, installed cell,
    observation, generated candidate, strategy reference and physical destination.
    EXACT evaluates the first candidate once, without candidate substitution.
    """
    result = {
        "schema": "workcell_task_intent_resolution/v1",
        "normalized_intent_sha256": None,
        "capability_profile": {},
        "grasp_resolution": {},
        "place_resolution": {},
        "readiness_status": "BLOCKED",
        "readiness": {"primary_code": None, "reason": "", "checks": []},
    }
    try:
        parsed = parse_validate_normalize(intent)
    except ValueError as exc:
        return _readiness(result, "BLOCKED", "INTENT_INVALID", str(exc))
    if parsed["diagnostics"]:
        result["diagnostics"] = parsed["diagnostics"]
        return _readiness(result, "BLOCKED", "INTENT_INVALID",
                          parsed["diagnostics"][0]["message"])
    normalized = parsed["normalized"]
    result["normalized_intent_sha256"] = normalized_intent_hash(normalized)
    grasp = normalized["pick"]["grasp"]
    placement = normalized["place"]["placement"]
    grasp_resolution = result["grasp_resolution"] = {
        "policy": grasp["policy"],
        "requested_strategy_ref": grasp.get("strategy_ref"),
        "selected_strategy_ref": None,
        "selected_object_id": None,
        "selected_candidate_id": None,
        "fallback": {"used": False},
        "attempts": [],
    }
    place_resolution = result["place_resolution"] = {
        "policy": placement["policy"],
        "requested_local_pose": copy.deepcopy(placement.get("requested_local_pose")),
        "selected_local_pose": None,
        "world_pose": None,
        "fallback": {"used": False},
    }
    tool = cell.get("end_effector") or {}
    capabilities = sorted(TOOL_CAPABILITY_MAP.get(tool.get("id"), set()))
    if tool.get("type") == "suction":
        capabilities = []
    result["capability_profile"] = {
        "installed_tool_id": tool.get("id"),
        "capabilities": capabilities,
        "release_mapping": {"tool_release": "fingers_open"} if capabilities else {},
    }
    if grasp["required_capability"] not in capabilities:
        return _readiness(result, "BLOCKED", "REQUIRED_CAPABILITY_UNAVAILABLE",
                          "Installed tool does not provide the required grasp capability.")

    target = normalized["place"]["target"]
    try:
        destination = resolve_destination(environment, target["region_ref"])
        if destination["target_id"] != target["asset_ref"]:
            raise ValueError("Place target asset does not match the physical region target.")
        if placement["policy"] != "AUTO":
            try:
                destination = resolve_local_destination(
                    environment, target["asset_ref"], target["region_ref"],
                    placement["requested_local_pose"])
            except RequestedLocalPoseError as exc:
                if placement["policy"] == "EXACT":
                    return _readiness(result, "BLOCKED", "PLACE_LOCAL_POSE_OUTSIDE_REGION", str(exc))
                place_resolution["fallback"] = {
                    "used": True, "reason_code": "PLACE_LOCAL_POSE_OUTSIDE_REGION",
                    "reason": str(exc),
                }
    except (ValueError, TypeError, AttributeError, KeyError) as exc:
        return _readiness(result, "BLOCKED", "PHYSICAL_DESTINATION_INVALID", str(exc))
    local = destination["placement_local"]
    place_resolution.update(
        selected_local_pose={
            "xyz_m": list(local.get("pose_xyz", local.get("pose", {}).get("xyz"))),
            "rpy_rad": list(local.get("pose_rpy", local.get("pose", {}).get("rpy"))),
        },
        world_pose={"xyz_m": list(destination["pose_xyz"]),
                    "rpy_rad": list(destination["pose_rpy"])},
        physical_destination_contract=destination["physical_destination_contract"],
    )

    if not _finite(now):
        return _readiness(result, "BLOCKED", "OBSERVATION_TIME_INVALID", "now must be finite.")
    eligible = _eligible_observations(
        observations, normalized["pick"]["selection"]["object_filter"], now)
    if not eligible:
        return _readiness(result, "BLOCKED", "NO_ELIGIBLE_OBSERVATIONS",
                          "No current observation satisfies the selection and geometry requirements.")
    requested = grasp.get("strategy_ref")
    strategies = list(STRATEGY_ORDER)
    if grasp["policy"] == "EXACT":
        strategies = [requested]
    elif grasp["policy"] == "PREFERRED":
        strategies = [requested] + [item for item in strategies if item != requested]

    preferred_failure = None
    last_failure = {"reason_code": "NO_FEASIBLE_CYCLE", "reason": "No complete cycle succeeded."}
    for strategy in strategies:
        for observation in eligible:
            try:
                candidates = generate_strategy_candidates(
                    strategy, observation,
                    {"approach_distance_m": grasp.get("approach", {}).get("distance_m", 0.12)})
            except ValueError as exc:
                return _readiness(result, "BLOCKED", "GRASP_CANDIDATE_INVALID", str(exc))
            for candidate in candidates:
                evaluation = cycle_evaluator({
                    "intent": copy.deepcopy(normalized), "cell": copy.deepcopy(cell),
                    "strategy_ref": strategy, "observation": copy.deepcopy(observation),
                    "candidate": candidate, "destination": copy.deepcopy(destination),
                })
                checks = copy.deepcopy(evaluation.get("checks", []))
                success = evaluation.get("success") is True and not any(
                    check.get("status") in ("FAIL", "BLOCKED") for check in checks)
                grasp_resolution["attempts"].append({
                    "strategy_ref": strategy, "object_id": observation["id"],
                    "candidate_id": candidate.candidate_id, "success": success,
                    "checks": checks,
                    "reason_code": evaluation.get("reason_code"),
                    "reason": evaluation.get("reason"),
                })
                result["readiness"]["checks"] = checks
                if success:
                    grasp_resolution.update(selected_strategy_ref=strategy,
                                            selected_object_id=observation["id"],
                                            selected_candidate_id=candidate.candidate_id)
                    if grasp["policy"] == "PREFERRED" and strategy != requested:
                        grasp_resolution["fallback"] = {"used": True, **preferred_failure}
                    fallbacks = [block["fallback"] for block in (grasp_resolution, place_resolution)
                                 if block["fallback"]["used"]]
                    if fallbacks:
                        return _readiness(result, "WARNING", fallbacks[0]["reason_code"],
                                          fallbacks[0]["reason"])
                    return _readiness(result, "READY", "COMPLETE_CYCLE_RESOLVED",
                                      "Complete cycle evaluation succeeded.")
                last_failure = {
                    "reason_code": evaluation.get("reason_code") or "NO_FEASIBLE_CYCLE",
                    "reason": evaluation.get("reason") or "Complete cycle evaluation failed.",
                }
                if strategy == requested and preferred_failure is None:
                    preferred_failure = last_failure
                if grasp["policy"] == "EXACT":
                    return _readiness(result, "BLOCKED", last_failure["reason_code"], last_failure["reason"])
    return _readiness(result, "BLOCKED", last_failure["reason_code"], last_failure["reason"])


def write_resolution_artifacts(result, output_dir) -> dict[str, str]:
    """Write the same resolution payload in YAML and JSON."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    paths = {format_: output_dir / f"task_intent_resolution.{format_}"
             for format_ in ("yaml", "json")}
    paths["yaml"].write_text(yaml.safe_dump(result, sort_keys=True), encoding="utf-8")
    paths["json"].write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n",
                             encoding="utf-8")
    return {format_: str(path) for format_, path in paths.items()}
