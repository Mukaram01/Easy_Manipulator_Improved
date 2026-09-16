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
    canonical_hash,
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
                or (confidence is not None and (not _finite(confidence) or not 0 <= confidence <= 1))
                or (filters.get("min_confidence") is not None and
                    (confidence is None or confidence < filters["min_confidence"]))
                or not _finite(timestamp) or timestamp > now
                or now - timestamp > (filters.get("max_age_seconds") or math.inf)
                or not _vector(pose, 7) or math.hypot(*pose[3:]) < 1e-9
                or not _vector(dimensions, 3) or any(v <= 0 for v in dimensions)):
            continue
        if any(filters.get(key) not in (None, "") and observation.get(key) != filters[key]
               for key in ("class_id", "color")):
            continue
        eligible.append(copy.deepcopy(observation))
    return sorted(eligible, key=lambda item: (item["confidence"] is None, -(item["confidence"] or 0.0), item["id"]))


def select_observations(intent, environment, observations, now):
    from runtime_pick_inputs import zone, contained
    import perceived_object_grasp_plan as geometry
    selection = intent['pick']['selection']
    source_type, source_ref = selection.get('source_type'), selection.get('source_ref')
    if not ((source_type == 'perception' and source_ref == 'detected_objects/v1') or
            (source_type == 'manual_simulated' and source_ref == selection['zone_ref'])):
        raise ValueError('PICK_SOURCE_UNSUPPORTED: select detected_objects/v1 or the manual simulated pick region')
    region = zone({'environment': environment}, selection['zone_ref'])
    return [item for item in _eligible_observations(observations, selection['object_filter'], now)
            if contained(item, region, geometry)]


def _readiness(result, status, code, reason):
    result["readiness_status"] = status
    result["readiness"].update(primary_code=code, reason=reason)
    result["resolution_sha256"] = resolution_hash(result)
    return result


def resolve_task_intent(intent, environment, cell, observations, cycle_evaluator, *, now, resolved=None) -> dict:
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
    result["context_sha256"] = context_hash(normalized, environment, cell)
    result["pick_selection"] = copy.deepcopy(normalized["pick"]["selection"])
    routing = normalized.get('routing') or {}
    rules = routing.get('rules') or []
    if (normalized['task']['type'] != 'pick_place' or routing.get('mode', 'direct') != 'direct' or
            any(rule.get('when') not in ({'always': True}, {'default': True}) or
                rule.get('destination') != normalized['place']['target']['region_ref'] for rule in rules)):
        return _readiness(result, 'BLOCKED', 'TASK_ROUTING_UNSUPPORTED',
                          'The full-cycle planner supports direct pick/place to the authored region only.')
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
        default_destination = copy.deepcopy(destination)
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
        destination=copy.deepcopy(destination),
    )

    if cycle_evaluator is None:
        return _readiness(result, "BLOCKED", "CYCLE_EVALUATION_REQUIRED",
                          "Run fake-hardware plan-only resolution for the saved task before task generation.")

    if not _finite(now):
        return _readiness(result, "BLOCKED", "OBSERVATION_TIME_INVALID", "now must be finite.")
    try:
        eligible = select_observations(normalized, environment, observations, now)
    except (ValueError, KeyError, TypeError) as exc:
        return _readiness(result, "BLOCKED", "PICK_SELECTION_INVALID", str(exc))
    if not eligible:
        return _readiness(result, "BLOCKED", "NO_ELIGIBLE_OBSERVATIONS",
                          "No current observation satisfies the selection and geometry requirements.")
    result["observations_sha256"] = canonical_hash([
        {k: v for k, v in item.items() if k != "timestamp"} for item in eligible])
    if resolved is not None:
        # Runtime consumes the chosen contract. Recheck its one candidate with
        # MoveIt, without a new AUTO search that could silently change selection.
        checked = consume_resolution(resolved, normalized, environment, cell)
        if checked.get('observations_sha256') != result['observations_sha256']:
            return _readiness(result, 'BLOCKED', 'TASK_OBSERVATIONS_CHANGED',
                              'Observed objects changed; explicitly resolve the saved task again.')
        chosen = checked['grasp_resolution']
        observation = next(item for item in eligible if item['id'] == chosen['selected_object_id'])
        candidates = generate_strategy_candidates(chosen['selected_strategy_ref'], observation,
            {'approach_distance_m': grasp['approach']['distance_m']})
        candidate = next((item for item in candidates if item.candidate_id == chosen['selected_candidate_id']), None)
        if candidate is None:
            return _readiness(result, 'BLOCKED', 'TASK_CANDIDATE_CHANGED', 'Resolved candidate is no longer available.')
        evaluation = cycle_evaluator({'intent': copy.deepcopy(normalized), 'cell': copy.deepcopy(cell),
            'strategy_ref': candidate.strategy_ref, 'observation': copy.deepcopy(observation),
            'candidate': candidate, 'destination': copy.deepcopy(checked['place_resolution']['destination'])})
        checks = copy.deepcopy(evaluation.get('checks', []))
        checked['readiness']['checks'] = checks
        if evaluation.get('success') is not True or any(c.get('status') in ('FAIL', 'BLOCKED') for c in checks):
            return _readiness(checked, 'BLOCKED', evaluation.get('reason_code') or 'RESOLVED_CYCLE_BLOCKED',
                              evaluation.get('reason') or 'Resolved candidate failed; no substitution was attempted.')
        return checked
    requested = grasp.get("strategy_ref")
    strategies = list(STRATEGY_ORDER)
    if grasp["policy"] == "EXACT":
        strategies = [requested]
    elif grasp["policy"] == "PREFERRED":
        strategies = [requested] + [item for item in strategies if item != requested]

    preferred_failure = None
    last_failure = {"reason_code": "NO_FEASIBLE_CYCLE", "reason": "No complete cycle succeeded."}
    destinations = [destination]
    if (placement['policy'] == 'PREFERRED' and not place_resolution['fallback']['used']
            and (destination['pose_xyz'] != default_destination['pose_xyz'] or
                 destination['pose_rpy'] != default_destination['pose_rpy'])):
        destinations.append(default_destination)
    for destination_index, destination in enumerate(destinations):
        if destination_index:
            place_resolution['fallback'] = {'used': True, **last_failure}
            local = destination['placement_local']
            place_resolution.update(
                selected_local_pose={'xyz_m': list(local.get('pose_xyz', local.get('pose', {}).get('xyz'))),
                                     'rpy_rad': list(local.get('pose_rpy', local.get('pose', {}).get('rpy')))},
                world_pose={'xyz_m': list(destination['pose_xyz']), 'rpy_rad': list(destination['pose_rpy'])},
                destination=copy.deepcopy(destination))
        for strategy in strategies:
            for observation in eligible:
                try:
                    candidates = generate_strategy_candidates(
                        strategy, observation,
                        {"approach_distance_m": grasp["approach"]["distance_m"]})
                except (ValueError, KeyError, TypeError) as exc:
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
                        break
                if grasp["policy"] == "EXACT":
                    break
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


def resolution_hash(result):
    """Identity of semantic resolution, excluding diagnostic wording/timing."""
    grasp = result.get('grasp_resolution', {})
    place = result.get('place_resolution', {})
    def semantic(value):
        if isinstance(value, dict):
            return {k: semantic(v) for k, v in value.items() if k not in ('reason', 'review')}
        if isinstance(value, list):
            return [semantic(v) for v in value]
        return value
    return canonical_hash(semantic({
        'normalized_intent_sha256': result.get('normalized_intent_sha256'),
        'context_sha256': result.get('context_sha256'),
        'observations_sha256': result.get('observations_sha256'),
        'grasp': {k: v for k, v in grasp.items() if k != 'attempts'},
        'place': place,
        'status': result.get('readiness_status'),
        'code': result.get('readiness', {}).get('primary_code'),
    }))


def context_hash(intent, environment, cell):
    # Physical input, including obstacles, is semantic. Metadata prose is not.
    def semantic(value):
        if isinstance(value, dict):
            return {k: semantic(v) for k, v in value.items()
                    if k not in ('review', 'metadata', 'provenance', 'source_file')}
        if isinstance(value, list):
            return [semantic(v) for v in value]
        return value
    return canonical_hash({'intent': normalized_intent_hash(intent),
                           'environment': semantic(environment),
                           'robot': semantic(cell.get('robot', {})),
                           'tool': semantic(cell.get('end_effector', {}))})


def consume_resolution(result, intent, environment, cell, *, require_ready=True, generated_cell=None):
    """Validate derived evidence against current inputs; never repair stale data."""
    if generated_cell is not None and generated_cell.get('resolution_sha256') != result.get('resolution_sha256'):
        raise ValueError('TASK_HANDOFF_STALE: Generate the current resolution before consuming it')
    if result.get('context_sha256') != context_hash(intent, environment, cell):
        raise ValueError('TASK_RESOLUTION_STALE: saved task or physical scene changed; resolve again')
    if result.get('normalized_intent_sha256') != normalized_intent_hash(intent):
        raise ValueError('TASK_RESOLUTION_STALE: normalized task differs')
    if result.get('resolution_sha256') != resolution_hash(result):
        raise ValueError('TASK_RESOLUTION_CORRUPT: resolution identity differs')
    if require_ready and result.get('readiness_status') not in ('READY', 'WARNING'):
        raise ValueError(result['readiness']['primary_code'] + ': ' + result['readiness']['reason'])
    return copy.deepcopy(result)


def scene_resolution(scene, intent, environment, cell, *, require_ready=False):
    """Consume the existing derived artifact, or expose unresolved physical truth.

    Missing/stale evidence is diagnostic only; executable consumers fail closed.
    Corrupt evidence is always rejected.
    There is no independent resolver cache or PASS-file bypass.
    """
    path = Path(scene) / 'generated/task_intent_resolution.json'
    if path.is_file():
        try:
            return consume_resolution(json.loads(path.read_text()), intent, environment, cell,
                                      require_ready=require_ready)
        except ValueError as exc:
            if require_ready or not str(exc).startswith('TASK_RESOLUTION_STALE'):
                raise
            result = resolve_task_intent(intent, environment, cell, [], None, now=0.)
            result['diagnostics'] = [{'code': 'TASK_RESOLUTION_STALE', 'message': str(exc)}]
            return result
    result = resolve_task_intent(intent, environment, cell, [], None, now=0.)
    if require_ready:
        raise ValueError(result['readiness']['primary_code'] + ': ' + result['readiness']['reason'])
    return result


def read_scene_task(scene):
    """Read the same persisted model used by the existing authoring adapter."""
    from task_intent_authoring import load_authoring
    scene = Path(scene)
    if not (scene / 'config/workcell_builder_task_intent.yaml').is_file():
        raise ValueError('TASK_INTENT_MISSING: Save the task in Builder before generating or planning')
    report = load_authoring(scene)
    if report['status'] == 'FAIL':
        raise ValueError('; '.join(report['errors']))
    document = yaml.safe_load((scene / 'environment.yaml').read_text())
    return report['task_intent'], document.get('environment', document), document


def resolved_recipe(intent, result):
    """Existing task_recipe/v1 is a projection of a checked resolver result."""
    if result['readiness_status'] not in ('READY', 'WARNING'):
        raise ValueError(result['readiness']['primary_code'] + ': ' + result['readiness']['reason'])
    destination = result['place_resolution']['destination']
    selection = intent['pick']['selection']
    # Direct pick/place is already authored semantics, not a routing fallback.
    rules = copy.deepcopy(intent.get('routing', {}).get('rules') or [
        {'id': 'direct_place', 'when': {'always': True}, 'destination': destination['id']}])
    return {
        'schema_version': 'task_recipe/v1',
        'enabled': True, 'recipe_id': intent['task']['id'], 'id': intent['task']['id'],
        'task_type': intent['task']['type'], 'type': intent['task']['type'],
        'pick': {'source': selection['source_ref'], 'object_source': selection['source_type'],
                 'pick_zone': selection['zone_ref'], 'allowed_grasp_methods': ['finger'],
                 'grasp_strategy': {'strategy_ref': result['grasp_resolution']['selected_strategy_ref']}},
        'decision_rules': copy.deepcopy(rules),
        'destinations': [{'id': destination['id'], 'target_ref': destination['target_id'],
                          'frame_id': 'world', 'pose_xyz': destination['pose_xyz'],
                          'pose_rpy': destination['pose_rpy'], 'action': 'place'}],
        'expected': {'allow_fallback_rule': False, 'require_destination_pose': True},
        'normalized_intent_sha256': result['normalized_intent_sha256'],
        'resolution_sha256': result['resolution_sha256'],
        'task': {**copy.deepcopy(intent['task']),
                 'object_source': selection['source_type'],
                 'source_object': selection['source_ref'],
                 'object_filter': copy.deepcopy(selection['object_filter']),
                 'destinations': [{'id': destination['id'], 'target_ref': destination['target_id'],
                                   'frame': destination['frame_id'], 'pose_xyz': destination['pose_xyz'],
                                   'pose_rpy': destination['pose_rpy']}],
                 'decision_rules': copy.deepcopy(rules)},
        'grasp': {'strategy_ref': result['grasp_resolution']['selected_strategy_ref']},
        'task_intent_resolution': copy.deepcopy(result),
    }
