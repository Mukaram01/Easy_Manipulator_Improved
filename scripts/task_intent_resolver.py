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
    PLACEMENT_TOLERANCE_M,
    RequestedLocalPoseError,
    place_conflicts,
    placement_fit_problems,
    placement_slots,
    resolve_destination,
    resolve_local_destination,
)
from task_intent_v2 import (
    STRATEGY_DIR,
    installed_tool_capabilities,
    strategy_supports_tool,
    canonical_hash,
    normalized_intent_hash,
    parse_validate_normalize,
    class_routing_rules,
    enabled_class_rules,
)


STRATEGY_ORDER = ("top_2f", "side_grip_basic", "finger_pinch_basic")


def _catalog_strategy(strategy_ref):
    path = STRATEGY_DIR / f"{strategy_ref}.yaml"
    if not path.is_file():
        raise ValueError(f"strategy catalog entry missing: {strategy_ref}")
    payload = (yaml.safe_load(path.read_text()) or {}).get("grasp_strategy") or {}
    if payload.get("id") != strategy_ref:
        raise ValueError(f"strategy catalog identity mismatch: {strategy_ref}")
    return payload


def _candidate_constraints(strategy_ref, effective_grasp):
    """Generator constraints: the approach distance, plus the authored TCP offset for the pinch."""
    constraints = {"approach_distance_m": effective_grasp["approach"]["distance_m"]}
    offset = effective_grasp.get("tcp_offset_xyz_m", [0.0, 0.0, 0.0])
    if strategy_ref == "finger_pinch_basic" and any(abs(v) > 1e-12 for v in offset):
        constraints["tcp_offset_xyz_m"] = list(offset)
    return constraints


def _effective_intent_for_strategy(normalized, strategy_ref):
    """Return the exact grasp constraints evaluated for one strategy.

    EXACT and the requested PREFERRED strategy consume authored constraints
    unchanged. AUTO choices and PREFERRED substitute strategies use the
    selected strategy's reviewed catalog geometry instead of inheriting a
    different strategy's axis/orientation.
    """
    effective = copy.deepcopy(normalized)
    grasp = effective["pick"]["grasp"]
    requested = grasp.get("strategy_ref")
    if grasp["policy"] == "EXACT" or (
            grasp["policy"] == "PREFERRED" and strategy_ref == requested):
        return effective

    catalog = _catalog_strategy(strategy_ref)
    grasp["strategy_ref"] = strategy_ref
    grasp["approach"] = {
        "axis": catalog["approach_axis"],
        "distance_m": catalog["approach_distance_m"],
    }
    grasp["orientation"] = {
        "mode": catalog["orientation_mode"],
        "allowed_roll_deg": list(catalog.get("allowed_roll_angles_deg", [0.0])),
        "allowed_yaw_deg": list(catalog.get("allowed_yaw_angles_deg", [0.0, 90.0, 180.0, 270.0])),
        "tolerance_rad": [0.0, 0.0, 0.0],
    }
    grasp["tcp_offset_xyz_m"] = list(catalog.get("tool_frame_offset_xyz", [0.0, 0.0, 0.0]))
    grasp["tcp_offset_rpy_rad"] = list(catalog.get("tool_frame_offset_rpy", [0.0, 0.0, 0.0]))
    grasp["lift"] = {
        "axis": "z_up",
        "distance_m": catalog["retreat_distance_m"],
    }
    return effective


def _finite(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def _vector(value, size):
    return (isinstance(value, (list, tuple)) and len(value) == size
            and all(_finite(item) for item in value))


def observation_rejection(observation, filters, now, max_age_cap=None):
    """Return (code, reason) for the first eligibility check that fails, else None.

    This is the only object-eligibility rule set; selection and operator
    explanations both use it. ``max_age_cap`` is the source's freshness ceiling
    (observation_age_cap); the task's max_age_seconds can only tighten it.
    """
    if not isinstance(observation, dict):
        return "OBSERVATION_INVALID", "Observation is not a mapping."
    confidence = observation.get("confidence")
    timestamp = observation.get("timestamp")
    pose = observation.get("pose")
    dimensions = observation.get("dimensions")
    max_age = min(filters.get("max_age_seconds") or math.inf, math.inf if max_age_cap is None else max_age_cap)
    if not isinstance(observation.get("id"), str) or not observation["id"]:
        return "OBSERVATION_INVALID", "Observation has no identifier."
    if observation.get("frame_id") != "world":
        return "FRAME_NOT_WORLD", "Pose is not expressed in the world frame."
    if observation.get("shape") != "BOX":
        return "SHAPE_UNSUPPORTED", "Only box-bounded observations are supported."
    if confidence is not None and (not _finite(confidence) or not 0 <= confidence <= 1):
        return "CONFIDENCE_INVALID", "Confidence is not a probability."
    if filters.get("min_confidence") is not None and (
            confidence is None or confidence < filters["min_confidence"]):
        return "CONFIDENCE_BELOW_MINIMUM", (
            "Confidence is unavailable or below the task minimum.")
    if not _finite(timestamp) or timestamp > now:
        return "TIMESTAMP_INVALID", "Observation timestamp is missing or in the future."
    if now - timestamp > max_age:
        return "OBSERVATION_STALE", "Observation is older than the task's maximum age or the source's freshness limit."
    if not _vector(pose, 7) or math.hypot(*pose[3:]) < 1e-9:
        return "POSE_INVALID", "Observation pose is not finite."
    if not _vector(dimensions, 3) or any(v <= 0 for v in dimensions):
        return "DIMENSIONS_INVALID", "Observation dimensions are not positive."
    for key, code in (("class_id", "CLASS_NOT_TARGETED"), ("color", "COLOR_NOT_TARGETED")):
        if filters.get(key) not in (None, "") and observation.get(key) != filters[key]:
            return code, f"{key} {observation.get(key)!r} is not the task target {filters[key]!r}."
    if observation.get("class_id") in (filters.get("exclude_class_ids") or []):
        # never a pick target; an obstacle only with validated geometry and frame
        return "CLASS_EXCLUDED", f"class_id {observation.get('class_id')!r} is excluded by the task."
    routed = filters.get("routed_class_ids")
    if routed is not None and observation.get("class_id") not in routed:
        # class routing: an observed class without an enabled rule is never picked (it stays an obstacle)
        if observation.get("class_id") in (filters.get("disabled_class_ids") or []):
            return "CLASS_ROUTE_DISABLED", f"class_id {observation.get('class_id')!r} has a disabled routing rule; not picked."
        return "CLASS_NOT_ROUTED", f"class_id {observation.get('class_id')!r} has no routing rule; not picked."
    return None


def _selection_filters(intent):
    """The pick eligibility filters, plus the routed classes of a class-routed task."""
    filters = intent['pick']['selection']['object_filter']
    rules = class_routing_rules(intent)
    if rules is None:
        return filters
    enabled = enabled_class_rules(intent)
    return {**filters, 'routed_class_ids': sorted(rule['when']['equals'] for rule in enabled),
            'disabled_class_ids': sorted(rule['when']['equals'] for rule in rules if rule not in enabled),
            # Higher rule priority is picked first; ties keep the deterministic observation order.
            'route_priorities': {rule['when']['equals']: int(rule.get('priority', 0)) for rule in enabled}}


def class_routes(intent, environment):
    """{class: route} of a class-routed task, each with its physically resolved destination; None if direct.

    Every rule's region must resolve to a contained placement in a collision-modelled target asset
    (physical_destination.resolve_destination). Raises ValueError with the failing rule.
    """
    rules = class_routing_rules(intent)
    if rules is None:
        return None
    enabled = enabled_class_rules(intent)
    assets = {asset.get('id'): asset for asset in environment.get('assets', []) if isinstance(asset, dict)}
    routes = {}
    for rule in rules:   # disabled rules are checked too, so a saved table never hides a broken row
        try:
            destination = resolve_destination(environment, rule['destination'])
        except (ValueError, KeyError, TypeError, AttributeError) as exc:
            raise ValueError(f"rule {rule['id']}: destination {rule['destination']!r}: {exc}") from exc
        collision = (assets.get(destination['target_id']) or {}).get('collision') or {}
        if collision.get('enabled') is not True:
            raise ValueError(f"rule {rule['id']}: target {destination['target_id']!r} of {rule['destination']!r} "
                             'has no enabled collision geometry, so the planner cannot check the place')
        if rule in enabled:
            routes[rule['when']['equals']] = {
                'rule_id': rule['id'], 'class_id': rule['when']['equals'], 'region_ref': destination['id'],
                'asset_ref': destination['target_id'], 'destination': destination,
                'priority': int(rule.get('priority', 0))}
    return routes


def _route_summary(route):
    destination = route['destination']
    return {'rule_id': route['rule_id'], 'class_id': route['class_id'], 'region_ref': route['region_ref'],
            'asset_ref': route['asset_ref'], 'world_pose': {'xyz_m': list(destination['pose_xyz']),
                                                           'rpy_rad': list(destination['pose_rpy'])}}


def _record_route(place_resolution, route, destination=None):
    """place_resolution names the selected object's routed destination and place (route identity for revalidation)."""
    destination = destination or route['destination']
    local = destination['placement_local']
    place_resolution.update(
        selected_local_pose={'xyz_m': list(local['pose_xyz']), 'rpy_rad': list(local['pose_rpy'])},
        world_pose={'xyz_m': list(destination['pose_xyz']), 'rpy_rad': list(destination['pose_rpy'])},
        destination=copy.deepcopy(destination), route=_route_summary(route))


def _observation_priority(item):
    """Prefer confidence, then physically exposed/high targets, then stable ID.

    Missing-confidence simulator/perception observations previously fell back to
    lexical IDs, which makes a piled-bin search spend its budget on buried
    objects first. Height is only a deterministic search-order hint: complete
    collision-aware cycle planning remains the acceptance authority.
    """
    try:
        from perceived_object_grasp_plan import build_grasp_target, oriented_box_extents
        z_extent = oriented_box_extents(build_grasp_target(item))[2]
        top_z = float(item["pose"][2]) + float(z_extent) / 2.0
    except (KeyError, TypeError, ValueError, IndexError):
        top_z = float(item.get("pose", [0.0, 0.0, -math.inf])[2])
    confidence = item.get("confidence")
    return (confidence is None, -(confidence or 0.0), -top_z, item["id"])


def _eligible_observations(observations, filters, now, max_age_cap=None):
    eligible = [copy.deepcopy(observation) for observation in observations
                if observation_rejection(observation, filters, now, max_age_cap) is None]
    priorities = filters.get('route_priorities') or {}
    return sorted(eligible, key=lambda item: (-priorities.get(item.get('class_id'), 0),) + _observation_priority(item))


def select_observations(intent, environment, observations, now, max_age_cap=None):
    """``max_age_cap``: the actual producer's freshness ceiling (task_intent_v2.observation_age_cap)."""
    from runtime_pick_inputs import zone, contained
    import perceived_object_grasp_plan as geometry
    selection = intent['pick']['selection']
    source_type, source_ref = selection.get('source_type'), selection.get('source_ref')
    if not ((source_type == 'perception' and source_ref == 'detected_objects/v1') or
            (source_type == 'manual_simulated' and source_ref == selection['zone_ref'])):
        raise ValueError('PICK_SOURCE_UNSUPPORTED: select detected_objects/v1 or the manual simulated pick region')
    region = zone({'environment': environment}, selection['zone_ref'])
    return [item for item in _eligible_observations(observations, _selection_filters(intent), now, max_age_cap)
            if contained(item, region, geometry)]


def explain_observations(intent, environment, observations, now, max_age_cap=None):
    """Per-observation selection decisions, using exactly select_observations' rules."""
    from runtime_pick_inputs import zone, contained
    import perceived_object_grasp_plan as geometry
    selection = intent['pick']['selection']
    region = zone({'environment': environment}, selection['zone_ref'])
    selected = {item['id'] for item in select_observations(intent, environment, observations, now, max_age_cap)}
    decisions = []
    for observation in observations:
        rejection = observation_rejection(observation, _selection_filters(intent), now, max_age_cap)
        if rejection is None and not contained(observation, region, geometry):
            rejection = ("OUTSIDE_PICK_ZONE", f"Outside pick zone {selection['zone_ref']}.")
        accepted = rejection is None and observation.get('id') in selected
        decisions.append({
            'id': observation.get('id'),
            'object_id': observation.get('object_id'),
            'class_id': observation.get('class_id'),
            'confidence': observation.get('confidence'),
            'accepted': accepted,
            'reason_code': 'SELECTED' if accepted else rejection[0],
            'reason': 'Matches the task target and pick zone.' if accepted else rejection[1],
        })
    return decisions


def _placement_fit(eligible, destinations, clearance, floor_z):
    """Objects and destinations a placement can satisfy, before any motion is planned.

    Returns (fitting objects, fitting destinations, {object id: first problem per destination}).
    """
    problems = {item["id"]: [placement_fit_problems(destination, item["dimensions"], clearance, floor_z)
                             for destination in destinations] for item in eligible}
    usable = [i for i in range(len(destinations)) if any(not found[i] for found in problems.values())]
    fitting = [item for item in eligible if any(not problems[item["id"]][i] for i in usable)]
    unfit = {key: [f"{destinations[i]['id']}: {found[i][0]}" for i in range(len(destinations)) if found[i]]
             for key, found in problems.items() if any(found)}
    return fitting, [destinations[i] for i in usable], unfit


def place_destinations(intent, environment):
    """The destinations resolve_task_intent plans: (requested or region, region default, fallback, ordered list).

    EXACT outside the region raises RequestedLocalPoseError; PREFERRED records its region fallback.
    """
    target = intent["place"]["target"]
    placement = intent["place"]["placement"]
    destination = resolve_destination(environment, target["region_ref"])
    default_destination = copy.deepcopy(destination)
    if destination["target_id"] != target["asset_ref"]:
        raise ValueError("Place target asset does not match the physical region target.")
    fallback = {"used": False}
    if placement["policy"] != "AUTO":
        try:
            destination = resolve_local_destination(
                environment, target["asset_ref"], target["region_ref"],
                placement["requested_local_pose"])
        except RequestedLocalPoseError as exc:
            if placement["policy"] == "EXACT":
                raise
            fallback = {"used": True, "reason_code": "PLACE_LOCAL_POSE_OUTSIDE_REGION", "reason": str(exc)}
    destinations = [destination]
    if (placement['policy'] == 'PREFERRED' and not fallback['used']
            and (destination['pose_xyz'] != default_destination['pose_xyz'] or
                 destination['pose_rpy'] != default_destination['pose_rpy'])):
        destinations.append(default_destination)
    return destination, default_destination, fallback, destinations


MAX_PLACE_SLOTS = 3


def routed_places(environment, route, item, clearance, occupied):
    """Destinations to try for `item` in its own route: the region default, or free slots given occupancy.

    `occupied` (None: occupancy unknown, the single-object behaviour) lists the other objects in the cell as
    {id, pose (world 7-pose), dimensions}. With it, places are packed from the region corner around objects
    already there (physical_destination.placement_slots), at most MAX_PLACE_SLOTS, each a contained
    target-local placement. An empty list means the destination has no free place for this object.
    """
    destination = route['destination']
    if occupied is None:
        return [destination]
    boxes = [(other['pose'], other['dimensions']) for other in occupied if other.get('id') != item['id']]
    slots = placement_slots(destination, item['dimensions'], clearance, boxes, separation=PLACEMENT_TOLERANCE_M)
    places = []
    for slot in slots:
        if all(math.dist(slot['xyz_m'], other['placement_local']['pose_xyz']) >= 0.02 for other in places):
            places.append(resolve_local_destination(environment, route['asset_ref'], route['region_ref'], slot))
        if len(places) == MAX_PLACE_SLOTS:
            break
    return places


def _routed_placement_fit(eligible, routes, clearance, floor_z, environment=None, occupied=None):
    """Class routing: each object fits only its own class's destination, in a free place when occupancy is known.

    Returns (fitting objects, unfit reasons, {object id: places}, ids refused only because the destination is full).
    """
    unfit, places, full = {}, {}, set()
    for item in eligible:
        route = routes[item['class_id']]
        destination = route['destination']
        problems = placement_fit_problems(destination, item['dimensions'], clearance, floor_z)
        if problems:
            unfit[item['id']] = [f"{destination['id']}: {problems[0]}"]
            continue
        places[item['id']] = routed_places(environment, route, item, clearance, occupied)
        if not places[item['id']]:
            full.add(item['id'])
            unfit[item['id']] = [f"{destination['id']}: DESTINATION_FULL: no free place for its "
                                 f"{item['dimensions']} m footprint with {clearance} m clearance beside the objects "
                                 "already there"]
    return [item for item in eligible if item['id'] not in unfit], unfit, places, full


def placement_fit_blocker(intent, environment, observations, destination_floor_z=None):
    """Why none of these selected observations can be placed where the task plans, else None.

    Destination errors are left to task validation, which reports them with their own code.
    """
    clearance = intent["place"]["placement"]["clearance_m"]
    try:
        routes = class_routes(intent, environment)
        destinations = place_destinations(intent, environment)[3]
    except (ValueError, KeyError, TypeError, AttributeError):
        return None
    if routes is not None:
        fitting, unfit, _, _ = _routed_placement_fit(observations, routes, clearance, destination_floor_z)
    else:
        fitting, _, unfit = _placement_fit(observations, destinations, clearance, destination_floor_z)
    return None if fitting else "PLACE_OBJECT_DOES_NOT_FIT: " + _unfit_reason(unfit)


def _unfit_reason(unfit):
    key, problems = next(iter(unfit.items()))
    return f"{key} cannot be placed at {problems[0]}."


def _readiness(result, status, code, reason):
    result["readiness_status"] = status
    result["readiness"].update(primary_code=code, reason=reason)
    result["resolution_sha256"] = resolution_hash(result)
    return result


def resolve_task_intent(intent, environment, cell, observations, cycle_evaluator, *, now, resolved=None,
                        destination_floor_z=None, occupied=None) -> dict:
    """Resolve deterministically; only the evaluator establishes cycle feasibility.

    Each evaluator request contains the normalized intent, installed cell,
    observation, generated candidate, strategy reference and physical destination.
    EXACT evaluates the first candidate once, without candidate substitution.
    An object that cannot be contained at the destination (placement_fit_problems,
    with the planner's floor seat ``destination_floor_z``) is refused before planning.
    ``occupied`` (a sorting cycle: every other object in the cell, {id, pose, dimensions}) makes routed
    placement occupancy-aware: each object goes to a free place of its own destination, or is refused
    with DESTINATION_FULL; the considered objects are part of the resolution identity.
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
    routes = None
    if routing.get('mode') == 'class':
        # Class routing (structure checked by task_intent_v2): every rule's destination must be physically true.
        try:
            routes = class_routes(normalized, environment)
        except ValueError as exc:
            return _readiness(result, 'BLOCKED', 'TASK_ROUTE_DESTINATION_INVALID', str(exc))
    elif (normalized['task']['type'] != 'pick_place' or routing.get('mode', 'direct') != 'direct' or
            any(rule.get('when') not in ({'always': True}, {'default': True}) or
                rule.get('destination') != normalized['place']['target']['region_ref'] for rule in rules)):
        return _readiness(result, 'BLOCKED', 'TASK_ROUTING_UNSUPPORTED',
                          'The full-cycle planner supports direct pick/place to the authored region, or '
                          'class routing (routing.mode class) to authored regions only.')
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
    capabilities = sorted(installed_tool_capabilities(tool))
    result["capability_profile"] = {
        "installed_tool_id": tool.get("id"),
        "capabilities": capabilities,
        "release_mapping": {"tool_release": "fingers_open"} if capabilities else {},
    }
    if grasp["required_capability"] not in capabilities:
        return _readiness(result, "BLOCKED", "REQUIRED_CAPABILITY_UNAVAILABLE",
                          "Installed tool does not provide the required grasp capability.")

    try:
        destination, default_destination, fallback, destinations = place_destinations(normalized, environment)
        place_resolution["fallback"] = fallback
    except RequestedLocalPoseError as exc:
        return _readiness(result, "BLOCKED", "PLACE_LOCAL_POSE_OUTSIDE_REGION", str(exc))
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
    if routes is not None:
        # Until an object is selected, the task's own region is shown; the selected object's route replaces it.
        place_resolution["routes"] = [_route_summary(routes[key]) for key in sorted(routes)]
    if occupied is not None:
        place_resolution["occupancy"] = {"considered_object_ids": sorted(o["id"] for o in occupied)}

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
        if routes is not None:
            # The route chosen at resolve is kept; another bin is never substituted after planning.
            route = routes.get(observation['class_id'])
            resolved_place = checked['place_resolution']['destination']
            if (route is None or checked['place_resolution'].get('route') != _route_summary(route) or
                    resolved_place['id'] != route['region_ref'] or resolved_place['target_id'] != route['asset_ref']):
                return _readiness(checked, 'BLOCKED', 'TASK_ROUTE_CHANGED',
                                  f"{observation['id']} ({observation['class_id']!r}) no longer routes to the resolved "
                                  'destination; resolve the saved task again.')
        try:
            fitting, _, unfit = _placement_fit([observation], [checked['place_resolution']['destination']],
                                               placement['clearance_m'], destination_floor_z)
        except (ValueError, KeyError, TypeError) as exc:
            return _readiness(checked, 'BLOCKED', 'PHYSICAL_DESTINATION_INVALID', str(exc))
        if not fitting:
            return _readiness(checked, 'BLOCKED', 'PLACE_OBJECT_DOES_NOT_FIT', _unfit_reason(unfit))
        if occupied is not None:
            # The resolved place must still be free; it is never moved to another place or bin here.
            boxes = [(o['pose'], o['dimensions']) for o in occupied if o['id'] != observation['id']]
            if place_conflicts(checked['place_resolution']['destination'], checked['place_resolution']['selected_local_pose'],
                               observation['dimensions'], placement['clearance_m'], boxes, separation=PLACEMENT_TOLERANCE_M):
                return _readiness(checked, 'BLOCKED', 'DESTINATION_OCCUPIED',
                                  'The resolved place is no longer free; resolve the cycle again.')
        try:
            effective_intent = _effective_intent_for_strategy(
                normalized, chosen['selected_strategy_ref'])
        except (ValueError, KeyError, TypeError) as exc:
            return _readiness(result, 'BLOCKED', 'GRASP_CANDIDATE_INVALID', str(exc))
        effective_grasp = effective_intent['pick']['grasp']
        if (chosen.get('effective_grasp') is not None and
                chosen['effective_grasp'] != effective_grasp):
            return _readiness(result, 'BLOCKED', 'TASK_GRASP_EFFECTIVE_CHANGED',
                              'Resolved strategy-effective grasp constraints changed; resolve again.')
        candidates = generate_strategy_candidates(chosen['selected_strategy_ref'], observation,
            _candidate_constraints(chosen['selected_strategy_ref'], effective_grasp))
        candidate = next((item for item in candidates if item.candidate_id == chosen['selected_candidate_id']), None)
        if candidate is None:
            return _readiness(result, 'BLOCKED', 'TASK_CANDIDATE_CHANGED', 'Resolved candidate is no longer available.')
        evaluation = cycle_evaluator({'intent': effective_intent, 'cell': copy.deepcopy(cell),
            'strategy_ref': candidate.strategy_ref, 'observation': copy.deepcopy(observation),
            'candidate': candidate, 'destination': copy.deepcopy(checked['place_resolution']['destination']),
            'approach_ik': copy.deepcopy(chosen.get('approach_ik')),
            'transfer_ik_seed': copy.deepcopy(chosen.get('transfer_ik_seed')),
            'extraction_intent': copy.deepcopy(chosen.get('extraction_intent'))})
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
    # The strategy catalog states which tools each strategy suits; never try one it excludes.
    excluded = [item for item in strategies if not strategy_supports_tool(item, tool)]
    strategies = [item for item in strategies if item not in excluded]
    if excluded:
        grasp_resolution["tool_incompatible_strategies"] = excluded
    if not strategies:
        return _readiness(result, "BLOCKED", "STRATEGY_TOOL_INCOMPATIBLE",
                          "No requested grasp strategy is compatible with the installed tool: " + ", ".join(excluded))

    preferred_failure = None
    last_failure = {"reason_code": "NO_FEASIBLE_CYCLE", "reason": "No complete cycle succeeded."}
    try:
        if routes is None:
            eligible, fitting_destinations, unfit = _placement_fit(eligible, destinations, placement['clearance_m'],
                                                                  destination_floor_z)
        else:
            eligible, unfit, routed, full = _routed_placement_fit(eligible, routes, placement['clearance_m'],
                                                                  destination_floor_z, environment, occupied)
            fitting_destinations = destinations   # AUTO only: one pass; each object is planned to its own route
    except (ValueError, KeyError, TypeError) as exc:
        return _readiness(result, 'BLOCKED', 'PHYSICAL_DESTINATION_INVALID', str(exc))
    if unfit:
        place_resolution['placement_fit'] = unfit
    if not eligible:
        full_only = routes is not None and unfit and set(unfit) <= full
        return _readiness(result, 'BLOCKED', 'DESTINATION_FULL' if full_only else 'PLACE_OBJECT_DOES_NOT_FIT',
                          _unfit_reason(unfit))
    if not any(destinations[0] is item for item in fitting_destinations):
        # The PREFERRED pose fits no eligible object; only its region default is planned.
        last_failure = {'reason_code': 'PLACE_OBJECT_DOES_NOT_FIT', 'reason': _unfit_reason(unfit)}
    for destination_index, destination in enumerate(destinations):
        if destination_index:
            place_resolution['fallback'] = {'used': True, **last_failure}
            local = destination['placement_local']
            place_resolution.update(
                selected_local_pose={'xyz_m': list(local.get('pose_xyz', local.get('pose', {}).get('xyz'))),
                                     'rpy_rad': list(local.get('pose_rpy', local.get('pose', {}).get('rpy')))},
                world_pose={'xyz_m': list(destination['pose_xyz']), 'rpy_rad': list(destination['pose_rpy'])},
                destination=copy.deepcopy(destination))
        if not any(destination is item for item in fitting_destinations):
            continue
        for strategy in strategies:
            try:
                effective_intent = _effective_intent_for_strategy(normalized, strategy)
                effective_grasp = effective_intent["pick"]["grasp"]
            except (ValueError, KeyError, TypeError) as exc:
                return _readiness(result, "BLOCKED", "GRASP_CANDIDATE_INVALID", str(exc))
            for observation in eligible:
                try:
                    candidates = generate_strategy_candidates(
                        strategy, observation, _candidate_constraints(strategy, effective_grasp))
                except (ValueError, KeyError, TypeError) as exc:
                    return _readiness(result, "BLOCKED", "GRASP_CANDIDATE_INVALID", str(exc))
                route = None if routes is None else routes[observation["class_id"]]
                for target in ([destination] if route is None else routed[observation["id"]]):
                    for candidate in candidates:
                        evaluation = cycle_evaluator({
                            "intent": copy.deepcopy(effective_intent), "cell": copy.deepcopy(cell),
                            "strategy_ref": strategy, "observation": copy.deepcopy(observation),
                            "candidate": candidate, "destination": copy.deepcopy(target),
                        })
                        # A phased evaluator may defer this candidate without
                        # testing it. Keep control flow out of feasibility evidence.
                        if evaluation.get("deferred") is True:
                            if evaluation.get("stop_search") or grasp["policy"] == "EXACT":
                                return _readiness(result, "BLOCKED", last_failure["reason_code"],
                                                  last_failure["reason"])
                            continue
                        checks = copy.deepcopy(evaluation.get("checks", []))
                        success = evaluation.get("success") is True and not any(
                            check.get("status") in ("FAIL", "BLOCKED") for check in checks)
                        grasp_resolution["attempts"].append({
                            "strategy_ref": strategy, "object_id": observation["id"],
                            "candidate_id": candidate.candidate_id, "success": success,
                            **({"route_rule_id": route["rule_id"], "destination_id": target["id"]} if route else {}),
                            "checks": checks,
                            "reason_code": evaluation.get("reason_code"),
                            "reason": evaluation.get("reason"),
                            "extraction_attempts": copy.deepcopy(evaluation.get("extraction_attempts", [])),
                        })
                        result["readiness"]["checks"] = checks
                        if evaluation.get("stop_search"):
                            return _readiness(
                                result, "BLOCKED",
                                evaluation.get("reason_code") or "SEARCH_BUDGET_EXHAUSTED",
                                evaluation.get("reason") or "Candidate search budget exhausted.")
                        if success:
                            if route is not None:
                                _record_route(place_resolution, route, target)
                            grasp_resolution.update(selected_strategy_ref=strategy,
                                                    selected_object_id=observation["id"],
                                                    selected_candidate_id=candidate.candidate_id,
                                                    effective_grasp=copy.deepcopy(effective_grasp))
                            if evaluation.get('extraction_intent') is not None:
                                grasp_resolution['extraction_intent'] = copy.deepcopy(evaluation['extraction_intent'])
                            if evaluation.get('approach_ik') is not None:
                                grasp_resolution['approach_ik'] = copy.deepcopy(evaluation['approach_ik'])
                            if evaluation.get('transfer_ik_seed') is not None:
                                grasp_resolution['transfer_ik_seed'] = copy.deepcopy(evaluation['transfer_ik_seed'])
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
    from asset_mounting import validate_physical
    validate_physical(document.get('environment', document))
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
    # Class routing: every rule's validated destination, so each rule names a destination of the recipe.
    targets = list({route['region_ref']: {'id': route['region_ref'], 'target_ref': route['asset_ref'], 'frame_id': 'world',
                                          'pose_xyz': route['world_pose']['xyz_m'], 'pose_rpy': route['world_pose']['rpy_rad']}
                    for route in result['place_resolution'].get('routes') or []}.values()) or [
        {'id': destination['id'], 'target_ref': destination['target_id'], 'frame_id': destination['frame_id'],
         'pose_xyz': destination['pose_xyz'], 'pose_rpy': destination['pose_rpy']}]
    return {
        'schema_version': 'task_recipe/v1',
        'enabled': True, 'recipe_id': intent['task']['id'], 'id': intent['task']['id'],
        'task_type': intent['task']['type'], 'type': intent['task']['type'],
        'pick': {'source': selection['source_ref'], 'object_source': selection['source_type'],
                 'pick_zone': selection['zone_ref'], 'allowed_grasp_methods': ['finger'],
                 'grasp_strategy': {'strategy_ref': result['grasp_resolution']['selected_strategy_ref']}},
        'decision_rules': copy.deepcopy(rules),
        'destinations': [{'id': t['id'], 'target_ref': t['target_ref'], 'frame_id': 'world', 'pose_xyz': t['pose_xyz'],
                          'pose_rpy': t['pose_rpy'], 'action': 'place'} for t in targets],
        'expected': {'allow_fallback_rule': False, 'require_destination_pose': True},
        'normalized_intent_sha256': result['normalized_intent_sha256'],
        'resolution_sha256': result['resolution_sha256'],
        'task': {**copy.deepcopy(intent['task']),
                 'object_source': selection['source_type'],
                 'source_object': selection['source_ref'],
                 'object_filter': copy.deepcopy(selection['object_filter']),
                 'destinations': [{'id': t['id'], 'target_ref': t['target_ref'], 'frame': t['frame_id'],
                                   'pose_xyz': t['pose_xyz'], 'pose_rpy': t['pose_rpy']} for t in targets],
                 'decision_rules': copy.deepcopy(rules)},
        'grasp': {'strategy_ref': result['grasp_resolution']['selected_strategy_ref']},
        'task_intent_resolution': copy.deepcopy(result),
    }
