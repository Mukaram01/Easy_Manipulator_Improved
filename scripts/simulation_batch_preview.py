#!/usr/bin/env python3
"""No-EPD multi-object commissioning task preview for the existing Workcell Studio Task.

This is an *eligibility queue*, NOT MoveIt planning, robot execution or
live simulator physics. Only explicit, user-selected, fixed commissioning
observations are accepted. Missing/non-world geometry fails closed.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from physical_destination import (PLACEMENT_TOLERANCE_M, check_object_containment, local_to_world_pose,
                                  place_conflicts, place_options, placement_slots, resolve_destination,
                                  resolve_local_destination, wall_margin_ok)
from task_intent_authoring import load_authoring
from task_intent_resolver import select_observations
from task_intent_v2 import normalized_intent_hash
from runtime_pick_inputs import contained, normalize, replay_snapshot, zone
import perceived_object_grasp_plan as geometry


def _load_explicit_replay(scene: Path, intent: dict) -> tuple[Path, dict, str]:
    binding = intent.get("perception")
    if not isinstance(binding, dict) or binding.get("mode") not in ("off", "replayed_snapshot"):
        raise ValueError("SIMULATION_SOURCE_REQUIRED: select Off/commissioning or Recorded observations in Task")
    selected = intent["pick"]["selection"]
    mode = binding["mode"]
    if mode == "off":
        if selected.get("source_type") != "manual_simulated" or selected.get("source_ref") != selected.get("zone_ref"):
            raise ValueError("PICK_SOURCE_MODE_MISMATCH: commissioning source must equal the saved pick zone")
    elif selected.get("source_type") != "perception" or selected.get("source_ref") != "detected_objects/v1":
        raise ValueError("PICK_SOURCE_MODE_MISMATCH: recording must use the normalized observation contract")

    name = binding.get("replay_path")
    if not isinstance(name, str) or not name.strip():
        raise ValueError("EXPLICIT_SIMULATION_INPUT_REQUIRED: Choose an existing commissioning observation file in Task")
    path = Path(name).expanduser()
    if not path.is_absolute():
        path = scene / path
    path = path.resolve(strict=True)
    raw = path.read_bytes()
    document = yaml.safe_load(raw)
    if not isinstance(document, dict) or document.get("schema_version") != "detected_objects/v1":
        raise ValueError("SIMULATION_INPUT_INVALID: require detected_objects/v1")
    source = document.get("source")
    if not isinstance(source, dict) or source.get("mode") != "replayed_snapshot":
        raise ValueError("REPLAY_INPUT_REQUIRED: only explicit immutable commissioning recordings can be previewed")
    if not isinstance(document.get("objects"), list) or not document["objects"]:
        raise ValueError("SIMULATION_INPUT_INVALID: recorded objects are missing")
    return path, document, hashlib.sha256(raw).hexdigest()


def _reject_reason(item: dict, intent: dict, region: dict, now: float) -> str:
    filt = intent["pick"]["selection"]["object_filter"]
    if any(filt.get(key) not in (None, "") and item.get(key) != filt[key]
           for key in ("class_id", "color")):
        return "CLASS_OR_COLOUR_MISMATCH"
    conf = item.get("confidence")
    minimum = filt.get("min_confidence")
    if minimum is not None:
        if conf is None:
            return "CONFIDENCE_MISSING"
        if conf < minimum:
            return "LOW_CONFIDENCE"
    age = now - item["timestamp"]
    if age < 0 or age > (filt.get("max_age_seconds") or math.inf):
        return "STALE_OR_FUTURE_OBSERVATION"
    if not contained(item, region, geometry):
        return "OUTSIDE_PICK_VOLUME"
    return "NOT_ELIGIBLE_UNDER_SAVED_TASK"


def preview_batch(scene: Path, *, limit: int = 20, now: float | None = None) -> dict[str, Any]:
    """Build deterministic inspection-only object queue from saved task and replay.

    Reuses exactly the runtime normalizer, TaskIntent v2 validator, shared
    resolver selection, oriented-zone containment and destination authority.
    No candidate is ever labelled collision/reachability/planning qualified.
    """
    if isinstance(limit, bool) or not isinstance(limit, int) or not 1 <= limit <= 100:
        raise ValueError("QUEUE_LIMIT_INVALID: enter an integer from 1 to 100")
    scene = Path(scene).resolve(strict=True)
    when = time.time() if now is None else now
    if isinstance(when, bool) or not isinstance(when, (int, float)) or not math.isfinite(when):
        raise ValueError("TIME_INVALID: finite observation reference time required")
    task_path = scene / "config/workcell_builder_task_intent.yaml"
    if not task_path.is_file():
        raise ValueError("TASK_INTENT_MISSING: save the Task in Builder first")
    # Reject unsafe or live/EPD modes *before* evaluating unrelated authoring
    # errors, so a user receives the true first safety/authority blocker.
    authored = yaml.safe_load(task_path.read_text(encoding="utf-8"))
    if not isinstance(authored, dict):
        raise ValueError("TASK_INVALID: saved TaskIntent must be a mapping")
    binding = authored.get("perception")
    if not isinstance(binding, dict) or binding.get("mode") not in ("off", "replayed_snapshot"):
        raise ValueError("SIMULATION_SOURCE_REQUIRED: select a saved Off/commissioning or Recorded observation mode")
    safety = authored.get("safety")
    if not isinstance(safety, dict) or safety.get("real_hardware_enabled") is not False or safety.get("require_fake_hardware") is not True:
        raise ValueError("FAKE_HARDWARE_REQUIRED: commissioning preview requires the real robot locked")
    report = load_authoring(scene)
    if report.get("status") == "FAIL" or not isinstance(report.get("task_intent"), dict):
        raise ValueError("TASK_INVALID: " + "; ".join(map(str, report.get("errors", []))))
    result = _queue(scene, report["task_intent"], limit, when)
    result["sequence_review"] = review_sequence(scene, report["task_intent"], preview=result)
    return result


def _environment(scene: Path) -> dict:
    env_doc = yaml.safe_load((scene / "environment.yaml").read_text(encoding="utf-8"))
    if not isinstance(env_doc, dict):
        raise ValueError("ENVIRONMENT_INVALID: physical environment must be a mapping")
    environment = env_doc.get("environment", env_doc)
    if not isinstance(environment, dict):
        raise ValueError("ENVIRONMENT_INVALID: physical environment must be a mapping")
    return env_doc, environment


def _place_choices(destination: dict | None, dimensions: list, clearance: float, robot_xyz) -> list[dict]:
    """Places the operator may pick for one object in the empty region (geometry only, not planned)."""
    if destination is None:
        return []
    places = place_options(destination, dimensions, clearance, robot_xyz)
    return [{"id": f"place-{k}", "local_pose": pose, "description": where,
             "label": f"Place {k} of {len(places)} — {where}"} for k, (pose, where) in enumerate(places, 1)]


def _queue(scene: Path, intent: dict, limit: int, when: float) -> dict[str, Any]:
    """The eligibility queue for a validated intent: shared by Preview and the sequence review."""
    path, recording, source_sha = _load_explicit_replay(scene, intent)
    # Single explicit replay acquisition; NOT a claim of sensor freshness.
    normalized = normalize(replay_snapshot(recording, when), when, geometry)
    env_doc, environment = _environment(scene)
    region = zone({"environment": environment}, intent["pick"]["selection"]["zone_ref"])
    eligible = select_observations(intent, environment, normalized, when)
    eligible_ids = {item["id"] for item in eligible}
    if len(eligible_ids) != len(eligible):
        raise ValueError("OBJECT_ID_AMBIGUOUS: duplicate normalized object identity")
    selected = eligible[:limit]
    try:
        destination = resolve_destination(environment, intent["place"]["target"]["region_ref"])
    except ValueError:
        destination = None  # Generate/plan report the destination blocker; the queue stays inspectable.
    clearance = intent["place"]["placement"]["clearance_m"]
    robot_xyz = (env_doc.get("robot") or {}).get("pose_xyz")

    queue = [{
        "order": i + 1, "object_id": item["id"], "class_id": item["class_id"],
        "status": "ELIGIBLE_NOT_PLANNED",
        "destination_region_ref": intent["place"]["target"]["region_ref"],
        "grasp_strategy_policy": intent["pick"]["grasp"]["policy"],
        "place_options": _place_choices(destination, item["dimensions"], clearance, robot_xyz),
    } for i, item in enumerate(selected)]
    # All detected/non-target objects remain reported as scene obstacles;
    # none is silently discarded or claimed to be verified in MoveIt.
    objects = [{
        "object_id": item["id"], "class_id": item["class_id"],
        "dimensions_m": item["dimensions"], "world_pose": item["pose"],
        "role": "PICK_CANDIDATE" if item["id"] in eligible_ids else "NON_TARGET_OBSTACLE",
        "reason": ("MATCHES_SAVED_PICK_POLICY" if item["id"] in eligible_ids
                   else _reject_reason(item, intent, region, when)),
        "planning_scene_verified": False,
    } for item in normalized]
    objects.sort(key=lambda x: x["object_id"])
    return {
        "schema_version": "workcell_simulation_batch_preview/v1",
        "status": "ELIGIBILITY_ONLY" if queue else "BLOCKED",
        "reason": ("" if queue else "NO_ELIGIBLE_TARGETS: no object matches the saved task"),
        "source_mode": "explicit_commissioning_replay",
        "source_path": str(path), "source_sha256": source_sha,
        "task_intent_sha256": normalized_intent_hash(intent),
        "task_id": intent.get("task", {}).get("id"),
        "pick_zone_ref": region["id"],
        "destination_region_ref": intent["place"]["target"]["region_ref"],
        "total_observed": len(normalized), "total_eligible": len(eligible),
        "queued": len(queue), "not_queued_due_to_limit": len(eligible) - len(queue),
        "queue": queue, "observed_objects": objects,
        "simulator_live_connected": False, "epd_used": False,
        "moveit_planning_attempted": False, "motion_commanded": False,
        "physical_grasp_verified": False,
        "note": "Explicit commissioning recording only. Eligible is not reachable/collision-free. "
                "All objects must be kept as obstacles for later MoveIt plan-only validation.",
    }


def review_sequence(scene: Path, intent: dict, *, preview: dict | None = None,
                    now: float | None = None) -> dict[str, Any]:
    """Is the saved pick sequence still the one the operator chose? Geometry only, never planned.

    Checks, against the current scene and recording: the recording is byte-identical
    (source_sha256), every saved object is still observed, unchanged and eligible,
    MANUAL places are in the task's destination, fit it and do not overlap, and the
    AUTO objects fit around them (manual places reserved first). Nothing is ever
    re-bound or re-placed: a problem is reported as NEEDS_REVIEW (choose again) or
    BLOCKED (the input changed or the destination is full). Planning uses the
    sequence only when the status is READY.
    """
    sequence = intent.get("sequence")
    if sequence is None:
        return {"status": "NOT_CONFIGURED", "items": [], "order": [], "placements": {},
                "summary": "No saved pick sequence: every eligible object, in queue order, with automatic places."}
    scene = Path(scene)
    saved = sequence["items"]
    items = [{"position": i, "object_id": item["object_id"], "class_id": item["class_id"],
              "mode": item["placement"]["mode"], "local_pose": item["placement"].get("local_pose"),
              "dimensions_m": item["dimensions_m"], "observed_xyz_m": item["observed_xyz_m"],
              "status": "READY", "message": "", "place_options": [], "place_label": "Automatic"}
             for i, item in enumerate(saved, 1)]
    review = {"items": items, "order": [i["object_id"] for i in items],
              "placements": {i["object_id"]: i["local_pose"] for i in items if i["mode"] == "MANUAL"}}

    def flag(item: dict, status: str, message: str) -> None:
        if item["status"] == "READY" or status == "BLOCKED":
            item.update(status=status, message=message)

    try:
        when = time.time() if now is None else now
        preview = preview or _queue(scene, intent, 100, when)
        env_doc, environment = _environment(scene)
    except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError) as exc:
        for item in items:
            flag(item, "BLOCKED", f"cannot check the saved sequence: {exc}")
        return _summarize(review)
    target = intent["place"]["target"]
    clearance = intent["place"]["placement"]["clearance_m"]
    robot_xyz = (env_doc.get("robot") or {}).get("pose_xyz")
    try:
        destination = resolve_destination(environment, target["region_ref"])
    except ValueError as exc:
        for item in items:
            flag(item, "BLOCKED", f"DESTINATION_INVALID: {exc}")
        return _summarize(review)
    for item in items:  # Saved sizes: the places this object could be offered, whatever else changed.
        item["place_options"] = _place_choices(destination, item["dimensions_m"], clearance, robot_xyz)
    for item in items:
        if item["mode"] == "MANUAL":
            match = next((o for o in item["place_options"] if _same_pose(o["local_pose"], item["local_pose"])), None)
            item["place_label"] = match["label"] if match else "Custom place (x {:+.3f}, y {:+.3f} m)".format(
                *item["local_pose"]["xyz_m"][:2])
    if preview["source_sha256"] != sequence["source_sha256"]:
        for item in items:
            flag(item, "BLOCKED", "SEQUENCE_SOURCE_CHANGED: the observation recording changed since this sequence "
                                  "was saved, so its objects may be different detections. Preview the queue and "
                                  "save the sequence again.")
        return _summarize(review)
    observed = {o["object_id"]: o for o in preview["observed_objects"]}
    queued = {q["object_id"] for q in preview["queue"]}
    for item in items:
        obj = observed.get(item["object_id"])
        if obj is None:
            flag(item, "NEEDS_REVIEW", "OBJECT_MISSING: not in the recording any more; remove it from the sequence.")
            continue
        if (obj["class_id"] != item["class_id"] or obj["dimensions_m"] != item["dimensions_m"] or
                math.dist(obj["world_pose"][:3], item["observed_xyz_m"]) > 1e-6):
            flag(item, "BLOCKED", "SEQUENCE_OBJECT_CHANGED: this ID now names a different detection "
                                  "(class, size or position differ from the saved one).")
            continue
        if item["object_id"] not in queued:
            flag(item, "NEEDS_REVIEW", f"OBJECT_NOT_ELIGIBLE: {obj['reason']} under the current task; "
                                       "remove it from the sequence or change the task.")
    # Manual places: in the task's destination, inside it, and apart from each other.
    reserved = []
    for item in items:
        if item["mode"] != "MANUAL":
            continue
        place = saved[item["position"]-1]["placement"]
        if (place["asset_ref"], place["region_ref"]) != (target["asset_ref"], target["region_ref"]):
            flag(item, "NEEDS_REVIEW", f"DESTINATION_CHANGED: this place was chosen in {place['region_ref']}, but "
                                       f"the task now places into {target['region_ref']}. Choose its place again.")
            continue
        try:
            chosen = resolve_local_destination(environment, target["asset_ref"], target["region_ref"],
                                               place["local_pose"])
            world = local_to_world_pose(chosen, place["local_pose"])
            check_object_containment(chosen, world, item["dimensions_m"], clearance)
            if not wall_margin_ok(destination, place["local_pose"], item["dimensions_m"], clearance, PLACEMENT_TOLERANCE_M):
                raise ValueError("closer to the region edge than the planner's placement tolerance allows")
        except ValueError as exc:
            flag(item, "NEEDS_REVIEW", f"PLACE_OUTSIDE_REGION: the chosen place no longer fits {target['region_ref']} "
                                       f"with {clearance:.3f} m clearance and the planner's {PLACEMENT_TOLERANCE_M:.3f} m "
                                       f"placement tolerance ({exc}). Choose its place again.")
            continue
        hits = place_conflicts(destination, place["local_pose"], item["dimensions_m"], clearance,
                               [(pose, dims) for _, pose, dims in reserved], separation=PLACEMENT_TOLERANCE_M)
        if hits:
            other = reserved[hits[0]][0]
            flag(item, "NEEDS_REVIEW", f"PLACE_CONFLICT: overlaps the place chosen for {other}. "
                                       "Choose a different place for one of them.")
            continue
        reserved.append((item["object_id"], world, item["dimensions_m"]))
    # Automatic objects fit around the reserved places, in order; clearance is never relaxed.
    taken = [(pose, dims) for _, pose, dims in reserved]
    exact_used = False
    for item in items:
        if item["mode"] != "AUTO" or item["status"] != "READY":
            continue
        if intent["place"]["placement"]["policy"] == "EXACT":
            # EXACT plans every automatic object at the one authored place (see sequence_place_poses).
            if exact_used:
                flag(item, "NEEDS_REVIEW", "EXACT_PLACE_SHARED: the task's EXACT placement policy sends every "
                                           "Automatic object to the same place. Choose a place for this object, "
                                           "or set Placement policy to AUTO.")
            item["place_label"] = "Automatic — the task's EXACT place"
            exact_used = True
            continue
        slots = placement_slots(destination, item["dimensions_m"], clearance, taken, separation=PLACEMENT_TOLERANCE_M)
        if not slots:
            flag(item, "BLOCKED", f"INSUFFICIENT_CAPACITY: {target['region_ref']} has no room left for this "
                                  f"{item['dimensions_m'][0]:.3f} x {item['dimensions_m'][1]:.3f} m object with "
                                  f"{clearance:.3f} m clearance after {len(taken)} other object(s). Remove an object "
                                  "from the sequence or use a larger destination.")
            continue
        expected = slots[0]
        # Name an offered place only when the expected position is that place (within a grid step).
        same = next((o for o in item["place_options"]
                     if math.dist(o["local_pose"]["xyz_m"], expected["xyz_m"]) < 0.01), None)
        item["place_label"] = f"Automatic (expected: {same['description']})" if same else "Automatic (next free place)"
        item["expected_local_pose"] = expected
        taken.append((local_to_world_pose(destination, expected), item["dimensions_m"]))
    return _summarize(review)


def _same_pose(a: dict, b: dict) -> bool:
    return math.dist(a["xyz_m"], b["xyz_m"]) < 1e-6 and math.dist(a["rpy_rad"], b["rpy_rad"]) < 1e-6


def _summarize(review: dict) -> dict:
    items = review["items"]
    statuses = {i["status"] for i in items}
    review["status"] = "BLOCKED" if "BLOCKED" in statuses else "NEEDS_REVIEW" if "NEEDS_REVIEW" in statuses else "READY"
    manual = sum(1 for i in items if i["mode"] == "MANUAL")
    if review["status"] == "READY":
        review["summary"] = (f"Saved sequence: {len(items)} object(s), {manual} chosen place(s), "
                             f"{len(items) - manual} automatic. Ready for the full-sequence check.")
    else:
        bad = [i for i in items if i["status"] != "READY"]
        review["summary"] = (f"{review['status'].replace('_', ' ')}: {len(bad)} of {len(items)} saved object(s) "
                             f"need attention. {bad[0]['object_id']}: {bad[0]['message']}")
    return review


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--limit", type=int, default=20)
    args = parser.parse_args(argv)
    try:
        result = preview_batch(args.scene, limit=args.limit)
    except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError) as exc:
        result = {"schema_version": "workcell_simulation_batch_preview/v1",
                  "status": "BLOCKED", "reason": str(exc), "queue": [],
                  "moveit_planning_attempted": False, "motion_commanded": False,
                  "epd_used": False}
    print(json.dumps(result, sort_keys=True, allow_nan=False))
    return 0 if result["status"] == "ELIGIBILITY_ONLY" else 1


if __name__ == "__main__":
    raise SystemExit(main())
