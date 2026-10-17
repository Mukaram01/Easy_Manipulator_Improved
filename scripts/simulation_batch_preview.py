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

from physical_destination import place_options, resolve_destination
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
    intent = report["task_intent"]
    path, recording, source_sha = _load_explicit_replay(scene, intent)
    # Single explicit replay acquisition; NOT a claim of sensor freshness.
    normalized = normalize(replay_snapshot(recording, when), when, geometry)
    env_doc = yaml.safe_load((scene / "environment.yaml").read_text(encoding="utf-8"))
    if not isinstance(env_doc, dict):
        raise ValueError("ENVIRONMENT_INVALID: physical environment must be a mapping")
    environment = env_doc.get("environment", env_doc)
    if not isinstance(environment, dict):
        raise ValueError("ENVIRONMENT_INVALID: physical environment must be a mapping")
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
    options_by_size: dict[tuple, list] = {}

    def choices(item: dict) -> list[dict]:
        """Places the operator may pick for this object in the empty region (geometry only, not planned)."""
        key = tuple(item["dimensions"])
        if destination is None:
            return []
        if key not in options_by_size:
            slots = place_options(destination, item["dimensions"], clearance)
            options_by_size[key] = [{
                "id": f"place-{k}", "local_pose": slot,
                "label": (f"Place {k} of {len(slots)} (x {slot['xyz_m'][0]:+.3f}, y {slot['xyz_m'][1]:+.3f} m)"),
            } for k, slot in enumerate(slots, 1)]
        return options_by_size[key]

    queue = [{
        "order": i + 1, "object_id": item["id"], "class_id": item["class_id"],
        "status": "ELIGIBLE_NOT_PLANNED",
        "destination_region_ref": intent["place"]["target"]["region_ref"],
        "grasp_strategy_policy": intent["pick"]["grasp"]["policy"],
        "place_options": choices(item),
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
