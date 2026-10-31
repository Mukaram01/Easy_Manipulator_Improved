#!/usr/bin/env python3
"""Read-only operator projection of a Workcell Builder scene.

The Operator HMI consumes this projection; it never edits the scene. Every
field is copied or derived from existing contracts:

* task intent through the Builder's own authoring adapter
  (``task_intent_authoring.load_authoring``), saved or an unsaved draft;
* task resolution through ``task_intent_resolver.scene_resolution``;
* the last Validate / offline-validation reports the Builder wrote;
* the bound task input (commissioning replay) through the runtime
  normalizer and the resolver's own selection rules;
* bundle approval from ``generated/generated_workcell_summary.json``.

Nothing is written. Missing evidence is reported as such, never filled in.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path
from typing import Any

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

SCHEMA = "workcell_operator_preview/v1"
INTENT_FILE = "config/workcell_builder_task_intent.yaml"
EXECUTION_UNAVAILABLE = (
    "No qualified execution backend is connected to the Operator HMI. "
    "Physical robot motion is not authorised from this preview.")


def _sha256_file(path: Path) -> str | None:
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        return None


def _mtime(path: Path) -> float | None:
    try:
        return path.stat().st_mtime
    except OSError:
        return None


def _load_mapping(path: Path) -> dict[str, Any] | None:
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return None
    try:
        data = json.loads(text) if path.suffix == ".json" else yaml.safe_load(text)
    except (ValueError, yaml.YAMLError):
        return None
    return data if isinstance(data, dict) else None


def _event(events: list, level: str, code: str, message: str, **detail: Any) -> None:
    events.append({"level": level, "code": code, "message": message,
                   "detail": {k: v for k, v in detail.items() if v is not None}})


def _cell(scene: Path, environment: dict[str, Any], document: dict[str, Any] | None = None) -> dict[str, Any]:
    """Robot, tool, camera and perception as the saved cell declares them: the scene manifest first, then the
    environment document (Create New Cell keeps its tool and camera at the top level of environment.yaml)."""
    from task_intent_v2 import TOOL_CAPABILITY_MAP
    manifest = _load_mapping(scene / "scene_manifest.yaml") or {}
    document = document or {}
    robot = manifest.get("robot") or {}
    tool = manifest.get("end_effector") or {}
    env_tool = (environment.get("end_effector") or environment.get("tool") or document.get("end_effector")
                or document.get("tool") or {})
    tool_id = (env_tool.get("id") or env_tool.get("name")) if isinstance(env_tool, dict) else env_tool
    tool_id = tool_id or tool.get("model")
    camera = manifest.get("camera") or (document.get("camera") if isinstance(document.get("camera"), dict) else {})
    perception = manifest.get("perception") or {}
    return {
        "robot": {"model": robot.get("model"), "planning_group": robot.get("planning_group")},
        "end_effector": {
            "model": tool.get("model") or tool_id,
            "type": tool.get("type") or (env_tool.get("type") if isinstance(env_tool, dict) else None),
            "capabilities": sorted(TOOL_CAPABILITY_MAP.get(str(tool_id), set())),
        },
        "camera": {"id": camera.get("camera_id"), "enabled": camera.get("enabled")},
        "perception": {"mode": perception.get("mode"),
                       "contract": perception.get("normalized_output_contract")},
    }


def _session_observed(intent: dict[str, Any]) -> bool:
    """A sorting session observes the cell itself each cycle (rendered frames classified by EPD)."""
    from task_intent_v2 import freshness_source_type, session_rendered_perception
    return (freshness_source_type(intent) == "rendered_perception_in_simulation"
            or session_rendered_perception(intent))


def selection_ref(intent: dict[str, Any]) -> str | None:
    return ((intent.get("pick") or {}).get("selection") or {}).get("source_ref")


def _zone_geometry(environment: dict[str, Any], zone_id: str | None) -> dict[str, Any] | None:
    for zone in environment.get("task_zones", []) or []:
        if zone.get("id") == zone_id and zone.get("pose_xyz") and zone.get("dimensions"):
            return {"id": zone_id, "type": zone.get("type"), "target_ref": zone.get("target_ref"),
                    "pose_xyz": zone.get("pose_xyz"), "pose_rpy": zone.get("pose_rpy", [0, 0, 0]),
                    "dimensions": zone.get("dimensions")}
    return None


def _reports(scene: Path) -> dict[str, Any]:
    reports: dict[str, Any] = {}
    readiness = scene / "validation/readiness_report.json"
    data = _load_mapping(readiness)
    reports["validate_generated_scene"] = None if data is None else {
        "path": str(readiness), "modified": _mtime(readiness), "status": data.get("status"),
        "readiness": data.get("readiness"), "runtime_readiness": data.get("runtime_readiness"),
        "warnings": data.get("warnings", []), "errors": data.get("errors", []),
    }
    acceptance = scene / "acceptance/generated_scene_acceptance.json"
    data = _load_mapping(acceptance)
    if data is None:
        reports["offline_validation"] = None
    else:
        current = None
        recorded = data.get("authored_input_fingerprint")
        if recorded:
            try:
                from validate_workcell_studio_generated_scene import authored_input_fingerprint
                current = recorded == authored_input_fingerprint(scene)
            except Exception:  # noqa: BLE001 - currency is optional evidence
                current = None
        reports["offline_validation"] = {
            "path": str(acceptance), "modified": _mtime(acceptance), "status": data.get("status"),
            "blockers": data.get("blockers", []), "warnings": data.get("warnings", []),
            "current": current,
        }
    return reports


def _approval(scene: Path, draft: bool) -> dict[str, Any]:
    summary = scene / "generated/generated_workcell_summary.json"
    data = _load_mapping(summary)
    approval = (data or {}).get("approval") if data else None
    out = {"bundle_summary": str(summary) if data is not None else None,
           "status": "NOT_APPROVED", "approved_by": None, "approved_at": None}
    if isinstance(approval, dict) and approval.get("status") == "approved":
        out.update(status="APPROVED", approved_by=approval.get("approved_by"),
                   approved_at=approval.get("approved_at"))
    if draft:
        out["draft_note"] = "Unsaved draft edits are never approved."
    return out


def _evaluate_input(scene: Path, intent: dict[str, Any], environment: dict[str, Any],
                    planning_input: dict[str, Any], now: float) -> dict[str, Any]:
    """Run the bound replay through the resolver's own selection rules."""
    import perceived_object_grasp_plan as geometry
    from runtime_pick_inputs import normalize, replay_snapshot
    from task_intent_resolver import explain_observations
    path = Path(planning_input["path"])
    snapshot = yaml.safe_load(path.read_text(encoding="utf-8"))
    observations = normalize(replay_snapshot(snapshot, now), now, geometry)
    decisions = explain_observations(intent, environment, observations, now)
    by_id = {item["id"]: item for item in observations}
    for decision in decisions:
        item = by_id.get(decision["id"], {})
        decision["pose_xyz"] = (item.get("pose") or [None] * 3)[:3]
        decision["dimensions"] = item.get("dimensions")
    accepted = [d for d in decisions if d["accepted"]]
    return {"source": str(path), "mode": (snapshot.get("source") or {}).get("mode"),
            "description": (snapshot.get("source") or {}).get("description"),
            "evaluated_at": now, "objects": decisions,
            "selected_object": accepted[0]["object_id"] if accepted else None}


def project(scene: Path, draft: dict[str, Any] | None = None, *, evaluate_input: bool = False,
            now: float | None = None) -> dict[str, Any]:
    from task_intent_authoring import load_authoring
    from workcell_roles import ROLE_OPERATOR, list_capabilities
    scene = Path(scene).expanduser().resolve()
    now = time.time() if now is None else now
    events: list[dict[str, Any]] = []
    intent_path = scene / INTENT_FILE
    out: dict[str, Any] = {
        "schema": SCHEMA, "generated_at": now, "read_only": True,
        "scene": {"name": scene.name, "path": str(scene)},
        "revision": {"kind": "draft" if draft is not None else "saved",
                     "intent_file": str(intent_path), "intent_file_exists": intent_path.is_file(),
                     "intent_file_sha256": _sha256_file(intent_path),
                     "intent_file_modified": _mtime(intent_path),
                     "intent_file_schema": None, "normalized_intent_sha256": None,
                     "saved_normalized_intent_sha256": None, "draft_differs_from_saved": None},
        "cell": {}, "task": None, "validation": None, "reports": _reports(scene),
        "resolution": None, "input": None, "observations": None,
        "zones": {}, "approval": _approval(scene, draft is not None),
        "operator_capabilities": list_capabilities(ROLE_OPERATOR),
        "execution": {"backend": "UNAVAILABLE", "motion_authorised": False,
                      "reason": EXECUTION_UNAVAILABLE,
                      "simulation": "Fake-hardware simulation runs from Workcell Builder "
                                    "(Plan / Simulate); it is not started from this HMI.",
                      "live": "Live operation is not supported in this release."},
        "generated_bundle": {"summary": None, "dry_run_available": False,
                             "reason": "Generate the scene package in Workcell Builder first."},
        "events": events, "errors": [],
    }
    if not scene.is_dir() or not (scene / "environment.yaml").is_file():
        out["errors"].append(f"SCENE_MISSING: {scene} is not a Workcell Builder scene "
                             "(environment.yaml not found).")
        _event(events, "error", "SCENE_MISSING", out["errors"][-1])
        return out

    document = _load_mapping(scene / "environment.yaml") or {}
    environment = document.get("environment", document)
    out["cell"] = _cell(scene, environment, document)
    raw = _load_mapping(intent_path) if intent_path.is_file() else None
    out["revision"]["intent_file_schema"] = (raw or {}).get("schema")
    summary = scene / "generated/generated_workcell_summary.json"
    if summary.is_file():
        out["generated_bundle"] = {"summary": str(summary), "dry_run_available": True, "reason": ""}

    try:
        report = load_authoring(scene, draft)
    except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError) as exc:
        out["errors"].append(f"TASK_INTENT_UNREADABLE: {exc}")
        _event(events, "error", "TASK_INTENT_UNREADABLE", str(exc))
        return out
    intent = report.get("task_intent")
    out["validation"] = {key: report.get(key) for key in (
        "status", "errors", "warnings", "readiness_classification",
        "missing_required_fields", "suggested_next_actions")}
    out["revision"]["normalized_intent_sha256"] = report.get("normalized_intent_sha256")
    if draft is not None:
        try:
            saved = load_authoring(scene)
            saved_hash = saved.get("normalized_intent_sha256")
        except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError):
            saved_hash = None
        out["revision"]["saved_normalized_intent_sha256"] = saved_hash
        out["revision"]["draft_differs_from_saved"] = (
            None if saved_hash is None else saved_hash != report.get("normalized_intent_sha256"))
    else:
        out["revision"]["saved_normalized_intent_sha256"] = report.get("normalized_intent_sha256")

    if not intent_path.is_file() and draft is None:
        _event(events, "warning", "TASK_INTENT_NOT_SAVED",
               "No saved task intent; showing the Builder's proposed default. Save the task in Builder.")
    session_observed = bool(intent) and _session_observed(intent)
    if intent:
        binding = intent.get("perception") or {}
        if binding.get("mode"):
            out["cell"]["perception"] = {
                "mode": binding.get("mode"), "camera_source": binding.get("camera_source"),
                "contract": out["cell"]["perception"].get("contract") or selection_ref(intent),
                "producer": "simulator_render" if session_observed else None,
                "epd_runtime_registered": (SCRIPT_DIR.parents[2] / "workcell_perception_runtime.json").is_file()}
        selection = intent.get("pick", {}).get("selection", {})
        filters = selection.get("object_filter", {}) or {}
        place = intent.get("place", {})
        rules = (intent.get("routing") or {}).get("rules") or []
        destination_ids = sorted({r.get("destination") for r in rules if r.get("destination")}
                                 | ({place.get("target", {}).get("region_ref")} - {None, ""}))
        out["task"] = {
            "id": intent.get("task", {}).get("id"), "type": intent.get("task", {}).get("type"),
            "template": intent.get("task", {}).get("template"),
            "target": {"class_id": filters.get("class_id"), "color": filters.get("color"),
                       "min_confidence": filters.get("min_confidence"),
                       "max_age_seconds": filters.get("max_age_seconds"),
                       "pick_zone": selection.get("zone_ref"),
                       "source_type": selection.get("source_type"),
                       "source_ref": selection.get("source_ref")},
            # TaskIntent v2 has no explicit ignore list: everything the pick
            # filter does not select is left in place.
            "not_targeted": {"explicit_list": None,
                             "rule": ("Objects whose class has no routing rule are not picked."
                                      if (intent.get("routing") or {}).get("mode") == "class" else
                                      "Objects that do not match the pick filter are not picked.")},
            "destinations": [{"id": d, "target_ref": (report.get("regions") or {}).get(d)}
                             for d in destination_ids],
            "place_target": place.get("target"),
            "routing_rules": rules,
            "resolved_destination": report.get("resolved_destination"),
            "grasp": {k: intent.get("pick", {}).get("grasp", {}).get(k)
                      for k in ("policy", "strategy_ref", "required_capability")},
            "safety": intent.get("safety"),
        }
        out["zones"] = {"pick": _zone_geometry(environment, selection.get("zone_ref")),
                        "place": _zone_geometry(environment, place.get("target", {}).get("region_ref"))}
        _event(events, "info", "TASK_SELECTED",
               f"Task {out['task']['id']} ({out['task']['type']}) from the "
               f"{'unsaved draft' if draft is not None else 'saved task intent'}.",
               target_class=filters.get("class_id"), pick_zone=selection.get("zone_ref"))
        if rules and (intent.get("routing") or {}).get("mode") == "class":
            _event(events, "info", "ROUTING_SELECTS",
                   "The routing table decides which classes are picked and where each goes.",
                   rules=len(rules))
        elif not filters.get("class_id"):
            _event(events, "warning", "TARGET_CLASS_UNSET",
                   "No target class: any object in the pick zone matching the other filters is eligible.")
        if report.get("resolved_destination"):
            _event(events, "info", "DESTINATION_RESOLVED",
                   f"Destination {place.get('target', {}).get('region_ref')} resolved in the saved scene.")
        else:
            _event(events, "error", "DESTINATION_MISSING",
                   "The task destination does not resolve to a physical place zone in environment.yaml.")
    status = (out["validation"] or {}).get("status")
    if status == "PASS":
        _event(events, "info", "TASK_VALIDATION_PASS", "Task intent is valid for offline use.",
               readiness=report.get("readiness_classification"))
    else:
        _event(events, "error", "TASK_VALIDATION_FAIL",
               "; ".join(report.get("errors") or ["Task intent is not valid."]))

    if intent:
        try:
            from task_intent_resolver import scene_resolution
            resolution = scene_resolution(scene, intent, environment, document)
            readiness = resolution.get("readiness", {})
            out["resolution"] = {"status": resolution.get("readiness_status"),
                                 "primary_code": readiness.get("primary_code"),
                                 "reason": readiness.get("reason"),
                                 "diagnostics": resolution.get("diagnostics", [])}
        except Exception as exc:  # noqa: BLE001 - corrupt evidence is reported, not hidden
            out["resolution"] = {"status": "BLOCKED", "primary_code": "RESOLUTION_REJECTED",
                                 "reason": str(exc), "diagnostics": []}
        if session_observed and out["resolution"]["status"] not in ("READY", "WARNING"):
            # The sorting session resolves each selected object in its own cycle from that cycle's frame.
            out["resolution"] = {"status": "PER_CYCLE", "primary_code": "SESSION_RESOLVED_PER_CYCLE",
                                 "reason": "The sorting session plans each selected object in its own cycle, from that "
                                           "cycle's rendered and EPD-classified frame.",
                                 "diagnostics": out["resolution"]["diagnostics"]}
        level = "info" if out["resolution"]["status"] in ("READY", "WARNING", "PER_CYCLE") else "warning"
        _event(events, level, "TASK_RESOLUTION_" + str(out["resolution"]["status"]),
               out["resolution"]["reason"] or "Task resolution status.",
               primary_code=out["resolution"]["primary_code"])

    planning_input = report.get("planning_input")
    if session_observed:
        out["input"] = {"kind": "session", "path": None, "blocker": None, "producer": "simulator_render"}
        _event(events, "info", "INPUT_SESSION_RENDERED",
               "Task input: each sorting cycle renders the cell with its camera and classifies the frame with EPD "
               "(simulator-rendered, synthetic). There is no saved input to replay.")
    elif planning_input:
        kind = "replay" if planning_input.get("replay") else "live"
        out["input"] = {"kind": kind, "path": planning_input.get("path"), "blocker": None}
        _event(events, "info", "INPUT_REPLAY_BOUND" if kind == "replay" else "INPUT_LIVE_BOUND",
               "Task input: recorded commissioning replay." if kind == "replay"
               else "Task input: live normalized perception (not available in preview).",
               path=planning_input.get("path"))
    else:
        out["input"] = {"kind": "unavailable", "path": None,
                        "blocker": report.get("planning_input_blocker")}
        _event(events, "error", "PERCEPTION_INPUT_UNAVAILABLE",
               report.get("planning_input_blocker") or "No task input is available.")

    if evaluate_input and intent and out["input"]["kind"] == "replay":
        try:
            out["observations"] = _evaluate_input(scene, intent, environment, planning_input, now)
        except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError) as exc:
            out["observations"] = {"error": str(exc), "objects": []}
            _event(events, "error", "REPLAY_CHECK_FAILED", str(exc))
        else:
            for decision in out["observations"]["objects"]:
                _event(events, "info" if decision["accepted"] else "notice",
                       "OBJECT_ACCEPTED" if decision["accepted"] else "OBJECT_REJECTED",
                       f"{decision['object_id']} ({decision['class_id']}): {decision['reason']}",
                       reason_code=decision["reason_code"])
    elif evaluate_input and out["input"]["kind"] == "live":
        _event(events, "warning", "LIVE_INPUT_NOT_EVALUATED",
               "Live perception is not consumed by the preview; no objects were evaluated.")

    _event(events, "warning", "EXECUTION_BACKEND_UNAVAILABLE", EXECUTION_UNAVAILABLE)
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scene", required=True, type=Path)
    parser.add_argument("--draft-stdin", action="store_true",
                        help="Read an unsaved v2 task-intent draft (JSON) from stdin.")
    parser.add_argument("--evaluate-input", action="store_true",
                        help="Evaluate the bound replay input with the task selection rules.")
    args = parser.parse_args(argv)
    draft = None
    if args.draft_stdin:
        try:
            draft = json.load(sys.stdin)
        except ValueError as exc:
            print(json.dumps({"schema": SCHEMA, "errors": [f"DRAFT_INVALID: {exc}"]}))
            return 2
    print(json.dumps(project(args.scene, draft, evaluate_input=args.evaluate_input), default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
