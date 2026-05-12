#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, time
from pathlib import Path
from typing import Any


def parse_snapshot_json(payload: str) -> dict[str, Any]:
    try:
        data = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise ValueError(f"malformed detection JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError("snapshot payload must be a JSON object")
    required = ["schema_version", "source", "runtime_mode", "camera", "timestamp_sec", "detections"]
    missing = [k for k in required if k not in data]
    if missing:
        raise ValueError(f"snapshot missing fields: {', '.join(missing)}")
    if not isinstance(data.get("detections"), list):
        raise ValueError("detections must be a list")
    return data


def map_live_snapshot(snapshot: dict[str, Any], environment: dict[str, Any], task_intent: dict[str, Any], min_confidence: float = 0.5) -> dict[str, Any]:
    zones = {z.get("name"): z for z in (environment.get("work_zones") or []) if isinstance(z, dict)}
    flows = environment.get("conveyor_flows") or []
    flow_by_detection = {f.get("detection_zone"): f for f in flows if isinstance(f, dict)}
    mapped = []
    warns: list[str] = []

    camera_names = [c.get("name") for c in (environment.get("camera_placements") or []) if isinstance(c, dict)]
    camera_matched = snapshot.get("camera") in camera_names
    if not camera_matched:
        raise ValueError("snapshot camera does not match scene camera")

    for det in snapshot.get("detections", []):
        if not isinstance(det, dict):
            continue
        conf = float(det.get("confidence", 0.0) or 0.0)
        if conf < min_confidence:
            warns.append("detection confidence below threshold")
            continue
        zone = det.get("zone_hint")
        if zone not in zones:
            warns.append("detection zone hint does not exist")
            continue
        flow = flow_by_detection.get(zone)
        if flow is None:
            warns.append("no conveyor flow for detection zone")
            continue
        speed = float(flow.get("speed_mps", 0.25) or 0.25)
        distance = float(flow.get("estimated_distance_m", 0.5) or 0.5)
        time_to_pick_s = round(distance / max(speed, 1e-3), 3)
        mapped.append({
            "id": det.get("id"),
            "class_label": det.get("class_label"),
            "confidence": conf,
            "camera": snapshot.get("camera"),
            "zone_hint": zone,
            "conveyor_flow": flow.get("name"),
            "time_to_pick_s": time_to_pick_s,
            "pick_ready": time_to_pick_s <= float((task_intent.get("max_time_to_pick_s") or 5.0)),
        })

    pick_ready = any(item.get("pick_ready") for item in mapped)
    return {
        "schema_version": 1,
        "source": "epd_live",
        "runtime_mode": "live_adapter_metadata_only",
        "robot_motion_commanded": False,
        "moveit_plan_service_called": False,
        "gripper_command_sent": False,
        "real_conveyor_commanded": False,
        "camera_matched": camera_matched,
        "detections_mapped": mapped,
        "warnings": warns,
        "task_intent_preview_status": "ready" if mapped else "warn_no_task_intent_preview",
        "pick_ready": pick_ready,
    }


def write_live_preview_artifacts(scene_dir: Path, snapshot: dict[str, Any], mapping: dict[str, Any]) -> dict[str, Path]:
    preview = scene_dir / "preview"
    preview.mkdir(parents=True, exist_ok=True)
    task_preview = {
        "schema_version": 1,
        "source": "live_epd_feed_bridge",
        "runtime_mode": "live_adapter_metadata_only",
        "pick_ready": mapping.get("pick_ready", False),
        "robot_motion_commanded": False,
        "moveit_plan_service_called": False,
        "gripper_command_sent": False,
    }

    outputs = {
        "snapshot_yaml": preview / "live_epd_detection_snapshot.yaml",
        "snapshot_json": preview / "live_epd_detection_snapshot.json",
        "mapping_yaml": preview / "live_epd_detection_mapping.yaml",
        "mapping_json": preview / "live_epd_detection_mapping.json",
        "task_yaml": preview / "live_task_intent_preview.yaml",
        "task_json": preview / "live_task_intent_preview.json",
    }
    outputs["snapshot_json"].write_text(json.dumps(snapshot, indent=2) + "\n", encoding="utf-8")
    outputs["mapping_json"].write_text(json.dumps(mapping, indent=2) + "\n", encoding="utf-8")
    outputs["task_json"].write_text(json.dumps(task_preview, indent=2) + "\n", encoding="utf-8")
    # yaml-compatible JSON for minimal deps
    outputs["snapshot_yaml"].write_text(json.dumps(snapshot, indent=2) + "\n", encoding="utf-8")
    outputs["mapping_yaml"].write_text(json.dumps(mapping, indent=2) + "\n", encoding="utf-8")
    outputs["task_yaml"].write_text(json.dumps(task_preview, indent=2) + "\n", encoding="utf-8")
    return outputs


def build_scene_status(mapping: dict[str, Any], snapshot_ts: float, now: float | None = None) -> dict[str, Any]:
    now_s = now if now is not None else time.time()
    age = max(0.0, now_s - float(snapshot_ts))
    return {
        "Live EPD feed bridge configured": "OK",
        "Live snapshot received": "OK" if snapshot_ts else "WARN",
        "Last EPD snapshot age": "WARN" if age > 5.0 else "INFO",
        "Detections mapped": "OK" if mapping.get("detections_mapped") else "WARN",
        "Pick readiness from live feed": "OK" if mapping.get("pick_ready") else "WARN",
        "No robot motion commanded": "OK",
        "No MoveIt call": "OK",
        "EPD GUI separate": "INFO",
        "live_adapter_metadata_only": True,
        "robot_motion_commanded": False,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Live EPD feed bridge (metadata-only)")
    ap.add_argument("--scene-path", type=Path, required=True)
    ap.add_argument("--snapshot-json", required=True)
    ap.add_argument("--min-confidence", type=float, default=0.5)
    args = ap.parse_args()
    scene = args.scene_path
    env = json.loads((scene / "config" / "environment_layout.yaml").read_text()) if (scene / "config" / "environment_layout.yaml").exists() else {}
    task = json.loads((scene / "config" / "task_intent.yaml").read_text()) if (scene / "config" / "task_intent.yaml").exists() else {}
    snapshot = parse_snapshot_json(args.snapshot_json)
    mapping = map_live_snapshot(snapshot, env, task, min_confidence=args.min_confidence)
    write_live_preview_artifacts(scene, snapshot, mapping)
    print(json.dumps(build_scene_status(mapping, float(snapshot.get("timestamp_sec") or 0.0)), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
