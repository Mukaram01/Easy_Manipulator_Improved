from __future__ import annotations

from typing import Any

DEFAULT_STRATEGY = {
    "strategy_id": "auto",
    "approach_axis": "z_down",
    "orientation_mode": "auto_align",
    "approach_distance_m": 0.1,
    "retreat_distance_m": 0.1,
    "allowed_roll_angles_deg": [0.0],
    "allowed_yaw_angles_deg": [0.0, 180.0],
    "gripper_tcp_offset": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
}

FINGER_STRATEGIES = {"finger_top", "finger_side"}
SUCTION_STRATEGIES = {"suction_top", "suction_side"}
SUPPORTED_STRATEGIES = {"auto", *FINGER_STRATEGIES, *SUCTION_STRATEGIES}


def normalize_grasp_strategy(selected: dict[str, Any] | None, end_effector: dict[str, Any] | None) -> tuple[dict[str, Any], list[str]]:
    payload = dict(DEFAULT_STRATEGY)
    payload.update(selected or {})
    payload["strategy_id"] = str(payload.get("strategy_id") or "auto").strip()
    warnings: list[str] = []

    if payload["strategy_id"] not in SUPPORTED_STRATEGIES:
        warnings.append(f"Unsupported grasp strategy '{payload['strategy_id']}' selected; using metadata WARN mode.")

    ee = end_effector or {}
    family = str(ee.get("family") or ee.get("type") or "").lower()
    capability = str(ee.get("capability_id") or ee.get("capability") or ee.get("name") or "").lower()
    is_finger = ("finger" in family) or ("2f" in capability) or ("robotiq" in capability)
    is_suction = ("suction" in family) or ("vacuum" in family) or ("airpick" in capability)

    sid = payload["strategy_id"]
    if sid in FINGER_STRATEGIES and not is_finger:
        warnings.append(f"Strategy '{sid}' is typically for finger grippers; selected tool may be incompatible.")
    if sid in SUCTION_STRATEGIES and not is_suction:
        warnings.append(f"Strategy '{sid}' is typically for suction tools; selected tool may be incompatible.")
    if sid == "suction_side":
        warnings.append("'suction_side' is currently a placeholder strategy; kept as WARN metadata only.")

    payload["metadata_only"] = True
    payload["runtime_applied"] = False
    payload["compatibility_status"] = "WARN" if warnings else "PASS"
    payload["compatibility_warnings"] = warnings
    return payload, warnings

